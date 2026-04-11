/* =========================================================================================
 *  File: switch_haptics.c
 *  Project: RetroOnyx Virtual Boy Wireless Controller
 *  Summary:
 *      Nintendo Switch rumble decode and playback layer for dual DRV2625 devices.
 *      This revision keeps change-only logging, clamps lookup table access to avoid
 *      out-of-bounds reads at the +2.0 endpoint, and guards runtime queue use so
 *      haptics_rumble_translate() is safe if called before haptics_init().
 * ========================================================================================= */
#include "drv2605_esp.h"
#include "switch_haptics.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "HAPTIC";

const float MinFrequency = -2.0f;
const float MaxFrequency = 2.0f;
const float DefaultFrequency = 0.0f;
const float MinAmplitude = -8.0f;
const float MaxAmplitude = 0.0f;
const float DefaultAmplitude = -8.0f;
const float StartingAmplitude = -7.9375f;

const float CenterFreqHigh = 320.0f;
const float CenterFreqLow = 160.0f;

#define HAPTICS_QUEUE_DEPTH              1
#define HAPTICS_IDLE_TIMEOUT_MS          100
#define HAPTICS_SUBFRAME_DELAY_MS        8
#define HAPTICS_MIN_AMPLITUDE_THRESHOLD  0.001f
#define HAPTICS_OUTPUT_GAIN              6.0f
#define HAPTICS_DEFAULT_OD_CLAMP         0x70

typedef enum
{
    HAPTICS_PATH_NONE = 0,
    HAPTICS_PATH_LEFT,
    HAPTICS_PATH_RIGHT,
    HAPTICS_PATH_BROADCAST
} haptics_output_path_t;

typedef struct
{
    hoja_rumble_msg_s left;
    hoja_rumble_msg_s right;
} haptics_packet_frame_s;

static void haptics_playback_task(void *param);

static QueueHandle_t s_haptics_queue = NULL;
static TaskHandle_t s_haptics_task_handle = NULL;
static bool _haptics_init = false;
static bool s_play_stop_logged = false;

static hoja_rumble_msg_s s_decode_left = {0};
static hoja_rumble_msg_s s_decode_right = {0};

static drv2625_test_mode_t s_last_mode_left = DRV2625_TEST_MODE_OL_LRA_160_SINE;
static drv2625_test_mode_t s_last_mode_right = DRV2625_TEST_MODE_OL_LRA_160_SINE;
static drv2625_test_mode_t s_last_mode_broadcast = DRV2625_TEST_MODE_OL_LRA_160_SINE;

static bool s_mode_valid_left = false;
static bool s_mode_valid_right = false;
static bool s_mode_valid_broadcast = false;

static haptics_output_path_t s_current_path = HAPTICS_PATH_NONE;

static inline float clampf(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}

static bool haptics_sample_active(const hoja_haptic_frame_s *s)
{
    if (s == NULL) {
        return false;
    }

    return ((s->low_amplitude + s->high_amplitude) >= HAPTICS_MIN_AMPLITUDE_THRESHOLD);
}

static const hoja_haptic_frame_s *haptics_get_sample_at(const hoja_rumble_msg_s *msg, uint8_t idx)
{
    if ((msg == NULL) || (idx >= msg->sample_count))
    {
        return NULL;
    }

    return &msg->samples[idx];
}

static bool haptics_msg_has_active_samples(const hoja_rumble_msg_s *msg)
{
    uint8_t i;

    if (msg == NULL)
    {
        return false;
    }

    for (i = 0; i < msg->sample_count; i++)
    {
        if (haptics_sample_active(&msg->samples[i]))
        {
            return true;
        }
    }

    return false;
}

const Switch5BitCommand_s CommandTable[] = {
    {.am_action = Switch5BitAction_Default,    .fm_action = Switch5BitAction_Default,    .am_offset = 0.0f,      .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = 0.0f,      .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -0.5f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -1.0f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -1.5f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -2.0f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -2.5f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -3.0f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -3.5f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -4.0f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -4.5f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore,     .am_offset = -5.0f,     .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f,      .fm_offset = -0.375f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f,      .fm_offset = -0.1875f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f,      .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f,      .fm_offset = 0.1875f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f,      .fm_offset = 0.375f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = 0.125f,    .fm_offset = 0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Ignore,     .am_offset = 0.125f,    .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = 0.125f,    .fm_offset = -0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = 0.03125f,  .fm_offset = 0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Ignore,     .am_offset = 0.03125f,  .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = 0.03125f,  .fm_offset = -0.03125f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Sum,        .am_offset = 0.0f,      .fm_offset = 0.03125f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Ignore,     .am_offset = 0.0f,      .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Ignore,     .fm_action = Switch5BitAction_Sum,        .am_offset = 0.0f,      .fm_offset = -0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = -0.03125f, .fm_offset = 0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Ignore,     .am_offset = -0.03125f, .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = -0.03125f, .fm_offset = -0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = -0.125f,   .fm_offset = 0.03125f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Ignore,     .am_offset = -0.125f,   .fm_offset = 0.0f},
    {.am_action = Switch5BitAction_Sum,        .fm_action = Switch5BitAction_Sum,        .am_offset = -0.125f,   .fm_offset = -0.03125f}
};

#define EXP_BASE2_RANGE_START (-8.0f)
#define EXP_BASE2_RANGE_END (2.0f)
#define EXP_BASE2_LOOKUP_RESOLUTION (1 / 32.0f)
#define EXP_BASE2_LOOKUP_LENGTH 320

static float ExpBase2Lookup[EXP_BASE2_LOOKUP_LENGTH];
static float RumbleAmpLookup[128];
static float RumbleFreqLookup[128];

void initialize_exp_base2_lookup(void)
{
    for (size_t i = 0; i < EXP_BASE2_LOOKUP_LENGTH; ++i)
    {
        float f = EXP_BASE2_RANGE_START + i * EXP_BASE2_LOOKUP_RESOLUTION;
        if (f >= StartingAmplitude)
        {
            ExpBase2Lookup[i] = exp2f(f);
        }
        else
        {
            ExpBase2Lookup[i] = 0.0f;
        }
    }
}

void initialize_rumble_amp_lookup(void)
{
    for (size_t i = 0; i < 128; ++i)
    {
        if (i == 0)
        {
            RumbleAmpLookup[i] = -8.0f;
        }
        else if (i < 16)
        {
            RumbleAmpLookup[i] = 0.25f * i - 7.75f;
        }
        else if (i < 32)
        {
            RumbleAmpLookup[i] = 0.0625f * i - 4.9375f;
        }
        else
        {
            RumbleAmpLookup[i] = 0.03125f * i - 3.96875f;
        }
    }
}

void initialize_rumble_freq_lookup(void)
{
    for (size_t i = 0; i < 128; ++i)
    {
        RumbleFreqLookup[i] = 0.03125f * i - 2.0f;
    }
}

uint32_t haptics_get_lookup_index(float input)
{
    float clamped;
    int32_t idx;

    clamped = clampf(input, EXP_BASE2_RANGE_START, EXP_BASE2_RANGE_END);
    idx = (int32_t)((clamped - EXP_BASE2_RANGE_START) / EXP_BASE2_LOOKUP_RESOLUTION);

    if (idx < 0)
    {
        idx = 0;
    }
    else if (idx >= (int32_t)EXP_BASE2_LOOKUP_LENGTH)
    {
        idx = (int32_t)EXP_BASE2_LOOKUP_LENGTH - 1;
    }

    return (uint32_t)idx;
}

float haptics_apply_command(Switch5BitAction_t action,
                            float offset,
                            float current,
                            float default_val,
                            float min,
                            float max)
{
    switch (action)
    {
        case Switch5BitAction_Ignore:
            return current;
        case Switch5BitAction_Substitute:
            return offset;
        case Switch5BitAction_Sum:
            return clampf(current + offset, min, max);
        default:
            return default_val;
    }
}

void haptics_initialize_lookup_tables(void)
{
    initialize_exp_base2_lookup();
    initialize_rumble_amp_lookup();
    initialize_rumble_freq_lookup();
}

bool haptics_disabled_check(uint8_t *data)
{
    if (data[0] & 0b00000001)
    {
        if (data[3] & 0b01000000)
        {
            return true;
        }
    }
    return false;
}

void haptics_linear_set_default(hoja_haptic_frame_linear_s *state)
{
    state->hi_amp_linear = DefaultAmplitude;
    state->lo_amp_linear = DefaultAmplitude;
    state->hi_freq_linear = DefaultFrequency;
    state->lo_freq_linear = DefaultFrequency;
}

void haptics_linear_to_normal(hoja_haptic_frame_linear_s *linear, hoja_haptic_frame_s *decoded)
{
    decoded->high_frequency = ExpBase2Lookup[haptics_get_lookup_index(linear->hi_freq_linear)] * CenterFreqHigh;
    decoded->low_frequency  = ExpBase2Lookup[haptics_get_lookup_index(linear->lo_freq_linear)] * CenterFreqLow;
    decoded->high_amplitude = ExpBase2Lookup[haptics_get_lookup_index(linear->hi_amp_linear)];
    decoded->low_amplitude  = ExpBase2Lookup[haptics_get_lookup_index(linear->lo_amp_linear)];
}

void _haptics_decode_type_1(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    uint8_t samples = encoded->frame_count;
    Switch5BitCommand_s hi_cmd = {0};
    Switch5BitCommand_s low_cmd = {0};

    decoded->sample_count = samples;

    if (samples > 0)
    {
        hi_cmd = CommandTable[encoded->type1.cmd_hi_0];
        decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                               decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                              decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        low_cmd = CommandTable[encoded->type1.cmd_lo_0];
        decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                               decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                              decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[0]));
    }

    if (samples > 1)
    {
        hi_cmd = CommandTable[encoded->type1.cmd_hi_1];
        decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                               decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                              decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        low_cmd = CommandTable[encoded->type1.cmd_lo_1];
        decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                               decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                              decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[1]));
    }

    if (samples > 2)
    {
        hi_cmd = CommandTable[encoded->type1.cmd_hi_2];
        decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                               decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                              decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        low_cmd = CommandTable[encoded->type1.cmd_lo_2];
        decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                               decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                              decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[2]));
    }
}

void _haptics_decode_type_2(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    uint8_t samples = encoded->frame_count;
    decoded->sample_count = samples;

    decoded->linear.hi_freq_linear = RumbleFreqLookup[encoded->type2.freq_hi];
    decoded->linear.lo_freq_linear = RumbleFreqLookup[encoded->type2.freq_lo];
    decoded->linear.hi_amp_linear  = RumbleAmpLookup[encoded->type2.amp_hi];
    decoded->linear.lo_amp_linear  = RumbleAmpLookup[encoded->type2.amp_lo];

    haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[0]));
}

void _haptics_decode_type_3(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    Switch5BitCommand_s hi_cmd = {0};
    Switch5BitCommand_s low_cmd = {0};
    uint8_t samples = encoded->frame_count;

    decoded->sample_count = samples;

    if (samples > 0)
    {
        if (encoded->type3.high_select)
        {
            decoded->linear.hi_freq_linear = RumbleFreqLookup[encoded->type3.freq_xx_0];
            decoded->linear.hi_amp_linear  = RumbleAmpLookup[encoded->type3.amp_xx_0];

            low_cmd = CommandTable[encoded->type3.cmd_xx_0];
            decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                                   decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
            decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                                  decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);
        }
        else
        {
            decoded->linear.lo_freq_linear = RumbleFreqLookup[encoded->type3.freq_xx_0];
            decoded->linear.lo_amp_linear  = RumbleAmpLookup[encoded->type3.amp_xx_0];

            hi_cmd = CommandTable[encoded->type3.cmd_xx_0];
            decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                                   decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
            decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                                  decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);
        }

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[0]));
    }

    if (samples > 1)
    {
        hi_cmd = CommandTable[encoded->type3.cmd_hi_1];
        decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                               decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                              decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        low_cmd = CommandTable[encoded->type3.cmd_lo_1];
        decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                               decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                              decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[1]));
    }
}

void _haptics_decode_type_4(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    Switch5BitCommand_s hi_cmd = {0};
    Switch5BitCommand_s low_cmd = {0};
    uint8_t samples = encoded->frame_count;

    decoded->sample_count = samples;

    if (samples > 0)
    {
        if (encoded->type4.high_select)
        {
            if (encoded->type4.freq_select)
            {
                decoded->linear.hi_freq_linear = RumbleFreqLookup[encoded->type4.xx_xx_0];
            }
            else
            {
                decoded->linear.hi_amp_linear = RumbleAmpLookup[encoded->type4.xx_xx_0];
            }
        }
        else
        {
            if (encoded->type4.freq_select)
            {
                decoded->linear.lo_freq_linear = RumbleFreqLookup[encoded->type4.xx_xx_0];
            }
            else
            {
                decoded->linear.lo_amp_linear = RumbleAmpLookup[encoded->type4.xx_xx_0];
            }
        }

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[0]));
    }

    if (samples > 1)
    {
        hi_cmd = CommandTable[encoded->type4.cmd_hi_1];
        decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                               decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                              decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        low_cmd = CommandTable[encoded->type4.cmd_lo_1];
        decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                               decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                              decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[1]));
    }

    if (samples > 2)
    {
        hi_cmd = CommandTable[encoded->type4.cmd_hi_2];
        decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                               decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                              decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        low_cmd = CommandTable[encoded->type4.cmd_lo_2];
        decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                               decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
        decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                              decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);

        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[2]));
    }
}

void _haptics_decode_samples(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    switch (encoded->frame_count)
    {
        case 0:
            decoded->sample_count = 0;
            break;

        case 1:
            if ((encoded->data & 0xFFFFF) == 0)
            {
                _haptics_decode_type_1(encoded, decoded);
            }
            else if ((encoded->data & 0x3) == 0)
            {
                _haptics_decode_type_2(encoded, decoded);
            }
            else if ((encoded->data & 0x2) == 2)
            {
                _haptics_decode_type_4(encoded, decoded);
            }
            break;

        case 2:
            if ((encoded->data & 0x3FF) == 0)
            {
                _haptics_decode_type_1(encoded, decoded);
            }
            else
            {
                _haptics_decode_type_3(encoded, decoded);
            }
            break;

        case 3:
            _haptics_decode_type_1(encoded, decoded);
            break;
    }
}

static drv2625_test_mode_t haptics_mode_from_frequency(float frequency_hz)
{
    float diff160 = fabsf(frequency_hz - 160.0f);
    float diff320 = fabsf(frequency_hz - 320.0f);

    if (diff160 <= diff320)
    {
        return DRV2625_TEST_MODE_OL_LRA_160_SINE;
    }

    return DRV2625_TEST_MODE_OL_LRA_320_SINE;
}

static int8_t haptics_rtp_from_amplitude(float amplitude)
{
    float scaled = amplitude * HAPTICS_OUTPUT_GAIN;

    if (scaled < 0.0f)
    {
        scaled = 0.0f;
    }
    if (scaled > 1.0f)
    {
        scaled = 1.0f;
    }

    return (int8_t)(scaled * 127.0f);
}

static void haptics_log_play_left(const hoja_haptic_frame_s *sample, drv2625_test_mode_t mode, int8_t rtp)
{
    float total_amp = sample->low_amplitude + sample->high_amplitude;
    float selected_freq = (sample->high_amplitude > sample->low_amplitude) ?
                          sample->high_frequency : sample->low_frequency;

    s_play_stop_logged = false;

    ESP_LOGI(TAG,
             "PLAY L: hi_f=%.1f lo_f=%.1f hi_a=%.3f lo_a=%.3f total=%.3f sel_f=%.1f mode=%d rtp=%d",
             sample->high_frequency,
             sample->low_frequency,
             sample->high_amplitude,
             sample->low_amplitude,
             total_amp,
             selected_freq,
             (int)mode,
             (int)rtp);
}

static void haptics_log_play_right(const hoja_haptic_frame_s *sample, drv2625_test_mode_t mode, int8_t rtp)
{
    float total_amp = sample->low_amplitude + sample->high_amplitude;
    float selected_freq = (sample->high_amplitude > sample->low_amplitude) ?
                          sample->high_frequency : sample->low_frequency;

    s_play_stop_logged = false;

    ESP_LOGI(TAG,
             "PLAY R: hi_f=%.1f lo_f=%.1f hi_a=%.3f lo_a=%.3f total=%.3f sel_f=%.1f mode=%d rtp=%d",
             sample->high_frequency,
             sample->low_frequency,
             sample->high_amplitude,
             sample->low_amplitude,
             total_amp,
             selected_freq,
             (int)mode,
             (int)rtp);
}

static void haptics_log_play_broadcast(const hoja_haptic_frame_s *left_sample,
                                       const hoja_haptic_frame_s *right_sample,
                                       drv2625_test_mode_t mode,
                                       int8_t rtp,
                                       float total_amp,
                                       float selected_freq)
{
    s_play_stop_logged = false;

    ESP_LOGI(TAG,
             "PLAY BCAST: "
             "L[hi_f=%.1f lo_f=%.1f hi_a=%.3f lo_a=%.3f] "
             "R[hi_f=%.1f lo_f=%.1f hi_a=%.3f lo_a=%.3f] "
             "total=%.3f sel_f=%.1f mode=%d rtp=%d",
             left_sample ? left_sample->high_frequency : 0.0f,
             left_sample ? left_sample->low_frequency : 0.0f,
             left_sample ? left_sample->high_amplitude : 0.0f,
             left_sample ? left_sample->low_amplitude : 0.0f,
             right_sample ? right_sample->high_frequency : 0.0f,
             right_sample ? right_sample->low_frequency : 0.0f,
             right_sample ? right_sample->high_amplitude : 0.0f,
             right_sample ? right_sample->low_amplitude : 0.0f,
             total_amp,
             selected_freq,
             (int)mode,
             (int)rtp);
}

static void haptics_log_play_stop(void)
{
    if (!s_play_stop_logged)
    {
        ESP_LOGI(TAG, "PLAY STOP");
        s_play_stop_logged = true;
    }
}

static void haptics_stop_all_paths(void)
{
    (void)drv2625_stop_broadcast();
    (void)drv2625_stop_all();
    s_current_path = HAPTICS_PATH_NONE;
}

static esp_err_t haptics_enter_left_path(void)
{
    if (s_current_path == HAPTICS_PATH_LEFT)
    {
        return ESP_OK;
    }

    (void)drv2625_stop_broadcast();
    s_current_path = HAPTICS_PATH_LEFT;
    return ESP_OK;
}

static esp_err_t haptics_enter_right_path(void)
{
    if (s_current_path == HAPTICS_PATH_RIGHT)
    {
        return ESP_OK;
    }

    (void)drv2625_stop_broadcast();
    s_current_path = HAPTICS_PATH_RIGHT;
    return ESP_OK;
}

static esp_err_t haptics_enter_broadcast_path(drv2625_test_mode_t mode)
{
    esp_err_t err;

    if (s_current_path != HAPTICS_PATH_BROADCAST)
    {
        err = drv2625_init_broadcast_mode(mode, HAPTICS_DEFAULT_OD_CLAMP);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Broadcast init failed: %s", esp_err_to_name(err));
            return err;
        }

        s_last_mode_broadcast = mode;
        s_mode_valid_broadcast = true;
        s_current_path = HAPTICS_PATH_BROADCAST;
        return ESP_OK;
    }

    if ((!s_mode_valid_broadcast) || (mode != s_last_mode_broadcast))
    {
        err = drv2625_set_broadcast_mode(mode, HAPTICS_DEFAULT_OD_CLAMP);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Broadcast mode set failed: %s", esp_err_to_name(err));
            return err;
        }

        s_last_mode_broadcast = mode;
        s_mode_valid_broadcast = true;
    }

    return ESP_OK;
}

static void haptics_play_left_sample(const hoja_haptic_frame_s *sample)
{
    float total_amp;
    float selected_freq;
    drv2625_test_mode_t mode;
    int8_t rtp;

    if ((sample == NULL) || !haptics_sample_active(sample))
    {
        haptics_log_play_stop();
        (void)drv2625_set_rtp_channel(DRV2625_CH_LEFT, 0);
        return;
    }

    total_amp = sample->low_amplitude + sample->high_amplitude;
    selected_freq = (sample->high_amplitude > sample->low_amplitude) ?
                    sample->high_frequency : sample->low_frequency;
    mode = haptics_mode_from_frequency(selected_freq);
    rtp = haptics_rtp_from_amplitude(total_amp);

    haptics_log_play_left(sample, mode, rtp);

    if ((!s_mode_valid_left) || (mode != s_last_mode_left))
    {
        if (drv2625_set_test_mode_channel(DRV2625_CH_LEFT, mode, HAPTICS_DEFAULT_OD_CLAMP) == ESP_OK)
        {
            s_last_mode_left = mode;
            s_mode_valid_left = true;
        }
    }

    (void)drv2625_set_rtp_channel(DRV2625_CH_LEFT, rtp);
}

static void haptics_play_right_sample(const hoja_haptic_frame_s *sample)
{
    float total_amp;
    float selected_freq;
    drv2625_test_mode_t mode;
    int8_t rtp;

    if ((sample == NULL) || !haptics_sample_active(sample))
    {
        haptics_log_play_stop();
        (void)drv2625_set_rtp_channel(DRV2625_CH_RIGHT, 0);
        return;
    }

    total_amp = sample->low_amplitude + sample->high_amplitude;
    selected_freq = (sample->high_amplitude > sample->low_amplitude) ?
                    sample->high_frequency : sample->low_frequency;
    mode = haptics_mode_from_frequency(selected_freq);
    rtp = haptics_rtp_from_amplitude(total_amp);

    haptics_log_play_right(sample, mode, rtp);

    if ((!s_mode_valid_right) || (mode != s_last_mode_right))
    {
        if (drv2625_set_test_mode_channel(DRV2625_CH_RIGHT, mode, HAPTICS_DEFAULT_OD_CLAMP) == ESP_OK)
        {
            s_last_mode_right = mode;
            s_mode_valid_right = true;
        }
    }

    (void)drv2625_set_rtp_channel(DRV2625_CH_RIGHT, rtp);
}

static void haptics_play_broadcast_sample(const hoja_haptic_frame_s *left_sample,
                                          const hoja_haptic_frame_s *right_sample)
{
    float left_total;
    float right_total;
    float total_amp;
    float left_freq;
    float right_freq;
    float selected_freq;
    drv2625_test_mode_t mode;
    int8_t rtp;

    if ((!haptics_sample_active(left_sample)) && (!haptics_sample_active(right_sample)))
    {
        haptics_log_play_stop();
        (void)drv2625_stop_broadcast();
        return;
    }

    left_total = haptics_sample_active(left_sample) ?
                 (left_sample->low_amplitude + left_sample->high_amplitude) : 0.0f;
    right_total = haptics_sample_active(right_sample) ?
                  (right_sample->low_amplitude + right_sample->high_amplitude) : 0.0f;

    left_freq = haptics_sample_active(left_sample) ?
                ((left_sample->high_amplitude > left_sample->low_amplitude) ?
                 left_sample->high_frequency : left_sample->low_frequency) : 160.0f;
    right_freq = haptics_sample_active(right_sample) ?
                 ((right_sample->high_amplitude > right_sample->low_amplitude) ?
                  right_sample->high_frequency : right_sample->low_frequency) : 160.0f;

    total_amp = 0.5f * (left_total + right_total);
    selected_freq = 0.5f * (left_freq + right_freq);

    mode = haptics_mode_from_frequency(selected_freq);
    rtp = haptics_rtp_from_amplitude(total_amp);

    haptics_log_play_broadcast(left_sample, right_sample, mode, rtp, total_amp, selected_freq);

    if (haptics_enter_broadcast_path(mode) != ESP_OK)
    {
        return;
    }

    (void)drv2625_set_broadcast_rtp(rtp);
}

static void haptics_apply_frame(const haptics_packet_frame_s *frame)
{
    uint8_t i;
    uint8_t max_samples;

    max_samples = frame->left.sample_count;
    if (frame->right.sample_count > max_samples)
    {
        max_samples = frame->right.sample_count;
    }

    if (max_samples == 0)
    {
        haptics_log_play_stop();
        haptics_stop_all_paths();
        return;
    }

    for (i = 0; i < max_samples; i++)
    {
        const hoja_haptic_frame_s *left_sample;
        const hoja_haptic_frame_s *right_sample;
        bool left_active;
        bool right_active;

        left_sample = haptics_get_sample_at(&frame->left, i);
        right_sample = haptics_get_sample_at(&frame->right, i);

        left_active = haptics_sample_active(left_sample);
        right_active = haptics_sample_active(right_sample);

        if (left_active && right_active)
        {
            haptics_play_broadcast_sample(left_sample, right_sample);
        }
        else if (left_active)
        {
            if (haptics_enter_left_path() == ESP_OK)
            {
                haptics_play_left_sample(left_sample);
            }
        }
        else if (right_active)
        {
            if (haptics_enter_right_path() == ESP_OK)
            {
                haptics_play_right_sample(right_sample);
            }
        }
        else
        {
            haptics_log_play_stop();
            haptics_stop_all_paths();
        }

        if ((i + 1U) < max_samples)
        {
            vTaskDelay(pdMS_TO_TICKS(HAPTICS_SUBFRAME_DELAY_MS));
        }
    }
}

static void haptics_playback_task(void *param)
{
    haptics_packet_frame_s frame;
    bool active = false;

    (void)param;

    while (1)
    {
        if (xQueueReceive(s_haptics_queue, &frame, pdMS_TO_TICKS(HAPTICS_IDLE_TIMEOUT_MS)) == pdTRUE)
        {
            haptics_apply_frame(&frame);
            active = true;
        }
        else if (active)
        {
            haptics_log_play_stop();
            haptics_stop_all_paths();
            active = false;
        }
    }
}

void haptics_rumble_translate(const uint8_t *data)
{
    haptics_packet_frame_s frame;
    uint8_t i;
    bool rumble_active;
    static bool s_prev_valid = false;
    static uint8_t s_prev_raw[8] = {0};

    if ((data == NULL) || (!_haptics_init) || (s_haptics_queue == NULL))
    {
        return;
    }

    _haptics_decode_samples((const SwitchHapticPacket_s *)data, &s_decode_left);
    _haptics_decode_samples((const SwitchHapticPacket_s *)&data[4], &s_decode_right);

    rumble_active = haptics_msg_has_active_samples(&s_decode_left) ||
                    haptics_msg_has_active_samples(&s_decode_right);

    if (rumble_active && ((!s_prev_valid) || (memcmp(s_prev_raw, data, 8) != 0)))
    {
        ESP_LOGI(TAG,
                 "RUMBLE raw: %02X %02X %02X %02X  %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3],
                 data[4], data[5], data[6], data[7]);

        ESP_LOGI(TAG, "RUMBLE L sample_count=%u", s_decode_left.sample_count);
        for (i = 0; i < s_decode_left.sample_count; i++)
        {
            ESP_LOGI(TAG,
                     "RUMBLE L[%u]: hi_f=%.1f lo_f=%.1f hi_a=%.3f lo_a=%.3f",
                     i,
                     s_decode_left.samples[i].high_frequency,
                     s_decode_left.samples[i].low_frequency,
                     s_decode_left.samples[i].high_amplitude,
                     s_decode_left.samples[i].low_amplitude);
        }

        ESP_LOGI(TAG, "RUMBLE R sample_count=%u", s_decode_right.sample_count);
        for (i = 0; i < s_decode_right.sample_count; i++)
        {
            ESP_LOGI(TAG,
                     "RUMBLE R[%u]: hi_f=%.1f lo_f=%.1f hi_a=%.3f lo_a=%.3f",
                     i,
                     s_decode_right.samples[i].high_frequency,
                     s_decode_right.samples[i].low_frequency,
                     s_decode_right.samples[i].high_amplitude,
                     s_decode_right.samples[i].low_amplitude);
        }
    }

    memcpy(s_prev_raw, data, 8);
    s_prev_valid = true;

    frame.left = s_decode_left;
    frame.right = s_decode_right;

    xQueueOverwrite(s_haptics_queue, &frame);
}

void haptics_init(void)
{
    esp_err_t err;

    if (_haptics_init)
    {
        return;
    }

    ESP_LOGI(TAG, "Initializing haptic subsystem...");

    err = drv2625_init();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "DRV2625 init returned %s", esp_err_to_name(err));
    }

    haptics_initialize_lookup_tables();

    memset(&s_decode_left, 0, sizeof(s_decode_left));
    memset(&s_decode_right, 0, sizeof(s_decode_right));
    haptics_linear_set_default(&s_decode_left.linear);
    haptics_linear_set_default(&s_decode_right.linear);

    s_last_mode_left = DRV2625_TEST_MODE_OL_LRA_160_SINE;
    s_last_mode_right = DRV2625_TEST_MODE_OL_LRA_160_SINE;
    s_last_mode_broadcast = DRV2625_TEST_MODE_OL_LRA_160_SINE;

    s_mode_valid_left = false;
    s_mode_valid_right = false;
    s_mode_valid_broadcast = false;

    s_current_path = HAPTICS_PATH_NONE;

    s_haptics_queue = xQueueCreate(HAPTICS_QUEUE_DEPTH, sizeof(haptics_packet_frame_s));
    if (s_haptics_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create haptics queue");
        return;
    }

    if (xTaskCreatePinnedToCore(
            haptics_playback_task,
            "haptics_playback",
            4096,
            NULL,
            2,
            &s_haptics_task_handle,
            tskNO_AFFINITY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create haptics playback task");
        return;
    }

    _haptics_init = true;
}