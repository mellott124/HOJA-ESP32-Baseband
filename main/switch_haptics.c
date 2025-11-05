/* =========================================================================================
 *  HAPTICS SUBSYSTEM — RETROONYX VIRTUAL BOY WIRELESS CONTROLLER
 *  -----------------------------------------------------------------------------------------
 *  CURRENT IMPLEMENTATION (DRV2605L)
 *
 *  Overview:
 *      This module provides real Nintendo Switch rumble support using a DRV2605L haptic driver.
 *      Incoming vibration packets from the Switch are decoded and converted into RTP amplitude
 *      commands over I²C. The current build supports single-channel output (Right haptic module)
 *      and is ready for expansion to dual-channel DRV2625 hardware.
 *
 *  Data Flow:
 *      Switch (BT HID)
 *          ↓
 *      ns_report_handler()
 *          ↓
 *      app_set_switch_haptic()
 *          ↓
 *      haptics_rumble_translate()
 *          ↓
 *      drv2605_set_rtp()  →  I²C  →  DRV2605L
 *
 *  Hardware Interface:
 *      SDA     → IO21   (I²C Data)
 *      SCL     → IO22   (I²C Clock)
 *      GPIO_R  → IO19   (Right haptic enable/trigger)
 *      GPIO_L  → TBD    (Left haptic enable/trigger, future DRV2625 support)
 *
 *  Current Behavior:
 *      - Fully decodes and responds to Switch rumble packets.
 *      - Haptics active in all controller modes (Pro, SNES, N64, etc.).
 *      - Verified operation via Switch “Find Controllers” and trigger test events.
 *      - No watchdog stalls or I²C contention.
 *
 *  Deferred / Future Work:
 *      [ ] Add dual-channel support using DRV2625 (Left & Right haptics).
 *      [ ] Implement GPIO_L enable logic and proper power gating.
 *      [ ] Calibrate amplitude curves for LRA tuning on production hardware.
 *      [ ] Optional: add RTP fade-out for smoother stop transitions.
 *      [ ] Unify initialization via haptics_init() call.
 *
 *  Notes:
 *      - DRV2605L is operating in RTP mode for real-time amplitude control.
 *      - All Switch modes are left unfiltered to ensure consistent rumble behavior.
 *      - Safe to run without haptic hardware connected; driver initialization will skip if not found.
 * ========================================================================================= */


#include "drv2605_esp.h"
#include "switch_haptics.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

static void haptic_stop_task(void *param);

static inline float clampf(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}

const Switch5BitCommand_s CommandTable[] = {
            {.am_action = Switch5BitAction_Default, .fm_action = Switch5BitAction_Default, .am_offset = 0.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = 0.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -0.5f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -1.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -1.5f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -2.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -2.5f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -3.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -3.5f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -4.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -4.5f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Substitute, .fm_action = Switch5BitAction_Ignore, .am_offset = -5.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f, .fm_offset = -0.375f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f, .fm_offset = -0.1875f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f, .fm_offset = 0.1875f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Substitute, .am_offset = 0.0f, .fm_offset = 0.375f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = 0.125f, .fm_offset = 0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Ignore, .am_offset = 0.125f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = 0.125f, .fm_offset = -0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = 0.03125f, .fm_offset = 0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Ignore, .am_offset = 0.03125f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = 0.03125f, .fm_offset = -0.03125f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Sum, .am_offset = 0.0f, .fm_offset = 0.03125f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Ignore, .am_offset = 0.0f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Ignore, .fm_action = Switch5BitAction_Sum, .am_offset = 0.0f, .fm_offset = -0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = -0.03125f, .fm_offset = 0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Ignore, .am_offset = -0.03125f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = -0.03125f, .fm_offset = -0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = -0.125f, .fm_offset = 0.03125f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Ignore, .am_offset = -0.125f, .fm_offset = 0.0f},
            {.am_action = Switch5BitAction_Sum, .fm_action = Switch5BitAction_Sum, .am_offset = -0.125f, .fm_offset = -0.03125f}
        };

#define EXP_BASE2_RANGE_START (-8.0f)
#define EXP_BASE2_RANGE_END (2.0f)
#define EXP_BASE2_LOOKUP_RESOLUTION (1 / 32.0f)
#define EXP_BASE2_LOOKUP_LENGTH 320 //((size_t)(((EXP_BASE2_RANGE_END - EXP_BASE2_RANGE_START) + EXP_BASE2_LOOKUP_RESOLUTION) / EXP_BASE2_LOOKUP_RESOLUTION))

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
    return (uint32_t)((input - EXP_BASE2_RANGE_START) / EXP_BASE2_LOOKUP_RESOLUTION);
}

float haptics_apply_command(Switch5BitAction_t action, float offset, float current, float default_val, float min, float max)
{
    switch (action)
    {
    case Switch5BitAction_Ignore: // Cmd 0
        return current;
    case Switch5BitAction_Substitute: // Cmd 2
        return offset;
    case Switch5BitAction_Sum: // Cmd 3
        return clampf(current + offset, min, max);
    default: // Cmd 1
        return default_val;
    }
}

static bool _haptics_init = false;
// Call this function to initialize all lookup tables
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

// Functionally does what GetOutputValue does in the original documentation
void haptics_linear_to_normal(hoja_haptic_frame_linear_s *linear, hoja_haptic_frame_s *decoded)
{
    decoded->high_frequency = ExpBase2Lookup[haptics_get_lookup_index(linear->hi_freq_linear)] * CenterFreqHigh;
    decoded->low_frequency = ExpBase2Lookup[haptics_get_lookup_index(linear->lo_freq_linear)] * CenterFreqLow;
    decoded->high_amplitude = ExpBase2Lookup[haptics_get_lookup_index(linear->hi_amp_linear)];
    decoded->low_amplitude = ExpBase2Lookup[haptics_get_lookup_index(linear->lo_amp_linear)];
}

void _haptics_decode_type_1(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    uint8_t samples = encoded->frame_count;
    decoded->sample_count = samples;

    // decoded->count = 3;
    Switch5BitCommand_s hi_cmd = {0};
    Switch5BitCommand_s low_cmd = {0};

    // Decode sample 0
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

    // Decode sample 1
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

    // Decode sample 2
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
    decoded->linear.hi_amp_linear = RumbleAmpLookup[encoded->type2.amp_hi];
    decoded->linear.lo_amp_linear = RumbleAmpLookup[encoded->type2.amp_lo];

    haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[0]));
}

void _haptics_decode_type_3(const SwitchHapticPacket_s *encoded, hoja_rumble_msg_s *decoded)
{
    // decoded->count = 2;
    Switch5BitCommand_s hi_cmd = {0};
    Switch5BitCommand_s low_cmd = {0};

    uint8_t samples = encoded->frame_count;
    decoded->sample_count = samples;

    // Decode sample 0
    if (samples > 0)
    {
        if (encoded->type3.high_select)
        {
            decoded->linear.hi_freq_linear = RumbleFreqLookup[encoded->type3.freq_xx_0];
            decoded->linear.hi_amp_linear = RumbleAmpLookup[encoded->type3.amp_xx_0];

            low_cmd = CommandTable[encoded->type3.cmd_xx_0];
            decoded->linear.lo_freq_linear = haptics_apply_command(low_cmd.fm_action, low_cmd.fm_offset,
                                                                   decoded->linear.lo_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
            decoded->linear.lo_amp_linear = haptics_apply_command(low_cmd.am_action, low_cmd.am_offset,
                                                                  decoded->linear.lo_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);
        }
        else
        {
            decoded->linear.lo_freq_linear = RumbleFreqLookup[encoded->type3.freq_xx_0];
            decoded->linear.lo_amp_linear = RumbleAmpLookup[encoded->type3.amp_xx_0];

            hi_cmd = CommandTable[encoded->type3.cmd_xx_0];
            decoded->linear.hi_freq_linear = haptics_apply_command(hi_cmd.fm_action, hi_cmd.fm_offset,
                                                                   decoded->linear.hi_freq_linear, DefaultFrequency, MinFrequency, MaxFrequency);
            decoded->linear.hi_amp_linear = haptics_apply_command(hi_cmd.am_action, hi_cmd.am_offset,
                                                                  decoded->linear.hi_amp_linear, DefaultAmplitude, MinAmplitude, MaxAmplitude);
        }
        haptics_linear_to_normal(&(decoded->linear), &(decoded->samples[0]));
    }

    // Decode sample 1
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

    // Decode sample 0
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

    // Decode sample 1
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

    // Decode sample 2
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

// This will detect and call the appropriate decoding schema
void _haptics_decode_samples(const SwitchHapticPacket_s *encoded,
                            hoja_rumble_msg_s *decoded)
{
    switch (encoded->frame_count)
    {
        case 0:
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
    };
}

/**
 * Translate Switch rumble packets to DRV2605 RTP output.
 * Called whenever the Switch sends 8-byte rumble data.
 */
void haptics_rumble_translate(const uint8_t *data)
{
    if (!data) return;

    static hoja_rumble_msg_s left = {0};
    static hoja_rumble_msg_s right = {0};
    static uint8_t last_amp = 0;

    // Decode the left and right 4-byte haptic packets
    _haptics_decode_samples((const SwitchHapticPacket_s *)data, &left);
    _haptics_decode_samples((const SwitchHapticPacket_s *)&data[4], &right);

    float amp_f_left  = left.samples[0].high_amplitude;
    float amp_f_right = right.samples[0].high_amplitude;

    // Scale up Switch amplitudes (~0–0.25 typical)
    amp_f_left  *= 6.0f;
    amp_f_right *= 6.0f;
    if (amp_f_left  > 1.0f) amp_f_left  = 1.0f;
    if (amp_f_right > 1.0f) amp_f_right = 1.0f;

    uint8_t amp_left  = (uint8_t)(amp_f_left  * 127.0f);
    uint8_t amp_right = (uint8_t)(amp_f_right * 127.0f);
    uint8_t amp       = (amp_left + amp_right) / 2;

    if(amp>0)
		ESP_LOGI(TAG, "Decoded amp L=%.3f R=%.3f (RTP=%d)", amp_f_left, amp_f_right, amp);

    // If the rumble signal is near zero, stop the motor
    if (amp < 5) {
        if (last_amp != 0) {
            drv2605_set_rtp(0);
            //ESP_LOGI(TAG, "Stop rumble (low amp)");
            last_amp = 0;
        }
        return;
    }

    // Re-enter RTP mode (safe guard)
    drv2605_write(DRV2605_REG_MODE, DRV2605_MODE_RTP);
    drv2605_set_rtp(amp);
    last_amp = amp;

    // Start a short background task that turns off rumble after 200 ms
    xTaskCreatePinnedToCore(
        haptic_stop_task,
        "haptic_stop_timer",
        2048,
        NULL,
        1,
        NULL,
        tskNO_AFFINITY);
}

/**
 * Task that waits briefly, then stops the rumble motor.
 */
static void haptic_stop_task(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(200));   // 0.2 s rumble duration
    drv2605_set_rtp(0);
    //ESP_LOGI("HAPTIC", "Auto-stop rumble (timeout)");
    vTaskDelete(NULL);
}

void haptics_init(void)
{
    if (_haptics_init)
        return;

    ESP_LOGI("HAPTIC", "Initializing haptic subsystem...");
    drv2605_init();
    haptics_initialize_lookup_tables();
    _haptics_init = true;
}



