// --------------------------------------------------------------------------
// main.c — Virtual Boy → Nintendo Switch (ESP32) Firmware Entry
// --------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "hoja.h"
#include "core_bt_switch.h"
#include "core_bt_dinput.h"
#include "esp_gap_bt_api.h"
#include "hoja_includes.h"
#include "hoja_types.h"
#include "LED.h"
#include "switch_haptics.h"
#include "drv2605_esp.h"
#include "bt_serial_logger.h"

#define TAG "MAIN"
#define HIGH 1
#define LOW  0

// --------------------------------------------------------------------------
// GPIO MAPPING (Virtual Boy layout)
// --------------------------------------------------------------------------
#define GPIO_BTN_A          GPIO_NUM_35
#define GPIO_BTN_B          GPIO_NUM_32
#define GPIO_BTN_DPAD_U     GPIO_NUM_14
#define GPIO_BTN_DPAD_L     GPIO_NUM_7
#define GPIO_BTN_DPAD_D     GPIO_NUM_8
#define GPIO_BTN_DPAD_R     GPIO_NUM_13
#define GPIO_BTN_C_U        GPIO_NUM_34
#define GPIO_BTN_C_L        GPIO_NUM_33
#define GPIO_BTN_C_D        GPIO_NUM_38
#define GPIO_BTN_C_R        GPIO_NUM_39
#define GPIO_BTN_L          GPIO_NUM_5
#define GPIO_BTN_R          GPIO_NUM_37
#define GPIO_BTN_START      GPIO_NUM_20
#define GPIO_BTN_SELECT     GPIO_NUM_4
#define GPIO_BTN_SYNC       GPIO_NUM_2   // active-low SYNC button

// --------------------------------------------------------------------------
// APP GLOBAL STATE
// --------------------------------------------------------------------------
static bool bt_connected=false, bt_pairing=false, bt_error=false;
hoja_settings_s global_loaded_settings={.magic=HOJA_MAGIC_NUM};
hoja_live_s global_live_data={0};

static volatile uint64_t _app_report_timer_us = 8000;
static volatile uint64_t _app_report_timer_us_default = 8000;
static volatile bool _sniff = true;

uint8_t _i2c_buffer_in[32];

// --------------------------------------------------------------------------
// MODE MANAGEMENT
// --------------------------------------------------------------------------
static input_mode_t current_mode = INPUT_MODE_SWPRO;
input_mode_t get_current_mode(void) { return current_mode; }

// --------------------------------------------------------------------------
// Boot-time mode selection using Right D-pad (C cluster)
// --------------------------------------------------------------------------
static void select_boot_mode_from_right_dpad(void)
{
    bool c_left_pressed  = (gpio_get_level(GPIO_BTN_C_L) == 0);
    bool c_up_pressed    = (gpio_get_level(GPIO_BTN_C_U) == 0);
    bool c_down_pressed  = (gpio_get_level(GPIO_BTN_C_D) == 0);
    bool c_right_pressed = (gpio_get_level(GPIO_BTN_C_R) == 0);

    if (c_left_pressed) {
        current_mode = INPUT_MODE_DINPUT_BR;      // BlueRetro / general PC DInput mode
    } else if (c_up_pressed) {
        current_mode = INPUT_MODE_DINPUT_VBGO;    // VBGo-optimized DInput mode
    } else if (c_right_pressed) {
        current_mode = INPUT_MODE_SNES;           // SNES mode
    } else if (c_down_pressed) {
        current_mode = INPUT_MODE_N64;            // N64 mode
    } else {
        current_mode = INPUT_MODE_SWPRO;          // Default: Switch Pro mode
    }

    if ((get_current_mode() == INPUT_MODE_DINPUT_BR) ||
        (get_current_mode() == INPUT_MODE_DINPUT_VBGO)) {
        ESP_LOGI(TAG, "Stored paired host (DInput): %02X:%02X:%02X:%02X:%02X:%02X",
                 global_loaded_settings.paired_host_dinput_mac[0],
                 global_loaded_settings.paired_host_dinput_mac[1],
                 global_loaded_settings.paired_host_dinput_mac[2],
                 global_loaded_settings.paired_host_dinput_mac[3],
                 global_loaded_settings.paired_host_dinput_mac[4],
                 global_loaded_settings.paired_host_dinput_mac[5]);
    } else {
        ESP_LOGI(TAG, "Stored paired host (Switch): %02X:%02X:%02X:%02X:%02X:%02X",
                 global_loaded_settings.paired_host_switch_mac[0],
                 global_loaded_settings.paired_host_switch_mac[1],
                 global_loaded_settings.paired_host_switch_mac[2],
                 global_loaded_settings.paired_host_switch_mac[3],
                 global_loaded_settings.paired_host_switch_mac[4],
                 global_loaded_settings.paired_host_switch_mac[5]);
    }

    led_set_state(LED_PAIRING);
}


// --------------------------------------------------------------------------
// NVS: SAVE / LOAD APPLICATION SETTINGS
// --------------------------------------------------------------------------
void app_settings_save(void)
{
    nvs_handle_t h;
    if(nvs_open("hoja",NVS_READWRITE,&h)!=ESP_OK){
        ESP_LOGE(TAG,"NVS open fail (save)");
        return;
    }
    nvs_set_blob(h,"settings",&global_loaded_settings,sizeof(global_loaded_settings));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG,"Settings saved");
}

void app_settings_load(void)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open("hoja", NVS_READWRITE, &h);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "NVS open fail (load)");
        memset(&global_loaded_settings, 0, sizeof(global_loaded_settings));
        global_loaded_settings.magic = HOJA_MAGIC_NUM;
        return;
    }

    size_t len = sizeof(global_loaded_settings);
    e = nvs_get_blob(h, "settings", &global_loaded_settings, &len);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "No settings in NVS");
        memset(&global_loaded_settings, 0, sizeof(global_loaded_settings));
        global_loaded_settings.magic = HOJA_MAGIC_NUM;
    }
    nvs_close(h);

    if (!global_loaded_settings.has_paired_switch &&
        memcmp(global_loaded_settings.paired_host_switch_mac, "\0\0\0\0\0\0", 6) != 0) {
        global_loaded_settings.has_paired_switch = true;
    }
    if (!global_loaded_settings.has_paired_dinput &&
        memcmp(global_loaded_settings.paired_host_dinput_mac, "\0\0\0\0\0\0", 6) != 0) {
        global_loaded_settings.has_paired_dinput = true;
    }

    if (memcmp(global_loaded_settings.device_mac_dinput, "\0\0\0\0\0\0", 6) == 0) {
        ESP_LOGI(TAG, "Generating independent DInput MAC from Switch MAC...");
        memcpy(global_loaded_settings.device_mac_dinput,
               global_loaded_settings.device_mac_switch, 6);
        global_loaded_settings.device_mac_dinput[5] += 3;
        app_settings_save();
    }

    if (global_loaded_settings.magic != HOJA_MAGIC_NUM) {
        ESP_LOGW(TAG, "Settings invalid — restoring defaults");
        memset(&global_loaded_settings, 0, sizeof(global_loaded_settings));
        global_loaded_settings.magic = HOJA_MAGIC_NUM;
        app_settings_save();
    }
}

// --------------------------------------------------------------------------
// GPIO INITIALIZATION
// --------------------------------------------------------------------------
static void gpio_input_init(void)
{
    gpio_config_t c={0};
    uint64_t inputs_mask=
        ((1ULL<<GPIO_BTN_A)|(1ULL<<GPIO_BTN_B)|(1ULL<<GPIO_BTN_DPAD_U)|(1ULL<<GPIO_BTN_DPAD_D)|
         (1ULL<<GPIO_BTN_DPAD_L)|(1ULL<<GPIO_BTN_DPAD_R)|(1ULL<<GPIO_BTN_L)|(1ULL<<GPIO_BTN_R)|
         (1ULL<<GPIO_BTN_START)|(1ULL<<GPIO_BTN_SELECT)|(1ULL<<GPIO_BTN_C_U)|(1ULL<<GPIO_BTN_C_D)|
         (1ULL<<GPIO_BTN_C_L)|(1ULL<<GPIO_BTN_C_R));

    c.pin_bit_mask=inputs_mask;
    c.mode=GPIO_MODE_INPUT;
    gpio_config(&c);

    c.pin_bit_mask=(1ULL<<GPIO_BTN_SYNC);
    c.mode=GPIO_MODE_INPUT;
    c.pull_up_en=GPIO_PULLUP_ENABLE;
    gpio_config(&c);
}

// --------------------------------------------------------------------------
// CONTROLLER INPUT TASK (reads GPIO + handles SYNC button + LEDs)
// --------------------------------------------------------------------------
static void controller_task(void* arg)
{
    i2cinput_input_s input = {0};
    led_set_state(LED_IDLE);

    while (true)
    {
        memset(&input, 0, sizeof(input));

        // =====================================================
        // SYNC combo — L + R + C-Down
        //   >= 3000 ms: mode-specific re-pair / change host
        // =====================================================
        static bool    combo_active = false;
        static int64_t combo_start_us = 0;
        static bool    combo_reset_fired = false;
        static bool    combo_reset_armed = true;

        bool combo_now = (!gpio_get_level(GPIO_BTN_L) &&
                          !gpio_get_level(GPIO_BTN_R) &&
                          !gpio_get_level(GPIO_BTN_C_D));
        int64_t now_us = esp_timer_get_time();

        if (combo_now && !combo_active) {
            combo_active = true;
            combo_start_us = now_us;
            combo_reset_fired = false;
            ESP_LOGI(TAG, "SYNC combo started");
        }

        if (!combo_now) {
            combo_reset_armed = true;
        }

        if (combo_now && combo_active) {
            int64_t held_ms = (now_us - combo_start_us) / 1000;

            if (combo_reset_armed && !combo_reset_fired && held_ms >= 3000) {
                combo_reset_fired = true;
                combo_reset_armed = false;
                ESP_LOGW(TAG, "SYNC hold >= 3000 ms → factory reset");

                led_set_state(LED_ERROR);
                vTaskDelay(pdMS_TO_TICKS(250));

                input_mode_t mode = get_current_mode();

                switch (mode) {

                    case INPUT_MODE_DINPUT_BR:
                    case INPUT_MODE_DINPUT_VBGO:
                        ESP_LOGW(TAG, "Clearing DInput pairing…");

                        memset(global_loaded_settings.paired_host_dinput_mac, 0, 6);
                        global_loaded_settings.has_paired_dinput = false;

                        memset(global_loaded_settings.device_mac_dinput, 0, 6);
                        ESP_LOGW(TAG, "Cleared device_mac_dinput");

                        app_settings_save();
                        break;

                    case INPUT_MODE_SWPRO:
                    case INPUT_MODE_SNES:
                    case INPUT_MODE_N64:
                        ESP_LOGW(TAG, "Clearing Switch-family pairing ONLY");
                        memset(global_loaded_settings.paired_host_switch_mac, 0, 6);
                        global_loaded_settings.has_paired_switch = false;
                        break;

                    default:
                        ESP_LOGW(TAG, "Unknown mode — no pairing cleared");
                        break;
                }

                app_settings_save();

                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                led_set_state(LED_PAIRING);
                vTaskDelay(pdMS_TO_TICKS(250));
                esp_restart();
            }
        }

        if (!combo_now && combo_active) {
            combo_active = false;
        }

        // =====================================================
        // MAIN CONTROLLER MAPPING
        // =====================================================
        switch (get_current_mode())
        {
            case INPUT_MODE_SWPRO:
                input.dpad_up       = !gpio_get_level(GPIO_BTN_DPAD_U);
                input.dpad_down     = !gpio_get_level(GPIO_BTN_DPAD_D);
                input.dpad_left     = !gpio_get_level(GPIO_BTN_DPAD_L);
                input.dpad_right    = !gpio_get_level(GPIO_BTN_DPAD_R);

                input.lx            = 2048;
                input.ly            = 2048;
                input.rx            = 2048;
                input.ry            = 2048;

                if (!gpio_get_level(GPIO_BTN_C_R)) input.rx = 4095;
                else if (!gpio_get_level(GPIO_BTN_C_L)) input.rx = 0;
                if (!gpio_get_level(GPIO_BTN_C_U)) input.ry = 4095;
                else if (!gpio_get_level(GPIO_BTN_C_D)) input.ry = 0;

                input.button_plus   = !gpio_get_level(GPIO_BTN_START);
                input.button_minus  = false;

                input.button_east   = !gpio_get_level(GPIO_BTN_A);
                input.button_south  = !gpio_get_level(GPIO_BTN_B);

                input.trigger_l     = !gpio_get_level(GPIO_BTN_L);
                input.trigger_r     = !gpio_get_level(GPIO_BTN_R);

                input.button_stick_left  = false;
                input.button_stick_right = false;
                input.button_home        = false;
                input.button_capture     = false;
                input.trigger_zl         = false;
                input.trigger_zr         = false;
                input.button_north       = false;
                input.button_west        = false;
                break;

            case INPUT_MODE_SNES:
                input.button_east   = !gpio_get_level(GPIO_BTN_C_R);
                input.button_south  = !gpio_get_level(GPIO_BTN_C_D);
                input.button_west   = !gpio_get_level(GPIO_BTN_C_L);
                input.button_north  = !gpio_get_level(GPIO_BTN_C_U);
                input.dpad_up       = !gpio_get_level(GPIO_BTN_DPAD_U);
                input.dpad_down     = !gpio_get_level(GPIO_BTN_DPAD_D);
                input.dpad_left     = !gpio_get_level(GPIO_BTN_DPAD_L);
                input.dpad_right    = !gpio_get_level(GPIO_BTN_DPAD_R);
                input.trigger_l     = !gpio_get_level(GPIO_BTN_L);
                input.trigger_r     = !gpio_get_level(GPIO_BTN_R);
                input.trigger_zl    = !gpio_get_level(GPIO_BTN_B);
                input.trigger_zr    = !gpio_get_level(GPIO_BTN_A);
                input.button_plus   = !gpio_get_level(GPIO_BTN_START);
                input.button_minus  = !gpio_get_level(GPIO_BTN_SELECT);
                input.lx = 2048; input.ly = 2048;
                input.rx = 2048; input.ry = 2048;
                break;

            case INPUT_MODE_DINPUT_BR:
            case INPUT_MODE_DINPUT_VBGO:
                input.button_east   = !gpio_get_level(GPIO_BTN_A);
                input.button_south  = !gpio_get_level(GPIO_BTN_B);
                input.button_west   = !gpio_get_level(GPIO_BTN_C_L);
                input.button_north  = !gpio_get_level(GPIO_BTN_C_U);

                input.dpad_up       = !gpio_get_level(GPIO_BTN_DPAD_U);
                input.dpad_down     = !gpio_get_level(GPIO_BTN_DPAD_D);
                input.dpad_left     = !gpio_get_level(GPIO_BTN_DPAD_L);
                input.dpad_right    = !gpio_get_level(GPIO_BTN_DPAD_R);

                input.lt            = 0x00;
                input.rt            = 0x00;

                input.rx            = !gpio_get_level(GPIO_BTN_C_R) ? 0xFFFF : 0x7FFF;
                input.ry            = !gpio_get_level(GPIO_BTN_C_D) ? 0x0000 : 0x7FFF;

                input.button_plus   = !gpio_get_level(GPIO_BTN_START);

                if (get_current_mode() == INPUT_MODE_DINPUT_BR) {
                    input.button_minus = !gpio_get_level(GPIO_BTN_SELECT);
                } else {
                    input.button_minus = false;
                }

                input.trigger_l     = !gpio_get_level(GPIO_BTN_L);
                input.trigger_r     = !gpio_get_level(GPIO_BTN_R);
                input.trigger_zl    = false;
                input.trigger_zr    = false;
                break;

            case INPUT_MODE_N64:
            default:
                input.button_east        = !gpio_get_level(GPIO_BTN_A);
                input.button_south       = !gpio_get_level(GPIO_BTN_B);
                input.dpad_up            = !gpio_get_level(GPIO_BTN_DPAD_U);
                input.dpad_down          = !gpio_get_level(GPIO_BTN_DPAD_D);
                input.dpad_left          = !gpio_get_level(GPIO_BTN_DPAD_L);
                input.dpad_right         = !gpio_get_level(GPIO_BTN_DPAD_R);
                input.button_north       = !gpio_get_level(GPIO_BTN_C_L);
                input.button_west        = !gpio_get_level(GPIO_BTN_C_U);
                input.button_minus       = !gpio_get_level(GPIO_BTN_C_R);
                input.trigger_zr         = !gpio_get_level(GPIO_BTN_C_D);
                input.trigger_l          = !gpio_get_level(GPIO_BTN_L);
                input.trigger_r          = !gpio_get_level(GPIO_BTN_R);
                input.button_plus        = !gpio_get_level(GPIO_BTN_START);
                input.trigger_zl         = false;
                input.button_stick_left  = false;
                input.button_stick_right = false;
                input.lx = 2048; input.ly = 2048;
                input.rx = 2048; input.ry = 2048;
                break;
        }

        // =====================================================
        // SIMPLE COMBOS
        // =====================================================
        bool sel    = !gpio_get_level(GPIO_BTN_SELECT);
        bool trig_l = !gpio_get_level(GPIO_BTN_L);
        bool trig_r = !gpio_get_level(GPIO_BTN_R);
        bool c_u    = !gpio_get_level(GPIO_BTN_C_U);
        bool c_l    = !gpio_get_level(GPIO_BTN_C_L);
        bool c_d    = !gpio_get_level(GPIO_BTN_C_D);

        bool mode_swpro  = (get_current_mode() == INPUT_MODE_SWPRO);
        bool mode_n64    = (get_current_mode() == INPUT_MODE_N64);
        bool mode_dinput = ((get_current_mode() == INPUT_MODE_DINPUT_BR) ||
                            (get_current_mode() == INPUT_MODE_DINPUT_VBGO));

        // --- SWPRO ---
        if (mode_swpro) {
            static bool    sel_prev = false;
            static bool    sel_modified = false;
            static uint8_t minus_pulse = 0;

            static uint8_t swpro_lr_pending = 0;
            static uint8_t swpro_lr_wait_frames = 0;
            const uint8_t SWPRO_LR_GRACE_FRAMES = 6;

            bool consume_select = false;
            if (sel && trig_l && trig_r) consume_select = true;
            else if (sel && c_u)         consume_select = true;
            else if (sel && c_l)         consume_select = true;
            else if (sel && c_d)         consume_select = true;
            else if (sel && trig_l && !trig_r) consume_select = true;
            else if (sel && trig_r && !trig_l) consume_select = true;

            bool any_modifier = trig_l || trig_r || c_u || c_l || c_d;

            if (minus_pulse > 0) {
                input.button_minus = true;
                minus_pulse--;
            }

            if (sel) {
                if (any_modifier || consume_select) {
                    sel_modified = true;
                }
            } else {
                if (sel_prev && !sel_modified) {
                    minus_pulse = 3;
                }
                sel_modified = false;
            }
            sel_prev = sel;

            if (!sel) {
                swpro_lr_pending = 0;
                swpro_lr_wait_frames = 0;
            } else {
                if (trig_l && trig_r) {
                    swpro_lr_pending = 0;
                    swpro_lr_wait_frames = 0;
                } else if (trig_l ^ trig_r) {
                    uint8_t side = trig_l ? 1 : 2;
                    if (swpro_lr_pending != side) {
                        swpro_lr_pending = side;
                        swpro_lr_wait_frames = 0;
                    }
                    if (swpro_lr_wait_frames < 255) swpro_lr_wait_frames++;
                } else {
                    swpro_lr_pending = 0;
                    swpro_lr_wait_frames = 0;
                }
            }

            if (sel && trig_l && trig_r) {
                input.trigger_zl = true;
                input.trigger_zr = true;
            }
            else if (sel && c_u) {
                input.button_north = true;
                input.rx = 2048; input.ry = 2048;
            }
            else if (sel && c_l) {
                input.button_west = true;
                input.rx = 2048; input.ry = 2048;
            }
            else if (sel && c_d) {
                input.button_stick_right = true;
                input.rx = 2048; input.ry = 2048;
            }
            else if (sel &&
                     swpro_lr_pending == 1 &&
                     swpro_lr_wait_frames >= SWPRO_LR_GRACE_FRAMES &&
                     trig_l && !trig_r) {
                input.button_home = true;
            }
            else if (sel &&
                     swpro_lr_pending == 2 &&
                     swpro_lr_wait_frames >= SWPRO_LR_GRACE_FRAMES &&
                     trig_r && !trig_l) {
                input.button_capture = true;
            }
        }

        // --- N64 ---
        if (mode_n64) {
            if (sel && trig_l && trig_r) {
                input.button_stick_left = true;
                input.trigger_zl        = true;
            }
            else if (sel) {
                input.trigger_zl = true;
            }
        }

                // --- DINPUT ---
        if (mode_dinput) {
            if (get_current_mode() == INPUT_MODE_DINPUT_VBGO) {
                static bool    dsel_prev = false;
                static bool    dsel_modified = false;
                static uint8_t dminus_pulse = 0;

                static uint8_t dinput_lr_pending = 0;
                static uint8_t dinput_lr_wait_frames = 0;
                const uint8_t DINPUT_LR_GRACE_FRAMES = 6;

                bool d_combo_x = sel && trig_l && !trig_r;
                bool d_combo_y = sel && trig_r && !trig_l;

                bool d_consume_select = d_combo_x || d_combo_y;
                bool d_any_modifier   = trig_l || trig_r;

                if (dminus_pulse > 0) {
                    input.button_minus = true;
                    dminus_pulse--;
                }

                if (sel) {
                    if (d_any_modifier || d_consume_select) {
                        dsel_modified = true;
                    }
                } else {
                    if (dsel_prev && !dsel_modified) {
                        dminus_pulse = 3;
                    }
                    dsel_modified = false;
                }
                dsel_prev = sel;

                if (!sel) {
                    dinput_lr_pending = 0;
                    dinput_lr_wait_frames = 0;
                } else {
                    if (trig_l && trig_r) {
                        dinput_lr_pending = 0;
                        dinput_lr_wait_frames = 0;
                    } else if (trig_l ^ trig_r) {
                        uint8_t side = trig_l ? 1 : 2;
                        if (dinput_lr_pending != side) {
                            dinput_lr_pending = side;
                            dinput_lr_wait_frames = 0;
                        }
                        if (dinput_lr_wait_frames < 255) dinput_lr_wait_frames++;
                    } else {
                        dinput_lr_pending = 0;
                        dinput_lr_wait_frames = 0;
                    }
                }

                if (sel && dinput_lr_pending != 0) {
                    input.button_minus = false;

                    if (dinput_lr_pending == 1) {
                        input.trigger_l = false;
                    } else if (dinput_lr_pending == 2) {
                        input.trigger_r = false;
                    }
                }

                if (sel &&
                    dinput_lr_pending == 1 &&
                    dinput_lr_wait_frames >= DINPUT_LR_GRACE_FRAMES &&
                    trig_l && !trig_r) {
                    input.trigger_zl   = true;
                    input.button_minus = false;
                    input.trigger_l    = false;
                }
                else if (sel &&
                         dinput_lr_pending == 2 &&
                         dinput_lr_wait_frames >= DINPUT_LR_GRACE_FRAMES &&
                         trig_r && !trig_l) {
                    input.trigger_zr   = true;
                    input.button_minus = false;
                    input.trigger_r    = false;
                }
            }
            else {
                // DINPUT_BR:
                // normal held Select handled in the base mapping above.
            }
        }

        // =====================================================
        // SEND FINAL INPUT REPORT
        // =====================================================
        if ((get_current_mode() == INPUT_MODE_DINPUT_BR) ||
            (get_current_mode() == INPUT_MODE_DINPUT_VBGO))
            dinput_bt_sendinput(&input);
        else
            switch_bt_sendinput(&input);

        vTaskDelay(8/portTICK_PERIOD_MS);
    }
}

// --------------------------------------------------------------------------
// HOJA CALLBACK STUBS
// --------------------------------------------------------------------------
void app_set_connected_status(uint8_t s)
{
    bt_connected = (s != 0);
    bt_pairing = !bt_connected;
    bt_error = false;
    led_set_player_number(s);
}

void app_set_standard_haptic(uint8_t l,uint8_t r){ (void)l; (void)r; }
void app_set_sinput_haptic(uint8_t*d,uint8_t l){ (void)d; (void)l; }

void app_set_switch_haptic(uint8_t *d)
{
    if (!d) return;
    haptics_rumble_translate(d);
}

void app_set_power_setting(i2c_power_code_t p){ (void)p; }

void app_save_host_mac(input_mode_t m, uint8_t *a)
{
    if (!a) return;

    bool changed = false;

    switch (m)
    {
        case INPUT_MODE_DINPUT_BR:
        case INPUT_MODE_DINPUT_VBGO:
            if (memcmp(global_loaded_settings.paired_host_dinput_mac, a, 6) != 0)
            {
                memcpy(global_loaded_settings.paired_host_dinput_mac, a, 6);
                global_loaded_settings.has_paired_dinput = true;
                changed = true;
                ESP_LOGI("MAIN", "Updated paired DInput host MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                         a[0], a[1], a[2], a[3], a[4], a[5]);
            }
            break;

        case INPUT_MODE_SWPRO:
        case INPUT_MODE_SNES:
        case INPUT_MODE_N64:
        default:
            if (memcmp(global_loaded_settings.paired_host_switch_mac, a, 6) != 0)
            {
                memcpy(global_loaded_settings.paired_host_switch_mac, a, 6);
                global_loaded_settings.has_paired_switch = true;
                changed = true;
                ESP_LOGI("MAIN", "Updated paired Switch host MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                         a[0], a[1], a[2], a[3], a[4], a[5]);
            }
            break;
    }

    if (changed)
    {
        app_settings_save();
    }
}

bool app_compare_mac(uint8_t*a,uint8_t*b){ return memcmp(a,b,6)==0; }

uint64_t get_timestamp_ms(void){ return esp_timer_get_time()/1000ULL; }
uint64_t get_timestamp_us(void){ return esp_timer_get_time(); }
uint32_t get_timer_value(void){ return (uint32_t)esp_timer_get_time(); }

void app_set_report_timer(uint64_t t)
{
    _app_report_timer_us_default = t;
    _app_report_timer_us = _app_report_timer_us_default;
}

uint64_t app_get_report_timer(void){
    return _app_report_timer_us;
}

// --------------------------------------------------------------------------
// APP MAIN
// --------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "System start");

    uint8_t mac[6];
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    ESP_LOGI("MACCHECK", "esp_efuse_mac_get_default() returned %s", esp_err_to_name(ret));
    ESP_LOGI("MACCHECK", "Base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    app_settings_load();
    gpio_input_init();
    haptics_init();
    led_init();
    led_boot_sweep();
    led_set_state(LED_IDLE);

    select_boot_mode_from_right_dpad();

    switch (get_current_mode()) {
        case INPUT_MODE_SWPRO:       led_set_state(LED_PAIRING);   break;
        case INPUT_MODE_SNES:        led_set_state(LED_CONNECTED); break;
        case INPUT_MODE_DINPUT_BR:   led_set_state(LED_DINPUT);    break;
        case INPUT_MODE_DINPUT_VBGO: led_set_state(LED_DINPUT);    break;
        case INPUT_MODE_N64:
        default:                     led_set_state(LED_IDLE);      break;
    }

    uint8_t base_mac[6];
    esp_read_mac(base_mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "Applying base BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             base_mac[0], base_mac[1], base_mac[2],
             base_mac[3], base_mac[4], base_mac[5]);

    esp_base_mac_addr_set(base_mac);

    switch (get_current_mode())
    {
        case INPUT_MODE_DINPUT_BR:
        case INPUT_MODE_DINPUT_VBGO:
            ESP_LOGI(TAG, "Stored paired host (DInput): %02X:%02X:%02X:%02X:%02X:%02X",
                global_loaded_settings.paired_host_dinput_mac[0],
                global_loaded_settings.paired_host_dinput_mac[1],
                global_loaded_settings.paired_host_dinput_mac[2],
                global_loaded_settings.paired_host_dinput_mac[3],
                global_loaded_settings.paired_host_dinput_mac[4],
                global_loaded_settings.paired_host_dinput_mac[5]);
            break;

        case INPUT_MODE_SWPRO:
        case INPUT_MODE_SNES:
        case INPUT_MODE_N64:
        default:
            ESP_LOGI(TAG, "Stored paired host (Switch): %02X:%02X:%02X:%02X:%02X:%02X",
                global_loaded_settings.paired_host_switch_mac[0],
                global_loaded_settings.paired_host_switch_mac[1],
                global_loaded_settings.paired_host_switch_mac[2],
                global_loaded_settings.paired_host_switch_mac[3],
                global_loaded_settings.paired_host_switch_mac[4],
                global_loaded_settings.paired_host_switch_mac[5]);
            break;
    }

    ESP_LOGI(TAG, "Emulating as: %s",
        get_current_mode() == INPUT_MODE_SWPRO       ? "Pro Controller" :
        get_current_mode() == INPUT_MODE_SNES        ? "SNES Controller" :
        get_current_mode() == INPUT_MODE_N64         ? "N64 Controller" :
        get_current_mode() == INPUT_MODE_DINPUT_BR   ? "DInput Controller (BlueRetro)" :
        get_current_mode() == INPUT_MODE_DINPUT_VBGO ? "DInput Controller (VBGo)" :
        "Unknown");

    int bt_status = 0;

    switch (get_current_mode()) {
        case INPUT_MODE_DINPUT_BR:
        case INPUT_MODE_DINPUT_VBGO:
            ESP_LOGI(TAG, "Starting DInput mode...");
            bt_status = core_bt_dinput_start();
            break;

        case INPUT_MODE_SWPRO:
        case INPUT_MODE_SNES:
        case INPUT_MODE_N64:
        default:
            ESP_LOGI(TAG, "Starting Switch mode...");
            bt_status = core_bt_switch_start();
            break;
    }

    if (bt_status == ESP_OK) {
        led_set_state(LED_PAIRING);
    } else {
        led_set_state(LED_ERROR);
    }

    esp_read_mac(global_live_data.current_mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "Post-start BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             global_live_data.current_mac[0], global_live_data.current_mac[1],
             global_live_data.current_mac[2], global_live_data.current_mac[3],
             global_live_data.current_mac[4], global_live_data.current_mac[5]);

    bt_pairing = true;
    xTaskCreatePinnedToCore(controller_task, "controller_task", 4096, NULL, 1, NULL, 1);
}