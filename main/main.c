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

static volatile uint64_t _app_report_timer_us = 16000;
static volatile uint64_t _app_report_timer_us_default = 16000;
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
        current_mode = INPUT_MODE_N64;   // Pro Controller
    } else if (c_up_pressed) {
        current_mode = INPUT_MODE_SNES;    // SNES Controller
    } else if (c_right_pressed) {
        current_mode = INPUT_MODE_NES;     // NES Controller
    } else if (c_down_pressed) {
        current_mode = INPUT_MODE_DINPUT;  // DInput Controller
    } else {
        current_mode = current_mode;     // default.  Use the global above.
    }

    if (get_current_mode() == INPUT_MODE_DINPUT) {
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


    // Optional: visual LED feedback (e.g. blink pattern per mode)
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

    // ---------------------------------------------------------------------
    // Ensure new pairing flags exist and are valid
    // ---------------------------------------------------------------------
    if (!global_loaded_settings.has_paired_switch &&
        memcmp(global_loaded_settings.paired_host_switch_mac, "\0\0\0\0\0\0", 6) != 0) {
        global_loaded_settings.has_paired_switch = true;
    }
    if (!global_loaded_settings.has_paired_dinput &&
        memcmp(global_loaded_settings.paired_host_dinput_mac, "\0\0\0\0\0\0", 6) != 0) {
        global_loaded_settings.has_paired_dinput = true;
    }

    // ---------------------------------------------------------------------
    // Initialize the new DInput device MAC if missing
    // ---------------------------------------------------------------------
    if (memcmp(global_loaded_settings.device_mac_dinput, "\0\0\0\0\0\0", 6) == 0) {
        ESP_LOGI(TAG, "Generating independent DInput MAC from Switch MAC...");
        memcpy(global_loaded_settings.device_mac_dinput,
               global_loaded_settings.device_mac_switch, 6);
        global_loaded_settings.device_mac_dinput[5] += 3;  // offset ensures unique identity
        app_settings_save();
    }

    // ---------------------------------------------------------------------
    // Validate struct magic and reset if needed
    // ---------------------------------------------------------------------
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
// RESTART BLUETOOTH PAIRING
// --------------------------------------------------------------------------
static void restart_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing paired host...");
    bt_pairing = true; 
    bt_connected = false; 
    bt_error = false;

    memset(global_loaded_settings.paired_host_switch_mac, 0, 6);
	memset(global_loaded_settings.paired_host_dinput_mac, 0, 6);
	global_loaded_settings.has_paired_dinput = false;
	global_loaded_settings.has_paired_switch = false;
    app_settings_save();

    led_set_state(LED_ERROR);  
    vTaskDelay(pdMS_TO_TICKS(400));
    led_set_state(LED_PAIRING); 
    esp_restart();
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
        // Start each frame clean so no stale bits carry between modes/frames.
        memset(&input, 0, sizeof(input));

        // =====================================================
        // SYNC combo — L + R + C-Down
        // =====================================================
        static bool combo_active = false;
        static int64_t combo_start_us = 0;
        static bool combo_pair_started = false;
        static bool combo_reset_fired = false;
        static bool combo_reset_armed = true;

        bool combo_now = (!gpio_get_level(GPIO_BTN_L) &&
                          !gpio_get_level(GPIO_BTN_R) &&
                          !gpio_get_level(GPIO_BTN_C_D));
        int64_t now_us = esp_timer_get_time();

        if (combo_now && !combo_active) {
            combo_active = true;
            combo_start_us = now_us;
            combo_pair_started = false;
            combo_reset_fired = false;
            ESP_LOGI(TAG, "SYNC combo started");
        }
        if (!combo_now) {
            combo_reset_armed = true;
        }

        if (combo_now && combo_active) {
            int64_t held_ms = (now_us - combo_start_us) / 1000;

            // Switch Pro-style: enter pairing after >= 1000 ms (non-destructive)
            if (!combo_pair_started && held_ms >= 1000) {
                combo_pair_started = true;
                ESP_LOGI(TAG, "SYNC hold >= 1000 ms → enter pairing");

                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                led_set_state(LED_PAIRING);
            }

            // Long hold: factory reset after >= 5000 ms (destructive)
            if (combo_reset_armed && !combo_reset_fired && held_ms >= 5000) {
                combo_reset_fired = true;
                combo_reset_armed = false;
                ESP_LOGW(TAG, "SYNC hold >= 5000 ms → factory reset");

                led_set_state(LED_ERROR);
                vTaskDelay(pdMS_TO_TICKS(250));

                input_mode_t mode = get_current_mode();

                switch (mode) {

                    case INPUT_MODE_DINPUT:
                        ESP_LOGW(TAG, "Clearing DInput pairing…");

                        memset(global_loaded_settings.paired_host_dinput_mac, 0, 6);
                        global_loaded_settings.has_paired_dinput = false;

                        // ALSO RESET THE DEVICE’S OWN DINPUT MAC
                        memset(global_loaded_settings.device_mac_dinput, 0, 6);
                        ESP_LOGW(TAG, "Cleared device_mac_dinput");

                        app_settings_save();
                        break;

                    case INPUT_MODE_SWPRO:
                    case INPUT_MODE_SNES:
                    case INPUT_MODE_NES:
                    case INPUT_MODE_N64:
                        ESP_LOGW(TAG, "Clearing Switch-family pairing ONLY");
                        memset(global_loaded_settings.paired_host_switch_mac, 0, 6);
                        global_loaded_settings.has_paired_switch = false;
                        break;

                    default:
                        ESP_LOGW(TAG, "Unknown mode — no pairing cleared");
                        break;
                }

                // Save updated pairing state
                app_settings_save();

                // Visual feedback + restart
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                led_set_state(LED_PAIRING);
                vTaskDelay(pdMS_TO_TICKS(250));
                esp_restart();
            }
        }

        if (!combo_now && combo_active) {
            combo_active = false;
            int64_t held_ms = (now_us - combo_start_us) / 1000;
            //ESP_LOGI(TAG, "SYNC combo released after %lld ms", held_ms);
        }

        // =====================================================
        // MAIN CONTROLLER MAPPING (base buttons per mode)
        // =====================================================
        switch (get_current_mode())
        {
            case INPUT_MODE_SWPRO:
                //Used RED ALARM in NSO VB to map these out to correctly fit the VB controller
                //Left Dpad
                input.dpad_up       = !gpio_get_level(GPIO_BTN_DPAD_U); // D-pad   →Up
                input.dpad_down     = !gpio_get_level(GPIO_BTN_DPAD_D); // D-pad   →Down
                input.dpad_left     = !gpio_get_level(GPIO_BTN_DPAD_L); // D-pad   →Left
                input.dpad_right    = !gpio_get_level(GPIO_BTN_DPAD_R); // D-pad   →Right

                //Left Analog Stick
                input.lx            = 2048;   // Default center
                input.ly            = 2048;   // Default center

                //Right Dpad via Right analog stick
                input.rx            = 2048;    // Default center
                input.ry            = 2048;    // Default center
                // VB right D-pad (C-pad) → right analog stick
                if (!gpio_get_level(GPIO_BTN_C_R)) input.rx = 4095;     // right
                else if (!gpio_get_level(GPIO_BTN_C_L)) input.rx = 0;   // left
                if (!gpio_get_level(GPIO_BTN_C_U)) input.ry = 4095;     // up
                else if (!gpio_get_level(GPIO_BTN_C_D)) input.ry = 0;   // down

                //Select/Start
                input.button_plus   = !gpio_get_level(GPIO_BTN_START);   // Start → Plus (+)
                input.button_minus  = false;                             // handled in combo section

                //A and B buttons
                input.button_east   = !gpio_get_level(GPIO_BTN_A);       // A
                input.button_south  = !gpio_get_level(GPIO_BTN_B);       // B

                //Triggers
                input.trigger_l     = !gpio_get_level(GPIO_BTN_L);      // L
                input.trigger_r     = !gpio_get_level(GPIO_BTN_R);      // R

                //Analog stick button clicks
                input.button_stick_left  = false;
                input.button_stick_right = false; // generated by combo when needed (IPD)

                //Capture and Home handled below as combo buttons
                break;

            case INPUT_MODE_SNES:
                input.button_east   = !gpio_get_level(GPIO_BTN_C_R);    // C-Right → A
                input.button_south  = !gpio_get_level(GPIO_BTN_C_D);    // C-Down  → B
                input.button_west   = !gpio_get_level(GPIO_BTN_C_L);    // C-Up    → X
                input.button_north  = !gpio_get_level(GPIO_BTN_C_U);    // C-Left  → Y
                input.dpad_up      = !gpio_get_level(GPIO_BTN_DPAD_U);  // D-pad   →Up
                input.dpad_down    = !gpio_get_level(GPIO_BTN_DPAD_D);  // D-pad   →Down
                input.dpad_left    = !gpio_get_level(GPIO_BTN_DPAD_L);  // D-pad   →Left
                input.dpad_right   = !gpio_get_level(GPIO_BTN_DPAD_R);  // D-pad   →Right
                input.trigger_l    = !gpio_get_level(GPIO_BTN_L);       // L
                input.trigger_r    = !gpio_get_level(GPIO_BTN_R);       // R
                input.trigger_zl   = !gpio_get_level(GPIO_BTN_B);       // Optional: ZL = VB B
                input.trigger_zr   = !gpio_get_level(GPIO_BTN_A);       // Optional: ZR = VB A
                input.button_plus  = !gpio_get_level(GPIO_BTN_START);   // Start → Plus
                input.button_minus = !gpio_get_level(GPIO_BTN_SELECT);  // Select → Minus
                input.lx = 2048; input.ly = 2048;
                input.rx = 2048; input.ry = 2048;
                break;

            case INPUT_MODE_NES:
                input.button_east  = !gpio_get_level(GPIO_BTN_A);        // A
                input.button_south = !gpio_get_level(GPIO_BTN_B);        // B
                input.dpad_up      = !gpio_get_level(GPIO_BTN_DPAD_U);   // Up
                input.dpad_down    = !gpio_get_level(GPIO_BTN_DPAD_D);   // Down
                input.dpad_left    = !gpio_get_level(GPIO_BTN_DPAD_L);   // Left
                input.dpad_right   = !gpio_get_level(GPIO_BTN_DPAD_R);   // Right
                input.trigger_l    = !gpio_get_level(GPIO_BTN_L);        // L
                input.trigger_r    = !gpio_get_level(GPIO_BTN_R);        // R
                input.trigger_zl   = false;
                input.trigger_zr   = false;
                input.button_plus  = !gpio_get_level(GPIO_BTN_START);    // Start → Plus
                input.button_minus = !gpio_get_level(GPIO_BTN_SELECT);   // Select → Minus
                input.lx = 2048; input.ly = 2048;
                input.rx = 2048; input.ry = 2048;
                break;

            case INPUT_MODE_DINPUT:
                input.button_east  = !gpio_get_level(GPIO_BTN_A);
                input.button_south = !gpio_get_level(GPIO_BTN_B);
                input.button_west  = !gpio_get_level(GPIO_BTN_C_L);
                input.button_north = !gpio_get_level(GPIO_BTN_C_U);
                // Dpad → dpad bits
                input.dpad_up      = !gpio_get_level(GPIO_BTN_DPAD_U);
                input.dpad_down    = !gpio_get_level(GPIO_BTN_DPAD_D);
                input.dpad_left    = !gpio_get_level(GPIO_BTN_DPAD_L);
                input.dpad_right   = !gpio_get_level(GPIO_BTN_DPAD_R);
                // Triggers → analog
                input.lt           = !gpio_get_level(GPIO_BTN_L) ? 0xFF : 0x00;
                input.rt           = !gpio_get_level(GPIO_BTN_R) ? 0xFF : 0x00;
                // Map VB right dpad → right stick
                input.rx           = !gpio_get_level(GPIO_BTN_C_R) ? 0xFFFF : 0x7FFF;
                input.ry           = !gpio_get_level(GPIO_BTN_C_D) ? 0x0000 : 0x7FFF;
                input.button_plus  = !gpio_get_level(GPIO_BTN_START);
                input.button_minus = !gpio_get_level(GPIO_BTN_SELECT);
                // Optional: emulate LB/RB via triggers or dedicated
                input.trigger_l    = !gpio_get_level(GPIO_BTN_L);
                input.trigger_r    = !gpio_get_level(GPIO_BTN_R);
                input.trigger_zl   = false;
                input.trigger_zr   = false;
                break;

            case INPUT_MODE_N64:
            default:
                input.button_east        = !gpio_get_level(GPIO_BTN_A);        // A
                input.button_south       = !gpio_get_level(GPIO_BTN_B);        // B
                input.dpad_up            = !gpio_get_level(GPIO_BTN_DPAD_U);   // Left D-pad Up
                input.dpad_down          = !gpio_get_level(GPIO_BTN_DPAD_D);   // Left D-pad Down
                input.dpad_left          = !gpio_get_level(GPIO_BTN_DPAD_L);   // Left D-pad Left
                input.dpad_right         = !gpio_get_level(GPIO_BTN_DPAD_R);   // Left D-pad Right
                input.button_north       = !gpio_get_level(GPIO_BTN_C_L);
                input.button_west        = !gpio_get_level(GPIO_BTN_C_U);
                input.button_minus       = !gpio_get_level(GPIO_BTN_C_R);
                input.trigger_zr         = !gpio_get_level(GPIO_BTN_C_D);
                input.trigger_l          = !gpio_get_level(GPIO_BTN_L);        // Trigger Left
                input.trigger_r          = !gpio_get_level(GPIO_BTN_R);        // Trigger Right
                input.button_plus        = !gpio_get_level(GPIO_BTN_START);    // Start → Plus
                input.trigger_zl         = false;   // Confirmed triggers ZL on Switch in N64 mode when set true elsewhere
                input.button_stick_left  = false;   // Confirmed triggers ZR on Switch in N64 mode when set true elsewhere
                input.button_stick_right = false;   // Appears ignored on Switch in N64 mode
                input.lx = 2048; input.ly = 2048;
                input.rx = 2048; input.ry = 2048;
                break;
        }

        // =====================================================
        // SELECT / MODIFIER COMBOS (HOME / CAPTURE / ZL+ZR / X / R3)
        //
        // Added: 2-frame Select qualification for SWPRO Minus to reduce
        // "Select blips" when chord-pressing combos.
        //   - Combos can trigger immediately (no delay)
        //   - Minus only asserts after Select has been held >= 2 frames
        //     AND we are not in a Select+combo this frame.
        // =====================================================
        bool sel    = !gpio_get_level(GPIO_BTN_SELECT);
        bool trig_l = !gpio_get_level(GPIO_BTN_L);
        bool trig_r = !gpio_get_level(GPIO_BTN_R);
        bool c_u    = !gpio_get_level(GPIO_BTN_C_U);  // do NOT change wiring here
        bool c_d    = !gpio_get_level(GPIO_BTN_C_D);

        bool mode_swpro = (get_current_mode() == INPUT_MODE_SWPRO);
        bool mode_n64   = (get_current_mode() == INPUT_MODE_N64);

        // Track whether we matched a Select combo this frame.
        bool consume_select = false;

        // -----------------------------------------------------
        // Combo priority (most specific first)
        // -----------------------------------------------------
        if (mode_swpro && sel && trig_l && trig_r) {
            consume_select = true;
        }
        else if (mode_swpro && sel && c_u) {
            consume_select = true;
        }
        else if (mode_swpro && sel && c_d) {
            consume_select = true;
        }
        else if (mode_swpro && sel && trig_l && !trig_r) {
            consume_select = true;
        }
        else if (mode_swpro && sel && trig_r && !trig_l) {
            consume_select = true;
        }
        else if (!mode_swpro && mode_n64 && sel && trig_l && trig_r) {
            consume_select = true;
        }
        else if (!mode_swpro && mode_n64 && sel) {
            consume_select = true;
        }

        // -----------------------------------------------------
        // 2-frame Select counting (SWPRO only)
        // -----------------------------------------------------
        static uint8_t swpro_sel_frames = 0;

        if (mode_swpro) {
            if (sel) {
                if (swpro_sel_frames < 255) swpro_sel_frames++;
            } else {
                swpro_sel_frames = 0;
            }
        } else {
            swpro_sel_frames = 0;
        }

        // Minus only after Select is held >= 4 frames,
		// not consumed by a combo,
		// AND no combo modifiers are currently down
		if (mode_swpro) {
			bool any_modifier = trig_l || trig_r || c_u || c_d;
			input.button_minus = (sel && (swpro_sel_frames >= 4) && !consume_select && !any_modifier);
		}

        // -----------------------------------------------------
		// Clear ONLY what this combo block can generate,
		// and ONLY in the modes where we generate it.
		// -----------------------------------------------------

		// Always safe: these are only used by combo logic here.
		input.button_home    = false;
		input.button_capture = false;

		if (mode_swpro) {
			// SWPRO combo-generated fields
			input.trigger_zl         = false;
			input.trigger_zr         = false;
			input.button_stick_right = false;  // R3
			input.button_north       = false;  // X
			// (Do NOT clear button_stick_left unless you also generate it in SWPRO)
		}

		if (mode_n64) {
			// N64 combo-generated fields
			input.trigger_zl        = false;
			input.button_stick_left = false;

			// IMPORTANT:
			// Do NOT clear trigger_zr here (N64 base mapping uses it)
			// Do NOT clear button_north here (N64 base mapping uses it)
			// Do NOT clear button_stick_right here unless you generate it in N64
		}

        // --- SWPRO combos (immediate) ---
        if (mode_swpro && sel && trig_l && trig_r) {
            input.trigger_zl = true;
            input.trigger_zr = true;
        }
        else if (mode_swpro && sel && c_u) {
            input.button_north = true;          // X
            input.rx = 2048; input.ry = 2048;
        }
        else if (mode_swpro && sel && c_d) {
            input.button_stick_right = true;    // R3
            input.rx = 2048; input.ry = 2048;
        }
        else if (mode_swpro && sel && trig_l && !trig_r) {
            input.button_home = true;
        }
        else if (mode_swpro && sel && trig_r && !trig_l) {
            input.button_capture = true;
        }
        // --- N64 combos ---
        else if (!mode_swpro && mode_n64 && sel && trig_l && trig_r) {
            input.button_stick_left = true;
            input.trigger_zl        = true;
        }
        else if (!mode_swpro && mode_n64 && sel) {
            input.trigger_zl = true;
        }

        // =====================================================
        // SEND FINAL INPUT REPORT
        // =====================================================
        if (get_current_mode() == INPUT_MODE_DINPUT)
            dinput_bt_sendinput(&input);
        else
            switch_bt_sendinput(&input);

        vTaskDelay(pdMS_TO_TICKS(8));  // ~125 Hz
    }
}

// --------------------------------------------------------------------------
// HOJA CALLBACK STUBS (unchanged)
// --------------------------------------------------------------------------
void app_set_connected_status(uint8_t s)
{
    bt_connected = (s != 0);
    bt_pairing = !bt_connected;
    bt_error = false;
    led_set_player_number(s);  // this now triggers blink + solid
}

void app_set_standard_haptic(uint8_t l,uint8_t r){ (void)l; (void)r; }
void app_set_sinput_haptic(uint8_t*d,uint8_t l){ (void)d; (void)l; }

void app_set_switch_haptic(uint8_t *d)
{
    if (!d) return;
    /* ESP_LOGI("HAPTIC", "Rumble data received:");
    ESP_LOG_BUFFER_HEX("HAPTIC", d, 8); */
    haptics_rumble_translate(d);  // 
}

void app_set_power_setting(i2c_power_code_t p){ (void)p; }

void app_save_host_mac(input_mode_t m, uint8_t *a)
{
    if (!a) return;

    bool changed = false;

    switch (m)
    {
        case INPUT_MODE_DINPUT:
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
        case INPUT_MODE_NES:
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

void app_set_report_timer(uint64_t t){ _app_report_timer_us_default=t; if(!_sniff)_app_report_timer_us=_app_report_timer_us_default; }
uint64_t app_get_report_timer(void){ return _app_report_timer_us; }

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

    // 🌈 Select mode before BT start
    select_boot_mode_from_right_dpad();

    // Optional LED feedback per mode
    switch (get_current_mode()) {
        case INPUT_MODE_SWPRO:  led_set_state(LED_PAIRING);   break;   // Amber
        case INPUT_MODE_SNES:   led_set_state(LED_CONNECTED); break;   // Green
        case INPUT_MODE_NES:    led_set_state(LED_CONNECTED); break;   // Green
        case INPUT_MODE_DINPUT: led_set_state(LED_DINPUT);    break;   // Purple
        case INPUT_MODE_N64:
        default:                led_set_state(LED_IDLE);      break;   // Blue
    }

    uint8_t base_mac[6];
    esp_read_mac(base_mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "Applying base BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             base_mac[0], base_mac[1], base_mac[2],
             base_mac[3], base_mac[4], base_mac[5]);

    esp_base_mac_addr_set(base_mac);

    // ------------------------------------------------------
    // FIXED: Correctly display the stored paired host per mode
    // ------------------------------------------------------
    switch (get_current_mode())
    {
        case INPUT_MODE_DINPUT:
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
        case INPUT_MODE_NES:
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

    // Log emulated identity
    ESP_LOGI(TAG, "Emulating as: %s",
        get_current_mode() == INPUT_MODE_SWPRO ? "Pro Controller" :
        get_current_mode() == INPUT_MODE_SNES  ? "SNES Controller" :
        get_current_mode() == INPUT_MODE_NES   ? "NES Controller" :
        get_current_mode() == INPUT_MODE_N64   ? "N64 Controller" :
        get_current_mode() == INPUT_MODE_DINPUT ? "DInput Controller" :
        "Unknown");

    int bt_status = 0;

    switch (get_current_mode()) {
        case INPUT_MODE_DINPUT:
            ESP_LOGI(TAG, "Starting DInput mode...");
            bt_status = core_bt_dinput_start();
            break;

        case INPUT_MODE_SWPRO:
        case INPUT_MODE_SNES:
        case INPUT_MODE_NES:
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

