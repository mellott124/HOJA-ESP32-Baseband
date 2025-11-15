#include "core_bt_dinput.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LED.h"
#include "drv2605_esp.h"

extern void app_settings_save(void);
static i2cinput_input_s _last_input = {0};

// -------------------------------
// Globals
// -------------------------------
static volatile bool _hid_connected = false;
static volatile bool _dinput_paired = false;

TaskHandle_t _dinput_bt_task_handle = NULL;
static interval_s _dinput_interval = {0};

// Internal working report
static dinput_report_s _dinput_report = {
    .x = 128,
    .y = 128,
    .z = 128,
    .rz = 128,
    .buttons = 0
};

// -------------------------------
// DInput HID Descriptor
// -------------------------------
// Report ID = 1
// 4 axes + 16 buttons
// -------------------------------
// HID Report Descriptor (DInput)
// -------------------------------
static const uint8_t dinput_hid_descriptor[] = {

    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Gamepad)
    0xA1, 0x01,        // Collection (Application)

        // Report ID
        0x85, 0x01,    // Report ID = 1

        // -------------------------------
        // 4 Analog Axes (X,Y,Z,Rz)
        // -------------------------------
        0x05, 0x01,    // Usage Page (Generic Desktop)

        0x09, 0x30,    // Usage (X)
        0x09, 0x31,    // Usage (Y)
        0x09, 0x32,    // Usage (Z)
        0x09, 0x35,    // Usage (Rz)

        0x15, 0x00,    // Logical Min = 0
        0x26, 0xFF, 0x00, // Logical Max = 255
        0x75, 0x08,    // Report Size = 8 bits
        0x95, 0x04,    // Report Count = 4
        0x81, 0x02,    // Input (Data,Var,Abs)

        // -------------------------------
        // 16 Buttons
        // -------------------------------
        0x05, 0x09,    // Usage Page (Buttons)
        0x19, 0x01,    // Usage Minimum = Button 1
        0x29, 0x10,    // Usage Maximum = Button 16

        0x15, 0x00,    // Logical Min = 0
        0x25, 0x01,    // Logical Max = 1

        0x75, 0x01,    // Report Size = 1 bit
        0x95, 0x10,    // Report Count = 16 buttons

        0x81, 0x02,    // Input (Data,Var,Abs)
		
    // ------ NEW HAPTIC OUTPUT REPORT ------
        0x85, 0x02,     // Report ID 2
        0x09, 0x20,     // Usage (FF Placeholder)
        0x15, 0x00,
        0x26, 0xFF, 0x00,
        0x75, 0x08,
        0x95, 0x02,     // strong + weak rumble
        0x91, 0x02,     // Output (Data,Var,Abs)
    // ---------------------------------------
    0xC0              // End Collection
};

esp_hid_raw_report_map_t dinput_report_maps[1] = {
    {
        .data = dinput_hid_descriptor,
        .len  = sizeof(dinput_hid_descriptor)
    }
};

// -------------------------------
// Device Identity (Community VID/PID)
// -------------------------------
util_bt_app_params_s dinput_app_params = {
    .hidd_cb   = dinput_bt_hidd_cb,
    .gap_cb    = dinput_bt_gap_cb,
    .bt_mode   = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t dinput_hidd_config = {
    .vendor_id = 0x1209,
    .product_id = 0x0001,
    .version = 0x0001,
    .device_name = "RetroOnyx Controller",
    .manufacturer_name = "RetroOnyx",
    .serial_number = "000001",
    .report_maps = dinput_report_maps,
    .report_maps_len = 1,
};

// -------------------------------
// GAP Callback
// -------------------------------
void dinput_bt_gap_cb(esp_bt_gap_cb_event_t event,
                      esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "dinput_gap";

    switch (event)
    {
        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
            ESP_LOGI(TAG, "ACL CONNECT.");
            if (_dinput_bt_task_handle == NULL) {
                xTaskCreatePinnedToCore(
                    _dinput_bt_task,
                    "dinput_send_task",
                    4096,
                    NULL,
                    0,
                    &_dinput_bt_task_handle,
                    0
                );
            }
            break;

        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
            ESP_LOGI(TAG, "ACL DISCONNECT.");
            if (_dinput_bt_task_handle) {
                ESP_LOGI(TAG, "Deferring send_task delete");
                xTaskNotifyGive(_dinput_bt_task_handle);
                _dinput_bt_task_handle = NULL;
            }
            app_set_connected_status(0);
            app_set_power_setting(POWER_CODE_OFF);
            break;

        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "authentication success");

                if (!_dinput_paired) {
                    _dinput_paired = true;
                    app_save_host_mac(INPUT_MODE_DINPUT,
                                      &param->auth_cmpl.bda[0]);
                }
            } else {
                ESP_LOGI(TAG, "auth failed: %d",
                         param->auth_cmpl.stat);
            }
            break;

        default:
            ESP_LOGI(TAG, "GAP EVT: %d", event);
            break;
    }
}

// -------------------------------
// HID Callback
// -------------------------------
void dinput_bt_hidd_cb(void *handler_args,
                       esp_event_base_t base,
                       int32_t id,
                       void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "dinput_hidd";

    switch (event)
    {
        case ESP_HIDD_START_EVENT:
            if (param->start.status == ESP_OK) {
                ESP_LOGI(TAG, "START OK");
                led_set_state(LED_PAIRING);

                if (_dinput_paired) {
                    // --- Autoconnect to last paired Windows host ---
                    ESP_LOGI(TAG, "Autoconnect: trying stored host...");
                    int64_t uptime = esp_timer_get_time() / 1000;
                    int delay = (uptime < 3000) ? 700 : 200;
                    vTaskDelay(pdMS_TO_TICKS(delay));

                    util_bluetooth_connect(
                        global_loaded_settings.paired_host_dinput_mac);

                } else {
                    // --- First-time pairing mode ---
                    ESP_LOGI(TAG, "No pair record — entering discoverable mode");
                    esp_bt_gap_set_scan_mode(
                        ESP_BT_CONNECTABLE,
                        ESP_BT_GENERAL_DISCOVERABLE
                    );
                }
            } else {
                ESP_LOGE(TAG, "START failed");
                led_set_state(LED_ERROR);
            }
            break;

        case ESP_HIDD_CONNECT_EVENT:
            if (param->connect.status == ESP_OK) {
                _hid_connected = true;
                ESP_LOGI(TAG, "CONNECT OK");
                led_set_player_number(1); //Blinking Green Connected State to mimic Switch
            } else {
                ESP_LOGE(TAG, "connect failed");
                led_set_state(LED_ERROR);
            }
            break;

        case ESP_HIDD_DISCONNECT_EVENT:
            _hid_connected = false;
            ESP_LOGI(TAG, "DISCONNECT");

            led_set_state(LED_PAIRING);
            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_GENERAL_DISCOVERABLE
            );
            break;

        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(TAG, "HID STOP");
            _hid_connected = false;
            led_set_state(LED_IDLE);
            break;
			
		case ESP_HIDD_OUTPUT_EVENT: {
			const uint8_t *data = param->output.data;
			int len = param->output.length;

			ESP_LOGI(TAG, "HID OUT (ReportID=%d, len=%d)", data[0], len);

			// Report ID must be 2 for rumble
			if (len >= 3 && data[0] == 0x02) {
				uint8_t strong = data[1];
				uint8_t weak   = data[2];

				// Pick max for single-motor DRV2605
				uint8_t level = strong > weak ? strong : weak;

				ESP_LOGI(TAG, "Rumble strong=%u weak=%u final=%u",
						 strong, weak, level);

				drv2605_set_rtp(level);
			}

				break;
		}

        default:
            break;
    }
}

// -------------------------------
// Start / Stop
// -------------------------------
int core_bt_dinput_start(void)
{
    const char *TAG = "core_bt_dinput_start";
    ESP_LOGI(TAG, "Starting DInput Bluetooth stack...");

    int err = 1;

    uint8_t tmpmac[6] = {0};

    // ------------------------------------------------------
    // Correct MAC handling: EFUSE fallback + save
    // ------------------------------------------------------
    uint8_t base_mac[6];
    esp_efuse_mac_get_default(base_mac);

    ESP_LOGI(TAG,
        "EFUSE base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        base_mac[0], base_mac[1], base_mac[2],
        base_mac[3], base_mac[4], base_mac[5]);

    const uint8_t zero[6] = {0};

    // Case 1: stored MAC is valid
    if (memcmp(global_loaded_settings.device_mac_dinput, zero, 6) != 0) {
        ESP_LOGI(TAG, "Using stored DInput MAC.");
        memcpy(tmpmac, global_loaded_settings.device_mac_dinput, 6);
    }
    // Case 2: stored MAC is zero → regenerate from EFUSE
    else {
        ESP_LOGI(TAG, "Generating new DInput MAC from efuse base.");
        memcpy(tmpmac, base_mac, 6);
        tmpmac[5] -= 2;      // Espressif rule for BT Classic

        // Save the new MAC
        memcpy(global_loaded_settings.device_mac_dinput, tmpmac, 6);
        app_settings_save();
        ESP_LOGI(TAG, "Saved new DInput MAC.");
    }

    ESP_LOGI(TAG,
        "Final DInput BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        tmpmac[0], tmpmac[1], tmpmac[2],
        tmpmac[3], tmpmac[4], tmpmac[5]);

    // ------------------------------------------------------
    // Pairing status
    // ------------------------------------------------------
    if (memcmp(global_loaded_settings.paired_host_dinput_mac, zero, 6) != 0) {

        // ------------------------------------------------------
        // SAFETY FIX:
        // If paired-host MAC equals OUR OWN MAC → INVALID.
        // This happens after long-sync resets and causes
        // endless autoconnect failures.
        // ------------------------------------------------------
        if (memcmp(global_loaded_settings.paired_host_dinput_mac,
                   tmpmac, 6) == 0)
        {
            ESP_LOGW(TAG,
                "Paired host matches device MAC — forcing discoverable mode");

            memset(global_loaded_settings.paired_host_dinput_mac, 0, 6);
            app_settings_save();

            _dinput_paired = false;

            esp_bt_gap_set_scan_mode(
                ESP_BT_CONNECTABLE,
                ESP_BT_GENERAL_DISCOVERABLE
            );
        }
        else
        {
            _dinput_paired = true;
            ESP_LOGI(TAG,
                "Stored DInput paired host found — enabling autoconnect");
        }
    }
    else {
        _dinput_paired = false;
        ESP_LOGI(TAG, "No paired host — entering discoverable mode");
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        );
    }

    // ------------------------------------------------------
    // Initialize Bluetooth
    // ------------------------------------------------------
    err = util_bluetooth_init(tmpmac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "util_bluetooth_init FAILED: %d", err);
        return err;
    }

    // ------------------------------------------------------
    // Register HID app
    // ------------------------------------------------------
    err = util_bluetooth_register_app(&dinput_app_params, &dinput_hidd_config);
    ESP_LOGI(TAG, "HID App registration returned: %d", err);

    return err;
}


void core_bt_dinput_stop(void)
{
    util_bluetooth_deinit();
}

// -------------------------------
// DInput Report Task
// -------------------------------
void _dinput_bt_task(void *parameters)
{
    ESP_LOGI("dinput_task", "Starting send loop...");

    app_set_report_timer(8000);

    for (;;)
    {
        // Defer delete if notified
        if (ulTaskNotifyTake(pdTRUE, 0)) {
            ESP_LOGI("dinput_task", "Delete requested");
            vTaskDelete(NULL);
        }

        if (_hid_connected)
        {
            if (interval_run(get_timestamp_us(),
                             app_get_report_timer(),
                             &_dinput_interval))
            {
                uint8_t buf[DINPUT_REPORT_SIZE];
                buf[0] = _dinput_report.x;
                buf[1] = _dinput_report.y;
                buf[2] = _dinput_report.z;
                buf[3] = _dinput_report.rz;
                buf[4] = (_dinput_report.buttons & 0xFF);
                buf[5] = (_dinput_report.buttons >> 8);

                esp_bt_hid_device_send_report(
                    ESP_HIDD_REPORT_TYPE_INTRDATA,
                    DINPUT_REPORT_ID,
                    DINPUT_REPORT_SIZE,
                    buf
                );
            }
        }
        else {
            vTaskDelay(16 / portTICK_PERIOD_MS);
        }
    }
}

// ============================================================================
// IMPORTANT: HOJA INPUT FIELD INTERPRETATION
// ============================================================================
//
// HOJA input fields (button_north, button_west, rx, ry, ax, ay, etc.) DO NOT
// have semantic meaning. They DO NOT represent Up/Down/Left/Right or X/Y/Z.
//
// They are simply BIT LANES coming out of the HOJA input multiplexer.
// The meaning of each bit depends entirely on the controller mode:
//   • Virtual Boy mode
//   • N64 mode
//   • Switch Pro mode
//   • SNES mode
//   • Others...
//
// For example:
//   - In VB mode, right-dpad UP appears on input->button_north
//   - In another mode, input->button_north might be "X button"
//   - In another, it might represent "face button west"
//   - rx/ry fields are ALWAYS populated (analog reads) and must not be used
//     as boolean truth tests.
//
// Because of this, we must *never* rely on HOJA field NAMES to infer direction
// or meaning. They are simply signals.
//
// Instead, we derive meaning only from:
//   1. Real debug logs that show which HOJA bit fires for each VB button
//   2. Explicit comparisons (e.g., input->rx == 65535) instead of boolean tests
//
// This prevents:
//   - Wrong direction detection
//   - Axes stuck at max
//   - Inputs firing continuously
//   - Drift between modes
//
// Summary:
//   Treat HOJA fields as raw wires, not meaningful names.
//   Only map based on your validated debug table.
//
// ============================================================================
// +-----------------------+----------------------+---------------------------+
// | VB PHYSICAL BUTTON    | HOJA FIELD FIRED     | DINPUT OUTPUT             |
// +-----------------------+----------------------+---------------------------+
// | A                     | button_south         | Button 1                  |
// | B                     | button_east          | Button 2                  |
// | Select                | button_minus         | Button 15                 |
// | Start                 | button_plus          | Button 16                 |
// | L                     | trigger_l            | Button 9                  |
// | R                     | trigger_r            | Button 10                 |
// +-----------------------+----------------------+---------------------------+
// | LEFT DPAD UP          | dpad_up              | Button 5                  |
// | LEFT DPAD DOWN        | dpad_down            | Button 6                  |
// | LEFT DPAD LEFT        | dpad_left            | Button 7                  |
// | LEFT DPAD RIGHT       | dpad_right           | Button 8                  |
// +-----------------------+----------------------+---------------------------+
// | RIGHT DPAD UP         | button_north         | Z Rotation = 0            |
// | RIGHT DPAD DOWN       | ry == 0              | Z Rotation = 255          |
// | RIGHT DPAD LEFT       | button_west          | Z = 0                     |
// | RIGHT DPAD RIGHT      | rx == 65535          | Z = 255                   |
// +-----------------------+----------------------+---------------------------+
// | NO RIGHT DPAD PRESS   | rx≈32767 ry≈32767    | Z = 128, ZRot = 128       |
// +-----------------------+----------------------+---------------------------+
void dinput_bt_sendinput(i2cinput_input_s *input)
{
    if (!input) return;

    debug_print_input_changes(input);

    // -------- LEFT DPAD → BUTTONS ONLY --------
    uint16_t b = 0;

    b |= (input->button_south) << 0;    // A
    b |= (input->button_east)  << 1;    // B
    //b |= (input->button_west)  << 2;    // Y
    //b |= (input->button_north) << 3;    // X

    b |= (input->dpad_up)      << 4;
    b |= (input->dpad_down)    << 5;
    b |= (input->dpad_left)    << 6;
    b |= (input->dpad_right)   << 7;

    b |= (input->trigger_l)    << 8;
    b |= (input->trigger_r)    << 9;
    b |= (input->trigger_zl)   << 10;
    b |= (input->trigger_zr)   << 11;

    b |= (input->button_stick_left)  << 12;
    b |= (input->button_stick_right) << 13;

    b |= (input->button_minus) << 14;  // SELECT
    b |= (input->button_plus)  << 15;  // START

    _dinput_report.buttons = b;

	// -------- RIGHT DPAD → DINPUT AXES (Z + ZROT) --------
	uint8_t z  = 128;   // X-axis of right stick (left/right)
	uint8_t rz = 128;   // Y-axis of right stick (up/down)

	// --- RIGHT DPAD UP ---
	if (input->button_north) {        
		rz = 0;          // Zrot MIN
	}
	// --- RIGHT DPAD DOWN ---
	else if (input->ry == 0) { 
		rz = 255;        // Zrot MAX
	}

	// --- RIGHT DPAD LEFT ---
	if (input->button_west) {    
		z = 0;           // Z MIN
	}
	// --- RIGHT DPAD RIGHT ---
	else if (input->rx == 65535) { 
		z = 255;         // Z MAX
	}

	_dinput_report.z  = z;
	_dinput_report.rz = rz;

}


void dinput_bt_end_task()
{
    if (_dinput_bt_task_handle) {
        xTaskNotifyGive(_dinput_bt_task_handle);
        _dinput_bt_task_handle = NULL;
    }
}

uint8_t normalize_axis(uint16_t v)
{
    // v is 0–4095. Convert to signed (-2048..+2047),
    // then shift to 0–255 with center = 128.
    int16_t centered = (int16_t)v - 2048;      // now -2048..+2047
    centered >>= 3;                            // now approx -256..+255
    return (uint8_t)(centered + 128);          // center becomes 128
}

void debug_print_input_changes(const i2cinput_input_s *in)
{
    static const char *TAG = "INPUTDBG";

    if (memcmp(&_last_input, in, sizeof(i2cinput_input_s)) != 0)
    {
        ESP_LOGI(TAG,
            "UDLR=%d%d%d%d  A=%d B=%d X=%d Y=%d "
            "L=%d ZL=%d R=%d ZR=%d  +/−=%d%d  L3=%d R3=%d | "
            "RX=%u RY=%u LX=%u LY=%u",
            in->dpad_up, in->dpad_down, in->dpad_left, in->dpad_right,
            in->button_south, in->button_east, in->button_north, in->button_west,
            in->trigger_l, in->trigger_zl, in->trigger_r, in->trigger_zr,
            in->button_plus, in->button_minus,
            in->button_stick_left, in->button_stick_right,
            in->rx, in->ry, in->lx, in->ly
        );

        _last_input = *in;
    }
}

