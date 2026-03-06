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
static volatile uint8_t _dinput_hat = 0x08;

TaskHandle_t _dinput_bt_task_handle = NULL;
static interval_s _dinput_interval = {0};

// Internal working report
static dinput_report_s _dinput_report = {
    .x = 0,
    .y = 0,
    .z = 0,
    .rz = 0,
    .buttons = 0
};

// -------------------------------
// DInput HID Descriptor
// -------------------------------
// Report ID = 1
// 16 buttons + hat + X/Y/Rx/Ry
// -------------------------------
static const uint8_t dinput_hid_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)

    0x85, 0x01,        //   Report ID (1)

    // ------------------------------------------------
    // 16 buttons
    // ------------------------------------------------
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x10,        //   Usage Maximum (Button 16)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x10,        //   Report Count (16)
    0x81, 0x02,        //   Input (Data,Var,Abs)

    // ------------------------------------------------
    // Hat switch for LEFT VB D-pad
    // ------------------------------------------------
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (English Rotation, Angular Position)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (Data,Var,Abs,Null State)

    // Padding nibble
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Const,Var,Abs)

    // ------------------------------------------------
    // 4 axes: X, Y, Rx, Ry
    // X/Y centered, RIGHT VB D-pad on Rx/Ry
    // ------------------------------------------------
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x81,        //   Logical Minimum (-127)
    0x25, 0x7F,        //   Logical Maximum (127)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs)

    0xC0               // End Collection
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
    .device_name = "RetroOnyx Gamepad",
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
            if (param && param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "authentication success");

                if (!_dinput_paired) {
                    _dinput_paired = true;
                    app_save_host_mac(INPUT_MODE_DINPUT,
                                      &param->auth_cmpl.bda[0]);
                }
            } else {
                ESP_LOGI(TAG, "auth failed");
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled GAP event: %d", event);
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
                    ESP_LOGI(TAG, "Autoconnect: trying stored host...");
                    int64_t uptime = esp_timer_get_time() / 1000;
                    int delay = (uptime < 3000) ? 700 : 200;
                    vTaskDelay(pdMS_TO_TICKS(delay));

                    util_bluetooth_connect(
                        global_loaded_settings.paired_host_dinput_mac);

                } else {
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
                led_set_player_number(1);
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

        case ESP_HIDD_OUTPUT_EVENT:
            ESP_LOGI(TAG, "Ignoring HID output event");
            break;

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

    // ------------------------------------------------------
    // Validate or regenerate DInput device MAC
    // ------------------------------------------------------
    bool mac_valid = true;

    if (memcmp(global_loaded_settings.device_mac_dinput, zero, 6) == 0) {
        mac_valid = false;
    }

    if (memcmp(global_loaded_settings.device_mac_dinput, base_mac, 6) == 0) {
        mac_valid = false;
    }

    uint8_t expected_mac[6];
    memcpy(expected_mac, base_mac, 6);
    expected_mac[5] -= 2;

    if (memcmp(global_loaded_settings.device_mac_dinput, expected_mac, 6) != 0) {
        mac_valid = false;
    }

    if (!mac_valid) {
        ESP_LOGW(TAG, "Invalid DInput MAC detected — regenerating.");

        memcpy(tmpmac, base_mac, 6);
        tmpmac[5] -= 2;

        memcpy(global_loaded_settings.device_mac_dinput, tmpmac, 6);
        app_settings_save();

        ESP_LOGI(TAG,
            "Saved regenerated DInput MAC: %02X:%02X:%02X:%02X:%02X:%02X",
            tmpmac[0], tmpmac[1], tmpmac[2],
            tmpmac[3], tmpmac[4], tmpmac[5]);
    }
    else {
        memcpy(tmpmac, global_loaded_settings.device_mac_dinput, 6);
        ESP_LOGI(TAG,
            "Using stored DInput MAC: %02X:%02X:%02X:%02X:%02X:%02X",
            tmpmac[0], tmpmac[1], tmpmac[2],
            tmpmac[3], tmpmac[4], tmpmac[5]);
    }

    ESP_LOGI(TAG,
        "Final DInput BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        tmpmac[0], tmpmac[1], tmpmac[2],
        tmpmac[3], tmpmac[4], tmpmac[5]);

    // ------------------------------------------------------
    // Pairing status
    // ------------------------------------------------------
    if (memcmp(global_loaded_settings.paired_host_dinput_mac, zero, 6) != 0) {

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

                buf[0] = (_dinput_report.buttons & 0xFF);
                buf[1] = (_dinput_report.buttons >> 8);
                buf[2] = (_dinput_hat & 0x0F);
                buf[3] = _dinput_report.x;
                buf[4] = _dinput_report.y;
                buf[5] = _dinput_report.z;   // Rx
                buf[6] = _dinput_report.rz;  // Ry

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
void dinput_bt_sendinput(i2cinput_input_s *input)
{
    if (!input) return;

    debug_print_input_changes(input);

    // VB semantic buttons (derived from HOJA fields for DInput mode)
    bool vb_a      = !!input->button_east;
    bool vb_b      = !!input->button_south;
    bool vb_l      = !!input->trigger_l;
    bool vb_r      = !!input->trigger_r;
    bool vb_start  = !!input->button_plus;
    bool vb_select = !!input->button_minus;

    // VB left d-pad
    bool vb_left_up    = !!input->dpad_up;
    bool vb_left_down  = !!input->dpad_down;
    bool vb_left_left  = !!input->dpad_left;
    bool vb_left_right = !!input->dpad_right;

    // VB right d-pad
    bool vb_right_up    = !!input->button_north;
    bool vb_right_left  = !!input->button_west;
    bool vb_right_right = (input->rx == 65535);
    bool vb_right_down  = (input->ry == 0);

    // BlueRetro VB default mapping discovered empirically
	//
	// Primary slots
	// bit 0  = A
	// bit 3  = B
	// bit 6  = L
	// bit 7  = R
	// bit 10 = Select
	// bit 11 = Start
	//
	// Duplicate slots
	// bit 8  = L
	// bit 9  = R
	// bit 12 = Start
	// bit 15 = Select
	//
	// Unknown / unused
	// bit 1
	// bit 2
	// bit 4
	// bit 5
	// bit 13
	// bit 14
    enum {
		BR_SLOT_SELECT = 10, //confirmed!
        BR_SLOT_START  = 11, //confirmed!  //Bit 12 triggers same behavior
        BR_SLOT_A      = 0, //confirmed!
		BR_SLOT_B      = 3, //confirmed!
        BR_SLOT_L      = 6, //confirmed!
        BR_SLOT_R      = 7, //confirmed! //bit 9 triggers same behavior
    };

    uint16_t b = 0;
	b |= vb_select << BR_SLOT_SELECT;
	b |= vb_start  << BR_SLOT_START;
    b |= vb_a      << BR_SLOT_A;
	b |= vb_b      << BR_SLOT_B;
    b |= vb_l      << BR_SLOT_L;
    b |= vb_r      << BR_SLOT_R;

    _dinput_report.buttons = b;

    // Left VB D-pad -> Hat
    uint8_t hat = 0x08; // neutral

    if (vb_left_up && vb_left_right)          hat = 1;
    else if (vb_left_right && vb_left_down)   hat = 3;
    else if (vb_left_down && vb_left_left)    hat = 5;
    else if (vb_left_left && vb_left_up)      hat = 7;
    else if (vb_left_up)                      hat = 0;
    else if (vb_left_right)                   hat = 2;
    else if (vb_left_down)                    hat = 4;
    else if (vb_left_left)                    hat = 6;

    _dinput_hat = hat;

    // Keep X/Y centered
    _dinput_report.x = 0;
    _dinput_report.y = 0;

    // Right VB D-pad -> Rx/Ry
    int8_t rx = 0;
    int8_t ry = 0;

    if (vb_right_left) {
        rx = -127;
    }
    else if (vb_right_right) {
        rx = 127;
    }

    if (vb_right_up) {
        ry = -127;
    }
    else if (vb_right_down) {
        ry = 127;
    }

    _dinput_report.z  = (uint8_t)rx;  // Rx
    _dinput_report.rz = (uint8_t)ry;  // Ry
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
    int16_t centered = (int16_t)v - 2048;
    centered >>= 3;
    return (uint8_t)(centered + 128);
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