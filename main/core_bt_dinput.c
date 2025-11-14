#include "core_bt_dinput.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LED.h"

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
    .device_name = "RetroOnyx DInput",
    .manufacturer_name = "RetroOnyx",
    .serial_number = "000001",
    .report_maps = dinput_report_maps,
    .report_maps_len = 1
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
                    int64_t uptime = esp_timer_get_time() / 1000;
                    int delay = (uptime < 3000) ? 700 : 200;
                    vTaskDelay(pdMS_TO_TICKS(delay));
                    util_bluetooth_connect(
                        global_loaded_settings.paired_host_dinput_mac);
                } else {
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
                led_set_state(LED_CONNECTED);
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
                ESP_BT_GENERAL_DISCOVERABLE);
            break;

        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(TAG, "HID STOP");
            _hid_connected = false;
            led_set_state(LED_IDLE);
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
    int err = 1;

    uint8_t tmpmac[6];
    uint8_t *mac = global_live_data.current_mac;

    if ((mac[0] == 0) && (mac[1] == 0)) {
        mac = global_loaded_settings.device_mac_dinput;
    }

    memcpy(tmpmac, mac, 6);
    tmpmac[5] -= 2;

    // ---------------------------------------------------------
    // DINPUT MUST ALWAYS START FRESH FOR WINDOWS
    // Disable reconnect logic. Ignore stored MAC.
    // ---------------------------------------------------------
    _dinput_paired = false;

    // Always enter discoverable/connectable mode immediately
    esp_bt_gap_set_scan_mode(
        ESP_BT_CONNECTABLE,
        ESP_BT_GENERAL_DISCOVERABLE
    );

    // ---------------------------------------------------------
    // Initialize BT controller + stack
    // (Classic HID security is handled automatically in IDF 5.x)
    // ---------------------------------------------------------
    err = util_bluetooth_init(tmpmac);

    // ---------------------------------------------------------
    // REGISTER HID DEVICE
    // ---------------------------------------------------------
    err = util_bluetooth_register_app(&dinput_app_params, &dinput_hidd_config);

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

// -------------------------------
// Input Mapping
// -------------------------------
void dinput_bt_sendinput(i2cinput_input_s *input)
{
    if (!input) return;

    // Normalize 0–4095 into 0–255
    _dinput_report.x  = (input->lx >> 4);
    _dinput_report.y  = (input->ly >> 4);
    _dinput_report.z  = (input->rx >> 4);
    _dinput_report.rz = (input->ry >> 4);

    uint16_t b = 0;

    // Basic 16-button map
    // You can customize easily
    b |= (input->button_south) << 0;
    b |= (input->button_east)  << 1;
    b |= (input->button_west)  << 2;
    b |= (input->button_north) << 3;

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

    b |= (input->button_minus) << 14;
    b |= (input->button_plus)  << 15;

    _dinput_report.buttons = b;
}

void dinput_bt_end_task()
{
    if (_dinput_bt_task_handle) {
        xTaskNotifyGive(_dinput_bt_task_handle);
        _dinput_bt_task_handle = NULL;
    }
}
