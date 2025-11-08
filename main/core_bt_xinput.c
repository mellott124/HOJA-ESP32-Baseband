// -----------------------------------------------------------------------------
// core_bt_xinput.c — Classic Bluetooth HID implementation for Xbox 360 mode
// Merges RetroOnyx Switch core structure + OpenController XInput proven code
// -----------------------------------------------------------------------------

#include "core_bt_xinput.h"
#include "hoja_includes.h"
#include "LED.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// -----------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------
static const char *TAG = "core_bt_xinput";
static volatile bool _hid_connected = false;
static volatile bool _xinput_paired = false;

#define HID_VEND_XINPUT  0x045E   // Microsoft
#define HID_PROD_XINPUT  0x028E   // Xbox 360 Controller
#define DEFAULT_US_DELAY (8*1000)
#define DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)

interval_s _xinput_interval = {0};
TaskHandle_t _xinput_bt_task_handle = NULL;

// -----------------------------------------------------------------------------
// Background discoverability task (C version, replaces lambdas)
// -----------------------------------------------------------------------------
static void xinput_pairing_task(void *arg)
{
    while (!_hid_connected)
    {
        ESP_LOGI(TAG, "Still discoverable — waiting for host connection...");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Avoid watchdog reset
    }

    ESP_LOGI(TAG, "Host connected — disabling discoverability.");
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    vTaskDelete(NULL);
}

// -----------------------------------------------------------------------------
// Working OpenController HID Descriptor for Xbox 360 gamepad (Classic BT)
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Xbox 360 Classic HID Descriptor (52-byte reports)
// -----------------------------------------------------------------------------
static const uint8_t xinput_hid_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x10,        //     Usage Maximum (Button 16)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x10,        //     Report Count (16)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x33,        //     Usage (Rx)
    0x09, 0x34,        //     Usage (Ry)
    0x16, 0x00, 0x80,  //     Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x04,        //     Report Count (4)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x05, 0x02,        //     Usage Page (Simulation Controls)
    0x09, 0xC4,        //     Usage (Accelerator/Left Trigger)
    0x09, 0xC5,        //     Usage (Brake/Right Trigger)
    0x15, 0x00,        //     Logical Minimum (0)
    0x26, 0xFF, 0x00,  //     Logical Maximum (255)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

#define XINPUT_HID_REPORT_MAP_LEN sizeof(xinput_hid_descriptor)

// -----------------------------------------------------------------------------
// Report map + app config
// -----------------------------------------------------------------------------
esp_hid_raw_report_map_t xinput_report_maps[1] = {
    {.data = xinput_hid_descriptor, .len = (uint16_t)XINPUT_HID_REPORT_MAP_LEN}
};

// Bluetooth App setup data
util_bt_app_params_s xinput_app_params = {
    .hidd_cb    = NULL,
    .gap_cb     = NULL,
    .bt_mode    = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

// HID device config (matches Xbox 360 controller identity)
esp_hid_device_config_t xinput_hidd_config = {
    .vendor_id          = HID_VEND_XINPUT,
    .product_id         = HID_PROD_XINPUT,
    .version            = 0x0100,
    .device_name        = "Virtual Boy Controller",
    .manufacturer_name  = "RetroOnyx",
    .serial_number      = "000000",
    .report_maps        = xinput_report_maps,
    .report_maps_len    = 1,
};

// -----------------------------------------------------------------------------
// GAP Callback (Classic BT)
// -----------------------------------------------------------------------------
static void xinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
            ESP_LOGI(TAG, "ACL connect complete. Hiding discoverability.");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

            if (_xinput_bt_task_handle == NULL)
            {
                xTaskCreatePinnedToCore(_xinput_bt_task_standard,
                                        "XInput_Send_Task", 4048,
                                        NULL, 0, &_xinput_bt_task_handle, 0);
            }
            break;

        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
            ESP_LOGI(TAG, "ACL disconnect complete.");
            if (_xinput_bt_task_handle != NULL)
            {
                xTaskNotifyGive(_xinput_bt_task_handle);
                _xinput_bt_task_handle = NULL;
            }
            app_set_connected_status(0);
            app_set_power_setting(POWER_CODE_OFF);
            break;

        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
                if (!_xinput_paired)
                {
                    _xinput_paired = true;
                    app_save_host_mac(INPUT_MODE_XINPUT, &param->auth_cmpl.bda[0]);
                }
            }
            else
            {
                ESP_LOGW(TAG, "Authentication failed (status=%d)", param->auth_cmpl.stat);
            }
            break;

        default:
            ESP_LOGI(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

// -----------------------------------------------------------------------------
// HID Callback (Classic BT) — XInput Mode
// -----------------------------------------------------------------------------
static void xinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event)
    {
        case ESP_HIDD_START_EVENT:
            if (param->start.status == ESP_OK)
            {
                ESP_LOGI(TAG, "XInput HID start OK");
                led_set_state(LED_PAIRING);

                if (_xinput_paired || global_loaded_settings.has_paired_xinput)
                {
                    ESP_LOGI(TAG, "Known host found. Attempting reconnect...");
                    util_bluetooth_connect(global_loaded_settings.paired_host_xinput_mac);
                    vTaskDelay(pdMS_TO_TICKS(3000));

                    if (!_hid_connected)
                    {
                        ESP_LOGW(TAG, "Reconnect failed — entering continuous discoverable mode...");
                        xTaskCreatePinnedToCore(xinput_pairing_task, "xinput_pair_loop",
                                                4096, NULL, 1, NULL, tskNO_AFFINITY);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Reconnect successful.");
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "No paired host — entering continuous discoverable mode...");
                    xTaskCreatePinnedToCore(xinput_pairing_task, "xinput_pair_loop",
                                            4096, NULL, 1, NULL, tskNO_AFFINITY);
                }
            }
            else
            {
                ESP_LOGE(TAG, "XInput HID start failed");
                led_set_state(LED_ERROR);
            }
            break;

        case ESP_HIDD_CONNECT_EVENT:
			if (param->connect.status == ESP_OK)
			{
				_hid_connected = true;
				ESP_LOGI(TAG, "XInput HID connected");
				led_set_state(LED_CONNECTED);
				esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
			}
			else
			{
				ESP_LOGE(TAG, "XInput HID connect failed");
				led_set_state(LED_ERROR);
			}
			break;


        case ESP_HIDD_DISCONNECT_EVENT:
            _hid_connected = false;
            ESP_LOGI(TAG, "XInput HID disconnected");
            led_set_state(LED_PAIRING);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;

        case ESP_HIDD_OUTPUT_EVENT:
            ESP_LOGI(TAG, "Output report received (rumble/LEDs)");
            break;

        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(TAG, "HID stop");
            _hid_connected = false;
            led_set_state(LED_IDLE);
            break;

        default:
            ESP_LOGI(TAG, "Unhandled HID event: %d", event);
            break;
    }
}

// -----------------------------------------------------------------------------
// Initialize and register the XInput Classic HID app
// -----------------------------------------------------------------------------
esp_err_t core_bt_xinput_start(void)
{
    ESP_LOGI(TAG, "Starting XInput Classic BT HID mode...");

    uint8_t tmpmac[6];
    memcpy(tmpmac, global_loaded_settings.device_mac_xinput, 6);

    ESP_LOGI(TAG, "Applying base BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             tmpmac[0], tmpmac[1], tmpmac[2],
             tmpmac[3], tmpmac[4], tmpmac[5]);

    const uint8_t zero_mac[6] = {0};

	if (global_loaded_settings.has_paired_xinput &&
		memcmp(global_loaded_settings.paired_host_xinput_mac, zero_mac, 6) != 0)
	{
		ESP_LOGI(TAG, "Stored paired XInput host found");
		_xinput_paired = true;
	}
	else
	{
		ESP_LOGI(TAG, "No stored XInput host — starting in pairing mode");
		_xinput_paired = false;
		// Do NOT clear has_paired_xinput or paired_host_xinput_mac here
	}

	esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
	esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

	// --- Enforce persistent identity for Windows re-pairing ---
	esp_bt_gap_set_device_name("Virtual Boy Controller");

	esp_bt_cod_t cod = {
		.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL,
		.minor = 0x08,  // Joystick subclass
		.service = ESP_BT_COD_SRVC_RENDERING
	};
	esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
	esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));

	// --- Proceed with standard Bluetooth init ---
	if (util_bluetooth_init(tmpmac) != 1)
    {
        ESP_LOGE(TAG, "Bluetooth init failed");
        return ESP_FAIL;
    }

    xinput_app_params.hidd_cb = xinput_bt_hidd_cb;
    xinput_app_params.gap_cb  = xinput_bt_gap_cb;

    if (util_bluetooth_register_app(&xinput_app_params, &xinput_hidd_config) != 1)
    {
        ESP_LOGE(TAG, "App registration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "XInput Classic HID mode started successfully");
    led_set_state(LED_PAIRING);
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Stop Classic HID app
// -----------------------------------------------------------------------------
void core_bt_xinput_stop(void)
{
    ESP_LOGI(TAG, "Stopping XInput Classic HID mode");
    util_bluetooth_deinit();
    _hid_connected = false;
}

static inline uint8_t to_u8(int16_t v)
{
    // Convert signed 16-bit stick value (-32768..32767) to unsigned 0..255
    return (uint8_t)(((int32_t)v + 32768) >> 8);
}

void xinput_bt_sendinput(i2cinput_input_s *input)
{
    if (!_hid_connected) return;

    uint8_t report[12] = {0};
    uint16_t buttons = 0;

    // --- Buttons (16 bits) ---
    buttons |= (input->button_south ? (1 << 0) : 0);  // A
    buttons |= (input->button_east  ? (1 << 1) : 0);  // B
    buttons |= (input->button_west  ? (1 << 2) : 0);  // X
    buttons |= (input->button_north ? (1 << 3) : 0);  // Y
    buttons |= (input->button_minus ? (1 << 4) : 0);  // Select
    buttons |= (input->button_plus  ? (1 << 5) : 0);  // Start
    buttons |= (input->button_home  ? (1 << 6) : 0);  // Home
    buttons |= (input->button_capture ? (1 << 7) : 0); // Capture
    buttons |= (input->dpad_up    ? (1 << 8)  : 0);
    buttons |= (input->dpad_down  ? (1 << 9)  : 0);
    buttons |= (input->dpad_left  ? (1 << 10) : 0);
    buttons |= (input->dpad_right ? (1 << 11) : 0);

    report[0] = buttons & 0xFF;
    report[1] = (buttons >> 8) & 0xFF;

    // --- Sticks (16-bit) ---
    int16_t lx = input->lx;
    int16_t ly = input->ly;
    int16_t rx = input->rx;
    int16_t ry = input->ry;

    memcpy(&report[2],  &lx, 2);
    memcpy(&report[4],  &ly, 2);
    memcpy(&report[6],  &rx, 2);
    memcpy(&report[8],  &ry, 2);

    // --- Triggers (8-bit) ---
    report[10] = input->trigger_l ? 255 : 0;
    report[11] = input->trigger_r ? 255 : 0;

    // --- Debug output ---
    ESP_LOG_BUFFER_HEX_LEVEL("XInput OUT", report, sizeof(report), ESP_LOG_INFO);

    esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_INTRDATA,
        0x00,  // Report ID
        sizeof(report),
        report
    );
}




// -----------------------------------------------------------------------------
// Background send loop task
// -----------------------------------------------------------------------------
static void _xinput_bt_task_standard(void *parameters)
{
    ESP_LOGI(TAG, "Starting XInput input loop task");
    app_set_report_timer(DEFAULT_US_DELAY);

    uint32_t last_log = 0;

    for (;;)
    {
        // Allow other tasks to delete this one cleanly
        if (ulTaskNotifyTake(pdTRUE, 0))
        {
            ESP_LOGI(TAG, "Delete requested — exiting task");
            vTaskDelete(NULL);
        }

        if (_hid_connected)
        {
            // Optional heartbeat every ~5 seconds
            if (esp_log_timestamp() - last_log > 5000)
            {
                ESP_LOGD(TAG, "XInput HID active");
                last_log = esp_log_timestamp();
            }

            // Let input updates be handled by xinput_bt_sendinput() elsewhere
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        else
        {
            // Controller not yet connected — periodically remind user
            if (esp_log_timestamp() - last_log > 5000)
            {
                ESP_LOGI(TAG, "Still discoverable — waiting for host connection...");
                last_log = esp_log_timestamp();
            }

            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}


bool xinput_is_connected(void) { return _hid_connected; }
