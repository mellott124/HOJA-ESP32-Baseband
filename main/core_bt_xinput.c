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
#include "XboxDescriptors.h"

// -----------------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------------
static const char *TAG = "core_bt_xinput";
static volatile bool _hid_connected = false;
static volatile bool _xinput_paired = false;

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
// Report map + app config
// -----------------------------------------------------------------------------
esp_hid_raw_report_map_t xinput_report_maps[1] = {
    {.data = XboxOneS_1708_HIDDescriptor, .len = sizeof(XboxOneS_1708_HIDDescriptor)}
};

// Bluetooth App setup data
util_bt_app_params_s xinput_app_params = {
    .hidd_cb    = NULL,
    .gap_cb     = NULL,
    .bt_mode    = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

// HID device config (matches Xbox 360 controller identity)
static esp_hid_raw_report_map_t xinput_report_map = {
    .data = XboxOneS_1708_HIDDescriptor,
    .len  = sizeof(XboxOneS_1708_HIDDescriptor)
};

esp_hid_device_config_t xinput_hidd_config = {
    .vendor_id          = XBOX_VENDOR_ID,          // 0x045E
    .product_id         = XBOX_1708_PRODUCT_ID,    // 0x02FD
    .version            = XBOX_1708_BCD_DEVICE_ID, // 0x0408
    .device_name        = "Xbox Wireless Controller",
    .manufacturer_name  = "Microsoft",
    .serial_number      = XBOX_1708_SERIAL,
    .report_maps        = &xinput_report_map,
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
		ESP_LOGI(TAG, "Output report (rumble)");
		if (param->output.length >= 8) {
			const uint8_t *data = param->output.data;
			uint8_t left_motor  = data[3];
			uint8_t right_motor = data[4];
			//haptics_rumble_translate(data);
			ESP_LOGI(TAG, "Rumble L=%d R=%d", left_motor, right_motor);
		}
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
	esp_bt_gap_set_device_name("Xbox Wireless Controller");

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
    uint8_t report[47] = {0};

    // --- Xbox One S Bluetooth Input Report (0x30) ---
    report[0] = 0x01;

    // ------------------------------------------------
    //  Bytes 1–2: Button bitmask (matches 0x1708 layout)
    // ------------------------------------------------
    uint16_t buttons = 0;

    // Face buttons
    if (input->button_south) buttons |= 0x1000;   // A
    if (input->button_east)  buttons |= 0x2000;   // B
    if (input->button_west)  buttons |= 0x4000;   // X
    if (input->button_north) buttons |= 0x8000;   // Y

    // Shoulder buttons
    if (input->trigger_l)    buttons |= 0x0100;   // LB
    if (input->trigger_r)    buttons |= 0x0200;   // RB

    // Menu buttons
    if (input->button_minus) buttons |= 0x0010;   // View / Back
    if (input->button_plus)  buttons |= 0x0020;   // Menu / Start

    // Stick buttons
    if (input->button_stick_left)  buttons |= 0x0040;
    if (input->button_stick_right) buttons |= 0x0080;

    // D-Pad bits
    if (input->dpad_up)    buttons |= 0x0001;
    if (input->dpad_down)  buttons |= 0x0002;
    if (input->dpad_left)  buttons |= 0x0004;
    if (input->dpad_right) buttons |= 0x0008;

    // Copy into packet
    report[1] = buttons & 0xFF;
    report[2] = (buttons >> 8) & 0xFF;

    // ------------------------------------------------
    //  Bytes 3–4: Triggers (LT / RT)
    // ------------------------------------------------
    report[3] = input->lt & 0xFF;  // Left trigger
    report[4] = input->rt & 0xFF;  // Right trigger

    // ------------------------------------------------
    //  Bytes 5–8: Left Stick (LX, LY)
    // ------------------------------------------------
    uint16_t lx = input->lx;
    uint16_t ly = input->ly;
    report[5] = lx & 0xFF;
    report[6] = lx >> 8;
    report[7] = ly & 0xFF;
    report[8] = ly >> 8;

    // ------------------------------------------------
    //  Bytes 9–12: Right Stick (RX, RY)
    // ------------------------------------------------
    uint16_t rx = input->rx;
    uint16_t ry = input->ry;
    report[9]  = rx & 0xFF;
    report[10] = rx >> 8;
    report[11] = ry & 0xFF;
    report[12] = ry >> 8;

    // ------------------------------------------------
    //  Remaining bytes (13–46): padding / status
    // ------------------------------------------------
    for (int i = 13; i < 47; i++)
        report[i] = 0x00;

    // ------------------------------------------------
    //  Transmit report only if HID link is active
    // ------------------------------------------------
    if (_hid_connected) {
        esp_bt_hid_device_send_report(
            ESP_HIDD_REPORT_TYPE_INPUT,  // Report type
            0,                           // Report ID (handled in data)
            sizeof(report),              // Length
            report                       // Data
        );
    }
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
