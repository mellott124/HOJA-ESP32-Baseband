#include "core_bt_switch.h"
#include "esp_log.h"
#include "driver/gpio.h"   // Needed for gpio_set_level(), GPIO_NUM_xx
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LED.h"           // Needed for led_all_off(), led_set_state(), LED_IDLE, etc.
#include "bt_serial_logger.h"


#include "hoja_includes.h"   // already used elsewhere
extern input_mode_t get_current_mode(void);  // NEW: access selected controller type

#define HID_PROD_NSPRO  0x2009
#define HID_VEND_NSPRO  0x057E
#define PROCON_HID_REPORT_MAP_LEN 170

const uint8_t procon_hid_descriptor[PROCON_HID_REPORT_MAP_LEN] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x06, 0x01, 0xFF,  //   Usage Page (Vendor Defined 0xFF01)

    0x85, 0x21,  //   Report ID (33)
    0x09, 0x21,  //   Usage (0x21)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x30,  //   Report ID (48)
    0x09, 0x30,  //   Usage (0x30)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x31,        //   Report ID (49)
    0x09, 0x31,        //   Usage (0x31)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x69, 0x01,  //   Report Count (361)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x32,        //   Report ID (50)
    0x09, 0x32,        //   Usage (0x32)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x69, 0x01,  //   Report Count (361)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x33,        //   Report ID (51)
    0x09, 0x33,        //   Usage (0x33)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x69, 0x01,  //   Report Count (361)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x3F,  //   Report ID (63)
    0x05, 0x09,  //   Usage Page (Button)
    0x19, 0x01,  //   Usage Minimum (0x01)
    0x29, 0x10,  //   Usage Maximum (0x10)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x10,  //   Report Count (16)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)
    0x05, 0x01,  //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,  //   Usage (Hat switch)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x07,  //   Logical Maximum (7)
    0x75, 0x04,  //   Report Size (4)
    0x95, 0x01,  //   Report Count (1)
    0x81, 0x42,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null
                 //   State)
    0x05, 0x09,  //   Usage Page (Button)
    0x75, 0x04,  //   Report Size (4)
    0x95, 0x01,  //   Report Count (1)
    0x81, 0x01,  //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position)
    0x05, 0x01,  //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,  //   Usage (X)
    0x09, 0x31,  //   Usage (Y)
    0x09, 0x33,  //   Usage (Rx)
    0x09, 0x34,  //   Usage (Ry)
    0x16, 0x00, 0x00,              //   Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65534)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x04,                    //   Report Count (4)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)
                 
    0x06, 0x01, 0xFF,  //   Usage Page (Vendor Defined 0xFF01)

    0x85, 0x01,  //   Report ID (1)
    0x09, 0x01,  //   Usage (0x01)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)

    0x85, 0x10,  //   Report ID (16)
    0x09, 0x10,  //   Usage (0x10)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x09,  //   Report Count (9)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)

    0x85, 0x11,  //   Report ID (17)
    0x09, 0x11,  //   Usage (0x11)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)

    0x85, 0x12,  //   Report ID (18)
    0x09, 0x12,  //   Usage (0x12)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)
    0xC0,        // End Collection
};

#define DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define DEFAULT_US_DELAY (8*1000)
static volatile bool        _hid_connected = false;

static volatile bool        _switch_paired = false;
uint8_t _switch_imu_mode = 0x00;

interval_s _ns_interval = {0};

sw_input_s _switch_input_data = {.ls_x = 2047, .ls_y = 2047, .rs_x = 2047, .rs_y = 2047};

void ns_set_imu_mode(uint8_t mode)
{
    _switch_imu_mode = mode;
}

void ns_report_setinputreport_full(uint8_t *buffer)
{
    // Nintendo Switch expects 48 bytes total
    // 0x30 = Input report ID
    //buffer[1] = 0x30; //This is handled by later function calls

    // Byte[0]: timer (increments every frame)
    static uint8_t timer = 0;
    buffer[0] = timer++;
    
    // Byte[1]: battery + connection
    // 0x8E = full battery, connected via Bluetooth
    buffer[1] = 0x8E;

    // Bytes[2–4]: button data
    // bit order:
    //   byte2: A/B/X/Y/R/ZR/L/ZL
    //   byte3: Minus/Plus/RS/LS/Home/Capture
    //   byte4: D-pad + bits
    buffer[2] = _switch_input_data.right_buttons;
	buffer[3] = _switch_input_data.shared_buttons;
	buffer[4] = _switch_input_data.left_buttons;

    // --- Left stick (12-bit packed: X/Y) ---
    uint16_t lx = _switch_input_data.ls_x & 0x0FFF;
    uint16_t ly = _switch_input_data.ls_y & 0x0FFF;

    // Left stick: bytes 6–8
    buffer[5]  = lx & 0xFF;
    buffer[6]  = ((ly & 0x0F) << 4) | ((lx >> 8) & 0x0F);
    buffer[7]  = (ly >> 4) & 0xFF;

	// --- Right stick (12-bit packed: X/Y) ---
	uint16_t rx = _switch_input_data.rs_x & 0x0FFF;
    uint16_t ry = _switch_input_data.rs_y & 0x0FFF;
	
    // Right stick: bytes 9–11
    buffer[8]  = rx & 0xFF;
    buffer[9] = ((ry & 0x0F) << 4) | ((rx >> 8) & 0x0F);
    buffer[10] = (ry >> 4) & 0xFF;

    // Fill remaining bytes with zeros (rumble, subcmd replies, etc.)
    for (int i = 11; i < 48; i++) {
        buffer[i] = 0x00;
    }
}



void _ns_reset_report_spacer()
{
    uint64_t timestamp = get_timestamp_us();
    uint64_t delay_time = app_get_report_timer();
    interval_resettable_run(timestamp, delay_time, true, &_ns_interval);
}

bool _ns_send_check_nonblocking()
{
    uint64_t timestamp = get_timestamp_us();
    uint64_t delay_time = app_get_report_timer();
    return interval_run(timestamp, delay_time, &_ns_interval);
}

/**
 * @brief NS Core Report mode enums
 */
typedef enum
{
    NS_REPORT_MODE_BLANK,
    NS_REPORT_MODE_SIMPLE,
    NS_REPORT_MODE_FULL,
    NS_REPORT_MODE_MAX,
} ns_report_mode_t;

/**
 * @brief NS Core power handle state types
 */
typedef enum
{
    NS_POWER_AWAKE,
    NS_POWER_SLEEP,
} ns_power_handle_t;

/**
 * @brief NS Core Status
 */
typedef enum
{
    NS_STATUS_IDLE,
    NS_STATUS_SUBCORESET,
    NS_STATUS_RUNNING,
} ns_core_status_t;

TaskHandle_t _switch_bt_task_handle = NULL;
ns_power_handle_t _switch_power_state = NS_POWER_AWAKE;


void _switch_bt_task_standard(void *parameters);
void _switch_bt_task_empty(void *parameters);
void _switch_bt_task_short(void *parameters);


void switch_bt_end_task()
{
    if (_switch_bt_task_handle != NULL) {
        // Defer deletion to the task context (safe FreeRTOS method)
        ESP_LOGI("switch_bt_end_task", "Deferring delete of _switch_bt_task_standard");
        xTaskNotifyGive(_switch_bt_task_handle);
        _switch_bt_task_handle = NULL;
    }
}


// Unused
void btsnd_hcic_sniff_mode_cb(bool sniff, uint16_t tx_lat, uint16_t rx_lat)
{
    // Ignore all of this for debug
    return;
    if(sniff)
    {
        //_sniff = true;
        //_delay_time_us = rx_lat*1000;//_ns_interval_to_us(rx_lat);
        printf("Delay (ms): %d\n", rx_lat);
    }
    else
    {
        //_sniff = false;
        //_delay_time_us = 8000;
        printf("UnSniff: \n");
    }
    _ns_reset_report_spacer();
}



/* HCI mode defenitions */
#define HCI_MODE_ACTIVE                 0x00
#define HCI_MODE_HOLD                   0x01
#define HCI_MODE_SNIFF                  0x02
#define HCI_MODE_PARK                   0x03

// SWITCH BTC GAP Event Callback
void switch_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "switch_bt_gap_cb";
    switch (event)
    {
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_DISC_RES_EVT");
        // esp_log_buffer_hex(TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        ESP_LOGI(TAG, "%d", param->rmt_srvcs.num_uuids);
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        ESP_LOGI(TAG, "ACL Connect Complete.");

        // Start input loop task
        if(_switch_bt_task_handle==NULL)
        {
            xTaskCreatePinnedToCore(_switch_bt_task_standard,
                                "Standard Send Task", 4048,
                                NULL, 0, &_switch_bt_task_handle, 0);
        }
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
		ESP_LOGI(TAG, "ACL Disconnect Complete.");

		// 🧩 Defer task deletion safely to avoid crash
		if (_switch_bt_task_handle != NULL) {
			ESP_LOGI(TAG, "Deferring delete of _switch_bt_task_standard");
			xTaskNotifyGive(_switch_bt_task_handle);
			_switch_bt_task_handle = NULL;
		}

		app_set_connected_status(0);
		app_set_power_setting(POWER_CODE_OFF); // your existing shutdown call
		break;

    
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            //esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            
            if(!_switch_paired)
            {
                _switch_paired = true;
                app_save_host_mac(INPUT_MODE_SWPRO, &param->auth_cmpl.bda[0]);
            }
        }
        else
        {
            ESP_LOGI(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }

    case ESP_BT_GAP_ENC_CHG_EVT:
    {

        break;
    }

    case ESP_BT_GAP_MODE_CHG_EVT:
    {
        // Depreciated, not needed I guess
        break;
    }

    default:
        ESP_LOGI(TAG, "UNKNOWN GAP EVT: %d", event);
        break;
    }
}

// --------------------------------------------------------------------------
// HID Callback: handles connection, disconnection, and state transitions
// --------------------------------------------------------------------------
void switch_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "ns_bt_hidd_cb";

    switch (event)
    {
        // --------------------------------------------------------------
        // HID Start: controller begins advertising or reconnecting
        // --------------------------------------------------------------
        case ESP_HIDD_START_EVENT:
        {
            if (param->start.status == ESP_OK)
            {
                ESP_LOGI(TAG, "START OK");
                led_set_state(LED_PAIRING);  // 🟠 Amber while advertising/reconnecting

                if (_switch_paired) {
					// Adaptive reconnect timing
					int64_t uptime_ms = esp_timer_get_time() / 1000;
					int reconnect_delay = (uptime_ms < 3000) ? 700 : 200;  // longer on cold boot, short after reset

					ESP_LOGI(TAG,
							 "Known host found — waiting %d ms before reconnect...",
							 reconnect_delay);

					// Visual feedback while waiting
					led_set_state(LED_PAIRING);  // amber blink to show it's reconnecting

					vTaskDelay(pdMS_TO_TICKS(reconnect_delay));

					// Attempt quick reconnect
					util_bluetooth_connect(global_loaded_settings.paired_host_switch_mac);
				} else {
					ESP_LOGI(TAG, "No paired host found — entering discoverable mode.");
					esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
				}



            }
            else
            {
                ESP_LOGE(TAG, "START failed!");
                led_set_state(LED_ERROR);    // 🔴 Red on failure
            }
            break;
        }

        // --------------------------------------------------------------
        // HID Connect: connection established successfully
        // --------------------------------------------------------------
        case ESP_HIDD_CONNECT_EVENT:
        {
            if (param->connect.status == ESP_OK)
            {
                _hid_connected = true;
                ESP_LOGI(TAG, "CONNECT OK");
                led_set_state(LED_CONNECTED); // 🟢 Green = connected
            }
            else
            {
                ESP_LOGE(TAG, "CONNECT failed!");
                led_set_state(LED_ERROR);     // 🔴 Red = connection failure
            }
            break;
        }

        // --------------------------------------------------------------
        // HID Disconnect: controller disconnected (graceful or lost link)
        // --------------------------------------------------------------
        case ESP_HIDD_DISCONNECT_EVENT:
        {
            _hid_connected = false;
            _ns_reset_report_spacer();

            if (param->disconnect.status == ESP_OK)
            {
                ESP_LOGI(TAG, "DISCONNECT OK");

                // Return to advertising mode for pairing
                led_set_state(LED_PAIRING);   // 🟠 Amber while re-advertising
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
            else
            {
                ESP_LOGE(TAG, "DISCONNECT failed!");
                led_set_state(LED_ERROR);     // 🔴 Red on disconnect error
            }
            break;
        }

        // --------------------------------------------------------------
        // Protocol mode: report/boot (not visually significant)
        // --------------------------------------------------------------
        case ESP_HIDD_PROTOCOL_MODE_EVENT:
        {
            ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s",
                     param->protocol_mode.map_index,
                     param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
            break;
        }

        // --------------------------------------------------------------
        // Output report (rumble, LEDs, etc.)
        // --------------------------------------------------------------
        case ESP_HIDD_OUTPUT_EVENT:
        {
            ns_report_handler(param->output.report_id,
                              param->output.data,
                              param->output.length);
            break;
        }

        // --------------------------------------------------------------
        // Feature report (debug/log only)
        // --------------------------------------------------------------
        case ESP_HIDD_FEATURE_EVENT:
        {
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
            break;
        }

        // --------------------------------------------------------------
        // HID Stop: Bluetooth subsystem halted
        // --------------------------------------------------------------
        case ESP_HIDD_STOP_EVENT:
        {
            ESP_LOGI(TAG, "HID STOP");
            _hid_connected = false;
            led_set_state(LED_IDLE);          // 🔵 Blue = idle/sleeping
            break;
        }

        // --------------------------------------------------------------
        // Default: no-op
        // --------------------------------------------------------------
        default:
            break;
    }
}



// Switch HID report maps
esp_hid_raw_report_map_t switch_report_maps[1] = {
    {
        .data = procon_hid_descriptor,
        .len = (uint16_t)PROCON_HID_REPORT_MAP_LEN,
    }};

// Bluetooth App setup data
util_bt_app_params_s switch_app_params = {
    .hidd_cb = switch_bt_hidd_cb,
    .gap_cb = switch_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t switch_hidd_config = {
    .vendor_id = HID_VEND_NSPRO,
    .product_id = HID_PROD_NSPRO,
    .version = 0x0100,
    .device_name = "Pro Controller",
    .manufacturer_name = "Nintendo",
    .serial_number = "000000",
    .report_maps = switch_report_maps,
    .report_maps_len = 1,
};

// Attempt start of Nintendo Switch controller core
int core_bt_switch_start(void)
{
    const char *TAG = "core_bt_switch_start";
    esp_err_t ret;
    int err;

    // --------------------------------------------------
    // 🧹 TEMP: Force fresh pairing mode (clears stored host)
    // --------------------------------------------------
    ESP_LOGW("PAIRING", "⚠️  Forcing controller to forget stored host and re-enter pairing mode");
    memset(global_loaded_settings.paired_host_switch_mac, 0, 6);
    _switch_paired = false;
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Convert calibration data
    switch_analog_calibration_init();

    uint8_t tmpmac[6] = {0};
    uint8_t *mac = global_live_data.current_mac;

    if ((mac[0] == 0) && (mac[1] == 0)) {
        mac = global_loaded_settings.device_mac_switch;
    }

    memcpy(tmpmac, mac, 6);
    // On ESP32, the Bluetooth address is the base MAC with the last octet -= 2
    tmpmac[5] -= 2;

    // --------------------------------------------------
    // 🧩 Force paired flag if a stored host MAC exists
    // --------------------------------------------------
    const uint8_t zero_mac[6] = {0};
    if (memcmp(global_loaded_settings.paired_host_switch_mac, zero_mac, 6) != 0) {
        _switch_paired = true;
        ESP_LOGI(TAG,
                 "Detected stored paired host %02X:%02X:%02X:%02X:%02X:%02X — enabling quick reconnect",
                 global_loaded_settings.paired_host_switch_mac[0],
                 global_loaded_settings.paired_host_switch_mac[1],
                 global_loaded_settings.paired_host_switch_mac[2],
                 global_loaded_settings.paired_host_switch_mac[3],
                 global_loaded_settings.paired_host_switch_mac[4],
                 global_loaded_settings.paired_host_switch_mac[5]);
        ESP_LOGI(TAG, "Quick reconnect path enabled");
    } else {
        _switch_paired = false;
        ESP_LOGI(TAG, "No stored paired host found — entering pairing mode.");

        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    }

    // --------------------------------------------------
    // Initialize Bluetooth with valid MAC
    // --------------------------------------------------
    err = util_bluetooth_init(tmpmac);

    // --------------------------------------------------
    // Register HID application
    // --------------------------------------------------
    err = util_bluetooth_register_app(&switch_app_params, &switch_hidd_config);
	
	// --------------------------------------------------
	// Start Bluetooth serial logger AFTER HID app is up
	// --------------------------------------------------
	vTaskDelay(pdMS_TO_TICKS(1000));  // let HID settle
	ESP_LOGI("core_bt_switch_start", "Starting Bluetooth serial logger...");
	bt_serial_logger_init();

	// --------------------------------------------------
	// Redirect ESP_LOG output to SPP
	// --------------------------------------------------
	bt_serial_logger_redirect_esp_log();
	//ESP_LOGI(TAG, "ESP_LOG redirected to SPP output");

	// --------------------------------------------------
	// Reactivate discoverability for HID + SPP visibility
	// --------------------------------------------------
	esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    return 1;
}


// Stop Nintendo Switch controller core
void core_bt_switch_stop(void)
{
    //const char *TAG = "core_ns_stop";
    util_bluetooth_deinit();
}

/* void _switch_bt_task_standard(void *parameters)
{
    ESP_LOGI("_switch_bt_task_standard", "Starting input loop task...");

    static ns_report_mode_t _report_mode = NS_REPORT_MODE_FULL;

    //_report_mode = NS_REPORT_MODE_BLANK;
    _hid_connected = false;
    app_set_report_timer(DEFAULT_US_DELAY); 

    for (;;)
    {
        static uint8_t _full_buffer[64] = {0};
        uint8_t tmp[64] = {0x00, 0x00};

        if(_hid_connected)
        {
            if(_ns_send_check_nonblocking())
            {
                if((_report_mode == NS_REPORT_MODE_FULL))
                {
                    ns_report_clear(_full_buffer, 64);
                    ns_report_setinputreport_full(_full_buffer);
                    ns_report_settimer(_full_buffer);
                    ns_report_setbattconn(_full_buffer);
                    if(_hid_connected){
                        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30, SWITCH_BT_REPORT_SIZE, _full_buffer);
					}
                }
                else
                {
                    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x00, 1, tmp);
                }
            }
        }
        else
        {
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
} */


void _switch_bt_task_standard(void *parameters)
{
    ESP_LOGI("_switch_bt_task_standard", "Starting input loop task...");

    static ns_report_mode_t _report_mode = NS_REPORT_MODE_FULL;
    _hid_connected = false;

    app_set_report_timer(DEFAULT_US_DELAY);

    for (;;)
	{
		// Handle deferred shutdown requests from GAP callback
		if (ulTaskNotifyTake(pdTRUE, 0)) {
			ESP_LOGI("_switch_bt_task_standard", "Delete requested, exiting task cleanly");
			vTaskDelete(NULL); // self-delete safely
		}

		static uint8_t _full_buffer[64] = {0};

        uint8_t tmp[64] = {0x00, 0x00};

        // Only send when connected
        if (_hid_connected)
        {
            if (_ns_send_check_nonblocking())
            {
                if (_report_mode == NS_REPORT_MODE_FULL)
                {
                    ns_report_clear(_full_buffer, 64);
                    ns_report_setinputreport_full(_full_buffer);
					//ns_report_settimer(_full_buffer);
                    //ns_report_setbattconn(_full_buffer);

                    // Debug: log first few button bytes to confirm dynamic input
#if 1
                    static uint8_t last_btn[3] = {0};
                    bool changed = false;
                    for (int i = 0; i < 3; i++) {
                        if (_full_buffer[2 + i] != last_btn[i]) {
                            changed = true;
                            break;
                        }
                    }
                    if (changed) {
                        ESP_LOGI("MAP/out", "Buttons [2..4]: %02X %02X %02X",
                                 _full_buffer[2], _full_buffer[3], _full_buffer[4]);
                        memcpy(last_btn, &_full_buffer[2], 3);
                    }
#endif
                    if (_hid_connected) {
						//ESP_LOG_BUFFER_HEX("HID_TX_FINAL", _full_buffer, 16);
                        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30, SWITCH_BT_REPORT_SIZE, _full_buffer);
                    }
                }
                else
                {
                    // Minimal report (used only in handshake mode)
                    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x00, 1, tmp);
                }
            }
        }
        else
        {
            // If not connected, sleep lightly to prevent watchdog reset
            vTaskDelay(16 / portTICK_PERIOD_MS);
        }
    }
}


void switch_bt_sendinput(i2cinput_input_s *input)
{
    _switch_input_data.ls_x = input->lx;
    _switch_input_data.ls_y = input->ly;

    _switch_input_data.rs_x = input->rx;
    _switch_input_data.rs_y = input->ry;

    _switch_input_data.b_a = input->button_east;
    _switch_input_data.b_b = input->button_south;
    _switch_input_data.b_x = input->button_north;
    _switch_input_data.b_y = input->button_west;

    _switch_input_data.d_down   = input->dpad_down;
    _switch_input_data.d_left   = input->dpad_left;
    _switch_input_data.d_right  = input->dpad_right;
    _switch_input_data.d_up     = input->dpad_up;

    _switch_input_data.b_capture    = input->button_capture;
    _switch_input_data.b_home       = input->button_home;
    _switch_input_data.b_minus      = input->button_minus;
    _switch_input_data.b_plus       = input->button_plus;

    _switch_input_data.t_l  = input->trigger_l;
    _switch_input_data.t_r  = input->trigger_r;
    _switch_input_data.t_zl = input->trigger_zl;
    _switch_input_data.t_zr = input->trigger_zr;

    _switch_input_data.sb_left  = input->button_stick_left;
    _switch_input_data.sb_right = input->button_stick_right;
}