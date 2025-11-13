#include "core_bt_xinput.h"
#include "esp_log.h"
#include "driver/gpio.h"   // Needed for gpio_set_level(), GPIO_NUM_xx
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LED.h"           // Needed for led_all_off(), led_set_state(), LED_IDLE, etc.

#define HID_PROD_XINPUT  0x045E  // Microsoft
#define HID_VEND_XINPUT  0x028E  // Xbox 360 Controller (example)
#define DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define DEFAULT_US_DELAY (8*1000)

static TaskHandle_t _xinput_bt_task_handle = NULL;
static volatile bool _xinput_paired = false;
static volatile bool _hid_connected = false;
static xinput_input_s _xinput_input_data = {0};

// --------------------------------------------------------------------------
// HID Descriptor for XInput
// --------------------------------------------------------------------------
static const uint8_t xinput_hid_descriptor[] = {

    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)

    // Buttons (12 total)
    0x05, 0x09,       // Usage Page (Button)
    0x19, 0x01,       // Usage Minimum (1)
    0x29, 0x0C,       // Usage Maximum (12)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x95, 0x0C,       // Report Count (12)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x02,       // Input (Data,Var,Abs)

    // Padding to next byte
    0x95, 0x04,       // Report Count (4)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x03,       // Input (Const,Var,Abs)

    // Hat switch (8-way)
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x39,       // Usage (Hat switch)
    0x15, 0x00,       // Logical Min (0)
    0x25, 0x07,       // Logical Max (7)
    0x35, 0x00,       // Physical Min (0)
    0x46, 0x3B, 0x01, // Physical Max (315 deg)
    0x65, 0x14,       // Unit (Degrees)
    0x75, 0x04,       // Report Size (4)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x42,       // Input (Data,Var,Abs,Null)

    // Padding to next byte
    0x75, 0x04,       // Report Size (4)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x03,       // Input (Const,Var,Abs)

    // Axes: LX, LY, RX, RY (0–255)
    0x09, 0x30,       // Usage (X)
    0x09, 0x31,       // Usage (Y)
    0x09, 0x32,       // Usage (Z)  -> use as RX
    0x09, 0x33,       // Usage (Rz) -> use as RY
    0x15, 0x00,       // Logical Min (0)
    0x26, 0xFF, 0x00, // Logical Max (255)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x04,       // Report Count (4)
    0x81, 0x02,       // Input (Data,Var,Abs)

    0xC0              // End Collection
};

#define XINPUT_HID_REPORT_MAP_LEN  (sizeof(xinput_hid_descriptor))


// XInput HID report maps
static const esp_hid_raw_report_map_t xinput_report_maps[1] = {
    {
        .data = xinput_hid_descriptor,
        .len  = (uint16_t)XINPUT_HID_REPORT_MAP_LEN,
    }
};

// Bluetooth App setup data
util_bt_app_params_s xinput_app_params = {
    .hidd_cb = xinput_bt_hidd_cb,
    .gap_cb = xinput_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

// Classic Bluetooth HID Device Config for XInput
esp_hid_device_config_t xinput_hidd_config = {
    .vendor_id = HID_VEND_XINPUT,
    .product_id = HID_PROD_XINPUT,
    .version = 0x0100,
    .device_name = "Virtual Boy Controller",
    .manufacturer_name = "RetroOnyx",
    .serial_number = "000000",
    .report_maps = (esp_hid_raw_report_map_t *)xinput_report_maps,
    .report_maps_len = 1,
};


void xinput_bt_end_task()
{
    if (_xinput_bt_task_handle != NULL) {
        // Defer deletion to the task context (safe FreeRTOS method)
        ESP_LOGI("xinput_bt_end_task", "Deferring delete of _xinput_bt_task_standard");
        xTaskNotifyGive(_xinput_bt_task_handle);
        _xinput_bt_task_handle = NULL;
    }
}

// XINPUT BTC GAP Event Callback
void xinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "xinput_bt_gap_cb";
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
    ESP_LOGI(TAG, "ACL Connect Complete.");
    // Do nothing here — wait for ESP_HIDD_CONNECT_EVENT before starting input task
    break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
		ESP_LOGI(TAG, "ACL Disconnect Complete.");

		// 🧩 Defer task deletion safely to avoid crash
		if (_xinput_bt_task_handle != NULL) {
			ESP_LOGI(TAG, "Deferring delete of _xinput_bt_task_standard");
			xTaskNotifyGive(_xinput_bt_task_handle);
			_xinput_bt_task_handle = NULL;
		}

		app_set_connected_status(0);
		app_set_power_setting(POWER_CODE_OFF); // your existing shutdown call
		break;

    
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
        _xinput_paired = true;
        app_save_host_mac(INPUT_MODE_XINPUT, &param->auth_cmpl.bda[0]);
        // ✅ Only now disable visibility
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    } else {
        ESP_LOGW(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        // Stay visible for retry
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    }
    break;

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
void xinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "xinput_bt_hidd_cb";

    switch(event)
    {
        // --------------------------------------------------------------
        // HID Start: controller begins advertising or reconnecting
        // --------------------------------------------------------------
		case ESP_HIDD_START_EVENT:
            if(param->start.status == ESP_OK)
            {
                ESP_LOGI(TAG, "START OK");
                led_set_state(LED_PAIRING); 
				
                if (_xinput_paired) {
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
					util_bluetooth_connect(global_loaded_settings.paired_host_xinput_mac);
				} else {
					ESP_LOGI(TAG, "No paired host found — entering discoverable mode.");
					esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
				}
            }
            else
            {
                ESP_LOGE(TAG, "START failed!");
                led_set_state(LED_ERROR);
            }
            break;

        // --------------------------------------------------------------
		// HID Connect: connection established successfully
		// --------------------------------------------------------------
		case ESP_HIDD_CONNECT_EVENT:
			if (param->connect.status == ESP_OK)
			{
				_hid_connected = true;
				ESP_LOGI(TAG, "CONNECT OK");
				led_set_state(LED_CONNECTED);

				// --- Send initial zero report so Windows HID enumerates properly ---
				uint8_t probe[11] = {0};
				esp_err_t r = esp_bt_hid_device_send_report(
					ESP_HIDD_REPORT_TYPE_INTRDATA,  // matches esp_hidd usage for interrupt reports
					0x00,                           // report ID = 0
					sizeof(probe),
					probe
				);
				ESP_LOGI(TAG, "Sent initial zero report (%d bytes), ret=%s", sizeof(probe), esp_err_to_name(r));

				// --- Allow L2CAP config to stabilize before we start sending ---
				vTaskDelay(pdMS_TO_TICKS(300));

				// Ensure we only spawn the loop task once
				if (_xinput_bt_task_handle == NULL)
				{
					BaseType_t res = xTaskCreatePinnedToCore(
						_xinput_bt_task_standard,
						"Standard Send Task",
						4048,
						NULL,
						1,  // Priority 1 is fine for I/O tasks
						&_xinput_bt_task_handle,
						0   // Pin to core 0
					);

					if (res == pdPASS)
					{
						ESP_LOGI(TAG, "Started _xinput_bt_task_standard after CONNECT delay");
					}
					else
					{
						ESP_LOGE(TAG, "Failed to create _xinput_bt_task_standard (res=%ld)", res);
						led_set_state(LED_ERROR);
					}
				}
				else
				{
					ESP_LOGW(TAG, "_xinput_bt_task_standard already running, skipping new start");
				}
			}
			else
			{
				ESP_LOGE(TAG, "CONNECT failed!");
				led_set_state(LED_ERROR);
			}
			break;



        // --------------------------------------------------------------
        // HID Disconnect: controller disconnected (graceful or lost link)
        // --------------------------------------------------------------
		case ESP_HIDD_DISCONNECT_EVENT:
            _hid_connected = false;
            if(param->disconnect.status == ESP_OK)
            {
                ESP_LOGI(TAG, "DISCONNECT OK");
                led_set_state(LED_PAIRING);
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
            else
            {
                ESP_LOGE(TAG, "DISCONNECT failed!");
                led_set_state(LED_ERROR);
            }
            break;

        // --------------------------------------------------------------
        // Protocol mode: report/boot (not visually significant)
        // --------------------------------------------------------------
		case ESP_HIDD_PROTOCOL_MODE_EVENT:
            ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s",
                     param->protocol_mode.map_index,
                     param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
            break;

        // --------------------------------------------------------------
        // Output report (rumble, LEDs, etc.)
        // --------------------------------------------------------------
		case ESP_HIDD_OUTPUT_EVENT:
            // ns_report_handler(param->output.report_id,
                              // param->output.data,
                              // param->output.length);
            break;

		// --------------------------------------------------------------
        // Feature report (debug/log only)
        // --------------------------------------------------------------
        case ESP_HIDD_FEATURE_EVENT:
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
            break;
		
		// --------------------------------------------------------------
        // HID Stop: Bluetooth subsystem halted
        // --------------------------------------------------------------
        case ESP_HIDD_STOP_EVENT:
            ESP_LOGI(TAG, "HID STOP");
            _hid_connected = false;
            led_set_state(LED_IDLE);
            break;
			
		// --------------------------------------------------------------
        // Default: no-op
        // --------------------------------------------------------------
        default:
            break;
    }
}

// --------------------------------------------------------------------------
// Core start function
// --------------------------------------------------------------------------
int core_bt_xinput_start(void)
{
    const char *TAG = "core_bt_xinput";
    int err=1;

    // Initialize analog calibration not needed for XInput
    //xinput_analog_calibration_init();

    // Determine MAC
    uint8_t tmpmac[6] = {0};
    uint8_t *mac = global_live_data.current_mac;

    if ((mac[0] == 0) && (mac[1] == 0)) {
        mac = global_loaded_settings.device_mac_xinput;
    }
    memcpy(tmpmac, mac, 6);

    // --------------------------------------------------
    // Paired host check
    // --------------------------------------------------
    //const uint8_t zero_mac[6] = {0};
    // if (memcmp(global_loaded_settings.paired_host_xinput_mac, zero_mac, 6) != 0) {
        // _xinput_paired = true;
        // ESP_LOGI(TAG,
                 // "Detected stored paired host %02X:%02X:%02X:%02X:%02X:%02X — enabling quick reconnect",
                 // global_loaded_settings.paired_host_xinput_mac[0],
                 // global_loaded_settings.paired_host_xinput_mac[1],
                 // global_loaded_settings.paired_host_xinput_mac[2],
                 // global_loaded_settings.paired_host_xinput_mac[3],
                 // global_loaded_settings.paired_host_xinput_mac[4],
                 // global_loaded_settings.paired_host_xinput_mac[5]);

        // ESP_LOGI(TAG, "Quick reconnect path enabled");
    // } else {
        // _xinput_paired = false;
        // ESP_LOGI(TAG, "No stored paired host found — entering pairing mode.");

        // esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    // }
	// --------------------------------------------------
	// Paired host check (dev override: always force pairing mode)
	// --------------------------------------------------
	_xinput_paired = false; // Force pairing mode for dev builds

    // --------------------------------------------------
    // Initialize Bluetooth with valid MAC
    // --------------------------------------------------
    err = util_bluetooth_init(tmpmac);
	
	ESP_LOGW(TAG, "FORCING PAIRING MODE for development build");
	esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
	
	    // ------------------------------------------------------------------
    // Clear any old Bluetooth bonds before registering the HID app
    // ------------------------------------------------------------------
    int num = esp_bt_gap_get_bond_device_num();
    if (num > 0) {
        esp_bd_addr_t bonded_devices[num];
        esp_bt_gap_get_bond_device_list(&num, bonded_devices);
        for (int i = 0; i < num; i++) {
            esp_bt_gap_remove_bond_device(bonded_devices[i]);
            ESP_LOGI("core_bt_xinput", "Removed bonded device: %02X:%02X:%02X:%02X:%02X:%02X",
                     bonded_devices[i][0], bonded_devices[i][1], bonded_devices[i][2],
                     bonded_devices[i][3], bonded_devices[i][4], bonded_devices[i][5]);
        }
    }

    // --------------------------------------------------
    // Register HID application
    // --------------------------------------------------
	ESP_LOG_BUFFER_HEX("HID_MAP", xinput_report_maps[0].data, xinput_report_maps[0].len);
    err = util_bluetooth_register_app(&xinput_app_params, &xinput_hidd_config);

    return err;
}

// Stop XInput controller core
void core_bt_xinput_stop(void)
{
    util_bluetooth_deinit();
}

void xinput_build_report_8b(const xinput_input_s *in, uint8_t out[8])
{
    uint16_t b = 0;

    b |= (in->btn_a      & 1) << 0;  // Button 1
    b |= (in->btn_b      & 1) << 1;  // Button 2
    b |= (in->btn_select & 1) << 2;  // Button 3
    b |= (in->btn_start  & 1) << 3;  // Button 4
    b |= (in->btn_l      & 1) << 4;  // Button 5
    b |= (in->btn_r      & 1) << 5;  // Button 6
    // Bits 6–11 unused for now

    out[0] = b & 0xFF;
    out[1] = (b >> 8) & 0x0F;

    // ---- Hat (matches your circular d-pad) ----
    if (in->dpad_up) {
        if (in->dpad_right)      out[2] = 1; // up-right
        else if (in->dpad_left)  out[2] = 7; // up-left
        else                     out[2] = 0; // up
    }
    else if (in->dpad_down) {
        if (in->dpad_right)      out[2] = 3; // down-right
        else if (in->dpad_left)  out[2] = 5; // down-left
        else                     out[2] = 4; // down
    }
    else if (in->dpad_right)     out[2] = 2; // right
    else if (in->dpad_left)      out[2] = 6; // left
    else                         out[2] = 0x0F; // **neutral (Null)**

    // ---- Axes (just pass through / center) ----
    out[3] = in->lx;  // LX
    out[4] = in->ly;  // LY
    out[5] = in->rx;  // RX
    out[6] = in->ry;  // RY

    out[7] = 0;       // padding / unused
}


// --------------------------------------------------------------------------
// Standard XInput task: sends input reports over Bluetooth
// --------------------------------------------------------------------------
void _xinput_bt_task_standard(void *parameters)
{
    const char *TAG = "_xinput_bt_task_standard";
	
	uint8_t report[8];
	
	// choose correct report type for your IDF
	#ifndef ESP_HIDD_REPORT_TYPE_INPUT
	  #define HIDD_RPT_TYPE ESP_HIDD_REPORT_TYPE_INTRDATA
	#else
	  #define HIDD_RPT_TYPE ESP_HIDD_REPORT_TYPE_INPUT
	#endif
	
	ESP_LOGI(TAG, "Starting input loop task...");
	app_set_report_timer(DEFAULT_US_DELAY);
	
    for (;;)
    {
        // Handle deferred shutdown requests from GAP callback
		if (ulTaskNotifyTake(pdTRUE, 0)) {
			ESP_LOGI(TAG, "Delete requested, exiting task cleanly");
			vTaskDelete(NULL); // self-delete safely
		}

        if (_hid_connected)
        {
			xinput_build_report_8b(&_xinput_input_data, report);
			const size_t report_len = sizeof(report);
			ESP_LOG_BUFFER_HEX("XIN_SEND", report, report_len);
			esp_err_t ret = esp_bt_hid_device_send_report(
                ESP_HIDD_REPORT_TYPE_INTRDATA,  // interrupt data
                0x00,                           // report ID (0 for XInput)
                sizeof(report),
				report
            );
			
			// strong logging to validate what's actually going out
			ESP_LOGD("_xinput_bt_task_standard",
					 "TX len=%d  data: %02X %02X  %02X %02X  %02X %02X  %02X %02X  %02X %02X %02X  ret=%s",
					 (int)sizeof(report),
					 report[0], report[1], report[2], report[3], report[4], report[5],
					 report[6], report[7], report[8], report[9], report[10],
					 esp_err_to_name(ret));

            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to send XInput report: %d", ret);
            }
			vTaskDelay(pdMS_TO_TICKS(8));
        }
		else
        {
            // If not connected, sleep lightly to prevent watchdog reset
            vTaskDelay(16 / portTICK_PERIOD_MS);
        }
    }
}


// --------------------------------------------------------------------------
// XInput Send Input Function
// Mirrors switch_bt_sendinput exactly
// --------------------------------------------------------------------------
void xinput_bt_sendinput(const i2cinput_input_s *input)
{
    // -------- DPAD --------
    _xinput_input_data.dpad_up    = input->dpad_up;
    _xinput_input_data.dpad_down  = input->dpad_down;
    _xinput_input_data.dpad_left  = input->dpad_left;
    _xinput_input_data.dpad_right = input->dpad_right;

    // -------- FACE BUTTONS (VB: A,B,Select,Start) --------
    _xinput_input_data.btn_a      = input->button_south;  // A
    _xinput_input_data.btn_b      = input->button_east;   // B
    _xinput_input_data.btn_select = input->button_minus;  // SELECT
    _xinput_input_data.btn_start  = input->button_plus;   // START

    // -------- SHOULDERS (VB L/R) --------
    _xinput_input_data.btn_l = input->trigger_l;
    _xinput_input_data.btn_r = input->trigger_r;

    // -------- ANALOG (VB always centered but HOJA provides values) --------
    _xinput_input_data.lx = input->lx >> 8;
    _xinput_input_data.ly = input->ly >> 8;
    _xinput_input_data.rx = input->rx >> 8;
    _xinput_input_data.ry = input->ry >> 8;
}


void log_i2cinput_state(const i2cinput_input_s *in)
{
    ESP_LOGI("XIN_IN",
        "DPAD: U=%u D=%u L=%u R=%u | "
        "BTN: S=%u E=%u W=%u N=%u | "
        "TRIG: L=%u ZL=%u R=%u ZR=%u | "
        "SYS: CAP=%u HOME=%u SAFE=%u SHIP=%u SYNC=%u UNBIND=%u GL=%u GR=%u",
        in->dpad_up, in->dpad_down, in->dpad_left, in->dpad_right,
        in->button_south, in->button_east, in->button_west, in->button_north,
        in->trigger_l, in->trigger_zl, in->trigger_r, in->trigger_zr,
        in->button_capture, in->button_home, in->button_safemode,
        in->button_shipping, in->button_sync, in->button_unbind,
        in->trigger_gl, in->trigger_gr
    );

    ESP_LOGI("XIN_AXES",
        "LX=%u LY=%u RX=%u RY=%u LT=%u RT=%u",
        in->lx, in->ly, in->rx, in->ry, in->lt, in->rt
    );

    ESP_LOGI("XIN_IMU",
        "AX=%d AY=%d AZ=%d  GX=%d GY=%d GZ=%d   POWER=%u",
        in->ax, in->ay, in->az, in->gx, in->gy, in->gz, in->power_stat
    );
}


