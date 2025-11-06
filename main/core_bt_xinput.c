#include "hoja_includes.h"   // must come first
#include "core_bt_xinput.h"
#include "LED.h"

static const char *TAG = "core_bt_xinput";
static bool xinput_connected = false;

// External definitions (same as used in your original library)
extern esp_hid_raw_report_map_t xinput_report_maps[1];
extern util_bt_app_params_s xinput_app_params;
extern esp_hid_device_config_t xinput_hidd_config;

// XInput HID Descriptor
const uint8_t xinput_hid_report_descriptor[XINPUT_HID_REPORT_MAP_LEN] = {
0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x05,        // Usage (Game Pad)
0xA1, 0x01,        // Collection (Application)
0x85, 0x01,        //   Report ID (1)
0x09, 0x01,        //   Usage (Pointer)
0xA1, 0x00,        //   Collection (Physical)
0x09, 0x30,        //     Usage (X)
0x09, 0x31,        //     Usage (Y)
0x15, 0x00,        //     Logical Minimum (0)
0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65534)
0x95, 0x02,        //     Report Count (2)
0x75, 0x10,        //     Report Size (16)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0x09, 0x01,        //   Usage (Pointer)
0xA1, 0x00,        //   Collection (Physical)
0x09, 0x32,        //     Usage (Z)
0x09, 0x35,        //     Usage (Rz)
0x15, 0x00,        //     Logical Minimum (0)
0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65534)
0x95, 0x02,        //     Report Count (2)
0x75, 0x10,        //     Report Size (16)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0x05, 0x02,        //   Usage Page (Sim Ctrls)
0x09, 0xC5,        //   Usage (Brake)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
0x95, 0x01,        //   Report Count (1)
0x75, 0x0A,        //   Report Size (10)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x75, 0x06,        //   Report Size (6)
0x95, 0x01,        //   Report Count (1)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x02,        //   Usage Page (Sim Ctrls)
0x09, 0xC4,        //   Usage (Accelerator)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
0x95, 0x01,        //   Report Count (1)
0x75, 0x0A,        //   Report Size (10)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x75, 0x06,        //   Report Size (6)
0x95, 0x01,        //   Report Count (1)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
0x09, 0x39,        //   Usage (Hat switch)
0x15, 0x01,        //   Logical Minimum (1)
0x25, 0x08,        //   Logical Maximum (8)
0x35, 0x00,        //   Physical Minimum (0)
0x46, 0x3B, 0x01,  //   Physical Maximum (315)
0x66, 0x14, 0x00,  //   Unit (System: English Rotation, Length: Centimeter)
0x75, 0x04,        //   Report Size (4)
0x95, 0x01,        //   Report Count (1)
0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
0x75, 0x04,        //   Report Size (4)
0x95, 0x01,        //   Report Count (1)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x35, 0x00,        //   Physical Minimum (0)
0x45, 0x00,        //   Physical Maximum (0)
0x65, 0x00,        //   Unit (None)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x09,        //   Usage Page (Button)
0x19, 0x01,        //   Usage Minimum (0x01)
0x29, 0x0F,        //   Usage Maximum (0x0F)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x01,        //   Logical Maximum (1)
0x75, 0x01,        //   Report Size (1)
0x95, 0x0F,        //   Report Count (15)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x75, 0x01,        //   Report Size (1)
0x95, 0x01,        //   Report Count (1)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x0C,        //   Usage Page (Consumer)
0x0A, 0x24, 0x02,  //   Usage (AC Back)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x01,        //   Logical Maximum (1)
0x95, 0x01,        //   Report Count (1)
0x75, 0x01,        //   Report Size (1)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x15, 0x00,        //   Logical Minimum (0)
0x25, 0x00,        //   Logical Maximum (0)
0x75, 0x07,        //   Report Size (7)
0x95, 0x01,        //   Report Count (1)
0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x0C,        //   Usage Page (Consumer)
0x09, 0x01,        //   Usage (Consumer Control)
0x85, 0x02,        //   Report ID (2)
0xA1, 0x01,        //   Collection (Application)
0x05, 0x0C,        //     Usage Page (Consumer)
0x0A, 0x23, 0x02,  //     Usage (AC Home)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x01,        //     Logical Maximum (1)
0x95, 0x01,        //     Report Count (1)
0x75, 0x01,        //     Report Size (1)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x00,        //     Logical Maximum (0)
0x75, 0x07,        //     Report Size (7)
0x95, 0x01,        //     Report Count (1)
0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0x05, 0x0F,        //   Usage Page (PID Page)
0x09, 0x21,        //   Usage (0x21)
0x85, 0x03,        //   Report ID (3)
0xA1, 0x02,        //   Collection (Logical)
0x09, 0x97,        //     Usage (0x97)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x01,        //     Logical Maximum (1)
0x75, 0x04,        //     Report Size (4)
0x95, 0x01,        //     Report Count (1)
0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x00,        //     Logical Maximum (0)
0x75, 0x04,        //     Report Size (4)
0x95, 0x01,        //     Report Count (1)
0x91, 0x03,        //     Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0x09, 0x70,        //     Usage (0x70)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x64,        //     Logical Maximum (100)
0x75, 0x08,        //     Report Size (8)
0x95, 0x04,        //     Report Count (4)
0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0x09, 0x50,        //     Usage (0x50)
0x66, 0x01, 0x10,  //     Unit (System: SI Linear, Time: Seconds)
0x55, 0x0E,        //     Unit Exponent (-2)
0x15, 0x00,        //     Logical Minimum (0)
0x26, 0xFF, 0x00,  //     Logical Maximum (255)
0x75, 0x08,        //     Report Size (8)
0x95, 0x01,        //     Report Count (1)
0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0x09, 0xA7,        //     Usage (0xA7)
0x15, 0x00,        //     Logical Minimum (0)
0x26, 0xFF, 0x00,  //     Logical Maximum (255)
0x75, 0x08,        //     Report Size (8)
0x95, 0x01,        //     Report Count (1)
0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0x65, 0x00,        //     Unit (None)
0x55, 0x00,        //     Unit Exponent (0)
0x09, 0x7C,        //     Usage (0x7C)
0x15, 0x00,        //     Logical Minimum (0)
0x26, 0xFF, 0x00,  //     Logical Maximum (255)
0x75, 0x08,        //     Report Size (8)
0x95, 0x01,        //     Report Count (1)
0x91, 0x02,        //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
0xC0,              //   End Collection
0x05, 0x06,        //   Usage Page (Generic Dev Ctrls)
0x09, 0x20,        //   Usage (Battery Strength)
0x85, 0x04,        //   Report ID (4)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x75, 0x08,        //   Report Size (8)
0x95, 0x01,        //   Report Count (1)
0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              // End Collection

// 334 bytes
};

// XInput HID report maps
esp_hid_raw_report_map_t xinput_report_maps[1] = {
    {
        .data = xinput_hid_report_descriptor,
        .len = (uint16_t) XINPUT_HID_REPORT_MAP_LEN
    }
};

uint8_t xinput_hidd_service_uuid128[] = {
        0xEC, 0x67, 0x63, 0x03, 0x3D, 0x9A, 0x42, 0x64, 0xB8, 0x95, 0xAC, 0x1B, 0xE9, 0x9D, 0x63, 0x79,
    };

// Bluetooth App setup data
util_bt_app_params_s xinput_app_params = {
    .ble_hidd_cb        = xinput_ble_hidd_cb,
    .ble_gap_cb         = xinput_ble_gap_cb,
    .bt_mode            = ESP_BT_MODE_BLE,
    .appearance         = ESP_HID_APPEARANCE_GAMEPAD,
    .uuid128            = xinput_hidd_service_uuid128,
};

esp_hid_device_config_t xinput_hidd_config = {
    .vendor_id  = HID_VEND_XINPUT,
    .product_id = HID_PROD_XINPUT,
    .version    = 0x0000,
    .device_name = "XInput BLE Gamepad",
    .manufacturer_name = "HHL",
    .serial_number = "000000",
    .report_maps    = xinput_report_maps,
    .report_maps_len = 1,
};

// ============================================================
// BLE HIDD CALLBACK
// ============================================================
void xinput_ble_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    //esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "BLE start (advertising for XInput)");
        led_set_state(LED_PAIRING);
        esp_hid_ble_gap_adv_start();
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "XInput BLE connected");
        xinput_connected = true;
        led_set_state(LED_PLAYER_FLASH);
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "XInput BLE disconnected");
        xinput_connected = false;
        led_set_state(LED_PAIRING);
        esp_hid_ble_gap_adv_start();
        break;

    default:
        break;
    }
}

// ============================================================
// BLE GAP CALLBACK
// ============================================================
void xinput_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (param->ble_security.auth_cmpl.success)
        {
            ESP_LOGI(TAG, "BLE auth success (XInput)");
            led_set_state(LED_PLAYER_FLASH);
        }
        else
        {
            ESP_LOGW(TAG, "BLE auth failed, restarting advertising");
            esp_hid_ble_gap_adv_start();
        }
        break;

    default:
        break;
    }
}

// ============================================================
// START / STOP
// ============================================================
esp_err_t core_bt_xinput_start(void)
{
    esp_err_t err;
    ESP_LOGI(TAG, "Initializing XInput BLE mode...");

    // Initialize Bluetooth
    err = util_bluetooth_init(global_loaded_settings.paired_host_switch_mac);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluetooth init failed (%d)", err);
        return err;
    }

    // Register the XInput HID profile
	err = util_bluetooth_register_app(&xinput_app_params, &xinput_hidd_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "App register failed (%d)", err);
        return err;
    }

    ESP_LOGI(TAG, "XInput BLE profile started.");
    return ESP_OK;

}

void core_bt_xinput_stop(void)
{
    ESP_LOGI(TAG, "Stopping XInput BLE mode...");
    util_bluetooth_deinit();
    xinput_connected = false;
}

// ============================================================
// SEND INPUT (CALLED FROM controller_task() IN main.c)
// ============================================================
void xinput_bt_sendinput(i2cinput_input_s *input)
{
    if (!xinput_connected) return;
	if (!xinput_app_params.hid_dev) {
		ESP_LOGW(TAG, "XInput HID device not ready");
		return;
	}

    uint8_t report[XI_HID_LEN] = {0};

    // Map to XInput HID structure
    xi_input_s xi = {0};
    xi.button_a = input->button_east;
    xi.button_b = input->button_south;
    xi.button_x = input->button_north;
    xi.button_y = input->button_west;
    xi.dpad_up = input->dpad_up;
    xi.dpad_down = input->dpad_down;
    xi.dpad_left = input->dpad_left;
    xi.dpad_right = input->dpad_right;

    xi.lt = input->lt;   // left trigger analog
    xi.rt = input->rt;   // right trigger analog
    xi.stick_left_x  = input->lx;
	xi.stick_left_y  = input->ly;
	xi.stick_right_x = input->rx;
	xi.stick_right_y = input->ry;


    // Copy the struct into the outgoing report
    memcpy(report, &xi, sizeof(xi_input_s));

    esp_err_t res = esp_hidd_dev_input_set(
        xinput_app_params.hid_dev,
        0,
        XI_INPUT_REPORT_ID,
        report,
        XI_HID_LEN);

    if (res != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to send XInput report: %d", res);
    }
}

// ============================================================
// OPTIONAL HELPERS (STATUS)
// ============================================================
bool xinput_is_connected(void)
{
    return xinput_connected;
}
