#include "core_bt_xinput.h"
#include "util_bt.h"
#include "util_bt_hid.h"
#include "LED.h"
#include "main.h"
#include "XboxDescriptors.h"

#define TAG "core_bt_xinput"

static bool _xinput_paired = false;
static bool _hid_connected = false;

/* -------------------------------------------------------------------------- */
/*  Forward Declarations                                                      */
/* -------------------------------------------------------------------------- */
static void xinput_pairing_task(void *arg);
static void xinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
static void xinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void xinput_bt_sendinput(i2cinput_input_s *input);

/* -------------------------------------------------------------------------- */
/*  GAP Callback                                                              */
/* -------------------------------------------------------------------------- */
static void xinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    ESP_LOGI(TAG, "GAP_EVT=%d", event);

    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
            memcpy(global_loaded_settings.paired_host_xinput_mac, param->auth_cmpl.bda, 6);
            global_loaded_settings.has_paired_xinput = true;
            save_settings();
        }
        else
        {
            ESP_LOGE(TAG, "Authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL connect complete. Hiding discoverability.");
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL disconnect complete.");
        led_set_state(LED_PAIRING);
        break;

    default:
        ESP_LOGI(TAG, "Unhandled GAP event: %d", event);
        break;
    }
}

/* -------------------------------------------------------------------------- */
/*  HID Device Callback                                                       */
/* -------------------------------------------------------------------------- */
static void xinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_cb_event_t event = (esp_hidd_cb_event_t)id;
    esp_hidd_cb_param_t *param = (esp_hidd_cb_param_t *)event_data;

    switch (event)
    {
        /* ---------------- HID REGISTER / START ---------------- */
        case ESP_HIDD_REGISTER_APP_EVT:
        case 0:
            if (param->register_app.status == ESP_OK)
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

        /* ---------------- CONNECT / DISCONNECT ---------------- */
        case ESP_HIDD_CONNECT_EVENT:
        case ESP_HIDD_OPEN_EVT:
            _hid_connected = true;
            ESP_LOGI(TAG, "XInput HID connected");
            led_set_state(LED_CONNECTED);
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            break;

        case ESP_HIDD_CLOSE_EVT:
            _hid_connected = false;
            ESP_LOGI(TAG, "XInput HID disconnected");
            led_set_state(LED_PAIRING);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;

        /* ---------------- HOST → DEVICE OUTPUT ---------------- */
        case ESP_HIDD_SET_REPORT_EVT:
            ESP_LOGI(TAG, "Host sent Output/Rumble data");
            if (param->set_report.len >= 8)
            {
                const uint8_t *data = param->set_report.data;
                uint8_t left_motor  = data[3];
                uint8_t right_motor = data[4];
                ESP_LOGI(TAG, "Rumble L=%d R=%d", left_motor, right_motor);
            }
            break;

        case ESP_HIDD_INTR_DATA_EVT:
            ESP_LOGI(TAG, "Host sent interrupt/feedback data");
            if (param->intr_data.len > 0)
            {
                ESP_LOG_BUFFER_HEX_LEVEL("HID_INTR_RX", param->intr_data.data,
                                         param->intr_data.len, ESP_LOG_INFO);
            }
            break;

        /* ---------------- STOP / DEFAULT ---------------- */
        case ESP_HIDD_UNREGISTER_APP_EVT:
            _hid_connected = false;
            ESP_LOGI(TAG, "HID stop");
            led_set_state(LED_IDLE);
            break;

        default:
            ESP_LOGI(TAG, "Unhandled HID event: %d", event);
            break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Input Report Builder                                                      */
/* -------------------------------------------------------------------------- */
static void xinput_bt_sendinput(i2cinput_input_s *input)
{
    if (!_hid_connected) return;

    uint8_t report[47] = {0};
    report[0] = 0x01;   // Report ID (matches descriptor)

    uint16_t buttons = 0;
    if (input->button_south) buttons |= 0x0001; // A
    if (input->button_east)  buttons |= 0x0002; // B
    if (input->button_west)  buttons |= 0x0004; // X
    if (input->button_north) buttons |= 0x0008; // Y
    if (input->trigger_l)    buttons |= 0x0010; // LB
    if (input->trigger_r)    buttons |= 0x0020; // RB
    if (input->button_minus) buttons |= 0x0040; // View
    if (input->button_plus)  buttons |= 0x0080; // Menu
    if (input->button_home)  buttons |= 0x0400; // Xbox Logo

    report[1] = buttons & 0xFF;
    report[2] = (buttons >> 8) & 0xFF;

    // Triggers
    report[3] = input->lt >> 8;
    report[4] = input->rt >> 8;

    // Left stick
    report[5] = input->lx & 0xFF;
    report[6] = (input->lx >> 8) & 0xFF;
    report[7] = input->ly & 0xFF;
    report[8] = (input->ly >> 8) & 0xFF;

    // Right stick
    report[9]  = input->rx & 0xFF;
    report[10] = (input->rx >> 8) & 0xFF;
    report[11] = input->ry & 0xFF;
    report[12] = (input->ry >> 8) & 0xFF;

    // D-pad (hat)
    uint8_t dpad = 0x08;
    if (input->dpad_up)    dpad = 0x00;
    if (input->dpad_up && input->dpad_right)  dpad = 0x01;
    if (input->dpad_right) dpad = 0x02;
    if (input->dpad_down && input->dpad_right) dpad = 0x03;
    if (input->dpad_down)  dpad = 0x04;
    if (input->dpad_down && input->dpad_left)  dpad = 0x05;
    if (input->dpad_left)  dpad = 0x06;
    if (input->dpad_up && input->dpad_left)    dpad = 0x07;
    report[13] = dpad;

    esp_err_t res = esp_bt_hid_device_send_report(
                        ESP_HIDD_REPORT_TYPE_INPUT, 0x01,
                        sizeof(report), report);
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Send report failed: %d", res);
}

/* -------------------------------------------------------------------------- */
/*  Pairing Loop                                                              */
/* -------------------------------------------------------------------------- */
static void xinput_pairing_task(void *arg)
{
    while (!_hid_connected)
    {
        ESP_LOGI(TAG, "Still discoverable — waiting for host connection...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/*  Entry Point                                                               */
/* -------------------------------------------------------------------------- */
esp_err_t core_bt_xinput_start(void)
{
    ESP_LOGI(TAG, "Starting XInput Classic BT HID mode...");

    _xinput_paired = false;
    global_loaded_settings.has_paired_xinput = false;
    memset(global_loaded_settings.paired_host_xinput_mac, 0,
           sizeof(global_loaded_settings.paired_host_xinput_mac));

    uint8_t tmpmac[6];
    memcpy(tmpmac, global_loaded_settings.device_mac_xinput, 6);

    ESP_LOGI(TAG, "Applying base BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             tmpmac[0], tmpmac[1], tmpmac[2],
             tmpmac[3], tmpmac[4], tmpmac[5]);

    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    int bonded = esp_bt_gap_get_bond_device_num();
    if (bonded)
    {
        esp_bd_addr_t bonded_dev_list[20];
        esp_bt_gap_get_bond_device_list(&bonded, bonded_dev_list);
        for (int i = 0; i < bonded; i++)
        {
            ESP_LOGW(TAG, "Removing bonded device: %02x:%02x:%02x:%02x:%02x:%02x",
                     bonded_dev_list[i][0], bonded_dev_list[i][1],
                     bonded_dev_list[i][2], bonded_dev_list[i][3],
                     bonded_dev_list[i][4], bonded_dev_list[i][5]);
            esp_bt_gap_remove_bond_device(bonded_dev_list[i]);
        }
    }

    ESP_LOGI(TAG, "Starting in discoverable pairing mode (no stored host)");
    esp_bt_gap_set_device_name("RetroOnyx Gamepad");

    tmpmac[5] ^= 0x02;

    esp_bt_cod_t cod = {
        .major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL,
        .minor = 0x08,
        .service = ESP_BT_COD_SRVC_RENDERING,
    };
    esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));

    if (util_bluetooth_init(tmpmac) != 1)
    {
        ESP_LOGE(TAG, "Bluetooth init failed");
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(800));

    xinput_app_params.hidd_cb = xinput_bt_hidd_cb;
    xinput_app_params.gap_cb  = xinput_bt_gap_cb;

    if (util_bluetooth_register_app(&xinput_app_params, &xinput_hidd_config) != 1)
    {
        ESP_LOGE(TAG, "App registration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "XInput Classic HID mode started successfully");

    vTaskDelay(pdMS_TO_TICKS(600));
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    led_set_state(LED_PAIRING);
    return ESP_OK;
}
