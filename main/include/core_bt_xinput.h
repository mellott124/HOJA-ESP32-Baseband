#ifndef CORE_BT_XINPUT_H
#define CORE_BT_XINPUT_H

#include "hoja_includes.h"

// ============================================================
// DEFINES
// ============================================================

#define XI_HID_LEN           16
#define XI_INPUT_REPORT_ID   0x01
#define HID_PROD_XINPUT 0x0B13
#define HID_VEND_XINPUT 0x045E
#define XINPUT_HID_REPORT_MAP_LEN 334

// ============================================================
// STRUCTURES
// ============================================================

// Basic XInput-style input report format (used for HID packet building)
typedef struct
{
    uint16_t stick_left_x;
    uint16_t stick_left_y;
    uint16_t stick_right_x;
    uint16_t stick_right_y;
    uint8_t  lt;
    uint8_t  rt;
    uint8_t  dpad_up;
    uint8_t  dpad_down;
    uint8_t  dpad_left;
    uint8_t  dpad_right;
    uint8_t  button_a;
    uint8_t  button_b;
    uint8_t  button_x;
    uint8_t  button_y;
} __attribute__((packed)) xi_input_s;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

// Core Bluetooth lifecycle
esp_err_t core_bt_xinput_start(void);
void core_bt_xinput_stop(void);

// Send input (called from controller_task in main.c)
void xinput_bt_sendinput(i2cinput_input_s *input);

// BLE callbacks
void xinput_ble_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
void xinput_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// Optional status query
bool xinput_is_connected(void);

#endif // CORE_BT_XINPUT_H
