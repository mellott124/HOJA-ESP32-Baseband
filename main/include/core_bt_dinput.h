#ifndef CORE_BT_DINPUT_H
#define CORE_BT_DINPUT_H

#include "hoja_includes.h"
#include "hoja.h"

#define DINPUT_REPORT_SIZE 6   // X Y Z Rz + 16 buttons
#define DINPUT_REPORT_ID   1

// 6-byte DInput packet:
// [0] X
// [1] Y
// [2] Z
// [3] Rz
// [4] Buttons LSB
// [5] Buttons MSB
typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t z;
    uint8_t rz;
    uint16_t buttons;
} __attribute__((packed)) dinput_report_s;

void dinput_bt_sendinput(i2cinput_input_s *input);

int  core_bt_dinput_start(void);
void core_bt_dinput_stop(void);

void dinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void dinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

void dinput_bt_end_task();
void _dinput_bt_task(void *parameters);
uint8_t normalize_axis(uint16_t v);
void debug_print_input_changes(const i2cinput_input_s *in);

#endif
