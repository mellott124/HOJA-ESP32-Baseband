#ifndef CORE_BT_XINPUT_H
#define CORE_BT_XINPUT_H

// Include any necessary includes from HOJA backend
#include "hoja_includes.h"
#include "hoja.h"

typedef struct {
    uint8_t dpad_up;
    uint8_t dpad_down;
    uint8_t dpad_left;
    uint8_t dpad_right;

    uint8_t btn_a;     // VB A
    uint8_t btn_b;     // VB B
    uint8_t btn_select;
    uint8_t btn_start;
    uint8_t btn_l;     // VB L
    uint8_t btn_r;     // VB R

    // Always centered for VB
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;

} xinput_input_s;


int core_bt_xinput_start(void);
void core_bt_xinput_stop(void);
void xinput_bt_sendinput(const i2cinput_input_s *input);;
void xinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void xinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
void switch_bt_end_task(void);
void _xinput_bt_task_standard(void *parameters);
void xinput_build_report_8b(const xinput_input_s *in, uint8_t out[8]);
void log_i2cinput_state(const i2cinput_input_s *in);

#endif // CORE_BT_XINPUT_H
