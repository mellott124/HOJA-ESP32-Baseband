#ifndef CORE_BT_SWITCH_H
#define CORE_BT_SWITCH_H

#define SWITCH_BT_REPORT_SIZE 48

// Include any necessary includes from HOJA backend
#include "hoja_includes.h"
#include "hoja.h"

typedef struct
{
    union
    {
        struct
        {
            // Y and C-Up (N64)
            uint8_t b_y       : 1;

            // X and C-Left (N64)
            uint8_t b_x       : 1;

            uint8_t b_b       : 1;
            uint8_t b_a       : 1;
            uint8_t t_r_sr    : 1;
            uint8_t t_r_sl    : 1;
            uint8_t t_r       : 1;

            // ZR and C-Down (N64)
            uint8_t t_zr      : 1;
        };
        uint8_t right_buttons;
    };
    union
    {
        struct
        {
            // Minus and C-Right (N64)
            uint8_t b_minus     : 1;

            // Plus and Start
            uint8_t b_plus      : 1;

            uint8_t sb_right    : 1;
            uint8_t sb_left     : 1;
            uint8_t b_home      : 1;
            uint8_t b_capture   : 1;
            uint8_t none        : 1;
            uint8_t charge_grip_active : 1;
        };
        uint8_t shared_buttons;
    };
    union
    {
        struct
        {
            uint8_t d_down    : 1;
            uint8_t d_up      : 1;
            uint8_t d_right   : 1;
            uint8_t d_left    : 1;
            uint8_t t_l_sr    : 1;
            uint8_t t_l_sl    : 1;
            uint8_t t_l       : 1;

            // ZL and Z (N64)
            uint8_t t_zl      : 1;

        };
        uint8_t left_buttons;
    };

    uint16_t ls_x;
    uint16_t ls_y;
    uint16_t rs_x;
    uint16_t rs_y;

} __attribute__ ((packed)) sw_input_s;

void ns_report_setinputreport_full(uint8_t *buffer);
void ns_set_imu_mode(uint8_t mode);
void ns_reset_report_spacer();

// void ns_controller_setinputreportmode(uint8_t report_mode);
int core_bt_switch_start();
void core_bt_switch_stop(void);
void switch_bt_sendinput(i2cinput_input_s *input);
void switch_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void switch_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
void switch_bt_end_task();
void _switch_bt_task_standard(void *parameters);

#endif