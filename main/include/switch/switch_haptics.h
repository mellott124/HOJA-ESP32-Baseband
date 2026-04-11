/* =========================================================================================
 *  File: switch_haptics.h
 *  Project: RetroOnyx Virtual Boy Wireless Controller
 *  Summary:
 *      Public interface for the Nintendo Switch haptics subsystem. This interface
 *      exposes initialization and rumble translation entry points for dual DRV2625
 *      playback, including left-only, right-only, and broadcast playback paths.
 * ========================================================================================= */
#ifndef SWITCH_HAPTICS_H
#define SWITCH_HAPTICS_H

#include "hoja_includes.h"
#include "drv2605_esp.h"

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

typedef enum
{
    Switch5BitAction_Ignore = 0x0,
    Switch5BitAction_Default = 0x1,
    Switch5BitAction_Substitute = 0x2,
    Switch5BitAction_Sum = 0x3,
} Switch5BitAction_t;

typedef struct
{
    Switch5BitAction_t am_action : 8;
    Switch5BitAction_t fm_action : 8;
    float am_offset;
    float fm_offset;
} Switch5BitCommand_s;

// This represents the 4 uint8_t bytes of data for a single side of controller rumble data
typedef struct
{
    union
    {
        struct
        {
            uint32_t data : 30;
            uint32_t frame_count : 2;
        };

        // Type 1
        struct
        {
            uint32_t cmd_hi_2 : 5;
            uint32_t cmd_lo_2 : 5;
            uint32_t cmd_hi_1 : 5;
            uint32_t cmd_lo_1 : 5;
            uint32_t cmd_hi_0 : 5;
            uint32_t cmd_lo_0 : 5;
            uint32_t frame_count : 2;
        } type1;

        // Type 2
        struct
        {
            uint32_t padding        : 2;
            uint32_t freq_hi        : 7;
            uint32_t amp_hi         : 7;
            uint32_t freq_lo        : 7;
            uint32_t amp_lo         : 7;
            uint32_t frame_count    : 2;
        } type2;

        // Type 3
        struct
        {
            uint32_t high_select    : 1;
            uint32_t freq_xx_0      : 7;
            uint32_t cmd_hi_1       : 5;
            uint32_t cmd_lo_1       : 5;
            uint32_t cmd_xx_0       : 5;
            uint32_t amp_xx_0       : 7;
            uint32_t frame_count    : 2;
        } type3;

        // Type 4
        struct
        {
            uint32_t high_select    : 1;
            uint32_t blank          : 1;
            uint32_t freq_select    : 1;
            uint32_t cmd_hi_2       : 5;
            uint32_t cmd_lo_2       : 5;
            uint32_t cmd_hi_1       : 5;
            uint32_t cmd_lo_1       : 5;
            uint32_t xx_xx_0        : 7;
            uint32_t frame_count    : 2;
        } type4;
    };
} __attribute__((packed)) SwitchHapticPacket_s;

void haptics_rumble_translate(const uint8_t *data);
void haptics_initialize_lookup_tables(void);
void haptics_init(void);

#endif