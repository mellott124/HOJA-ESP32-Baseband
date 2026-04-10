#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

// -----------------------------------------------------------------------------
// I2C configuration
// -----------------------------------------------------------------------------
#define DRV2625_I2C_PORT       I2C_NUM_0
#define DRV2625_I2C_ADDR       0x5A
#define DRV2625_SDA_PIN        21
#define DRV2625_SCL_PIN        22
#define DRV2625_I2C_SPEED_HZ   400000

// -----------------------------------------------------------------------------
// DRV2625 NRST pins
// Left  = GPIO12
// Right = GPIO19
// -----------------------------------------------------------------------------
#define DRV2625_GPIO_NRST_L    12
#define DRV2625_GPIO_NRST_R    19

typedef enum {
    DRV2625_CH_LEFT  = 0,
    DRV2625_CH_RIGHT = 1
} drv2625_channel_t;

// -----------------------------------------------------------------------------
// DRV2625 register map (subset)
// -----------------------------------------------------------------------------
#define DRV2625_REG_ID                 0x00
#define DRV2625_REG_STATUS             0x01
#define DRV2625_REG_MODE               0x07
#define DRV2625_REG_ACTUATOR           0x08
#define DRV2625_REG_GO                 0x0C
#define DRV2625_REG_RTP_INPUT          0x0E
#define DRV2625_REG_RATED_VOLTAGE      0x1F
#define DRV2625_REG_OD_CLAMP           0x20
#define DRV2625_REG_DRIVE_TIME         0x27
#define DRV2625_REG_OL_CONTROL         0x2C
#define DRV2625_REG_OL_LRA_PERIOD_H    0x2E
#define DRV2625_REG_OL_LRA_PERIOD_L    0x2F

// -----------------------------------------------------------------------------
// Open-loop control
// -----------------------------------------------------------------------------
#define DRV2625_AUTO_OPEN_LOOP_OFF     (0u << 7)
#define DRV2625_AUTO_OL_CNT_3          (0u << 5)
#define DRV2625_LRA_WAVE_SHAPE_SQUARE  (0u << 0)
#define DRV2625_LRA_WAVE_SHAPE_SINE    (1u << 0)

#define DRV2625_OL_CONTROL_SQUARE \
    (DRV2625_AUTO_OPEN_LOOP_OFF | DRV2625_AUTO_OL_CNT_3 | DRV2625_LRA_WAVE_SHAPE_SQUARE)

#define DRV2625_OL_CONTROL_SINE \
    (DRV2625_AUTO_OPEN_LOOP_OFF | DRV2625_AUTO_OL_CNT_3 | DRV2625_LRA_WAVE_SHAPE_SINE)

// -----------------------------------------------------------------------------
// Register 0x07: MODE / TRIG
// TRIG_PIN_FUNC[3:2], MODE[1:0]
// -----------------------------------------------------------------------------
#define DRV2625_TRIG_PULSE             (0u << 2)
#define DRV2625_TRIG_LEVEL             (1u << 2)
#define DRV2625_TRIG_INT_I2C           (2u << 2)

#define DRV2625_MODE_RTP               0x00
#define DRV2625_MODE_WAVEFORM          0x01
#define DRV2625_MODE_DIAG              0x02
#define DRV2625_MODE_AUTOCAL           0x03

#define DRV2625_MODECFG_RTP_I2C \
    (DRV2625_TRIG_INT_I2C | DRV2625_MODE_RTP)

// -----------------------------------------------------------------------------
// Register 0x08: actuator / loop config
// -----------------------------------------------------------------------------
#define DRV2625_ACTUATOR_ERM           (0u << 7)
#define DRV2625_ACTUATOR_LRA           (1u << 7)

#define DRV2625_LOOP_CLOSED            (0u << 6)
#define DRV2625_LOOP_OPEN              (1u << 6)

#define DRV2625_HYBRID_LOOP_OFF        (0u << 5)
#define DRV2625_AUTO_BRK_OL_OFF        (0u << 4)
#define DRV2625_AUTO_BRK_STBY_ON       (1u << 3)
#define DRV2625_INPUT_SLOPE_OFF        (0u << 2)

#define DRV2625_ACT_CFG_LRA_CLOSED \
    (DRV2625_ACTUATOR_LRA | DRV2625_LOOP_CLOSED | \
     DRV2625_HYBRID_LOOP_OFF | DRV2625_AUTO_BRK_OL_OFF | \
     DRV2625_AUTO_BRK_STBY_ON | DRV2625_INPUT_SLOPE_OFF)

// -----------------------------------------------------------------------------
// Register 0x0C: GO
// -----------------------------------------------------------------------------
#define DRV2625_GO                     0x01
#define DRV2625_STOP                   0x00

typedef enum {
    DRV2625_TEST_MODE_CLOSED_LOOP_RTP = 0,
    DRV2625_TEST_MODE_OL_LRA_160_SQUARE,
    DRV2625_TEST_MODE_OL_LRA_320_SQUARE,
    DRV2625_TEST_MODE_OL_LRA_160_SINE,
    DRV2625_TEST_MODE_OL_LRA_320_SINE
} drv2625_test_mode_t;

// Compatibility API using existing file/function names
esp_err_t drv2605_init(void);
esp_err_t drv2605_set_rtp(uint8_t value);
esp_err_t drv2605_stop(void);
void      drv2605_gpio_enable_r(bool on);
void      drv2605_gpio_enable_l(bool on);

// New channel-aware API
esp_err_t drv2625_init(void);
esp_err_t drv2625_select(drv2625_channel_t ch);
esp_err_t drv2625_init_channel(drv2625_channel_t ch);
esp_err_t drv2625_set_rtp_channel(drv2625_channel_t ch, int8_t value);
esp_err_t drv2625_stop_channel(drv2625_channel_t ch);
esp_err_t drv2625_stop_all(void);

esp_err_t drv2625_set_test_mode_channel(drv2625_channel_t ch,
                                        drv2625_test_mode_t mode,
                                        uint8_t od_clamp);

esp_err_t drv2625_test_pulse3_channel(drv2625_channel_t ch,
                                      drv2625_test_mode_t mode,
                                      uint8_t od_clamp,
                                      uint32_t pulse_on_ms,
                                      uint32_t pulse_off_ms);

esp_err_t drv2625_test_pulse3_both(drv2625_test_mode_t mode,
                                   uint8_t od_clamp,
                                   uint32_t pulse_on_ms,
                                   uint32_t pulse_off_ms);
