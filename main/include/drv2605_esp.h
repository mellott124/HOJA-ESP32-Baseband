#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

// I2C configuration
#define DRV2605_I2C_PORT      I2C_NUM_0
#define DRV2605_I2C_ADDR      0x5A
#define DRV2605_SDA_PIN       21
#define DRV2605_SCL_PIN       22
#define DRV2605_I2C_SPEED_HZ  400000

// Stubbed enable pins for future DRV2625
#define DRV2605_GPIO_R        19
#define DRV2605_GPIO_L        (-1)   // not wired yet

// Register map (subset)
#define DRV2605_REG_STATUS        0x00
#define DRV2605_REG_MODE          0x01
#define DRV2605_REG_RTPIN         0x02
#define DRV2605_REG_LIBRARY_SEL   0x03
#define DRV2605_REG_WAVESEQ1      0x04
#define DRV2605_REG_GO            0x0C
#define DRV2605_REG_FEEDBACK      0x1A
#define DRV2605_REG_CONTROL1      0x1B
#define DRV2605_REG_CONTROL3      0x1D

// Modes
#define DRV2605_MODE_INT_TRIG     0x00
#define DRV2605_MODE_EXT_TRIG_EDGE 0x01
#define DRV2605_MODE_RTP          0x05
#define DRV2605_MODE_DIAG         0x06
#define DRV2605_MODE_AUTO_CAL     0x07
#define DRV2605_MODE_STANDBY      0x40

esp_err_t drv2605_init(void);
esp_err_t drv2605_set_mode(uint8_t mode);
esp_err_t drv2605_set_rtp(uint8_t value);
esp_err_t drv2605_stop(void);
void      drv2605_gpio_enable_r(bool on);
void      drv2605_gpio_enable_l(bool on);
esp_err_t drv2605_write(uint8_t reg, uint8_t val);
