#ifndef LED_H
#define LED_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "stdint.h"

// --------------------------------------------------------------------------
// LED DEFINES (Common-anode RGB LED; LOW = ON, HIGH = OFF)
// --------------------------------------------------------------------------
#define HEART_BEAT_RED      GPIO_NUM_26
#define HEART_BEAT_GREEN    GPIO_NUM_27
#define HEART_BEAT_BLUE     GPIO_NUM_25

#define GPIO_OUTPUT_LED_MASK ((1ULL<<HEART_BEAT_RED)|(1ULL<<HEART_BEAT_GREEN)|(1ULL<<HEART_BEAT_BLUE))
#define HIGH 1
#define LOW  0

// --------------------------------------------------------------------------
// LED STATE ENUMS
// --------------------------------------------------------------------------
typedef enum {
    LED_OFF = 0,
    LED_IDLE,            // Breathing blue
    LED_PAIRING,         // Fast blue blink
    LED_CONNECTED,       // Solid green
    LED_ERROR,           // Flashing red
    LED_PLAYER_FLASH,    // Blink N times to indicate player number
    LED_DISCONNECTED     // Off
} led_state_t;

// --------------------------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------------------------

/**
 * @brief Initialize LED hardware and start LED control task.
 *        Configures LEDC hardware PWM and reclaims DAC pins.
 */
void led_init(void);

/**
 * @brief Set the active LED state (color/animation behavior).
 */
void led_set_state(led_state_t new_state);

/**
 * @brief Task that handles LED state updates.
 *        Runs automatically from led_init().
 */
void led_task(void *arg);

/**
 * @brief Boot-time RGB sweep animation.
 */
void led_boot_sweep(void);

/**
 * @brief Set player number (for LED_PLAYER_FLASH mode).
 */
void led_set_player_number(uint8_t num);

/**
 * @brief Get the currently stored player number.
 */
uint8_t led_get_player_number(void);

/**
 * @brief Reclaim DAC pins for LED PWM output.
 *        Disables DAC on GPIO25/26 to allow LEDC PWM usage.
 */
void led_reclaim_dac_pins(void);

/**
 * @brief Reinitialize LEDC PWM configuration (if needed after BT startup).
 */
void led_pwm_reinit(void);

#endif // LED_H
