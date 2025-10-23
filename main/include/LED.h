#ifndef LED_H
#define LED_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --------------------------------------------------------------------------
// LED DEFINES (active-low RGB LED)
// --------------------------------------------------------------------------
#define HEART_BEAT_RED      GPIO_NUM_26
#define HEART_BEAT_BLUE     GPIO_NUM_25
#define HEART_BEAT_GREEN    GPIO_NUM_27
#define GPIO_OUTPUT_LED_MASK ((1ULL<<HEART_BEAT_RED)|(1ULL<<HEART_BEAT_BLUE)|(1ULL<<HEART_BEAT_GREEN))
#define HIGH 1
#define LOW  0

// LED states
typedef enum {
    LED_IDLE,       // Blue blink (boot/idle)
    LED_PAIRING,    // Amber blink (advertising)
    LED_CONNECTED,  // Green pulse (connected)
    LED_ERROR       // Red blink (error)
} led_state_t;

// Public API
void led_init(void);
void led_task(void *arg);
void led_set_state(led_state_t state);
void led_boot_sweep(void);   // ✅ new helper for startup animation

#endif
