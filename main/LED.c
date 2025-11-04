#include "LED.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/dac.h"
#include "esp_timer.h"


#define TAG "LED"

// -------------------------------------------------------------
// Common-anode RGB LED wiring (LOW = ON, HIGH = OFF)
// -------------------------------------------------------------
#define LED_R_PIN  GPIO_NUM_26
#define LED_G_PIN  GPIO_NUM_27
#define LED_B_PIN  GPIO_NUM_25

#define LED_MAX_DUTY     225
#define LED_FREQ_HZ      5000
#define LED_RES_BITS     LEDC_TIMER_8_BIT
#define LED_OFF_DUTY     0  // after inversion, 0 = fully OFF, 255 = fully ON

// -------------------------------------------------------------
// LED state storage
// -------------------------------------------------------------
static led_state_t current_led_state = LED_OFF;
static uint8_t player_number = 0;

// -------------------------------------------------------------
// Helper: reclaim DAC pins for LEDC use
// -------------------------------------------------------------
void led_reclaim_dac_pins(void)
{
    dac_output_disable(DAC_CHAN_0);
    dac_output_disable(DAC_CHAN_1);
    ESP_LOGI(TAG, "DAC outputs disabled — pins reclaimed for LED PWM");
}

// -------------------------------------------------------------
// Hardware PWM setup with output inversion
// -------------------------------------------------------------
static void led_hw_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LED_RES_BITS,
        .freq_hz          = LED_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    const int pins[3] = {LED_R_PIN, LED_G_PIN, LED_B_PIN};
    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = LED_OFF_DUTY,
            .hpoint     = 0,
            .flags.output_invert = true // invert output for common-anode LEDs
        };
        ledc_channel_config(&ch);
    }

    ESP_LOGI(TAG, "LED initialized (pins: R=%d, G=%d, B=%d, inverted outputs)", LED_R_PIN, LED_G_PIN, LED_B_PIN);
}

// -------------------------------------------------------------
// Helper: set RGB intensity (0–255 now maps normally)
// -------------------------------------------------------------
static inline void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (r > LED_MAX_DUTY) r = LED_MAX_DUTY;
    if (g > LED_MAX_DUTY) g = LED_MAX_DUTY;
    if (b > LED_MAX_DUTY) b = LED_MAX_DUTY;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

// -------------------------------------------------------------
// Boot-up sweep animation
// -------------------------------------------------------------
void led_boot_sweep(void)
{
    //ESP_LOGI(TAG, "LED boot sweep start");
    for (int i = 0; i <= LED_MAX_DUTY; i += 4) {
        led_set_rgb(i, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    for (int i = 0; i <= LED_MAX_DUTY; i += 4) {
        led_set_rgb(0, i, 0);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    for (int i = 0; i <= LED_MAX_DUTY; i += 4) {
        led_set_rgb(0, 0, i);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    led_set_rgb(0, 0, 0);
    //ESP_LOGI(TAG, "LED boot sweep done");
}

// -------------------------------------------------------------
// Task for LED animation states
// -------------------------------------------------------------
void led_task(void *arg)
{
    bool on = false;
    int fade = 0;
    int dir = 1;

    led_boot_sweep();

    while (true) {
        switch (current_led_state) {
            case LED_IDLE: // soft blue breathing
                fade += dir * 4;
                if (fade >= LED_MAX_DUTY) { fade = LED_MAX_DUTY; dir = -1; }
                if (fade <= 0)             { fade = 0; dir = 1; }
                led_set_rgb(0, 0, fade);
                vTaskDelay(pdMS_TO_TICKS(16));
                break;

            case LED_PAIRING: // fast blue blink
                on = !on;
                led_set_rgb(0, 0, on ? LED_MAX_DUTY : 0);
                vTaskDelay(pdMS_TO_TICKS(250));
                break;

            case LED_CONNECTED: // solid green
                led_set_rgb(0, LED_MAX_DUTY, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_ERROR: // red flash
                on = !on;
                led_set_rgb(on ? LED_MAX_DUTY : 0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;

            case LED_PLAYER_FLASH:
			{
				// Continuous blink of player number forever
				for (uint8_t i = 0; i < player_number; i++) {
					led_set_rgb(0, LED_MAX_DUTY, 0);           // LED on (green)
					vTaskDelay(pdMS_TO_TICKS(150));            // on duration
					led_set_rgb(0, 0, 0);                      // off
					vTaskDelay(pdMS_TO_TICKS(150));            // off duration
				}

				// pause between sets of flashes
				vTaskDelay(pdMS_TO_TICKS(600));

				// stay in LED_PLAYER_FLASH; do NOT change current_led_state
				// the main LED loop will revisit this case again next cycle
			}
			break;


            case LED_DISCONNECTED:
            case LED_OFF:
            default:
                led_set_rgb(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
        }
    }
}

// -------------------------------------------------------------
// Public API
// -------------------------------------------------------------
void led_init(void)
{
    led_reclaim_dac_pins();
    led_hw_init();
    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, 4, NULL, 0);
}

void led_set_state(led_state_t state)
{
    current_led_state = state;
    //ESP_LOGI(TAG, "LED: state -> %d", state);
}

void led_set_player_number(uint8_t num)
{
    player_number = num;
    current_led_state = LED_PLAYER_FLASH;  // trigger blink sequence
}


uint8_t led_get_player_number(void)
{
    return player_number;
}
