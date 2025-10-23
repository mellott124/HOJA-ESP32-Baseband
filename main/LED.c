#include "LED.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// --------------------------------------------------------------------------
// Internal state
// --------------------------------------------------------------------------
static led_state_t current_led_state = LED_IDLE;
static SemaphoreHandle_t led_mutex = NULL;

static const char *TAG = "LED";

// --------------------------------------------------------------------------
// Initialization
// --------------------------------------------------------------------------
void led_init(void)
{
    // Configure RGB GPIOs as outputs
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL << HEART_BEAT_RED) |
                         (1ULL << HEART_BEAT_GREEN) |
                         (1ULL << HEART_BEAT_BLUE)),
    };
    gpio_config(&io_conf);

    // Turn all off (active-low LEDs = HIGH)
    gpio_set_level(HEART_BEAT_RED, HIGH);
    gpio_set_level(HEART_BEAT_GREEN, HIGH);
    gpio_set_level(HEART_BEAT_BLUE, HIGH);

    // Create mutex for thread-safe updates
    if (!led_mutex) {
        led_mutex = xSemaphoreCreateMutex();
    }

    ESP_LOGI(TAG, "LED initialized (pins: R=%d, G=%d, B=%d)",
             HEART_BEAT_RED, HEART_BEAT_GREEN, HEART_BEAT_BLUE);
}

void led_boot_sweep(void)
{
    // Ensure all off first (active-low LEDs mean HIGH = off)
    gpio_set_level(HEART_BEAT_RED,   HIGH);
    gpio_set_level(HEART_BEAT_GREEN, HIGH);
    gpio_set_level(HEART_BEAT_BLUE,  HIGH);

    // Quick RGB sweep animation
    gpio_set_level(HEART_BEAT_RED, LOW);   vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(HEART_BEAT_RED, HIGH);
    gpio_set_level(HEART_BEAT_GREEN, LOW); vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(HEART_BEAT_GREEN, HIGH);
    gpio_set_level(HEART_BEAT_BLUE, LOW);  vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(HEART_BEAT_BLUE, HIGH);
}


// --------------------------------------------------------------------------
// State control
// --------------------------------------------------------------------------
void led_set_state(led_state_t new_state)
{
    if (led_mutex && xSemaphoreTake(led_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        current_led_state = new_state;
        xSemaphoreGive(led_mutex);
    }
}

static led_state_t led_get_state(void)
{
    led_state_t state;
    if (led_mutex && xSemaphoreTake(led_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        state = current_led_state;
        xSemaphoreGive(led_mutex);
    } else {
        state = current_led_state; // fallback
    }
    return state;
}

// --------------------------------------------------------------------------
// LED helper (turn all off)
// --------------------------------------------------------------------------
static inline void led_all_off(void)
{
    gpio_set_level(HEART_BEAT_RED, HIGH);
    gpio_set_level(HEART_BEAT_GREEN, HIGH);
    gpio_set_level(HEART_BEAT_BLUE, HIGH);
}

// --------------------------------------------------------------------------
// Heartbeat / status task
// --------------------------------------------------------------------------
void led_task(void *arg)
{
    while (1) {
        switch (current_led_state) {
            case LED_IDLE:
                gpio_set_level(HEART_BEAT_BLUE, LOW);
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(HEART_BEAT_BLUE, HIGH);
                vTaskDelay(pdMS_TO_TICKS(1250));
                break;

            case LED_PAIRING:
                gpio_set_level(HEART_BEAT_RED, LOW);
                gpio_set_level(HEART_BEAT_GREEN, LOW); // Amber
                vTaskDelay(pdMS_TO_TICKS(300));
                gpio_set_level(HEART_BEAT_RED, HIGH);
                gpio_set_level(HEART_BEAT_GREEN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(300));
                break;

            case LED_CONNECTED:
                gpio_set_level(HEART_BEAT_GREEN, LOW);
                vTaskDelay(pdMS_TO_TICKS(400));
                gpio_set_level(HEART_BEAT_GREEN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case LED_ERROR:
                gpio_set_level(HEART_BEAT_RED, LOW);
                vTaskDelay(pdMS_TO_TICKS(800));
                gpio_set_level(HEART_BEAT_RED, HIGH);
                vTaskDelay(pdMS_TO_TICKS(800));
                break;
        }
    }
}




