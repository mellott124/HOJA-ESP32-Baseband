#include "bt_serial_logger.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define SPP_SERVER_NAME "VBC_Logger"

static const char *TAG = "BT_SERIAL_LOGGER";
static uint32_t spp_handle = 0;
static bool spp_connected = false;
static SemaphoreHandle_t spp_mutex = NULL;
static bool spp_ready = false;

/* -------------------------------------------------------------------------- */
/*  vprintf mirroring                                                         */
/* -------------------------------------------------------------------------- */
int bt_serial_logger_vprintf(const char *fmt, va_list args)
{
    static char buffer[512];
    va_list copy;
    va_copy(copy, args);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    vprintf(fmt, copy);      // normal UART log output
    va_end(copy);

    if (spp_connected && spp_handle) {
        if (xSemaphoreTake(spp_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            esp_spp_write(spp_handle, len, (uint8_t *)buffer);
            xSemaphoreGive(spp_mutex);
        }
    }

    return len;
}

/* -------------------------------------------------------------------------- */
/*  SPP Callback                                                              */
/* -------------------------------------------------------------------------- */
static void bt_serial_logger_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized, delaying start to allow HID to pair...");
            vTaskDelay(pdMS_TO_TICKS(2000));   // allow HID to pair first
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;

        case ESP_SPP_START_EVT:
            ESP_LOGI(TAG, "SPP server started as '%s'", SPP_SERVER_NAME);
            spp_ready = true;
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            spp_handle = param->srv_open.handle;
            spp_connected = true;
            ESP_LOGI(TAG, "SPP client connected.");
            break;

        case ESP_SPP_CLOSE_EVT:
            spp_connected = false;
            spp_handle = 0;
            ESP_LOGW(TAG, "SPP connection closed. Restarting server after clean teardown...");

            esp_spp_deinit();
            spp_ready = true;

            // Wait for SPP_UNINIT_EVT to signal full teardown
            int wait_ms = 0;
            while (spp_ready && wait_ms < 2000) {
                vTaskDelay(pdMS_TO_TICKS(100));
                wait_ms += 100;
            }

            // Give Windows more time to release the COM port
            vTaskDelay(pdMS_TO_TICKS(1000));

            ESP_LOGI(TAG, "Reinitializing SPP after disconnect...");
            esp_spp_init(ESP_SPP_MODE_CB);
            break;

        case ESP_SPP_UNINIT_EVT:
            ESP_LOGI(TAG, "SPP stack uninitialized.");
            spp_ready = false;
            break;

        default:
            break;
    }
}

/* -------------------------------------------------------------------------- */
/*  Public Functions                                                          */
/* -------------------------------------------------------------------------- */
esp_err_t bt_serial_logger_init(void)
{
    esp_err_t ret;

    if (!spp_mutex)
        spp_mutex = xSemaphoreCreateMutex();

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_LOGE(TAG, "Bluedroid not enabled, skipping logger init");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Initializing Classic SPP logger...");
    ret = esp_spp_register_callback(bt_serial_logger_spp_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register SPP callback: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_spp_init(ESP_SPP_MODE_CB);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPP: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPP server registered — waiting for initialization...");
    return ESP_OK;
}

void bt_serial_logger_redirect_esp_log(void)
{
    esp_log_set_vprintf(bt_serial_logger_vprintf);
    ESP_LOGI(TAG, "ESP_LOG output mirrored to SPP + UART");
}
