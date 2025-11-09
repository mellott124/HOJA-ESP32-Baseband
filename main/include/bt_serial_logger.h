#ifndef BT_SERIAL_LOGGER_H
#define BT_SERIAL_LOGGER_H

#include "esp_err.h"
#include "esp_log.h"

// Name used for the SPP device (modifiable in the .c)
#ifndef BT_SPP_DEVICE_NAME
#define BT_SPP_DEVICE_NAME "VBC_Logger"
#endif

// Initialize Classic SPP logger. Safe to call after Bluetooth controller + Bluedroid are up.
// Returns ESP_OK on success, or an esp_err_t on failure.
esp_err_t bt_serial_logger_init(void);

// Optionally schedule init from app_main / non-blocking context.
// If you already call bt_serial_logger_init() directly, you don't need this.
void bt_serial_logger_schedule_init(void);

// Mirror ESP_LOG output to both UART and SPP.
// Call to enable redirection (idempotent).
void bt_serial_logger_redirect_esp_log(void);

// Stop mirroring ESP_LOG output to SPP (keeps UART logging).
void bt_serial_logger_stop_redirect(void);

#endif // BT_SERIAL_LOGGER_H
