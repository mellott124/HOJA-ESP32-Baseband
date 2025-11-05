#include "drv2605_esp.h"

static const char *TAG = "DRV2605";

esp_err_t drv2605_write(uint8_t reg, uint8_t val){
    uint8_t data[2] = { reg, val };
    return i2c_master_write_to_device(DRV2605_I2C_PORT, DRV2605_I2C_ADDR,
                                      data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t drv2605_read(uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(DRV2605_I2C_PORT, DRV2605_I2C_ADDR,
                                        &reg, 1, val, 1, pdMS_TO_TICKS(100));
}

static esp_err_t drv2605_i2c_init(void) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DRV2605_SDA_PIN,
        .scl_io_num = DRV2605_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = DRV2605_I2C_SPEED_HZ
    };
    esp_err_t err = i2c_param_config(DRV2605_I2C_PORT, &cfg);
    if (err != ESP_OK) return err;
    return i2c_driver_install(DRV2605_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

void drv2605_gpio_enable_r(bool on) {
    gpio_config_t io = { .pin_bit_mask = 1ULL << DRV2605_GPIO_R,
                         .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0,
                         .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&io);
    gpio_set_level(DRV2605_GPIO_R, on ? 1 : 0);
    ESP_LOGI(TAG, "GPIO_R (pin %d) -> %s [stub]", DRV2605_GPIO_R, on ? "ON" : "OFF");
}

void drv2605_gpio_enable_l(bool on) {
#if DRV2605_GPIO_L >= 0
    gpio_config_t io = { .pin_bit_mask = 1ULL << DRV2605_GPIO_L,
                         .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0,
                         .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&io);
    gpio_set_level(DRV2605_GPIO_L, on ? 1 : 0);
#else
    ESP_LOGI(TAG, "GPIO_L stubbed (future DRV2625)");
#endif
}

esp_err_t drv2605_init(void) {
    ESP_LOGI(TAG, "Initializing I2C bus...");
    esp_err_t err = drv2605_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Resetting DRV2605 sequence...");
    drv2605_write(DRV2605_REG_MODE, DRV2605_MODE_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(10));
    drv2605_write(DRV2605_REG_MODE, DRV2605_MODE_INT_TRIG);
    drv2605_write(DRV2605_REG_LIBRARY_SEL, 1); // default library
    drv2605_write(DRV2605_REG_WAVESEQ1, 1);   // simple waveform
    drv2605_write(DRV2605_REG_GO, 0);         // stop
    drv2605_write(DRV2605_REG_RTPIN, 0x00);   // clear strength
    drv2605_write(DRV2605_REG_MODE, DRV2605_MODE_RTP);

	// Switch to LRA mode
	drv2605_write(DRV2605_REG_FEEDBACK, 0xB6);
	drv2605_write(DRV2605_REG_CONTROL1, 0x93);
	drv2605_write(DRV2605_REG_CONTROL3, 0xA3);

	uint8_t status = 0;
	esp_err_t detect = drv2605_read(DRV2605_REG_STATUS, &status);
	if (detect != ESP_OK) {
		ESP_LOGW(TAG, "DRV2605 not detected on I2C bus (no ACK). Disabling haptics.");
		return ESP_OK;
	}
	ESP_LOGI(TAG, "DRV2605 detected, status=0x%02X", status);

	ESP_LOGI(TAG, "DRV2605 initialized (RTP mode, LRA)");
	return ESP_OK;
}

esp_err_t drv2605_set_mode(uint8_t mode) {
    esp_err_t err = drv2605_write(DRV2605_REG_MODE, mode);
    ESP_LOGI(TAG, "Set mode 0x%02X -> %s", mode, esp_err_to_name(err));
    return err;
}

esp_err_t drv2605_set_rtp(uint8_t value) {
    esp_err_t err = drv2605_write(DRV2605_REG_RTPIN, value);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Set RTP strength=%d", value);
    else
        ESP_LOGE(TAG, "RTP write failed: %s", esp_err_to_name(err));
    return err;
}

esp_err_t drv2605_stop(void) {
    esp_err_t err = drv2605_write(DRV2605_REG_GO, 0x00);
    ESP_LOGI(TAG, "Stop vibration -> %s", esp_err_to_name(err));
    return err;
}
