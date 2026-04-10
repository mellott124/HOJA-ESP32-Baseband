#include "drv2605_esp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DRV2625";

static bool s_i2c_ready = false;
static bool s_gpio_ready = false;
static bool s_available[2] = { false, false };
static drv2625_channel_t s_active_channel = DRV2625_CH_LEFT;

static esp_err_t drv2625_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { reg, val };
    return i2c_master_write_to_device(
        DRV2625_I2C_PORT,
        DRV2625_I2C_ADDR,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
}

static esp_err_t drv2625_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(
        DRV2625_I2C_PORT,
        DRV2625_I2C_ADDR,
        &reg,
        1,
        val,
        1,
        pdMS_TO_TICKS(100));
}

static esp_err_t drv2625_i2c_init_once(void)
{
    if (s_i2c_ready) {
        return ESP_OK;
    }

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DRV2625_SDA_PIN,
        .scl_io_num = DRV2625_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = DRV2625_I2C_SPEED_HZ
    };

    esp_err_t err = i2c_param_config(DRV2625_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(DRV2625_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    s_i2c_ready = true;
    return ESP_OK;
}

static esp_err_t drv2625_gpio_init_once(void)
{
    if (s_gpio_ready) {
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << DRV2625_GPIO_NRST_L) |
                        (1ULL << DRV2625_GPIO_NRST_R),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(DRV2625_GPIO_NRST_L, 0);
    gpio_set_level(DRV2625_GPIO_NRST_R, 0);

    s_gpio_ready = true;
    return ESP_OK;
}

static gpio_num_t drv2625_nrst_pin(drv2625_channel_t ch)
{
    return (ch == DRV2625_CH_LEFT) ? DRV2625_GPIO_NRST_L : DRV2625_GPIO_NRST_R;
}

static const char *drv2625_ch_name(drv2625_channel_t ch)
{
    return (ch == DRV2625_CH_LEFT) ? "LEFT" : "RIGHT";
}

static void drv2625_assert_reset(drv2625_channel_t ch, bool release_reset)
{
    gpio_set_level(drv2625_nrst_pin(ch), release_reset ? 1 : 0);
}

static void drv2625_disable_both(void)
{
    drv2625_assert_reset(DRV2625_CH_LEFT, false);
    drv2625_assert_reset(DRV2625_CH_RIGHT, false);
}

void drv2605_gpio_enable_l(bool on)
{
    drv2625_gpio_init_once();
    drv2625_assert_reset(DRV2625_CH_LEFT, on);
    ESP_LOGI(TAG, "NRST_L GPIO%d -> %s", DRV2625_GPIO_NRST_L, on ? "HIGH" : "LOW");
}

void drv2605_gpio_enable_r(bool on)
{
    drv2625_gpio_init_once();
    drv2625_assert_reset(DRV2625_CH_RIGHT, on);
    ESP_LOGI(TAG, "NRST_R GPIO%d -> %s", DRV2625_GPIO_NRST_R, on ? "HIGH" : "LOW");
}

esp_err_t drv2625_select(drv2625_channel_t ch)
{
    esp_err_t err = drv2625_gpio_init_once();
    if (err != ESP_OK) {
        return err;
    }

    drv2625_disable_both();
    vTaskDelay(pdMS_TO_TICKS(2));

    drv2625_assert_reset(ch, true);
    vTaskDelay(pdMS_TO_TICKS(2));

    s_active_channel = ch;
    return ESP_OK;
}

esp_err_t drv2625_init_channel(drv2625_channel_t ch)
{
    esp_err_t err;
    uint8_t id = 0;
    uint8_t status = 0;
    uint8_t chipid = 0;

    err = drv2625_i2c_init_once();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = drv2625_select(ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Channel select failed for %s: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        return err;
    }

    err = drv2625_read_reg(DRV2625_REG_ID, &id);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s DRV2625 not detected on I2C bus", drv2625_ch_name(ch));
        s_available[ch] = false;
        return ESP_OK;
    }

    chipid = (id >> 4) & 0x0F;
    if (chipid != 0x01) {
        ESP_LOGW(TAG, "%s unexpected device ID 0x%02X", drv2625_ch_name(ch), id);
        s_available[ch] = false;
        return ESP_ERR_NOT_FOUND;
    }

    err = drv2625_read_reg(DRV2625_REG_STATUS, &status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s status read failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
    }

    err = drv2625_write_reg(DRV2625_REG_MODE, DRV2625_MODECFG_RTP_I2C);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s mode write failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        s_available[ch] = false;
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_ACTUATOR, DRV2625_ACT_CFG_LRA_CLOSED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s actuator cfg write failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        s_available[ch] = false;
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_RTP_INPUT, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s RTP init write failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        s_available[ch] = false;
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_GO, DRV2625_STOP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s GO init write failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        s_available[ch] = false;
        return err;
    }

    s_available[ch] = true;

    ESP_LOGI(TAG, "%s DRV2625 initialized: ID=0x%02X STATUS=0x%02X (LRA, closed-loop, RTP)",
             drv2625_ch_name(ch), id, status);

    return ESP_OK;
}

esp_err_t drv2625_init(void)
{
    esp_err_t err;

    err = drv2625_gpio_init_once();
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_i2c_init_once();
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_init_channel(DRV2625_CH_LEFT);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_init_channel(DRV2625_CH_RIGHT);
    if (err != ESP_OK) {
        return err;
    }

    drv2625_disable_both();

    ESP_LOGI(TAG, "DRV2625 init complete. LEFT=%s RIGHT=%s",
             s_available[DRV2625_CH_LEFT] ? "OK" : "MISSING",
             s_available[DRV2625_CH_RIGHT] ? "OK" : "MISSING");

    return ESP_OK;
}

esp_err_t drv2625_set_rtp_channel(drv2625_channel_t ch, int8_t value)
{
    esp_err_t err;

    if (!s_available[ch]) {
        err = drv2625_init_channel(ch);
        if (err != ESP_OK) {
            return err;
        }
        if (!s_available[ch]) {
            return ESP_ERR_INVALID_STATE;
        }
    } else {
        err = drv2625_select(ch);
        if (err != ESP_OK) {
            return err;
        }

        err = drv2625_write_reg(DRV2625_REG_MODE, DRV2625_MODECFG_RTP_I2C);
        if (err != ESP_OK) {
            return err;
        }

        err = drv2625_write_reg(DRV2625_REG_ACTUATOR, DRV2625_ACT_CFG_LRA_CLOSED);
        if (err != ESP_OK) {
            return err;
        }
    }

    err = drv2625_write_reg(DRV2625_REG_RTP_INPUT, (uint8_t)value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s RTP write failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_GO, (value == 0) ? DRV2625_STOP : DRV2625_GO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s GO write failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

esp_err_t drv2625_stop_channel(drv2625_channel_t ch)
{
    esp_err_t err;
    uint8_t dummy_status = 0;

    err = drv2625_select(ch);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_RTP_INPUT, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_GO, DRV2625_STOP);
    if (err != ESP_OK) {
        return err;
    }

    (void)drv2625_read_reg(DRV2625_REG_STATUS, &dummy_status);

    return ESP_OK;
}

esp_err_t drv2625_stop_all(void)
{
    esp_err_t err_l = drv2625_stop_channel(DRV2625_CH_LEFT);
    esp_err_t err_r = drv2625_stop_channel(DRV2625_CH_RIGHT);

    drv2625_disable_both();

    if (err_l != ESP_OK) {
        return err_l;
    }

    if (err_r != ESP_OK) {
        return err_r;
    }

    return ESP_OK;
}

esp_err_t drv2605_init(void)
{
    return drv2625_init();
}

esp_err_t drv2605_set_rtp(uint8_t value)
{
    return drv2625_set_rtp_channel(DRV2625_CH_LEFT, (int8_t)value);
}

esp_err_t drv2605_stop(void)
{
    return drv2625_stop_channel(DRV2625_CH_LEFT);
}