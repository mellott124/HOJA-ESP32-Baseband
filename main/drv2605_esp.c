#include "drv2605_esp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DRV2625";

static bool s_i2c_ready = false;
static bool s_gpio_ready = false;
static bool s_available[2] = { false, false };
static drv2625_channel_t s_active_channel = DRV2625_CH_LEFT;

static drv2625_test_mode_t s_test_mode[2] = {
    DRV2625_TEST_MODE_CLOSED_LOOP_RTP,
    DRV2625_TEST_MODE_CLOSED_LOOP_RTP
};

static uint8_t s_od_clamp[2] = { 0x50, 0x50 };

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

static esp_err_t drv2625_write_ol_lra_period(uint16_t period)
{
    esp_err_t err;

    err = drv2625_write_reg(DRV2625_REG_OL_LRA_PERIOD_H, (uint8_t)((period >> 8) & 0x03));
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_OL_LRA_PERIOD_L, (uint8_t)(period & 0xFF));
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t drv2625_config_closed_loop_rtp(uint8_t rated_voltage, uint8_t od_clamp)
{
    esp_err_t err;

    err = drv2625_write_reg(DRV2625_REG_RATED_VOLTAGE, rated_voltage);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_OD_CLAMP, od_clamp);
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

    err = drv2625_write_reg(DRV2625_REG_GO, DRV2625_STOP);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_RTP_INPUT, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t drv2625_config_open_loop_rtp(uint16_t ol_period,
                                              bool sine_wave,
                                              uint8_t od_clamp)
{
    esp_err_t err;
    uint8_t actuator_cfg;
    uint8_t ol_ctrl;

    actuator_cfg = DRV2625_ACTUATOR_LRA |
                   DRV2625_LOOP_OPEN |
                   DRV2625_HYBRID_LOOP_OFF |
                   DRV2625_AUTO_BRK_OL_OFF |
                   DRV2625_AUTO_BRK_STBY_ON |
                   DRV2625_INPUT_SLOPE_OFF;

    ol_ctrl = sine_wave ? DRV2625_OL_CONTROL_SINE : DRV2625_OL_CONTROL_SQUARE;

    err = drv2625_write_reg(DRV2625_REG_OD_CLAMP, od_clamp);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_MODE, DRV2625_MODECFG_RTP_I2C);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_ACTUATOR, actuator_cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_OL_CONTROL, ol_ctrl);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_ol_lra_period(ol_period);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_GO, DRV2625_STOP);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_write_reg(DRV2625_REG_RTP_INPUT, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

static esp_err_t drv2625_apply_channel_mode(drv2625_channel_t ch)
{
    switch (s_test_mode[ch])
    {
        case DRV2625_TEST_MODE_CLOSED_LOOP_RTP:
            return drv2625_config_closed_loop_rtp(0x3F, s_od_clamp[ch]);

        case DRV2625_TEST_MODE_OL_LRA_160_SQUARE:
            return drv2625_config_open_loop_rtp(254, false, s_od_clamp[ch]);

        case DRV2625_TEST_MODE_OL_LRA_320_SQUARE:
            return drv2625_config_open_loop_rtp(127, false, s_od_clamp[ch]);

        case DRV2625_TEST_MODE_OL_LRA_160_SINE:
            return drv2625_config_open_loop_rtp(254, true, s_od_clamp[ch]);

        case DRV2625_TEST_MODE_OL_LRA_320_SINE:
            return drv2625_config_open_loop_rtp(127, true, s_od_clamp[ch]);

        default:
            return ESP_ERR_INVALID_ARG;
    }
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

    s_test_mode[ch] = DRV2625_TEST_MODE_CLOSED_LOOP_RTP;
    s_od_clamp[ch] = 0x50;

    err = drv2625_apply_channel_mode(ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s initial mode apply failed: %s",
                 drv2625_ch_name(ch), esp_err_to_name(err));
        s_available[ch] = false;
        return err;
    }

    s_available[ch] = true;

    ESP_LOGI(TAG, "%s DRV2625 initialized: ID=0x%02X STATUS=0x%02X",
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

esp_err_t drv2625_set_test_mode_channel(drv2625_channel_t ch,
                                        drv2625_test_mode_t mode,
                                        uint8_t od_clamp)
{
    esp_err_t err;

    s_test_mode[ch] = mode;
    s_od_clamp[ch] = od_clamp;

    err = drv2625_select(ch);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_apply_channel_mode(ch);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "%s test mode set to %d, OD_CLAMP=0x%02X",
             drv2625_ch_name(ch), (int)mode, od_clamp);

    return ESP_OK;
}

esp_err_t drv2625_test_pulse3_channel(drv2625_channel_t ch,
                                      drv2625_test_mode_t mode,
                                      uint8_t od_clamp,
                                      uint32_t pulse_on_ms,
                                      uint32_t pulse_off_ms)
{
    esp_err_t err;
    int i;

    err = drv2625_set_test_mode_channel(ch, mode, od_clamp);
    if (err != ESP_OK) {
        return err;
    }

    for (i = 0; i < 3; i++)
    {
        err = drv2625_set_rtp_channel(ch, 127);
        if (err != ESP_OK) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(pulse_on_ms));

        err = drv2625_set_rtp_channel(ch, 0);
        if (err != ESP_OK) {
            return err;
        }

        if (i < 2) {
            vTaskDelay(pdMS_TO_TICKS(pulse_off_ms));
        }
    }

    return ESP_OK;
}

esp_err_t drv2625_test_pulse3_both(drv2625_test_mode_t mode,
                                   uint8_t od_clamp,
                                   uint32_t pulse_on_ms,
                                   uint32_t pulse_off_ms)
{
    esp_err_t err;

    err = drv2625_test_pulse3_channel(DRV2625_CH_LEFT, mode, od_clamp, pulse_on_ms, pulse_off_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_test_pulse3_channel(DRV2625_CH_RIGHT, mode, od_clamp, pulse_on_ms, pulse_off_ms);
    if (err != ESP_OK) {
        return err;
    }

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
    }

    err = drv2625_select(ch);
    if (err != ESP_OK) {
        return err;
    }

    err = drv2625_apply_channel_mode(ch);
    if (err != ESP_OK) {
        return err;
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

    err = drv2625_apply_channel_mode(ch);
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