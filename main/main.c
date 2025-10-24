// --------------------------------------------------------------------------
// main.c — Virtual Boy → Nintendo Switch (ESP32) Firmware Entry
// --------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "hoja.h"
#include "core_bt_switch.h"
#include "esp_gap_bt_api.h"
#include "hoja_includes.h"
#include "hoja_types.h"
#include "LED.h"

#define TAG "MAIN"
#define HIGH 1
#define LOW  0

// --------------------------------------------------------------------------
// GPIO MAPPING (Virtual Boy layout)
// --------------------------------------------------------------------------
#define GPIO_BTN_A          GPIO_NUM_35
#define GPIO_BTN_B          GPIO_NUM_32
#define GPIO_BTN_DPAD_U     GPIO_NUM_14 
#define GPIO_BTN_DPAD_L     GPIO_NUM_7
#define GPIO_BTN_DPAD_D     GPIO_NUM_8
#define GPIO_BTN_DPAD_R     GPIO_NUM_13
#define GPIO_BTN_C_U        GPIO_NUM_34 
#define GPIO_BTN_C_L        GPIO_NUM_33
#define GPIO_BTN_C_D        GPIO_NUM_38
#define GPIO_BTN_C_R        GPIO_NUM_39
#define GPIO_BTN_L          GPIO_NUM_5
#define GPIO_BTN_R          GPIO_NUM_37
#define GPIO_BTN_START      GPIO_NUM_20
#define GPIO_BTN_SELECT     GPIO_NUM_4
#define GPIO_BTN_SYNC       GPIO_NUM_2   // active-low SYNC button

// --------------------------------------------------------------------------
// APP GLOBAL STATE
// --------------------------------------------------------------------------
static bool bt_connected=false, bt_pairing=false, bt_error=false;
hoja_settings_s global_loaded_settings={.magic=HOJA_MAGIC_NUM};
hoja_live_s global_live_data={0};

static volatile uint64_t _app_report_timer_us = 16000;
static volatile uint64_t _app_report_timer_us_default = 16000;
static volatile bool _sniff = true;

uint8_t _i2c_buffer_in[32];


// --------------------------------------------------------------------------
// NVS: SAVE / LOAD APPLICATION SETTINGS
// --------------------------------------------------------------------------
void app_settings_save(void)
{
    nvs_handle_t h;
    if(nvs_open("hoja",NVS_READWRITE,&h)!=ESP_OK){
        ESP_LOGE(TAG,"NVS open fail (save)");
        return;
    }
    nvs_set_blob(h,"settings",&global_loaded_settings,sizeof(global_loaded_settings));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG,"Settings saved");
}

void app_settings_load(void)
{
    nvs_handle_t h;
    esp_err_t e=nvs_open("hoja",NVS_READWRITE,&h);
    if(e!=ESP_OK){
        ESP_LOGW(TAG,"NVS open fail (load)");
        memset(&global_loaded_settings,0,sizeof(global_loaded_settings));
        global_loaded_settings.magic=HOJA_MAGIC_NUM;
        return;
    }
    size_t len=sizeof(global_loaded_settings);
    e=nvs_get_blob(h,"settings",&global_loaded_settings,&len);
    if(e!=ESP_OK){
        ESP_LOGW(TAG,"No settings in NVS");
        memset(&global_loaded_settings,0,sizeof(global_loaded_settings));
        global_loaded_settings.magic=HOJA_MAGIC_NUM;
    }
    nvs_close(h);

    if(global_loaded_settings.magic != HOJA_MAGIC_NUM) {
        ESP_LOGW(TAG,"Settings invalid — restoring defaults");
        memset(&global_loaded_settings,0,sizeof(global_loaded_settings));
        global_loaded_settings.magic=HOJA_MAGIC_NUM;
        app_settings_save();
    }
}

// --------------------------------------------------------------------------
// ENSURE VALID BLUETOOTH MAC ADDRESS
// --------------------------------------------------------------------------
static void sanitize_mac(uint8_t *mac)
{
    mac[0] &= 0xFC;
    bool allzero = true;
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0x00) { allzero = false; break; }
    }
    if (allzero) {
        ESP_LOGW(TAG, "MAC all zeros — using hardware MAC instead");
        esp_read_mac(mac, ESP_MAC_BT);
    }
}

// --------------------------------------------------------------------------
// GPIO INITIALIZATION
// --------------------------------------------------------------------------
static void gpio_input_init(void)
{
    gpio_config_t c={0};
    uint64_t inputs_mask=
        ((1ULL<<GPIO_BTN_A)|(1ULL<<GPIO_BTN_B)|(1ULL<<GPIO_BTN_DPAD_U)|(1ULL<<GPIO_BTN_DPAD_D)|
         (1ULL<<GPIO_BTN_DPAD_L)|(1ULL<<GPIO_BTN_DPAD_R)|(1ULL<<GPIO_BTN_L)|(1ULL<<GPIO_BTN_R)|
         (1ULL<<GPIO_BTN_START)|(1ULL<<GPIO_BTN_SELECT)|(1ULL<<GPIO_BTN_C_U)|(1ULL<<GPIO_BTN_C_D)|
         (1ULL<<GPIO_BTN_C_L)|(1ULL<<GPIO_BTN_C_R));

    c.pin_bit_mask=inputs_mask;
    c.mode=GPIO_MODE_INPUT;
    gpio_config(&c);

    c.pin_bit_mask=(1ULL<<GPIO_BTN_SYNC);
    c.mode=GPIO_MODE_INPUT;
    c.pull_up_en=GPIO_PULLUP_ENABLE;
    gpio_config(&c);

}

// --------------------------------------------------------------------------
// RESTART BLUETOOTH PAIRING
// --------------------------------------------------------------------------
/* static void restart_bluetooth_pairing(void)
{
    ESP_LOGW(TAG, "Restarting for pairing (non-destructive)...");
    bt_pairing = true; 
    bt_connected = false; 
    bt_error = false;

    esp_read_mac(global_loaded_settings.device_mac_switch, ESP_MAC_BT);
    sanitize_mac(global_loaded_settings.device_mac_switch);
    memset(global_live_data.current_mac, 0, sizeof(global_live_data.current_mac));

    led_set_state(LED_PAIRING);

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
} */

static void restart_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing paired host...");
    bt_pairing = true; 
    bt_connected = false; 
    bt_error = false;

    memset(global_loaded_settings.paired_host_switch_mac, 0, 6);
    app_settings_save();

    led_set_state(LED_ERROR);  // Red flash for reset
    vTaskDelay(pdMS_TO_TICKS(400));
    led_set_state(LED_PAIRING); // Amber after clear

    esp_restart();
}

// --------------------------------------------------------------------------
// CONTROLLER INPUT TASK (reads GPIO + handles SYNC button + LEDs)
// --------------------------------------------------------------------------
static void controller_task(void* arg)
{
    bool sync_prev_high = true;
    int64_t sync_t0 = 0;

    ESP_LOGI(TAG, "Monitoring SYNC pin (GPIO%d)", GPIO_BTN_SYNC);

    i2cinput_input_s input = {0};

    // Set initial LED to IDLE (slow blue pulse)
    led_set_state(LED_IDLE);

    while (true)
    {
        // ------------------------------------------------------------------
        // SYNC BUTTON — Match Official Nintendo Behavior
        // ------------------------------------------------------------------
        bool level_high = gpio_get_level(GPIO_BTN_SYNC);
        int64_t now = esp_timer_get_time();

        // Detect press
        if (sync_prev_high && !level_high) {
            sync_t0 = now;
            ESP_LOGI(TAG, "SYNC pressed (active low)");
        }

        // Detect release
        if (!sync_prev_high && level_high) {
            int64_t held_ms = (now - sync_t0) / 1000;
            ESP_LOGI(TAG, "SYNC released after %lld ms", held_ms);

            if (held_ms >= 2000) {
                // --- Long press (2+ sec): Factory reset pairing ---
                ESP_LOGW(TAG, "SYNC long-press — factory reset pairing...");
                led_set_state(LED_ERROR);
                vTaskDelay(pdMS_TO_TICKS(200));
                led_set_state(LED_PAIRING);
                vTaskDelay(pdMS_TO_TICKS(200));
                restart_factory_reset();
            } else {
                // --- Short press (<2 sec): quick Joy-Con style amber pulse ---
                ESP_LOGI(TAG, "SYNC short-press — quick amber pulse (no reset)");
                led_set_state(LED_PAIRING);
                vTaskDelay(pdMS_TO_TICKS(100));   // fast flash, Joy-Con style
                led_set_state(LED_IDLE);
            }
        }

        sync_prev_high = level_high;

        // ------------------------------------------------------------------
        // MAIN CONTROLLER BUTTON READS
        // ------------------------------------------------------------------
        input.button_east      = !gpio_get_level(GPIO_BTN_A);     // VB A  → Switch A
        input.button_south     = !gpio_get_level(GPIO_BTN_B);     // VB B  → Switch B

        input.dpad_up          = !gpio_get_level(GPIO_BTN_DPAD_U);
        input.dpad_down        = !gpio_get_level(GPIO_BTN_DPAD_D);
        input.dpad_left        = !gpio_get_level(GPIO_BTN_DPAD_L);
        input.dpad_right       = !gpio_get_level(GPIO_BTN_DPAD_R);

        input.button_north     = !gpio_get_level(GPIO_BTN_C_L);   // VB C-Left  → Switch Y
        input.button_west      = !gpio_get_level(GPIO_BTN_C_U);   // VB C-Up    → Switch X
        input.button_minus     = !gpio_get_level(GPIO_BTN_C_R);   // VB C-Right → Switch Minus
        input.trigger_zr       = !gpio_get_level(GPIO_BTN_C_D);   // VB C-Down  → Switch ZR

        input.trigger_l        = !gpio_get_level(GPIO_BTN_L);
        input.trigger_r        = !gpio_get_level(GPIO_BTN_R);
        input.button_plus      = !gpio_get_level(GPIO_BTN_START); // VB Start   → Switch Plus

        // --- Center analog sticks (no movement yet) ---
        input.lx = 0x7FFF;
        input.ly = 0x7FFF;
        input.rx = 0x7FFF;
        input.ry = 0x7FFF;

        // ------------------------------------------------------------------
        // SELECT BUTTON + MODIFIER COMBOS (with debounce)
        // ------------------------------------------------------------------
        static int64_t last_select_press = 0;
        const int DEBOUNCE_MS = 50;

        bool sel    = !gpio_get_level(GPIO_BTN_SELECT);
        bool trig_l = !gpio_get_level(GPIO_BTN_L);
        bool trig_r = !gpio_get_level(GPIO_BTN_R);

        input.button_capture     = false;
        input.button_home        = false;
        input.button_stick_left  = false;
        input.button_stick_right = false;

        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_ms = (now_us - last_select_press) / 1000;

        if (sel && elapsed_ms > DEBOUNCE_MS) {
            last_select_press = now_us;

            if (trig_l) {
                input.button_home = true;           // Select + L → Home
                ESP_LOGI(TAG, "Select+L combo (Home)");
            } 
            else if (trig_r) {
                input.button_stick_left = true;     // Select + R → L-stick click
                ESP_LOGI(TAG, "Select+R combo (L-Stick)");
            } 
            else {
                input.button_capture = true;        // Select alone → Capture
                ESP_LOGI(TAG, "Select alone (Capture)");
            }
        }

        // ------------------------------------------------------------------
        // SEND INPUT DATA TO SWITCH
        // ------------------------------------------------------------------
        switch_bt_sendinput(&input);

        vTaskDelay(pdMS_TO_TICKS(8));  // ~125 Hz update rate
    }
}




// --------------------------------------------------------------------------
// HOJA CALLBACK STUBS
// --------------------------------------------------------------------------
void app_set_connected_status(uint8_t s){ bt_connected=(s!=0); bt_pairing=!bt_connected; bt_error=false; }
void app_set_standard_haptic(uint8_t l,uint8_t r){ (void)l; (void)r; }
void app_set_sinput_haptic(uint8_t*d,uint8_t l){ (void)d; (void)l; }
void app_set_switch_haptic(uint8_t*d){ (void)d; }
void app_set_power_setting(i2c_power_code_t p){ (void)p; }
void app_save_host_mac(input_mode_t m, uint8_t *a)
{
    (void)m;

    // Only save if MAC actually changed (to avoid NVS wear)
    if (memcmp(global_loaded_settings.paired_host_switch_mac, a, 6) != 0)
    {
        memcpy(global_loaded_settings.paired_host_switch_mac, a, 6);
        app_settings_save();
        ESP_LOGI("MAIN", "Updated paired host MAC in NVS: %02X:%02X:%02X:%02X:%02X:%02X",
                 a[0], a[1], a[2], a[3], a[4], a[5]);
    }
    else
    {
        ESP_LOGI("MAIN", "Paired host MAC unchanged, skipping NVS save.");
    }
}


bool app_compare_mac(uint8_t*a,uint8_t*b){ return memcmp(a,b,6)==0; }

uint64_t get_timestamp_ms(void){ return esp_timer_get_time()/1000ULL; }
uint64_t get_timestamp_us(void){ return esp_timer_get_time(); }
uint32_t get_timer_value(void){ return (uint32_t)esp_timer_get_time(); }

void app_set_report_timer(uint64_t t){ _app_report_timer_us_default=t; if(!_sniff)_app_report_timer_us=_app_report_timer_us_default; }
uint64_t app_get_report_timer(void){ return _app_report_timer_us; }

// --------------------------------------------------------------------------
// APP MAIN
// --------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "System start");
    uint8_t mac[6];
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    ESP_LOGI("MACCHECK", "esp_efuse_mac_get_default() returned %s", esp_err_to_name(ret));
    ESP_LOGI("MACCHECK", "Base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // --------------------------------------------------
    // 🧭 STEP 1: Initialize NVS
    // --------------------------------------------------
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // --------------------------------------------------
    // 🟦 STEP 2: Initialize settings, GPIO, and LED early
    // --------------------------------------------------
    app_settings_load();
    gpio_input_init();
    led_init();
    led_boot_sweep(); // Boot animation
    xTaskCreate(led_task, "led_task", 2048, NULL, 2, NULL);
    led_set_state(LED_IDLE);

    // --------------------------------------------------
    // 🧩 STEP 3: Log and sanitize MAC
    // --------------------------------------------------
    ESP_LOGI(TAG, "Pre-BT global_live_data.current_mac %02X:%02X:%02X:%02X:%02X:%02X",
             global_loaded_settings.device_mac_switch[0],
             global_loaded_settings.device_mac_switch[1],
             global_loaded_settings.device_mac_switch[2],
             global_loaded_settings.device_mac_switch[3],
             global_loaded_settings.device_mac_switch[4],
             global_loaded_settings.device_mac_switch[5]);

    // --------------------------------------------------
    // 🟧 STEP 4: Apply correct base MAC before Bluetooth startup
    // --------------------------------------------------
    uint8_t base_mac[6];
	esp_read_mac(base_mac, ESP_MAC_BT);
	ESP_LOGI(TAG, "Applying base BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
			 base_mac[0], base_mac[1], base_mac[2],
			 base_mac[3], base_mac[4], base_mac[5]);
	esp_base_mac_addr_set(base_mac);


    // --------------------------------------------------
    // 🟪 STEP 5: Initialize Bluetooth core
    // --------------------------------------------------
    ESP_LOGI(TAG, "Switch BT Mode Init...");
    int bt_status = core_bt_switch_start();

    if (bt_status == 1) {
        led_set_state(LED_PAIRING);  // Amber = advertising/reconnecting
    } else {
        led_set_state(LED_ERROR);    // Red = failure
    }

    // --------------------------------------------------
    // 🧾 STEP 6: Verify MAC post-start
    // --------------------------------------------------
    esp_read_mac(global_live_data.current_mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "Refreshed BT MAC after core start: %02X:%02X:%02X:%02X:%02X:%02X",
             global_live_data.current_mac[0],
             global_live_data.current_mac[1],
             global_live_data.current_mac[2],
             global_live_data.current_mac[3],
             global_live_data.current_mac[4],
             global_live_data.current_mac[5]);

    bool mac_zero = true;
    for (int i = 0; i < 6; i++) {
        if (global_live_data.current_mac[i] != 0x00) {
            mac_zero = false;
            break;
        }
    }
    if (mac_zero) {
        ESP_LOGW(TAG, "MAC still zero — refreshing from hardware again");
        esp_read_mac(global_live_data.current_mac, ESP_MAC_BT);
        ESP_LOGI(TAG, "Final MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 global_live_data.current_mac[0],
                 global_live_data.current_mac[1],
                 global_live_data.current_mac[2],
                 global_live_data.current_mac[3],
                 global_live_data.current_mac[4],
                 global_live_data.current_mac[5]);
    }

    // --------------------------------------------------
    // 🕹️ STEP 7: Launch controller input task
    // --------------------------------------------------
    bt_pairing = true;  // initial pairing mode (used by LED state machine)
    xTaskCreatePinnedToCore(controller_task, "controller_task", 4096, NULL, 1, NULL, 1);
}


