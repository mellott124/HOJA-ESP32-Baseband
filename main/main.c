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
// LED DEFINES (active-low RGB LED)
// --------------------------------------------------------------------------
#define HEART_BEAT_RED      GPIO_NUM_26
#define HEART_BEAT_BLUE     GPIO_NUM_25
#define HEART_BEAT_GREEN    GPIO_NUM_27
#define GPIO_OUTPUT_LED_MASK ((1ULL<<HEART_BEAT_RED)|(1ULL<<HEART_BEAT_BLUE)|(1ULL<<HEART_BEAT_GREEN))

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

    c.pin_bit_mask=GPIO_OUTPUT_LED_MASK;
    c.mode=GPIO_MODE_OUTPUT;
    gpio_config(&c);

    gpio_set_level(HEART_BEAT_RED,LOW);   vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(HEART_BEAT_RED,HIGH);
    gpio_set_level(HEART_BEAT_GREEN,LOW); vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(HEART_BEAT_GREEN,HIGH);
    gpio_set_level(HEART_BEAT_BLUE,LOW);  vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(HEART_BEAT_BLUE,HIGH);
}

// --------------------------------------------------------------------------
// HEARTBEAT LED TASK
// --------------------------------------------------------------------------
static void heartbeat_task(void*arg)
{
    while(true){
        gpio_set_level(HEART_BEAT_RED,HIGH);
        gpio_set_level(HEART_BEAT_GREEN,HIGH);
        gpio_set_level(HEART_BEAT_BLUE,HIGH);

        if(bt_error){
            gpio_set_level(HEART_BEAT_RED,LOW);
            vTaskDelay(pdMS_TO_TICKS(800));
            gpio_set_level(HEART_BEAT_RED,HIGH);
            vTaskDelay(pdMS_TO_TICKS(800));
        } else if(bt_connected){
            gpio_set_level(HEART_BEAT_GREEN,LOW);
            vTaskDelay(pdMS_TO_TICKS(400));
            gpio_set_level(HEART_BEAT_GREEN,HIGH);
            vTaskDelay(pdMS_TO_TICKS(400));
        } else if(bt_pairing){
            gpio_set_level(HEART_BEAT_BLUE,LOW);
            vTaskDelay(pdMS_TO_TICKS(400));
            gpio_set_level(HEART_BEAT_BLUE,HIGH);
            vTaskDelay(pdMS_TO_TICKS(400));
        } else {
            gpio_set_level(HEART_BEAT_BLUE,LOW);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(HEART_BEAT_BLUE,HIGH);
            vTaskDelay(pdMS_TO_TICKS(1250));
        }
    }
}

// --------------------------------------------------------------------------
// RESTART BLUETOOTH PAIRING
// --------------------------------------------------------------------------
static void restart_bluetooth_pairing(void)
{
    ESP_LOGW(TAG,"Restarting for pairing...");
    bt_pairing=true; bt_connected=false; bt_error=false;

    memset(global_loaded_settings.paired_host_switch_mac,0,6);
    esp_read_mac(global_loaded_settings.device_mac_switch, ESP_MAC_BT);
    sanitize_mac(global_loaded_settings.device_mac_switch);
    app_settings_save();
    memset(global_live_data.current_mac,0,sizeof(global_live_data.current_mac));

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

// --------------------------------------------------------------------------
// CONTROLLER INPUT TASK (reads GPIO + handles SYNC button + LEDs)
// --------------------------------------------------------------------------
static void controller_task(void* arg)
{
    bool sync_prev_high = true;
    int64_t sync_t0 = 0;
    bool sync_fire = false, seen_rel = false;
    const int LONG_MS = 2000, ARM_MS = 3000;
    int64_t boot_t0 = esp_timer_get_time();

    ESP_LOGI(TAG, "Monitoring SYNC pin (GPIO%d)", GPIO_BTN_SYNC);

    i2cinput_input_s input = {0};

    while (true)
    {
        // --- Handle SYNC button ---
        bool level_high = gpio_get_level(GPIO_BTN_SYNC);
        int64_t now = esp_timer_get_time();
        int64_t uptime_ms = (now - boot_t0) / 1000;
        if (level_high) seen_rel = true;

        if (sync_prev_high && !level_high) {
            sync_t0 = now;
            sync_fire = false;
            ESP_LOGI(TAG, "SYNC pressed (active low)");
        }

        if (!sync_prev_high && level_high) {
            sync_t0 = 0;
            sync_fire = false;
            ESP_LOGI(TAG, "SYNC released");
            gpio_set_level(HEART_BEAT_BLUE, HIGH);
        }

        if (!level_high && sync_t0 && !sync_fire) {
            static bool blink = false; blink = !blink;
            gpio_set_level(HEART_BEAT_BLUE, blink ? LOW : HIGH);
        }

        if (!level_high && sync_t0 && (now - sync_t0) / 1000 >= LONG_MS &&
            !sync_fire && uptime_ms >= ARM_MS && seen_rel) {
            sync_fire = true;
            ESP_LOGW(TAG, "SYNC long-press — clearing pairing...");
            for (int i = 0; i < 6; i++) {
                gpio_set_level(HEART_BEAT_RED, LOW);
                gpio_set_level(HEART_BEAT_BLUE, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(HEART_BEAT_RED, HIGH);
                gpio_set_level(HEART_BEAT_BLUE, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            restart_bluetooth_pairing();
        }
        sync_prev_high = level_high;
		
		// ------------------------------N64 Mapping to Switch-------------------------------
		input.button_east      = !gpio_get_level(GPIO_BTN_A);
		input.button_south     = !gpio_get_level(GPIO_BTN_B);
		
		input.dpad_up    = !gpio_get_level(GPIO_BTN_DPAD_U);
		input.dpad_down  = !gpio_get_level(GPIO_BTN_DPAD_D);
		input.dpad_left  = !gpio_get_level(GPIO_BTN_DPAD_L);
		input.dpad_right = !gpio_get_level(GPIO_BTN_DPAD_R);

		input.button_north = !gpio_get_level(GPIO_BTN_C_L);  // VB C-Left → Switch Y
		input.button_west = !gpio_get_level(GPIO_BTN_C_U);  // VB C-Up   → Switch X
		input.trigger_zr   = !gpio_get_level(GPIO_BTN_C_D);  // VB C-Down → Switch B
		input.button_minus = !gpio_get_level(GPIO_BTN_C_R);  // VB C-Right→ Switch A


		input.trigger_l   	= !gpio_get_level(GPIO_BTN_L); 
		input.trigger_r     = !gpio_get_level(GPIO_BTN_R);

		input.button_plus   = !gpio_get_level(GPIO_BTN_START); 
		//input.button_minus  = !gpio_get_level(GPIO_BTN_SELECT);

		// --- Center analog sticks (no movement yet) ---
		input.lx = 0x7FFF;
		input.ly = 0x7FFF;
		input.rx = 0x7FFF;
		input.ry = 0x7FFF; 

		// --- Push to Bluetooth HID ---
		switch_bt_sendinput(&input);

        vTaskDelay(pdMS_TO_TICKS(8));  // ~125Hz update rate
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
void app_save_host_mac(input_mode_t m,uint8_t*a){ (void)m; memcpy(global_loaded_settings.paired_host_switch_mac,a,6); }
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
    ESP_LOGI(TAG,"System start");

    esp_err_t ret=nvs_flash_init();
    if(ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND){
        nvs_flash_erase();
        nvs_flash_init();
    }

    app_settings_load();
    gpio_input_init();

    //sanitize_mac(global_loaded_settings.device_mac_switch);
    ESP_LOGI(TAG,"Pre-BT global_live_data.current_mac %02X:%02X:%02X:%02X:%02X:%02X",
             global_loaded_settings.device_mac_switch[0],
             global_loaded_settings.device_mac_switch[1],
             global_loaded_settings.device_mac_switch[2],
             global_loaded_settings.device_mac_switch[3],
             global_loaded_settings.device_mac_switch[4],
             global_loaded_settings.device_mac_switch[5]);

    ESP_LOGI(TAG, "Switch BT Mode Init...");
    core_bt_switch_start();

    // ------------------------------------------------------------------
    // 🔧 Added: MAC fallback verification & refresh
    // ------------------------------------------------------------------
    esp_read_mac(global_live_data.current_mac, ESP_MAC_BT);
    ESP_LOGI(TAG,"Refreshed BT MAC after core start: %02X:%02X:%02X:%02X:%02X:%02X",
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
    // ------------------------------------------------------------------

    bt_pairing=true;

    xTaskCreatePinnedToCore(heartbeat_task,"heartbeat_task",2048,NULL,1,NULL,0);
    xTaskCreatePinnedToCore(controller_task,"controller_task",4096,NULL,1,NULL,1);
}
