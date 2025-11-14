#set(COMPONENT_SRCS "core_bt_gamecube.c" "mitch_i2c.c" "switch_haptics.c" "interval.c" "core_bt_ds4.c" "switch_analog.c" "switch_spi.c" "switch_commands.c" "core_bt_switch.c" "main.c")
#set(COMPONENT_ADD_INCLUDEDIRS "")

#register_component()

idf_component_register(
    SRCS
        "main.c"
        "mitch_i2c.c"
        "core_bt_switch.c"
        "core_bt_sinput.c"
        "core_bt_dinput.c"
        "esp_hid_gap.c"
        "util_bt_hid.c"
        "switch_analog.c"
        "switch_commands.c"
        "switch_spi.c"
        "switch_haptics.c"
        "imu_tool.c"
        "interval.c"
        "LED.c"
        "drv2605_esp.c"
        #"bt_serial_logger.c"
    INCLUDE_DIRS
        "."
        "include"
        "include/switch"
    REQUIRES
        bt                # Classic Bluetooth + SPP now enabled
        esp_hid           # HID stack (used by Switch mode)
        esp_driver_ledc   # LED PWM driver
        driver            # I2C, SPI, GPIO, DAC, etc.
        nvs_flash         # NVS storage
        esp_adc           # Only if hoja_includes.h still pulls in ADC header
)

