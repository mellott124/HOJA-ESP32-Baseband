#include "switch_commands.h"
#include <math.h>
#include "hoja_includes.h"       // added
extern input_mode_t get_current_mode(void);  // added

uint8_t _switch_input_buffer[64] = {0};
uint8_t _switch_input_report_id = 0x00;

void ns_report_clear(uint8_t *buffer, uint16_t size)
{
  memset(buffer, 0, size);
}

void ns_report_setid(uint8_t report_id)
{
  _switch_input_report_id = report_id;
}

void ns_report_setack(uint8_t ack)
{
  _switch_input_buffer[12] = ack;
}

void ns_report_setsubcmd(uint8_t *buffer, uint8_t command)
{
  buffer[13] = command;
}

void ns_report_settimer(uint8_t *buffer)
{
  static uint64_t time;
  time = get_timestamp_ms();
  buffer[1] = (uint8_t)(time % 0xFF);
}

void ns_report_setbattconn(uint8_t *buffer)
{
    uint8_t battery_level = 8;  // 0–8
    uint8_t charging = 0;       // 1 if charging
    uint8_t connection = 1;     // 1 = Bluetooth
    buffer[2] = (battery_level << 4) | (charging << 3) | connection;
}

// --------------------------------------------------------------------------
// Device Info: identifies controller type to Switch
// --------------------------------------------------------------------------
void ns_report_sub_setdevinfo(uint8_t *buffer)
{
  _switch_input_buffer[14] = 0x04; // Firmware major (4.x)
  _switch_input_buffer[15] = 0x33; // Firmware minor (x.21)

  // Controller identity bytes:
  // Procon   - 0x03, 0x02
  // N64      - 0x0C, 0x11
  // SNES     - 0x0B, 0x02
  // Famicom  - 0x07, 0x02
  // NES      - 0x09, 0x02
  // Genesis  - 0x0D, 0x02
  switch (get_current_mode())
  {
      case INPUT_MODE_SWPRO:
          buffer[16] = 0x03;
          buffer[17] = 0x02;
          break;

      case INPUT_MODE_SNES:
          buffer[16] = 0x0B;
          buffer[17] = 0x02;
          break;

      case INPUT_MODE_N64:
      default:
          buffer[16] = 0x0C;
          buffer[17] = 0x11;
          break;
  }

  // Copy MAC address into 18–23
  buffer[18] = global_live_data.current_mac[0];
  buffer[19] = global_live_data.current_mac[1];
  buffer[20] = global_live_data.current_mac[2];
  buffer[21] = global_live_data.current_mac[3];
  buffer[22] = global_live_data.current_mac[4];
  buffer[23] = global_live_data.current_mac[5];

  buffer[24] = 0x00;
  buffer[25] = 0x02;
}

// --------------------------------------------------------------------------
void ns_report_sub_triggertime(uint8_t *buffer, uint16_t time_10_ms)
{
  uint8_t upper_ms = 0xFF & time_10_ms;
  uint8_t lower_ms = (0xFF00 & time_10_ms) >> 8;

  for (uint8_t i = 0; i < 14; i += 2)
  {
    buffer[14 + i] = upper_ms;
    buffer[15 + i] = lower_ms;
  }
}

// --------------------------------------------------------------------------
void ns_subcommand_handler(uint8_t subcommand, uint8_t *data, uint16_t len)
{
  uint16_t _report_len = 15;

  ns_report_clear(_switch_input_buffer, 64);
  ns_report_settimer(_switch_input_buffer);
  ns_report_setbattconn(_switch_input_buffer);
  ns_report_setinputreport_full(_switch_input_buffer);
  ns_report_setsubcmd(_switch_input_buffer, subcommand);

  printf("CMD: ");

  switch (subcommand)
  {
  case SW_CMD_SET_NFC:
    printf("Set NFC MCU:\n");
    ns_report_setack(0x80);
    break;

  case SW_CMD_ENABLE_IMU:
    printf("Enable IMU: %d\n", data[10]);
    ns_set_imu_mode(data[10]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_PAIRING:
    printf("Set pairing.\n");
    break;

  case SW_CMD_SET_INPUTMODE:
    printf("Input mode change: %X\n", data[10]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_GET_DEVICEINFO:
    printf("Get device info.\n");
    _report_len += 12;
    ns_report_setack(0x82);
    ns_report_sub_setdevinfo(_switch_input_buffer);
    break;

  case SW_CMD_SET_SHIPMODE:
    printf("Set ship mode: %X\n", data[10]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_HCI:
    printf("Set HCI %X\n", data[10]);
    switch_bt_end_task();
    app_set_power_setting(POWER_CODE_OFF);
    break;

  case SW_CMD_GET_SPI:
    printf("Read SPI. Address: %X, %X | Len: %d\n", data[11], data[10], data[14]);
    ns_report_setack(0x90);
    sw_spi_readfromaddress(_switch_input_buffer, data[11], data[10], data[14]);
    _report_len += data[14];
    break;

  case SW_CMD_SET_SPI:
    printf("Write SPI. Address: %X, %X | Len: %d\n", data[11], data[10], data[14]);
    ns_report_setack(0x80);
    break;

  case SW_CMD_GET_TRIGGERET:
    printf("Get trigger ET.\n");
    ns_report_setack(0x83);
    ns_report_sub_triggertime(_switch_input_buffer, 100);
    _report_len += 14;
    break;

  case SW_CMD_ENABLE_VIBRATE:
    printf("Enable vibration.\n");
    ns_report_setack(0x80);
    break;

  case SW_CMD_SET_PLAYER:
    ns_report_setack(0x80);
    {
      uint8_t player = data[10] & 0xF;
      uint8_t set_num = 0;
      switch (player)
      {
        case 0b1:    set_num = 1; break;
        case 0b11:   set_num = 2; break;
        case 0b111:  set_num = 3; break;
        case 0b1111: set_num = 4; break;
        case 0b1001: set_num = 5; break;
        case 0b1010: set_num = 6; break;
        case 0b1011: set_num = 7; break;
        case 0b0110: set_num = 8; break;
        default:     set_num = 1; break;
      }
      app_set_connected_status(set_num);
      printf("Set player: %d\n", set_num);
    }
    break;

  default:
    printf("Unhandled: %X\n", subcommand);
    for (uint16_t i = 0; i < len; i++) printf("%X, ", data[i]);
    printf("\n");
    ns_report_setack(0x80);
    break;
  }

  esp_bt_hid_device_send_report(
    ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
    SWITCH_BT_REPORT_SIZE, _switch_input_buffer);
}

// --------------------------------------------------------------------------
void ns_report_handler(uint8_t report_id, uint8_t *data, uint16_t len)
{
  switch (report_id)
  {
  case SW_OUT_ID_RUMBLE_CMD:
    app_set_switch_haptic(&data[1]);
    ns_subcommand_handler(data[9], data, len);
    break;

  case SW_OUT_ID_RUMBLE:
    app_set_switch_haptic(&data[1]);
    break;

  default:
    printf("Unknown report: %X\n", report_id);
    break;
  }
}

void ns_report_bulkset(uint8_t *buffer, uint8_t start_idx, uint8_t *data, uint8_t len)
{
  memcpy(&buffer[start_idx], data, len);
}
