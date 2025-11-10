#ifndef XBOX_DESCRIPTORS_H
#define XBOX_DESCRIPTORS_H

#include <stdint.h>

#define XBOX_VENDOR_ID 0x045E

// Product: Xbox Wireless controller
#define XBOX_1708_ALT_PRODUCT_ID 0x02e0 

// Product: Xbox One S controller (supports linux kernel < 6.5)
// Menu/select button replaces share button
#define XBOX_1708_PRODUCT_ID 0x02fd
#define XBOX_1708_BCD_DEVICE_ID 0x0408
#define XBOX_1708_SERIAL "3033363030343037323136373239"

#define XBOX_INPUT_REPORT_ID 0x01
#define XBOX_EXTRA_INPUT_REPORT_ID 0x02
#define XBOX_OUTPUT_REPORT_ID 0x03
#define XBOX_EXTRA_OUTPUT_REPORT_ID 0x04

// Xbox One S Bluetooth HID descriptor (Model 1708)
static const uint8_t XboxOneS_1708_HIDDescriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Gamepad)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)
    0x09, 0x35,       //   Usage (Rz)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x04,       //   Report Count (4)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (Button 1)
    0x29, 0x0A,       //   Usage Maximum (Button 10)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x0A,       //   Report Count (10)
    0x81, 0x02,       //   Input (Data,Var,Abs)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const,Var,Abs)
    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x02,
    0xC0              // End Collection
};

#endif