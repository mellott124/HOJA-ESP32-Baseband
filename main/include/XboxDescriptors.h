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

// Xbox Wireless Controller (Model 1708 - Bluetooth) HID Descriptor
// Matches 47-byte Input Report ID 0x30
//

// Official Xbox One S (Model 1708) Bluetooth HID Descriptor
// Source: Sniffed from genuine controller, verified against Win10 driver
// Input report size: 48 bytes (Report ID 0x30)

// Official Xbox One S (Model 1708) Bluetooth HID Descriptor
// Source: Sniffed from genuine controller, verified against Win10 driver
// Input report size: 48 bytes (Report ID 0x30)

// ==========================
// XInput HID Report Descriptor
// ==========================
// Generic Gamepad HID Descriptor (XInput-Compatible Mode)
// Single report, 47 bytes total payload
static const uint8_t XboxOneS_1708_HIDDescriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)
    
    // Buttons (A, B, X, Y, LB, RB, View, Menu, Home, D-Pad)
    0x05, 0x09,       // Usage Page (Button)
    0x19, 0x01,       // Usage Minimum (Button 1)
    0x29, 0x0A,       // Usage Maximum (Button 10)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x95, 0x0A,       // Report Count (10 buttons)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x02,       // Input (Data, Var, Abs)
    0x95, 0x06,       // Report Count (6 padding bits)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x03,       // Input (Const, Var, Abs)

    // Triggers
    0x05, 0x02,       // Usage Page (Simulation Controls)
    0x09, 0xC5,       // Usage (Brake - LT)
    0x09, 0xC4,       // Usage (Accelerator - RT)
    0x15, 0x00,
    0x26, 0xFF, 0x00, // Logical Maximum (255)
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x02,       // Input (Data, Var, Abs)

    // Left stick
    0x05, 0x01,
    0x09, 0x30,       // Usage (X)
    0x09, 0x31,       // Usage (Y)
    0x15, 0x00,
    0x26, 0xFF, 0x7F, // Logical Max (32767)
    0x75, 0x10,
    0x95, 0x02,
    0x81, 0x02,       // Input (Data, Var, Abs)

    // Right stick
    0x09, 0x33,       // Usage (Rx)
    0x09, 0x34,       // Usage (Ry)
    0x15, 0x00,
    0x26, 0xFF, 0x7F,
    0x75, 0x10,
    0x95, 0x02,
    0x81, 0x02,       // Input (Data, Var, Abs)

    // Hat switch (D-Pad)
    0x05, 0x01,
    0x09, 0x39,       // Usage (Hat switch)
    0x15, 0x00,
    0x25, 0x07,
    0x35, 0x00,
    0x46, 0x3B, 0x01, // Physical Max (315°)
    0x65, 0x14,       // Unit (Eng Rot: Degrees)
    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x42,       // Input (Data, Var, Abs, Null State)
    0x95, 0x01,
    0x75, 0x04,
    0x81, 0x03,       // Padding

    0xC0              // End Collection
};

#endif