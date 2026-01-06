#ifndef USB_DEMO_DESCRIPTORS_H
#define USB_DEMO_DESCRIPTORS_H

#define USB_DEVICE_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W (5U)


#define USB_CONFIG_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W (3U)
#define USB_CONFIG_DESCRIPTOR_SIZE_B (9U)


// 17 half-words (16-bit)
// or 9 words (32-bit)
// or 34 bytes
#define USB_CONFIG_COMBINED_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W (9U)
#define USB_CONFIG_COMBINED_DESCRIPTOR_SIZE_B (34U)


// bonus means send shift + 'q', and not just 'q'
#define HID_REPORT_DESCRIPTOR_KEYBOARD_BONUS_SIZE_HW (20U)
// wDescriptorLength_L = 0x27 (HID descriptor)
#define HID_REPORT_DESCRIPTOR_KEYBOARD_BONUS_SIZE_B (39U)

#define HID_REPORT_DESCRIPTOR_KEYBOARD_SIZE_W (10U)

#define HID_REPORT_DESCRIPTOR_KEYBOARD_SIZE_B (40U)



// ****************************************************************************
//
//                    Gamepad Demo Descriptor Sizes
//
// ****************************************************************************
// Device Descriptor
#define USB_DEVICE_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W (5U)
#define USB_DEVICE_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B (18U)

// Configuration Descriptor
#define USB_CONFIGURATION_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W (3U)
#define USB_CONFIGURATION_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B (9U)

// Configuration, Interface, HID, Endpoint (combined) Descriptors
#define USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W (9U)
#define USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B (34U)

// HID Report Descriptor
// without any buttons
// #define HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W (6)
// #define HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B (21)
// with 1 button
#define HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W (12)
#define HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_B (45)


#endif