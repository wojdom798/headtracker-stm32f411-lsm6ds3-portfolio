#include <stdint.h>
#include "usbDemoDescriptors.h"


// 1. Device Descriptor
uint32_t usbDeviceDescriptor[USB_DEVICE_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W] = {
    // bDescriptorType (MSB), bLength (LSB)
    // bcdUSB_H (MSB), bcdUSB_L (LSB)
    0x02000112, 
    
    // bDeviceSubClass (MSB), bDeviceClass (LSB)
    // bMaxPacketSize0 (MSB), bDeviceProtocol (LSB)
    0x40000000,
   
    // idVendor_H (MSB), idVendor_L (LSB)
    // idProduct_H (MSB), idProduct_L (LSB)
    // 0x33732323,
    0xF4113369,
    
    // bcdDevice_H (MSB), bcdDevice_L (LSB)
    // iProduct (MSB), iManufacturer (LSB)
    0x00000001, 
    
    0x00000100  // bNumConfigurations (MSB), iSerialNumber (LSB)
};


uint32_t usbConfigurationDescriptor[USB_CONFIG_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W] = {
    // 1. Configuration Descriptor
    // bDescriptorType (MSB), bLength (LSB)
    // wTotalLength (MSB), wTotalLength (LSB)
    0x00220209,
    
    // bConfigurationValue (MSB), bNumInterfaces (LSB)
    // bmAttributes (MSB), iConfiguration (LSB)
    0xC0006901,

    //bMaxPower (LSB)
    0x00000096
};


uint32_t usbCombinedConfigDescriptors[USB_CONFIG_COMBINED_DESCRIPTOR_KEYBOARD_DEMO_SIZE_W] = {
    // 1. Configuration Descriptor

    // bDescriptorType (MSB), bLength (LSB)
    // wTotalLength (MSB), wTotalLength (LSB)
    0x00220209, 
    
    // bConfigurationValue (MSB), bNumInterfaces (LSB)
    // bmAttributes (MSB), iConfiguration (LSB)
    0xC0006901,


    // 2. Interface Descriptor

    // Interface Descriptor (MSB), Configuration Descriptor (LSB)
    // bLength (MSB), bMaxPower (LSB)
    // bInterfaceNumber (MSB), bDescriptorType (LSB)
    0x00040996,
    
    // bNumEndpoints (MSB), bAlternateSetting (LSB)
    // bInterfaceSubClass (MSB), bInterfaceClass (LSB)
    0x00030100,
    
    

    // 3. HID Descriptor

    // iInterface (MSB), bInterfaceProtocol (LSB)
    // bDescriptorType (MSB), bLength (LSB)
    0x21090001,
    
    // bcdHID (MSB), bcdHID (LSB)
    // bNumDescriptors (MSB), bCountryCode (LSB)
    0x01000111, 


//    0x1722, // wDescriptorLength_L (MSB), bDescriptorType (LSB) // wDescriptorLength_L = 0x17 - lowercase q example
    //0x, // wDescriptorLength_L (MSB), bDescriptorType (LSB) // uppercase Q bonus example

    // Endpoint Descriptor (MSB), HID Descriptor (LSB)
    // bLength (MSB), wDescriptorLength_H (LSB)
    0x07002722, 

    // 4. Endpoint Descriptor

    // bEndpointAddress (MSB), bDescriptorType (LSB)
    // wMaxPacketSize_L (MSB), bmAttributes (LSB)
    0x40038105, 

    // bInterval (MSB), wMaxPacketSize_H (LSB)
    0x00006400, 
};


uint32_t myKeyboardHidReportDescriptor[HID_REPORT_DESCRIPTOR_KEYBOARD_SIZE_W] = {
    
    // Usage Page (Generic Desktop);
    // Usage (Keyboard);
    0x06090105,
    
    // Collection (Application);
    // "modiefier" key input (shitf, ctrl, etc.)
    0x070501A1, // Usage Page (Key Codes);
    
    // Usage Minimum (224);
    // Usage Maximum (231);
    0xE729E019,
    
    // Logical Minimum (0);
    // Logical Maximum (1);
    0x01250015,

    // Report Count; (8);
    // Report Size; (1);
    0x01750895,

    // Input (Data, Variable);
    // "letter" key input
    0x07050281, // Usage Page (Key Codes);
    
    // Usage Minimum (0);
    // Usage Maximum (101);
    0x65290019,
    
    // Logical Minimum (0);
    // Logical Maximum (101);
    0x65250015,
    
    // Report Count; (1);
    // Report Size; (8);
    0x08750195,
    
    // Input (Data, Array);
    // End Collection;
    0x00C00081
};


// LSB = modifier key (set to shift only)
// MSB = character key (q)
// no button pressed: 0x0000
// shift + q: 0x1402
// keyboard Usage ID for LeftShift is 0xE1
// hidReportBonus_LSB[0] = 0xE0 (1=on, 0=off)
// hidReportBonus_LSB[1] = 0xE1 (1=on, 0=off)
// ...
uint16_t hidReportBonus = 0x0000; // no button pressed

uint8_t shouldToggleHidReportActive = 0;
uint8_t isSendingHidReportActive = 0;

// uint8_t shouldEnableEndpoint1 = 0;
uint8_t wasHidReportSent = 0; // check if was sent after button press



// ****************************************************************************
//
//                           Gamepad Demo Descriptors
//
// ****************************************************************************
uint32_t deviceDescriptorGamepadDemo[USB_DEVICE_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W] = {
    0x02000112,
    0x40000000,
    0x337CF411,
    0x00000001,
    0x00000100
};

uint32_t configurationDescriptorGamepadDemo[USB_CONFIGURATION_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W] = {
    0x00220209,
    0xC0006901,
    0x00000096
};

// no buttons
// uint32_t configurationDescriptorCombinedGamepadDemo[USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W] = {
//     // Configuration Descriptor
//     0x00220209,
//     0xC0006901,
//     // Configuration + Interface Descriptors
//     0x00040996,
//     // Interface Descriptor
//     0x00030100,
//     // Interface + HID Descriptors
//     0x21090000,
//     // HID Descriptor
//     0x01000111,
//     // HID + Endpoint Descriptors
//     0x07001522,
//     // Endpoint Descriptor
//     0x40038105,
//     0x00006400
// };

// 1 button
uint32_t configurationDescriptorCombinedGamepadDemo[USB_CONFIG_COMBINED_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W] = {
    // Configuration Descriptor
    0x00220209,
    0xC0006901,
    // Configuration + Interface Descriptors
    0x00040996,
    // Interface Descriptor
    0x00030100,
    // Interface + HID Descriptors
    0x21090000,
    // HID Descriptor
    0x01000111,
    // HID + Endpoint Descriptors
    0x07002D22,
    // Endpoint Descriptor
    0x40038105,
    0x00006400
};


// without any buttons
// uint32_t hidReportDescriptorGamepadDemo[HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W] = {
//     0x05090105,
//     0x300901A1,
//     0x81153109,
//     0x08757F25,
//     0x02810295,
//     0x000000C0
// };

// with 1 button
uint32_t hidReportDescriptorGamepadDemo[HID_REPORT_DESCRIPTOR_GAMEPAD_DEMO_SIZE_W] = {
    0x05090105,
    0x090501A1,
    0x01290119,
    0x01250015,
    0x01750195,
    0x01950281,
    0x03810775,
    0x30090105,
    0x81153109,
    0x08757F25,
    0x02810295,
    0x000000C0,
};