#include "../../Drivers/USBClient/USBClient.h"

// ATTENTION:
// 2.0 is the lowest version that works with WinUSB on Windows 8!!!
// use older values below if you do not care about that
#define DEVICE_RELEASE_VERSION              0x0200

//string descriptor
#define USB_STRING_DESCRIPTOR_SIZE          32

// index for the strings
#define MANUFACTURER_NAME_INDEX             1
#define PRODUCT_NAME_INDEX                  2
#define SERIAL_NUMBER_INDEX                 0

// configuration for extended descriptor
#define OS_DESCRIPTOR_EX_VERSION            0x0100

#define USB_DISPLAY_STRING_NUM     4
#define USB_FRIENDLY_STRING_NUM    5

#define OS_DESCRIPTOR_STRING_INDEX        0xEE
#define OS_DESCRIPTOR_STRING_VENDOR_CODE  0xA5

// USB 2.0 response structure lengths
#define USB_DEVICE_DESCRIPTOR_LENGTH             18
#define USB_CONFIGURATION_DESCRIPTOR_LENGTH       9
#define USB_STRING_DESCRIPTOR_HEADER_LENGTH       2

//XProperties Os WinUsb
#define EX_PROPERTY_DATA_TYPE__RESERVED                 0
#define EX_PROPERTY_DATA_TYPE__REG_SZ                   1
#define EX_PROPERTY_DATA_TYPE__REG_SZ_ENV               2
#define EX_PROPERTY_DATA_TYPE__REG_BINARY               3
#define EX_PROPERTY_DATA_TYPE__REG_DWORD_LITTLE_ENDIAN  4
#define EX_PROPERTY_DATA_TYPE__REG_DWORD_BIG_ENDIAN     5
#define EX_PROPERTY_DATA_TYPE__REG_LINK                 6
#define EX_PROPERTY_DATA_TYPE__REG_MULTI_SZ             7

// Configuration for extended descriptor
#define OS_DESCRIPTOR_EX_VERSION            0x0100

#define USB_ENDPOINT_ATTRIBUTE_BULK 2

#define STM32F4_USB_ENDPOINT_WRITE 1
#define STM32F4_USB_ENDPOINT_READ 2

// Device descriptor
const TinyCLR_UsbClient_DeviceDescriptor STM32F4_UsbClient_DeviceDescriptor = {
    {
        USB_DEVICE_DESCRIPTOR_MARKER,
        0,
        sizeof(TinyCLR_UsbClient_DeviceDescriptor)
    },
    USB_DEVICE_DESCRIPTOR_LENGTH,               // Length of device descriptor
    USB_DEVICE_DESCRIPTOR_TYPE,                 // USB device descriptor type
    0x0200,                                     // USB Version 2.00 (BCD) (2.0 required for Extended ID recognition)
    0,                                          // Device class (none)
    0,                                          // Device subclass (none)
    0,                                          // Device protocol (none)
    CONCAT(DEVICE_TARGET, _USB_ENDPOINT0_SIZE),   // Endpoint 0 size
    USB_DEBUGGER_VENDOR_ID,                     // Vendor ID
    USB_DEBUGGER_PRODUCT_ID,                    // Product ID
    DEVICE_RELEASE_VERSION,                     // Product version 1.00 (BCD)
    MANUFACTURER_NAME_INDEX,                    // Manufacturer name string index
    PRODUCT_NAME_INDEX,                         // Product name string index
    0,                                          // Serial number string index (none)
    1                                           // Number of configurations
};

// Configuration descriptor
const TinyCLR_UsbClient_ConfigurationDescriptor STM32F4_UsbClient_ConfigurationDescriptor = {
    {
        USB_CONFIGURATION_DESCRIPTOR_MARKER,
        0,
        sizeof(TinyCLR_UsbClient_ConfigurationDescriptor)
    },
    USB_CONFIGURATION_DESCRIPTOR_LENGTH,
    USB_CONFIGURATION_DESCRIPTOR_TYPE,
    USB_CONFIGURATION_DESCRIPTOR_LENGTH
        + sizeof(TinyCLR_UsbClient_InterfaceDescriptor)
        + sizeof(TinyCLR_UsbClient_EndpointDescriptor)
        + sizeof(TinyCLR_UsbClient_EndpointDescriptor),
    1,                                                  // Number of interfaces
    1,                                                  // Number of this configuration
    0,                                                  // Config descriptor string index (none)
    (USB_ATTRIBUTE_BASE | USB_ATTRIBUTE_SELF_POWER),    // Config attributes
    50                                                     // Device max current draw
};

// Interface Descriptor
const TinyCLR_UsbClient_InterfaceDescriptor STM32F4_UsbClient_InterfaceDescriptor = {
    sizeof(TinyCLR_UsbClient_InterfaceDescriptor),
    USB_INTERFACE_DESCRIPTOR_TYPE,
    0,                                          // Interface number
    0,                                          // Alternate number (main)
    2,                                          // Number of endpoints
    0xFF,                                       // Interface class (vendor)
    1,                                          // Interface subclass
    1,                                          // Interface protocol
    0 ,                                         // Interface descriptor string index (none)
};

// Endpoint Descriptors
const TinyCLR_UsbClient_EndpointDescriptor STM32F4_UsbClient_EndpointDescriptor[] = {
    { // Endpoint
        sizeof(TinyCLR_UsbClient_EndpointDescriptor),
        USB_ENDPOINT_DESCRIPTOR_TYPE,
        USB_ENDPOINT_DIRECTION_IN | STM32F4_USB_ENDPOINT_WRITE,
        USB_ENDPOINT_ATTRIBUTE_BULK,
        CONCAT(DEVICE_TARGET, _USB_ENDPOINT_SIZE),                                // Endpoint 1 packet size
        0                                           // Endpoint 1 interval
    },

    {
        //Endpoint
       sizeof(TinyCLR_UsbClient_EndpointDescriptor),
       USB_ENDPOINT_DESCRIPTOR_TYPE,
       USB_ENDPOINT_DIRECTION_OUT | STM32F4_USB_ENDPOINT_READ,
       USB_ENDPOINT_ATTRIBUTE_BULK,
       CONCAT(DEVICE_TARGET, _USB_ENDPOINT_SIZE),                                // Endpoint 1 packet size
       0
   }

};

const TinyCLR_UsbClient_StringDescriptorHeader STM32F4_UsbClient_StringDescriptor[] = {
    {
        {
            USB_STRING_DESCRIPTOR_MARKER,
            MANUFACTURER_NAME_INDEX,
            sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
        },
        USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
        USB_STRING_DESCRIPTOR_TYPE,
        CONCAT(L, DEVICE_MANUFACTURER)
    },

    {
        {
            USB_STRING_DESCRIPTOR_MARKER,
            PRODUCT_NAME_INDEX,
            sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
        },
        USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
        USB_STRING_DESCRIPTOR_TYPE,
        CONCAT(L, DEVICE_NAME)
    },

    {
        {
            USB_STRING_DESCRIPTOR_MARKER,
            USB_DISPLAY_STRING_NUM,
            sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
        },
        USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
        USB_STRING_DESCRIPTOR_TYPE,
        CONCAT(L, DEVICE_NAME)
    },

    {
        {
            USB_STRING_DESCRIPTOR_MARKER,
            USB_FRIENDLY_STRING_NUM,
            sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
        },
        USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
        USB_STRING_DESCRIPTOR_TYPE,
        CONCAT(L, DEVICE_NAME)
    },

    {
        {
            0,
            0,
            0
        },
        0,
        0,
        0
    }
};

// OS Descriptor string for Extended OS Compat ID
const TinyCLR_UsbClient_OsStringDescriptor STM32F4_UsbClient_OsStringDescriptor = {
    {
        USB_STRING_DESCRIPTOR_MARKER,
        OS_DESCRIPTOR_STRING_INDEX,
        sizeof(TinyCLR_UsbClient_DescriptorHeader) + OS_DESCRIPTOR_STRING_SIZE
    },
    OS_DESCRIPTOR_STRING_SIZE,
    USB_STRING_DESCRIPTOR_TYPE,
    { 'M', 'S', 'F', 'T', '1', '0', '0' },
    OS_DESCRIPTOR_STRING_VENDOR_CODE,
    0x00
};

// OS Extended Compatible ID for WinUSB
const TinyCLR_UsbClient_XCompatibleOsId STM32F4_UsbClient_XCompatibleOsId = {
    // Generic Descriptor header
    {
        {
            USB_GENERIC_DESCRIPTOR_MARKER,
            0,
            sizeof(TinyCLR_UsbClient_GenericDescriptorHeader) + USB_XCOMPATIBLE_OS_SIZE
        },
        USB_REQUEST_TYPE_IN | USB_REQUEST_TYPE_VENDOR,
        OS_DESCRIPTOR_STRING_VENDOR_CODE,
        0,                                              // Intfc # << 8 + Page #
        USB_XCOMPATIBLE_OS_REQUEST                      // Extended Compatible OS ID request
    },
    USB_XCOMPATIBLE_OS_SIZE,                            // Size of this descriptor
    OS_DESCRIPTOR_EX_VERSION,                           // Version 1.00 (BCD)
    USB_XCOMPATIBLE_OS_REQUEST,                         // Extended Compatible OS ID response
    1,                                                  // Only 1 function record
    { 0, 0, 0, 0, 0, 0, 0 },                            // (padding)
    // Extended Compatible OS ID function record
    0,                                                  // Interface 0
    1,                                                  // (reserved)
    { 'W', 'I', 'N', 'U', 'S', 'B',  0,  0 },           // Compatible ID
    {  0,   0,   0,   0,   0,   0,   0,  0 },           // Sub-compatible ID
    { 0, 0, 0, 0, 0, 0 }                                // Padding
};

// OS Extended Property
const TinyCLR_UsbClient_XPropertiesOsWinUsb STM32F4_UsbClient_XPropertiesOsWinUsb = {
    // Generic Descriptor header
    {
        {
            USB_GENERIC_DESCRIPTOR_MARKER,
            0,
            sizeof(TinyCLR_UsbClient_GenericDescriptorHeader) + USB_XPROPERTY_OS_SIZE_WINUSB
        },
        USB_REQUEST_TYPE_IN | USB_REQUEST_TYPE_VENDOR | USB_REQUEST_TYPE_INTERFACE,
        OS_DESCRIPTOR_STRING_VENDOR_CODE,
        0,                                              // Intfc # << 8 + Page #
        USB_XPROPERTY_OS_REQUEST                        // Extended Property OS ID request
    },
    USB_XPROPERTY_OS_SIZE_WINUSB,                       // Size of this descriptor (78 bytes for guid + 40 bytes for the property name + 24 bytes for other fields = 142 bytes)
    OS_DESCRIPTOR_EX_VERSION,                           // Version 1.00 (BCD)
    USB_XPROPERTY_OS_REQUEST,                           // Extended Compatible OS ID response
    1,                                                  // Only 1 ex property record
    // Extended Property OS ID function record
    0x00000084,                                         // size in bytes
    EX_PROPERTY_DATA_TYPE__REG_SZ,                      // data type (unicode string)
    40,                                                 // name length
    // property name (null -terminated unicode string: 'DeviceInterfaceGuid\0')
    { 'D','\0', 'e','\0', 'v','\0', 'i','\0', 'c','\0', 'e','\0', 'I','\0', 'n','\0', 't','\0', 'e','\0', 'r','\0', 'f','\0', 'a','\0', 'c','\0', 'e','\0', 'G','\0', 'u','\0', 'i','\0', 'd','\0', '\0','\0' },
    78,                                                 // data length
    // data {C13BCFE9-5E84-4187-9BAA-45597FFCBB6F}
    { '{','\0', 'C','\0', '1','\0', '3','\0', 'B','\0', 'C','\0', 'F','\0', 'E','\0', '9','\0', '-','\0', '5','\0', 'E','\0', '8','\0', '4','\0', '-','\0', '4','\0', '1','\0', '8','\0', '7','\0', '-','\0', '9','\0', 'B','\0', 'A','\0', 'A','\0', '-','\0', '4','\0', '5','\0', '5','\0', '9','\0', '7','\0', 'F','\0', 'F','\0', 'C','\0', 'B','\0', 'B','\0', '6','\0', 'F','\0', '}','\0', '\0','\0' }
};

void TinyCLR_UsbClient_SetupConfiguration(TinyCLR_UsbClient_Configuration* configuration) {
    memset((uint8_t*)configuration, 0, sizeof(TinyCLR_UsbClient_Configuration));

    configuration->deviceDescriptor = (TinyCLR_UsbClient_DeviceDescriptor*)&STM32F4_UsbClient_DeviceDescriptor;
    configuration->configurationDescriptor = (TinyCLR_UsbClient_ConfigurationDescriptor*)&STM32F4_UsbClient_ConfigurationDescriptor;
    configuration->interfaceDescriptor = (TinyCLR_UsbClient_InterfaceDescriptor*)&STM32F4_UsbClient_InterfaceDescriptor;
    configuration->endpointDescriptor = (TinyCLR_UsbClient_EndpointDescriptor*)STM32F4_UsbClient_EndpointDescriptor;
    configuration->stringsDescriptor = (TinyCLR_UsbClient_StringDescriptorHeader*)STM32F4_UsbClient_StringDescriptor;
    configuration->OsStringDescriptor = (TinyCLR_UsbClient_OsStringDescriptor*)&STM32F4_UsbClient_OsStringDescriptor;
    configuration->OsXCompatibleId = (TinyCLR_UsbClient_XCompatibleOsId*)&STM32F4_UsbClient_XCompatibleOsId;
    configuration->OsXProperty = (TinyCLR_UsbClient_XPropertiesOsWinUsb*)&STM32F4_UsbClient_XPropertiesOsWinUsb;
}
