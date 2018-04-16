// Copyright Microsoft Corporation
// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include "LPC17.h"

#define __min(a,b)  (((a) < (b)) ? (a) : (b))

#if defined(__GNUC__)
#define PACKED(x) x __attribute__((packed))
#elif defined(arm) || defined(__arm)
#define PACKED(x) __packed x
#endif

///////////////////////////////////////////////////////////////////////////////////////////
/// USB Debugger driver
///////////////////////////////////////////////////////////////////////////////////////////

// USB 2.0 host requests
#define USB_GET_STATUS           0
#define USB_CLEAR_FEATURE        1
#define USB_SET_FEATURE          3
#define USB_SET_ADDRESS          5
#define USB_GET_DESCRIPTOR       6
#define USB_SET_DESCRIPTOR       7
#define USB_GET_CONFIGURATION    8
#define USB_SET_CONFIGURATION    9

// USB 2.0 defined descriptor types
#define USB_DEVICE_DESCRIPTOR_TYPE        1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE 2
#define USB_STRING_DESCRIPTOR_TYPE        3
#define USB_INTERFACE_DESCRIPTOR_TYPE     4
#define USB_ENDPOINT_DESCRIPTOR_TYPE      5

// USB 2.0 host request type defines
#define USB_SETUP_RECIPIENT(n)          ((n) & 0x0F)
#define USB_SETUP_RECIPIENT_DEVICE             0x00
#define USB_SETUP_RECIPIENT_INTERFACE          0x01
#define USB_SETUP_RECIPIENT_ENDPOINT           0x02

// Local device status defines
#define USB_STATUS_DEVICE_SELF_POWERED   0x0001
#define USB_STATUS_DEVICE_REMOTE_WAKEUP  0x0002

#define USB_STATUS_ENDPOINT_HALT         0x0001

#define USB_FEATURE_ENDPOINT_HALT        0x0000
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP 0x0001

// Local device possible states
#define USB_DEVICE_STATE_DETACHED       0
#define USB_DEVICE_STATE_ATTACHED       1
#define USB_DEVICE_STATE_POWERED        2
#define USB_DEVICE_STATE_DEFAULT        3
#define USB_DEVICE_STATE_ADDRESS        4
#define USB_DEVICE_STATE_CONFIGURED     5
#define USB_DEVICE_STATE_SUSPENDED      6
#define USB_DEVICE_STATE_UNINITIALIZED  0xFF

// Possible responses to host requests
#define USB_STATE_DATA                  0
#define USB_STATE_STALL                 1
#define USB_STATE_DONE                  2
#define USB_STATE_ADDRESS               3
#define USB_STATE_STATUS                4
#define USB_STATE_CONFIGURATION         5
#define USB_STATE_REMOTE_WAKEUP         6


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

// Configuration for extended descriptor
#define OS_DESCRIPTOR_EX_VERSION            0x0100

#define USB_DISPLAY_STRING_NUM     4
#define USB_FRIENDLY_STRING_NUM    5

#define OS_DESCRIPTOR_STRING_INDEX        0xEE
#define OS_DESCRIPTOR_STRING_VENDOR_CODE  0xA5

// USB 2.0 response structure lengths
#define USB_DEVICE_DESCRIPTOR_LENGTH             18
#define USB_CONFIGURATION_DESCRIPTOR_LENGTH       9
#define USB_STRING_DESCRIPTOR_HEADER_LENGTH       2


// USB Configuration list structures
#define USB_END_DESCRIPTOR_MARKER           0x00
#define USB_DEVICE_DESCRIPTOR_MARKER        0x01
#define USB_CONFIGURATION_DESCRIPTOR_MARKER 0x02
#define USB_STRING_DESCRIPTOR_MARKER        0x03
#define USB_GENERIC_DESCRIPTOR_MARKER       0xFF

// Configuration Descriptor
#define USB_ATTRIBUTE_REMOTE_WAKEUP    0x20
#define USB_ATTRIBUTE_SELF_POWER       0x40
#define USB_ATTRIBUTE_BASE             0x80

// Endpoint Direction
#define USB_ENDPOINT_DIRECTION_IN 0x80
#define USB_ENDPOINT_DIRECTION_OUT 0x00
#define USB_ENDPOINT_NULL 0xFF

// Endpoint Attribute
#define ENDPOINT_INUSED_MASK        0x01
#define ENDPOINT_DIR_IN_MASK        0x02
#define ENDPOINT_DIR_OUT_MASK       0x04

#define USB_ENDPOINT_ATTRIBUTE_BULK 2
#define USB_MAX_DATA_PACKET_SIZE 64

#define USB_MAX_EP_SIZE            64 // maximum control channel packet size
#define USB_MAX_EP0_SIZE           64 // default control channel packet size
#define USB_MAX_EP_COUNT           16 // OTG FS supports 4 endpoints

// This version of the USB code supports only one language - which
// is not specified by USB configuration records - it is defined here.
// This is the String 0 descriptor.This array includes the String descriptor
// header and exactly one language.
#define USB_LANGUAGE_DESCRIPTOR_SIZE 4

// USB 2.0 request packet from host
PACKED(struct) USB_SETUP_PACKET {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

PACKED(struct) USB_DYNAMIC_CONFIGURATION;

struct USB_PACKET64 {
    uint32_t Size;
    uint8_t  Buffer[USB_MAX_DATA_PACKET_SIZE];
};

struct USB_PIPE_MAP {
    uint8_t RxEP;
    uint8_t TxEP;
};

struct USB_CONTROLLER_STATE;

typedef void(*USB_NEXT_CALLBACK)(USB_CONTROLLER_STATE*);

struct USB_CONTROLLER_STATE {
    bool                                                        Initialized;
    uint8_t                                                     CurrentState;
    uint8_t                                                     controllerNum;
    uint32_t                                                    Event;

    const USB_DYNAMIC_CONFIGURATION*                            Configuration;

    /* Queues & MaxPacketSize must be initialized by the HAL */
    USB_PACKET64                                   	            *Queues[LPC17_USB_QUEUE_SIZE];
    uint8_t                                                     CurrentPacketOffset[LPC17_USB_QUEUE_SIZE];
    uint8_t                                                     MaxPacketSize[LPC17_USB_QUEUE_SIZE];
    bool                                                        IsTxQueue[LPC17_USB_QUEUE_SIZE];

    /* Arbitrarily as many pipes as endpoints since that is the maximum number of pipes
       necessary to represent the maximum number of endpoints */
    USB_PIPE_MAP                                                pipes[LPC17_USB_QUEUE_SIZE];

    /* used for transferring packets between upper & lower */
    uint8_t*                                                    Data;
    uint8_t                                                     DataSize;

    /* USB hardware information */
    uint8_t                                                     Address;
    uint8_t                                                     DeviceState;
    uint8_t                                                     PacketSize;
    uint8_t                                                     ConfigurationNum;
    uint32_t                                                    FirstGetDescriptor;

    /* USB status information, used in
       GET_STATUS, SET_FEATURE, CLEAR_FEATURE */
    uint16_t                                                    DeviceStatus;
    uint16_t*                                                   EndpointStatus;
    uint8_t                                                     EndpointCount;
    uint8_t                                                     EndpointStatusChange;

    /* callback function for getting next packet */
    USB_NEXT_CALLBACK                                           DataCallback;

    /* for helping out upper layer during callbacks */
    uint8_t*                                                    ResidualData;
    uint16_t                                                    ResidualCount;
    uint16_t                                                    Expected;
};

PACKED(struct) TinyCLR_UsbClient_DescriptorHeader {
    uint8_t  marker;
    uint8_t  iValue;
    uint16_t size;
};

PACKED(struct) TinyCLR_UsbClient_GenericDescriptorHeader {
    TinyCLR_UsbClient_DescriptorHeader header;

    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
};

PACKED(struct) TinyCLR_UsbClient_DeviceDescriptor {
    TinyCLR_UsbClient_DescriptorHeader header;

    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
};

PACKED(struct) TinyCLR_UsbClient_InterfaceDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
};

PACKED(struct) TinyCLR_UsbClient_EndpointDescriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
};

PACKED(struct) TinyCLR_UsbClient_ClassDescriptorHeader {
    uint8_t bLength;
    uint8_t bDescriptorType;
};

PACKED(struct) TinyCLR_UsbClient_StringDescriptorHeader {
    TinyCLR_UsbClient_DescriptorHeader header;

    uint8_t bLength;
    uint8_t bDescriptorType;
    wchar_t stringDescriptor[32];
};

PACKED(struct) TinyCLR_UsbClient_ConfigurationDescriptor {
    TinyCLR_UsbClient_DescriptorHeader header;

    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;

    TinyCLR_UsbClient_InterfaceDescriptor   itfc0;
    TinyCLR_UsbClient_EndpointDescriptor    epWrite;
    TinyCLR_UsbClient_EndpointDescriptor    epRead;
};

PACKED(struct) TinyCLR_UsbClient_OsStringDescriptor {
    TinyCLR_UsbClient_DescriptorHeader header;

    uint8_t   bLength;
    uint8_t   bDescriptorType;
    wchar_t signature[7];
    uint8_t   bMS_VendorCode;
    uint8_t   padding;
};

PACKED(struct) TinyCLR_UsbClient_XCompatibleOsId {
    TinyCLR_UsbClient_GenericDescriptorHeader header;

    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint8_t  bCount;
    uint8_t  padding1[7];
    uint8_t  bFirstInterfaceNumber;
    uint8_t  reserved;
    uint8_t  compatibleID[8];
    uint8_t  subCompatibleID[8];
    uint8_t  padding2[6];
};

PACKED(struct) TinyCLR_UsbClient_XPropertiesOsWinUsb {
    TinyCLR_UsbClient_GenericDescriptorHeader header;

    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint16_t  bCount;

    uint32_t dwSize;
    uint32_t dwPropertyDataType;
    uint16_t wPropertyNameLengh;
    uint8_t  bPropertyName[40];
    uint32_t dwPropertyDataLengh;
    uint8_t  bPropertyData[78];
};

PACKED(struct) USB_DYNAMIC_CONFIGURATION {
    TinyCLR_UsbClient_DeviceDescriptor                  *device;
    TinyCLR_UsbClient_ConfigurationDescriptor           *config;
    TinyCLR_UsbClient_StringDescriptorHeader            *manHeader;
    TinyCLR_UsbClient_StringDescriptorHeader            *prodHeader;
    TinyCLR_UsbClient_StringDescriptorHeader            *displayStringHeader;
    TinyCLR_UsbClient_StringDescriptorHeader            *friendlyStringHeader;
    TinyCLR_UsbClient_OsStringDescriptor                *OS_String;
    TinyCLR_UsbClient_XCompatibleOsId                   *OS_XCompatible_ID;
    TinyCLR_UsbClient_XPropertiesOsWinUsb               *OS_XProperty;
    TinyCLR_UsbClient_DescriptorHeader                  *endList;
};

USB_DYNAMIC_CONFIGURATION UsbDefaultConfiguration;

const TinyCLR_UsbClient_DescriptorHeader * USB_FindRecord(USB_CONTROLLER_STATE* usbState, uint8_t marker, USB_SETUP_PACKET * iValue);
void LPC17_UsbClient_ClearEndpoints(int32_t endpoint);
bool LPC17_UsbClient_RxEnable(USB_CONTROLLER_STATE* usbState, int32_t endpoint);

TinyCLR_UsbClient_DataReceivedHandler LPC17_UsbClient_DataReceivedHandler;
TinyCLR_UsbClient_OsExtendedPropertyHandler LPC17_UsbClient_OsExtendedPropertyHandler;

// usb fifo buffer
static int32_t usb_fifo_buffer_in[LPC17_USB_QUEUE_SIZE];
static int32_t usb_fifo_buffer_out[LPC17_USB_QUEUE_SIZE];
static int32_t usb_fifo_buffer_count[LPC17_USB_QUEUE_SIZE];

int8_t LPC17_UsbClient_EndpointMap[] = { ENDPOINT_INUSED_MASK,                          // Endpoint 0
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 1
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 2
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK   // Endpoint 3
};

#define TOTAL_USB_CONTROLLER 1

#define USB_USBCLIENT_ID 0

USB_CONTROLLER_STATE usbClientState[TOTAL_USB_CONTROLLER];

uint8_t USB_LanguageDescriptor[USB_LANGUAGE_DESCRIPTOR_SIZE] =
{
    USB_LANGUAGE_DESCRIPTOR_SIZE,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09, 0x04                      // U.S. English
};

TinyCLR_UsbClient_DeviceDescriptor deviceDescriptor = {

    {
        USB_DEVICE_DESCRIPTOR_MARKER,
        0,
        sizeof(TinyCLR_UsbClient_DeviceDescriptor)
    },
    USB_DEVICE_DESCRIPTOR_LENGTH,       // Length of device descriptor
    USB_DEVICE_DESCRIPTOR_TYPE,         // USB device descriptor type
    0x0200,                             // USB Version 2.00 (BCD) (2.0 required for Extended ID recognition)
    0,                                  // Device class (none)
    0,                                  // Device subclass (none)
    0,                                  // Device protocol (none)
    USB_MAX_EP0_SIZE,                       // Endpoint 0 size
    USB_DEBUGGER_VENDOR_ID,             // Vendor ID
    USB_DEBUGGER_PRODUCT_ID,            // Product ID
    DEVICE_RELEASE_VERSION,             // Product version 1.00 (BCD)
    MANUFACTURER_NAME_INDEX,            // Manufacturer name string index
    PRODUCT_NAME_INDEX,                 // Product name string index
    0,                                  // Serial number string index (none)
    1                                   // Number of configurations
};

// Configuration descriptor
TinyCLR_UsbClient_ConfigurationDescriptor configDescriptor = {
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
    50,                                                 // Device max current draw

    // Interface
    sizeof(TinyCLR_UsbClient_InterfaceDescriptor),
    USB_INTERFACE_DESCRIPTOR_TYPE,
    0,                                          // Interface number
    0,                                          // Alternate number (main)
    2,                                          // Number of endpoints
    0xFF,                                       // Interface class (vendor)
    1,                                          // Interface subclass
    1,                                          // Interface protocol
    0 ,                                          // Interface descriptor string index (none)

    // Endpoint
    sizeof(TinyCLR_UsbClient_EndpointDescriptor),
    USB_ENDPOINT_DESCRIPTOR_TYPE,
    USB_ENDPOINT_DIRECTION_IN,
    USB_ENDPOINT_ATTRIBUTE_BULK,
    USB_MAX_EP_SIZE,                                // Endpoint 1 packet size
    0 ,                                          // Endpoint 1 interval

    //Endpoint
    sizeof(TinyCLR_UsbClient_EndpointDescriptor),
    USB_ENDPOINT_DESCRIPTOR_TYPE,
    USB_ENDPOINT_DIRECTION_OUT,
    USB_ENDPOINT_ATTRIBUTE_BULK,
    USB_MAX_EP_SIZE,                                // Endpoint 1 packet size
    0                                           // Endpoint 1 interval
};

// Manufacturer name string descriptor header
TinyCLR_UsbClient_StringDescriptorHeader stringManufacturerDescriptorHeader = {
        {
            USB_STRING_DESCRIPTOR_MARKER,
            MANUFACTURER_NAME_INDEX,
            sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
        },
        USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
        USB_STRING_DESCRIPTOR_TYPE,
        CONCAT(L, DEVICE_MANUFACTURER)
};

// Product name string descriptor header
TinyCLR_UsbClient_StringDescriptorHeader stringProductNameDescriptorHeader = {
    {
        USB_STRING_DESCRIPTOR_MARKER,
        PRODUCT_NAME_INDEX,
        sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
    },
    USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
    USB_STRING_DESCRIPTOR_TYPE,
    CONCAT(L, DEVICE_NAME)
};

// String 4 descriptor header (display name)
TinyCLR_UsbClient_StringDescriptorHeader stringDisplayNameDescriptorHeader = {
    {
        USB_STRING_DESCRIPTOR_MARKER,
        USB_DISPLAY_STRING_NUM,
        sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
    },
    USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
    USB_STRING_DESCRIPTOR_TYPE,
    CONCAT(L, DEVICE_NAME)
};

// String 5 descriptor header (friendly name)
TinyCLR_UsbClient_StringDescriptorHeader stringFriendlyNameDescriptorHeader = {
    {
        USB_STRING_DESCRIPTOR_MARKER,
        USB_FRIENDLY_STRING_NUM,
        sizeof(TinyCLR_UsbClient_StringDescriptorHeader)
    },
    USB_STRING_DESCRIPTOR_HEADER_LENGTH + (sizeof(wchar_t) * USB_STRING_DESCRIPTOR_SIZE),
    USB_STRING_DESCRIPTOR_TYPE,
    CONCAT(L, DEVICE_NAME)
};

// OS Descriptor string for Extended OS Compat ID
TinyCLR_UsbClient_OsStringDescriptor LPC17_UsbClient_OsStringDescriptor;

// OS Extended Compatible ID for WinUSB
TinyCLR_UsbClient_XCompatibleOsId LPC17_UsbClient_XCompatibleOsId;

// OS Extended Property
TinyCLR_UsbClient_XPropertiesOsWinUsb LPC17_UsbClient_XPropertiesOsWinUsb;

// End of configuration marker
const TinyCLR_UsbClient_DescriptorHeader usbDescriptorHeader = {
    USB_END_DESCRIPTOR_MARKER,
    0,
    0
};

void LPC17_UsbClient_SetEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t old_event = usbState->Event;

    usbState->Event |= event;

    if (old_event != usbState->Event) {
        LPC17_UsbClient_DataReceivedHandler(nullptr);
    }
}

void LPC17_UsbClient_ClearEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    usbState->Event &= ~event;
}

void LPC17_UsbClient_ClearQueues(USB_CONTROLLER_STATE *usbState, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (ClrRxQueue) {
        for (int32_t endpoint = 0; endpoint < LPC17_USB_QUEUE_SIZE; endpoint++) {
            if (usbState->Queues[endpoint] == nullptr || usbState->IsTxQueue[endpoint])
                continue;

            LPC17_UsbClient_ClearEndpoints(endpoint);

            /* since this queue is now reset, we have room available for newly arrived packets */
            LPC17_UsbClient_RxEnable(usbState, endpoint);
        }
    }

    if (ClrTxQueue) {
        for (int32_t endpoint = 0; endpoint < LPC17_USB_QUEUE_SIZE; endpoint++) {
            if (usbState->Queues[endpoint] && usbState->IsTxQueue[endpoint])
                LPC17_UsbClient_ClearEndpoints(endpoint);
        }
    }
}

void LPC17_UsbClientControllerCallback(USB_CONTROLLER_STATE* usbState) {
    if (usbState->CurrentState != usbState->DeviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
        //Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
        // The RxQueue is clear when it is configured.
        if (USB_DEVICE_STATE_CONFIGURED == usbState->CurrentState) {
            LPC17_UsbClient_ClearQueues(usbState, true, true);
        }

        usbState->CurrentState = usbState->DeviceState;

        switch (usbState->DeviceState) {
        case USB_DEVICE_STATE_DETACHED:
            usbState->ResidualCount = 0;
            usbState->DataCallback = nullptr;

            break;
        case USB_DEVICE_STATE_CONFIGURED:
            /* whenever we enter the configured state, re-initialize all of the RxQueues */
            /* Txqueue has stored some data to be transmitted */
            LPC17_UsbClient_ClearQueues(usbState, true, false);
            break;
        }
    }
}

void LPC17_UsbClient_DataCallback(USB_CONTROLLER_STATE* usbState) {
    uint32_t length = __min(usbState->PacketSize, usbState->ResidualCount);

    memcpy(usbState->Data, usbState->ResidualData, length);

    usbState->DataSize = length;
    usbState->ResidualData += length;
    usbState->ResidualCount -= length;

    if (length == usbState->PacketSize) {
        usbState->Expected -= length;
    }
    else {
        usbState->Expected = 0;
    }

    if (usbState->Expected) {
        usbState->DataCallback = LPC17_UsbClient_DataCallback;
    }
    else {
        usbState->DataCallback = nullptr;
    }
}

uint8_t LPC17_UsbClient_HandleGetStatus(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    uint16_t* status;
    uint16_t  zero = 0;

    /* validate setup packet */
    if (Setup->wValue != 0 || Setup->wLength != 2) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        status = &usbState->DeviceStatus;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        if (usbState->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
            return USB_STATE_STALL;
        }

        status = &zero;
        break;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbState->DeviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->wIndex &= 0x7F;

        if (Setup->wIndex >= usbState->EndpointCount) {
            return USB_STATE_STALL;
        }

        status = &usbState->EndpointStatus[Setup->wIndex];
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send requested status to host */
    usbState->ResidualData = (uint8_t*)status;
    usbState->ResidualCount = 2;
    usbState->DataCallback = LPC17_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t LPC17_UsbClient_HandleClearFeature(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->wValue != USB_FEATURE_DEVICE_REMOTE_WAKEUP)
            return USB_STATE_STALL;

        // Locate the configuration descriptor
        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)USB_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);

        if (Config && (Config->bmAttributes & USB_ATTRIBUTE_REMOTE_WAKEUP)) {
            usbState->DeviceStatus &= ~USB_STATUS_DEVICE_REMOTE_WAKEUP;
            retState = USB_STATE_REMOTE_WAKEUP;
        }
        else {
            return USB_STATE_STALL;
        }
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        /* there are no interface features to clear */
        return USB_STATE_STALL;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbState->DeviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0)
            return USB_STATE_STALL;

        /* bit 0x80 designates direction, which we dont utilize in this calculation */
        Setup->wIndex &= 0x7F;

        if (Setup->wIndex == 0 || Setup->wIndex >= usbState->EndpointCount)
            return USB_STATE_STALL;

        if (Setup->wValue != USB_FEATURE_ENDPOINT_HALT)
            return USB_STATE_STALL;

        /* clear the halt feature */
        usbState->EndpointStatus[Setup->wIndex] &= ~USB_STATUS_ENDPOINT_HALT;
        usbState->EndpointStatusChange = Setup->wIndex;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbState->ResidualCount = 0;
    usbState->DataCallback = LPC17_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t LPC17_UsbClient_HandleSetFeature(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->wValue != USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
            return USB_STATE_STALL;
        }

        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)USB_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
        if (Config == nullptr)        // If the configuration record could not be found
            return USB_STATE_STALL; // Something pretty serious is wrong

        if (Config->bmAttributes & USB_ATTRIBUTE_REMOTE_WAKEUP) {
            usbState->DeviceStatus |= USB_STATUS_DEVICE_REMOTE_WAKEUP;
        }

        retState = USB_STATE_REMOTE_WAKEUP;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        /* there are no interface features to set */
        return USB_STATE_STALL;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbState->DeviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->wIndex &= 0x7F;

        if (Setup->wIndex == 0 || Setup->wIndex >= usbState->EndpointCount) {
            return USB_STATE_STALL;
        }

        if (Setup->wValue != USB_FEATURE_ENDPOINT_HALT) {
            return USB_STATE_STALL;
        }

        /* set the halt feature */
        usbState->EndpointStatus[Setup->wIndex] |= USB_STATUS_ENDPOINT_HALT;
        usbState->EndpointStatusChange = Setup->wIndex;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbState->ResidualCount = 0;
    usbState->DataCallback = LPC17_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t LPC17_UsbClient_HandleSetAddress(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    /* validate setup packet */
    if (Setup->wValue > 127 || Setup->wIndex != 0 || Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->DeviceState >= USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    /* set address */
    usbState->Address = Setup->wValue;

    /* catch state changes */
    if (usbState->Address == 0) {
        usbState->DeviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else {
        usbState->DeviceState = USB_DEVICE_STATE_ADDRESS;
    }

    LPC17_UsbClientControllerCallback(usbState);

    /* send zero-length packet to tell host we're done */
    usbState->ResidualCount = 0;
    usbState->DataCallback = LPC17_UsbClient_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t LPC17_UsbClient_HandleConfigurationRequests(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    const TinyCLR_UsbClient_DescriptorHeader * header;
    uint8_t       type;
    uint8_t       DescriptorIndex;

    /* this request is valid regardless of device state */
    type = ((Setup->wValue & 0xFF00) >> 8);
    DescriptorIndex = (Setup->wValue & 0x00FF);
    usbState->Expected = Setup->wLength;

    if (usbState->Expected == 0) {
        // just return an empty Status packet
        usbState->ResidualCount = 0;
        usbState->DataCallback = LPC17_UsbClient_DataCallback;
        return USB_STATE_DATA;
    }

    // The very first GET_DESCRIPTOR command out of reset should always return at most PacketSize bytes.
    // After that, you can return as many as the host has asked.
    if (usbState->DeviceState <= USB_DEVICE_STATE_DEFAULT) {
        if (usbState->FirstGetDescriptor) {
            usbState->FirstGetDescriptor = false;

            usbState->Expected = __min(usbState->Expected, usbState->PacketSize);
        }
    }

    usbState->ResidualData = nullptr;
    usbState->ResidualCount = 0;

    if (Setup->bRequest == USB_GET_DESCRIPTOR) {
        switch (type) {
        case USB_DEVICE_DESCRIPTOR_TYPE:
            header = USB_FindRecord(usbState, USB_DEVICE_DESCRIPTOR_MARKER, Setup);
            if (header) {
                const TinyCLR_UsbClient_DeviceDescriptor * device = (TinyCLR_UsbClient_DeviceDescriptor *)header;
                usbState->ResidualData = (uint8_t *)&device->bLength;      // Start of the device descriptor
                usbState->ResidualCount = __min(usbState->Expected, device->bLength);
            }
            break;

        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            header = USB_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
            if (header) {
                const TinyCLR_UsbClient_ConfigurationDescriptor * Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)header;
                usbState->ResidualData = (uint8_t *)&Config->bLength;
                usbState->ResidualCount = __min(usbState->Expected, Config->wTotalLength);
            }
            break;

        case USB_STRING_DESCRIPTOR_TYPE:
            if (DescriptorIndex == 0)        // If host is requesting the language list
            {
                usbState->ResidualData = USB_LanguageDescriptor;
                usbState->ResidualCount = __min(usbState->Expected, USB_LANGUAGE_DESCRIPTOR_SIZE);
            }
            else if (nullptr != (header = USB_FindRecord(usbState, USB_STRING_DESCRIPTOR_MARKER, Setup))) {
                const TinyCLR_UsbClient_StringDescriptorHeader * string = (TinyCLR_UsbClient_StringDescriptorHeader *)header;
                usbState->ResidualData = (uint8_t *)&string->bLength;
                usbState->ResidualCount = __min(usbState->Expected, string->bLength);
            }
            break;

        default:
            break;
        }
    }

    // If the request was not recognized, the generic types should be searched
    if (usbState->ResidualData == nullptr) {
        if (nullptr != (header = USB_FindRecord(usbState, USB_GENERIC_DESCRIPTOR_MARKER, Setup))) {
            usbState->ResidualData = (uint8_t *)header;
            usbState->ResidualData += sizeof(TinyCLR_UsbClient_GenericDescriptorHeader);       // Data is located right after the header
            usbState->ResidualCount = __min(usbState->Expected, header->size - sizeof(TinyCLR_UsbClient_GenericDescriptorHeader));
        }
        else
            return USB_STATE_STALL;
    }

    usbState->DataCallback = LPC17_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t LPC17_UsbClient_HandleGetConfiguration(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    /* validate setup packet */
    if (Setup->wValue != 0 || Setup->wIndex != 0 || Setup->wLength != 1) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    usbState->ResidualData = &usbState->ConfigurationNum;
    usbState->ResidualCount = 1;
    usbState->Expected = 1;
    usbState->DataCallback = LPC17_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t LPC17_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup, bool DataPhase) {
    /* validate setup packet */
    if (Setup->wIndex != 0 || Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    /* we only support one configuration */
    if (Setup->wValue > 1) {
        return USB_STATE_STALL;
    }

    usbState->ConfigurationNum = Setup->wValue;

    /* catch state changes */
    if (usbState->ConfigurationNum == 0) {
        usbState->DeviceState = USB_DEVICE_STATE_ADDRESS;
    }
    else {
        usbState->DeviceState = USB_DEVICE_STATE_CONFIGURED;
    }

    LPC17_UsbClientControllerCallback(usbState);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        usbState->ResidualCount = 0;
        usbState->DataCallback = LPC17_UsbClient_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB Configuration records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const TinyCLR_UsbClient_DescriptorHeader * USB_FindRecord(USB_CONTROLLER_STATE* usbState, uint8_t marker, USB_SETUP_PACKET * setup) {
    bool Done = false;

    const TinyCLR_UsbClient_DescriptorHeader * header = (const TinyCLR_UsbClient_DescriptorHeader *)usbState->Configuration;
    TinyCLR_UsbClient_DescriptorHeader* ptr = (TinyCLR_UsbClient_DescriptorHeader*)(*(uint32_t*)header);
    TinyCLR_UsbClient_ConfigurationDescriptor *config;

    // If there is no configuration for this controller
    if (nullptr == header)
        return header;

    while (!Done) {
        const uint8_t *next = (const uint8_t *)header;
        ptr = (TinyCLR_UsbClient_DescriptorHeader*)(*(uint32_t*)header);
        next += 4;      // Calculate address of next record

        const TinyCLR_UsbClient_GenericDescriptorHeader *generic = (TinyCLR_UsbClient_GenericDescriptorHeader *)ptr;

        switch (ptr->marker) {
        case USB_DEVICE_DESCRIPTOR_MARKER:
            if (ptr->marker == marker)
                Done = true;
            break;

        case USB_CONFIGURATION_DESCRIPTOR_MARKER:
            config = (TinyCLR_UsbClient_ConfigurationDescriptor *)UsbDefaultConfiguration.config;
            if (config->header.marker == marker)
                Done = true;
            break;

        case USB_STRING_DESCRIPTOR_MARKER:
            // If String descriptor then the index is significant
            if ((ptr->marker == marker) && (ptr->iValue == (setup->wValue & 0x00FF)))
                Done = true;
            break;
        case USB_GENERIC_DESCRIPTOR_MARKER:
            if (generic->bmRequestType == setup->bmRequestType &&
                generic->bRequest == setup->bRequest &&
                generic->wValue == setup->wValue &&
                generic->wIndex == setup->wIndex) {
                Done = true;
            }
            break;
        case USB_END_DESCRIPTOR_MARKER:
            Done = true;
            header = nullptr;
            ptr = nullptr;
            break;
        }
        if (!Done)
            header = (const TinyCLR_UsbClient_DescriptorHeader *)next;    // Try next record
    }

    return ptr;
}

uint8_t LPC17_UsbClient_ControlCallback(USB_CONTROLLER_STATE* usbState) {
    USB_SETUP_PACKET* Setup;

    if (usbState->DataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (USB_SETUP_PACKET*)usbState->Data;

    switch (Setup->bRequest) {
    case USB_GET_STATUS:
        return LPC17_UsbClient_HandleGetStatus(usbState, Setup);
    case USB_CLEAR_FEATURE:
        return LPC17_UsbClient_HandleClearFeature(usbState, Setup);
    case USB_SET_FEATURE:
        return LPC17_UsbClient_HandleSetFeature(usbState, Setup);
    case USB_SET_ADDRESS:
        return LPC17_UsbClient_HandleSetAddress(usbState, Setup);
    case USB_GET_CONFIGURATION:
        return LPC17_UsbClient_HandleGetConfiguration(usbState, Setup);
    case USB_SET_CONFIGURATION:
        return LPC17_UsbClient_HandleSetConfiguration(usbState, Setup, true);
    default:
        return LPC17_UsbClient_HandleConfigurationRequests(usbState, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* LPC17_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint, bool& disableRx) {
    USB_PACKET64* packet;

    if (usb_fifo_buffer_count[endpoint] == LPC17_USB_FIFO_BUFFER_SIZE) {
        disableRx = true;

        return nullptr;
    }

    disableRx = false;

    packet = &usbState->Queues[endpoint][usb_fifo_buffer_in[endpoint]];

    usb_fifo_buffer_in[endpoint]++;
    usb_fifo_buffer_count[endpoint]++;

    if (usb_fifo_buffer_in[endpoint] == LPC17_USB_FIFO_BUFFER_SIZE)
        usb_fifo_buffer_in[endpoint] = 0;

    LPC17_UsbClient_SetEvent(usbState, 1 << endpoint);

    return packet;
}

USB_PACKET64* LPC17_UsbClient_TxDequeue(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    USB_PACKET64* packet;

    if (usb_fifo_buffer_count[endpoint] == 0) {
        return nullptr;
    }

    packet = &usbState->Queues[endpoint][usb_fifo_buffer_out[endpoint]];

    usb_fifo_buffer_count[endpoint]--;
    usb_fifo_buffer_out[endpoint]++;

    if (usb_fifo_buffer_out[endpoint] == LPC17_USB_FIFO_BUFFER_SIZE)
        usb_fifo_buffer_out[endpoint] = 0;

    return packet;
}

void LPC17_UsbClient_ClearEndpoints(int32_t endpoint) {
    usb_fifo_buffer_in[endpoint] = usb_fifo_buffer_out[endpoint] = usb_fifo_buffer_count[endpoint] = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
/// TinyCLR USBClient API
///////////////////////////////////////////////////////////////////////////////////////////
bool LPC17_UsbClient_Initialize(USB_CONTROLLER_STATE* usbState);
bool LPC17_UsbClient_Uninitialize(USB_CONTROLLER_STATE* usbState);
bool LPC17_UsbClient_StartOutput(USB_CONTROLLER_STATE* usbState, int32_t endpoint);

static TinyCLR_UsbClient_Provider usbClientProvider;
static TinyCLR_Api_Info usbClientApi;

const TinyCLR_Api_Info* LPC17_UsbClient_GetApi() {
    usbClientProvider.Parent = &usbClientApi;
    usbClientProvider.Index = 0;
    usbClientProvider.Acquire = &LPC17_UsbClient_Acquire;
    usbClientProvider.Release = &LPC17_UsbClient_Release;
    usbClientProvider.Open = &LPC17_UsbClient_Open;
    usbClientProvider.Close = &LPC17_UsbClient_Close;
    usbClientProvider.Write = &LPC17_UsbClient_Write;
    usbClientProvider.Read = &LPC17_UsbClient_Read;
    usbClientProvider.Flush = &LPC17_UsbClient_Flush;
    usbClientProvider.SetDeviceDescriptor = &LPC17_UsbClient_SetDeviceDescriptor;
    usbClientProvider.SetConfigDescriptor = &LPC17_UsbClient_SetConfigDescriptor;
    usbClientProvider.SetStringDescriptor = &LPC17_UsbClient_SetStringDescriptor;
    usbClientProvider.SetDataReceivedHandler = &LPC17_UsbClient_SetDataReceivedHandler;
    usbClientProvider.SetOsExtendedPropertyHandler = &LPC17_UsbClient_SetOsExtendedPropertyHandler;

    usbClientApi.Author = "GHI Electronics, LLC";
    usbClientApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.UsbClientProvider";
    usbClientApi.Type = TinyCLR_Api_Type::UsbClientProvider;
    usbClientApi.Version = 0;
    usbClientApi.Count = 1;
    usbClientApi.Implementation = &usbClientProvider;

    return &usbClientApi;
}
TinyCLR_Result LPC17_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    uint8_t *osStringDescriptor = (uint8_t*)&LPC17_UsbClient_OsStringDescriptor;
    uint8_t *xCompatibleOsId = (uint8_t*)&LPC17_UsbClient_XCompatibleOsId;
    uint8_t *xPropertiesOsWinUsb = (uint8_t*)&LPC17_UsbClient_XPropertiesOsWinUsb;

    LPC17_UsbClient_OsExtendedPropertyHandler(self, osStringDescriptor, xCompatibleOsId, xPropertiesOsWinUsb);

    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Init UsbDefaultConfiguration
    memset(&UsbDefaultConfiguration, 0, sizeof(USB_DYNAMIC_CONFIGURATION));

    configDescriptor.epWrite.bEndpointAddress = USB_ENDPOINT_DIRECTION_IN;
    configDescriptor.epRead.bEndpointAddress = USB_ENDPOINT_DIRECTION_OUT;

    UsbDefaultConfiguration.device = (TinyCLR_UsbClient_DeviceDescriptor*)&deviceDescriptor;
    UsbDefaultConfiguration.config = (TinyCLR_UsbClient_ConfigurationDescriptor*)&configDescriptor;

    UsbDefaultConfiguration.manHeader = (TinyCLR_UsbClient_StringDescriptorHeader*)&stringManufacturerDescriptorHeader;
    UsbDefaultConfiguration.prodHeader = (TinyCLR_UsbClient_StringDescriptorHeader*)&stringProductNameDescriptorHeader;
    UsbDefaultConfiguration.displayStringHeader = (TinyCLR_UsbClient_StringDescriptorHeader*)&stringDisplayNameDescriptorHeader;
    UsbDefaultConfiguration.friendlyStringHeader = (TinyCLR_UsbClient_StringDescriptorHeader*)&stringFriendlyNameDescriptorHeader;

    UsbDefaultConfiguration.OS_String = (TinyCLR_UsbClient_OsStringDescriptor*)&LPC17_UsbClient_OsStringDescriptor;
    UsbDefaultConfiguration.OS_XCompatible_ID = (TinyCLR_UsbClient_XCompatibleOsId*)&LPC17_UsbClient_XCompatibleOsId;
    UsbDefaultConfiguration.OS_XProperty = (TinyCLR_UsbClient_XPropertiesOsWinUsb*)&LPC17_UsbClient_XPropertiesOsWinUsb;

    UsbDefaultConfiguration.endList = (TinyCLR_UsbClient_DescriptorHeader*)&usbDescriptorHeader;

    // Init Usb usbState
    memset(usbState, 0, sizeof(USB_CONTROLLER_STATE));

    usbState->controllerNum = controller;
    usbState->Configuration = &UsbDefaultConfiguration;
    usbState->CurrentState = USB_DEVICE_STATE_UNINITIALIZED;
    usbState->DeviceStatus = USB_STATUS_DEVICE_SELF_POWERED;
    usbState->EndpointCount = USB_MAX_EP_COUNT;
    usbState->PacketSize = USB_MAX_EP0_SIZE;
    usbState->Initialized = true;

    for (auto i = 0; i < LPC17_USB_QUEUE_SIZE; i++) {
        usbState->pipes[i].RxEP = USB_ENDPOINT_NULL;
        usbState->pipes[i].TxEP = USB_ENDPOINT_NULL;
        usbState->MaxPacketSize[i] = USB_MAX_EP_SIZE;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Release(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (usbState->Initialized) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        LPC17_UsbClient_Uninitialize(usbState);

        usbState->Initialized = false;

        // for soft reboot allow the USB to be off for at least 100ms
        LPC17_Time_Delay(nullptr, 100000); // 100ms
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t& pipe, TinyCLR_UsbClient_PipeMode mode) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE * usbState = &usbClientState[controller];

    if (!usbState->Initialized)     // If no such controller exists (or it is not initialized)
        return TinyCLR_Result::NotAvailable;

    int32_t writeEp = USB_ENDPOINT_NULL;
    int32_t readEp = USB_ENDPOINT_NULL;

    if (mode != TinyCLR_UsbClient_PipeMode::InOut)
        return TinyCLR_Result::NotSupported;

    for (auto i = 0; i < SIZEOF_ARRAY(LPC17_UsbClient_EndpointMap); i++) {
        if ((LPC17_UsbClient_EndpointMap[i] & ENDPOINT_INUSED_MASK)) // in used
            continue;

        if (writeEp == USB_ENDPOINT_NULL && ((LPC17_UsbClient_EndpointMap[i] & ENDPOINT_DIR_IN_MASK) == ENDPOINT_DIR_IN_MASK)) {
            writeEp = i;
            LPC17_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

            continue;
        }

        if (readEp == USB_ENDPOINT_NULL && ((LPC17_UsbClient_EndpointMap[i] & ENDPOINT_DIR_OUT_MASK) == ENDPOINT_DIR_OUT_MASK)) {
            readEp = i;
            LPC17_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

            continue;
        }

        if (writeEp != 0 && readEp != 0) {
            break;
        }
    }
    // Check the usbPipe and the two endpoint numbers for validity (both endpoints cannot be zero)
    if ((readEp == USB_ENDPOINT_NULL && writeEp == USB_ENDPOINT_NULL)
        || (readEp != USB_ENDPOINT_NULL && (readEp < 1 || readEp >= LPC17_USB_QUEUE_SIZE))
        || (writeEp != USB_ENDPOINT_NULL && (writeEp < 1 || writeEp >= LPC17_USB_QUEUE_SIZE)))
        return TinyCLR_Result::NotAvailable;

    // The specified endpoints must not be in use by another pipe
    for (int32_t i = 0; i < LPC17_USB_QUEUE_SIZE; i++) {
        if (readEp != USB_ENDPOINT_NULL && (usbState->pipes[i].RxEP == readEp || usbState->pipes[i].TxEP == readEp))
            return TinyCLR_Result::NotAvailable;
        if (writeEp != USB_ENDPOINT_NULL && (usbState->pipes[i].RxEP == writeEp || usbState->pipes[i].TxEP == writeEp))
            return TinyCLR_Result::NotAvailable;
    }

    for (pipe = 0; pipe < LPC17_USB_QUEUE_SIZE; pipe++) {
        // The Pipe must be currently closed
        if (usbState->pipes[pipe].RxEP == USB_ENDPOINT_NULL && usbState->pipes[pipe].TxEP == USB_ENDPOINT_NULL)
            break;
    }

    if (pipe == LPC17_USB_QUEUE_SIZE)
        return TinyCLR_Result::NotAvailable;

    // All tests pass, assign the endpoints to the pipe
    usbState->pipes[pipe].RxEP = readEp;
    usbState->pipes[pipe].TxEP = writeEp;

    TinyCLR_UsbClient_ConfigurationDescriptor *config = (TinyCLR_UsbClient_ConfigurationDescriptor *)UsbDefaultConfiguration.config;
    TinyCLR_UsbClient_EndpointDescriptor  *ep = (TinyCLR_UsbClient_EndpointDescriptor  *)(((uint8_t *)config) + USB_CONFIGURATION_DESCRIPTOR_LENGTH + sizeof(TinyCLR_UsbClient_DescriptorHeader) + sizeof(TinyCLR_UsbClient_InterfaceDescriptor));

    uint8_t * end = ((uint8_t *)config) + config->header.size;

    while (((uint8_t *)ep) != nullptr && ((uint8_t *)ep) < end) {
        if (USB_ENDPOINT_DESCRIPTOR_TYPE != ep->bDescriptorType || sizeof(TinyCLR_UsbClient_EndpointDescriptor) != ep->bLength)
            break;

        auto idx = 0;

        if (ep->bEndpointAddress == USB_ENDPOINT_DIRECTION_IN) {
            ep->bEndpointAddress |= writeEp;
            idx = writeEp;
            usbState->IsTxQueue[idx] = true;
        }

        else if (ep->bEndpointAddress == USB_ENDPOINT_DIRECTION_OUT) {
            ep->bEndpointAddress |= readEp;
            idx = readEp;
            usbState->IsTxQueue[idx] = false;
        }

        if (idx > 0) {
            if (apiProvider != nullptr) {
                auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

                usbState->Queues[idx] = (USB_PACKET64*)memoryProvider->Allocate(memoryProvider, LPC17_USB_FIFO_BUFFER_SIZE * sizeof(USB_PACKET64));

                if (usbState->Queues[idx] == nullptr)
                    return TinyCLR_Result::ArgumentNull;;

                memset(reinterpret_cast<uint8_t*>(usbState->Queues[idx]), 0x00, LPC17_USB_FIFO_BUFFER_SIZE * sizeof(USB_PACKET64));
            }

            LPC17_UsbClient_ClearEndpoints(idx);


            usbState->MaxPacketSize[idx] = ep->wMaxPacketSize;
        }

        ep = (TinyCLR_UsbClient_EndpointDescriptor  *)(((uint8_t *)ep) + ep->bLength);
    }

    if (usbState->CurrentState == USB_DEVICE_STATE_UNINITIALIZED) {
        LPC17_UsbClient_Initialize(usbState);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t pipe) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE * usbState = &usbClientState[controller];

    if (!usbState->Initialized || pipe >= LPC17_USB_QUEUE_SIZE)
        return TinyCLR_Result::NotAvailable;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Close the Rx pipe
    int32_t endpoint = usbState->pipes[pipe].RxEP;
    if (endpoint != USB_ENDPOINT_NULL && usbState->Queues[endpoint]) {
        LPC17_UsbClient_ClearEndpoints(endpoint);
    }

    usbState->pipes[pipe].RxEP = USB_ENDPOINT_NULL;
    //Free endpoint
    LPC17_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    // Close the TX pipe
    endpoint = usbState->pipes[pipe].TxEP;
    if (endpoint != USB_ENDPOINT_NULL && usbState->Queues[endpoint] != nullptr) {
        LPC17_UsbClient_ClearEndpoints(endpoint);

        if (apiProvider != nullptr) {
            auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

            if (usbState->Queues[endpoint] != nullptr)
                memoryProvider->Free(memoryProvider, usbState->Queues[endpoint]);

            usbState->Queues[endpoint] = nullptr;
        }
    }

    usbState->pipes[pipe].TxEP = USB_ENDPOINT_NULL;

    //Free endpoint
    LPC17_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    configDescriptor.epWrite.bEndpointAddress = USB_ENDPOINT_DIRECTION_IN;
    configDescriptor.epRead.bEndpointAddress = USB_ENDPOINT_DIRECTION_OUT;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t pipe, const uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE * usbState = &usbClientState[controller];

    if (pipe >= LPC17_USB_QUEUE_SIZE
        || data == nullptr
        || usbState->DeviceState != USB_DEVICE_STATE_CONFIGURED
        || length == 0) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    int32_t endpoint = usbState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->Queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    DISABLE_INTERRUPTS_SCOPED(irq);

    const uint8_t*      ptr = data;
    uint32_t            count = length;
    bool                Done = false;
    uint32_t            WaitLoopCnt = 0;
    int32_t                 totWrite = 0;

    // This loop packetizes the data and sends it out.  All packets sent have
    // the maximum length for the given endpoint except for the last packet which
    // will always have less than the maximum length - even if the packet length
    // must be zero for this to occur.   This is done to comply with standard
    // USB bulk-mode transfers.
    while (!Done) {

        USB_PACKET64* Packet64 = nullptr;

        if (usb_fifo_buffer_count[endpoint] < LPC17_USB_FIFO_BUFFER_SIZE) {
            Packet64 = &usbState->Queues[endpoint][usb_fifo_buffer_in[endpoint]];

            usb_fifo_buffer_in[endpoint]++;
            usb_fifo_buffer_count[endpoint]++;

            if (usb_fifo_buffer_in[endpoint] == LPC17_USB_FIFO_BUFFER_SIZE)
                usb_fifo_buffer_in[endpoint] = 0;
        }

        if (Packet64) {
            uint32_t max_move;

            if (count > usbState->MaxPacketSize[endpoint])
                max_move = usbState->MaxPacketSize[endpoint];
            else
                max_move = count;

            if (max_move) {
                memcpy(Packet64->Buffer, ptr, max_move);
            }

            // we are done when we send a non-full length packet
            if (max_move < usbState->MaxPacketSize[endpoint]) {
                Done = true;
            }

            Packet64->Size = max_move;
            count -= max_move;
            ptr += max_move;

            totWrite += max_move;

            WaitLoopCnt = 0;
        }
        if (Packet64 == nullptr) {
            // a 64-byte USB packet takes less than 50uSec
            // according to the timing calculations of the USB Chief
            // this is way too short to bother with a call
            // to WaitForEventsInternal, so just uSec delay the path
            // here for 50uSec.

            // if in ISR, return

            // if more than 100*50us=5ms,still no packet avaialable, PC side go wrong,stop the loop
            // otherwise it will spin here forever and stopwatch get kick in.
            WaitLoopCnt++;
            if (WaitLoopCnt > 100) {
                // if we were unable to send any data then no one is listening so lets
                if (count == length) {
                    LPC17_UsbClient_ClearEndpoints(endpoint);
                }

                goto done_write;
            }

            if (irq.WasDisabled()) // @todo - this really needs more checks to be totally valid
            {
                goto done_write;
            }

            if (usbState->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
                goto done_write;
            }

            LPC17_UsbClient_StartOutput(usbState, endpoint);

            irq.Release();

            LPC17_Time_Delay(nullptr, 50);

            irq.Acquire();
        }
    }

    // here we have a post-condition that IRQs are disabled for all paths through conditional block above
    if (usbState->DeviceState == USB_DEVICE_STATE_CONFIGURED) {
        LPC17_UsbClient_StartOutput(usbState, endpoint);
    }

done_write:
    length = totWrite;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t pipe, uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    int32_t endpoint;
    USB_CONTROLLER_STATE * usbState = &usbClientState[controller];

    if (pipe >= LPC17_USB_QUEUE_SIZE
        || usbState->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    endpoint = usbState->pipes[pipe].RxEP;
    // If no Read side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->Queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_PACKET64* Packet64 = nullptr;
    uint8_t*        ptr = (uint8_t*)data;
    uint32_t        count = 0;
    uint32_t        remain = length;

    while (count < length) {
        uint32_t max_move;

        if (usb_fifo_buffer_count[endpoint] > 0) {
            Packet64 = &usbState->Queues[endpoint][usb_fifo_buffer_out[endpoint]];

            usb_fifo_buffer_count[endpoint]--;
            usb_fifo_buffer_out[endpoint]++;

            if (usb_fifo_buffer_out[endpoint] == LPC17_USB_FIFO_BUFFER_SIZE) {
                usb_fifo_buffer_out[endpoint] = 0;
            }
        }

        if (!Packet64) {
            LPC17_UsbClient_ClearEvent(usbState, 1 << endpoint);
            break;
        }

        max_move = Packet64->Size - usbState->CurrentPacketOffset[endpoint];
        if (remain < max_move) max_move = remain;

        memcpy(ptr, &Packet64->Buffer[usbState->CurrentPacketOffset[endpoint]], max_move);

        usbState->CurrentPacketOffset[endpoint] += max_move;
        ptr += max_move;
        count += max_move;
        remain -= max_move;

        /* if we're done with this packet, move onto the next */
        if (usbState->CurrentPacketOffset[endpoint] == Packet64->Size) {
            usbState->CurrentPacketOffset[endpoint] = 0;
            Packet64 = nullptr;

            LPC17_UsbClient_RxEnable(usbState, endpoint);
        }
    }

    length = count;

    return TinyCLR_Result::Success;
}

#define USB_FLUSH_RETRY_COUNT 30
TinyCLR_Result LPC17_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t pipe) {
    int32_t controller = self->Index;

    int32_t endpoint;
    int32_t retries = USB_FLUSH_RETRY_COUNT;
    int32_t queueCnt;
    USB_CONTROLLER_STATE * usbState = &usbClientState[controller];

    if (pipe >= LPC17_USB_QUEUE_SIZE) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    /* not configured, no data can go in or out */
    if (usbState->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::InvalidOperation;
    }

    endpoint = usbState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->Queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    queueCnt = usb_fifo_buffer_count[endpoint];

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while (usb_fifo_buffer_count[endpoint] > 0 && retries > 0) {
        LPC17_UsbClient_StartOutput(usbState, endpoint);

        LPC17_Time_Delay(nullptr, queueCnt == usb_fifo_buffer_count[endpoint] ? 100 : 0); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == usb_fifo_buffer_count[endpoint]) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = usb_fifo_buffer_count[endpoint];
    }

    if (retries <= 0)
        LPC17_UsbClient_ClearEndpoints(endpoint);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    int32_t controller = self->Index;

    LPC17_UsbClient_DataReceivedHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetOsExtendedPropertyHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler) {
    int32_t controller = self->Index;

    LPC17_UsbClient_OsExtendedPropertyHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length) {
    memcpy(&deviceDescriptor, descriptor, length);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length) {
    memcpy(&configDescriptor, descriptor, length);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value) {
    switch (type) {
    case TinyCLR_UsbClient_StringDescriptorType::ManufacturerName:
        memcpy(&stringManufacturerDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
        break;

    case TinyCLR_UsbClient_StringDescriptorType::ProductName:
        memcpy(&stringProductNameDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
        break;

    case TinyCLR_UsbClient_StringDescriptorType::DisplayName:
        memcpy(&stringDisplayNameDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
        break;

    case TinyCLR_UsbClient_StringDescriptorType::FriendlyName:
        memcpy(&stringFriendlyNameDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
        break;
    }

    return TinyCLR_Result::Success;
}

void LPC17_UsbClient_Reset() {
    for (auto pipe = 0; pipe < LPC17_USB_QUEUE_SIZE; pipe++) {
        LPC17_UsbClient_Close(&usbClientProvider, pipe);
    }

    LPC17_UsbClient_Release(&usbClientProvider);
}

///////////////////////////////////////////////////////////////////////////////////////////
/// LPC17 USB Hardware driver
///////////////////////////////////////////////////////////////////////////////////////////
/* Device Interrupt Bit Definitions */
#define FRAME_INT           0x00000001
#define EP_FAST_INT         0x00000002
#define EP_SLOW_INT         0x00000004
#define DEV_STAT_INT        0x00000008
#define CCEMTY_INT          0x00000010
#define CDFULL_INT          0x00000020
#define RxENDPKT_INT        0x00000040
#define TxENDPKT_INT        0x00000080
#define EP_RLZED_INT        0x00000100
#define ERR_INT             0x00000200

/* Rx & Tx Packet Length Definitions */
#define PKT_LNGTH_MASK      0x000003FF
#define PKT_DV              0x00000400
#define PKT_RDY             0x00000800

/* USB Control Definitions */
#define CTRL_RD_EN          0x00000001
#define CTRL_WR_EN          0x00000002

/* Command Codes */
#define CMD_SET_ADDR        0x00D00500
#define CMD_CFG_DEV         0x00D80500
#define CMD_SET_MODE        0x00F30500
#define CMD_RD_FRAME        0x00F50500
#define DAT_RD_FRAME        0x00F50200
#define CMD_RD_TEST         0x00FD0500
#define DAT_RD_TEST         0x00FD0200
#define CMD_SET_DEV_STAT    0x00FE0500
#define CMD_GET_DEV_STAT    0x00FE0500
#define DAT_GET_DEV_STAT    0x00FE0200
#define CMD_GET_ERR_CODE    0x00FF0500
#define DAT_GET_ERR_CODE    0x00FF0200
#define CMD_RD_ERR_STAT     0x00FB0500
#define DAT_RD_ERR_STAT     0x00FB0200
#define DAT_WR_BYTE(x)     (0x00000100 | ((x) << 16))
#define CMD_SEL_EP(x)      (0x00000500 | ((x) << 16))
#define DAT_SEL_EP(x)      (0x00000200 | ((x) << 16))
#define CMD_SEL_EP_CLRI(x) (0x00400500 | ((x) << 16))
#define DAT_SEL_EP_CLRI(x) (0x00400200 | ((x) << 16))
#define CMD_SET_EP_STAT(x) (0x00400500 | ((x) << 16))
#define CMD_CLR_BUF         0x00F20500
#define DAT_CLR_BUF         0x00F20200
#define CMD_VALID_BUF       0x00FA0500

/* Device Address Register Definitions */
#define DEV_ADDR_MASK       0x7F
#define DEV_EN              0x80

/* Device Configure Register Definitions */
#define CONF_DVICE          0x01

/* Device Mode Register Definitions */
#define AP_CLK              0x01
#define INAK_CI             0x02
#define INAK_CO             0x04
#define INAK_II             0x08
#define INAK_IO             0x10
#define INAK_BI             0x20
#define INAK_BO             0x40

/* Device Status Register Definitions */
#define DEV_CON             0x01
#define DEV_CON_CH          0x02
#define DEV_SUS             0x04
#define DEV_SUS_CH          0x08
#define DEV_RST             0x10

/* Error Code Register Definitions */
#define ERR_EC_MASK         0x0F
#define ERR_EA              0x10

/* Error Status Register Definitions */
#define ERR_PID             0x01
#define ERR_UEPKT           0x02
#define ERR_DCRC            0x04
#define ERR_TIMOUT          0x08
#define ERR_EOP             0x10
#define ERR_B_OVRN          0x20
#define ERR_BTSTF           0x40
#define ERR_TGL             0x80

/* Endpoint Select Register Definitions */
#define EP_SEL_F            0x01
#define EP_SEL_ST           0x02
#define EP_SEL_STP          0x04
#define EP_SEL_PO           0x08
#define EP_SEL_EPN          0x10
#define EP_SEL_B_1_FULL     0x20
#define EP_SEL_B_2_FULL     0x40

/* Endpoint Status Register Definitions */
#define EP_STAT_ST          0x01
#define EP_STAT_DA          0x20
#define EP_STAT_RF_MO       0x40
#define EP_STAT_CND_ST      0x80

/* USB registers*/
#define OTGStCtrl (*(volatile uint32_t *)0x2008C110)

#define USBDevIntSt (*(volatile uint32_t *)0x2008C200)
#define USBDevIntEn (*(volatile uint32_t *)0x2008C204)
#define USBDevIntClr (*(volatile uint32_t *)0x2008C208)

#define USBCmdCode (*(volatile uint32_t *)0x2008C210)
#define USBCmdData (*(volatile uint32_t *)0x2008C214)

#define USBRxData (*(volatile uint32_t *)0x2008C218)
#define USBTxData (*(volatile uint32_t *)0x2008C21C)
#define USBRxPLen (*(volatile uint32_t *)0x2008C220)
#define USBTxPLen (*(volatile uint32_t *)0x2008C224)

#define USBCtrl (*(volatile uint32_t *)0x2008C228)

#define USBEpIntSt (*(volatile uint32_t *)0x2008C230)
#define USBEpIntEn (*(volatile uint32_t *)0x2008C234)
#define USBEpIntClr (*(volatile uint32_t *)0x2008C238)
#define USBEpIntSet (*(volatile uint32_t *)0x2008C23C)

#define USBReEp (*(volatile uint32_t *)0x2008C244)
#define USBEpInd (*(volatile uint32_t *)0x2008C248)

#define USBEpMaxPSize (*(volatile uint32_t *)0x2008C24C)

#define USBClkCtrl (*(volatile uint32_t *)0x2008CFF4)
#define USBClkSt (*(volatile uint32_t *)0x2008CFF8)
#define OTGClkCtrl (*(volatile uint32_t *)0x2008CFF4)
#define OTGClkSt (*(volatile uint32_t *)0x2008CFF8)

struct LPC17_UsbClientController {
    USB_CONTROLLER_STATE *usbState;

    uint8_t controlPacketBuffer[USB_MAX_EP0_SIZE];
    uint16_t EndpointStatus[USB_MAX_EP_COUNT];
    bool txRunning[USB_MAX_EP_COUNT];
    bool txNeedZLPS[USB_MAX_EP_COUNT];

    uint8_t previousDeviceState;
    bool firstDescriptorPacket;
};

LPC17_UsbClientController lcp17_UsbClientController[TOTAL_USB_CONTROLLER];

union EndpointConfiguration {
    struct {
        unsigned EE : 1;      // Endpoint enable (1 = enable)
        unsigned DE : 1;      // Double buffer enable (1 = double buffered)
        unsigned MPS : 10;      // Maximum packet size (iso=1-1023, blk=8,16,32,64, int32_t=1-64
        unsigned ED : 1;      // Endpoint direction (1 = IN)
        unsigned ET : 2;      // Endpoint type (1=iso, 2=blk, 3=int32_t)
        unsigned EN : 4;      // Endpoint number (1-15)
        unsigned AISN : 3;      // Alternate Interface number
        unsigned IN : 3;      // Interface number
        unsigned CN : 2;      // Configuration number
    } bits;
    uint32_t word;
};

static EndpointConfiguration EndpointInit[USB_MAX_EP_COUNT];     // Corresponds to endpoint configuration RAM at LPC17xx_USB::UDCCRx
static int32_t nacking_rx_OUT_data[USB_MAX_EP_COUNT];

bool LPC17_UsbClient_ProtectPins(int32_t controller, bool On);
void LPC17_UsbClient_InterruptHandler(void* param);
void LPC17_UsbClient_TxPacket(USB_CONTROLLER_STATE* usbState, int32_t endpoint);
void LPC17_UsbClient_ProcessEP0(USB_CONTROLLER_STATE* usbState, int32_t in, int32_t setup);
void LPC17_UsbClient_ProcessEndPoint(USB_CONTROLLER_STATE* usbState, int32_t ep, int32_t in);
void LPC17_UsbClient_Enpoint_TxInterruptHandler(USB_CONTROLLER_STATE* usbState, uint32_t endpoint);
void LPC17_UsbClient_Enpoint_RxInterruptHandler(USB_CONTROLLER_STATE* usbState, uint32_t endpoint);
void LPC17_UsbClient_ResetEvent(USB_CONTROLLER_STATE* usbState);
void LPC17_UsbClient_SuspendEvent(USB_CONTROLLER_STATE* usbState);
void LPC17_UsbClient_ResumeEvent(USB_CONTROLLER_STATE* usbState);
void LPC17_UsbClient_ControlNext(USB_CONTROLLER_STATE* usbState);
uint32_t LPC17_UsbClient_EPAdr(uint32_t EPNum, int8_t in);

bool LPC17_UsbClient_Initialize(USB_CONTROLLER_STATE *usbState) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (usbState == nullptr)
        return false;

    int32_t controller = usbState->controllerNum;

    lcp17_UsbClientController[controller].usbState = usbState;

    LPC17_Interrupt_Activate(USB_IRQn, (uint32_t*)&LPC17_UsbClient_InterruptHandler, 0);

    for (int32_t i = 0; i < USB_MAX_EP_COUNT; i++)
        EndpointInit[i].word = 0;       // All useable endpoints initialize to unused

    for (auto pipe = 0; pipe < USB_MAX_EP_COUNT; pipe++) {
        auto idx = 0;
        if (usbState->pipes[pipe].RxEP != USB_ENDPOINT_NULL) {
            idx = usbState->pipes[pipe].RxEP;
            EndpointInit[idx].bits.ED = 0;
            EndpointInit[idx].bits.DE = 0;
        }

        if (usbState->pipes[pipe].TxEP != USB_ENDPOINT_NULL) {
            idx = usbState->pipes[pipe].TxEP;
            EndpointInit[idx].bits.ED = 1;
            EndpointInit[idx].bits.DE = 1;
        }

        if (idx != 0) {
            EndpointInit[idx].bits.EN = idx;
            EndpointInit[idx].bits.IN = 0;//itfc->bInterfaceNumber;
            EndpointInit[idx].bits.ET = USB_ENDPOINT_ATTRIBUTE_BULK & 0x03; //ep->bmAttributes & 0x03;
            EndpointInit[idx].bits.CN = 1;        // Always only 1 configuration = 1
            EndpointInit[idx].bits.AISN = 0;        // No alternate interfaces
            EndpointInit[idx].bits.EE = 1;        // Enable this endpoint
            EndpointInit[idx].bits.MPS = usbState->MaxPacketSize[idx];
        }
    }

    usbState->EndpointStatus = &lcp17_UsbClientController[controller].EndpointStatus[0];
    usbState->EndpointCount = USB_MAX_EP_COUNT;
    usbState->PacketSize = USB_MAX_EP0_SIZE;

    usbState->FirstGetDescriptor = true;

    LPC17_UsbClient_ProtectPins(controller, true);

    return true;
}

bool LPC17_UsbClient_Uninitialize(USB_CONTROLLER_STATE *usbState) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17_Interrupt_Deactivate(USB_IRQn);

    if (usbState != nullptr) {
        LPC17_UsbClient_ProtectPins(usbState->controllerNum, false);
        usbState->CurrentState = USB_DEVICE_STATE_UNINITIALIZED;
    }

    return true;
}

bool LPC17_UsbClient_StartOutput(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    int32_t m, n, val;

    DISABLE_INTERRUPTS_SCOPED(irq);

    /* if the halt feature for this endpoint is set, then just
       clear all the characters */
    if (usbState->EndpointStatus[endpoint] & USB_STATUS_ENDPOINT_HALT) {
        LPC17_UsbClient_ClearEndpoints(endpoint);
        return true;
    }

    //If txRunning, interrupts will drain the queue
    if (!lcp17_UsbClientController[usbState->controllerNum].txRunning[endpoint]) {
        lcp17_UsbClientController[usbState->controllerNum].txRunning[endpoint] = true;

        // Calling both LPC17_UsbClient_TxPacket & EP_TxISR in this routine could cause a TX FIFO overflow
        LPC17_UsbClient_TxPacket(usbState, endpoint);
    }
    else if (irq.WasDisabled()) {

        n = LPC17_UsbClient_EPAdr(endpoint, 1); // It is an output endpoint for sure
        if ((USBEpIntSt & (1 << n)))//&& (USBEpIntEn & (1 << n)) )//only if enabled
        {
            m = n >> 1;

            if (m == 0)//EP0
            {
                USBEpIntClr = 1 << n;
                while ((USBDevIntSt & CDFULL_INT) == 0);
                val = USBCmdData;

                if (val & EP_SEL_STP)        /* Setup Packet */
                {
                    LPC17_UsbClient_ProcessEP0(usbState, 0, 1);// out setup
                }
                else {
                    if ((n & 1) == 0)                /* OUT Endpoint */
                    {
                        LPC17_UsbClient_ProcessEP0(usbState, 0, 0);// out not setup
                    }
                    else {
                        LPC17_UsbClient_ProcessEP0(usbState, 1, 0);// in not setup
                    }
                }
            }
            else {
                if (usbState->Queues[m] && usbState->IsTxQueue[endpoint])
                    LPC17_UsbClient_ProcessEndPoint(usbState, m, 1);//out
                else
                    LPC17_UsbClient_ProcessEndPoint(usbState, m, 0);//in
            }
        }
    }

    return true;
}

bool LPC17_UsbClient_RxEnable(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    if (endpoint >= USB_MAX_EP_COUNT)
        return false;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (nacking_rx_OUT_data[endpoint])
        LPC17_UsbClient_Enpoint_RxInterruptHandler(usbState, endpoint);//force interrupt to read the pending EP

    return true;
}

static uint8_t  LPC17_UsbClient_DeviceAddress = 0;

#define CONTORL_EP_ADDR	0x80

#define USB_POWER           0
#define USB_IF_NUM          4
#define USB_EP_NUM          32
#define USB_MAX_PACKET0     64
#define USB_DMA             0
#define USB_DMA_EP          0x00000000

#define USB_POWER_EVENT     0
#define USB_RESET_EVENT     1
#define USB_WAKEUP_EVENT    0
#define USB_SOF_EVENT       0
#define USB_ERROR_EVENT     0
#define USB_EP_EVENT        0x0003
#define USB_CONFIGURE_EVENT 1
#define USB_INTERFACE_EVENT 0
#define USB_FEATURE_EVENT   0

#define EP_MSK_CTRL 0x0001      /* Control Endpoint Logical Address Mask */
#define EP_MSK_BULK 0xC924      /* Bulk Endpoint Logical Address Mask */
#define EP_MSK_INT  0x4492      /* Interrupt Endpoint Logical Address Mask */
#define EP_MSK_ISO  0x1248      /* Isochronous Endpoint Logical Address Mask */

static void LPC17_UsbClient_WrCmd(uint32_t cmd) {
    USBDevIntClr = CCEMTY_INT | CDFULL_INT;
    USBCmdCode = cmd;
    while ((USBDevIntSt & CCEMTY_INT) == 0);
}
static void LPC17_UsbClient_WrCmdDat(uint32_t cmd, uint32_t val) {
    USBDevIntClr = CCEMTY_INT;
    USBCmdCode = cmd;
    while ((USBDevIntSt & CCEMTY_INT) == 0);
    USBDevIntClr = CCEMTY_INT;
    USBCmdCode = val;
    while ((USBDevIntSt & CCEMTY_INT) == 0);
}
static uint32_t LPC17_UsbClient_RdCmdDat(uint32_t cmd) {
    USBDevIntClr = CCEMTY_INT | CDFULL_INT;
    USBCmdCode = cmd;
    while ((USBDevIntSt & CDFULL_INT) == 0);
    return (USBCmdData);
}

static void LPC17_UsbClient_SetAddress(uint32_t adr) {
    LPC17_UsbClient_WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /* Don't wait for next */
    LPC17_UsbClient_WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /*  Setup Status Phase */
}

static void LPC17_UsbClient_HardwareReset(void) {
    USBEpInd = 0;
    USBEpMaxPSize = USB_MAX_PACKET0;
    USBEpInd = 1;
    USBEpMaxPSize = USB_MAX_PACKET0;

    while ((USBDevIntSt & EP_RLZED_INT) == 0);

    USBEpIntClr = 0xFFFFFFFF;
    USBEpIntEn = 0xFFFFFFFF ^ USB_DMA_EP;
    USBDevIntClr = 0xFFFFFFFF;
    USBDevIntEn = DEV_STAT_INT | EP_SLOW_INT |
        (USB_SOF_EVENT ? FRAME_INT : 0) |
        (USB_ERROR_EVENT ? ERR_INT : 0);

}

void LPC17_UsbClient_Connect(bool con) {
    LPC17_UsbClient_WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(con ? DEV_CON : 0));
}

uint32_t LPC17_UsbClient_EPAdr(uint32_t EPNum, int8_t in) {
    uint32_t val;

    val = (EPNum & 0x0F) << 1;
    if (in) {
        val += 1;
    }
    return (val);
}

static uint32_t USB_WriteEP(uint32_t EPNum, uint8_t *pData, uint32_t cnt) {
    uint32_t n, g;
    USBCtrl = ((EPNum & 0x0F) << 2) | CTRL_WR_EN;

    USBTxPLen = cnt;

    for (n = 0; n < (cnt + 3) / 4; n++) {
        g = *(pData + 3);
        g <<= 8;
        g |= *(pData + 2);
        g <<= 8;
        g |= *(pData + 1);
        g <<= 8;
        g |= *pData;
        USBTxData = g;
        pData += 4;
    }

    USBCtrl = 0;

    LPC17_UsbClient_WrCmd(CMD_SEL_EP(LPC17_UsbClient_EPAdr(EPNum, 1)));
    LPC17_UsbClient_WrCmd(CMD_VALID_BUF);

    return (cnt);
}

static uint32_t LPC17_UsbClient_ReadEP(uint32_t EPNum, uint8_t *pData) {
    uint32_t cnt, n, d;

    USBCtrl = ((EPNum & 0x0F) << 2) | CTRL_RD_EN;

    do {
        cnt = USBRxPLen;
    } while ((cnt & PKT_RDY) == 0);

    cnt &= PKT_LNGTH_MASK;

    for (n = 0; n < (cnt + 3) / 4; n++) {
        d = USBRxData;
        *pData++ = d;
        *pData++ = d >> 8;
        *pData++ = d >> 16;
        *pData++ = d >> 24;
    }

    USBCtrl = 0;

    if (((EP_MSK_ISO >> EPNum) & 1) == 0)    /* Non-Isochronous Endpoint */
    {
        LPC17_UsbClient_WrCmd(CMD_SEL_EP(LPC17_UsbClient_EPAdr(EPNum, 0)));
        LPC17_UsbClient_WrCmd(CMD_CLR_BUF);
    }

    return (cnt);
}

static void LPC17_UsbClient_SetStallEP(uint32_t EPNum, int8_t in) {
    LPC17_UsbClient_WrCmdDat(CMD_SET_EP_STAT(LPC17_UsbClient_EPAdr(EPNum, in)), DAT_WR_BYTE(EP_STAT_ST));
}

void LPC17_UsbClient_ProcessEndPoint(USB_CONTROLLER_STATE* usbState, int32_t ep, int32_t in) {
    int32_t val;

    if (in) {
        LPC17_UsbClient_Enpoint_TxInterruptHandler(usbState, ep);
    }
    else {
        USBEpIntClr = 1 << LPC17_UsbClient_EPAdr(ep, in);
        while ((USBDevIntSt & CDFULL_INT) == 0);
        val = USBCmdData;
        LPC17_UsbClient_Enpoint_RxInterruptHandler(usbState, ep);
    }
}

void LPC17_UsbClient_ConfigEP(uint8_t ep_addr, int8_t in, uint8_t size) {
    uint32_t num;

    num = LPC17_UsbClient_EPAdr(ep_addr, in);
    USBReEp |= (1 << num);
    USBEpInd = num;
    USBEpMaxPSize = size;

    while ((USBDevIntSt & EP_RLZED_INT) == 0);

    USBDevIntClr = EP_RLZED_INT;
}
void LPC17_UsbClient_EnableEP(uint32_t EPNum, int8_t in) {
    LPC17_UsbClient_WrCmdDat(CMD_SET_EP_STAT(LPC17_UsbClient_EPAdr(EPNum, in)), DAT_WR_BYTE(0));
}
void USB_DisableEP(int32_t EPNum, int8_t in) {
    LPC17_UsbClient_WrCmdDat(CMD_SET_EP_STAT(LPC17_UsbClient_EPAdr(EPNum, in)), DAT_WR_BYTE(EP_STAT_DA));
}
void LPC17_UsbClient_ResetEP(uint32_t EPNum, int8_t in) {
    LPC17_UsbClient_WrCmdDat(CMD_SET_EP_STAT(LPC17_UsbClient_EPAdr(EPNum, in)), DAT_WR_BYTE(0));
}

void USB_HW_Configure(bool cfg) {
    LPC17_UsbClient_WrCmdDat(CMD_CFG_DEV, DAT_WR_BYTE(cfg ? CONF_DVICE : 0));

    USBReEp = 0x00000003;
    while ((USBDevIntSt & EP_RLZED_INT) == 0);
    USBDevIntClr = EP_RLZED_INT;
}

void LPC17_UsbClient_StartHardware() {
    *(uint32_t*)0x400FC0C4 |= 0x80000000;
    USBClkCtrl = (1 << 1) | (1 << 3) | (1 << 4);

    OTGClkCtrl = 0x1F;
    while ((OTGClkSt & 0x1F) != 0x1F);

    LPC17_Gpio_ConfigurePin(14, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction3, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(31, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction1, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    OTGStCtrl |= 3;

    LPC17_UsbClient_HardwareReset();
    LPC17_UsbClient_SetAddress(0);

    USBDevIntEn = DEV_STAT_INT;	/* Enable Device Status Interrupt */

    LPC17_UsbClient_Connect(false);
    // delay if removed and then connected...
    LPC17_Time_Delay(nullptr, 120 * 1000);

    LPC17_UsbClient_Connect(true);
}

void LPC17_UsbClient_StopHardware() {
    LPC17_UsbClient_Connect(false);
}

void LPC17_UsbClient_TxPacket(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    // transmit a packet on UsbPortNum, if there are no more packets to transmit, then die
    USB_PACKET64* Packet64;

    for (;;) {
        Packet64 = LPC17_UsbClient_TxDequeue(usbState, endpoint);

        if (Packet64 == nullptr || Packet64->Size > 0) {
            break;
        }
    }

    if (Packet64) {

        USB_WriteEP(endpoint, Packet64->Buffer, Packet64->Size);

        lcp17_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint] = false;
        if (Packet64->Size == 64)
            lcp17_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint] = true;
    }
    else {
        // send the zero length packet since we landed on the FIFO boundary before
        // (and we queued a zero length packet to transmit)
        if (lcp17_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint]) {
            USB_WriteEP(endpoint, (uint8_t*)nullptr, 0);
            lcp17_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint] = false;
        }

        // no more data
        lcp17_UsbClientController[usbState->controllerNum].txRunning[endpoint] = false;
    }
}
void LPC17_UsbClient_ControlNext(USB_CONTROLLER_STATE *usbState) {
    if (usbState->DataCallback) {
        // this call can't fail
        usbState->DataCallback(usbState);

        if (usbState->DataSize == 0) {
            USB_WriteEP(CONTORL_EP_ADDR, (uint8_t*)nullptr, 0);
            usbState->DataCallback = nullptr;                         // Stop sending stuff if we're done
        }
        else {
            USB_WriteEP(CONTORL_EP_ADDR, usbState->Data, usbState->DataSize);

            if (usbState->DataSize < USB_MAX_EP0_SIZE) // If packet is less than full length
            {
                usbState->DataCallback = nullptr; // Stop sending stuff if we're done
            }

            // special handling the USB driver set address test, cannot use the first descriptor as the ADDRESS state is handle in the hardware
            if (lcp17_UsbClientController[usbState->controllerNum].firstDescriptorPacket) {
                usbState->DataCallback = nullptr;
            }

        }
    }
}

void LPC17_UsbClient_InterruptHandler(void* param) {
    DISABLE_INTERRUPTS_SCOPED(irq);
    int32_t disr, val, n, m;

    disr = USBDevIntSt;                      /* Device Interrupt Status */
    USBDevIntClr = disr;                       /* A known issue on LPC214x */

    USB_CONTROLLER_STATE *usbState = lcp17_UsbClientController[USB_USBCLIENT_ID].usbState;

    if (disr & DEV_STAT_INT) {
        LPC17_UsbClient_WrCmd(CMD_GET_DEV_STAT);
        val = LPC17_UsbClient_RdCmdDat(DAT_GET_DEV_STAT);       /* Device Status */

        if (val & DEV_RST)                     /* Reset */
        {
            LPC17_UsbClient_ResetEvent(usbState);
        }

        if (val & DEV_SUS_CH)                  /* Suspend/Resume */
        {
            if (val & DEV_SUS)                   /* Suspend */
            {
                LPC17_UsbClient_SuspendEvent(usbState);
            }
            else                               /* Resume */
            {
                LPC17_UsbClient_ResumeEvent(usbState);
            }
        }

        goto isr_end;
    }

    /* Endpoint's Slow Interrupt */
    if (disr & EP_SLOW_INT) {
        for (n = 0; n < USB_EP_NUM; n++)     /* Check All Endpoints */
        {
            if ((USBEpIntSt & (1 << n))) {
                m = n >> 1;

                if (m == 0)//EP0
                {
                    USBEpIntClr = 1 << n;
                    while ((USBDevIntSt & CDFULL_INT) == 0);
                    val = USBCmdData;

                    if (val & EP_SEL_STP)        /* Setup Packet */
                    {
                        LPC17_UsbClient_ProcessEP0(usbState, 0, 1);// out setup
                        continue;
                    }
                    if ((n & 1) == 0)                /* OUT Endpoint */
                    {
                        LPC17_UsbClient_ProcessEP0(usbState, 0, 0);// out not setup
                    }
                    else {
                        LPC17_UsbClient_ProcessEP0(usbState, 1, 0);// in not setup
                    }

                    continue;
                }
                if ((n & 1) == 0)                /* OUT Endpoint */
                {
                    LPC17_UsbClient_ProcessEndPoint(usbState, m, 0);//out
                }
                else                           /* IN Endpoint */
                {
                    LPC17_UsbClient_ProcessEndPoint(usbState, m, 1);//in
                }
            }
        }
    }
isr_end:
    return;
}

void LPC17_UsbClient_ProcessEP0(USB_CONTROLLER_STATE *usbState, int32_t in, int32_t setup) {
    uint32_t EP_INTR;
    int32_t i;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (setup) {
        uint8_t   len = 0;

        len = LPC17_UsbClient_ReadEP(0x00, lcp17_UsbClientController[usbState->controllerNum].controlPacketBuffer);

        // special handling for the very first SETUP command - Getdescriptor[DeviceType], the host looks for 8 bytes data only
        USB_SETUP_PACKET* Setup = (USB_SETUP_PACKET*)&lcp17_UsbClientController[usbState->controllerNum].controlPacketBuffer[0];
        if ((Setup->bRequest == USB_GET_DESCRIPTOR) && (((Setup->wValue & 0xFF00) >> 8) == USB_DEVICE_DESCRIPTOR_TYPE) && (Setup->wLength != 0x12))
            lcp17_UsbClientController[usbState->controllerNum].firstDescriptorPacket = true;
        else
            lcp17_UsbClientController[usbState->controllerNum].firstDescriptorPacket = false;

        // send it to the upper layer
        usbState->Data = &lcp17_UsbClientController[usbState->controllerNum].controlPacketBuffer[0];
        usbState->DataSize = len;

        uint8_t result = LPC17_UsbClient_ControlCallback(usbState);

        switch (result) {
        case USB_STATE_ADDRESS:
            LPC17_UsbClient_DeviceAddress = usbState->Address | 0x80;
            break;

        case USB_STATE_DONE:
            usbState->DataCallback = nullptr;
            break;

        case USB_STATE_STALL:
            LPC17_UsbClient_SetStallEP(0, 0);
            LPC17_UsbClient_SetStallEP(0, 1);
            break;

        case USB_STATE_CONFIGURATION:
            USB_HW_Configure(true);
            for (i = 1; i < 16; i++) {
                // direction in
                LPC17_UsbClient_ConfigEP(i, 1, 64);
                LPC17_UsbClient_EnableEP(i, 1);
                LPC17_UsbClient_ResetEP(i, 1);

                // direction out
                LPC17_UsbClient_ConfigEP(i, 0, 64);
                LPC17_UsbClient_EnableEP(i, 0);
                LPC17_UsbClient_ResetEP(i, 0);
            }

            break;

        }

        if (result != USB_STATE_STALL) {
            LPC17_UsbClient_ControlNext(usbState);

            // If the port is configured, then output any possible withheld data
            if (result == USB_STATE_CONFIGURATION) {
                for (int32_t ep = 0; ep < USB_MAX_EP_COUNT; ep++) {
                    if (usbState->IsTxQueue[ep])
                        LPC17_UsbClient_StartOutput(usbState, ep);
                }
            }
        }
    }
    else if (in) {
        // If previous packet has been sent and UDC is ready for more
        LPC17_UsbClient_ControlNext(usbState);      // See if there is more to send

        if (LPC17_UsbClient_DeviceAddress & 0x80) {
            LPC17_UsbClient_DeviceAddress &= 0x7F;
            LPC17_UsbClient_SetAddress(LPC17_UsbClient_DeviceAddress);
        }
    }
}

void LPC17_UsbClient_Enpoint_TxInterruptHandler(USB_CONTROLLER_STATE *usbState, uint32_t endpoint) {
    uint32_t EP_INTR;
    int32_t val;

    if (USBEpIntSt & (1 << LPC17_UsbClient_EPAdr(endpoint, 1)))//done sending?
    {
        //clear interrupt flag
        USBEpIntClr = 1 << LPC17_UsbClient_EPAdr(endpoint, 1);
        while ((USBDevIntSt & CDFULL_INT) == 0);
        val = USBCmdData;

        // successfully transmitted packet, time to send the next one
        LPC17_UsbClient_TxPacket(usbState, endpoint);
    }
}

void LPC17_UsbClient_Enpoint_RxInterruptHandler(USB_CONTROLLER_STATE *usbState, uint32_t endpoint) {
    bool          DisableRx;
    USB_PACKET64* Packet64 = LPC17_UsbClient_RxEnqueue(usbState, endpoint, DisableRx);

    /* copy packet in, making sure that Packet64->Buffer is never overflowed */
    if (Packet64) {
        uint8_t   len = 0;//USB.UDCBCRx[EPno] & LPC17xx_USB::UDCBCR_mask;
        uint32_t* packetBuffer = (uint32_t*)Packet64->Buffer;
        len = LPC17_UsbClient_ReadEP(endpoint, Packet64->Buffer);

        // clear packet status
        nacking_rx_OUT_data[endpoint] = 0;
        Packet64->Size = len;
    }
    else {
        /* flow control should absolutely protect us from ever
        getting here, so if we do, it is a bug */
        nacking_rx_OUT_data[endpoint] = 1;//we will need to triger next interrupt
    }

}

void LPC17_UsbClient_SuspendEvent(USB_CONTROLLER_STATE *usbState) {
    // SUSPEND event only happened when Host(PC) set the device to SUSPEND
    // as there is always SOF every 1ms on the BUS to keep the device from
    // suspending. Therefore, the REMOTE wake up is not necessary at the ollie side
    lcp17_UsbClientController[usbState->controllerNum].previousDeviceState = usbState->DeviceState;

    usbState->DeviceState = USB_DEVICE_STATE_SUSPENDED;

    LPC17_UsbClientControllerCallback(usbState);
}


void LPC17_UsbClient_ResumeEvent(USB_CONTROLLER_STATE *usbState) {
    usbState->DeviceState = lcp17_UsbClientController[usbState->controllerNum].previousDeviceState;

    LPC17_UsbClientControllerCallback(usbState);
}

void LPC17_UsbClient_ResetEvent(USB_CONTROLLER_STATE *usbState) {
    LPC17_UsbClient_HardwareReset();
    LPC17_UsbClient_DeviceAddress = 0;

    // clear all flags
    LPC17_UsbClient_ClearEvent(usbState, 0xFFFFFFFF);

    for (int32_t ep = 0; ep < USB_MAX_EP_COUNT; ep++) {
        lcp17_UsbClientController[usbState->controllerNum].txRunning[ep] = false;
        lcp17_UsbClientController[usbState->controllerNum].txNeedZLPS[ep] = false;
    }

    usbState->DeviceState = USB_DEVICE_STATE_DEFAULT;
    usbState->Address = 0;
    LPC17_UsbClientControllerCallback(usbState);
}

bool LPC17_UsbClient_ProtectPins(int32_t controller, bool On) {
    USB_CONTROLLER_STATE *usbState = lcp17_UsbClientController[controller].usbState;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (usbState) {
        if (On) {
            usbState->DeviceState = USB_DEVICE_STATE_ATTACHED;

            LPC17_UsbClientControllerCallback(usbState);

            LPC17_UsbClient_StartHardware();
        }
        else {
            LPC17_UsbClient_HardwareReset();

            LPC17_UsbClient_DeviceAddress = 0;

            LPC17_UsbClient_StopHardware();
        }

        return true;
    }

    return false;
}

