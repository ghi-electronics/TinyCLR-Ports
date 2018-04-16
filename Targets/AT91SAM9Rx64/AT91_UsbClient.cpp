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

#include "AT91.h"

#define __min(a,b)  (((a) < (b)) ? (a) : (b))
#if defined(__GNUC__)
#define PACKED(x)       x __attribute__((packed))
#elif defined(arm) || defined(__arm)
#define PACKED(x)       __packed x
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
    bool                                                        initialized;
    uint8_t                                                     currentState;
    uint8_t                                                     controllerNum;
    uint32_t                                                    event;

    const USB_DYNAMIC_CONFIGURATION*                            configuration;

    /* Queues & MaxPacketSize must be initialized by the HAL */
    USB_PACKET64                                                *queues[AT91_USB_QUEUE_SIZE];
    uint8_t                                                     currentPacketOffset[AT91_USB_QUEUE_SIZE];
    uint8_t                                                     maxPacketSize[AT91_USB_QUEUE_SIZE];
    bool                                                        isTxQueue[AT91_USB_QUEUE_SIZE];

    /* Arbitrarily as many pipes as endpoints since that is the maximum number of pipes
       necessary to represent the maximum number of endpoints */
    USB_PIPE_MAP                                                pipes[AT91_USB_QUEUE_SIZE];


    /* used for transferring packets between upper & lower */
    uint8_t*                                                    ptrData;
    uint8_t                                                     dataSize;

    /* USB hardware information */
    uint8_t                                                     address;
    uint8_t                                                     deviceState;
    uint8_t                                                     packetSize;
    uint8_t                                                     configurationNum;
    uint32_t                                                    firstGetDescriptor;

    /* USB status information, used in
       GET_STATUS, SET_FEATURE, CLEAR_FEATURE */
    uint16_t                                                    deviceStatus;

    uint16_t                                                    endpointType;
    uint16_t*                                                   endpointStatus;
    uint8_t                                                     endpointCount;
    uint8_t                                                     endpointStatusChange;

    /* callback function for getting next packet */
    USB_NEXT_CALLBACK                                           dataCallback;

    /* for helping out upper layer during callbacks */
    uint8_t*                                                    residualData;
    uint16_t                                                    residualCount;
    uint16_t                                                    expected;
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
void AT91_UsbClient_ClearEndpoints(int32_t endpoint);
bool AT91_UsbClient_RxEnable(int32_t endpoint);

TinyCLR_UsbClient_DataReceivedHandler AT91_UsbClient_DataReceivedHandler;
TinyCLR_UsbClient_OsExtendedPropertyHandler AT91_UsbClient_OsExtendedPropertyHandler;

// usb fifo buffer
static int32_t usb_fifo_buffer_in[AT91_USB_QUEUE_SIZE];
static int32_t usb_fifo_buffer_out[AT91_USB_QUEUE_SIZE];
static int32_t usb_fifo_buffer_count[AT91_USB_QUEUE_SIZE];

int8_t AT91_UsbClient_EndpointMap[] = { ENDPOINT_INUSED_MASK,                          // Endpoint 0
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 1
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 2
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK   // Endpoint 3
};

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

// configuration descriptor
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
TinyCLR_UsbClient_OsStringDescriptor AT91_UsbClient_OsStringDescriptor;

// OS Extended Compatible ID for WinUSB
TinyCLR_UsbClient_XCompatibleOsId AT91_UsbClient_XCompatibleOsId;

// OS Extended Property
TinyCLR_UsbClient_XPropertiesOsWinUsb AT91_UsbClient_XPropertiesOsWinUsb;

// End of configuration marker
const TinyCLR_UsbClient_DescriptorHeader usbDescriptorHeader = {
    USB_END_DESCRIPTOR_MARKER,
    0,
    0
};

void AT91_UsbClient_SetEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t old_event = usbState->event;

    usbState->event |= event;

    if (old_event != usbState->event) {
        AT91_UsbClient_DataReceivedHandler(nullptr);
    }
}

void AT91_UsbClient_ClearEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    usbState->event &= ~event;
}

void AT91_UsbClient_ClearQueues(USB_CONTROLLER_STATE *usbState, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (ClrRxQueue) {
        for (int32_t endpoint = 0; endpoint < AT91_USB_QUEUE_SIZE; endpoint++) {
            if (usbState->queues[endpoint] == nullptr || usbState->isTxQueue[endpoint])
                continue;

            AT91_UsbClient_ClearEndpoints(endpoint);

            /* since this queue is now reset, we have room available for newly arrived packets */
            AT91_UsbClient_RxEnable(endpoint);
        }
    }

    if (ClrTxQueue) {
        for (int32_t endpoint = 0; endpoint < AT91_USB_QUEUE_SIZE; endpoint++) {
            if (usbState->queues[endpoint] && usbState->isTxQueue[endpoint])
                AT91_UsbClient_ClearEndpoints(endpoint);
        }
    }
}

void AT91_UsbClient_StateCallback(USB_CONTROLLER_STATE* usbState) {
    if (usbState->currentState != usbState->deviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
//Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
// The RxQueue is clear when it is configured.

        if (USB_DEVICE_STATE_CONFIGURED == usbState->currentState) {
            AT91_UsbClient_ClearQueues(usbState, true, true);
        }

        usbState->currentState = usbState->deviceState;

        switch (usbState->deviceState) {
        case USB_DEVICE_STATE_DETACHED:
            usbState->residualCount = 0;
            usbState->dataCallback = nullptr;

            break;
        case USB_DEVICE_STATE_CONFIGURED:
            /* whenever we enter the configured state, re-initialize all of the RxQueues */
            /* Txqueue has stored some data to be transmitted */
            AT91_UsbClient_ClearQueues(usbState, true, false);
            break;
        }
    }
}

void AT91_UsbClient_DataCallback(USB_CONTROLLER_STATE* usbState) {
    uint32_t length = __min(usbState->packetSize, usbState->residualCount);

    memcpy(usbState->ptrData, usbState->residualData, length);

    usbState->dataSize = length;
    usbState->residualData += length;
    usbState->residualCount -= length;

    if (length == usbState->packetSize) {
        usbState->expected -= length;
    }
    else {
        usbState->expected = 0;
    }

    if (usbState->expected) {
        usbState->dataCallback = AT91_UsbClient_DataCallback;
    }
    else {
        usbState->dataCallback = nullptr;
    }
}

uint8_t AT91_UsbClient_HandleGetStatus(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    uint16_t* status;
    uint16_t  zero = 0;

    /* validate setup packet */
    if (Setup->wValue != 0 || Setup->wLength != 2) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        status = &usbState->deviceStatus;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
            return USB_STATE_STALL;
        }

        status = &zero;
        break;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->wIndex &= 0x7F;

        if (Setup->wIndex >= usbState->endpointCount) {
            return USB_STATE_STALL;
        }

        status = &usbState->endpointStatus[Setup->wIndex];
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send requested status to host */
    usbState->residualData = (uint8_t*)status;
    usbState->residualCount = 2;
    usbState->dataCallback = AT91_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t AT91_UsbClient_HandleClearFeature(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
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
            usbState->deviceStatus &= ~USB_STATUS_DEVICE_REMOTE_WAKEUP;
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
        if (usbState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0)
            return USB_STATE_STALL;

        /* bit 0x80 designates direction, which we dont utilize in this calculation */
        Setup->wIndex &= 0x7F;

        if (Setup->wIndex == 0 || Setup->wIndex >= usbState->endpointCount)
            return USB_STATE_STALL;

        if (Setup->wValue != USB_FEATURE_ENDPOINT_HALT)
            return USB_STATE_STALL;

        /* clear the halt feature */
        usbState->endpointStatus[Setup->wIndex] &= ~USB_STATUS_ENDPOINT_HALT;
        usbState->endpointStatusChange = Setup->wIndex;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = AT91_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t AT91_UsbClient_HandleSetFeature(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
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
            usbState->deviceStatus |= USB_STATUS_DEVICE_REMOTE_WAKEUP;
        }

        retState = USB_STATE_REMOTE_WAKEUP;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        /* there are no interface features to set */
        return USB_STATE_STALL;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->wIndex &= 0x7F;

        if (Setup->wIndex == 0 || Setup->wIndex >= usbState->endpointCount) {
            return USB_STATE_STALL;
        }

        if (Setup->wValue != USB_FEATURE_ENDPOINT_HALT) {
            return USB_STATE_STALL;
        }

        /* set the halt feature */
        usbState->endpointStatus[Setup->wIndex] |= USB_STATUS_ENDPOINT_HALT;
        usbState->endpointStatusChange = Setup->wIndex;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = AT91_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t AT91_UsbClient_HandleSetAddress(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    /* validate setup packet */
    if (Setup->wValue > 127 || Setup->wIndex != 0 || Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState >= USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    /* set address */
    usbState->address = Setup->wValue;

    /* catch state changes */
    if (usbState->address == 0) {
        usbState->deviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else {
        usbState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }

    AT91_UsbClient_StateCallback(usbState);

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = AT91_UsbClient_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t AT91_UsbClient_HandleConfigurationRequests(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    const TinyCLR_UsbClient_DescriptorHeader * header;
    uint8_t       type;
    uint8_t       DescriptorIndex;

    /* this request is valid regardless of device state */
    type = ((Setup->wValue & 0xFF00) >> 8);
    DescriptorIndex = (Setup->wValue & 0x00FF);
    usbState->expected = Setup->wLength;

    if (usbState->expected == 0) {
        // just return an empty Status packet
        usbState->residualCount = 0;
        usbState->dataCallback = AT91_UsbClient_DataCallback;
        return USB_STATE_DATA;
    }

    // The very first GET_DESCRIPTOR command out of reset should always return at most packetSize bytes.
    // After that, you can return as many as the host has asked.
    if (usbState->deviceState <= USB_DEVICE_STATE_DEFAULT) {
        if (usbState->firstGetDescriptor) {
            usbState->firstGetDescriptor = false;

            usbState->expected = __min(usbState->expected, usbState->packetSize);
        }
    }

    usbState->residualData = nullptr;
    usbState->residualCount = 0;

    if (Setup->bRequest == USB_GET_DESCRIPTOR) {
        switch (type) {
        case USB_DEVICE_DESCRIPTOR_TYPE:
            header = USB_FindRecord(usbState, USB_DEVICE_DESCRIPTOR_MARKER, Setup);
            if (header) {
                const TinyCLR_UsbClient_DeviceDescriptor * device = (TinyCLR_UsbClient_DeviceDescriptor *)header;
                usbState->residualData = (uint8_t *)&device->bLength;      // Start of the device descriptor
                usbState->residualCount = __min(usbState->expected, device->bLength);
            }
            break;

        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            header = USB_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
            if (header) {
                const TinyCLR_UsbClient_ConfigurationDescriptor * Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)header;
                usbState->residualData = (uint8_t *)&Config->bLength;
                usbState->residualCount = __min(usbState->expected, Config->wTotalLength);
            }
            break;

        case USB_STRING_DESCRIPTOR_TYPE:
            if (DescriptorIndex == 0)        // If host is requesting the language list
            {
                usbState->residualData = USB_LanguageDescriptor;
                usbState->residualCount = __min(usbState->expected, USB_LANGUAGE_DESCRIPTOR_SIZE);
            }
            else if (nullptr != (header = USB_FindRecord(usbState, USB_STRING_DESCRIPTOR_MARKER, Setup))) {
                const TinyCLR_UsbClient_StringDescriptorHeader * string = (TinyCLR_UsbClient_StringDescriptorHeader *)header;
                usbState->residualData = (uint8_t *)&string->bLength;
                usbState->residualCount = __min(usbState->expected, string->bLength);
            }
            break;

        default:
            break;
        }
    }

    // If the request was not recognized, the generic types should be searched
    if (usbState->residualData == nullptr) {
        if (nullptr != (header = USB_FindRecord(usbState, USB_GENERIC_DESCRIPTOR_MARKER, Setup))) {
            usbState->residualData = (uint8_t *)header;
            usbState->residualData += sizeof(TinyCLR_UsbClient_GenericDescriptorHeader);       // data is located right after the header
            usbState->residualCount = __min(usbState->expected, header->size - sizeof(TinyCLR_UsbClient_GenericDescriptorHeader));
        }
        else
            return USB_STATE_STALL;
    }

    usbState->dataCallback = AT91_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t AT91_UsbClient_HandleGetConfiguration(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
    /* validate setup packet */
    if (Setup->wValue != 0 || Setup->wIndex != 0 || Setup->wLength != 1) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    usbState->residualData = &usbState->configurationNum;
    usbState->residualCount = 1;
    usbState->expected = 1;
    usbState->dataCallback = AT91_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t AT91_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup, bool DataPhase) {
    /* validate setup packet */
    if (Setup->wIndex != 0 || Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    /* we only support one configuration */
    if (Setup->wValue > 1) {
        return USB_STATE_STALL;
    }

    usbState->configurationNum = Setup->wValue;

    /* catch state changes */
    if (usbState->configurationNum == 0) {
        usbState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }
    else {
        usbState->deviceState = USB_DEVICE_STATE_CONFIGURED;
    }

    AT91_UsbClient_StateCallback(usbState);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        usbState->residualCount = 0;
        usbState->dataCallback = AT91_UsbClient_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB configuration records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const TinyCLR_UsbClient_DescriptorHeader * USB_FindRecord(USB_CONTROLLER_STATE* usbState, uint8_t marker, USB_SETUP_PACKET * setup) {
    bool Done = false;


    const TinyCLR_UsbClient_DescriptorHeader * header = (const TinyCLR_UsbClient_DescriptorHeader *)usbState->configuration;
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

uint8_t AT91_UsbClient_ControlCallback(USB_CONTROLLER_STATE* usbState) {
    USB_SETUP_PACKET* Setup;

    if (usbState->dataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (USB_SETUP_PACKET*)usbState->ptrData;

    switch (Setup->bRequest) {
    case USB_GET_STATUS:
        return AT91_UsbClient_HandleGetStatus(usbState, Setup);
    case USB_CLEAR_FEATURE:
        return AT91_UsbClient_HandleClearFeature(usbState, Setup);
    case USB_SET_FEATURE:
        return AT91_UsbClient_HandleSetFeature(usbState, Setup);
    case USB_SET_ADDRESS:
        return AT91_UsbClient_HandleSetAddress(usbState, Setup);
    case USB_GET_CONFIGURATION:
        return AT91_UsbClient_HandleGetConfiguration(usbState, Setup);
    case USB_SET_CONFIGURATION:
        return AT91_UsbClient_HandleSetConfiguration(usbState, Setup, true);
    default:
        return AT91_UsbClient_HandleConfigurationRequests(usbState, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* AT91_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint, bool& disableRx) {
    USB_PACKET64* packet;

    if (usb_fifo_buffer_count[endpoint] == AT91_USB_FIFO_BUFFER_SIZE) {
        disableRx = true;

        return nullptr;
    }

    disableRx = false;

    packet = &usbState->queues[endpoint][usb_fifo_buffer_in[endpoint]];

    usb_fifo_buffer_in[endpoint]++;
    usb_fifo_buffer_count[endpoint]++;

    if (usb_fifo_buffer_in[endpoint] == AT91_USB_FIFO_BUFFER_SIZE)
        usb_fifo_buffer_in[endpoint] = 0;

    AT91_UsbClient_SetEvent(usbState, 1 << endpoint);

    return packet;
}

USB_PACKET64* AT91_UsbClient_TxDequeue(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    USB_PACKET64* packet;

    if (usb_fifo_buffer_count[endpoint] == 0) {
        return nullptr;
    }

    packet = &usbState->queues[endpoint][usb_fifo_buffer_out[endpoint]];

    usb_fifo_buffer_count[endpoint]--;
    usb_fifo_buffer_out[endpoint]++;

    if (usb_fifo_buffer_out[endpoint] == AT91_USB_FIFO_BUFFER_SIZE)
        usb_fifo_buffer_out[endpoint] = 0;

    return packet;
}

void AT91_UsbClient_ClearEndpoints(int32_t endpoint) {
    usb_fifo_buffer_in[endpoint] = usb_fifo_buffer_out[endpoint] = usb_fifo_buffer_count[endpoint] = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
/// TinyCLR USBClient API
///////////////////////////////////////////////////////////////////////////////////////////
bool AT91_UsbClient_Initialize(USB_CONTROLLER_STATE* usbState);
bool AT91_UsbClient_Uninitialize(USB_CONTROLLER_STATE* usbState);
bool AT91_UsbClient_StartOutput(USB_CONTROLLER_STATE* usbState, int32_t endpoint);

static TinyCLR_UsbClient_Provider usbClientProvider;
static TinyCLR_Api_Info usbClientApi;

const TinyCLR_Api_Info* AT91_UsbClient_GetApi() {
    usbClientProvider.Parent = &usbClientApi;
    usbClientProvider.Index = 0;
    usbClientProvider.Acquire = &AT91_UsbClient_Acquire;
    usbClientProvider.Release = &AT91_UsbClient_Release;
    usbClientProvider.Open = &AT91_UsbClient_Open;
    usbClientProvider.Close = &AT91_UsbClient_Close;
    usbClientProvider.Write = &AT91_UsbClient_Write;
    usbClientProvider.Read = &AT91_UsbClient_Read;
    usbClientProvider.Flush = &AT91_UsbClient_Flush;
    usbClientProvider.SetDeviceDescriptor = &AT91_UsbClient_SetDeviceDescriptor;
    usbClientProvider.SetConfigDescriptor = &AT91_UsbClient_SetConfigDescriptor;
    usbClientProvider.SetStringDescriptor = &AT91_UsbClient_SetStringDescriptor;
    usbClientProvider.SetDataReceivedHandler = &AT91_UsbClient_SetDataReceivedHandler;
    usbClientProvider.SetOsExtendedPropertyHandler = &AT91_UsbClient_SetOsExtendedPropertyHandler;

    usbClientApi.Author = "GHI Electronics, LLC";
    usbClientApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.UsbClientProvider";
    usbClientApi.Type = TinyCLR_Api_Type::UsbClientProvider;
    usbClientApi.Version = 0;
    usbClientApi.Count = 1;
    usbClientApi.Implementation = &usbClientProvider;

    return &usbClientApi;
}
TinyCLR_Result AT91_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    uint8_t *osStringDescriptor = (uint8_t*)&AT91_UsbClient_OsStringDescriptor;
    uint8_t *xCompatibleOsId = (uint8_t*)&AT91_UsbClient_XCompatibleOsId;
    uint8_t *xPropertiesOsWinUsb = (uint8_t*)&AT91_UsbClient_XPropertiesOsWinUsb;

    AT91_UsbClient_OsExtendedPropertyHandler(self, osStringDescriptor, xCompatibleOsId, xPropertiesOsWinUsb);

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

    UsbDefaultConfiguration.OS_String = (TinyCLR_UsbClient_OsStringDescriptor*)&AT91_UsbClient_OsStringDescriptor;
    UsbDefaultConfiguration.OS_XCompatible_ID = (TinyCLR_UsbClient_XCompatibleOsId*)&AT91_UsbClient_XCompatibleOsId;
    UsbDefaultConfiguration.OS_XProperty = (TinyCLR_UsbClient_XPropertiesOsWinUsb*)&AT91_UsbClient_XPropertiesOsWinUsb;

    UsbDefaultConfiguration.endList = (TinyCLR_UsbClient_DescriptorHeader*)&usbDescriptorHeader;

    // Init Usb usbState
    memset(usbState, 0, sizeof(USB_CONTROLLER_STATE));

    usbState->controllerNum = controller;
    usbState->configuration = &UsbDefaultConfiguration;
    usbState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
    usbState->deviceStatus = USB_STATUS_DEVICE_SELF_POWERED;
    usbState->endpointCount = USB_MAX_EP_COUNT;
    usbState->packetSize = USB_MAX_EP0_SIZE;
    usbState->initialized = true;

    for (auto i = 0; i < AT91_USB_QUEUE_SIZE; i++) {
        usbState->pipes[i].RxEP = USB_ENDPOINT_NULL;
        usbState->pipes[i].TxEP = USB_ENDPOINT_NULL;
        usbState->maxPacketSize[i] = USB_MAX_EP_SIZE;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_Release(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (usbState->initialized) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        AT91_UsbClient_Uninitialize(usbState);

        usbState->initialized = false;

        // for soft reboot allow the USB to be off for at least 100ms
        AT91_Time_Delay(nullptr, 100000); // 100ms
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t& pipe, TinyCLR_UsbClient_PipeMode mode) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (!usbState->initialized)     // If no such controller exists (or it is not initialized)
        return TinyCLR_Result::NotAvailable;

    int32_t writeEp = USB_ENDPOINT_NULL;
    int32_t readEp = USB_ENDPOINT_NULL;

    if (mode != TinyCLR_UsbClient_PipeMode::InOut)
        return TinyCLR_Result::NotSupported;

    for (auto i = 0; i < SIZEOF_ARRAY(AT91_UsbClient_EndpointMap); i++) {
        if ((AT91_UsbClient_EndpointMap[i] & ENDPOINT_INUSED_MASK)) // in used
            continue;

        if (writeEp == USB_ENDPOINT_NULL && ((AT91_UsbClient_EndpointMap[i] & ENDPOINT_DIR_IN_MASK) == ENDPOINT_DIR_IN_MASK)) {
            writeEp = i;
            AT91_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

            continue;
        }

        if (readEp == USB_ENDPOINT_NULL && ((AT91_UsbClient_EndpointMap[i] & ENDPOINT_DIR_OUT_MASK) == ENDPOINT_DIR_OUT_MASK)) {
            readEp = i;
            AT91_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

            continue;
        }

        if (writeEp != 0 && readEp != 0) {
            break;
        }
    }
    // Check the pipe and the two endpoint numbers for validity (both endpoints cannot be zero)
    if ((readEp == USB_ENDPOINT_NULL && writeEp == USB_ENDPOINT_NULL)
        || (readEp != USB_ENDPOINT_NULL && (readEp < 1 || readEp >= AT91_USB_QUEUE_SIZE))
        || (writeEp != USB_ENDPOINT_NULL && (writeEp < 1 || writeEp >= AT91_USB_QUEUE_SIZE)))
        return TinyCLR_Result::NotAvailable;

    // The specified endpoints must not be in use by another pipe
    for (int32_t i = 0; i < AT91_USB_QUEUE_SIZE; i++) {
        if (readEp != USB_ENDPOINT_NULL && (usbState->pipes[i].RxEP == readEp || usbState->pipes[i].TxEP == readEp))
            return TinyCLR_Result::NotAvailable;
        if (writeEp != USB_ENDPOINT_NULL && (usbState->pipes[i].RxEP == writeEp || usbState->pipes[i].TxEP == writeEp))
            return TinyCLR_Result::NotAvailable;
    }

    for (pipe = 0; pipe < AT91_USB_QUEUE_SIZE; pipe++) {
        // The Pipe must be currently closed
        if (usbState->pipes[pipe].RxEP == USB_ENDPOINT_NULL && usbState->pipes[pipe].TxEP == USB_ENDPOINT_NULL)
            break;
    }

    if (pipe == AT91_USB_QUEUE_SIZE)
        return TinyCLR_Result::NotAvailable; // full endpoint

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
            usbState->isTxQueue[idx] = true;
        }

        else if (ep->bEndpointAddress == USB_ENDPOINT_DIRECTION_OUT) {
            ep->bEndpointAddress |= readEp;
            idx = readEp;
            usbState->isTxQueue[idx] = false;
        }

        if (idx > 0) {
            if (apiProvider != nullptr) {
                auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

                usbState->queues[idx] = (USB_PACKET64*)memoryProvider->Allocate(memoryProvider, AT91_USB_FIFO_BUFFER_SIZE * sizeof(USB_PACKET64));

                if (usbState->queues[idx] == nullptr)
                    return TinyCLR_Result::ArgumentNull;

                memset(reinterpret_cast<uint8_t*>(usbState->queues[idx]), 0x00, AT91_USB_FIFO_BUFFER_SIZE * sizeof(USB_PACKET64));
            }

            AT91_UsbClient_ClearEndpoints(idx);


            usbState->maxPacketSize[idx] = ep->wMaxPacketSize;
        }

        ep = (TinyCLR_UsbClient_EndpointDescriptor  *)(((uint8_t *)ep) + ep->bLength);
    }

    if (usbState->currentState == USB_DEVICE_STATE_UNINITIALIZED) {
        AT91_UsbClient_Initialize(usbState);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t pipe) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (!usbState->initialized || pipe >= AT91_USB_QUEUE_SIZE)
        return TinyCLR_Result::NotAvailable;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Close the Rx pipe
    int32_t endpoint = usbState->pipes[pipe].RxEP;
    if (endpoint != USB_ENDPOINT_NULL && usbState->queues[endpoint]) {
        AT91_UsbClient_ClearEndpoints(endpoint);
    }

    usbState->pipes[pipe].RxEP = USB_ENDPOINT_NULL;
    //Free endpoint
    AT91_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    // Close the TX pipe
    endpoint = usbState->pipes[pipe].TxEP;
    if (endpoint != USB_ENDPOINT_NULL && usbState->queues[endpoint] != nullptr) {
        AT91_UsbClient_ClearEndpoints(endpoint);

        if (apiProvider != nullptr) {
            auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

            if (usbState->queues[endpoint] != nullptr)
                memoryProvider->Free(memoryProvider, usbState->queues[endpoint]);

            usbState->queues[endpoint] = nullptr;
        }
    }

    usbState->pipes[pipe].TxEP = USB_ENDPOINT_NULL;

    //Free endpoint
    AT91_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    configDescriptor.epWrite.bEndpointAddress = USB_ENDPOINT_DIRECTION_IN;
    configDescriptor.epRead.bEndpointAddress = USB_ENDPOINT_DIRECTION_OUT;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t pipe, const uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (pipe >= AT91_USB_QUEUE_SIZE
        || data == nullptr
        || usbState->deviceState != USB_DEVICE_STATE_CONFIGURED
        || length == 0) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    int32_t endpoint = usbState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->queues[endpoint] == nullptr) {
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

        if (usb_fifo_buffer_count[endpoint] < AT91_USB_FIFO_BUFFER_SIZE) {
            Packet64 = &usbState->queues[endpoint][usb_fifo_buffer_in[endpoint]];

            usb_fifo_buffer_in[endpoint]++;
            usb_fifo_buffer_count[endpoint]++;

            if (usb_fifo_buffer_in[endpoint] == AT91_USB_FIFO_BUFFER_SIZE)
                usb_fifo_buffer_in[endpoint] = 0;
        }

        if (Packet64) {
            uint32_t max_move;

            if (count > usbState->maxPacketSize[endpoint])
                max_move = usbState->maxPacketSize[endpoint];
            else
                max_move = count;

            if (max_move) {
                memcpy(Packet64->Buffer, ptr, max_move);
            }

            // we are done when we send a non-full length packet
            if (max_move < usbState->maxPacketSize[endpoint]) {
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
                    AT91_UsbClient_ClearEndpoints(endpoint);
                }

                goto done_write;
            }

            if (irq.WasDisabled()) // @todo - this really needs more checks to be totally valid
            {
                goto done_write;
            }

            if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
                goto done_write;
            }

            AT91_UsbClient_StartOutput(usbState, endpoint);

            irq.Release();

            AT91_Time_Delay(nullptr, 50);

            irq.Acquire();
        }
    }

    // here we have a post-condition that IRQs are disabled for all paths through conditional block above
    if (usbState->deviceState == USB_DEVICE_STATE_CONFIGURED) {
        AT91_UsbClient_StartOutput(usbState, endpoint);
    }

done_write:
    length = totWrite;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t pipe, uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    int32_t endpoint;
    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (pipe >= AT91_USB_QUEUE_SIZE
        || usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    endpoint = usbState->pipes[pipe].RxEP;
    // If no Read side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->queues[endpoint] == nullptr) {
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
            Packet64 = &usbState->queues[endpoint][usb_fifo_buffer_out[endpoint]];

            usb_fifo_buffer_count[endpoint]--;
            usb_fifo_buffer_out[endpoint]++;

            if (usb_fifo_buffer_out[endpoint] == AT91_USB_FIFO_BUFFER_SIZE) {
                usb_fifo_buffer_out[endpoint] = 0;
            }
        }

        if (!Packet64) {
            AT91_UsbClient_ClearEvent(usbState, 1 << endpoint);
            break;
        }

        max_move = Packet64->Size - usbState->currentPacketOffset[endpoint];
        if (remain < max_move) max_move = remain;

        memcpy(ptr, &Packet64->Buffer[usbState->currentPacketOffset[endpoint]], max_move);

        usbState->currentPacketOffset[endpoint] += max_move;
        ptr += max_move;
        count += max_move;
        remain -= max_move;

        /* if we're done with this packet, move onto the next */
        if (usbState->currentPacketOffset[endpoint] == Packet64->Size) {
            usbState->currentPacketOffset[endpoint] = 0;
            Packet64 = nullptr;

            AT91_UsbClient_RxEnable(endpoint);
        }
    }

    length = count;

    return TinyCLR_Result::Success;
}

#define USB_FLUSH_RETRY_COUNT 30
TinyCLR_Result AT91_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t pipe) {
    int32_t controller = self->Index;

    int32_t endpoint;
    int32_t retries = USB_FLUSH_RETRY_COUNT;
    int32_t queueCnt;
    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    if (pipe >= AT91_USB_QUEUE_SIZE) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    /* not configured, no data can go in or out */
    if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::InvalidOperation;
    }

    endpoint = usbState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    queueCnt = usb_fifo_buffer_count[endpoint];

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while (usb_fifo_buffer_count[endpoint] > 0 && retries > 0) {
        AT91_UsbClient_StartOutput(usbState, endpoint);

        AT91_Time_Delay(nullptr, queueCnt == usb_fifo_buffer_count[endpoint] ? 100 : 0); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == usb_fifo_buffer_count[endpoint]) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = usb_fifo_buffer_count[endpoint];
    }

    if (retries <= 0)
        AT91_UsbClient_ClearEndpoints(endpoint);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    int32_t controller = self->Index;

    AT91_UsbClient_DataReceivedHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_SetOsExtendedPropertyHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler) {
    int32_t controller = self->Index;

    AT91_UsbClient_OsExtendedPropertyHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length) {
    memcpy(&deviceDescriptor, descriptor, length);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length) {
    memcpy(&configDescriptor, descriptor, length);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value) {
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

void AT91_UsbClient_Reset() {
    for (auto pipe = 0; pipe < AT91_USB_QUEUE_SIZE; pipe++) {
        AT91_UsbClient_Close(&usbClientProvider, pipe);
    }

    AT91_UsbClient_Release(&usbClientProvider);
}

///////////////////////////////////////////////////////////////////////////////////////////
/// AT91 USB Hardware driver
///////////////////////////////////////////////////////////////////////////////////////////
    // -------- UDPHS_IEN : (UDPHS Offset: 0x10) UDPHS Interrupt Enable Register --------
#define AT91C_UDPHS_DET_SUSPD (0x1 <<  1) // (UDPHS) Suspend Interrupt Enable/Clear/Status
#define AT91C_UDPHS_MICRO_SOF (0x1 <<  2) // (UDPHS) Micro-SOF Interrupt Enable/Clear/Status
#define AT91C_UDPHS_IEN_SOF   (0x1 <<  3) // (UDPHS) SOF Interrupt Enable/Clear/Status
#define AT91C_UDPHS_ENDRESET  (0x1 <<  4) // (UDPHS) End Of Reset Interrupt Enable/Clear/Status
#define AT91C_UDPHS_WAKE_UP   (0x1 <<  5) // (UDPHS) Wake Up CPU Interrupt Enable/Clear/Status
#define AT91C_UDPHS_ENDOFRSM  (0x1 <<  6) // (UDPHS) End Of Resume Interrupt Enable/Clear/Status
#define AT91C_UDPHS_UPSTR_RES (0x1 <<  7) // (UDPHS) Uppipe Resume Interrupt Enable/Clear/Status
#define AT91C_UDPHS_EPT_INT_0 (0x1 <<  8) // (UDPHS) Endpoint 0 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_1 (0x1 <<  9) // (UDPHS) Endpoint 1 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_2 (0x1 << 10) // (UDPHS) Endpoint 2 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_3 (0x1 << 11) // (UDPHS) Endpoint 3 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_4 (0x1 << 12) // (UDPHS) Endpoint 4 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_5 (0x1 << 13) // (UDPHS) Endpoint 5 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_6 (0x1 << 14) // (UDPHS) Endpoint 6 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_7 (0x1 << 15) // (UDPHS) Endpoint 7 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_8 (0x1 << 16) // (UDPHS) Endpoint 8 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_9 (0x1 << 17) // (UDPHS) Endpoint 9 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_10 (0x1 << 18) // (UDPHS) Endpoint 10 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_11 (0x1 << 19) // (UDPHS) Endpoint 11 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_12 (0x1 << 20) // (UDPHS) Endpoint 12 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_13 (0x1 << 21) // (UDPHS) Endpoint 13 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_14 (0x1 << 22) // (UDPHS) Endpoint 14 Interrupt Enable/Status
#define AT91C_UDPHS_EPT_INT_15 (0x1 << 23) // (UDPHS) Endpoint 15 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_1 (0x1 << 25) // (UDPHS) DMA Channel 1 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_2 (0x1 << 26) // (UDPHS) DMA Channel 2 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_3 (0x1 << 27) // (UDPHS) DMA Channel 3 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_4 (0x1 << 28) // (UDPHS) DMA Channel 4 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_5 (0x1 << 29) // (UDPHS) DMA Channel 5 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_6 (0x1 << 30) // (UDPHS) DMA Channel 6 Interrupt Enable/Status
#define AT91C_UDPHS_DMA_INT_7 (0x1 << 31) // (UDPHS) DMA Channel 7 Interrupt Enable/Status

// -------- UDPHS_EPTCFG : (UDPHS_EPT Offset: 0x0) UDPHS Endpoint Config Register --------
#define AT91C_UDPHS_EPT_SIZE  (0x7 <<  0) // (UDPHS_EPT) Endpoint Size
#define     AT91C_UDPHS_EPT_SIZE_8                    (0x0) // (UDPHS_EPT)    8 bytes
#define     AT91C_UDPHS_EPT_SIZE_16                   (0x1) // (UDPHS_EPT)   16 bytes
#define     AT91C_UDPHS_EPT_SIZE_32                   (0x2) // (UDPHS_EPT)   32 bytes
#define     AT91C_UDPHS_EPT_SIZE_64                   (0x3) // (UDPHS_EPT)   64 bytes
#define     AT91C_UDPHS_EPT_SIZE_128                  (0x4) // (UDPHS_EPT)  128 bytes
#define     AT91C_UDPHS_EPT_SIZE_256                  (0x5) // (UDPHS_EPT)  256 bytes
#define     AT91C_UDPHS_EPT_SIZE_512                  (0x6) // (UDPHS_EPT)  512 bytes
#define     AT91C_UDPHS_EPT_SIZE_1024                 (0x7) // (UDPHS_EPT) 1024 bytes
#define AT91C_UDPHS_EPT_DIR   (0x1 <<  3) // (UDPHS_EPT) Endpoint Direction 0:OUT, 1:IN
#define     AT91C_UDPHS_EPT_DIR_OUT                  (0x0 <<  3) // (UDPHS_EPT) Direction OUT
#define     AT91C_UDPHS_EPT_DIR_IN                   (0x1 <<  3) // (UDPHS_EPT) Direction IN
#define AT91C_UDPHS_EPT_TYPE  (0x3 <<  4) // (UDPHS_EPT) Endpoint Type
#define     AT91C_UDPHS_EPT_TYPE_CTL_EPT              (0x0 <<  4) // (UDPHS_EPT) Control endpoint
#define     AT91C_UDPHS_EPT_TYPE_ISO_EPT              (0x1 <<  4) // (UDPHS_EPT) Isochronous endpoint
#define     AT91C_UDPHS_EPT_TYPE_BUL_EPT              (0x2 <<  4) // (UDPHS_EPT) Bulk endpoint
#define     AT91C_UDPHS_EPT_TYPE_INT_EPT              (0x3 <<  4) // (UDPHS_EPT) Interrupt endpoint
#define AT91C_UDPHS_BK_NUMBER (0x3 <<  6) // (UDPHS_EPT) Number of Banks
#define     AT91C_UDPHS_BK_NUMBER_0                    (0x0 <<  6) // (UDPHS_EPT) Zero Bank, the EndPoint is not mapped in memory
#define     AT91C_UDPHS_BK_NUMBER_1                    (0x1 <<  6) // (UDPHS_EPT) One Bank (Bank0)
#define     AT91C_UDPHS_BK_NUMBER_2                    (0x2 <<  6) // (UDPHS_EPT) Double bank (Ping-Pong : Bank0 / Bank1)
#define     AT91C_UDPHS_BK_NUMBER_3                    (0x3 <<  6) // (UDPHS_EPT) Triple Bank (Bank0 / Bank1 / Bank2)
#define AT91C_UDPHS_NB_TRANS  (0x3 <<  8) // (UDPHS_EPT) Number Of Transaction per Micro-Frame (High-Bandwidth iso only)
#define AT91C_UDPHS_EPT_MAPD  ((uint32_t)0x1 << 31) // (UDPHS_EPT) Endpoint Mapped (read only

// -------- UDPHS_EPTCTLENB : (UDPHS_EPT Offset: 0x4) UDPHS Endpoint Control Enable Register --------
#define AT91C_UDPHS_EPT_ENABL (0x1 <<  0) // (UDPHS_EPT) Endpoint Enable
#define AT91C_UDPHS_AUTO_VALID (0x1 <<  1) // (UDPHS_EPT) Packet Auto-Valid Enable/Disable
#define AT91C_UDPHS_INTDIS_DMA (0x1 <<  3) // (UDPHS_EPT) Endpoint Interrupts DMA Request Enable/Disable
#define AT91C_UDPHS_NYET_DIS  (0x1 <<  4) // (UDPHS_EPT) NYET Enable/Disable
#define AT91C_UDPHS_DATAX_RX  (0x1 <<  6) // (UDPHS_EPT) DATAx Interrupt Enable/Disable
#define AT91C_UDPHS_MDATA_RX  (0x1 <<  7) // (UDPHS_EPT) MDATA Interrupt Enabled/Disable
#define AT91C_UDPHS_ERR_OVFLW (0x1 <<  8) // (UDPHS_EPT) OverFlow Error Interrupt Enable/Disable/Status
#define AT91C_UDPHS_RX_BK_RDY (0x1 <<  9) // (UDPHS_EPT) Received OUT Data
#define AT91C_UDPHS_TX_COMPLT (0x1 << 10) // (UDPHS_EPT) Transmitted IN Data Complete Interrupt Enable/Disable or Transmitted IN Data Complete (clear)
#define AT91C_UDPHS_ERR_TRANS (0x1 << 11) // (UDPHS_EPT) Transaction Error Interrupt Enable/Disable
#define AT91C_UDPHS_TX_PK_RDY (0x1 << 11) // (UDPHS_EPT) TX Packet Ready Interrupt Enable/Disable
#define AT91C_UDPHS_RX_SETUP  (0x1 << 12) // (UDPHS_EPT) Received SETUP Interrupt Enable/Disable
#define AT91C_UDPHS_ERR_FL_ISO (0x1 << 12) // (UDPHS_EPT) Error Flow Clear/Interrupt Enable/Disable
#define AT91C_UDPHS_STALL_SNT (0x1 << 13) // (UDPHS_EPT) Stall Sent Clear
#define AT91C_UDPHS_ERR_CRISO (0x1 << 13) // (UDPHS_EPT) CRC error / Error NB Trans / Interrupt Enable/Disable
#define AT91C_UDPHS_NAK_IN    (0x1 << 14) // (UDPHS_EPT) NAKIN ERROR FLUSH / Clear / Interrupt Enable/Disable
#define AT91C_UDPHS_NAK_OUT   (0x1 << 15) // (UDPHS_EPT) NAKOUT / Clear / Interrupt Enable/Disable
#define AT91C_UDPHS_BUSY_BANK (0x1 << 18) // (UDPHS_EPT) Busy Bank Interrupt Enable/Disable
#define AT91C_UDPHS_SHRT_PCKT (0x1 << 31) // (UDPHS_EPT) Short Packet / Interrupt Enable/Disable
// -------- UDPHS_CTRL : (UDPHS Offset: 0x0) UDPHS Control Register --------
#define AT91C_UDPHS_DEV_ADDR  (0x7F <<  0) // (UDPHS) UDPHS Address
#define AT91C_UDPHS_FADDR_EN  (0x1 <<  7) // (UDPHS) Function Address Enable
#define AT91C_UDPHS_EN_UDPHS  (0x1 <<  8) // (UDPHS) UDPHS Enable
#define AT91C_UDPHS_DETACH    (0x1 <<  9) // (UDPHS) Detach Command
#define AT91C_UDPHS_REWAKEUP  (0x1 << 10) // (UDPHS) Send Remote Wake Up
#define AT91C_UDPHS_PULLD_DIS (0x1 << 11) // (UDPHS) PullDown Disable

// -------- UDPHS_IPFEATURES : (UDPHS Offset: 0xf8) UDPHS Features Register --------
#define AT91C_UDPHS_EPT_NBR_MAX (0xF <<  0) // (UDPHS) Max Number of Endpoints
#define AT91C_UDPHS_DMA_CHANNEL_NBR (0x7 <<  4) // (UDPHS) Number of DMA Channels
#define AT91C_UDPHS_DMA_B_SIZ (0x1 <<  7) // (UDPHS) DMA Buffer Size
#define AT91C_UDPHS_DMA_FIFO_WORD_DEPTH (0xF <<  8) // (UDPHS) DMA FIFO Depth in words
#define AT91C_UDPHS_FIFO_MAX_SIZE (0x7 << 12) // (UDPHS) DPRAM size
#define AT91C_UDPHS_BW_DPRAM  (0x1 << 15) // (UDPHS) DPRAM byte write capability
#define AT91C_UDPHS_DATAB16_8 (0x1 << 16) // (UDPHS) UTMI DataBus16_8
#define AT91C_UDPHS_ISO_EPT_1 (0x1 << 17) // (UDPHS) Endpoint 1 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_2 (0x1 << 18) // (UDPHS) Endpoint 2 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_3 (0x1 << 19) // (UDPHS) Endpoint 3 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_4 (0x1 << 20) // (UDPHS) Endpoint 4 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_5 (0x1 << 21) // (UDPHS) Endpoint 5 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_6 (0x1 << 22) // (UDPHS) Endpoint 6 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_7 (0x1 << 23) // (UDPHS) Endpoint 7 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_8 (0x1 << 24) // (UDPHS) Endpoint 8 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_9 (0x1 << 25) // (UDPHS) Endpoint 9 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_10 (0x1 << 26) // (UDPHS) Endpoint 10 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_11 (0x1 << 27) // (UDPHS) Endpoint 11 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_12 (0x1 << 28) // (UDPHS) Endpoint 12 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_13 (0x1 << 29) // (UDPHS) Endpoint 13 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_14 (0x1 << 30) // (UDPHS) Endpoint 14 High Bandwidth Isochronous Capability
#define AT91C_UDPHS_ISO_EPT_15 (0x1 << 31) // (UDPHS) Endpoint 15 High Bandwidth Isochronous Capability

#define AT91C_UDPHS_BYTE_COUNT (0x7FF << 20) // (UDPHS_EPT) UDPHS Byte Count

/* -------- UDPHS_EPTSETSTA : (UDPHS Offset: N/A) UDPHS Endpoint Set Status Register -------- */
#define UDPHS_EPTSETSTA_FRCESTALL (0x1u << 5) /**< \brief (UDPHS_EPTSETSTA) Stall Handshake Request Set */
#define UDPHS_EPTSETSTA_KILL_BANK (0x1u << 9) /**< \brief (UDPHS_EPTSETSTA) KILL Bank Set (for IN Endpoint) */
#define UDPHS_EPTSETSTA_TX_PK_RDY (0x1u << 11) /**< \brief (UDPHS_EPTSETSTA) TX Packet Ready Set */

/* -------- UDPHS_CTRL : (UDPHS Offset: 0x00) UDPHS Control Register -------- */
#define UDPHS_CTRL_DEV_ADDR_Pos 0
#define UDPHS_CTRL_DEV_ADDR_Msk (0x7fu << UDPHS_CTRL_DEV_ADDR_Pos) /**< \brief (UDPHS_CTRL) UDPHS Address */
#define UDPHS_CTRL_DEV_ADDR(value) ((UDPHS_CTRL_DEV_ADDR_Msk & ((value) << UDPHS_CTRL_DEV_ADDR_Pos)))
#define UDPHS_CTRL_FADDR_EN (0x1u << 7) /**< \brief (UDPHS_CTRL) Function Address Enable */
#define UDPHS_CTRL_EN_UDPHS (0x1u << 8) /**< \brief (UDPHS_CTRL) UDPHS Enable */
#define UDPHS_CTRL_DETACH (0x1u << 9) /**< \brief (UDPHS_CTRL) Detach Command */
#define UDPHS_CTRL_REWAKEUP (0x1u << 10) /**< \brief (UDPHS_CTRL) Send Remote Wake Up */
#define UDPHS_CTRL_PULLD_DIS (0x1u << 11) /**< \brief (UDPHS_CTRL) Pull-Down Disable */

#define AT91C_CKGR_UPLLEN_ENABLED  (0x1 << 16) // (PMC) The UTMI PLL is enabled
#define AT91C_CKGR_BIASEN_ENABLED  (0x1 << 24) // (PMC) The UTMI BIAS is enabled
#define SHIFT_INTERUPT             8

#define USB_CTRL_WMAXPACKETSIZE0_EP_WRITE                   64
#define USB_BULK_WMAXPACKETSIZE_EP_WRITE                    64
#define USB_BULK_WMAXPACKETSIZE_EP_READ                     64

struct AT91_UDPHS_EPT {
    volatile uint32_t     UDPHS_EPTCFG;     // UDPHS Endpoint Config Register
    volatile uint32_t     UDPHS_EPTCTLENB;     // UDPHS Endpoint Control Enable Register
    volatile uint32_t     UDPHS_EPTCTLDIS;     // UDPHS Endpoint Control Disable Register
    volatile uint32_t     UDPHS_EPTCTL;     // UDPHS Endpoint Control Register
    volatile uint32_t     Reserved0[1];     //
    volatile uint32_t     UDPHS_EPTSETSTA;     // UDPHS Endpoint Set Status Register
    volatile uint32_t     UDPHS_EPTCLRSTA;     // UDPHS Endpoint Clear Status Register
    volatile uint32_t     UDPHS_EPTSTA;     // UDPHS Endpoint Status Register
};

struct AT91_UDPHS_DMA {
    volatile uint32_t     UDPHS_DMANXTDSC;     // UDPHS DMA Channel Next Descriptor Address
    volatile uint32_t     UDPHS_DMAADDRESS;     // UDPHS DMA Channel Address Register
    volatile uint32_t     UDPHS_DMACONTROL;     // UDPHS DMA Channel Control Register
    volatile uint32_t     UDPHS_DMASTATUS;     // UDPHS DMA Channel Status Register
};

struct AT91_UDPHS {
    volatile uint32_t     UDPHS_CTRL;     // UDPHS Control Register                    0x0000
    volatile uint32_t     UDPHS_FNUM;     // UDPHS Frame Number Register                0x0004
    volatile uint32_t     Reserved0[2];     //                                            0x0008, 0x000C
    volatile uint32_t     UDPHS_IEN;     // UDPHS Interrupt Enable Register            0x0010
    volatile uint32_t     UDPHS_INTSTA;     // UDPHS Interrupt Status Register            0x0014
    volatile uint32_t     UDPHS_CLRINT;     // UDPHS Clear Interrupt Register            0x0018
    volatile uint32_t     UDPHS_EPTRST;     // UDPHS Endpoints Reset Register            0x001C
    volatile uint32_t     Reserved1[44];     //                                        0x0020, 0x0024, 0x0028, 0x002C, 0x0030, 0x0034, 0x0038, 0x003C, 0x0040, 0x0044, 0x0048, 0x004C, 0x0050, 0x0054, 0x0058, 0x005C,
                                        //                                        0x0060, 0x0064, 0x0068, 0x006C, 0x0070, 0x0074, 0x0078, 0x007C, 0x0080, 0x0084, 0x0088, 0x008C, 0x0090, 0x0094, 0x0098, 0x009C,
                                        //                                        0x00A0, 0x00A4, 0x00A8, 0x00AC, 0x00B0, 0x00B4, 0x00B8, 0x00BC, 0x00C0, 0x00C4, 0x00C8, 0x00CC
    volatile uint32_t     UDPHS_TSTSOFCNT;     // UDPHS Test SOF Counter Register        0x00D0
    volatile uint32_t     UDPHS_TSTCNTA;     // UDPHS Test A Counter Register        0x00D4
    volatile uint32_t     UDPHS_TSTCNTB;     // UDPHS Test B Counter Register        0x00D8
    volatile uint32_t     UDPHS_TSTMODREG;     // UDPHS Test Mode Register                0x00DC
    volatile uint32_t     UDPHS_TST;     // UDPHS Test Register                        0x00E0
    volatile uint32_t     Reserved2[2];     //                                            0x00E4, 0x00E8
    volatile uint32_t     UDPHS_RIPPADDRSIZE;     // UDPHS PADDRSIZE Register            0x00EC
    volatile uint32_t     UDPHS_RIPNAME1;     // UDPHS Name1 Register                    0x00F0
    volatile uint32_t     UDPHS_RIPNAME2;     // UDPHS Name2 Register                    0x00F4
    volatile uint32_t     UDPHS_IPFEATURES;     // UDPHS Features Register                0x00F8
    volatile uint32_t     UDPHS_IPVERSION;     // UDPHS Version Register                0x00FC
    struct AT91_UDPHS_EPT     UDPHS_EPT[16];     // UDPHS Endpoint struct        0x0100
    struct AT91_UDPHS_DMA     UDPHS_DMA[8];     // UDPHS DMA channel struct (not use [0])
};

struct AT91_UDPHS_EPTFIFO {
    volatile uint32_t     UDPHS_READEPT0[16384];     // FIFO Endpoint Data Register 0
    volatile uint32_t     UDPHS_READEPT1[16384];     // FIFO Endpoint Data Register 1
    volatile uint32_t     UDPHS_READEPT2[16384];     // FIFO Endpoint Data Register 2
    volatile uint32_t     UDPHS_READEPT3[16384];     // FIFO Endpoint Data Register 3
    volatile uint32_t     UDPHS_READEPT4[16384];     // FIFO Endpoint Data Register 4
    volatile uint32_t     UDPHS_READEPT5[16384];     // FIFO Endpoint Data Register 5
    volatile uint32_t     UDPHS_READEPT6[16384];     // FIFO Endpoint Data Register 6
    volatile uint32_t     UDPHS_READEPT7[16384];     // FIFO Endpoint Data Register 7
    volatile uint32_t     UDPHS_READEPT8[16384];     // FIFO Endpoint Data Register 8
    volatile uint32_t     UDPHS_READEPT9[16384];     // FIFO Endpoint Data Register 9
    volatile uint32_t     UDPHS_READEPTA[16384];     // FIFO Endpoint Data Register 10
    volatile uint32_t     UDPHS_READEPTB[16384];     // FIFO Endpoint Data Register 11
    volatile uint32_t     UDPHS_READEPTC[16384];     // FIFO Endpoint Data Register 12
    volatile uint32_t     UDPHS_READEPTD[16384];     // FIFO Endpoint Data Register 13
    volatile uint32_t     UDPHS_READEPTE[16384];     // FIFO Endpoint Data Register 14
    volatile uint32_t     UDPHS_READEPTF[16384];     // FIFO Endpoint Data Register 15
};

struct AT91_UsbClientController {
    USB_CONTROLLER_STATE *usbState;

    uint8_t            controlPacketBuffer[USB_MAX_EP0_SIZE];
    uint8_t            previousDeviceState;
    uint16_t        endpointStatus[USB_MAX_EP_COUNT];

    bool            firstDescriptorPacket;
    bool            txRunning[AT91_USB_QUEUE_SIZE];
    bool            txNeedZLPS[AT91_USB_QUEUE_SIZE];
};

AT91_UsbClientController at91_UsbClientController[TOTAL_USB_CONTROLLER];

struct AT91_UDP_ENDPOINT_ATTRIBUTE {
    uint16_t        Dir_Type;
    uint16_t        Payload;
    bool        DualBank;
    uint32_t        dFlag;
};

void AT91_UsbClient_StallEndPoint(uint32_t ep);

uint32_t AT91_UsbClient_WriteEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t cnt);
uint32_t AT91_UsbClient_ReadEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t len);

void AT91_UsbClient_ConfigEndPoint();
void AT91_UsbClient_EndpointIsr(USB_CONTROLLER_STATE* usbState, uint32_t endpoint);
void AT91_UsbClient_ClearTxQueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint);


__inline void AT91_UsbClient_PmcEnableUsbClock(void) {
    (*(volatile uint32_t *)0xFFFFFC10) = 1 << AT91C_ID_UDP;
    (*(volatile uint32_t *)0xFFFFFC1C) |= AT91C_CKGR_UPLLEN_ENABLED;
    (*(volatile uint32_t *)(AT91C_BASE_UDP + 0xE0)) |= 3; // Full Speed
}

__inline void AT91_UsbClient_PmcEnableUTMIBIAS(void) {
    // TO DO
}
__inline void AT91_UsbClient_PmcDisableUsbClock(void) {
    (*(volatile uint32_t *)0xFFFFFC14) = 1 << AT91C_ID_UDP;
    (*(volatile uint32_t *)0xFFFFFC1C) &= ~AT91C_CKGR_UPLLEN_ENABLED;
}

__inline void AT91_UsbClient_PmcDisableUTMIBIAS(void) {
    (*(volatile uint32_t *)0xFFFFFC1C) &= ~AT91C_CKGR_BIASEN_ENABLED;
}

AT91_UDP_ENDPOINT_ATTRIBUTE AT91_UsbClient_EndpointAttr[] =
{
    {AT91C_UDPHS_EPT_TYPE_CTL_EPT | AT91C_UDPHS_EPT_DIR_OUT , USB_CTRL_WMAXPACKETSIZE0_EP_WRITE, false, AT91C_UDPHS_BK_NUMBER_1},
    {0                                                      , USB_BULK_WMAXPACKETSIZE_EP_WRITE , true , AT91C_UDPHS_BK_NUMBER_2},
    {0                                                      , USB_BULK_WMAXPACKETSIZE_EP_READ  , true , AT91C_UDPHS_BK_NUMBER_2},
};

void AT91_UsbClient_SuspendEvent(int32_t controller) {
    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // Enable wakeup and resume
    pUdp->UDPHS_IEN |= AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_ENDOFRSM;
    // Disbale suspend interrupt
    pUdp->UDPHS_IEN &= ~AT91C_UDPHS_DET_SUSPD;

    // The device enters the Suspended state
    // MCK + UDPCK must be off
    // Pull-Up must be connected
    // Transceiver must be disabled

    // TODO: will be replaced by PMC API
    AT91_UsbClient_PmcDisableUsbClock();

    at91_UsbClientController[controller].previousDeviceState = usbState->deviceState;

    usbState->deviceState = USB_DEVICE_STATE_SUSPENDED;

    AT91_UsbClient_StateCallback(usbState);
}


void AT91_UsbClient_ResetEvent(USB_CONTROLLER_STATE *usbState) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // MCK + UDPCK are already enabled
    // Pull-Up is already connected
    // Transceiver must be enabled
    // Endpoint 0 must be enabled

    // program Endpoint Status/control
    int32_t endpointCount = sizeof(AT91_UsbClient_EndpointAttr) / sizeof(AT91_UDP_ENDPOINT_ATTRIBUTE);

    for (int32_t i = 0; i < endpointCount; i++) {
        uint32_t dwEptcfg = 0;
        // Reset endpoint fifos
        pUdp->UDPHS_EPTRST = 1 << i;
        // Enable endpoint interrupt
        pUdp->UDPHS_IEN |= (1 << SHIFT_INTERUPT << i);
        // Set endpoint configration
        dwEptcfg = AT91_UsbClient_EndpointAttr[i].Dir_Type | AT91_UsbClient_EndpointAttr[i].dFlag;
        switch (AT91_UsbClient_EndpointAttr[i].Payload) {
        case 8:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_8;
            break;
        case 16:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_16;
            break;
        case 32:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_32;
            break;
        case 64:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_64;
            break;
        case 128:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_128;
            break;
        case 256:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_256;
            break;
        case 512:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_512;
            break;
        case 1024:
            dwEptcfg |= AT91C_UDPHS_EPT_SIZE_1024;
            break;
        default:
            break;
        }

        pUdp->UDPHS_EPT[i].UDPHS_EPTCFG = dwEptcfg;


        pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL | AT91C_UDPHS_RX_BK_RDY;// | AT91C_UDPHS_TX_PK_RDY;
    }

    pUdp->UDPHS_EPT[0].UDPHS_EPTCTLENB = AT91C_UDPHS_RX_SETUP | AT91C_UDPHS_TX_PK_RDY;

    /* clear all flags */
    AT91_UsbClient_ClearEvent(usbState, 0xFFFFFFFF); // clear all events on all endpoints

    for (int32_t ep = 0; ep < USB_MAX_EP_COUNT; ep++) {
        at91_UsbClientController[usbState->controllerNum].txRunning[ep] = false;
        at91_UsbClientController[usbState->controllerNum].txNeedZLPS[ep] = false;
    }

    usbState->deviceState = USB_DEVICE_STATE_DEFAULT;
    usbState->address = 0;
    AT91_UsbClient_StateCallback(usbState);
}

void AT91_UsbClient_ResumeEvent(USB_CONTROLLER_STATE *usbState) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // The device enters Configured state
    // MCK + UDPCK must be on
    // Pull-Up must be connected
    // Transceiver must be enabled

    // TODO: will be replaced by PMC API
    AT91_UsbClient_PmcEnableUsbClock();
    AT91_UsbClient_PmcEnableUTMIBIAS();

    usbState->deviceState = at91_UsbClientController[usbState->controllerNum].previousDeviceState;

    AT91_UsbClient_StateCallback(usbState);

    // Enable end of reset and suspend interrupt
    pUdp->UDPHS_IEN |= AT91C_UDPHS_ENDOFRSM;// | AT91C_UDPHS_DET_SUSPD;

    // Disable Wakeup interrupt
    pUdp->UDPHS_IEN &= ~AT91C_UDPHS_WAKE_UP;

}

void AT91_UsbClient_InterruptHandler(void *param) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    uint32_t USB_INTR = (pUdp->UDPHS_INTSTA & pUdp->UDPHS_IEN);
    uint32_t endpoint = 0;

    //TQD
    //int32_t controller = *((int32_t*)param);

    USB_CONTROLLER_STATE *usbState = (USB_CONTROLLER_STATE*)param;

    // Handle all UDP interrupts
    while (USB_INTR != 0) {
        // Start Of Frame (SOF)
        if (USB_INTR & AT91C_UDPHS_IEN_SOF) {
            // Acknowledge interrupt
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_IEN_SOF;
            USB_INTR &= ~AT91C_UDPHS_IEN_SOF;
            // This interrupt should not happen, as it is not enabled.
        }

        // Suspend
        if (USB_INTR & AT91C_UDPHS_DET_SUSPD) {
            // Acknowledge interrupt
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_DET_SUSPD | AT91C_UDPHS_WAKE_UP;
            USB_INTR &= ~AT91C_UDPHS_DET_SUSPD;
        }

        // Resume or Wakeup
        if ((USB_INTR & AT91C_UDPHS_WAKE_UP) || (USB_INTR & AT91C_UDPHS_ENDOFRSM)) {
            AT91_UsbClient_ResumeEvent(usbState);

            // Acknowledge interrupt
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_ENDOFRSM;
            USB_INTR &= ~(AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_ENDOFRSM);
        }

        // End of bus reset
        if (USB_INTR & AT91C_UDPHS_ENDRESET) {
            // Acknowledge end of bus reset interrupt

            AT91_UsbClient_ResetEvent(usbState);
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_WAKE_UP | AT91C_UDPHS_DET_SUSPD | AT91C_UDPHS_ENDRESET;

            USB_INTR &= ~AT91C_UDPHS_ENDRESET;
        }

        if (USB_INTR & AT91C_UDPHS_UPSTR_RES) {
            pUdp->UDPHS_CLRINT = AT91C_UDPHS_UPSTR_RES;
            USB_INTR &= ~AT91C_UDPHS_UPSTR_RES;
        }
        else //Endpoint Interrupt
        {
            uint32_t i = 0;
            USB_INTR >>= 8;
            while (USB_INTR != 0) {
                if (USB_INTR & 1) {
                    endpoint = i;
                    AT91_UsbClient_EndpointIsr(usbState, endpoint);

                }
                USB_INTR >>= 1;
                i++;
            }
        }
        USB_INTR = pUdp->UDPHS_INTSTA & pUdp->UDPHS_IEN;
    }
}

void AT91_UsbClient_VbusInterruptHandler(int32_t controller, int32_t pin, bool pinState, void* param) {
    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    // VBus High
    if (pinState) {
        // Enable USB clock
        AT91_UsbClient_PmcEnableUsbClock();

        pUdp->UDPHS_CTRL |= AT91C_UDPHS_DETACH;

        // Reset and enable IP UDPHS
        pUdp->UDPHS_CTRL &= ~AT91C_UDPHS_EN_UDPHS;
        pUdp->UDPHS_CTRL |= AT91C_UDPHS_EN_UDPHS;

        // With OR without DMA !!!
        // Initialization of DMA
        for (int32_t i = 1; i <= ((pUdp->UDPHS_IPFEATURES & AT91C_UDPHS_DMA_CHANNEL_NBR) >> 4); i++) {
            // RESET endpoint canal DMA:
            // DMA stop channel command
            pUdp->UDPHS_DMA[i].UDPHS_DMACONTROL = 0;  // STOP command

            // Disable endpoint
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLDIS = 0xFFFFFFFF;

            // Reset endpoint config
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = 0;

            // Reset DMA channel (Buff count and Control field)
            pUdp->UDPHS_DMA[i].UDPHS_DMACONTROL = 0x02;  // NON STOP command

            // Reset DMA channel 0 (STOP)
            pUdp->UDPHS_DMA[i].UDPHS_DMACONTROL = 0;  // STOP command

            // Clear DMA channel status (read the register for clear it)
            pUdp->UDPHS_DMA[i].UDPHS_DMASTATUS = pUdp->UDPHS_DMA[i].UDPHS_DMASTATUS;

        }

        pUdp->UDPHS_IEN = AT91C_UDPHS_ENDOFRSM | AT91C_UDPHS_WAKE_UP;// | AT91C_UDPHS_DET_SUSPD;
        pUdp->UDPHS_CLRINT = 0xFE;

        // Pull up the DP line
        pUdp->UDPHS_CTRL &= ~AT91C_UDPHS_DETACH; // attach

        usbState->deviceState = USB_DEVICE_STATE_ATTACHED;

        AT91_UsbClient_StateCallback(usbState);
    }
    else // VBus Low
    {
        // clear USB Txbuffer
        for (int32_t ep = 0; ep < USB_MAX_EP_COUNT; ep++) {
            if (usbState->isTxQueue[ep] && usbState->queues[ep] != NULL)
                usb_fifo_buffer_in[ep] = usb_fifo_buffer_out[ep] = usb_fifo_buffer_count[ep] = 0;
        }

        pUdp->UDPHS_CTRL |= AT91C_UDPHS_DETACH;

        // TODO: will be replaced by PMC API
        // Disable  USB clock
        AT91_UsbClient_PmcDisableUsbClock();

        usbState->deviceState = USB_DEVICE_STATE_DETACHED;
        AT91_UsbClient_StateCallback(usbState);
    }
}

bool AT91_UsbClient_ProtectPins(int32_t controller, bool On) {
    USB_CONTROLLER_STATE *usbState = &usbClientState[controller];

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (usbState) {
        if (On) {
            AT91_UsbClient_VbusInterruptHandler(controller, 0, true, nullptr);

            struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;
            pUdp->UDPHS_CTRL &= ~AT91C_UDPHS_DETACH; // attach*)
        }
        else {
            for (int32_t ep = 0; ep < USB_MAX_EP_COUNT; ep++) {
                if (usbState->queues[ep] && usbState->isTxQueue[ep])
                    AT91_UsbClient_ClearTxQueue(usbState, ep);

            }

            //detach D plus line
            struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;
            pUdp->UDPHS_CTRL |= AT91C_UDPHS_DETACH; // dettach

            //--//
            usbState->deviceState = USB_DEVICE_STATE_DETACHED;
            AT91_UsbClient_StateCallback(usbState);
        }

        return true;
    }

    return false;
}

void AT91_UsbClient_ClearTxQueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    // it is much faster to just re-initialize the queue and it does the same thing
    if (usbState->queues[endpoint] != NULL) {
        usb_fifo_buffer_in[endpoint] = usb_fifo_buffer_out[endpoint] = usb_fifo_buffer_count[endpoint] = 0;
    }
}

#define USBH_TRANSFER_PACKET_TIMEOUT 4000
void AT91_UsbClient_TxPacket(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    int32_t timeout;

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    uint8_t * pDest = (uint8_t*)((((struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA)->UDPHS_READEPT0) + 16384 * endpoint);

    DISABLE_INTERRUPTS_SCOPED(irq);

    // transmit a packet on UsbPortNum, if there are no more packets to transmit, then die

    timeout = 0;
    while (!pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSTA & AT91C_UDPHS_TX_COMPLT) {
        timeout++;
        if (timeout > USBH_TRANSFER_PACKET_TIMEOUT) {
            return;
        }
    }
    timeout = 0;
    while (pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSTA & AT91C_UDPHS_TX_PK_RDY) {
        timeout++;
        if (timeout > USBH_TRANSFER_PACKET_TIMEOUT) {
            return;
        }
    }
    USB_PACKET64* Packet64;

    for (;;) {
        Packet64 = AT91_UsbClient_TxDequeue(usbState, endpoint);

        if (Packet64 == nullptr || Packet64->Size > 0) {
            break;
        }
    }

    if (Packet64) {
        int32_t i;

        AT91_UsbClient_WriteEndPoint(endpoint, Packet64->Buffer, Packet64->Size);
        at91_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint] = (Packet64->Size == USB_BULK_WMAXPACKETSIZE_EP_WRITE);
    }
    else {
        // send the zero leght packet since we landed on the FIFO boundary before
        // (and we queued a zero length packet to transmit)
        if (at91_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint]) {
            pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSETSTA = AT91C_UDPHS_TX_PK_RDY;
            at91_UsbClientController[usbState->controllerNum].txNeedZLPS[endpoint] = false;
        }

        // no more data
        at91_UsbClientController[usbState->controllerNum].txRunning[endpoint] = false;
        pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLDIS = AT91C_UDPHS_TX_PK_RDY;
    }
}

void AT91_UsbClient_ControlNext(USB_CONTROLLER_STATE *usbState) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    uint8_t *pFifo = (uint8_t *)&(((struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA)->UDPHS_READEPT0[0]);

    if (usbState->dataCallback) {
        /* this call can't fail */
        usbState->dataCallback(usbState);

        if (usbState->dataSize == 0) {
            AT91_UsbClient_WriteEndPoint(0, (uint8_t*)NULL, 0);
        }
        else {
            AT91_UsbClient_WriteEndPoint(0, usbState->ptrData, usbState->dataSize);

            // special handling the USB driver set address test, cannot use the first descriptor as the ADDRESS state is handle in the hardware
            if (at91_UsbClientController[usbState->controllerNum].firstDescriptorPacket) {
                usbState->dataCallback = NULL;
            }
        }
    }
    else {
        pUdp->UDPHS_EPT[0].UDPHS_EPTCLRSTA = AT91C_UDPHS_TX_COMPLT;
        pUdp->UDPHS_EPT[0].UDPHS_EPTCTLDIS = AT91C_UDPHS_TX_PK_RDY;
    }
}

void AT91_UsbClient_StallEndPoint(uint32_t ep) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    pUdp->UDPHS_EPT[ep].UDPHS_EPTSETSTA = UDPHS_EPTSETSTA_FRCESTALL;
}

uint32_t AT91_UsbClient_WriteEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t cnt) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);

    uint8_t * pDest = (uint8_t*)((((struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA)->UDPHS_READEPT0) + 16384 * EPNum);

    memcpy(pDest, pData, cnt);

    pUdp->UDPHS_EPT[EPNum].UDPHS_EPTSETSTA = AT91C_UDPHS_TX_PK_RDY;

    return cnt;
}
uint32_t AT91_UsbClient_ReadEndPoint(uint32_t EPNum, uint8_t *pData, uint32_t len) {
    struct AT91_UDPHS_EPTFIFO *pFifo = (struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA;
    uint8_t *pDest = (uint8_t *)(pFifo->UDPHS_READEPT0 + 16384 * EPNum);

    memcpy(pData, pDest, len);
    return len;
}

void AT91_UsbClient_ConfigEndPoint() {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    int32_t i;

    for (i = 1; i <= 2; i++) {
        if ((AT91_UsbClient_EndpointAttr[i].Dir_Type & AT91C_UDPHS_EPT_DIR_OUT) == AT91C_UDPHS_EPT_DIR_OUT) {
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL | AT91C_UDPHS_RX_BK_RDY;
        }
        else if ((AT91_UsbClient_EndpointAttr[i].Dir_Type & AT91C_UDPHS_EPT_DIR_IN) == AT91C_UDPHS_EPT_DIR_IN) {
            pUdp->UDPHS_EPT[i].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL;
        }
    }
    pUdp->UDPHS_EPT[4].UDPHS_EPTCTLENB = AT91C_UDPHS_EPT_ENABL;
}

void AT91_UsbClient_EndpointIsr(USB_CONTROLLER_STATE *usbState, uint32_t endpoint) {
    uint32_t Status;

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    struct AT91_UDPHS_EPTFIFO *pFifo = (struct AT91_UDPHS_EPTFIFO *)AT91C_BASE_UDP_DMA;

    Status = pUdp->UDPHS_EPT[endpoint].UDPHS_EPTSTA;

    // Control Endpoint
    if (endpoint == 0) {
        // ugly
        if (Status & AT91C_UDPHS_RX_BK_RDY) {
            while (pUdp->UDPHS_EPT[0].UDPHS_EPTSTA & AT91C_UDPHS_RX_BK_RDY)
                pUdp->UDPHS_EPT[0].UDPHS_EPTCLRSTA = AT91C_UDPHS_RX_BK_RDY;
        }

        // set up packet receive
        if (Status & AT91C_UDPHS_RX_SETUP) {
            uint8_t len = (pUdp->UDPHS_EPT[0].UDPHS_EPTSTA >> 20) & 0x7F;
            (*(uint32_t *)(&at91_UsbClientController[usbState->controllerNum].controlPacketBuffer[0])) = pFifo->UDPHS_READEPT0[0];
            (*(uint32_t *)(&at91_UsbClientController[usbState->controllerNum].controlPacketBuffer[4])) = pFifo->UDPHS_READEPT0[0];

            // special handling for the very first SETUP command - Getdescriptor[DeviceType], the host looks for 8 bytes data only
            USB_SETUP_PACKET* Setup = (USB_SETUP_PACKET*)&at91_UsbClientController[usbState->controllerNum].controlPacketBuffer[0];


            if ((Setup->bRequest == USB_GET_DESCRIPTOR) && (((Setup->wValue & 0xFF00) >> 8) == USB_DEVICE_DESCRIPTOR_TYPE) && (Setup->wLength != 0x12))
                at91_UsbClientController[usbState->controllerNum].firstDescriptorPacket = true;
            else
                at91_UsbClientController[usbState->controllerNum].firstDescriptorPacket = false;


            while (pUdp->UDPHS_EPT[0].UDPHS_EPTSTA & AT91C_UDPHS_RX_SETUP)
                pUdp->UDPHS_EPT[0].UDPHS_EPTCLRSTA = AT91C_UDPHS_RX_SETUP;

            /* send it to the upper layer */
            usbState->ptrData = &at91_UsbClientController[usbState->controllerNum].controlPacketBuffer[0];
            usbState->dataSize = len;

            pUdp->UDPHS_EPT[0].UDPHS_EPTCTLENB = AT91C_UDPHS_TX_PK_RDY;

            uint8_t result = AT91_UsbClient_ControlCallback(usbState);

            switch (result) {
            case USB_STATE_DATA:
                /* setup packet was handled and the upper layer has data to send */
                break;

            case USB_STATE_ADDRESS:
                /* upper layer needs us to change the address */
                // address stage handles in hardware
                break;

            case USB_STATE_DONE:
                usbState->dataCallback = NULL;
                break;

            case USB_STATE_STALL:
                // since the setup command all handled in the hardware, should not have this state
                //
                // setup packet failed to process successfully
                // set stall condition on the default control
                // endpoint
                //
                AT91_UsbClient_StallEndPoint(0);
                break;

            case USB_STATE_STATUS:
                // handle by hardware
                break;

            case USB_STATE_CONFIGURATION:
                /* USB spec 9.4.5 SET_CONFIGURATION resets halt conditions, resets toggle bits */
                AT91_UsbClient_ConfigEndPoint();
                break;

            case USB_STATE_REMOTE_WAKEUP:
                // It is not using currently as the device side won't go into SUSPEND mode unless
                // the PC is purposely to select it to SUSPEND, as there is always SOF in the bus
                // to keeping the device from SUSPEND.
                break;

            default:
                return;
                // the status change is only seen and taken care in hardware
            }

            if (result != USB_STATE_STALL) {
                AT91_UsbClient_ControlNext(usbState);
                // If port is now configured, output any queued data
                if (result == USB_STATE_CONFIGURATION) {
                    for (int32_t ep = 1; ep < USB_MAX_EP_COUNT; ep++) {
                        if (usbState->queues[ep] && usbState->isTxQueue[ep])
                            AT91_UsbClient_StartOutput(usbState, ep);
                    }
                }
            }
        }
        else if (!(Status & AT91C_UDPHS_TX_PK_RDY) && (pUdp->UDPHS_EPT[0].UDPHS_EPTCTL & AT91C_UDPHS_TX_PK_RDY)) {
            AT91_UsbClient_ControlNext(usbState);

            if (usbState->address) {
                if (!(pUdp->UDPHS_CTRL & AT91C_UDPHS_FADDR_EN))
                    pUdp->UDPHS_CTRL |= AT91C_UDPHS_FADDR_EN | usbState->address;
            }
        }
    }
    else {

        if ((AT91_UsbClient_EndpointAttr[endpoint].Dir_Type & AT91C_UDPHS_EPT_DIR_IN) == AT91C_UDPHS_EPT_DIR_IN) {
            AT91_UsbClient_TxPacket(usbState, endpoint);
        }
        // OUT packet received
        else if (((Status & AT91C_UDPHS_RX_BK_RDY) != 0) && ((AT91_UsbClient_EndpointAttr[endpoint].Dir_Type & AT91C_UDPHS_EPT_DIR_OUT) == AT91C_UDPHS_EPT_DIR_OUT)) {
            bool          DisableRx;

            uint32_t len = ((Status & AT91C_UDPHS_BYTE_COUNT) >> 20) & 0x7FF;
            uint8_t *pDest = (uint8_t *)(pFifo->UDPHS_READEPT0 + 16384 * endpoint);
            uint8_t block = len / USB_MAX_DATA_PACKET_SIZE;
            uint8_t rest = len % USB_MAX_DATA_PACKET_SIZE;
            while (block > 0) {
                USB_PACKET64* Packet64 = AT91_UsbClient_RxEnqueue(usbState, endpoint, DisableRx);
                if (!DisableRx) {

                    memcpy(&(Packet64->Buffer[0]), pDest, USB_MAX_DATA_PACKET_SIZE);
                    Packet64->Size = USB_MAX_DATA_PACKET_SIZE;
                    pDest += USB_MAX_DATA_PACKET_SIZE;
                    block--;
                }
            }
            if ((rest > 0) && (block == 0)) {
                USB_PACKET64* Packet64 = AT91_UsbClient_RxEnqueue(usbState, endpoint, DisableRx);
                if (!DisableRx) {
                    memcpy(&(Packet64->Buffer[0]), pDest, rest);
                    pDest += rest;
                    Packet64->Size = rest;
                }
            }

            pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCLRSTA = AT91C_UDPHS_RX_BK_RDY;

            if (DisableRx) {
                pUdp->UDPHS_IEN &= ~(1 << SHIFT_INTERUPT << endpoint);
                pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLDIS = AT91C_UDPHS_RX_BK_RDY;
            }
        }
    }
}

bool AT91_UsbClient_Initialize(USB_CONTROLLER_STATE *usbState) {
    if (usbState == nullptr)
        return false;

    int32_t controller = usbState->controllerNum;

    at91_UsbClientController[controller].usbState = usbState;

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) (AT91C_BASE_UDP);
    struct AT91_UDPHS_EPT *pEp = (struct AT91_UDPHS_EPT *) (AT91C_BASE_UDP + 0x100);

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Enable USB device clock
    AT91_UsbClient_PmcEnableUsbClock();
    AT91_UsbClient_PmcEnableUTMIBIAS();

    // Enable the interrupt for  UDP
    AT91_Interrupt_Activate(AT91C_ID_UDP, (uint32_t*)&AT91_UsbClient_InterruptHandler, (void*)usbState);

    pUdp->UDPHS_IEN |= AT91C_UDPHS_EPT_INT_0;
    pEp->UDPHS_EPTCFG |= 0x00000043; //configuration info for control ep

    for (auto pipe = 0; pipe < USB_MAX_EP_COUNT; pipe++) {
        auto idx = 0;
        if (usbState->pipes[pipe].RxEP != USB_ENDPOINT_NULL) {
            idx = usbState->pipes[pipe].RxEP;
            usbState->maxPacketSize[idx] = USB_BULK_WMAXPACKETSIZE_EP_READ;
            AT91_UsbClient_EndpointAttr[idx].Dir_Type = AT91C_UDPHS_EPT_TYPE_BUL_EPT;
            AT91_UsbClient_EndpointAttr[idx].Dir_Type |= AT91C_UDPHS_EPT_DIR_OUT;
            pUdp->UDPHS_IEN |= (AT91C_UDPHS_EPT_INT_0 << idx);
        }

        if (usbState->pipes[pipe].TxEP != USB_ENDPOINT_NULL) {
            idx = usbState->pipes[pipe].TxEP;
            usbState->maxPacketSize[idx] = USB_BULK_WMAXPACKETSIZE_EP_WRITE;
            AT91_UsbClient_EndpointAttr[idx].Dir_Type = AT91C_UDPHS_EPT_TYPE_BUL_EPT;
            AT91_UsbClient_EndpointAttr[idx].Dir_Type |= AT91C_UDPHS_EPT_DIR_IN;
            pUdp->UDPHS_IEN |= (AT91C_UDPHS_EPT_INT_0 << idx);
        }
    }

    usbState->endpointStatus = &at91_UsbClientController[usbState->controllerNum].endpointStatus[0];
    usbState->endpointCount = USB_MAX_EP_COUNT;
    usbState->packetSize = AT91_UsbClient_EndpointAttr[0].Payload;

    usbState->firstGetDescriptor = true;

    AT91_UsbClient_ProtectPins(controller, true);

    AT91_Time_Delay(nullptr, 100000); // 100ms

    return true;
}

bool AT91_UsbClient_Uninitialize(USB_CONTROLLER_STATE *usbState) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_UsbClient_PmcDisableUTMIBIAS();
    AT91_UsbClient_PmcDisableUsbClock();

    AT91_Interrupt_Deactivate(AT91C_ID_UDP);

    if (usbState != nullptr) {
        AT91_UsbClient_ProtectPins(usbState->controllerNum, false);
        usbState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
    }

    return true;
}

bool AT91_UsbClient_StartOutput(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;

    // If endpoint is not an output
    if (usbState->queues[endpoint] == NULL || !usbState->isTxQueue[endpoint])
        return false;

    pUdp->UDPHS_IEN |= 1 << SHIFT_INTERUPT << endpoint;
    pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLENB = AT91C_UDPHS_TX_PK_RDY;

    /* if the halt feature for this endpoint is set, then just
            clear all the characters */
    if (at91_UsbClientController[usbState->controllerNum].endpointStatus[endpoint] & USB_STATUS_ENDPOINT_HALT) {
        AT91_UsbClient_ClearTxQueue(usbState, endpoint);
        return true;
    }

    //If txRunning, interrupts will drain the queue
    if (!(bool)((uint8_t)at91_UsbClientController[usbState->controllerNum].txRunning[endpoint])) {

        at91_UsbClientController[usbState->controllerNum].txRunning[endpoint] = true;

        AT91_UsbClient_TxPacket(usbState, endpoint);
    }
    else if (irq.WasDisabled()) // We should not call EP_TxISR after calling AT91_UsbClient_TxPacket becuase it can cause a TX FIFO overflow error.
    {
        AT91_UsbClient_EndpointIsr(usbState, endpoint);
    }

    return true;
}

bool AT91_UsbClient_RxEnable(int32_t endpoint) {
    struct AT91_UDPHS *pUdp = (struct AT91_UDPHS *) AT91C_BASE_UDP;

    pUdp->UDPHS_IEN |= 1 << SHIFT_INTERUPT << endpoint;
    pUdp->UDPHS_EPT[endpoint].UDPHS_EPTCTLENB = AT91C_UDPHS_RX_BK_RDY;

    return true;
}
