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

#include <vector>
#include "LPC17.h"

#ifdef DEBUG
#define USB_DEBUG_ASSERT(x) while(!(x))
#else
#define USB_DEBUG_ASSERT(x)
#endif

// USB 2.0 host requests
#define USB_GET_STATUS           0
#define USB_CLEAR_FEATURE        1
#define USB_SET_FEATURE          3
#define USB_SET_ADDRESS          5
#define USB_GET_DESCRIPTOR       6
#define USB_SET_DESCRIPTOR       7
#define USB_GET_CONFIGURATION    8
#define USB_SET_CONFIGURATION    9
#define USB_GET_INTERFACE       10
#define USB_SET_INTERFACE       11
#define USB_SYNCH_FRAME         12

// USB 2.0 defined descriptor types
#define USB_DEVICE_DESCRIPTOR_TYPE        1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE 2
#define USB_STRING_DESCRIPTOR_TYPE        3
#define USB_INTERFACE_DESCRIPTOR_TYPE     4
#define USB_ENDPOINT_DESCRIPTOR_TYPE      5

// USB 2.0 host request type defines
#define USB_SETUP_DIRECTION(n)          ((n) & 0x80)
#define USB_SETUP_DIRECTION_DEVICE      0x00
#define USB_SETUP_DIRECTION_HOST        0x80

#define USB_SETUP_TYPE(n)        ((n) & 0x70)
#define USB_SETUP_TYPE_STANDARD         0x00
#define USB_SETUP_TYPE_CLASS            0x10
#define USB_SETUP_TYPE_VENDOR           0x20
#define USB_SETUP_TYPE_RESERVED         0x30

#define USB_SETUP_RECIPIENT(n)          ((n) & 0x0F)
#define USB_SETUP_RECIPIENT_DEVICE             0x00
#define USB_SETUP_RECIPIENT_INTERFACE          0x01
#define USB_SETUP_RECIPIENT_ENDPOINT           0x02
#define USB_SETUP_RECIPIENT_OTHER              0x03

// Local device status defines
#define USB_STATUS_DEVICE_NONE           0x0000
#define USB_STATUS_DEVICE_SELF_POWERED   0x0001
#define USB_STATUS_DEVICE_REMOTE_WAKEUP  0x0002

#define USB_STATUS_INTERFACE_NONE        0x0000

#define USB_STATUS_ENDPOINT_NONE         0x0000
#define USB_STATUS_ENDPOINT_HALT         0x0001

#define USB_FEATURE_DEVICE_REMOTE_WAKEUP 0x0001
#define USB_FEATURE_ENDPOINT_HALT        0x0000

// Local device possible states
#define USB_DEVICE_STATE_DETACHED       0
#define USB_DEVICE_STATE_ATTACHED       1
#define USB_DEVICE_STATE_POWERED        2
#define USB_DEVICE_STATE_DEFAULT        3
#define USB_DEVICE_STATE_ADDRESS        4
#define USB_DEVICE_STATE_CONFIGURED     5
#define USB_DEVICE_STATE_SUSPENDED      6
#define USB_DEVICE_STATE_NO_CONTROLLER  0xFE
#define USB_DEVICE_STATE_UNINITIALIZED  0xFF

// Possible responses to host requests
#define USB_STATE_DATA                  0
#define USB_STATE_STALL                 1
#define USB_STATE_DONE                  2
#define USB_STATE_ADDRESS               3
#define USB_STATE_STATUS                4
#define USB_STATE_CONFIGURATION         5
#define USB_STATE_REMOTE_WAKEUP         6

#define USB_CURRENT_UNIT                2

// Endpoint Attribute
#define USB_ENDPOINT_ATTRIBUTE_ISOCHRONOUS 1
#define USB_ENDPOINT_ATTRIBUTE_BULK 2
#define USB_ENDPOINT_ATTRIBUTE_INTERRUPT 3

#define USB_MAX_DATA_PACKET_SIZE 64

struct USB_PACKET64 {
    uint32_t Size;
    uint8_t  Buffer[USB_MAX_DATA_PACKET_SIZE];
};

#define USB_NULL_ENDPOINT 0xFF

struct USB_STREAM_MAP {
    uint8_t RxEP;
    uint8_t TxEP;
};

PACKED(struct) USB_DYNAMIC_CONFIGURATION;

struct USB_CONTROLLER_STATE;

typedef void(*USB_NEXT_CALLBACK)(USB_CONTROLLER_STATE*);

struct USB_CONTROLLER_STATE {
    bool                                                        Initialized;
    uint8_t                                                     CurrentState;
    uint8_t                                                     ControllerNum;
    uint32_t                                                    Event;

    const USB_DYNAMIC_CONFIGURATION*                            Configuration;

    /* Queues & MaxPacketSize must be initialized by the HAL */
    std::vector<USB_PACKET64>                                   *Queues[USB_MAX_QUEUES];
    uint8_t                                                     CurrentPacketOffset[USB_MAX_QUEUES];
    uint8_t                                                     MaxPacketSize[USB_MAX_QUEUES];
    bool                                                        IsTxQueue[USB_MAX_QUEUES];

    /* Arbitrarily as many streams as endpoints since that is the maximum number of streams
       necessary to represent the maximum number of endpoints */
    USB_STREAM_MAP                                              streams[USB_MAX_QUEUES];

    //--//

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

    bool                                                        Configured;
};

// USB 2.0 request packet from host
PACKED(struct) USB_SETUP_PACKET {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

struct UsbClient_Driver {
    static bool Initialize(int controller);
    static bool Uninitialize(int controller);

    static bool OpenStream(int controller, int32_t& stream, TinyCLR_UsbClient_StreamMode mode);
    static bool CloseStream(int controller, int stream);

    static int  Write(int controller, int usbStream, const char* Data, size_t size);
    static int  Read(int controller, int usbStream, char*       Data, size_t size);
    static bool Flush(int controller, int usbStream);

    static uint32_t SetEvent(int controller, uint32_t Event);
    static uint32_t ClearEvent(int controller, uint32_t Event);

    static TinyCLR_UsbClient_DataReceivedHandler DataReceivedHandler;
    static TinyCLR_UsbClient_OsExtendedPropertyHandler OsExtendedPropertyHandler;

};

extern USB_PACKET64* LPC17_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* State, int queue, bool& DisableRx);
extern USB_PACKET64* LPC17_UsbClient_TxDequeue(USB_CONTROLLER_STATE* State, int queue, bool  Done);
extern uint8_t LPC17_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup, bool DataPhase);
extern uint8_t LPC17_UsbClient_ControlCallback(USB_CONTROLLER_STATE* State);
extern void  LPC17_UsbClient_StateCallback(USB_CONTROLLER_STATE* State);

/////////////////////////////////////////////////////////////////////////
// ATTENTION:
// 2.0 is the lowest version that works with WinUSB on Windows 8!!!
// use older values below if you do not care about that
//
#define DEVICE_RELEASE_VERSION              0x0200

//string descriptor
#define USB_STRING_DESCRIPTOR_SIZE          32
// NOTE: Having more than (probably) 32 characters causes the MFUSB KERNEL driver
// to *CRASH* which, of course, causes Windows to crash

// NOTE: If these two strings ( displayString, firendly )are not present, the MFUSB KERNEL driver will *CRASH*
// which, of course, causes Windows to crash

// index for the strings
#define MANUFACTURER_NAME_INDEX             1
#define PRODUCT_NAME_INDEX                  2
#define SERIAL_NUMBER_INDEX                 0

// Configuration for extended descriptor
#define OS_DESCRIPTOR_EX_VERSION            0x0100
//
/////////////////////////////////////////////////////////////////////////
#define USB_DISPLAY_STRING_NUM     4
#define USB_FRIENDLY_STRING_NUM    5

#define OS_DESCRIPTOR_STRING_INDEX        0xEE
#define OS_DESCRIPTOR_STRING_VENDOR_CODE  0xA5

// USB 2.0 response structure lengths
#define USB_STRING_DESCRIPTOR_MAX_LENGTH        126  // Maximum number of characters allowed in USB string descriptor
#define USB_FRIENDLY_NAME_LENGTH                 32
#define USB_DEVICE_DESCRIPTOR_LENGTH             18
#define USB_CONFIGURATION_DESCRIPTOR_LENGTH       9
#define USB_STRING_DESCRIPTOR_HEADER_LENGTH       2
// Sideshow descriptor lengths
#define OS_DESCRIPTOR_STRING_SIZE                18
#define OS_DESCRIPTOR_STRING_LENGTH               7
#define USB_XCOMPATIBLE_OS_SIZE                  40
#define USB_XPROPERTY_OS_SIZE_WINUSB     0x0000008E  // Size of this descriptor (78 bytes for guid + 40 bytes for the property name + 24 bytes for other fields = 142 bytes)
#define USB_XCOMPATIBLE_OS_REQUEST                4
#define USB_XPROPERTY_OS_REQUEST                  5

/////////////////////////////////////////////////////////////////////////////////////
// USB Configuration list structures
// Dynamic USB controller configuration is implemented as a packed list of structures.
// Each structure contains two parts: a header that describes the host request
// the structure satisfies, and the exact byte by byte structure that satisfies
// the host request.  The header also contains the size of the structure so that
// the next structure in the list can be quickly located.

// Marker values for the header portion of the USB configuration list structures.
// These specify the type of host request that the structure satisfies
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


// Generic Descriptor Header
#define USB_REQUEST_TYPE_OUT       0x00
#define USB_REQUEST_TYPE_IN        0x80
#define USB_REQUEST_TYPE_STANDARD  0x00
#define USB_REQUEST_TYPE_CLASS     0x20
#define USB_REQUEST_TYPE_VENDOR    0x40
#define USB_REQUEST_TYPE_DEVICE    0x00
#define USB_REQUEST_TYPE_INTERFACE 0x01
#define USB_REQUEST_TYPE_ENDPOINT  0x02

//XProperties Os WinUsb
#define EX_PROPERTY_DATA_TYPE__RESERVED                 0
#define EX_PROPERTY_DATA_TYPE__REG_SZ                   1
#define EX_PROPERTY_DATA_TYPE__REG_SZ_ENV               2
#define EX_PROPERTY_DATA_TYPE__REG_BINARY               3
#define EX_PROPERTY_DATA_TYPE__REG_DWORD_LITTLE_ENDIAN  4
#define EX_PROPERTY_DATA_TYPE__REG_DWORD_BIG_ENDIAN     5
#define EX_PROPERTY_DATA_TYPE__REG_LINK                 6
#define EX_PROPERTY_DATA_TYPE__REG_MULTI_SZ             7



bool LPC17_UsbClient_Initialize(int controller);
bool LPC17_UsbClient_SoftReset(int controller);
bool LPC17_UsbClient_Uninitialize(int controller);
bool LPC17_UsbClient_StartOutput(USB_CONTROLLER_STATE* State, int endpoint);
bool LPC17_UsbClient_RxEnable(USB_CONTROLLER_STATE* State, int endpoint);

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



/////////////////////////////////////////////////////////////
// The following structure defines the USB descriptor
// for a basic device with a USB debug interface via the
// WinUSB extended Compat ID.
//
// This USB configuration is always used to define the USB
// configuration for TinyBooter.  It is also the default for
// the runtime if there is no USB configuration in the Flash
// configuration sector.

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

//--//

void USB_ClearQueues(USB_CONTROLLER_STATE *State, bool ClrRxQueue, bool ClrTxQueue);

const TinyCLR_UsbClient_DescriptorHeader * USB_FindRecord(USB_CONTROLLER_STATE* State, uint8_t marker, USB_SETUP_PACKET * iValue);


#define ENDPOINT_INUSED_MASK        0x01
#define ENDPOINT_DIR_IN_MASK        0x02
#define ENDPOINT_DIR_OUT_MASK       0x04

extern USB_CONTROLLER_STATE UsbControllerState[1];

int8_t LPC17_UsbClient_EndpointMap[] = { ENDPOINT_INUSED_MASK,                          // Endpoint 0
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 1
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 2
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK   // Endpoint 3
};

/* Queues for all data endpoints */
static std::vector<USB_PACKET64> QueueBuffers[USB_MAX_QUEUES - 1];

// Usb client driver

#define USB_FLUSH_RETRY_COUNT 30

//--//

// This version of the USB code supports only one language - which
// is not specified by USB configuration records - it is defined here.
// This is the String 0 descriptor.This array includes the String descriptor
// header and exactly one language.

#define USB_LANGUAGE_DESCRIPTOR_SIZE 4

uint8_t USB_LanguageDescriptor[USB_LANGUAGE_DESCRIPTOR_SIZE] =
{
    USB_LANGUAGE_DESCRIPTOR_SIZE,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09, 0x04                      // U.S. English
};

// Device descriptor
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
    64,                                 // Endpoint 0 size
    USB_VENDOR_ID,                      // Vendor ID
    USB_PRODUCT_ID,                     // Product ID
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
    (100 / USB_CURRENT_UNIT),                              // Device max current draw

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
    64,                                          // Endpoint 1 packet size
    0 ,                                          // Endpoint 1 interval

    //Endpoint
    sizeof(TinyCLR_UsbClient_EndpointDescriptor),
    USB_ENDPOINT_DESCRIPTOR_TYPE,
    USB_ENDPOINT_DIRECTION_OUT,
    USB_ENDPOINT_ATTRIBUTE_BULK,
    64,                                         // Endpoint 1 packet size
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
        USB_MANUFACTURER_NAME
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
    USB_PRODUCT_NAME
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
    USB_DISPLAY_NAME
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
    USB_FRIENDLY_NAME
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

USB_DYNAMIC_CONFIGURATION UsbDefaultConfiguration;


TinyCLR_UsbClient_DataReceivedHandler UsbClient_Driver::DataReceivedHandler;
TinyCLR_UsbClient_OsExtendedPropertyHandler UsbClient_Driver::OsExtendedPropertyHandler;

bool UsbClient_Driver::Initialize(int controller) {

    USB_CONTROLLER_STATE *State = &UsbControllerState[controller];

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (State == nullptr)
        return false;

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

    if (State->Configured)
        return true;

    // Init Usb State
    memset(State, 0, sizeof(USB_CONTROLLER_STATE));

    State->ControllerNum = controller;
    State->Configuration = &UsbDefaultConfiguration;
    State->CurrentState = USB_DEVICE_STATE_UNINITIALIZED;
    State->DeviceStatus = USB_STATUS_DEVICE_SELF_POWERED;
    State->EndpointCount = USB_MAX_QUEUES;
    State->PacketSize = 64;
    State->Initialized = true;
    State->Configured = false;

    for (auto i = 0; i < USB_MAX_QUEUES; i++) {
        State->streams[i].RxEP = USB_NULL_ENDPOINT;
        State->streams[i].TxEP = USB_NULL_ENDPOINT;
        State->MaxPacketSize[i] = 64;
    }

    return State->Initialized;
}

bool UsbClient_Driver::Uninitialize(int controller) {
    USB_CONTROLLER_STATE *State = &UsbControllerState[controller];

    if (State == nullptr)
        return false;

    if (State->Configured)
        return true;

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17_UsbClient_Uninitialize(controller);

    State->Initialized = false;

    // for soft reboot allow the USB to be off for at least 100ms
    LPC17_Time_DelayNoInterrupt(nullptr, 100000); // 100ms

    return true;
}

bool UsbClient_Driver::OpenStream(int controller, int32_t& usbStream, TinyCLR_UsbClient_StreamMode mode) {
    USB_CONTROLLER_STATE * State = &UsbControllerState[controller];

    if (nullptr == State || !State->Initialized)     // If no such controller exists (or it is not initialized)
        return false;

    int32_t writeEp = USB_NULL_ENDPOINT;
    int32_t readEp = USB_NULL_ENDPOINT;

    switch (mode) {
        case TinyCLR_UsbClient_StreamMode::In:
            // TODO
            return false;

        case TinyCLR_UsbClient_StreamMode::Out:
            // TODO
            return false;

        case TinyCLR_UsbClient_StreamMode::InOut:

            for (auto i = 0; i < SIZEOF_CONST_ARRAY(LPC17_UsbClient_EndpointMap); i++) {
                if ((LPC17_UsbClient_EndpointMap[i] & ENDPOINT_INUSED_MASK)) // in used
                    continue;

                if (writeEp == USB_NULL_ENDPOINT && ((LPC17_UsbClient_EndpointMap[i] & ENDPOINT_DIR_IN_MASK) == ENDPOINT_DIR_IN_MASK)) {
                    writeEp = i;
                    LPC17_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

                    continue;
                }

                if (readEp == USB_NULL_ENDPOINT && ((LPC17_UsbClient_EndpointMap[i] & ENDPOINT_DIR_OUT_MASK) == ENDPOINT_DIR_OUT_MASK)) {
                    readEp = i;
                    LPC17_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

                    continue;
                }

                if (writeEp != 0 && readEp != 0) {
                    break;
                }
            }
            // Check the usbStream and the two endpoint numbers for validity (both endpoints cannot be zero)
            if ((readEp == USB_NULL_ENDPOINT && writeEp == USB_NULL_ENDPOINT)
                || (readEp != USB_NULL_ENDPOINT && (readEp < 1 || readEp >= USB_MAX_QUEUES))
                || (writeEp != USB_NULL_ENDPOINT && (writeEp < 1 || writeEp >= USB_MAX_QUEUES)))
                return false;

            // The specified endpoints must not be in use by another stream
            for (int stream = 0; stream < USB_MAX_QUEUES; stream++) {
                if (readEp != USB_NULL_ENDPOINT && (State->streams[stream].RxEP == readEp || State->streams[stream].TxEP == readEp))
                    return false;
                if (writeEp != USB_NULL_ENDPOINT && (State->streams[stream].RxEP == writeEp || State->streams[stream].TxEP == writeEp))
                    return false;
            }

            for (usbStream = 0; usbStream < USB_MAX_QUEUES; usbStream++) {
                // The Stream must be currently closed
                if (State->streams[usbStream].RxEP == USB_NULL_ENDPOINT && State->streams[usbStream].TxEP == USB_NULL_ENDPOINT)
                    break;
            }

            if (usbStream == USB_MAX_QUEUES)
                return false; // full endpoint

            // All tests pass, assign the endpoints to the stream
            State->streams[usbStream].RxEP = readEp;
            State->streams[usbStream].TxEP = writeEp;

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
                    State->IsTxQueue[idx] = true;
                }

                else if (ep->bEndpointAddress == USB_ENDPOINT_DIRECTION_OUT) {
                    ep->bEndpointAddress |= readEp;
                    idx = readEp;
                    State->IsTxQueue[idx] = false;
                }

                if (idx > 0) {
                    QueueBuffers[idx - 1] = std::vector< USB_PACKET64>();
                    State->Queues[idx] = &QueueBuffers[idx - 1];


                    State->MaxPacketSize[idx] = ep->wMaxPacketSize;
                }

                ep = (TinyCLR_UsbClient_EndpointDescriptor  *)(((uint8_t *)ep) + ep->bLength);
            }


            break;
    }

    if (State->CurrentState == USB_DEVICE_STATE_UNINITIALIZED) {
        LPC17_UsbClient_Initialize(controller);
    }
    else if (State->Configured) {
        LPC17_UsbClient_SoftReset(controller);
    }

    State->Configured = true;

    return true;
}

bool UsbClient_Driver::CloseStream(int controller, int usbStream) {
    USB_CONTROLLER_STATE * State = &UsbControllerState[controller];

    if (nullptr == State || !State->Initialized || usbStream >= USB_MAX_QUEUES)
        return false;

    int endpoint;
    DISABLE_INTERRUPTS_SCOPED(irq);

    // Close the Rx stream
    endpoint = State->streams[usbStream].RxEP;
    if (endpoint != USB_NULL_ENDPOINT && State->Queues[endpoint]) {
        State->Queues[endpoint]->clear(); // Clear the queue
        QueueBuffers[endpoint - 1] = std::vector< USB_PACKET64>();
    }

    State->streams[usbStream].RxEP = USB_NULL_ENDPOINT;
    //Free endpoint
    LPC17_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    // Close the TX stream
    endpoint = State->streams[usbStream].TxEP;
    if (endpoint != USB_NULL_ENDPOINT && State->Queues[endpoint]) {
        State->Queues[endpoint]->clear(); // Clear the queue
        QueueBuffers[endpoint - 1] = std::vector< USB_PACKET64>();
    }

    State->streams[usbStream].TxEP = USB_NULL_ENDPOINT;

    //Free endpoint
    LPC17_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    configDescriptor.epWrite.bEndpointAddress = USB_ENDPOINT_DIRECTION_IN;
    configDescriptor.epRead.bEndpointAddress = USB_ENDPOINT_DIRECTION_OUT;

    return true;
}

int UsbClient_Driver::Write(int controller, int usbStream, const char* Data, size_t size) {
    int endpoint;
    int totWrite = 0;
    USB_CONTROLLER_STATE * State = &UsbControllerState[controller];

    if (nullptr == State || usbStream >= USB_MAX_QUEUES) {
        return -1;
    }

    if (size == 0) return 0;
    if (Data == nullptr) {
        return -1;
    }

    // If the controller is not yet initialized
    if (State->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        // No data can be sent until the controller is initialized
        return -1;
    }

    endpoint = State->streams[usbStream].TxEP;
    // If no Write side to stream (or if not yet open)
    if (endpoint == USB_NULL_ENDPOINT || State->Queues[endpoint] == nullptr) {
        return -1;
    }
    else {
        DISABLE_INTERRUPTS_SCOPED(irq);

        const char*   ptr = Data;
        uint32_t        count = size;
        bool          Done = false;
        uint32_t        WaitLoopCnt = 0;

        // This loop packetizes the data and sends it out.  All packets sent have
        // the maximum size for the given endpoint except for the last packet which
        // will always have less than the maximum size - even if the packet length
        // must be zero for this to occur.   This is done to comply with standard
        // USB bulk-mode transfers.
        while (!Done) {

            USB_PACKET64* Packet64 = nullptr;
            std::vector<USB_PACKET64>::iterator  packet;

            if ((int32_t)(State->Queues[endpoint]->size()) < ((int32_t)State->Queues[endpoint]->max_size())) {
                USB_PACKET64 pkg;

                State->Queues[endpoint]->push_back(pkg);
                packet = State->Queues[endpoint]->end();

                --packet;
                Packet64 = &(*packet);

            }

            if (Packet64) {
                uint32_t max_move;

                if (count > State->MaxPacketSize[endpoint])
                    max_move = State->MaxPacketSize[endpoint];
                else
                    max_move = count;

                if (max_move) {
                    memcpy(Packet64->Buffer, ptr, max_move);
                }

                // we are done when we send a non-full length packet
                if (max_move < State->MaxPacketSize[endpoint]) {
                    Done = true;
                }

                Packet64->Size = max_move;
                count -= max_move;
                ptr += max_move;

                totWrite += max_move;

                WaitLoopCnt = 0;
            }
            else {
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
                    if (count == size) {
                        State->Queues[endpoint]->clear();
                    }

                    return totWrite;
                }

                if (irq.WasDisabled()) // @todo - this really needs more checks to be totally valid
                {
                    return totWrite;
                }

                if (State->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
                    return totWrite;
                }

                LPC17_UsbClient_StartOutput(State, endpoint);

                irq.Release();
                //                lcd_printf("Looping in write\r\n");

                LPC17_Time_Delay(nullptr, 50);

                irq.Acquire();
            }
        }

        // here we have a post-condition that IRQs are disabled for all paths through conditional block above

        if (State->DeviceState == USB_DEVICE_STATE_CONFIGURED) {
            LPC17_UsbClient_StartOutput(State, endpoint);
        }

        return totWrite;
    }
}

int UsbClient_Driver::Read(int controller, int usbStream, char* Data, size_t size) {
    int endpoint;
    USB_CONTROLLER_STATE * State = &UsbControllerState[controller];

    if (nullptr == State || usbStream >= USB_MAX_QUEUES) {
        return 0;
    }

    /* not configured, no data can go in or out */
    if (State->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        return 0;
    }

    endpoint = State->streams[usbStream].RxEP;
    // If no Read side to stream (or if not yet open)
    if (endpoint == USB_NULL_ENDPOINT || State->Queues[endpoint] == nullptr) {
        return 0;
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        USB_PACKET64* Packet64 = nullptr;
        uint8_t*        ptr = (uint8_t*)Data;
        uint32_t        count = 0;
        uint32_t        remain = size;

        while (count < size) {
            uint32_t max_move;

            int32_t queu_size = (int32_t)(State->Queues[endpoint]->size());
            if (queu_size > 0 && Packet64 == nullptr) {
                std::vector<USB_PACKET64>::iterator  packet = State->Queues[endpoint]->begin();
                Packet64 = &(*packet);
            }

            if (!Packet64) {
                UsbClient_Driver::ClearEvent(controller, 1 << endpoint);
                break;
            }

            max_move = Packet64->Size - State->CurrentPacketOffset[endpoint];
            if (remain < max_move) max_move = remain;

            memcpy(ptr, &Packet64->Buffer[State->CurrentPacketOffset[endpoint]], max_move);

            State->CurrentPacketOffset[endpoint] += max_move;
            ptr += max_move;
            count += max_move;
            remain -= max_move;

            /* if we're done with this packet, move onto the next */
            if (State->CurrentPacketOffset[endpoint] == Packet64->Size) {
                State->CurrentPacketOffset[endpoint] = 0;
                Packet64 = nullptr;

                State->Queues[endpoint]->erase(State->Queues[endpoint]->begin());

                LPC17_UsbClient_RxEnable(State, endpoint);
            }
        }

        return count;
    }
}

bool UsbClient_Driver::Flush(int controller, int usbStream) {
    int endpoint;
    int retries = USB_FLUSH_RETRY_COUNT;
    int queueCnt;
    USB_CONTROLLER_STATE * State = &UsbControllerState[controller];

    if (nullptr == State || usbStream >= USB_MAX_QUEUES) {
        return false;
    }

    /* not configured, no data can go in or out */
    if (State->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        return true;
    }

    endpoint = State->streams[usbStream].TxEP;
    // If no Write side to stream (or if not yet open)
    if (endpoint == USB_NULL_ENDPOINT || State->Queues[endpoint] == nullptr) {
        return false;
    }

    queueCnt = (int32_t)State->Queues[endpoint]->size();

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while ((int32_t)State->Queues[endpoint]->size() > 0 && retries > 0) {
        LPC17_UsbClient_StartOutput(State, endpoint);

        int cnt = (int32_t)State->Queues[endpoint]->size();

        if (queueCnt == cnt)
            LPC17_Time_Delay(nullptr, 100); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == cnt) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = cnt;
    }

    if (retries <= 0)
        State->Queues[endpoint]->clear();

    return true;
}

uint32_t UsbClient_Driver::SetEvent(int controller, uint32_t Event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_CONTROLLER_STATE *State = &UsbControllerState[controller];

    if (State == nullptr)
        return 0;

    uint32_t OldEvent = State->Event;

    State->Event |= Event;

    if (OldEvent != State->Event) {
        UsbClient_Driver::DataReceivedHandler(nullptr);
    }

    //printf("SetEv %d\r\n",State->Event);
    return OldEvent;
}

uint32_t UsbClient_Driver::ClearEvent(int controller, uint32_t Event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_CONTROLLER_STATE *State = &UsbControllerState[controller];

    if (State == nullptr)
        return 0;

    uint32_t OldEvent = State->Event;

    State->Event &= ~Event;

    return OldEvent;
}

void USB_ClearQueues(USB_CONTROLLER_STATE *State, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (ClrRxQueue) {
        for (int endpoint = 0; endpoint < USB_MAX_QUEUES; endpoint++) {
            if (State->Queues[endpoint] == nullptr || State->IsTxQueue[endpoint])
                continue;
            State->Queues[endpoint]->clear();

            /* since this queue is now reset, we have room available for newly arrived packets */
            LPC17_UsbClient_RxEnable(State, endpoint);
        }
    }

    if (ClrTxQueue) {
        for (int endpoint = 0; endpoint < USB_MAX_QUEUES; endpoint++) {
            if (State->Queues[endpoint] && State->IsTxQueue[endpoint])
                State->Queues[endpoint]->clear();
        }
    }
}

void LPC17_UsbClient_StateCallback(USB_CONTROLLER_STATE* State) {
    if (State->CurrentState != State->DeviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
//Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
// The RxQueue is clear when it is configured.

        if (USB_DEVICE_STATE_CONFIGURED == State->CurrentState) {
            USB_ClearQueues(State, true, true);
        }

        State->CurrentState = State->DeviceState;

        switch (State->DeviceState) {
            case USB_DEVICE_STATE_DETACHED:
                State->ResidualCount = 0;
                State->DataCallback = nullptr;
                //            hal_printf("USB_DEVICE_STATE_DETACHED\r\n");
                break;

            case USB_DEVICE_STATE_ATTACHED:
                //            hal_printf("USB_DEVICE_STATE_ATTACHED\r\n");
                break;

            case USB_DEVICE_STATE_POWERED:
                //            hal_printf("USB_DEVICE_STATE_POWERED\r\n");
                break;

            case USB_DEVICE_STATE_DEFAULT:
                //            hal_printf("USB_DEVICE_STATE_DEFAULT\r\n");
                break;

            case USB_DEVICE_STATE_ADDRESS:
                //            hal_printf("USB_DEVICE_STATE_ADDRESS\r\n");
                break;

            case USB_DEVICE_STATE_CONFIGURED:
                //            hal_printf("USB_DEVICE_STATE_CONFIGURED\r\n");

                            /* whenever we enter the configured state, re-initialize all of the RxQueues */
                            /* Txqueue has stored some data to be transmitted */
                USB_ClearQueues(State, true, false);
                break;

            case USB_DEVICE_STATE_SUSPENDED:
                //            hal_printf("USB_DEVICE_STATE_SUSPENDED\r\n");
                break;

            default:
                USB_DEBUG_ASSERT(0);
                break;
        }
    }
}

void USB_DataCallback(USB_CONTROLLER_STATE* State) {
    uint32_t length = __min(State->PacketSize, State->ResidualCount);

    memcpy(State->Data, State->ResidualData, length);

    State->DataSize = length;
    State->ResidualData += length;
    State->ResidualCount -= length;

    if (length == State->PacketSize) {
        State->Expected -= length;
    }
    else {
        State->Expected = 0;
    }

    if (State->Expected) {
        State->DataCallback = USB_DataCallback;
    }
    else {
        State->DataCallback = nullptr;
    }
}

uint8_t USB_HandleGetStatus(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup) {
    uint16_t* status;
    uint16_t  zero = 0;

    /* validate setup packet */
    if (Setup->wValue != 0 || Setup->wLength != 2) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (State->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
        case USB_SETUP_RECIPIENT_DEVICE:
            status = &State->DeviceStatus;
            break;

        case USB_SETUP_RECIPIENT_INTERFACE:
            if (State->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
                return USB_STATE_STALL;
            }

            status = &zero;
            break;

        case USB_SETUP_RECIPIENT_ENDPOINT:
            if (State->DeviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0) {
                return USB_STATE_STALL;
            }

            /* bit 0x80 designates direction, which we don't utilize in this calculation */
            Setup->wIndex &= 0x7F;

            if (Setup->wIndex >= State->EndpointCount) {
                return USB_STATE_STALL;
            }

            status = &State->EndpointStatus[Setup->wIndex];
            break;

        default:
            return USB_STATE_STALL;
    }

    /* send requested status to host */
    State->ResidualData = (uint8_t*)status;
    State->ResidualCount = 2;
    State->DataCallback = USB_DataCallback;

    return USB_STATE_DATA;
}

uint8_t USB_HandleClearFeature(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (State->DeviceState != USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
        case USB_SETUP_RECIPIENT_DEVICE:
            // only support Remote wakeup
            if (Setup->wValue != USB_FEATURE_DEVICE_REMOTE_WAKEUP)
                return USB_STATE_STALL;

            // Locate the configuration descriptor
            Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)USB_FindRecord(State, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);

            if (Config && (Config->bmAttributes & USB_ATTRIBUTE_REMOTE_WAKEUP)) {
                State->DeviceStatus &= ~USB_STATUS_DEVICE_REMOTE_WAKEUP;
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
            if (State->DeviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0)
                return USB_STATE_STALL;

            /* bit 0x80 designates direction, which we dont utilize in this calculation */
            Setup->wIndex &= 0x7F;

            if (Setup->wIndex == 0 || Setup->wIndex >= State->EndpointCount)
                return USB_STATE_STALL;

            if (Setup->wValue != USB_FEATURE_ENDPOINT_HALT)
                return USB_STATE_STALL;

            /* clear the halt feature */
            State->EndpointStatus[Setup->wIndex] &= ~USB_STATUS_ENDPOINT_HALT;
            State->EndpointStatusChange = Setup->wIndex;
            retState = USB_STATE_STATUS;
            break;

        default:
            return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    State->ResidualCount = 0;
    State->DataCallback = USB_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t USB_HandleSetFeature(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (State->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->bmRequestType)) {
        case USB_SETUP_RECIPIENT_DEVICE:
            // only support Remote wakeup
            if (Setup->wValue != USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
                return USB_STATE_STALL;
            }

            Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)USB_FindRecord(State, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
            if (Config == nullptr)        // If the configuration record could not be found
                return USB_STATE_STALL; // Something pretty serious is wrong

            if (Config->bmAttributes & USB_ATTRIBUTE_REMOTE_WAKEUP) {
                State->DeviceStatus |= USB_STATUS_DEVICE_REMOTE_WAKEUP;
            }

            retState = USB_STATE_REMOTE_WAKEUP;
            break;

        case USB_SETUP_RECIPIENT_INTERFACE:
            /* there are no interface features to set */
            return USB_STATE_STALL;

        case USB_SETUP_RECIPIENT_ENDPOINT:
            if (State->DeviceState == USB_DEVICE_STATE_ADDRESS && Setup->wIndex != 0) {
                return USB_STATE_STALL;
            }

            /* bit 0x80 designates direction, which we don't utilize in this calculation */
            Setup->wIndex &= 0x7F;

            if (Setup->wIndex == 0 || Setup->wIndex >= State->EndpointCount) {
                return USB_STATE_STALL;
            }

            if (Setup->wValue != USB_FEATURE_ENDPOINT_HALT) {
                return USB_STATE_STALL;
            }

            /* set the halt feature */
            State->EndpointStatus[Setup->wIndex] |= USB_STATUS_ENDPOINT_HALT;
            State->EndpointStatusChange = Setup->wIndex;
            retState = USB_STATE_STATUS;
            break;

        default:
            return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    State->ResidualCount = 0;
    State->DataCallback = USB_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t USB_HandleSetAddress(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup) {
    /* validate setup packet */
    if (Setup->wValue > 127 || Setup->wIndex != 0 || Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (State->DeviceState >= USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    /* set address */
    State->Address = Setup->wValue;

    /* catch state changes */
    if (State->Address == 0) {
        State->DeviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else {
        State->DeviceState = USB_DEVICE_STATE_ADDRESS;
    }

    LPC17_UsbClient_StateCallback(State);

    /* send zero-length packet to tell host we're done */
    State->ResidualCount = 0;
    State->DataCallback = USB_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t USB_HandleConfigurationRequests(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup) {
    const TinyCLR_UsbClient_DescriptorHeader * header;
    uint8_t       type;
    uint8_t       DescriptorIndex;

    /* this request is valid regardless of device state */
    type = ((Setup->wValue & 0xFF00) >> 8);
    DescriptorIndex = (Setup->wValue & 0x00FF);
    State->Expected = Setup->wLength;

    if (State->Expected == 0) {
        // just return an empty Status packet
        State->ResidualCount = 0;
        State->DataCallback = USB_DataCallback;
        return USB_STATE_DATA;
    }

    //
    // The very first GET_DESCRIPTOR command out of reset should always return at most PacketSize bytes.
    // After that, you can return as many as the host has asked.
    //
    if (State->DeviceState <= USB_DEVICE_STATE_DEFAULT) {
        if (State->FirstGetDescriptor) {
            State->FirstGetDescriptor = false;

            State->Expected = __min(State->Expected, State->PacketSize);
        }
    }

    State->ResidualData = nullptr;
    State->ResidualCount = 0;

    if (Setup->bRequest == USB_GET_DESCRIPTOR) {
        switch (type) {
            case USB_DEVICE_DESCRIPTOR_TYPE:
                header = USB_FindRecord(State, USB_DEVICE_DESCRIPTOR_MARKER, Setup);
                if (header) {
                    const TinyCLR_UsbClient_DeviceDescriptor * device = (TinyCLR_UsbClient_DeviceDescriptor *)header;
                    State->ResidualData = (uint8_t *)&device->bLength;      // Start of the device descriptor
                    State->ResidualCount = __min(State->Expected, device->bLength);
                }
                break;

            case USB_CONFIGURATION_DESCRIPTOR_TYPE:
                header = USB_FindRecord(State, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
                if (header) {
                    const TinyCLR_UsbClient_ConfigurationDescriptor * Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)header;
                    State->ResidualData = (uint8_t *)&Config->bLength;
                    State->ResidualCount = __min(State->Expected, Config->wTotalLength);
                }
                break;

            case USB_STRING_DESCRIPTOR_TYPE:
                if (DescriptorIndex == 0)        // If host is requesting the language list
                {
                    State->ResidualData = USB_LanguageDescriptor;
                    State->ResidualCount = __min(State->Expected, USB_LANGUAGE_DESCRIPTOR_SIZE);
                }
                else if (nullptr != (header = USB_FindRecord(State, USB_STRING_DESCRIPTOR_MARKER, Setup))) {
                    const TinyCLR_UsbClient_StringDescriptorHeader * string = (TinyCLR_UsbClient_StringDescriptorHeader *)header;
                    State->ResidualData = (uint8_t *)&string->bLength;
                    State->ResidualCount = __min(State->Expected, string->bLength);
                }
                break;

            default:
                break;
        }
    }

    // If the request was not recognized, the generic types should be searched
    if (State->ResidualData == nullptr) {
        if (nullptr != (header = USB_FindRecord(State, USB_GENERIC_DESCRIPTOR_MARKER, Setup))) {
            State->ResidualData = (uint8_t *)header;
            State->ResidualData += sizeof(TinyCLR_UsbClient_GenericDescriptorHeader);       // Data is located right after the header
            State->ResidualCount = __min(State->Expected, header->size - sizeof(TinyCLR_UsbClient_GenericDescriptorHeader));
        }
        else
            return USB_STATE_STALL;
    }

    State->DataCallback = USB_DataCallback;

    return USB_STATE_DATA;
}

uint8_t USB_HandleGetConfiguration(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup) {
    /* validate setup packet */
    if (Setup->wValue != 0 || Setup->wIndex != 0 || Setup->wLength != 1) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (State->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    State->ResidualData = &State->ConfigurationNum;
    State->ResidualCount = 1;
    State->Expected = 1;
    State->DataCallback = USB_DataCallback;

    return USB_STATE_DATA;
}

uint8_t LPC17_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup, bool DataPhase) {
    /* validate setup packet */
    if (Setup->wIndex != 0 || Setup->wLength != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (State->DeviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    /* we only support one configuration */
    if (Setup->wValue > 1) {
        return USB_STATE_STALL;
    }

    State->ConfigurationNum = Setup->wValue;

    /* catch state changes */
    if (State->ConfigurationNum == 0) {
        State->DeviceState = USB_DEVICE_STATE_ADDRESS;
    }
    else {
        State->DeviceState = USB_DEVICE_STATE_CONFIGURED;
    }

    LPC17_UsbClient_StateCallback(State);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        State->ResidualCount = 0;
        State->DataCallback = USB_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB Configuration records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const TinyCLR_UsbClient_DescriptorHeader * USB_FindRecord(USB_CONTROLLER_STATE* State, uint8_t marker, USB_SETUP_PACKET * setup) {
    bool Done = false;


    const TinyCLR_UsbClient_DescriptorHeader * header = (const TinyCLR_UsbClient_DescriptorHeader *)State->Configuration;
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

uint8_t LPC17_UsbClient_ControlCallback(USB_CONTROLLER_STATE* State) {
    USB_SETUP_PACKET* Setup;

    if (State->DataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (USB_SETUP_PACKET*)State->Data;

    switch (Setup->bRequest) {
        case USB_GET_STATUS:
            return USB_HandleGetStatus(State, Setup);
        case USB_CLEAR_FEATURE:
            return USB_HandleClearFeature(State, Setup);
        case USB_SET_FEATURE:
            return USB_HandleSetFeature(State, Setup);
        case USB_SET_ADDRESS:
            return USB_HandleSetAddress(State, Setup);
        case USB_GET_CONFIGURATION:
            return USB_HandleGetConfiguration(State, Setup);
        case USB_SET_CONFIGURATION:
            return LPC17_UsbClient_HandleSetConfiguration(State, Setup, true);
        default:
            return USB_HandleConfigurationRequests(State, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* LPC17_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* State, int endpoint, bool& DisableRx) {
    ASSERT_IRQ_MUST_BE_OFF();
    USB_DEBUG_ASSERT(State && (endpoint < USB_MAX_QUEUES));
    USB_DEBUG_ASSERT(State->Queues[endpoint] && !State->IsTxQueue[endpoint]);

    std::vector<USB_PACKET64>::iterator  packet;

    USB_PACKET64 packet64;

    int32_t max_size = (int32_t)State->Queues[endpoint]->max_size();
    int32_t size = (int32_t)State->Queues[endpoint]->size();

    if (size < max_size) {

        State->Queues[endpoint]->push_back(packet64);

        packet = State->Queues[endpoint]->end();
        --packet;

        DisableRx = ((int32_t)(State->Queues[endpoint]->size()) >= max_size);

        UsbClient_Driver::SetEvent(State->ControllerNum, 1 << endpoint);

        return &(*packet);
    }

    return nullptr;
}

USB_PACKET64* LPC17_UsbClient_TxDequeue(USB_CONTROLLER_STATE* State, int endpoint, bool Done) {
    ASSERT_IRQ_MUST_BE_OFF();
    USB_DEBUG_ASSERT(State && (endpoint < USB_MAX_QUEUES));
    USB_DEBUG_ASSERT(State->Queues[endpoint] && State->IsTxQueue[endpoint]);

    std::vector<USB_PACKET64>::iterator  packet;

    if ((int32_t)(State->Queues[endpoint]->size()) > 0) {
        packet = State->Queues[endpoint]->begin();

        return &(*packet);
    }
    else {
        return nullptr;
    }
}

// For Api
TinyCLR_Result LPC17_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    uint8_t *osStringDescriptor = (uint8_t*)&LPC17_UsbClient_OsStringDescriptor;
    uint8_t *xCompatibleOsId = (uint8_t*)&LPC17_UsbClient_XCompatibleOsId;
    uint8_t *xPropertiesOsWinUsb = (uint8_t*)&LPC17_UsbClient_XPropertiesOsWinUsb;

    UsbClient_Driver::OsExtendedPropertyHandler(self, osStringDescriptor, xCompatibleOsId, xPropertiesOsWinUsb);

    UsbClient_Driver::Initialize(controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Release(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;
    UsbClient_Driver::Uninitialize(controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t& stream, TinyCLR_UsbClient_StreamMode mode) {
    int32_t controller = self->Index;
    int32_t availableStream;

    if (UsbClient_Driver::OpenStream(controller, availableStream, mode) == true) {
        stream = availableStream;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result LPC17_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t stream) {
    int32_t controller = self->Index;

    UsbClient_Driver::CloseStream(controller, stream);
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t stream, const uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    length = UsbClient_Driver::Write(controller, stream, (const char*)data, length);
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t stream, uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    length = UsbClient_Driver::Read(controller, stream, (char*)data, length);
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t stream) {
    int32_t controller = self->Index;

    UsbClient_Driver::Flush(controller, stream);
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    int32_t controller = self->Index;

    UsbClient_Driver::DataReceivedHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_UsbClient_SetOsExtendedProperty(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler) {
    int32_t controller = self->Index;

    UsbClient_Driver::OsExtendedPropertyHandler = handler;

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
            memcpy(&stringManufacturerDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_CONST_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
            break;

        case TinyCLR_UsbClient_StringDescriptorType::ProductName:
            memcpy(&stringProductNameDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_CONST_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
            break;

        case TinyCLR_UsbClient_StringDescriptorType::DisplayName:
            memcpy(&stringDisplayNameDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_CONST_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
            break;

        case TinyCLR_UsbClient_StringDescriptorType::FriendlyName:
            memcpy(&stringFriendlyNameDescriptorHeader.stringDescriptor, value, sizeof(wchar_t) * SIZEOF_CONST_ARRAY(stringManufacturerDescriptorHeader.stringDescriptor));
            break;
    }

    return TinyCLR_Result::Success;
}

static TinyCLR_UsbClient_Provider usbClientProvider;
static TinyCLR_Api_Info usbClientApi;

void LPC17_UsbClient_Reset() {
    for (auto controller = 0; controller < usbClientApi.Count; controller++) {
        // Close all stream if any opened
        for (auto stream = 0; stream < USB_MAX_QUEUES; stream++) {
            UsbClient_Driver::CloseStream(controller, stream);
        }

        // Close controller
        UsbClient_Driver::Uninitialize(controller);
    }
}

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
    usbClientProvider.SetDataReceivedHandler = &LPC17_UsbClient_SetDataReceivedHandler;
    usbClientProvider.SetOsExtendedPropertyHandler = &LPC17_UsbClient_SetOsExtendedProperty;
    usbClientProvider.SetDeviceDescriptor = &LPC17_UsbClient_SetDeviceDescriptor;
    usbClientProvider.SetConfigDescriptor = &LPC17_UsbClient_SetConfigDescriptor;
    usbClientProvider.SetStringDescriptor = &LPC17_UsbClient_SetStringDescriptor;

    usbClientApi.Author = "GHI Electronics, LLC";
    usbClientApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.UsbClientProvider";
    usbClientApi.Type = TinyCLR_Api_Type::UsbClientProvider;
    usbClientApi.Version = 0;
    usbClientApi.Count = 1;
    usbClientApi.Implementation = &usbClientProvider;

    return &usbClientApi;
}

// LPC178 USBD HAL driver

// 1PC178 usb register (usbreg.h)

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

/* Clear Buffer Register Definitions */
#define CLR_BUF_PO          0x01


/* DMA Interrupt Bit Definitions */
#define EOT_INT             0x01
#define NDD_REQ_INT         0x02
#define SYS_ERR_INT         0x04

// LPC178 usbdef.h

#define USB_BASE 0x2008C000

#define OTGIntSt (*(volatile uint32_t *)0x2008C100)
#define OTGIntSt_OFFSET 0x100
#define OTGIntSt_TMR_MASK 0x1
#define OTGIntSt_TMR 0x1
#define OTGIntSt_TMR_BIT 0
#define OTGIntSt_REMOVE_PU_MASK 0x2
#define OTGIntSt_REMOVE_PU 0x2
#define OTGIntSt_REMOVE_PU_BIT 1
#define OTGIntSt_HNP_FAILURE_MASK 0x4
#define OTGIntSt_HNP_FAILURE 0x4
#define OTGIntSt_HNP_FAILURE_BIT 2
#define OTGIntSt_HNP_SUCCESS_MASK 0x8
#define OTGIntSt_HNP_SUCCESS 0x8
#define OTGIntSt_HNP_SUCCESS_BIT 3

#define OTGIntEn (*(volatile uint32_t *)0x2008C104)
#define OTGIntEn_OFFSET 0x104
#define OTGIntEn_TMR_MASK 0x1
#define OTGIntEn_TMR 0x1
#define OTGIntEn_TMR_BIT 0
#define OTGIntEn_REMOVE_PU_MASK 0x2
#define OTGIntEn_REMOVE_PU 0x2
#define OTGIntEn_REMOVE_PU_BIT 1
#define OTGIntEn_HNP_FAILURE_MASK 0x4
#define OTGIntEn_HNP_FAILURE 0x4
#define OTGIntEn_HNP_FAILURE_BIT 2
#define OTGIntEn_HNP_SUCCESS_MASK 0x8
#define OTGIntEn_HNP_SUCCESS 0x8
#define OTGIntEn_HNP_SUCCESS_BIT 3

#define OTGIntSet (*(volatile uint32_t *)0x2008C108)
#define OTGIntSet_OFFSET 0x108
#define OTGIntSet_TMR_MASK 0x1
#define OTGIntSet_TMR 0x1
#define OTGIntSet_TMR_BIT 0
#define OTGIntSet_REMOVE_PU_MASK 0x2
#define OTGIntSet_REMOVE_PU 0x2
#define OTGIntSet_REMOVE_PU_BIT 1
#define OTGIntSet_HNP_FAILURE_MASK 0x4
#define OTGIntSet_HNP_FAILURE 0x4
#define OTGIntSet_HNP_FAILURE_BIT 2
#define OTGIntSet_HNP_SUCCESS_MASK 0x8
#define OTGIntSet_HNP_SUCCESS 0x8
#define OTGIntSet_HNP_SUCCESS_BIT 3

#define OTGIntClr (*(volatile uint32_t *)0x2008C10C)
#define OTGIntClr_OFFSET 0x10C
#define OTGIntClr_TMR_MASK 0x1
#define OTGIntClr_TMR 0x1
#define OTGIntClr_TMR_BIT 0
#define OTGIntClr_REMOVE_PU_MASK 0x2
#define OTGIntClr_REMOVE_PU 0x2
#define OTGIntClr_REMOVE_PU_BIT 1
#define OTGIntClr_HNP_FAILURE_MASK 0x4
#define OTGIntClr_HNP_FAILURE 0x4
#define OTGIntClr_HNP_FAILURE_BIT 2
#define OTGIntClr_HNP_SUCCESS_MASK 0x8
#define OTGIntClr_HNP_SUCCESS 0x8
#define OTGIntClr_HNP_SUCCESS_BIT 3

#define OTGStCtrl (*(volatile uint32_t *)0x2008C110)
#define OTGStCtrl_OFFSET 0x110
#define OTGStCtrl_PORT_FUNC_MASK 0x3
#define OTGStCtrl_PORT_FUNC_BIT 0
#define OTGStCtrl_TMR_SCALE_MASK 0xC
#define OTGStCtrl_TMR_SCALE_BIT 2
#define OTGStCtrl_TMR_MODE_MASK 0x10
#define OTGStCtrl_TMR_MODE 0x10
#define OTGStCtrl_TMR_MODE_BIT 4
#define OTGStCtrl_TMR_EN_MASK 0x20
#define OTGStCtrl_TMR_EN 0x20
#define OTGStCtrl_TMR_EN_BIT 5
#define OTGStCtrl_TMR_RST_MASK 0x40
#define OTGStCtrl_TMR_RST 0x40
#define OTGStCtrl_TMR_RST_BIT 6
#define OTGStCtrl_B_HNP_TRACK_MASK 0x100
#define OTGStCtrl_B_HNP_TRACK 0x100
#define OTGStCtrl_B_HNP_TRACK_BIT 8
#define OTGStCtrl_A_HNP_TRACK_MASK 0x200
#define OTGStCtrl_A_HNP_TRACK 0x200
#define OTGStCtrl_A_HNP_TRACK_BIT 9
#define OTGStCtrl_PU_REMOVED_MASK 0x400
#define OTGStCtrl_PU_REMOVED 0x400
#define OTGStCtrl_PU_REMOVED_BIT 10
#define OTGStCtrl_TMR_CNT_MASK 0xFFFF0000
#define OTGStCtrl_TMR_CNT_BIT 16

#define OTGTmr (*(volatile uint32_t *)0x2008C114)
#define OTGTmr_OFFSET 0x114
#define OTGTmr_TIMEOUT_CNT_MASK 0xFFFF
#define OTGTmr_TIMEOUT_CNT_BIT 0

#define USBPortSel (*(volatile uint32_t *)0x2008C110)
#define USBPortSel_OFFSET 0x110

#define USBDevIntSt (*(volatile uint32_t *)0x2008C200)
#define USBDevIntSt_OFFSET 0x200
#define USBDevIntSt_FRAME_MASK 0x1
#define USBDevIntSt_FRAME 0x1
#define USBDevIntSt_FRAME_BIT 0
#define USBDevIntSt_EP_FAST_MASK 0x2
#define USBDevIntSt_EP_FAST 0x2
#define USBDevIntSt_EP_FAST_BIT 1
#define USBDevIntSt_EP_SLOW_MASK 0x4
#define USBDevIntSt_EP_SLOW 0x4
#define USBDevIntSt_EP_SLOW_BIT 2
#define USBDevIntSt_DEV_STAT_MASK 0x8
#define USBDevIntSt_DEV_STAT 0x8
#define USBDevIntSt_DEV_STAT_BIT 3
#define USBDevIntSt_CCEMTY_MASK 0x10
#define USBDevIntSt_CCEMTY 0x10
#define USBDevIntSt_CCEMTY_BIT 4
#define USBDevIntSt_CDFULL_MASK 0x20
#define USBDevIntSt_CDFULL 0x20
#define USBDevIntSt_CDFULL_BIT 5
#define USBDevIntSt_RxENDPKT_MASK 0x40
#define USBDevIntSt_RxENDPKT 0x40
#define USBDevIntSt_RxENDPKT_BIT 6
#define USBDevIntSt_TxENDPKT_MASK 0x80
#define USBDevIntSt_TxENDPKT 0x80
#define USBDevIntSt_TxENDPKT_BIT 7
#define USBDevIntSt_EP_RLZED_MASK 0x100
#define USBDevIntSt_EP_RLZED 0x100
#define USBDevIntSt_EP_RLZED_BIT 8
#define USBDevIntSt_ERR_INT_MASK 0x200
#define USBDevIntSt_ERR_INT 0x200
#define USBDevIntSt_ERR_INT_BIT 9

#define USBDevIntEn (*(volatile uint32_t *)0x2008C204)
#define USBDevIntEn_OFFSET 0x204
#define USBDevIntEn_FRAME_MASK 0x1
#define USBDevIntEn_FRAME 0x1
#define USBDevIntEn_FRAME_BIT 0
#define USBDevIntEn_EP_FAST_MASK 0x2
#define USBDevIntEn_EP_FAST 0x2
#define USBDevIntEn_EP_FAST_BIT 1
#define USBDevIntEn_EP_SLOW_MASK 0x4
#define USBDevIntEn_EP_SLOW 0x4
#define USBDevIntEn_EP_SLOW_BIT 2
#define USBDevIntEn_DEV_STAT_MASK 0x8
#define USBDevIntEn_DEV_STAT 0x8
#define USBDevIntEn_DEV_STAT_BIT 3
#define USBDevIntEn_CCEMTY_MASK 0x10
#define USBDevIntEn_CCEMTY 0x10
#define USBDevIntEn_CCEMTY_BIT 4
#define USBDevIntEn_CDFULL_MASK 0x20
#define USBDevIntEn_CDFULL 0x20
#define USBDevIntEn_CDFULL_BIT 5
#define USBDevIntEn_RxENDPKT_MASK 0x40
#define USBDevIntEn_RxENDPKT 0x40
#define USBDevIntEn_RxENDPKT_BIT 6
#define USBDevIntEn_TxENDPKT_MASK 0x80
#define USBDevIntEn_TxENDPKT 0x80
#define USBDevIntEn_TxENDPKT_BIT 7
#define USBDevIntEn_EP_RLZED_MASK 0x100
#define USBDevIntEn_EP_RLZED 0x100
#define USBDevIntEn_EP_RLZED_BIT 8
#define USBDevIntEn_ERR_INT_MASK 0x200
#define USBDevIntEn_ERR_INT 0x200
#define USBDevIntEn_ERR_INT_BIT 9

#define USBDevIntClr (*(volatile uint32_t *)0x2008C208)
#define USBDevIntClr_OFFSET 0x208
#define USBDevIntClr_FRAME_MASK 0x1
#define USBDevIntClr_FRAME 0x1
#define USBDevIntClr_FRAME_BIT 0
#define USBDevIntClr_EP_FAST_MASK 0x2
#define USBDevIntClr_EP_FAST 0x2
#define USBDevIntClr_EP_FAST_BIT 1
#define USBDevIntClr_EP_SLOW_MASK 0x4
#define USBDevIntClr_EP_SLOW 0x4
#define USBDevIntClr_EP_SLOW_BIT 2
#define USBDevIntClr_DEV_STAT_MASK 0x8
#define USBDevIntClr_DEV_STAT 0x8
#define USBDevIntClr_DEV_STAT_BIT 3
#define USBDevIntClr_CCEMTY_MASK 0x10
#define USBDevIntClr_CCEMTY 0x10
#define USBDevIntClr_CCEMTY_BIT 4
#define USBDevIntClr_CDFULL_MASK 0x20
#define USBDevIntClr_CDFULL 0x20
#define USBDevIntClr_CDFULL_BIT 5
#define USBDevIntClr_RxENDPKT_MASK 0x40
#define USBDevIntClr_RxENDPKT 0x40
#define USBDevIntClr_RxENDPKT_BIT 6
#define USBDevIntClr_TxENDPKT_MASK 0x80
#define USBDevIntClr_TxENDPKT 0x80
#define USBDevIntClr_TxENDPKT_BIT 7
#define USBDevIntClr_EP_RLZED_MASK 0x100
#define USBDevIntClr_EP_RLZED 0x100
#define USBDevIntClr_EP_RLZED_BIT 8
#define USBDevIntClr_EPR_INT_MASK 0x200
#define USBDevIntClr_EPR_INT 0x200
#define USBDevIntClr_EPR_INT_BIT 9

#define USBDevIntSet (*(volatile uint32_t *)0x2008C20C)
#define USBDevIntSet_OFFSET 0x20C
#define USBDevIntSet_FRAME_MASK 0x1
#define USBDevIntSet_FRAME 0x1
#define USBDevIntSet_FRAME_BIT 0
#define USBDevIntSet_EP_FAST_MASK 0x2
#define USBDevIntSet_EP_FAST 0x2
#define USBDevIntSet_EP_FAST_BIT 1
#define USBDevIntSet_EP_SLOW_MASK 0x4
#define USBDevIntSet_EP_SLOW 0x4
#define USBDevIntSet_EP_SLOW_BIT 2
#define USBDevIntSet_DEV_STAT_MASK 0x8
#define USBDevIntSet_DEV_STAT 0x8
#define USBDevIntSet_DEV_STAT_BIT 3
#define USBDevIntSet_CCEMTY_MASK 0x10
#define USBDevIntSet_CCEMTY 0x10
#define USBDevIntSet_CCEMTY_BIT 4
#define USBDevIntSet_CDFULL_MASK 0x20
#define USBDevIntSet_CDFULL 0x20
#define USBDevIntSet_CDFULL_BIT 5
#define USBDevIntSet_RxENDPKT_MASK 0x40
#define USBDevIntSet_RxENDPKT 0x40
#define USBDevIntSet_RxENDPKT_BIT 6
#define USBDevIntSet_TxENDPKT_MASK 0x80
#define USBDevIntSet_TxENDPKT 0x80
#define USBDevIntSet_TxENDPKT_BIT 7
#define USBDevIntSet_EP_RLZED_MASK 0x100
#define USBDevIntSet_EP_RLZED 0x100
#define USBDevIntSet_EP_RLZED_BIT 8
#define USBDevIntSet_EPR_INT_MASK 0x200
#define USBDevIntSet_EPR_INT 0x200
#define USBDevIntSet_EPR_INT_BIT 9

#define USBCmdCode (*(volatile uint32_t *)0x2008C210)
#define USBCmdCode_OFFSET 0x210
#define USBCmdCode_CMD_PHASE_MASK 0xFF00
#define USBCmdCode_CMD_PHASE_BIT 8
#define USBCmdCode_CMD_CODE_MASK 0x7F0000
#define USBCmdCode_CMD_CODE_BIT 16

#define USBCmdData (*(volatile uint32_t *)0x2008C214)
#define USBCmdData_OFFSET 0x214
#define USBCmdData_CommandData_MASK 0xFF
#define USBCmdData_CommandData_BIT 0

#define USBRxData (*(volatile uint32_t *)0x2008C218)
#define USBRxData_OFFSET 0x218

#define USBTxData (*(volatile uint32_t *)0x2008C21C)
#define USBTxData_OFFSET 0x21C

#define USBRxPLen (*(volatile uint32_t *)0x2008C220)
#define USBRxPLen_OFFSET 0x220
#define USBRxPLen_PKT_LNGTH_MASK 0x3FF
#define USBRxPLen_PKT_LNGTH_BIT 0
#define USBRxPLen_DV_MASK 0x400
#define USBRxPLen_DV 0x400
#define USBRxPLen_DV_BIT 10
#define USBRxPLen_PKT_RDY_MASK 0x800
#define USBRxPLen_PKT_RDY 0x800
#define USBRxPLen_PKT_RDY_BIT 11

#define USBTxPLen (*(volatile uint32_t *)0x2008C224)
#define USBTxPLen_OFFSET 0x224
#define USBTxPLen_PKT_LNGTH_MASK 0x3FF
#define USBTxPLen_PKT_LNGTH_BIT 0

#define USBCtrl (*(volatile uint32_t *)0x2008C228)
#define USBCtrl_OFFSET 0x228
#define USBCtrl_RD_EN_MASK 0x1
#define USBCtrl_RD_EN 0x1
#define USBCtrl_RD_EN_BIT 0
#define USBCtrl_WR_EN_MASK 0x2
#define USBCtrl_WR_EN 0x2
#define USBCtrl_WR_EN_BIT 1
#define USBCtrl_LOG_ENDPOINT_MASK 0x3C
#define USBCtrl_LOG_ENDPOINT_BIT 2

#define USBDevIntPri (*(volatile uint32_t *)0x2008C22C)
#define USBDevIntPri_OFFSET 0x22C

#define USBEpIntSt (*(volatile uint32_t *)0x2008C230)
#define USBEpIntSt_OFFSET 0x230
#define USBEpIntSt_EP0RX_MASK 0x1
#define USBEpIntSt_EP0RX 0x1
#define USBEpIntSt_EP0RX_BIT 0
#define USBEpIntSt_EP0TX_MASK 0x2
#define USBEpIntSt_EP0TX 0x2
#define USBEpIntSt_EP0TX_BIT 1
#define USBEpIntSt_EP1RX_MASK 0x4
#define USBEpIntSt_EP1RX 0x4
#define USBEpIntSt_EP1RX_BIT 2
#define USBEpIntSt_EP1TX_MASK 0x8
#define USBEpIntSt_EP1TX 0x8
#define USBEpIntSt_EP1TX_BIT 3
#define USBEpIntSt_EP2RX_MASK 0x10
#define USBEpIntSt_EP2RX 0x10
#define USBEpIntSt_EP2RX_BIT 4
#define USBEpIntSt_EP2TX_MASK 0x20
#define USBEpIntSt_EP2TX 0x20
#define USBEpIntSt_EP2TX_BIT 5
#define USBEpIntSt_EP3RX_MASK 0x40
#define USBEpIntSt_EP3RX 0x40
#define USBEpIntSt_EP3RX_BIT 6
#define USBEpIntSt_EP3TX_MASK 0x80
#define USBEpIntSt_EP3TX 0x80
#define USBEpIntSt_EP3TX_BIT 7
#define USBEpIntSt_EP4RX_MASK 0x100
#define USBEpIntSt_EP4RX 0x100
#define USBEpIntSt_EP4RX_BIT 8
#define USBEpIntSt_EP4TX_MASK 0x200
#define USBEpIntSt_EP4TX 0x200
#define USBEpIntSt_EP4TX_BIT 9
#define USBEpIntSt_EP5RX_MASK 0x400
#define USBEpIntSt_EP5RX 0x400
#define USBEpIntSt_EP5RX_BIT 10
#define USBEpIntSt_EP5TX_MASK 0x800
#define USBEpIntSt_EP5TX 0x800
#define USBEpIntSt_EP5TX_BIT 11
#define USBEpIntSt_EP6RX_MASK 0x1000
#define USBEpIntSt_EP6RX 0x1000
#define USBEpIntSt_EP6RX_BIT 12
#define USBEpIntSt_EP6TX_MASK 0x2000
#define USBEpIntSt_EP6TX 0x2000
#define USBEpIntSt_EP6TX_BIT 13
#define USBEpIntSt_EP7RX_MASK 0x4000
#define USBEpIntSt_EP7RX 0x4000
#define USBEpIntSt_EP7RX_BIT 14
#define USBEpIntSt_EP7TX_MASK 0x8000
#define USBEpIntSt_EP7TX 0x8000
#define USBEpIntSt_EP7TX_BIT 15
#define USBEpIntSt_EP8RX_MASK 0x10000
#define USBEpIntSt_EP8RX 0x10000
#define USBEpIntSt_EP8RX_BIT 16
#define USBEpIntSt_EP8TX_MASK 0x20000
#define USBEpIntSt_EP8TX 0x20000
#define USBEpIntSt_EP8TX_BIT 17
#define USBEpIntSt_EP9RX_MASK 0x40000
#define USBEpIntSt_EP9RX 0x40000
#define USBEpIntSt_EP9RX_BIT 18
#define USBEpIntSt_EP9TX_MASK 0x80000
#define USBEpIntSt_EP9TX 0x80000
#define USBEpIntSt_EP9TX_BIT 19
#define USBEpIntSt_EP10RX_MASK 0x100000
#define USBEpIntSt_EP10RX 0x100000
#define USBEpIntSt_EP10RX_BIT 20
#define USBEpIntSt_EP10TX_MASK 0x200000
#define USBEpIntSt_EP10TX 0x200000
#define USBEpIntSt_EP10TX_BIT 21
#define USBEpIntSt_EP11RX_MASK 0x400000
#define USBEpIntSt_EP11RX 0x400000
#define USBEpIntSt_EP11RX_BIT 22
#define USBEpIntSt_EP11TX_MASK 0x800000
#define USBEpIntSt_EP11TX 0x800000
#define USBEpIntSt_EP11TX_BIT 23
#define USBEpIntSt_EP12RX_MASK 0x1000000
#define USBEpIntSt_EP12RX 0x1000000
#define USBEpIntSt_EP12RX_BIT 24
#define USBEpIntSt_EP12TX_MASK 0x2000000
#define USBEpIntSt_EP12TX 0x2000000
#define USBEpIntSt_EP12TX_BIT 25
#define USBEpIntSt_EP13RX_MASK 0x4000000
#define USBEpIntSt_EP13RX 0x4000000
#define USBEpIntSt_EP13RX_BIT 26
#define USBEpIntSt_EP13TX_MASK 0x8000000
#define USBEpIntSt_EP13TX 0x8000000
#define USBEpIntSt_EP13TX_BIT 27
#define USBEpIntSt_EP14RX_MASK 0x10000000
#define USBEpIntSt_EP14RX 0x10000000
#define USBEpIntSt_EP14RX_BIT 28
#define USBEpIntSt_EP14TX_MASK 0x20000000
#define USBEpIntSt_EP14TX 0x20000000
#define USBEpIntSt_EP14TX_BIT 29
#define USBEpIntSt_EP15RX_MASK 0x40000000
#define USBEpIntSt_EP15RX 0x40000000
#define USBEpIntSt_EP15RX_BIT 30
#define USBEpIntSt_EP15TX_MASK 0x80000000
#define USBEpIntSt_EP15TX 0x80000000
#define USBEpIntSt_EP15TX_BIT 31

#define USBEpIntEn (*(volatile uint32_t *)0x2008C234)
#define USBEpIntEn_OFFSET 0x234
#define USBEpIntEn_EP0RX_MASK 0x1
#define USBEpIntEn_EP0RX 0x1
#define USBEpIntEn_EP0RX_BIT 0
#define USBEpIntEn_EP0TX_MASK 0x2
#define USBEpIntEn_EP0TX 0x2
#define USBEpIntEn_EP0TX_BIT 1
#define USBEpIntEn_EP1RX_MASK 0x4
#define USBEpIntEn_EP1RX 0x4
#define USBEpIntEn_EP1RX_BIT 2
#define USBEpIntEn_EP1TX_MASK 0x8
#define USBEpIntEn_EP1TX 0x8
#define USBEpIntEn_EP1TX_BIT 3
#define USBEpIntEn_EP2RX_MASK 0x10
#define USBEpIntEn_EP2RX 0x10
#define USBEpIntEn_EP2RX_BIT 4
#define USBEpIntEn_EP2TX_MASK 0x20
#define USBEpIntEn_EP2TX 0x20
#define USBEpIntEn_EP2TX_BIT 5
#define USBEpIntEn_EP3RX_MASK 0x40
#define USBEpIntEn_EP3RX 0x40
#define USBEpIntEn_EP3RX_BIT 6
#define USBEpIntEn_EP3TX_MASK 0x80
#define USBEpIntEn_EP3TX 0x80
#define USBEpIntEn_EP3TX_BIT 7
#define USBEpIntEn_EP4RX_MASK 0x100
#define USBEpIntEn_EP4RX 0x100
#define USBEpIntEn_EP4RX_BIT 8
#define USBEpIntEn_EP4TX_MASK 0x200
#define USBEpIntEn_EP4TX 0x200
#define USBEpIntEn_EP4TX_BIT 9
#define USBEpIntEn_EP5RX_MASK 0x400
#define USBEpIntEn_EP5RX 0x400
#define USBEpIntEn_EP5RX_BIT 10
#define USBEpIntEn_EP5TX_MASK 0x800
#define USBEpIntEn_EP5TX 0x800
#define USBEpIntEn_EP5TX_BIT 11
#define USBEpIntEn_EP6RX_MASK 0x1000
#define USBEpIntEn_EP6RX 0x1000
#define USBEpIntEn_EP6RX_BIT 12
#define USBEpIntEn_EP6TX_MASK 0x2000
#define USBEpIntEn_EP6TX 0x2000
#define USBEpIntEn_EP6TX_BIT 13
#define USBEpIntEn_EP7RX_MASK 0x4000
#define USBEpIntEn_EP7RX 0x4000
#define USBEpIntEn_EP7RX_BIT 14
#define USBEpIntEn_EP7TX_MASK 0x8000
#define USBEpIntEn_EP7TX 0x8000
#define USBEpIntEn_EP7TX_BIT 15
#define USBEpIntEn_EP8RX_MASK 0x10000
#define USBEpIntEn_EP8RX 0x10000
#define USBEpIntEn_EP8RX_BIT 16
#define USBEpIntEn_EP8TX_MASK 0x20000
#define USBEpIntEn_EP8TX 0x20000
#define USBEpIntEn_EP8TX_BIT 17
#define USBEpIntEn_EP9RX_MASK 0x40000
#define USBEpIntEn_EP9RX 0x40000
#define USBEpIntEn_EP9RX_BIT 18
#define USBEpIntEn_EP9TX_MASK 0x80000
#define USBEpIntEn_EP9TX 0x80000
#define USBEpIntEn_EP9TX_BIT 19
#define USBEpIntEn_EP10RX_MASK 0x100000
#define USBEpIntEn_EP10RX 0x100000
#define USBEpIntEn_EP10RX_BIT 20
#define USBEpIntEn_EP10TX_MASK 0x200000
#define USBEpIntEn_EP10TX 0x200000
#define USBEpIntEn_EP10TX_BIT 21
#define USBEpIntEn_EP11RX_MASK 0x400000
#define USBEpIntEn_EP11RX 0x400000
#define USBEpIntEn_EP11RX_BIT 22
#define USBEpIntEn_EP11TX_MASK 0x800000
#define USBEpIntEn_EP11TX 0x800000
#define USBEpIntEn_EP11TX_BIT 23
#define USBEpIntEn_EP12RX_MASK 0x1000000
#define USBEpIntEn_EP12RX 0x1000000
#define USBEpIntEn_EP12RX_BIT 24
#define USBEpIntEn_EP12TX_MASK 0x2000000
#define USBEpIntEn_EP12TX 0x2000000
#define USBEpIntEn_EP12TX_BIT 25
#define USBEpIntEn_EP13RX_MASK 0x4000000
#define USBEpIntEn_EP13RX 0x4000000
#define USBEpIntEn_EP13RX_BIT 26
#define USBEpIntEn_EP13TX_MASK 0x8000000
#define USBEpIntEn_EP13TX 0x8000000
#define USBEpIntEn_EP13TX_BIT 27
#define USBEpIntEn_EP14RX_MASK 0x10000000
#define USBEpIntEn_EP14RX 0x10000000
#define USBEpIntEn_EP14RX_BIT 28
#define USBEpIntEn_EP14TX_MASK 0x20000000
#define USBEpIntEn_EP14TX 0x20000000
#define USBEpIntEn_EP14TX_BIT 29
#define USBEpIntEn_EP15RX_MASK 0x40000000
#define USBEpIntEn_EP15RX 0x40000000
#define USBEpIntEn_EP15RX_BIT 30
#define USBEpIntEn_EP15TX_MASK 0x80000000
#define USBEpIntEn_EP15TX 0x80000000
#define USBEpIntEn_EP15TX_BIT 31

#define USBEpIntClr (*(volatile uint32_t *)0x2008C238)
#define USBEpIntClr_OFFSET 0x238
#define USBEpIntClr_EP0RX_MASK 0x1
#define USBEpIntClr_EP0RX 0x1
#define USBEpIntClr_EP0RX_BIT 0
#define USBEpIntClr_EP0TX_MASK 0x2
#define USBEpIntClr_EP0TX 0x2
#define USBEpIntClr_EP0TX_BIT 1
#define USBEpIntClr_EP1RX_MASK 0x4
#define USBEpIntClr_EP1RX 0x4
#define USBEpIntClr_EP1RX_BIT 2
#define USBEpIntClr_EP1TX_MASK 0x8
#define USBEpIntClr_EP1TX 0x8
#define USBEpIntClr_EP1TX_BIT 3
#define USBEpIntClr_EP2RX_MASK 0x10
#define USBEpIntClr_EP2RX 0x10
#define USBEpIntClr_EP2RX_BIT 4
#define USBEpIntClr_EP2TX_MASK 0x20
#define USBEpIntClr_EP2TX 0x20
#define USBEpIntClr_EP2TX_BIT 5
#define USBEpIntClr_EP3RX_MASK 0x40
#define USBEpIntClr_EP3RX 0x40
#define USBEpIntClr_EP3RX_BIT 6
#define USBEpIntClr_EP3TX_MASK 0x80
#define USBEpIntClr_EP3TX 0x80
#define USBEpIntClr_EP3TX_BIT 7
#define USBEpIntClr_EP4RX_MASK 0x100
#define USBEpIntClr_EP4RX 0x100
#define USBEpIntClr_EP4RX_BIT 8
#define USBEpIntClr_EP4TX_MASK 0x200
#define USBEpIntClr_EP4TX 0x200
#define USBEpIntClr_EP4TX_BIT 9
#define USBEpIntClr_EP5RX_MASK 0x400
#define USBEpIntClr_EP5RX 0x400
#define USBEpIntClr_EP5RX_BIT 10
#define USBEpIntClr_EP5TX_MASK 0x800
#define USBEpIntClr_EP5TX 0x800
#define USBEpIntClr_EP5TX_BIT 11
#define USBEpIntClr_EP6RX_MASK 0x1000
#define USBEpIntClr_EP6RX 0x1000
#define USBEpIntClr_EP6RX_BIT 12
#define USBEpIntClr_EP6TX_MASK 0x2000
#define USBEpIntClr_EP6TX 0x2000
#define USBEpIntClr_EP6TX_BIT 13
#define USBEpIntClr_EP7RX_MASK 0x4000
#define USBEpIntClr_EP7RX 0x4000
#define USBEpIntClr_EP7RX_BIT 14
#define USBEpIntClr_EP7TX_MASK 0x8000
#define USBEpIntClr_EP7TX 0x8000
#define USBEpIntClr_EP7TX_BIT 15
#define USBEpIntClr_EP8RX_MASK 0x10000
#define USBEpIntClr_EP8RX 0x10000
#define USBEpIntClr_EP8RX_BIT 16
#define USBEpIntClr_EP8TX_MASK 0x20000
#define USBEpIntClr_EP8TX 0x20000
#define USBEpIntClr_EP8TX_BIT 17
#define USBEpIntClr_EP9RX_MASK 0x40000
#define USBEpIntClr_EP9RX 0x40000
#define USBEpIntClr_EP9RX_BIT 18
#define USBEpIntClr_EP9TX_MASK 0x80000
#define USBEpIntClr_EP9TX 0x80000
#define USBEpIntClr_EP9TX_BIT 19
#define USBEpIntClr_EP10RX_MASK 0x100000
#define USBEpIntClr_EP10RX 0x100000
#define USBEpIntClr_EP10RX_BIT 20
#define USBEpIntClr_EP10TX_MASK 0x200000
#define USBEpIntClr_EP10TX 0x200000
#define USBEpIntClr_EP10TX_BIT 21
#define USBEpIntClr_EP11RX_MASK 0x400000
#define USBEpIntClr_EP11RX 0x400000
#define USBEpIntClr_EP11RX_BIT 22
#define USBEpIntClr_EP11TX_MASK 0x800000
#define USBEpIntClr_EP11TX 0x800000
#define USBEpIntClr_EP11TX_BIT 23
#define USBEpIntClr_EP12RX_MASK 0x1000000
#define USBEpIntClr_EP12RX 0x1000000
#define USBEpIntClr_EP12RX_BIT 24
#define USBEpIntClr_EP12TX_MASK 0x2000000
#define USBEpIntClr_EP12TX 0x2000000
#define USBEpIntClr_EP12TX_BIT 25
#define USBEpIntClr_EP13RX_MASK 0x4000000
#define USBEpIntClr_EP13RX 0x4000000
#define USBEpIntClr_EP13RX_BIT 26
#define USBEpIntClr_EP13TX_MASK 0x8000000
#define USBEpIntClr_EP13TX 0x8000000
#define USBEpIntClr_EP13TX_BIT 27
#define USBEpIntClr_EP14RX_MASK 0x10000000
#define USBEpIntClr_EP14RX 0x10000000
#define USBEpIntClr_EP14RX_BIT 28
#define USBEpIntClr_EP14TX_MASK 0x20000000
#define USBEpIntClr_EP14TX 0x20000000
#define USBEpIntClr_EP14TX_BIT 29
#define USBEpIntClr_EP15RX_MASK 0x40000000
#define USBEpIntClr_EP15RX 0x40000000
#define USBEpIntClr_EP15RX_BIT 30
#define USBEpIntClr_EP15TX_MASK 0x80000000
#define USBEpIntClr_EP15TX 0x80000000
#define USBEpIntClr_EP15TX_BIT 31

#define USBEpIntSet (*(volatile uint32_t *)0x2008C23C)
#define USBEpIntSet_OFFSET 0x23C
#define USBEpIntSet_EP0RX_MASK 0x1
#define USBEpIntSet_EP0RX 0x1
#define USBEpIntSet_EP0RX_BIT 0
#define USBEpIntSet_EP0TX_MASK 0x2
#define USBEpIntSet_EP0TX 0x2
#define USBEpIntSet_EP0TX_BIT 1
#define USBEpIntSet_EP1RX_MASK 0x4
#define USBEpIntSet_EP1RX 0x4
#define USBEpIntSet_EP1RX_BIT 2
#define USBEpIntSet_EP1TX_MASK 0x8
#define USBEpIntSet_EP1TX 0x8
#define USBEpIntSet_EP1TX_BIT 3
#define USBEpIntSet_EP2RX_MASK 0x10
#define USBEpIntSet_EP2RX 0x10
#define USBEpIntSet_EP2RX_BIT 4
#define USBEpIntSet_EP2TX_MASK 0x20
#define USBEpIntSet_EP2TX 0x20
#define USBEpIntSet_EP2TX_BIT 5
#define USBEpIntSet_EP3RX_MASK 0x40
#define USBEpIntSet_EP3RX 0x40
#define USBEpIntSet_EP3RX_BIT 6
#define USBEpIntSet_EP3TX_MASK 0x80
#define USBEpIntSet_EP3TX 0x80
#define USBEpIntSet_EP3TX_BIT 7
#define USBEpIntSet_EP4RX_MASK 0x100
#define USBEpIntSet_EP4RX 0x100
#define USBEpIntSet_EP4RX_BIT 8
#define USBEpIntSet_EP4TX_MASK 0x200
#define USBEpIntSet_EP4TX 0x200
#define USBEpIntSet_EP4TX_BIT 9
#define USBEpIntSet_EP5RX_MASK 0x400
#define USBEpIntSet_EP5RX 0x400
#define USBEpIntSet_EP5RX_BIT 10
#define USBEpIntSet_EP5TX_MASK 0x800
#define USBEpIntSet_EP5TX 0x800
#define USBEpIntSet_EP5TX_BIT 11
#define USBEpIntSet_EP6RX_MASK 0x1000
#define USBEpIntSet_EP6RX 0x1000
#define USBEpIntSet_EP6RX_BIT 12
#define USBEpIntSet_EP6TX_MASK 0x2000
#define USBEpIntSet_EP6TX 0x2000
#define USBEpIntSet_EP6TX_BIT 13
#define USBEpIntSet_EP7RX_MASK 0x4000
#define USBEpIntSet_EP7RX 0x4000
#define USBEpIntSet_EP7RX_BIT 14
#define USBEpIntSet_EP7TX_MASK 0x8000
#define USBEpIntSet_EP7TX 0x8000
#define USBEpIntSet_EP7TX_BIT 15
#define USBEpIntSet_EP8RX_MASK 0x10000
#define USBEpIntSet_EP8RX 0x10000
#define USBEpIntSet_EP8RX_BIT 16
#define USBEpIntSet_EP8TX_MASK 0x20000
#define USBEpIntSet_EP8TX 0x20000
#define USBEpIntSet_EP8TX_BIT 17
#define USBEpIntSet_EP9RX_MASK 0x40000
#define USBEpIntSet_EP9RX 0x40000
#define USBEpIntSet_EP9RX_BIT 18
#define USBEpIntSet_EP9TX_MASK 0x80000
#define USBEpIntSet_EP9TX 0x80000
#define USBEpIntSet_EP9TX_BIT 19
#define USBEpIntSet_EP10RX_MASK 0x100000
#define USBEpIntSet_EP10RX 0x100000
#define USBEpIntSet_EP10RX_BIT 20
#define USBEpIntSet_EP10TX_MASK 0x200000
#define USBEpIntSet_EP10TX 0x200000
#define USBEpIntSet_EP10TX_BIT 21
#define USBEpIntSet_EP11RX_MASK 0x400000
#define USBEpIntSet_EP11RX 0x400000
#define USBEpIntSet_EP11RX_BIT 22
#define USBEpIntSet_EP11TX_MASK 0x800000
#define USBEpIntSet_EP11TX 0x800000
#define USBEpIntSet_EP11TX_BIT 23
#define USBEpIntSet_EP12RX_MASK 0x1000000
#define USBEpIntSet_EP12RX 0x1000000
#define USBEpIntSet_EP12RX_BIT 24
#define USBEpIntSet_EP12TX_MASK 0x2000000
#define USBEpIntSet_EP12TX 0x2000000
#define USBEpIntSet_EP12TX_BIT 25
#define USBEpIntSet_EP13RX_MASK 0x4000000
#define USBEpIntSet_EP13RX 0x4000000
#define USBEpIntSet_EP13RX_BIT 26
#define USBEpIntSet_EP13TX_MASK 0x8000000
#define USBEpIntSet_EP13TX 0x8000000
#define USBEpIntSet_EP13TX_BIT 27
#define USBEpIntSet_EP14RX_MASK 0x10000000
#define USBEpIntSet_EP14RX 0x10000000
#define USBEpIntSet_EP14RX_BIT 28
#define USBEpIntSet_EP14TX_MASK 0x20000000
#define USBEpIntSet_EP14TX 0x20000000
#define USBEpIntSet_EP14TX_BIT 29
#define USBEpIntSet_EP15RX_MASK 0x40000000
#define USBEpIntSet_EP15RX 0x40000000
#define USBEpIntSet_EP15RX_BIT 30
#define USBEpIntSet_EP15TX_MASK 0x80000000
#define USBEpIntSet_EP15TX 0x80000000
#define USBEpIntSet_EP15TX_BIT 31

#define USBEpIntPri (*(volatile uint32_t *)0x2008C240)
#define USBEpIntPri_OFFSET 0x240
#define USBEpIntPri_EP0RX_MASK 0x1
#define USBEpIntPri_EP0RX 0x1
#define USBEpIntPri_EP0RX_BIT 0
#define USBEpIntPri_EP0TX_MASK 0x2
#define USBEpIntPri_EP0TX 0x2
#define USBEpIntPri_EP0TX_BIT 1
#define USBEpIntPri_EP1RX_MASK 0x4
#define USBEpIntPri_EP1RX 0x4
#define USBEpIntPri_EP1RX_BIT 2
#define USBEpIntPri_EP1TX_MASK 0x8
#define USBEpIntPri_EP1TX 0x8
#define USBEpIntPri_EP1TX_BIT 3
#define USBEpIntPri_EP2RX_MASK 0x10
#define USBEpIntPri_EP2RX 0x10
#define USBEpIntPri_EP2RX_BIT 4
#define USBEpIntPri_EP2TX_MASK 0x20
#define USBEpIntPri_EP2TX 0x20
#define USBEpIntPri_EP2TX_BIT 5
#define USBEpIntPri_EP3RX_MASK 0x40
#define USBEpIntPri_EP3RX 0x40
#define USBEpIntPri_EP3RX_BIT 6
#define USBEpIntPri_EP3TX_MASK 0x80
#define USBEpIntPri_EP3TX 0x80
#define USBEpIntPri_EP3TX_BIT 7
#define USBEpIntPri_EP4RX_MASK 0x100
#define USBEpIntPri_EP4RX 0x100
#define USBEpIntPri_EP4RX_BIT 8
#define USBEpIntPri_EP4TX_MASK 0x200
#define USBEpIntPri_EP4TX 0x200
#define USBEpIntPri_EP4TX_BIT 9
#define USBEpIntPri_EP5RX_MASK 0x400
#define USBEpIntPri_EP5RX 0x400
#define USBEpIntPri_EP5RX_BIT 10
#define USBEpIntPri_EP5TX_MASK 0x800
#define USBEpIntPri_EP5TX 0x800
#define USBEpIntPri_EP5TX_BIT 11
#define USBEpIntPri_EP6RX_MASK 0x1000
#define USBEpIntPri_EP6RX 0x1000
#define USBEpIntPri_EP6RX_BIT 12
#define USBEpIntPri_EP6TX_MASK 0x2000
#define USBEpIntPri_EP6TX 0x2000
#define USBEpIntPri_EP6TX_BIT 13
#define USBEpIntPri_EP7RX_MASK 0x4000
#define USBEpIntPri_EP7RX 0x4000
#define USBEpIntPri_EP7RX_BIT 14
#define USBEpIntPri_EP7TX_MASK 0x8000
#define USBEpIntPri_EP7TX 0x8000
#define USBEpIntPri_EP7TX_BIT 15
#define USBEpIntPri_EP8RX_MASK 0x10000
#define USBEpIntPri_EP8RX 0x10000
#define USBEpIntPri_EP8RX_BIT 16
#define USBEpIntPri_EP8TX_MASK 0x20000
#define USBEpIntPri_EP8TX 0x20000
#define USBEpIntPri_EP8TX_BIT 17
#define USBEpIntPri_EP9RX_MASK 0x40000
#define USBEpIntPri_EP9RX 0x40000
#define USBEpIntPri_EP9RX_BIT 18
#define USBEpIntPri_EP9TX_MASK 0x80000
#define USBEpIntPri_EP9TX 0x80000
#define USBEpIntPri_EP9TX_BIT 19
#define USBEpIntPri_EP10RX_MASK 0x100000
#define USBEpIntPri_EP10RX 0x100000
#define USBEpIntPri_EP10RX_BIT 20
#define USBEpIntPri_EP10TX_MASK 0x200000
#define USBEpIntPri_EP10TX 0x200000
#define USBEpIntPri_EP10TX_BIT 21
#define USBEpIntPri_EP11RX_MASK 0x400000
#define USBEpIntPri_EP11RX 0x400000
#define USBEpIntPri_EP11RX_BIT 22
#define USBEpIntPri_EP11TX_MASK 0x800000
#define USBEpIntPri_EP11TX 0x800000
#define USBEpIntPri_EP11TX_BIT 23
#define USBEpIntPri_EP12RX_MASK 0x1000000
#define USBEpIntPri_EP12RX 0x1000000
#define USBEpIntPri_EP12RX_BIT 24
#define USBEpIntPri_EP12TX_MASK 0x2000000
#define USBEpIntPri_EP12TX 0x2000000
#define USBEpIntPri_EP12TX_BIT 25
#define USBEpIntPri_EP13RX_MASK 0x4000000
#define USBEpIntPri_EP13RX 0x4000000
#define USBEpIntPri_EP13RX_BIT 26
#define USBEpIntPri_EP13TX_MASK 0x8000000
#define USBEpIntPri_EP13TX 0x8000000
#define USBEpIntPri_EP13TX_BIT 27
#define USBEpIntPri_EP14RX_MASK 0x10000000
#define USBEpIntPri_EP14RX 0x10000000
#define USBEpIntPri_EP14RX_BIT 28
#define USBEpIntPri_EP14TX_MASK 0x20000000
#define USBEpIntPri_EP14TX 0x20000000
#define USBEpIntPri_EP14TX_BIT 29
#define USBEpIntPri_EP15RX_MASK 0x40000000
#define USBEpIntPri_EP15RX 0x40000000
#define USBEpIntPri_EP15RX_BIT 30
#define USBEpIntPri_EP15TX_MASK 0x80000000
#define USBEpIntPri_EP15TX 0x80000000
#define USBEpIntPri_EP15TX_BIT 31

#define USBReEp (*(volatile uint32_t *)0x2008C244)
#define USBReEp_OFFSET 0x244
#define USBReEp_EP0_MASK 0x1
#define USBReEp_EP0 0x1
#define USBReEp_EP0_BIT 0
#define USBReEp_EP1_MASK 0x2
#define USBReEp_EP1 0x2
#define USBReEp_EP1_BIT 1
#define USBReEp_EP2_MASK 0x4
#define USBReEp_EP2 0x4
#define USBReEp_EP2_BIT 2
#define USBReEp_EP3_MASK 0x8
#define USBReEp_EP3 0x8
#define USBReEp_EP3_BIT 3
#define USBReEp_EP4_MASK 0x10
#define USBReEp_EP4 0x10
#define USBReEp_EP4_BIT 4
#define USBReEp_EP5_MASK 0x20
#define USBReEp_EP5 0x20
#define USBReEp_EP5_BIT 5
#define USBReEp_EP6_MASK 0x40
#define USBReEp_EP6 0x40
#define USBReEp_EP6_BIT 6
#define USBReEp_EP7_MASK 0x80
#define USBReEp_EP7 0x80
#define USBReEp_EP7_BIT 7
#define USBReEp_EP8_MASK 0x100
#define USBReEp_EP8 0x100
#define USBReEp_EP8_BIT 8
#define USBReEp_EP9_MASK 0x200
#define USBReEp_EP9 0x200
#define USBReEp_EP9_BIT 9
#define USBReEp_EP10_MASK 0x400
#define USBReEp_EP10 0x400
#define USBReEp_EP10_BIT 10
#define USBReEp_EP11_MASK 0x800
#define USBReEp_EP11 0x800
#define USBReEp_EP11_BIT 11
#define USBReEp_EP12_MASK 0x1000
#define USBReEp_EP12 0x1000
#define USBReEp_EP12_BIT 12
#define USBReEp_EP13_MASK 0x2000
#define USBReEp_EP13 0x2000
#define USBReEp_EP13_BIT 13
#define USBReEp_EP14_MASK 0x4000
#define USBReEp_EP14 0x4000
#define USBReEp_EP14_BIT 14
#define USBReEp_EP15_MASK 0x8000
#define USBReEp_EP15 0x8000
#define USBReEp_EP15_BIT 15
#define USBReEp_EP16_MASK 0x10000
#define USBReEp_EP16 0x10000
#define USBReEp_EP16_BIT 16
#define USBReEp_EP17_MASK 0x20000
#define USBReEp_EP17 0x20000
#define USBReEp_EP17_BIT 17
#define USBReEp_EP18_MASK 0x40000
#define USBReEp_EP18 0x40000
#define USBReEp_EP18_BIT 18
#define USBReEp_EP19_MASK 0x80000
#define USBReEp_EP19 0x80000
#define USBReEp_EP19_BIT 19
#define USBReEp_EP20_MASK 0x100000
#define USBReEp_EP20 0x100000
#define USBReEp_EP20_BIT 20
#define USBReEp_EP21_MASK 0x200000
#define USBReEp_EP21 0x200000
#define USBReEp_EP21_BIT 21
#define USBReEp_EP22_MASK 0x400000
#define USBReEp_EP22 0x400000
#define USBReEp_EP22_BIT 22
#define USBReEp_EP23_MASK 0x800000
#define USBReEp_EP23 0x800000
#define USBReEp_EP23_BIT 23
#define USBReEp_EP24_MASK 0x1000000
#define USBReEp_EP24 0x1000000
#define USBReEp_EP24_BIT 24
#define USBReEp_EP25_MASK 0x2000000
#define USBReEp_EP25 0x2000000
#define USBReEp_EP25_BIT 25
#define USBReEp_EP26_MASK 0x4000000
#define USBReEp_EP26 0x4000000
#define USBReEp_EP26_BIT 26
#define USBReEp_EP27_MASK 0x8000000
#define USBReEp_EP27 0x8000000
#define USBReEp_EP27_BIT 27
#define USBReEp_EP28_MASK 0x10000000
#define USBReEp_EP28 0x10000000
#define USBReEp_EP28_BIT 28
#define USBReEp_EP29_MASK 0x20000000
#define USBReEp_EP29 0x20000000
#define USBReEp_EP29_BIT 29
#define USBReEp_EP30_MASK 0x40000000
#define USBReEp_EP30 0x40000000
#define USBReEp_EP30_BIT 30
#define USBReEp_EP31_MASK 0x80000000
#define USBReEp_EP31 0x80000000
#define USBReEp_EP31_BIT 31

#define USBEpInd (*(volatile uint32_t *)0x2008C248)
#define USBEpInd_OFFSET 0x248
#define USBEpInd_Phy_endpoint_MASK 0x1F
#define USBEpInd_Phy_endpoint_BIT 0

#define USBEpMaxPSize (*(volatile uint32_t *)0x2008C24C)
#define USBEpMaxPSize_OFFSET 0x24C
#define USBEpMaxPSize_MaxPacketSize_MASK 0x3FF
#define USBEpMaxPSize_MaxPacketSize_BIT 0

#define USBDMARSt (*(volatile uint32_t *)0x2008C250)
#define USBDMARSt_OFFSET 0x250
#define USBDMARSt_EP0_MASK 0x1
#define USBDMARSt_EP0 0x1
#define USBDMARSt_EP0_BIT 0
#define USBDMARSt_EP1_MASK 0x2
#define USBDMARSt_EP1 0x2
#define USBDMARSt_EP1_BIT 1
#define USBDMARSt_EP2_MASK 0x4
#define USBDMARSt_EP2 0x4
#define USBDMARSt_EP2_BIT 2
#define USBDMARSt_EP3_MASK 0x8
#define USBDMARSt_EP3 0x8
#define USBDMARSt_EP3_BIT 3
#define USBDMARSt_EP4_MASK 0x10
#define USBDMARSt_EP4 0x10
#define USBDMARSt_EP4_BIT 4
#define USBDMARSt_EP5_MASK 0x20
#define USBDMARSt_EP5 0x20
#define USBDMARSt_EP5_BIT 5
#define USBDMARSt_EP6_MASK 0x40
#define USBDMARSt_EP6 0x40
#define USBDMARSt_EP6_BIT 6
#define USBDMARSt_EP7_MASK 0x80
#define USBDMARSt_EP7 0x80
#define USBDMARSt_EP7_BIT 7
#define USBDMARSt_EP8_MASK 0x100
#define USBDMARSt_EP8 0x100
#define USBDMARSt_EP8_BIT 8
#define USBDMARSt_EP9_MASK 0x200
#define USBDMARSt_EP9 0x200
#define USBDMARSt_EP9_BIT 9
#define USBDMARSt_EP10_MASK 0x400
#define USBDMARSt_EP10 0x400
#define USBDMARSt_EP10_BIT 10
#define USBDMARSt_EP11_MASK 0x800
#define USBDMARSt_EP11 0x800
#define USBDMARSt_EP11_BIT 11
#define USBDMARSt_EP12_MASK 0x1000
#define USBDMARSt_EP12 0x1000
#define USBDMARSt_EP12_BIT 12
#define USBDMARSt_EP13_MASK 0x2000
#define USBDMARSt_EP13 0x2000
#define USBDMARSt_EP13_BIT 13
#define USBDMARSt_EP14_MASK 0x4000
#define USBDMARSt_EP14 0x4000
#define USBDMARSt_EP14_BIT 14
#define USBDMARSt_EP15_MASK 0x8000
#define USBDMARSt_EP15 0x8000
#define USBDMARSt_EP15_BIT 15
#define USBDMARSt_EP16_MASK 0x10000
#define USBDMARSt_EP16 0x10000
#define USBDMARSt_EP16_BIT 16
#define USBDMARSt_EP17_MASK 0x20000
#define USBDMARSt_EP17 0x20000
#define USBDMARSt_EP17_BIT 17
#define USBDMARSt_EP18_MASK 0x40000
#define USBDMARSt_EP18 0x40000
#define USBDMARSt_EP18_BIT 18
#define USBDMARSt_EP19_MASK 0x80000
#define USBDMARSt_EP19 0x80000
#define USBDMARSt_EP19_BIT 19
#define USBDMARSt_EP20_MASK 0x100000
#define USBDMARSt_EP20 0x100000
#define USBDMARSt_EP20_BIT 20
#define USBDMARSt_EP21_MASK 0x200000
#define USBDMARSt_EP21 0x200000
#define USBDMARSt_EP21_BIT 21
#define USBDMARSt_EP22_MASK 0x400000
#define USBDMARSt_EP22 0x400000
#define USBDMARSt_EP22_BIT 22
#define USBDMARSt_EP23_MASK 0x800000
#define USBDMARSt_EP23 0x800000
#define USBDMARSt_EP23_BIT 23
#define USBDMARSt_EP24_MASK 0x1000000
#define USBDMARSt_EP24 0x1000000
#define USBDMARSt_EP24_BIT 24
#define USBDMARSt_EP25_MASK 0x2000000
#define USBDMARSt_EP25 0x2000000
#define USBDMARSt_EP25_BIT 25
#define USBDMARSt_EP26_MASK 0x4000000
#define USBDMARSt_EP26 0x4000000
#define USBDMARSt_EP26_BIT 26
#define USBDMARSt_EP27_MASK 0x8000000
#define USBDMARSt_EP27 0x8000000
#define USBDMARSt_EP27_BIT 27
#define USBDMARSt_EP28_MASK 0x10000000
#define USBDMARSt_EP28 0x10000000
#define USBDMARSt_EP28_BIT 28
#define USBDMARSt_EP29_MASK 0x20000000
#define USBDMARSt_EP29 0x20000000
#define USBDMARSt_EP29_BIT 29
#define USBDMARSt_EP30_MASK 0x40000000
#define USBDMARSt_EP30 0x40000000
#define USBDMARSt_EP30_BIT 30
#define USBDMARSt_EP31_MASK 0x80000000
#define USBDMARSt_EP31 0x80000000
#define USBDMARSt_EP31_BIT 31

#define USBDMARClr (*(volatile uint32_t *)0x2008C254)
#define USBDMARClr_OFFSET 0x254
#define USBDMARClr_EP0_MASK 0x1
#define USBDMARClr_EP0 0x1
#define USBDMARClr_EP0_BIT 0
#define USBDMARClr_EP1_MASK 0x2
#define USBDMARClr_EP1 0x2
#define USBDMARClr_EP1_BIT 1
#define USBDMARClr_EP2_MASK 0x4
#define USBDMARClr_EP2 0x4
#define USBDMARClr_EP2_BIT 2
#define USBDMARClr_EP3_MASK 0x8
#define USBDMARClr_EP3 0x8
#define USBDMARClr_EP3_BIT 3
#define USBDMARClr_EP4_MASK 0x10
#define USBDMARClr_EP4 0x10
#define USBDMARClr_EP4_BIT 4
#define USBDMARClr_EP5_MASK 0x20
#define USBDMARClr_EP5 0x20
#define USBDMARClr_EP5_BIT 5
#define USBDMARClr_EP6_MASK 0x40
#define USBDMARClr_EP6 0x40
#define USBDMARClr_EP6_BIT 6
#define USBDMARClr_EP7_MASK 0x80
#define USBDMARClr_EP7 0x80
#define USBDMARClr_EP7_BIT 7
#define USBDMARClr_EP8_MASK 0x100
#define USBDMARClr_EP8 0x100
#define USBDMARClr_EP8_BIT 8
#define USBDMARClr_EP9_MASK 0x200
#define USBDMARClr_EP9 0x200
#define USBDMARClr_EP9_BIT 9
#define USBDMARClr_EP10_MASK 0x400
#define USBDMARClr_EP10 0x400
#define USBDMARClr_EP10_BIT 10
#define USBDMARClr_EP11_MASK 0x800
#define USBDMARClr_EP11 0x800
#define USBDMARClr_EP11_BIT 11
#define USBDMARClr_EP12_MASK 0x1000
#define USBDMARClr_EP12 0x1000
#define USBDMARClr_EP12_BIT 12
#define USBDMARClr_EP13_MASK 0x2000
#define USBDMARClr_EP13 0x2000
#define USBDMARClr_EP13_BIT 13
#define USBDMARClr_EP14_MASK 0x4000
#define USBDMARClr_EP14 0x4000
#define USBDMARClr_EP14_BIT 14
#define USBDMARClr_EP15_MASK 0x8000
#define USBDMARClr_EP15 0x8000
#define USBDMARClr_EP15_BIT 15
#define USBDMARClr_EP16_MASK 0x10000
#define USBDMARClr_EP16 0x10000
#define USBDMARClr_EP16_BIT 16
#define USBDMARClr_EP17_MASK 0x20000
#define USBDMARClr_EP17 0x20000
#define USBDMARClr_EP17_BIT 17
#define USBDMARClr_EP18_MASK 0x40000
#define USBDMARClr_EP18 0x40000
#define USBDMARClr_EP18_BIT 18
#define USBDMARClr_EP19_MASK 0x80000
#define USBDMARClr_EP19 0x80000
#define USBDMARClr_EP19_BIT 19
#define USBDMARClr_EP20_MASK 0x100000
#define USBDMARClr_EP20 0x100000
#define USBDMARClr_EP20_BIT 20
#define USBDMARClr_EP21_MASK 0x200000
#define USBDMARClr_EP21 0x200000
#define USBDMARClr_EP21_BIT 21
#define USBDMARClr_EP22_MASK 0x400000
#define USBDMARClr_EP22 0x400000
#define USBDMARClr_EP22_BIT 22
#define USBDMARClr_EP23_MASK 0x800000
#define USBDMARClr_EP23 0x800000
#define USBDMARClr_EP23_BIT 23
#define USBDMARClr_EP24_MASK 0x1000000
#define USBDMARClr_EP24 0x1000000
#define USBDMARClr_EP24_BIT 24
#define USBDMARClr_EP25_MASK 0x2000000
#define USBDMARClr_EP25 0x2000000
#define USBDMARClr_EP25_BIT 25
#define USBDMARClr_EP26_MASK 0x4000000
#define USBDMARClr_EP26 0x4000000
#define USBDMARClr_EP26_BIT 26
#define USBDMARClr_EP27_MASK 0x8000000
#define USBDMARClr_EP27 0x8000000
#define USBDMARClr_EP27_BIT 27
#define USBDMARClr_EP28_MASK 0x10000000
#define USBDMARClr_EP28 0x10000000
#define USBDMARClr_EP28_BIT 28
#define USBDMARClr_EP29_MASK 0x20000000
#define USBDMARClr_EP29 0x20000000
#define USBDMARClr_EP29_BIT 29
#define USBDMARClr_EP30_MASK 0x40000000
#define USBDMARClr_EP30 0x40000000
#define USBDMARClr_EP30_BIT 30
#define USBDMARClr_EP31_MASK 0x80000000
#define USBDMARClr_EP31 0x80000000
#define USBDMARClr_EP31_BIT 31

#define USBDMARSet (*(volatile uint32_t *)0x2008C258)
#define USBDMARSet_OFFSET 0x258
#define USBDMARSet_EP0_MASK 0x1
#define USBDMARSet_EP0 0x1
#define USBDMARSet_EP0_BIT 0
#define USBDMARSet_EP1_MASK 0x2
#define USBDMARSet_EP1 0x2
#define USBDMARSet_EP1_BIT 1
#define USBDMARSet_EP2_MASK 0x4
#define USBDMARSet_EP2 0x4
#define USBDMARSet_EP2_BIT 2
#define USBDMARSet_EP3_MASK 0x8
#define USBDMARSet_EP3 0x8
#define USBDMARSet_EP3_BIT 3
#define USBDMARSet_EP4_MASK 0x10
#define USBDMARSet_EP4 0x10
#define USBDMARSet_EP4_BIT 4
#define USBDMARSet_EP5_MASK 0x20
#define USBDMARSet_EP5 0x20
#define USBDMARSet_EP5_BIT 5
#define USBDMARSet_EP6_MASK 0x40
#define USBDMARSet_EP6 0x40
#define USBDMARSet_EP6_BIT 6
#define USBDMARSet_EP7_MASK 0x80
#define USBDMARSet_EP7 0x80
#define USBDMARSet_EP7_BIT 7
#define USBDMARSet_EP8_MASK 0x100
#define USBDMARSet_EP8 0x100
#define USBDMARSet_EP8_BIT 8
#define USBDMARSet_EP9_MASK 0x200
#define USBDMARSet_EP9 0x200
#define USBDMARSet_EP9_BIT 9
#define USBDMARSet_EP10_MASK 0x400
#define USBDMARSet_EP10 0x400
#define USBDMARSet_EP10_BIT 10
#define USBDMARSet_EP11_MASK 0x800
#define USBDMARSet_EP11 0x800
#define USBDMARSet_EP11_BIT 11
#define USBDMARSet_EP12_MASK 0x1000
#define USBDMARSet_EP12 0x1000
#define USBDMARSet_EP12_BIT 12
#define USBDMARSet_EP13_MASK 0x2000
#define USBDMARSet_EP13 0x2000
#define USBDMARSet_EP13_BIT 13
#define USBDMARSet_EP14_MASK 0x4000
#define USBDMARSet_EP14 0x4000
#define USBDMARSet_EP14_BIT 14
#define USBDMARSet_EP15_MASK 0x8000
#define USBDMARSet_EP15 0x8000
#define USBDMARSet_EP15_BIT 15
#define USBDMARSet_EP16_MASK 0x10000
#define USBDMARSet_EP16 0x10000
#define USBDMARSet_EP16_BIT 16
#define USBDMARSet_EP17_MASK 0x20000
#define USBDMARSet_EP17 0x20000
#define USBDMARSet_EP17_BIT 17
#define USBDMARSet_EP18_MASK 0x40000
#define USBDMARSet_EP18 0x40000
#define USBDMARSet_EP18_BIT 18
#define USBDMARSet_EP19_MASK 0x80000
#define USBDMARSet_EP19 0x80000
#define USBDMARSet_EP19_BIT 19
#define USBDMARSet_EP20_MASK 0x100000
#define USBDMARSet_EP20 0x100000
#define USBDMARSet_EP20_BIT 20
#define USBDMARSet_EP21_MASK 0x200000
#define USBDMARSet_EP21 0x200000
#define USBDMARSet_EP21_BIT 21
#define USBDMARSet_EP22_MASK 0x400000
#define USBDMARSet_EP22 0x400000
#define USBDMARSet_EP22_BIT 22
#define USBDMARSet_EP23_MASK 0x800000
#define USBDMARSet_EP23 0x800000
#define USBDMARSet_EP23_BIT 23
#define USBDMARSet_EP24_MASK 0x1000000
#define USBDMARSet_EP24 0x1000000
#define USBDMARSet_EP24_BIT 24
#define USBDMARSet_EP25_MASK 0x2000000
#define USBDMARSet_EP25 0x2000000
#define USBDMARSet_EP25_BIT 25
#define USBDMARSet_EP26_MASK 0x4000000
#define USBDMARSet_EP26 0x4000000
#define USBDMARSet_EP26_BIT 26
#define USBDMARSet_EP27_MASK 0x8000000
#define USBDMARSet_EP27 0x8000000
#define USBDMARSet_EP27_BIT 27
#define USBDMARSet_EP28_MASK 0x10000000
#define USBDMARSet_EP28 0x10000000
#define USBDMARSet_EP28_BIT 28
#define USBDMARSet_EP29_MASK 0x20000000
#define USBDMARSet_EP29 0x20000000
#define USBDMARSet_EP29_BIT 29
#define USBDMARSet_EP30_MASK 0x40000000
#define USBDMARSet_EP30 0x40000000
#define USBDMARSet_EP30_BIT 30
#define USBDMARSet_EP31_MASK 0x80000000
#define USBDMARSet_EP31 0x80000000
#define USBDMARSet_EP31_BIT 31

#define USBUDCAH (*(volatile uint32_t *)0x2008C280)
#define USBUDCAH_OFFSET 0x280
#define USBUDCAH_UDCA_Header_MASK 0xFFFFFF80
#define USBUDCAH_UDCA_Header_BIT 7

#define USBEpDMASt (*(volatile uint32_t *)0x2008C284)
#define USBEpDMASt_OFFSET 0x284
#define USBEpDMASt_EP0_DMA_ENABLE_MASK 0x1
#define USBEpDMASt_EP0_DMA_ENABLE 0x1
#define USBEpDMASt_EP0_DMA_ENABLE_BIT 0
#define USBEpDMASt_EP1_DMA_ENABLE_MASK 0x2
#define USBEpDMASt_EP1_DMA_ENABLE 0x2
#define USBEpDMASt_EP1_DMA_ENABLE_BIT 1
#define USBEpDMASt_EP2_DMA_ENABLE_MASK 0x4
#define USBEpDMASt_EP2_DMA_ENABLE 0x4
#define USBEpDMASt_EP2_DMA_ENABLE_BIT 2
#define USBEpDMASt_EP3_DMA_ENABLE_MASK 0x8
#define USBEpDMASt_EP3_DMA_ENABLE 0x8
#define USBEpDMASt_EP3_DMA_ENABLE_BIT 3
#define USBEpDMASt_EP4_DMA_ENABLE_MASK 0x10
#define USBEpDMASt_EP4_DMA_ENABLE 0x10
#define USBEpDMASt_EP4_DMA_ENABLE_BIT 4
#define USBEpDMASt_EP5_DMA_ENABLE_MASK 0x20
#define USBEpDMASt_EP5_DMA_ENABLE 0x20
#define USBEpDMASt_EP5_DMA_ENABLE_BIT 5
#define USBEpDMASt_EP6_DMA_ENABLE_MASK 0x40
#define USBEpDMASt_EP6_DMA_ENABLE 0x40
#define USBEpDMASt_EP6_DMA_ENABLE_BIT 6
#define USBEpDMASt_EP7_DMA_ENABLE_MASK 0x80
#define USBEpDMASt_EP7_DMA_ENABLE 0x80
#define USBEpDMASt_EP7_DMA_ENABLE_BIT 7
#define USBEpDMASt_EP8_DMA_ENABLE_MASK 0x100
#define USBEpDMASt_EP8_DMA_ENABLE 0x100
#define USBEpDMASt_EP8_DMA_ENABLE_BIT 8
#define USBEpDMASt_EP9_DMA_ENABLE_MASK 0x200
#define USBEpDMASt_EP9_DMA_ENABLE 0x200
#define USBEpDMASt_EP9_DMA_ENABLE_BIT 9
#define USBEpDMASt_EP10_DMA_ENABLE_MASK 0x400
#define USBEpDMASt_EP10_DMA_ENABLE 0x400
#define USBEpDMASt_EP10_DMA_ENABLE_BIT 10
#define USBEpDMASt_EP11_DMA_ENABLE_MASK 0x800
#define USBEpDMASt_EP11_DMA_ENABLE 0x800
#define USBEpDMASt_EP11_DMA_ENABLE_BIT 11
#define USBEpDMASt_EP12_DMA_ENABLE_MASK 0x1000
#define USBEpDMASt_EP12_DMA_ENABLE 0x1000
#define USBEpDMASt_EP12_DMA_ENABLE_BIT 12
#define USBEpDMASt_EP13_DMA_ENABLE_MASK 0x2000
#define USBEpDMASt_EP13_DMA_ENABLE 0x2000
#define USBEpDMASt_EP13_DMA_ENABLE_BIT 13
#define USBEpDMASt_EP14_DMA_ENABLE_MASK 0x4000
#define USBEpDMASt_EP14_DMA_ENABLE 0x4000
#define USBEpDMASt_EP14_DMA_ENABLE_BIT 14
#define USBEpDMASt_EP15_DMA_ENABLE_MASK 0x8000
#define USBEpDMASt_EP15_DMA_ENABLE 0x8000
#define USBEpDMASt_EP15_DMA_ENABLE_BIT 15
#define USBEpDMASt_EP16_DMA_ENABLE_MASK 0x10000
#define USBEpDMASt_EP16_DMA_ENABLE 0x10000
#define USBEpDMASt_EP16_DMA_ENABLE_BIT 16
#define USBEpDMASt_EP17_DMA_ENABLE_MASK 0x20000
#define USBEpDMASt_EP17_DMA_ENABLE 0x20000
#define USBEpDMASt_EP17_DMA_ENABLE_BIT 17
#define USBEpDMASt_EP18_DMA_ENABLE_MASK 0x40000
#define USBEpDMASt_EP18_DMA_ENABLE 0x40000
#define USBEpDMASt_EP18_DMA_ENABLE_BIT 18
#define USBEpDMASt_EP19_DMA_ENABLE_MASK 0x80000
#define USBEpDMASt_EP19_DMA_ENABLE 0x80000
#define USBEpDMASt_EP19_DMA_ENABLE_BIT 19
#define USBEpDMASt_EP20_DMA_ENABLE_MASK 0x100000
#define USBEpDMASt_EP20_DMA_ENABLE 0x100000
#define USBEpDMASt_EP20_DMA_ENABLE_BIT 20
#define USBEpDMASt_EP21_DMA_ENABLE_MASK 0x200000
#define USBEpDMASt_EP21_DMA_ENABLE 0x200000
#define USBEpDMASt_EP21_DMA_ENABLE_BIT 21
#define USBEpDMASt_EP22_DMA_ENABLE_MASK 0x400000
#define USBEpDMASt_EP22_DMA_ENABLE 0x400000
#define USBEpDMASt_EP22_DMA_ENABLE_BIT 22
#define USBEpDMASt_EP23_DMA_ENABLE_MASK 0x800000
#define USBEpDMASt_EP23_DMA_ENABLE 0x800000
#define USBEpDMASt_EP23_DMA_ENABLE_BIT 23
#define USBEpDMASt_EP24_DMA_ENABLE_MASK 0x1000000
#define USBEpDMASt_EP24_DMA_ENABLE 0x1000000
#define USBEpDMASt_EP24_DMA_ENABLE_BIT 24
#define USBEpDMASt_EP25_DMA_ENABLE_MASK 0x2000000
#define USBEpDMASt_EP25_DMA_ENABLE 0x2000000
#define USBEpDMASt_EP25_DMA_ENABLE_BIT 25
#define USBEpDMASt_EP26_DMA_ENABLE_MASK 0x4000000
#define USBEpDMASt_EP26_DMA_ENABLE 0x4000000
#define USBEpDMASt_EP26_DMA_ENABLE_BIT 26
#define USBEpDMASt_EP27_DMA_ENABLE_MASK 0x8000000
#define USBEpDMASt_EP27_DMA_ENABLE 0x8000000
#define USBEpDMASt_EP27_DMA_ENABLE_BIT 27
#define USBEpDMASt_EP28_DMA_ENABLE_MASK 0x10000000
#define USBEpDMASt_EP28_DMA_ENABLE 0x10000000
#define USBEpDMASt_EP28_DMA_ENABLE_BIT 28
#define USBEpDMASt_EP29_DMA_ENABLE_MASK 0x20000000
#define USBEpDMASt_EP29_DMA_ENABLE 0x20000000
#define USBEpDMASt_EP29_DMA_ENABLE_BIT 29
#define USBEpDMASt_EP30_DMA_ENABLE_MASK 0x40000000
#define USBEpDMASt_EP30_DMA_ENABLE 0x40000000
#define USBEpDMASt_EP30_DMA_ENABLE_BIT 30
#define USBEpDMASt_EP31_DMA_ENABLE_MASK 0x80000000
#define USBEpDMASt_EP31_DMA_ENABLE 0x80000000
#define USBEpDMASt_EP31_DMA_ENABLE_BIT 31

#define USBEpDMAEn (*(volatile uint32_t *)0x2008C288)
#define USBEpDMAEn_OFFSET 0x288
#define USBEpDMAEn_EP0_DMA_ENABLE_MASK 0x1
#define USBEpDMAEn_EP0_DMA_ENABLE 0x1
#define USBEpDMAEn_EP0_DMA_ENABLE_BIT 0
#define USBEpDMAEn_EP1_DMA_ENABLE_MASK 0x2
#define USBEpDMAEn_EP1_DMA_ENABLE 0x2
#define USBEpDMAEn_EP1_DMA_ENABLE_BIT 1
#define USBEpDMAEn_EP2_DMA_ENABLE_MASK 0x4
#define USBEpDMAEn_EP2_DMA_ENABLE 0x4
#define USBEpDMAEn_EP2_DMA_ENABLE_BIT 2
#define USBEpDMAEn_EP3_DMA_ENABLE_MASK 0x8
#define USBEpDMAEn_EP3_DMA_ENABLE 0x8
#define USBEpDMAEn_EP3_DMA_ENABLE_BIT 3
#define USBEpDMAEn_EP4_DMA_ENABLE_MASK 0x10
#define USBEpDMAEn_EP4_DMA_ENABLE 0x10
#define USBEpDMAEn_EP4_DMA_ENABLE_BIT 4
#define USBEpDMAEn_EP5_DMA_ENABLE_MASK 0x20
#define USBEpDMAEn_EP5_DMA_ENABLE 0x20
#define USBEpDMAEn_EP5_DMA_ENABLE_BIT 5
#define USBEpDMAEn_EP6_DMA_ENABLE_MASK 0x40
#define USBEpDMAEn_EP6_DMA_ENABLE 0x40
#define USBEpDMAEn_EP6_DMA_ENABLE_BIT 6
#define USBEpDMAEn_EP7_DMA_ENABLE_MASK 0x80
#define USBEpDMAEn_EP7_DMA_ENABLE 0x80
#define USBEpDMAEn_EP7_DMA_ENABLE_BIT 7
#define USBEpDMAEn_EP8_DMA_ENABLE_MASK 0x100
#define USBEpDMAEn_EP8_DMA_ENABLE 0x100
#define USBEpDMAEn_EP8_DMA_ENABLE_BIT 8
#define USBEpDMAEn_EP9_DMA_ENABLE_MASK 0x200
#define USBEpDMAEn_EP9_DMA_ENABLE 0x200
#define USBEpDMAEn_EP9_DMA_ENABLE_BIT 9
#define USBEpDMAEn_EP10_DMA_ENABLE_MASK 0x400
#define USBEpDMAEn_EP10_DMA_ENABLE 0x400
#define USBEpDMAEn_EP10_DMA_ENABLE_BIT 10
#define USBEpDMAEn_EP11_DMA_ENABLE_MASK 0x800
#define USBEpDMAEn_EP11_DMA_ENABLE 0x800
#define USBEpDMAEn_EP11_DMA_ENABLE_BIT 11
#define USBEpDMAEn_EP12_DMA_ENABLE_MASK 0x1000
#define USBEpDMAEn_EP12_DMA_ENABLE 0x1000
#define USBEpDMAEn_EP12_DMA_ENABLE_BIT 12
#define USBEpDMAEn_EP13_DMA_ENABLE_MASK 0x2000
#define USBEpDMAEn_EP13_DMA_ENABLE 0x2000
#define USBEpDMAEn_EP13_DMA_ENABLE_BIT 13
#define USBEpDMAEn_EP14_DMA_ENABLE_MASK 0x4000
#define USBEpDMAEn_EP14_DMA_ENABLE 0x4000
#define USBEpDMAEn_EP14_DMA_ENABLE_BIT 14
#define USBEpDMAEn_EP15_DMA_ENABLE_MASK 0x8000
#define USBEpDMAEn_EP15_DMA_ENABLE 0x8000
#define USBEpDMAEn_EP15_DMA_ENABLE_BIT 15
#define USBEpDMAEn_EP16_DMA_ENABLE_MASK 0x10000
#define USBEpDMAEn_EP16_DMA_ENABLE 0x10000
#define USBEpDMAEn_EP16_DMA_ENABLE_BIT 16
#define USBEpDMAEn_EP17_DMA_ENABLE_MASK 0x20000
#define USBEpDMAEn_EP17_DMA_ENABLE 0x20000
#define USBEpDMAEn_EP17_DMA_ENABLE_BIT 17
#define USBEpDMAEn_EP18_DMA_ENABLE_MASK 0x40000
#define USBEpDMAEn_EP18_DMA_ENABLE 0x40000
#define USBEpDMAEn_EP18_DMA_ENABLE_BIT 18
#define USBEpDMAEn_EP19_DMA_ENABLE_MASK 0x80000
#define USBEpDMAEn_EP19_DMA_ENABLE 0x80000
#define USBEpDMAEn_EP19_DMA_ENABLE_BIT 19
#define USBEpDMAEn_EP20_DMA_ENABLE_MASK 0x100000
#define USBEpDMAEn_EP20_DMA_ENABLE 0x100000
#define USBEpDMAEn_EP20_DMA_ENABLE_BIT 20
#define USBEpDMAEn_EP21_DMA_ENABLE_MASK 0x200000
#define USBEpDMAEn_EP21_DMA_ENABLE 0x200000
#define USBEpDMAEn_EP21_DMA_ENABLE_BIT 21
#define USBEpDMAEn_EP22_DMA_ENABLE_MASK 0x400000
#define USBEpDMAEn_EP22_DMA_ENABLE 0x400000
#define USBEpDMAEn_EP22_DMA_ENABLE_BIT 22
#define USBEpDMAEn_EP23_DMA_ENABLE_MASK 0x800000
#define USBEpDMAEn_EP23_DMA_ENABLE 0x800000
#define USBEpDMAEn_EP23_DMA_ENABLE_BIT 23
#define USBEpDMAEn_EP24_DMA_ENABLE_MASK 0x1000000
#define USBEpDMAEn_EP24_DMA_ENABLE 0x1000000
#define USBEpDMAEn_EP24_DMA_ENABLE_BIT 24
#define USBEpDMAEn_EP25_DMA_ENABLE_MASK 0x2000000
#define USBEpDMAEn_EP25_DMA_ENABLE 0x2000000
#define USBEpDMAEn_EP25_DMA_ENABLE_BIT 25
#define USBEpDMAEn_EP26_DMA_ENABLE_MASK 0x4000000
#define USBEpDMAEn_EP26_DMA_ENABLE 0x4000000
#define USBEpDMAEn_EP26_DMA_ENABLE_BIT 26
#define USBEpDMAEn_EP27_DMA_ENABLE_MASK 0x8000000
#define USBEpDMAEn_EP27_DMA_ENABLE 0x8000000
#define USBEpDMAEn_EP27_DMA_ENABLE_BIT 27
#define USBEpDMAEn_EP28_DMA_ENABLE_MASK 0x10000000
#define USBEpDMAEn_EP28_DMA_ENABLE 0x10000000
#define USBEpDMAEn_EP28_DMA_ENABLE_BIT 28
#define USBEpDMAEn_EP29_DMA_ENABLE_MASK 0x20000000
#define USBEpDMAEn_EP29_DMA_ENABLE 0x20000000
#define USBEpDMAEn_EP29_DMA_ENABLE_BIT 29
#define USBEpDMAEn_EP30_DMA_ENABLE_MASK 0x40000000
#define USBEpDMAEn_EP30_DMA_ENABLE 0x40000000
#define USBEpDMAEn_EP30_DMA_ENABLE_BIT 30
#define USBEpDMAEn_EP31_DMA_ENABLE_MASK 0x80000000
#define USBEpDMAEn_EP31_DMA_ENABLE 0x80000000
#define USBEpDMAEn_EP31_DMA_ENABLE_BIT 31

#define USBEpDMADis (*(volatile uint32_t *)0x2008C28C)
#define USBEpDMADis_OFFSET 0x28C
#define USBEpDMADis_EP0_DMA_ENABLE_MASK 0x1
#define USBEpDMADis_EP0_DMA_ENABLE 0x1
#define USBEpDMADis_EP0_DMA_ENABLE_BIT 0
#define USBEpDMADis_EP1_DMA_ENABLE_MASK 0x2
#define USBEpDMADis_EP1_DMA_ENABLE 0x2
#define USBEpDMADis_EP1_DMA_ENABLE_BIT 1
#define USBEpDMADis_EP2_DMA_ENABLE_MASK 0x4
#define USBEpDMADis_EP2_DMA_ENABLE 0x4
#define USBEpDMADis_EP2_DMA_ENABLE_BIT 2
#define USBEpDMADis_EP3_DMA_ENABLE_MASK 0x8
#define USBEpDMADis_EP3_DMA_ENABLE 0x8
#define USBEpDMADis_EP3_DMA_ENABLE_BIT 3
#define USBEpDMADis_EP4_DMA_ENABLE_MASK 0x10
#define USBEpDMADis_EP4_DMA_ENABLE 0x10
#define USBEpDMADis_EP4_DMA_ENABLE_BIT 4
#define USBEpDMADis_EP5_DMA_ENABLE_MASK 0x20
#define USBEpDMADis_EP5_DMA_ENABLE 0x20
#define USBEpDMADis_EP5_DMA_ENABLE_BIT 5
#define USBEpDMADis_EP6_DMA_ENABLE_MASK 0x40
#define USBEpDMADis_EP6_DMA_ENABLE 0x40
#define USBEpDMADis_EP6_DMA_ENABLE_BIT 6
#define USBEpDMADis_EP7_DMA_ENABLE_MASK 0x80
#define USBEpDMADis_EP7_DMA_ENABLE 0x80
#define USBEpDMADis_EP7_DMA_ENABLE_BIT 7
#define USBEpDMADis_EP8_DMA_ENABLE_MASK 0x100
#define USBEpDMADis_EP8_DMA_ENABLE 0x100
#define USBEpDMADis_EP8_DMA_ENABLE_BIT 8
#define USBEpDMADis_EP9_DMA_ENABLE_MASK 0x200
#define USBEpDMADis_EP9_DMA_ENABLE 0x200
#define USBEpDMADis_EP9_DMA_ENABLE_BIT 9
#define USBEpDMADis_EP10_DMA_ENABLE_MASK 0x400
#define USBEpDMADis_EP10_DMA_ENABLE 0x400
#define USBEpDMADis_EP10_DMA_ENABLE_BIT 10
#define USBEpDMADis_EP11_DMA_ENABLE_MASK 0x800
#define USBEpDMADis_EP11_DMA_ENABLE 0x800
#define USBEpDMADis_EP11_DMA_ENABLE_BIT 11
#define USBEpDMADis_EP12_DMA_ENABLE_MASK 0x1000
#define USBEpDMADis_EP12_DMA_ENABLE 0x1000
#define USBEpDMADis_EP12_DMA_ENABLE_BIT 12
#define USBEpDMADis_EP13_DMA_ENABLE_MASK 0x2000
#define USBEpDMADis_EP13_DMA_ENABLE 0x2000
#define USBEpDMADis_EP13_DMA_ENABLE_BIT 13
#define USBEpDMADis_EP14_DMA_ENABLE_MASK 0x4000
#define USBEpDMADis_EP14_DMA_ENABLE 0x4000
#define USBEpDMADis_EP14_DMA_ENABLE_BIT 14
#define USBEpDMADis_EP15_DMA_ENABLE_MASK 0x8000
#define USBEpDMADis_EP15_DMA_ENABLE 0x8000
#define USBEpDMADis_EP15_DMA_ENABLE_BIT 15
#define USBEpDMADis_EP16_DMA_ENABLE_MASK 0x10000
#define USBEpDMADis_EP16_DMA_ENABLE 0x10000
#define USBEpDMADis_EP16_DMA_ENABLE_BIT 16
#define USBEpDMADis_EP17_DMA_ENABLE_MASK 0x20000
#define USBEpDMADis_EP17_DMA_ENABLE 0x20000
#define USBEpDMADis_EP17_DMA_ENABLE_BIT 17
#define USBEpDMADis_EP18_DMA_ENABLE_MASK 0x40000
#define USBEpDMADis_EP18_DMA_ENABLE 0x40000
#define USBEpDMADis_EP18_DMA_ENABLE_BIT 18
#define USBEpDMADis_EP19_DMA_ENABLE_MASK 0x80000
#define USBEpDMADis_EP19_DMA_ENABLE 0x80000
#define USBEpDMADis_EP19_DMA_ENABLE_BIT 19
#define USBEpDMADis_EP20_DMA_ENABLE_MASK 0x100000
#define USBEpDMADis_EP20_DMA_ENABLE 0x100000
#define USBEpDMADis_EP20_DMA_ENABLE_BIT 20
#define USBEpDMADis_EP21_DMA_ENABLE_MASK 0x200000
#define USBEpDMADis_EP21_DMA_ENABLE 0x200000
#define USBEpDMADis_EP21_DMA_ENABLE_BIT 21
#define USBEpDMADis_EP22_DMA_ENABLE_MASK 0x400000
#define USBEpDMADis_EP22_DMA_ENABLE 0x400000
#define USBEpDMADis_EP22_DMA_ENABLE_BIT 22
#define USBEpDMADis_EP23_DMA_ENABLE_MASK 0x800000
#define USBEpDMADis_EP23_DMA_ENABLE 0x800000
#define USBEpDMADis_EP23_DMA_ENABLE_BIT 23
#define USBEpDMADis_EP24_DMA_ENABLE_MASK 0x1000000
#define USBEpDMADis_EP24_DMA_ENABLE 0x1000000
#define USBEpDMADis_EP24_DMA_ENABLE_BIT 24
#define USBEpDMADis_EP25_DMA_ENABLE_MASK 0x2000000
#define USBEpDMADis_EP25_DMA_ENABLE 0x2000000
#define USBEpDMADis_EP25_DMA_ENABLE_BIT 25
#define USBEpDMADis_EP26_DMA_ENABLE_MASK 0x4000000
#define USBEpDMADis_EP26_DMA_ENABLE 0x4000000
#define USBEpDMADis_EP26_DMA_ENABLE_BIT 26
#define USBEpDMADis_EP27_DMA_ENABLE_MASK 0x8000000
#define USBEpDMADis_EP27_DMA_ENABLE 0x8000000
#define USBEpDMADis_EP27_DMA_ENABLE_BIT 27
#define USBEpDMADis_EP28_DMA_ENABLE_MASK 0x10000000
#define USBEpDMADis_EP28_DMA_ENABLE 0x10000000
#define USBEpDMADis_EP28_DMA_ENABLE_BIT 28
#define USBEpDMADis_EP29_DMA_ENABLE_MASK 0x20000000
#define USBEpDMADis_EP29_DMA_ENABLE 0x20000000
#define USBEpDMADis_EP29_DMA_ENABLE_BIT 29
#define USBEpDMADis_EP30_DMA_ENABLE_MASK 0x40000000
#define USBEpDMADis_EP30_DMA_ENABLE 0x40000000
#define USBEpDMADis_EP30_DMA_ENABLE_BIT 30
#define USBEpDMADis_EP31_DMA_ENABLE_MASK 0x80000000
#define USBEpDMADis_EP31_DMA_ENABLE 0x80000000
#define USBEpDMADis_EP31_DMA_ENABLE_BIT 31

#define USBDMAIntSt (*(volatile uint32_t *)0x2008C290)
#define USBDMAIntSt_OFFSET 0x290
#define USBDMAIntSt_End_of_Transfer_Interrupt_MASK 0x1
#define USBDMAIntSt_End_of_Transfer_Interrupt 0x1
#define USBDMAIntSt_End_of_Transfer_Interrupt_BIT 0
#define USBDMAIntSt_New_DD_Request_Interrupt_MASK 0x2
#define USBDMAIntSt_New_DD_Request_Interrupt 0x2
#define USBDMAIntSt_New_DD_Request_Interrupt_BIT 1
#define USBDMAIntSt_System_Error_Interrupt_MASK 0x4
#define USBDMAIntSt_System_Error_Interrupt 0x4
#define USBDMAIntSt_System_Error_Interrupt_BIT 2

#define USBDMAIntEn (*(volatile uint32_t *)0x2008C294)
#define USBDMAIntEn_OFFSET 0x294
#define USBDMAIntEn_End_of_Transfer_Interrupt_MASK 0x1
#define USBDMAIntEn_End_of_Transfer_Interrupt 0x1
#define USBDMAIntEn_End_of_Transfer_Interrupt_BIT 0
#define USBDMAIntEn_New_DD_Request_Interrupt_MASK 0x2
#define USBDMAIntEn_New_DD_Request_Interrupt 0x2
#define USBDMAIntEn_New_DD_Request_Interrupt_BIT 1
#define USBDMAIntEn_System_Error_Interrupt_MASK 0x4
#define USBDMAIntEn_System_Error_Interrupt 0x4
#define USBDMAIntEn_System_Error_Interrupt_BIT 2

#define USBEoTIntSt (*(volatile uint32_t *)0x2008C2A0)
#define USBEoTIntSt_OFFSET 0x2A0
#define USBEoTIntSt_EP0_MASK 0x1
#define USBEoTIntSt_EP0 0x1
#define USBEoTIntSt_EP0_BIT 0
#define USBEoTIntSt_EP1_MASK 0x2
#define USBEoTIntSt_EP1 0x2
#define USBEoTIntSt_EP1_BIT 1
#define USBEoTIntSt_EP2_MASK 0x4
#define USBEoTIntSt_EP2 0x4
#define USBEoTIntSt_EP2_BIT 2
#define USBEoTIntSt_EP3_MASK 0x8
#define USBEoTIntSt_EP3 0x8
#define USBEoTIntSt_EP3_BIT 3
#define USBEoTIntSt_EP4_MASK 0x10
#define USBEoTIntSt_EP4 0x10
#define USBEoTIntSt_EP4_BIT 4
#define USBEoTIntSt_EP5_MASK 0x20
#define USBEoTIntSt_EP5 0x20
#define USBEoTIntSt_EP5_BIT 5
#define USBEoTIntSt_EP6_MASK 0x40
#define USBEoTIntSt_EP6 0x40
#define USBEoTIntSt_EP6_BIT 6
#define USBEoTIntSt_EP7_MASK 0x80
#define USBEoTIntSt_EP7 0x80
#define USBEoTIntSt_EP7_BIT 7
#define USBEoTIntSt_EP8_MASK 0x100
#define USBEoTIntSt_EP8 0x100
#define USBEoTIntSt_EP8_BIT 8
#define USBEoTIntSt_EP9_MASK 0x200
#define USBEoTIntSt_EP9 0x200
#define USBEoTIntSt_EP9_BIT 9
#define USBEoTIntSt_EP10_MASK 0x400
#define USBEoTIntSt_EP10 0x400
#define USBEoTIntSt_EP10_BIT 10
#define USBEoTIntSt_EP11_MASK 0x800
#define USBEoTIntSt_EP11 0x800
#define USBEoTIntSt_EP11_BIT 11
#define USBEoTIntSt_EP12_MASK 0x1000
#define USBEoTIntSt_EP12 0x1000
#define USBEoTIntSt_EP12_BIT 12
#define USBEoTIntSt_EP13_MASK 0x2000
#define USBEoTIntSt_EP13 0x2000
#define USBEoTIntSt_EP13_BIT 13
#define USBEoTIntSt_EP14_MASK 0x4000
#define USBEoTIntSt_EP14 0x4000
#define USBEoTIntSt_EP14_BIT 14
#define USBEoTIntSt_EP15_MASK 0x8000
#define USBEoTIntSt_EP15 0x8000
#define USBEoTIntSt_EP15_BIT 15
#define USBEoTIntSt_EP16_MASK 0x10000
#define USBEoTIntSt_EP16 0x10000
#define USBEoTIntSt_EP16_BIT 16
#define USBEoTIntSt_EP17_MASK 0x20000
#define USBEoTIntSt_EP17 0x20000
#define USBEoTIntSt_EP17_BIT 17
#define USBEoTIntSt_EP18_MASK 0x40000
#define USBEoTIntSt_EP18 0x40000
#define USBEoTIntSt_EP18_BIT 18
#define USBEoTIntSt_EP19_MASK 0x80000
#define USBEoTIntSt_EP19 0x80000
#define USBEoTIntSt_EP19_BIT 19
#define USBEoTIntSt_EP20_MASK 0x100000
#define USBEoTIntSt_EP20 0x100000
#define USBEoTIntSt_EP20_BIT 20
#define USBEoTIntSt_EP21_MASK 0x200000
#define USBEoTIntSt_EP21 0x200000
#define USBEoTIntSt_EP21_BIT 21
#define USBEoTIntSt_EP22_MASK 0x400000
#define USBEoTIntSt_EP22 0x400000
#define USBEoTIntSt_EP22_BIT 22
#define USBEoTIntSt_EP23_MASK 0x800000
#define USBEoTIntSt_EP23 0x800000
#define USBEoTIntSt_EP23_BIT 23
#define USBEoTIntSt_EP24_MASK 0x1000000
#define USBEoTIntSt_EP24 0x1000000
#define USBEoTIntSt_EP24_BIT 24
#define USBEoTIntSt_EP25_MASK 0x2000000
#define USBEoTIntSt_EP25 0x2000000
#define USBEoTIntSt_EP25_BIT 25
#define USBEoTIntSt_EP26_MASK 0x4000000
#define USBEoTIntSt_EP26 0x4000000
#define USBEoTIntSt_EP26_BIT 26
#define USBEoTIntSt_EP27_MASK 0x8000000
#define USBEoTIntSt_EP27 0x8000000
#define USBEoTIntSt_EP27_BIT 27
#define USBEoTIntSt_EP28_MASK 0x10000000
#define USBEoTIntSt_EP28 0x10000000
#define USBEoTIntSt_EP28_BIT 28
#define USBEoTIntSt_EP29_MASK 0x20000000
#define USBEoTIntSt_EP29 0x20000000
#define USBEoTIntSt_EP29_BIT 29
#define USBEoTIntSt_EP30_MASK 0x40000000
#define USBEoTIntSt_EP30 0x40000000
#define USBEoTIntSt_EP30_BIT 30
#define USBEoTIntSt_EP31_MASK 0x80000000
#define USBEoTIntSt_EP31 0x80000000
#define USBEoTIntSt_EP31_BIT 31

#define USBEoTIntClr (*(volatile uint32_t *)0x2008C2A4)
#define USBEoTIntClr_OFFSET 0x2A4
#define USBEoTIntClr_EP0_MASK 0x1
#define USBEoTIntClr_EP0 0x1
#define USBEoTIntClr_EP0_BIT 0
#define USBEoTIntClr_EP1_MASK 0x2
#define USBEoTIntClr_EP1 0x2
#define USBEoTIntClr_EP1_BIT 1
#define USBEoTIntClr_EP2_MASK 0x4
#define USBEoTIntClr_EP2 0x4
#define USBEoTIntClr_EP2_BIT 2
#define USBEoTIntClr_EP3_MASK 0x8
#define USBEoTIntClr_EP3 0x8
#define USBEoTIntClr_EP3_BIT 3
#define USBEoTIntClr_EP4_MASK 0x10
#define USBEoTIntClr_EP4 0x10
#define USBEoTIntClr_EP4_BIT 4
#define USBEoTIntClr_EP5_MASK 0x20
#define USBEoTIntClr_EP5 0x20
#define USBEoTIntClr_EP5_BIT 5
#define USBEoTIntClr_EP6_MASK 0x40
#define USBEoTIntClr_EP6 0x40
#define USBEoTIntClr_EP6_BIT 6
#define USBEoTIntClr_EP7_MASK 0x80
#define USBEoTIntClr_EP7 0x80
#define USBEoTIntClr_EP7_BIT 7
#define USBEoTIntClr_EP8_MASK 0x100
#define USBEoTIntClr_EP8 0x100
#define USBEoTIntClr_EP8_BIT 8
#define USBEoTIntClr_EP9_MASK 0x200
#define USBEoTIntClr_EP9 0x200
#define USBEoTIntClr_EP9_BIT 9
#define USBEoTIntClr_EP10_MASK 0x400
#define USBEoTIntClr_EP10 0x400
#define USBEoTIntClr_EP10_BIT 10
#define USBEoTIntClr_EP11_MASK 0x800
#define USBEoTIntClr_EP11 0x800
#define USBEoTIntClr_EP11_BIT 11
#define USBEoTIntClr_EP12_MASK 0x1000
#define USBEoTIntClr_EP12 0x1000
#define USBEoTIntClr_EP12_BIT 12
#define USBEoTIntClr_EP13_MASK 0x2000
#define USBEoTIntClr_EP13 0x2000
#define USBEoTIntClr_EP13_BIT 13
#define USBEoTIntClr_EP14_MASK 0x4000
#define USBEoTIntClr_EP14 0x4000
#define USBEoTIntClr_EP14_BIT 14
#define USBEoTIntClr_EP15_MASK 0x8000
#define USBEoTIntClr_EP15 0x8000
#define USBEoTIntClr_EP15_BIT 15
#define USBEoTIntClr_EP16_MASK 0x10000
#define USBEoTIntClr_EP16 0x10000
#define USBEoTIntClr_EP16_BIT 16
#define USBEoTIntClr_EP17_MASK 0x20000
#define USBEoTIntClr_EP17 0x20000
#define USBEoTIntClr_EP17_BIT 17
#define USBEoTIntClr_EP18_MASK 0x40000
#define USBEoTIntClr_EP18 0x40000
#define USBEoTIntClr_EP18_BIT 18
#define USBEoTIntClr_EP19_MASK 0x80000
#define USBEoTIntClr_EP19 0x80000
#define USBEoTIntClr_EP19_BIT 19
#define USBEoTIntClr_EP20_MASK 0x100000
#define USBEoTIntClr_EP20 0x100000
#define USBEoTIntClr_EP20_BIT 20
#define USBEoTIntClr_EP21_MASK 0x200000
#define USBEoTIntClr_EP21 0x200000
#define USBEoTIntClr_EP21_BIT 21
#define USBEoTIntClr_EP22_MASK 0x400000
#define USBEoTIntClr_EP22 0x400000
#define USBEoTIntClr_EP22_BIT 22
#define USBEoTIntClr_EP23_MASK 0x800000
#define USBEoTIntClr_EP23 0x800000
#define USBEoTIntClr_EP23_BIT 23
#define USBEoTIntClr_EP24_MASK 0x1000000
#define USBEoTIntClr_EP24 0x1000000
#define USBEoTIntClr_EP24_BIT 24
#define USBEoTIntClr_EP25_MASK 0x2000000
#define USBEoTIntClr_EP25 0x2000000
#define USBEoTIntClr_EP25_BIT 25
#define USBEoTIntClr_EP26_MASK 0x4000000
#define USBEoTIntClr_EP26 0x4000000
#define USBEoTIntClr_EP26_BIT 26
#define USBEoTIntClr_EP27_MASK 0x8000000
#define USBEoTIntClr_EP27 0x8000000
#define USBEoTIntClr_EP27_BIT 27
#define USBEoTIntClr_EP28_MASK 0x10000000
#define USBEoTIntClr_EP28 0x10000000
#define USBEoTIntClr_EP28_BIT 28
#define USBEoTIntClr_EP29_MASK 0x20000000
#define USBEoTIntClr_EP29 0x20000000
#define USBEoTIntClr_EP29_BIT 29
#define USBEoTIntClr_EP30_MASK 0x40000000
#define USBEoTIntClr_EP30 0x40000000
#define USBEoTIntClr_EP30_BIT 30
#define USBEoTIntClr_EP31_MASK 0x80000000
#define USBEoTIntClr_EP31 0x80000000
#define USBEoTIntClr_EP31_BIT 31

#define USBEoTIntSet (*(volatile uint32_t *)0x2008C2A8)
#define USBEoTIntSet_OFFSET 0x2A8
#define USBEoTIntSet_EP0_MASK 0x1
#define USBEoTIntSet_EP0 0x1
#define USBEoTIntSet_EP0_BIT 0
#define USBEoTIntSet_EP1_MASK 0x2
#define USBEoTIntSet_EP1 0x2
#define USBEoTIntSet_EP1_BIT 1
#define USBEoTIntSet_EP2_MASK 0x4
#define USBEoTIntSet_EP2 0x4
#define USBEoTIntSet_EP2_BIT 2
#define USBEoTIntSet_EP3_MASK 0x8
#define USBEoTIntSet_EP3 0x8
#define USBEoTIntSet_EP3_BIT 3
#define USBEoTIntSet_EP4_MASK 0x10
#define USBEoTIntSet_EP4 0x10
#define USBEoTIntSet_EP4_BIT 4
#define USBEoTIntSet_EP5_MASK 0x20
#define USBEoTIntSet_EP5 0x20
#define USBEoTIntSet_EP5_BIT 5
#define USBEoTIntSet_EP6_MASK 0x40
#define USBEoTIntSet_EP6 0x40
#define USBEoTIntSet_EP6_BIT 6
#define USBEoTIntSet_EP7_MASK 0x80
#define USBEoTIntSet_EP7 0x80
#define USBEoTIntSet_EP7_BIT 7
#define USBEoTIntSet_EP8_MASK 0x100
#define USBEoTIntSet_EP8 0x100
#define USBEoTIntSet_EP8_BIT 8
#define USBEoTIntSet_EP9_MASK 0x200
#define USBEoTIntSet_EP9 0x200
#define USBEoTIntSet_EP9_BIT 9
#define USBEoTIntSet_EP10_MASK 0x400
#define USBEoTIntSet_EP10 0x400
#define USBEoTIntSet_EP10_BIT 10
#define USBEoTIntSet_EP11_MASK 0x800
#define USBEoTIntSet_EP11 0x800
#define USBEoTIntSet_EP11_BIT 11
#define USBEoTIntSet_EP12_MASK 0x1000
#define USBEoTIntSet_EP12 0x1000
#define USBEoTIntSet_EP12_BIT 12
#define USBEoTIntSet_EP13_MASK 0x2000
#define USBEoTIntSet_EP13 0x2000
#define USBEoTIntSet_EP13_BIT 13
#define USBEoTIntSet_EP14_MASK 0x4000
#define USBEoTIntSet_EP14 0x4000
#define USBEoTIntSet_EP14_BIT 14
#define USBEoTIntSet_EP15_MASK 0x8000
#define USBEoTIntSet_EP15 0x8000
#define USBEoTIntSet_EP15_BIT 15
#define USBEoTIntSet_EP16_MASK 0x10000
#define USBEoTIntSet_EP16 0x10000
#define USBEoTIntSet_EP16_BIT 16
#define USBEoTIntSet_EP17_MASK 0x20000
#define USBEoTIntSet_EP17 0x20000
#define USBEoTIntSet_EP17_BIT 17
#define USBEoTIntSet_EP18_MASK 0x40000
#define USBEoTIntSet_EP18 0x40000
#define USBEoTIntSet_EP18_BIT 18
#define USBEoTIntSet_EP19_MASK 0x80000
#define USBEoTIntSet_EP19 0x80000
#define USBEoTIntSet_EP19_BIT 19
#define USBEoTIntSet_EP20_MASK 0x100000
#define USBEoTIntSet_EP20 0x100000
#define USBEoTIntSet_EP20_BIT 20
#define USBEoTIntSet_EP21_MASK 0x200000
#define USBEoTIntSet_EP21 0x200000
#define USBEoTIntSet_EP21_BIT 21
#define USBEoTIntSet_EP22_MASK 0x400000
#define USBEoTIntSet_EP22 0x400000
#define USBEoTIntSet_EP22_BIT 22
#define USBEoTIntSet_EP23_MASK 0x800000
#define USBEoTIntSet_EP23 0x800000
#define USBEoTIntSet_EP23_BIT 23
#define USBEoTIntSet_EP24_MASK 0x1000000
#define USBEoTIntSet_EP24 0x1000000
#define USBEoTIntSet_EP24_BIT 24
#define USBEoTIntSet_EP25_MASK 0x2000000
#define USBEoTIntSet_EP25 0x2000000
#define USBEoTIntSet_EP25_BIT 25
#define USBEoTIntSet_EP26_MASK 0x4000000
#define USBEoTIntSet_EP26 0x4000000
#define USBEoTIntSet_EP26_BIT 26
#define USBEoTIntSet_EP27_MASK 0x8000000
#define USBEoTIntSet_EP27 0x8000000
#define USBEoTIntSet_EP27_BIT 27
#define USBEoTIntSet_EP28_MASK 0x10000000
#define USBEoTIntSet_EP28 0x10000000
#define USBEoTIntSet_EP28_BIT 28
#define USBEoTIntSet_EP29_MASK 0x20000000
#define USBEoTIntSet_EP29 0x20000000
#define USBEoTIntSet_EP29_BIT 29
#define USBEoTIntSet_EP30_MASK 0x40000000
#define USBEoTIntSet_EP30 0x40000000
#define USBEoTIntSet_EP30_BIT 30
#define USBEoTIntSet_EP31_MASK 0x80000000
#define USBEoTIntSet_EP31 0x80000000
#define USBEoTIntSet_EP31_BIT 31

#define USBNDDRIntSt (*(volatile uint32_t *)0x2008C2AC)
#define USBNDDRIntSt_OFFSET 0x2AC
#define USBNDDRIntSt_EP0_MASK 0x1
#define USBNDDRIntSt_EP0 0x1
#define USBNDDRIntSt_EP0_BIT 0
#define USBNDDRIntSt_EP1_MASK 0x2
#define USBNDDRIntSt_EP1 0x2
#define USBNDDRIntSt_EP1_BIT 1
#define USBNDDRIntSt_EP2_MASK 0x4
#define USBNDDRIntSt_EP2 0x4
#define USBNDDRIntSt_EP2_BIT 2
#define USBNDDRIntSt_EP3_MASK 0x8
#define USBNDDRIntSt_EP3 0x8
#define USBNDDRIntSt_EP3_BIT 3
#define USBNDDRIntSt_EP4_MASK 0x10
#define USBNDDRIntSt_EP4 0x10
#define USBNDDRIntSt_EP4_BIT 4
#define USBNDDRIntSt_EP5_MASK 0x20
#define USBNDDRIntSt_EP5 0x20
#define USBNDDRIntSt_EP5_BIT 5
#define USBNDDRIntSt_EP6_MASK 0x40
#define USBNDDRIntSt_EP6 0x40
#define USBNDDRIntSt_EP6_BIT 6
#define USBNDDRIntSt_EP7_MASK 0x80
#define USBNDDRIntSt_EP7 0x80
#define USBNDDRIntSt_EP7_BIT 7
#define USBNDDRIntSt_EP8_MASK 0x100
#define USBNDDRIntSt_EP8 0x100
#define USBNDDRIntSt_EP8_BIT 8
#define USBNDDRIntSt_EP9_MASK 0x200
#define USBNDDRIntSt_EP9 0x200
#define USBNDDRIntSt_EP9_BIT 9
#define USBNDDRIntSt_EP10_MASK 0x400
#define USBNDDRIntSt_EP10 0x400
#define USBNDDRIntSt_EP10_BIT 10
#define USBNDDRIntSt_EP11_MASK 0x800
#define USBNDDRIntSt_EP11 0x800
#define USBNDDRIntSt_EP11_BIT 11
#define USBNDDRIntSt_EP12_MASK 0x1000
#define USBNDDRIntSt_EP12 0x1000
#define USBNDDRIntSt_EP12_BIT 12
#define USBNDDRIntSt_EP13_MASK 0x2000
#define USBNDDRIntSt_EP13 0x2000
#define USBNDDRIntSt_EP13_BIT 13
#define USBNDDRIntSt_EP14_MASK 0x4000
#define USBNDDRIntSt_EP14 0x4000
#define USBNDDRIntSt_EP14_BIT 14
#define USBNDDRIntSt_EP15_MASK 0x8000
#define USBNDDRIntSt_EP15 0x8000
#define USBNDDRIntSt_EP15_BIT 15
#define USBNDDRIntSt_EP16_MASK 0x10000
#define USBNDDRIntSt_EP16 0x10000
#define USBNDDRIntSt_EP16_BIT 16
#define USBNDDRIntSt_EP17_MASK 0x20000
#define USBNDDRIntSt_EP17 0x20000
#define USBNDDRIntSt_EP17_BIT 17
#define USBNDDRIntSt_EP18_MASK 0x40000
#define USBNDDRIntSt_EP18 0x40000
#define USBNDDRIntSt_EP18_BIT 18
#define USBNDDRIntSt_EP19_MASK 0x80000
#define USBNDDRIntSt_EP19 0x80000
#define USBNDDRIntSt_EP19_BIT 19
#define USBNDDRIntSt_EP20_MASK 0x100000
#define USBNDDRIntSt_EP20 0x100000
#define USBNDDRIntSt_EP20_BIT 20
#define USBNDDRIntSt_EP21_MASK 0x200000
#define USBNDDRIntSt_EP21 0x200000
#define USBNDDRIntSt_EP21_BIT 21
#define USBNDDRIntSt_EP22_MASK 0x400000
#define USBNDDRIntSt_EP22 0x400000
#define USBNDDRIntSt_EP22_BIT 22
#define USBNDDRIntSt_EP23_MASK 0x800000
#define USBNDDRIntSt_EP23 0x800000
#define USBNDDRIntSt_EP23_BIT 23
#define USBNDDRIntSt_EP24_MASK 0x1000000
#define USBNDDRIntSt_EP24 0x1000000
#define USBNDDRIntSt_EP24_BIT 24
#define USBNDDRIntSt_EP25_MASK 0x2000000
#define USBNDDRIntSt_EP25 0x2000000
#define USBNDDRIntSt_EP25_BIT 25
#define USBNDDRIntSt_EP26_MASK 0x4000000
#define USBNDDRIntSt_EP26 0x4000000
#define USBNDDRIntSt_EP26_BIT 26
#define USBNDDRIntSt_EP27_MASK 0x8000000
#define USBNDDRIntSt_EP27 0x8000000
#define USBNDDRIntSt_EP27_BIT 27
#define USBNDDRIntSt_EP28_MASK 0x10000000
#define USBNDDRIntSt_EP28 0x10000000
#define USBNDDRIntSt_EP28_BIT 28
#define USBNDDRIntSt_EP29_MASK 0x20000000
#define USBNDDRIntSt_EP29 0x20000000
#define USBNDDRIntSt_EP29_BIT 29
#define USBNDDRIntSt_EP30_MASK 0x40000000
#define USBNDDRIntSt_EP30 0x40000000
#define USBNDDRIntSt_EP30_BIT 30
#define USBNDDRIntSt_EP31_MASK 0x80000000
#define USBNDDRIntSt_EP31 0x80000000
#define USBNDDRIntSt_EP31_BIT 31

#define USBNDDRIntClr (*(volatile uint32_t *)0x2008C2B0)
#define USBNDDRIntClr_OFFSET 0x2B0
#define USBNDDRIntClr_EP0_MASK 0x1
#define USBNDDRIntClr_EP0 0x1
#define USBNDDRIntClr_EP0_BIT 0
#define USBNDDRIntClr_EP1_MASK 0x2
#define USBNDDRIntClr_EP1 0x2
#define USBNDDRIntClr_EP1_BIT 1
#define USBNDDRIntClr_EP2_MASK 0x4
#define USBNDDRIntClr_EP2 0x4
#define USBNDDRIntClr_EP2_BIT 2
#define USBNDDRIntClr_EP3_MASK 0x8
#define USBNDDRIntClr_EP3 0x8
#define USBNDDRIntClr_EP3_BIT 3
#define USBNDDRIntClr_EP4_MASK 0x10
#define USBNDDRIntClr_EP4 0x10
#define USBNDDRIntClr_EP4_BIT 4
#define USBNDDRIntClr_EP5_MASK 0x20
#define USBNDDRIntClr_EP5 0x20
#define USBNDDRIntClr_EP5_BIT 5
#define USBNDDRIntClr_EP6_MASK 0x40
#define USBNDDRIntClr_EP6 0x40
#define USBNDDRIntClr_EP6_BIT 6
#define USBNDDRIntClr_EP7_MASK 0x80
#define USBNDDRIntClr_EP7 0x80
#define USBNDDRIntClr_EP7_BIT 7
#define USBNDDRIntClr_EP8_MASK 0x100
#define USBNDDRIntClr_EP8 0x100
#define USBNDDRIntClr_EP8_BIT 8
#define USBNDDRIntClr_EP9_MASK 0x200
#define USBNDDRIntClr_EP9 0x200
#define USBNDDRIntClr_EP9_BIT 9
#define USBNDDRIntClr_EP10_MASK 0x400
#define USBNDDRIntClr_EP10 0x400
#define USBNDDRIntClr_EP10_BIT 10
#define USBNDDRIntClr_EP11_MASK 0x800
#define USBNDDRIntClr_EP11 0x800
#define USBNDDRIntClr_EP11_BIT 11
#define USBNDDRIntClr_EP12_MASK 0x1000
#define USBNDDRIntClr_EP12 0x1000
#define USBNDDRIntClr_EP12_BIT 12
#define USBNDDRIntClr_EP13_MASK 0x2000
#define USBNDDRIntClr_EP13 0x2000
#define USBNDDRIntClr_EP13_BIT 13
#define USBNDDRIntClr_EP14_MASK 0x4000
#define USBNDDRIntClr_EP14 0x4000
#define USBNDDRIntClr_EP14_BIT 14
#define USBNDDRIntClr_EP15_MASK 0x8000
#define USBNDDRIntClr_EP15 0x8000
#define USBNDDRIntClr_EP15_BIT 15
#define USBNDDRIntClr_EP16_MASK 0x10000
#define USBNDDRIntClr_EP16 0x10000
#define USBNDDRIntClr_EP16_BIT 16
#define USBNDDRIntClr_EP17_MASK 0x20000
#define USBNDDRIntClr_EP17 0x20000
#define USBNDDRIntClr_EP17_BIT 17
#define USBNDDRIntClr_EP18_MASK 0x40000
#define USBNDDRIntClr_EP18 0x40000
#define USBNDDRIntClr_EP18_BIT 18
#define USBNDDRIntClr_EP19_MASK 0x80000
#define USBNDDRIntClr_EP19 0x80000
#define USBNDDRIntClr_EP19_BIT 19
#define USBNDDRIntClr_EP20_MASK 0x100000
#define USBNDDRIntClr_EP20 0x100000
#define USBNDDRIntClr_EP20_BIT 20
#define USBNDDRIntClr_EP21_MASK 0x200000
#define USBNDDRIntClr_EP21 0x200000
#define USBNDDRIntClr_EP21_BIT 21
#define USBNDDRIntClr_EP22_MASK 0x400000
#define USBNDDRIntClr_EP22 0x400000
#define USBNDDRIntClr_EP22_BIT 22
#define USBNDDRIntClr_EP23_MASK 0x800000
#define USBNDDRIntClr_EP23 0x800000
#define USBNDDRIntClr_EP23_BIT 23
#define USBNDDRIntClr_EP24_MASK 0x1000000
#define USBNDDRIntClr_EP24 0x1000000
#define USBNDDRIntClr_EP24_BIT 24
#define USBNDDRIntClr_EP25_MASK 0x2000000
#define USBNDDRIntClr_EP25 0x2000000
#define USBNDDRIntClr_EP25_BIT 25
#define USBNDDRIntClr_EP26_MASK 0x4000000
#define USBNDDRIntClr_EP26 0x4000000
#define USBNDDRIntClr_EP26_BIT 26
#define USBNDDRIntClr_EP27_MASK 0x8000000
#define USBNDDRIntClr_EP27 0x8000000
#define USBNDDRIntClr_EP27_BIT 27
#define USBNDDRIntClr_EP28_MASK 0x10000000
#define USBNDDRIntClr_EP28 0x10000000
#define USBNDDRIntClr_EP28_BIT 28
#define USBNDDRIntClr_EP29_MASK 0x20000000
#define USBNDDRIntClr_EP29 0x20000000
#define USBNDDRIntClr_EP29_BIT 29
#define USBNDDRIntClr_EP30_MASK 0x40000000
#define USBNDDRIntClr_EP30 0x40000000
#define USBNDDRIntClr_EP30_BIT 30
#define USBNDDRIntClr_EP31_MASK 0x80000000
#define USBNDDRIntClr_EP31 0x80000000
#define USBNDDRIntClr_EP31_BIT 31

#define USBNDDRIntSet (*(volatile uint32_t *)0x2008C2B4)
#define USBNDDRIntSet_OFFSET 0x2B4
#define USBNDDRIntSet_EP0_MASK 0x1
#define USBNDDRIntSet_EP0 0x1
#define USBNDDRIntSet_EP0_BIT 0
#define USBNDDRIntSet_EP1_MASK 0x2
#define USBNDDRIntSet_EP1 0x2
#define USBNDDRIntSet_EP1_BIT 1
#define USBNDDRIntSet_EP2_MASK 0x4
#define USBNDDRIntSet_EP2 0x4
#define USBNDDRIntSet_EP2_BIT 2
#define USBNDDRIntSet_EP3_MASK 0x8
#define USBNDDRIntSet_EP3 0x8
#define USBNDDRIntSet_EP3_BIT 3
#define USBNDDRIntSet_EP4_MASK 0x10
#define USBNDDRIntSet_EP4 0x10
#define USBNDDRIntSet_EP4_BIT 4
#define USBNDDRIntSet_EP5_MASK 0x20
#define USBNDDRIntSet_EP5 0x20
#define USBNDDRIntSet_EP5_BIT 5
#define USBNDDRIntSet_EP6_MASK 0x40
#define USBNDDRIntSet_EP6 0x40
#define USBNDDRIntSet_EP6_BIT 6
#define USBNDDRIntSet_EP7_MASK 0x80
#define USBNDDRIntSet_EP7 0x80
#define USBNDDRIntSet_EP7_BIT 7
#define USBNDDRIntSet_EP8_MASK 0x100
#define USBNDDRIntSet_EP8 0x100
#define USBNDDRIntSet_EP8_BIT 8
#define USBNDDRIntSet_EP9_MASK 0x200
#define USBNDDRIntSet_EP9 0x200
#define USBNDDRIntSet_EP9_BIT 9
#define USBNDDRIntSet_EP10_MASK 0x400
#define USBNDDRIntSet_EP10 0x400
#define USBNDDRIntSet_EP10_BIT 10
#define USBNDDRIntSet_EP11_MASK 0x800
#define USBNDDRIntSet_EP11 0x800
#define USBNDDRIntSet_EP11_BIT 11
#define USBNDDRIntSet_EP12_MASK 0x1000
#define USBNDDRIntSet_EP12 0x1000
#define USBNDDRIntSet_EP12_BIT 12
#define USBNDDRIntSet_EP13_MASK 0x2000
#define USBNDDRIntSet_EP13 0x2000
#define USBNDDRIntSet_EP13_BIT 13
#define USBNDDRIntSet_EP14_MASK 0x4000
#define USBNDDRIntSet_EP14 0x4000
#define USBNDDRIntSet_EP14_BIT 14
#define USBNDDRIntSet_EP15_MASK 0x8000
#define USBNDDRIntSet_EP15 0x8000
#define USBNDDRIntSet_EP15_BIT 15
#define USBNDDRIntSet_EP16_MASK 0x10000
#define USBNDDRIntSet_EP16 0x10000
#define USBNDDRIntSet_EP16_BIT 16
#define USBNDDRIntSet_EP17_MASK 0x20000
#define USBNDDRIntSet_EP17 0x20000
#define USBNDDRIntSet_EP17_BIT 17
#define USBNDDRIntSet_EP18_MASK 0x40000
#define USBNDDRIntSet_EP18 0x40000
#define USBNDDRIntSet_EP18_BIT 18
#define USBNDDRIntSet_EP19_MASK 0x80000
#define USBNDDRIntSet_EP19 0x80000
#define USBNDDRIntSet_EP19_BIT 19
#define USBNDDRIntSet_EP20_MASK 0x100000
#define USBNDDRIntSet_EP20 0x100000
#define USBNDDRIntSet_EP20_BIT 20
#define USBNDDRIntSet_EP21_MASK 0x200000
#define USBNDDRIntSet_EP21 0x200000
#define USBNDDRIntSet_EP21_BIT 21
#define USBNDDRIntSet_EP22_MASK 0x400000
#define USBNDDRIntSet_EP22 0x400000
#define USBNDDRIntSet_EP22_BIT 22
#define USBNDDRIntSet_EP23_MASK 0x800000
#define USBNDDRIntSet_EP23 0x800000
#define USBNDDRIntSet_EP23_BIT 23
#define USBNDDRIntSet_EP24_MASK 0x1000000
#define USBNDDRIntSet_EP24 0x1000000
#define USBNDDRIntSet_EP24_BIT 24
#define USBNDDRIntSet_EP25_MASK 0x2000000
#define USBNDDRIntSet_EP25 0x2000000
#define USBNDDRIntSet_EP25_BIT 25
#define USBNDDRIntSet_EP26_MASK 0x4000000
#define USBNDDRIntSet_EP26 0x4000000
#define USBNDDRIntSet_EP26_BIT 26
#define USBNDDRIntSet_EP27_MASK 0x8000000
#define USBNDDRIntSet_EP27 0x8000000
#define USBNDDRIntSet_EP27_BIT 27
#define USBNDDRIntSet_EP28_MASK 0x10000000
#define USBNDDRIntSet_EP28 0x10000000
#define USBNDDRIntSet_EP28_BIT 28
#define USBNDDRIntSet_EP29_MASK 0x20000000
#define USBNDDRIntSet_EP29 0x20000000
#define USBNDDRIntSet_EP29_BIT 29
#define USBNDDRIntSet_EP30_MASK 0x40000000
#define USBNDDRIntSet_EP30 0x40000000
#define USBNDDRIntSet_EP30_BIT 30
#define USBNDDRIntSet_EP31_MASK 0x80000000
#define USBNDDRIntSet_EP31 0x80000000
#define USBNDDRIntSet_EP31_BIT 31

#define USBSysErrIntSt (*(volatile uint32_t *)0x2008C2B8)
#define USBSysErrIntSt_OFFSET 0x2B8
#define USBSysErrIntSt_EP0_MASK 0x1
#define USBSysErrIntSt_EP0 0x1
#define USBSysErrIntSt_EP0_BIT 0
#define USBSysErrIntSt_EP1_MASK 0x2
#define USBSysErrIntSt_EP1 0x2
#define USBSysErrIntSt_EP1_BIT 1
#define USBSysErrIntSt_EP2_MASK 0x4
#define USBSysErrIntSt_EP2 0x4
#define USBSysErrIntSt_EP2_BIT 2
#define USBSysErrIntSt_EP3_MASK 0x8
#define USBSysErrIntSt_EP3 0x8
#define USBSysErrIntSt_EP3_BIT 3
#define USBSysErrIntSt_EP4_MASK 0x10
#define USBSysErrIntSt_EP4 0x10
#define USBSysErrIntSt_EP4_BIT 4
#define USBSysErrIntSt_EP5_MASK 0x20
#define USBSysErrIntSt_EP5 0x20
#define USBSysErrIntSt_EP5_BIT 5
#define USBSysErrIntSt_EP6_MASK 0x40
#define USBSysErrIntSt_EP6 0x40
#define USBSysErrIntSt_EP6_BIT 6
#define USBSysErrIntSt_EP7_MASK 0x80
#define USBSysErrIntSt_EP7 0x80
#define USBSysErrIntSt_EP7_BIT 7
#define USBSysErrIntSt_EP8_MASK 0x100
#define USBSysErrIntSt_EP8 0x100
#define USBSysErrIntSt_EP8_BIT 8
#define USBSysErrIntSt_EP9_MASK 0x200
#define USBSysErrIntSt_EP9 0x200
#define USBSysErrIntSt_EP9_BIT 9
#define USBSysErrIntSt_EP10_MASK 0x400
#define USBSysErrIntSt_EP10 0x400
#define USBSysErrIntSt_EP10_BIT 10
#define USBSysErrIntSt_EP11_MASK 0x800
#define USBSysErrIntSt_EP11 0x800
#define USBSysErrIntSt_EP11_BIT 11
#define USBSysErrIntSt_EP12_MASK 0x1000
#define USBSysErrIntSt_EP12 0x1000
#define USBSysErrIntSt_EP12_BIT 12
#define USBSysErrIntSt_EP13_MASK 0x2000
#define USBSysErrIntSt_EP13 0x2000
#define USBSysErrIntSt_EP13_BIT 13
#define USBSysErrIntSt_EP14_MASK 0x4000
#define USBSysErrIntSt_EP14 0x4000
#define USBSysErrIntSt_EP14_BIT 14
#define USBSysErrIntSt_EP15_MASK 0x8000
#define USBSysErrIntSt_EP15 0x8000
#define USBSysErrIntSt_EP15_BIT 15
#define USBSysErrIntSt_EP16_MASK 0x10000
#define USBSysErrIntSt_EP16 0x10000
#define USBSysErrIntSt_EP16_BIT 16
#define USBSysErrIntSt_EP17_MASK 0x20000
#define USBSysErrIntSt_EP17 0x20000
#define USBSysErrIntSt_EP17_BIT 17
#define USBSysErrIntSt_EP18_MASK 0x40000
#define USBSysErrIntSt_EP18 0x40000
#define USBSysErrIntSt_EP18_BIT 18
#define USBSysErrIntSt_EP19_MASK 0x80000
#define USBSysErrIntSt_EP19 0x80000
#define USBSysErrIntSt_EP19_BIT 19
#define USBSysErrIntSt_EP20_MASK 0x100000
#define USBSysErrIntSt_EP20 0x100000
#define USBSysErrIntSt_EP20_BIT 20
#define USBSysErrIntSt_EP21_MASK 0x200000
#define USBSysErrIntSt_EP21 0x200000
#define USBSysErrIntSt_EP21_BIT 21
#define USBSysErrIntSt_EP22_MASK 0x400000
#define USBSysErrIntSt_EP22 0x400000
#define USBSysErrIntSt_EP22_BIT 22
#define USBSysErrIntSt_EP23_MASK 0x800000
#define USBSysErrIntSt_EP23 0x800000
#define USBSysErrIntSt_EP23_BIT 23
#define USBSysErrIntSt_EP24_MASK 0x1000000
#define USBSysErrIntSt_EP24 0x1000000
#define USBSysErrIntSt_EP24_BIT 24
#define USBSysErrIntSt_EP25_MASK 0x2000000
#define USBSysErrIntSt_EP25 0x2000000
#define USBSysErrIntSt_EP25_BIT 25
#define USBSysErrIntSt_EP26_MASK 0x4000000
#define USBSysErrIntSt_EP26 0x4000000
#define USBSysErrIntSt_EP26_BIT 26
#define USBSysErrIntSt_EP27_MASK 0x8000000
#define USBSysErrIntSt_EP27 0x8000000
#define USBSysErrIntSt_EP27_BIT 27
#define USBSysErrIntSt_EP28_MASK 0x10000000
#define USBSysErrIntSt_EP28 0x10000000
#define USBSysErrIntSt_EP28_BIT 28
#define USBSysErrIntSt_EP29_MASK 0x20000000
#define USBSysErrIntSt_EP29 0x20000000
#define USBSysErrIntSt_EP29_BIT 29
#define USBSysErrIntSt_EP30_MASK 0x40000000
#define USBSysErrIntSt_EP30 0x40000000
#define USBSysErrIntSt_EP30_BIT 30
#define USBSysErrIntSt_EP31_MASK 0x80000000
#define USBSysErrIntSt_EP31 0x80000000
#define USBSysErrIntSt_EP31_BIT 31

#define USBSysErrIntClr (*(volatile uint32_t *)0x2008C2BC)
#define USBSysErrIntClr_OFFSET 0x2BC
#define USBSysErrIntClr_EP0_MASK 0x1
#define USBSysErrIntClr_EP0 0x1
#define USBSysErrIntClr_EP0_BIT 0
#define USBSysErrIntClr_EP1_MASK 0x2
#define USBSysErrIntClr_EP1 0x2
#define USBSysErrIntClr_EP1_BIT 1
#define USBSysErrIntClr_EP2_MASK 0x4
#define USBSysErrIntClr_EP2 0x4
#define USBSysErrIntClr_EP2_BIT 2
#define USBSysErrIntClr_EP3_MASK 0x8
#define USBSysErrIntClr_EP3 0x8
#define USBSysErrIntClr_EP3_BIT 3
#define USBSysErrIntClr_EP4_MASK 0x10
#define USBSysErrIntClr_EP4 0x10
#define USBSysErrIntClr_EP4_BIT 4
#define USBSysErrIntClr_EP5_MASK 0x20
#define USBSysErrIntClr_EP5 0x20
#define USBSysErrIntClr_EP5_BIT 5
#define USBSysErrIntClr_EP6_MASK 0x40
#define USBSysErrIntClr_EP6 0x40
#define USBSysErrIntClr_EP6_BIT 6
#define USBSysErrIntClr_EP7_MASK 0x80
#define USBSysErrIntClr_EP7 0x80
#define USBSysErrIntClr_EP7_BIT 7
#define USBSysErrIntClr_EP8_MASK 0x100
#define USBSysErrIntClr_EP8 0x100
#define USBSysErrIntClr_EP8_BIT 8
#define USBSysErrIntClr_EP9_MASK 0x200
#define USBSysErrIntClr_EP9 0x200
#define USBSysErrIntClr_EP9_BIT 9
#define USBSysErrIntClr_EP10_MASK 0x400
#define USBSysErrIntClr_EP10 0x400
#define USBSysErrIntClr_EP10_BIT 10
#define USBSysErrIntClr_EP11_MASK 0x800
#define USBSysErrIntClr_EP11 0x800
#define USBSysErrIntClr_EP11_BIT 11
#define USBSysErrIntClr_EP12_MASK 0x1000
#define USBSysErrIntClr_EP12 0x1000
#define USBSysErrIntClr_EP12_BIT 12
#define USBSysErrIntClr_EP13_MASK 0x2000
#define USBSysErrIntClr_EP13 0x2000
#define USBSysErrIntClr_EP13_BIT 13
#define USBSysErrIntClr_EP14_MASK 0x4000
#define USBSysErrIntClr_EP14 0x4000
#define USBSysErrIntClr_EP14_BIT 14
#define USBSysErrIntClr_EP15_MASK 0x8000
#define USBSysErrIntClr_EP15 0x8000
#define USBSysErrIntClr_EP15_BIT 15
#define USBSysErrIntClr_EP16_MASK 0x10000
#define USBSysErrIntClr_EP16 0x10000
#define USBSysErrIntClr_EP16_BIT 16
#define USBSysErrIntClr_EP17_MASK 0x20000
#define USBSysErrIntClr_EP17 0x20000
#define USBSysErrIntClr_EP17_BIT 17
#define USBSysErrIntClr_EP18_MASK 0x40000
#define USBSysErrIntClr_EP18 0x40000
#define USBSysErrIntClr_EP18_BIT 18
#define USBSysErrIntClr_EP19_MASK 0x80000
#define USBSysErrIntClr_EP19 0x80000
#define USBSysErrIntClr_EP19_BIT 19
#define USBSysErrIntClr_EP20_MASK 0x100000
#define USBSysErrIntClr_EP20 0x100000
#define USBSysErrIntClr_EP20_BIT 20
#define USBSysErrIntClr_EP21_MASK 0x200000
#define USBSysErrIntClr_EP21 0x200000
#define USBSysErrIntClr_EP21_BIT 21
#define USBSysErrIntClr_EP22_MASK 0x400000
#define USBSysErrIntClr_EP22 0x400000
#define USBSysErrIntClr_EP22_BIT 22
#define USBSysErrIntClr_EP23_MASK 0x800000
#define USBSysErrIntClr_EP23 0x800000
#define USBSysErrIntClr_EP23_BIT 23
#define USBSysErrIntClr_EP24_MASK 0x1000000
#define USBSysErrIntClr_EP24 0x1000000
#define USBSysErrIntClr_EP24_BIT 24
#define USBSysErrIntClr_EP25_MASK 0x2000000
#define USBSysErrIntClr_EP25 0x2000000
#define USBSysErrIntClr_EP25_BIT 25
#define USBSysErrIntClr_EP26_MASK 0x4000000
#define USBSysErrIntClr_EP26 0x4000000
#define USBSysErrIntClr_EP26_BIT 26
#define USBSysErrIntClr_EP27_MASK 0x8000000
#define USBSysErrIntClr_EP27 0x8000000
#define USBSysErrIntClr_EP27_BIT 27
#define USBSysErrIntClr_EP28_MASK 0x10000000
#define USBSysErrIntClr_EP28 0x10000000
#define USBSysErrIntClr_EP28_BIT 28
#define USBSysErrIntClr_EP29_MASK 0x20000000
#define USBSysErrIntClr_EP29 0x20000000
#define USBSysErrIntClr_EP29_BIT 29
#define USBSysErrIntClr_EP30_MASK 0x40000000
#define USBSysErrIntClr_EP30 0x40000000
#define USBSysErrIntClr_EP30_BIT 30
#define USBSysErrIntClr_EP31_MASK 0x80000000
#define USBSysErrIntClr_EP31 0x80000000
#define USBSysErrIntClr_EP31_BIT 31

#define USBSysErrIntSet (*(volatile uint32_t *)0x2008C2C0)
#define USBSysErrIntSet_OFFSET 0x2C0
#define USBSysErrIntSet_EP0_MASK 0x1
#define USBSysErrIntSet_EP0 0x1
#define USBSysErrIntSet_EP0_BIT 0
#define USBSysErrIntSet_EP1_MASK 0x2
#define USBSysErrIntSet_EP1 0x2
#define USBSysErrIntSet_EP1_BIT 1
#define USBSysErrIntSet_EP2_MASK 0x4
#define USBSysErrIntSet_EP2 0x4
#define USBSysErrIntSet_EP2_BIT 2
#define USBSysErrIntSet_EP3_MASK 0x8
#define USBSysErrIntSet_EP3 0x8
#define USBSysErrIntSet_EP3_BIT 3
#define USBSysErrIntSet_EP4_MASK 0x10
#define USBSysErrIntSet_EP4 0x10
#define USBSysErrIntSet_EP4_BIT 4
#define USBSysErrIntSet_EP5_MASK 0x20
#define USBSysErrIntSet_EP5 0x20
#define USBSysErrIntSet_EP5_BIT 5
#define USBSysErrIntSet_EP6_MASK 0x40
#define USBSysErrIntSet_EP6 0x40
#define USBSysErrIntSet_EP6_BIT 6
#define USBSysErrIntSet_EP7_MASK 0x80
#define USBSysErrIntSet_EP7 0x80
#define USBSysErrIntSet_EP7_BIT 7
#define USBSysErrIntSet_EP8_MASK 0x100
#define USBSysErrIntSet_EP8 0x100
#define USBSysErrIntSet_EP8_BIT 8
#define USBSysErrIntSet_EP9_MASK 0x200
#define USBSysErrIntSet_EP9 0x200
#define USBSysErrIntSet_EP9_BIT 9
#define USBSysErrIntSet_EP10_MASK 0x400
#define USBSysErrIntSet_EP10 0x400
#define USBSysErrIntSet_EP10_BIT 10
#define USBSysErrIntSet_EP11_MASK 0x800
#define USBSysErrIntSet_EP11 0x800
#define USBSysErrIntSet_EP11_BIT 11
#define USBSysErrIntSet_EP12_MASK 0x1000
#define USBSysErrIntSet_EP12 0x1000
#define USBSysErrIntSet_EP12_BIT 12
#define USBSysErrIntSet_EP13_MASK 0x2000
#define USBSysErrIntSet_EP13 0x2000
#define USBSysErrIntSet_EP13_BIT 13
#define USBSysErrIntSet_EP14_MASK 0x4000
#define USBSysErrIntSet_EP14 0x4000
#define USBSysErrIntSet_EP14_BIT 14
#define USBSysErrIntSet_EP15_MASK 0x8000
#define USBSysErrIntSet_EP15 0x8000
#define USBSysErrIntSet_EP15_BIT 15
#define USBSysErrIntSet_EP16_MASK 0x10000
#define USBSysErrIntSet_EP16 0x10000
#define USBSysErrIntSet_EP16_BIT 16
#define USBSysErrIntSet_EP17_MASK 0x20000
#define USBSysErrIntSet_EP17 0x20000
#define USBSysErrIntSet_EP17_BIT 17
#define USBSysErrIntSet_EP18_MASK 0x40000
#define USBSysErrIntSet_EP18 0x40000
#define USBSysErrIntSet_EP18_BIT 18
#define USBSysErrIntSet_EP19_MASK 0x80000
#define USBSysErrIntSet_EP19 0x80000
#define USBSysErrIntSet_EP19_BIT 19
#define USBSysErrIntSet_EP20_MASK 0x100000
#define USBSysErrIntSet_EP20 0x100000
#define USBSysErrIntSet_EP20_BIT 20
#define USBSysErrIntSet_EP21_MASK 0x200000
#define USBSysErrIntSet_EP21 0x200000
#define USBSysErrIntSet_EP21_BIT 21
#define USBSysErrIntSet_EP22_MASK 0x400000
#define USBSysErrIntSet_EP22 0x400000
#define USBSysErrIntSet_EP22_BIT 22
#define USBSysErrIntSet_EP23_MASK 0x800000
#define USBSysErrIntSet_EP23 0x800000
#define USBSysErrIntSet_EP23_BIT 23
#define USBSysErrIntSet_EP24_MASK 0x1000000
#define USBSysErrIntSet_EP24 0x1000000
#define USBSysErrIntSet_EP24_BIT 24
#define USBSysErrIntSet_EP25_MASK 0x2000000
#define USBSysErrIntSet_EP25 0x2000000
#define USBSysErrIntSet_EP25_BIT 25
#define USBSysErrIntSet_EP26_MASK 0x4000000
#define USBSysErrIntSet_EP26 0x4000000
#define USBSysErrIntSet_EP26_BIT 26
#define USBSysErrIntSet_EP27_MASK 0x8000000
#define USBSysErrIntSet_EP27 0x8000000
#define USBSysErrIntSet_EP27_BIT 27
#define USBSysErrIntSet_EP28_MASK 0x10000000
#define USBSysErrIntSet_EP28 0x10000000
#define USBSysErrIntSet_EP28_BIT 28
#define USBSysErrIntSet_EP29_MASK 0x20000000
#define USBSysErrIntSet_EP29 0x20000000
#define USBSysErrIntSet_EP29_BIT 29
#define USBSysErrIntSet_EP30_MASK 0x40000000
#define USBSysErrIntSet_EP30 0x40000000
#define USBSysErrIntSet_EP30_BIT 30
#define USBSysErrIntSet_EP31_MASK 0x80000000
#define USBSysErrIntSet_EP31 0x80000000
#define USBSysErrIntSet_EP31_BIT 31

#define I2C_RX (*(volatile uint32_t *)0x2008C300)
#define I2C_RX_OFFSET 0x300
#define I2C_RX_RXData_MASK 0xFF
#define I2C_RX_RXData_BIT 0

#define I2C_TX (*(volatile uint32_t *)0x2008C300)
#define I2C_TX_OFFSET 0x300
#define I2C_TX_TXData_MASK 0xFF
#define I2C_TX_TXData_BIT 0
#define I2C_TX_START_MASK 0xFF00
#define I2C_TX_START_BIT 8
#define I2C_TX_STOP_MASK 0x1FE00
#define I2C_TX_STOP_BIT 9

#define I2C_STS (*(volatile uint32_t *)0x2008C304)
#define I2C_STS_OFFSET 0x304
#define I2C_STS_TDI_MASK 0x1
#define I2C_STS_TDI 0x1
#define I2C_STS_TDI_BIT 0
#define I2C_STS_AFI_MASK 0x2
#define I2C_STS_AFI 0x2
#define I2C_STS_AFI_BIT 1
#define I2C_STS_NAI_MASK 0x4
#define I2C_STS_NAI 0x4
#define I2C_STS_NAI_BIT 2
#define I2C_STS_DRMI_MASK 0x8
#define I2C_STS_DRMI 0x8
#define I2C_STS_DRMI_BIT 3
#define I2C_STS_DRSI_MASK 0x10
#define I2C_STS_DRSI 0x10
#define I2C_STS_DRSI_BIT 4
#define I2C_STS_Active_MASK 0x20
#define I2C_STS_Active 0x20
#define I2C_STS_Active_BIT 5
#define I2C_STS_SCL_MASK 0x40
#define I2C_STS_SCL 0x40
#define I2C_STS_SCL_BIT 6
#define I2C_STS_SDA_MASK 0x80
#define I2C_STS_SDA 0x80
#define I2C_STS_SDA_BIT 7
#define I2C_STS_RFF_MASK 0x100
#define I2C_STS_RFF 0x100
#define I2C_STS_RFF_BIT 8
#define I2C_STS_RFE_MASK 0x200
#define I2C_STS_RFE 0x200
#define I2C_STS_RFE_BIT 9
#define I2C_STS_TFF_MASK 0x400
#define I2C_STS_TFF 0x400
#define I2C_STS_TFF_BIT 10
#define I2C_STS_TFE_MASK 0x800
#define I2C_STS_TFE 0x800
#define I2C_STS_TFE_BIT 11

#define I2C_CTL (*(volatile uint32_t *)0x2008C308)
#define I2C_CTL_OFFSET 0x308
#define I2C_CTL_TDIE_MASK 0x1
#define I2C_CTL_TDIE 0x1
#define I2C_CTL_TDIE_BIT 0
#define I2C_CTL_AFIE_MASK 0x2
#define I2C_CTL_AFIE 0x2
#define I2C_CTL_AFIE_BIT 1
#define I2C_CTL_NAIE_MASK 0x4
#define I2C_CTL_NAIE 0x4
#define I2C_CTL_NAIE_BIT 2
#define I2C_CTL_DRMIE_MASK 0x8
#define I2C_CTL_DRMIE 0x8
#define I2C_CTL_DRMIE_BIT 3
#define I2C_CTL_DRSIE_MASK 0x10
#define I2C_CTL_DRSIE 0x10
#define I2C_CTL_DRSIE_BIT 4
#define I2C_CTL_REFIE_MASK 0x20
#define I2C_CTL_REFIE 0x20
#define I2C_CTL_REFIE_BIT 5
#define I2C_CTL_RFDAIE_MASK 0x40
#define I2C_CTL_RFDAIE 0x40
#define I2C_CTL_RFDAIE_BIT 6
#define I2C_CTL_TFFIE_MASK 0x80
#define I2C_CTL_TFFIE 0x80
#define I2C_CTL_TFFIE_BIT 7
#define I2C_CTL_SRST_MASK 0x100
#define I2C_CTL_SRST 0x100
#define I2C_CTL_SRST_BIT 8

#define I2C_CLKHI (*(volatile uint32_t *)0x2008C30C)
#define I2C_CLKHI_OFFSET 0x30C
#define I2C_CLKHI_CDHI_MASK 0xFF
#define I2C_CLKHI_CDHI_BIT 0

#define I2C_CLKLO (*(volatile uint32_t *)0x2008C310)
#define I2C_CLKLO_OFFSET 0x310
#define I2C_CLKLO_CDLO_MASK 0xFF
#define I2C_CLKLO_CDLO_BIT 0

#define USBClkCtrl (*(volatile uint32_t *)0x2008CFF4)
#define USBClkCtrl_OFFSET 0xFF4
#define USBClkCtrl_DEV_CLK_EN_MASK 0x2
#define USBClkCtrl_DEV_CLK_EN 0x2
#define USBClkCtrl_DEV_CLK_EN_BIT 1
#define USBClkCtrl_PORTSEL_CLK_EN_MASK 0x8
#define USBClkCtrl_PORTSEL_CLK_EN 0x8
#define USBClkCtrl_PORTSEL_CLK_EN_BIT 3
#define USBClkCtrl_AHB_CLK_EN_MASK 0x10
#define USBClkCtrl_AHB_CLK_EN 0x10
#define USBClkCtrl_AHB_CLK_EN_BIT 4

#define USBClkSt (*(volatile uint32_t *)0x2008CFF8)
#define USBClkSt_OFFSET 0xFF8
#define USBClkSt_DEV_CLK_EN_MASK 0x2
#define USBClkSt_DEV_CLK_EN 0x2
#define USBClkSt_DEV_CLK_EN_BIT 1
#define USBClkSt_PORTSEL_CLK_EN_MASK 0x8
#define USBClkSt_PORTSEL_CLK_EN 0x8
#define USBClkSt_PORTSEL_CLK_EN_BIT 3
#define USBClkSt_AHB_CLK_EN_MASK 0x10
#define USBClkSt_AHB_CLK_EN 0x10
#define USBClkSt_AHB_CLK_EN_BIT 4

#define OTGClkCtrl (*(volatile uint32_t *)0x2008CFF4)
#define OTGClkCtrl_OFFSET 0xFF4
#define OTGClkCtrl_HOST_CLK_EN_MASK 0x1
#define OTGClkCtrl_HOST_CLK_EN 0x1
#define OTGClkCtrl_HOST_CLK_EN_BIT 0
#define OTGClkCtrl_DEV_CLK_EN_MASK 0x2
#define OTGClkCtrl_DEV_CLK_EN 0x2
#define OTGClkCtrl_DEV_CLK_EN_BIT 1
#define OTGClkCtrl_I2C_CLK_EN_MASK 0x4
#define OTGClkCtrl_I2C_CLK_EN 0x4
#define OTGClkCtrl_I2C_CLK_EN_BIT 2
#define OTGClkCtrl_OTG_CLK_EN_MASK 0x8
#define OTGClkCtrl_OTG_CLK_EN 0x8
#define OTGClkCtrl_OTG_CLK_EN_BIT 3
#define OTGClkCtrl_AHB_CLK_EN_MASK 0x10
#define OTGClkCtrl_AHB_CLK_EN 0x10
#define OTGClkCtrl_AHB_CLK_EN_BIT 4

#define OTGClkSt (*(volatile uint32_t *)0x2008CFF8)
#define OTGClkSt_OFFSET 0xFF8
#define OTGClkSt_HOST_CLK_EN_MASK 0x1
#define OTGClkSt_HOST_CLK_EN 0x1
#define OTGClkSt_HOST_CLK_EN_BIT 0
#define OTGClkSt_DEV_CLK_EN_MASK 0x2
#define OTGClkSt_DEV_CLK_EN 0x2
#define OTGClkSt_DEV_CLK_EN_BIT 1
#define OTGClkSt_I2C_CLK_EN_MASK 0x4
#define OTGClkSt_I2C_CLK_EN 0x4
#define OTGClkSt_I2C_CLK_EN_BIT 2
#define OTGClkSt_OTG_CLK_EN_MASK 0x8
#define OTGClkSt_OTG_CLK_EN 0x8
#define OTGClkSt_OTG_CLK_EN_BIT 3
#define OTGClkSt_AHB_CLK_EN_MASK 0x10
#define OTGClkSt_AHB_CLK_EN 0x10
#define OTGClkSt_AHB_CLK_EN_BIT 4

struct LPC17xx_USB_Driver {
    static const uint32_t c_Used_Endpoints = USB_MAX_QUEUES;
    static const uint32_t c_default_ctrl_packet_size = 64;

    USB_CONTROLLER_STATE*   pUsbControllerState;

#if defined(USB_REMOTE_WAKEUP)
#define USB_MAX_REMOTE_WKUP_RETRY 5

    HAL_COMPLETION          RemoteWKUP10msCompletion;
    HAL_COMPLETION          RemoteWKUP100msEOPCompletion;
    uint32_t                  RemoteWkUpRetry;
#endif

    uint8_t                   ControlPacketBuffer[c_default_ctrl_packet_size];
    uint16_t                  EndpointStatus[c_Used_Endpoints];
    bool                    TxRunning[c_Used_Endpoints];
    bool                    TxNeedZLPS[c_Used_Endpoints];

    uint8_t                   PreviousDeviceState;
    uint8_t                   RxExpectedToggle[c_Used_Endpoints];
    bool                    PinsProtected;
    bool                    FirstDescriptorPacket;

#if defined(USB_METRIC_COUNTING)
    USB_PERFORMANCE_METRICS PerfMetrics;
#endif

    //--//

    static USB_CONTROLLER_STATE * GetState(int Controller);

    static bool Initialize(int Controller);

    static bool Uninitialize(int Controller);

    static bool StartOutput(USB_CONTROLLER_STATE* State, int endpoint);

    static bool RxEnable(USB_CONTROLLER_STATE* State, int endpoint);

    static bool GetInterruptState();

    static bool ProtectPins(int Controller, bool On);

    //private:

    static void ProcessEP0(int in, int setup);
    static void ProcessEndPoint(int ep, int in);

    static void ClearTxQueue(USB_CONTROLLER_STATE* State, int endpoint);

    static void StartHardware();

    static void StopHardware();

    static void TxPacket(USB_CONTROLLER_STATE* State, int endpoint);

    static void ControlNext();

    static void SuspendEvent();
    static void ResumeEvent();
    static void ResetEvent();

    static void Global_ISR(void*  Param);
    static void EP0_ISR(uint32_t Param);
    static void EP_TxISR(uint32_t Param);
    static void EP_RxISR(uint32_t Param);


#if defined(USB_REMOTE_WAKEUP)
    static void RemoteWkUp_ISR(void* Param);
    static void RemoteWkUp_Process(void* Param);
#endif
};

extern LPC17xx_USB_Driver g_LPC17xx_USB_Driver;

union EndpointConfiguration {
    struct {
        unsigned EE : 1;      // Endpoint enable (1 = enable)
        unsigned DE : 1;      // Double buffer enable (1 = double buffered)
        unsigned MPS : 10;      // Maximum packet size (iso=1-1023, blk=8,16,32,64, int=1-64
        unsigned ED : 1;      // Endpoint direction (1 = IN)
        unsigned ET : 2;      // Endpoint type (1=iso, 2=blk, 3=int)
        unsigned EN : 4;      // Endpoint number (1-15)
        unsigned AISN : 3;      // Alternate Interface number
        unsigned IN : 3;      // Interface number
        unsigned CN : 2;      // Configuration number
    } bits;
    uint32_t word;
};

LPC17xx_USB_Driver g_LPC17xx_USB_Driver;


bool LPC17_UsbClient_Initialize(int controller) {
    if ((uint32_t)controller >= TOTAL_USB_CONTROLLER)
        return false;

    return LPC17xx_USB_Driver::Initialize(controller);
}

bool LPC17_UsbClient_Uninitialize(int controller) {
    return LPC17xx_USB_Driver::Uninitialize(controller);
}

bool LPC17_UsbClient_StartOutput(USB_CONTROLLER_STATE* State, int ep) {

    return LPC17xx_USB_Driver::StartOutput(State, ep);
}

bool LPC17_UsbClient_RxEnable(USB_CONTROLLER_STATE* State, int ep) {

    return LPC17xx_USB_Driver::RxEnable(State, ep);
}

// Specific driver

//EP Types:
#define GHI_EP_TYPE_CONTROL 0
#define GHI_EP_TYPE_ISO     1
#define GHI_EP_TYPE_BULK    2
#define GHI_EP_TYPE_INT     3

// LPC datasheet
const char EP_Type_Table[16] =
{
GHI_EP_TYPE_CONTROL,
GHI_EP_TYPE_INT,
GHI_EP_TYPE_BULK,
GHI_EP_TYPE_ISO,
GHI_EP_TYPE_INT,
GHI_EP_TYPE_BULK,
GHI_EP_TYPE_ISO,
GHI_EP_TYPE_INT,
GHI_EP_TYPE_BULK,
GHI_EP_TYPE_ISO,
GHI_EP_TYPE_INT,
GHI_EP_TYPE_BULK,
GHI_EP_TYPE_ISO,
GHI_EP_TYPE_INT,
GHI_EP_TYPE_BULK,
GHI_EP_TYPE_BULK,
};

bool _appendZP = true;

USB_CONTROLLER_STATE UsbControllerState[1];     // Only 1 USB Controller for this device

//Hal_Queue_KnownSize<USB_PACKET64,USB_QUEUE_PACKET_COUNT> QueueBuffers[LPC17xx_USB_Driver::c_Used_Endpoints-1];

static EndpointConfiguration EndpointInit[LPC17xx_USB_Driver::c_Used_Endpoints];     // Corresponds to endpoint configuration RAM at LPC17xx_USB::UDCCRx

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Gus added USB functions////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
static uint8_t  USB_DeviceAddress = 0;
static int nacking_rx_OUT_data[LPC17xx_USB_Driver::c_Used_Endpoints];
#define CONTORL_EP_ADDR	0x80

#define USB_POWER           0
//#define USB_IF_NUM          4
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

uint8_t GHAL_USBC_Get_bMaxPacketSize0() {
    return 64;
}

uint16_t GHAL_USBC_GetEndpointMap() {
    return 0x1249;
}

void GHAL_USBC_AppendZeroPacketToWrite(bool appendZP) {
    _appendZP = appendZP;
}
static void WrCmd(uint32_t cmd) {
    USBDevIntClr = CCEMTY_INT | CDFULL_INT;
    USBCmdCode = cmd;
    while ((USBDevIntSt & CCEMTY_INT) == 0);
}
static void WrCmdDat(uint32_t cmd, uint32_t val) {
    USBDevIntClr = CCEMTY_INT;
    USBCmdCode = cmd;
    while ((USBDevIntSt & CCEMTY_INT) == 0);
    USBDevIntClr = CCEMTY_INT;
    USBCmdCode = val;
    while ((USBDevIntSt & CCEMTY_INT) == 0);
}
static uint32_t RdCmdDat(uint32_t cmd) {
    USBDevIntClr = CCEMTY_INT | CDFULL_INT;
    USBCmdCode = cmd;
    while ((USBDevIntSt & CDFULL_INT) == 0);
    return (USBCmdData);
}



static void USB_SetAddress(uint32_t adr) {
    WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /* Don't wait for next */
    WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /*  Setup Status Phase */
}

static void USB_Reset(void) {

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

void USB_Connect(bool con) {
    WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(con ? DEV_CON : 0));
}
uint32_t EPAdr(uint32_t EPNum, int8_t in) {
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

    WrCmd(CMD_SEL_EP(EPAdr(EPNum, 1)));
    WrCmd(CMD_VALID_BUF);

    return (cnt);
}

static uint32_t USB_ReadEP(uint32_t EPNum, uint8_t *pData) {
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
        WrCmd(CMD_SEL_EP(EPAdr(EPNum, 0)));
        WrCmd(CMD_CLR_BUF);
    }

    return (cnt);
}

static void USB_SetStallEP(uint32_t EPNum, int8_t in) {
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum, in)), DAT_WR_BYTE(EP_STAT_ST));
}

void LPC17xx_USB_Driver::ProcessEndPoint(int ep, int in) {
    int val;
    //interrupt flags are not cleared yet
    if (in) {
        EP_TxISR(ep);
    }
    else {
        USBEpIntClr = 1 << EPAdr(ep, in);
        while ((USBDevIntSt & CDFULL_INT) == 0);
        val = USBCmdData;
        EP_RxISR(ep);
    }

}

void USB_ConfigEP(uint8_t ep_addr, int8_t in, uint8_t size) {
    uint32_t num;

    num = EPAdr(ep_addr, in);
    USBReEp |= (1 << num);
    USBEpInd = num;
    USBEpMaxPSize = size;
    while ((USBDevIntSt & EP_RLZED_INT) == 0);
    USBDevIntClr = EP_RLZED_INT;
}
void USB_EnableEP(uint32_t EPNum, int8_t in) {
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum, in)), DAT_WR_BYTE(0));
}
void USB_DisableEP(int32_t EPNum, int8_t in) {
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum, in)), DAT_WR_BYTE(EP_STAT_DA));
}
void USB_ResetEP(uint32_t EPNum, int8_t in) {
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum, in)), DAT_WR_BYTE(0));
}

void USB_HW_Configure(bool cfg) {
    WrCmdDat(CMD_CFG_DEV, DAT_WR_BYTE(cfg ? CONF_DVICE : 0));

    USBReEp = 0x00000003;
    while ((USBDevIntSt & EP_RLZED_INT) == 0);
    USBDevIntClr = EP_RLZED_INT;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
USB_CONTROLLER_STATE * LPC17xx_USB_Driver::GetState(int Controller) {
    if (Controller != 0)       // There is only one controller for this device
        return nullptr;
    return &UsbControllerState[0];
}

bool LPC17xx_USB_Driver::Initialize(int Controller) {
    int endpointsUsed = 0;

    USB_CONTROLLER_STATE &State = UsbControllerState[0];

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17_Interrupt_Activate(USB_IRQn, (uint32_t*)&Global_ISR, 0);

    for (int i = 0; i < c_Used_Endpoints; i++)
        EndpointInit[i].word = 0;       // All useable endpoints initialize to unused

    for (auto stream = 0; stream < c_Used_Endpoints; stream++) {
        auto idx = 0;
        if (State.streams[stream].RxEP != USB_NULL_ENDPOINT) {
            idx = State.streams[stream].RxEP;
            EndpointInit[idx].bits.ED = 0;
            EndpointInit[idx].bits.DE = 0;
        }

        if (State.streams[stream].TxEP != USB_NULL_ENDPOINT) {
            idx = State.streams[stream].TxEP;
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
            EndpointInit[idx].bits.MPS = State.MaxPacketSize[idx];
        }
    }

    g_LPC17xx_USB_Driver.pUsbControllerState = &State;
    g_LPC17xx_USB_Driver.PinsProtected = true;

    State.EndpointStatus = &g_LPC17xx_USB_Driver.EndpointStatus[0];
    State.EndpointCount = c_Used_Endpoints;
    State.PacketSize = c_default_ctrl_packet_size;

    State.FirstGetDescriptor = true;

    ProtectPins(Controller, false);

    return true;
}

bool LPC17xx_USB_Driver::Uninitialize(int Controller) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    ProtectPins(Controller, true);

    g_LPC17xx_USB_Driver.pUsbControllerState = nullptr;

    //LPC17_Interrupt_Deactivate( LPC17xx_AITC::c_IRQ_INDEX_USB_CLIENT );
    LPC17_Interrupt_Deactivate(USB_IRQn);

    memset(UsbControllerState, 0, sizeof(USB_CONTROLLER_STATE));

    return true;
}
//volatile int test_flag = 1;
bool LPC17xx_USB_Driver::StartOutput(USB_CONTROLLER_STATE* State, int endpoint) {
    int32_t m, n, val;


    DISABLE_INTERRUPTS_SCOPED(irq);

    /* if the halt feature for this endpoint is set, then just
       clear all the characters */
    if (State->EndpointStatus[endpoint] & USB_STATUS_ENDPOINT_HALT) {
        ClearTxQueue(State, endpoint);
        return true;
    }

    //If TxRunning, interrupts will drain the queue
    if (!g_LPC17xx_USB_Driver.TxRunning[endpoint]) {
        g_LPC17xx_USB_Driver.TxRunning[endpoint] = true;

        // Calling both TxPacket & EP_TxISR in this routine could cause a TX FIFO overflow
        TxPacket(State, endpoint);
    }
    else if (irq.WasDisabled()) {

        n = EPAdr(endpoint, 1); // It is an output endpoint for sure
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
                    ProcessEP0(0, 1);// out setup
                }
                else {
                    if ((n & 1) == 0)                /* OUT Endpoint */
                    {
                        ProcessEP0(0, 0);// out not setup
                    }
                    else {
                        ProcessEP0(1, 0);// in not setup
                    }
                }


            }
            else {
                if (State->Queues[m] && State->IsTxQueue[endpoint])
                    ProcessEndPoint(m, 1);//out
                else
                    ProcessEndPoint(m, 0);//in
            }

        }



    }

    return true;
}

bool LPC17xx_USB_Driver::GetInterruptState() {
    return true;/////////////////////////////////////////////////
}

//--//

void LPC17xx_USB_Driver::ClearTxQueue(USB_CONTROLLER_STATE* State, int endpoint) {

    while (nullptr != LPC17_UsbClient_TxDequeue(State, endpoint, true)) {
        State->Queues[endpoint]->erase(State->Queues[endpoint]->begin());
    }
}


bool LPC17_UsbClient_SoftReset(int controller) {
    LPC17_Interrupt_Activate(USB_IRQn, (uint32_t*)&LPC17xx_USB_Driver::Global_ISR, 0);

    return true;
}
//--//
void LPC17xx_USB_Driver::StartHardware() {
    //   PCONP |= 0x80000000;
    *(uint32_t*)0x400FC0C4 |= 0x80000000;
    USBClkCtrl = (1 << 1) | (1 << 3) | (1 << 4);

    OTGClkCtrl = 0x1F;
    while ((OTGClkSt & 0x1F) != 0x1F);
    // connect

    LPC17_Gpio_ConfigurePin(14, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction3, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(31, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction1, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    OTGStCtrl |= 3;

    USB_Reset();
    USB_SetAddress(0);

    USBDevIntEn = DEV_STAT_INT;	/* Enable Device Status Interrupt */

    USB_Connect(false);
    // delay if removed and then connected...
    LPC17_Time_Delay(nullptr, 120 * 1000);

    USB_Connect(true);

}

void LPC17xx_USB_Driver::StopHardware() {
    USB_Connect(false);
}

void LPC17xx_USB_Driver::TxPacket(USB_CONTROLLER_STATE* State, int endpoint) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    // transmit a packet on UsbPortNum, if there are no more packets to transmit, then die
    USB_PACKET64* Packet64;
    bool done = false;

    for (;;) {
        done = true;
        Packet64 = LPC17_UsbClient_TxDequeue(State, endpoint, true);

        if (Packet64 == nullptr || Packet64->Size > 0) {
            break;
        }
        else if (Packet64->Size == 0) {
            if (done) {
                State->Queues[endpoint]->erase(State->Queues[endpoint]->begin());
            }
        }


    }

    if (Packet64) {

        USB_WriteEP(endpoint, Packet64->Buffer, Packet64->Size);

        g_LPC17xx_USB_Driver.TxNeedZLPS[endpoint] = false;
        if (Packet64->Size == 64 && _appendZP)
            g_LPC17xx_USB_Driver.TxNeedZLPS[endpoint] = true;

        if (done) {
            State->Queues[endpoint]->erase(State->Queues[endpoint]->begin());
        }

    }
    else {
        // send the zero length packet since we landed on the FIFO boundary before
        // (and we queued a zero length packet to transmit)
        if (g_LPC17xx_USB_Driver.TxNeedZLPS[endpoint]) {
            USB_WriteEP(endpoint, (uint8_t*)nullptr, 0);
            g_LPC17xx_USB_Driver.TxNeedZLPS[endpoint] = false;
        }

        // no more data
        g_LPC17xx_USB_Driver.TxRunning[endpoint] = false;
    }


}
void LPC17xx_USB_Driver::ControlNext() {

    USB_CONTROLLER_STATE *State = g_LPC17xx_USB_Driver.pUsbControllerState;

    if (State->DataCallback) {
        // this call can't fail
        State->DataCallback(State);

        if (State->DataSize == 0) {
            USB_WriteEP(CONTORL_EP_ADDR, (uint8_t*)nullptr, 0);
            State->DataCallback = nullptr;                         // Stop sending stuff if we're done
        }
        else {

            USB_WriteEP(CONTORL_EP_ADDR, State->Data, State->DataSize);

            if (State->DataSize < c_default_ctrl_packet_size)    // If packet is less than full length
            {
                State->DataCallback = nullptr;                     // Stop sending stuff if we're done
            }


            // special handling the USB driver set address test, cannot use the first descriptor as the ADDRESS state is handle in the hardware
            if (g_LPC17xx_USB_Driver.FirstDescriptorPacket) {
                State->DataCallback = nullptr;
            }

        }
    }
}

void LPC17xx_USB_Driver::Global_ISR(void* Param) {
    // we had a weird behavior without this here when disconnectin/connecting USB cable
    DISABLE_INTERRUPTS_SCOPED(irq);
    int32_t disr, val, n, m;

    disr = USBDevIntSt;                      /* Device Interrupt Status */
    USBDevIntClr = disr;                       /* A known issue on LPC214x */

    if (disr & DEV_STAT_INT) {
        //DeviceInterruptCount++;

        WrCmd(CMD_GET_DEV_STAT);
        val = RdCmdDat(DAT_GET_DEV_STAT);       /* Device Status */

        if (val & DEV_RST)                     /* Reset */
        {
            ResetEvent();
        }

        if (val & DEV_SUS_CH)                  /* Suspend/Resume */
        {
            if (val & DEV_SUS)                   /* Suspend */
            {
                SuspendEvent();
            }
            else                               /* Resume */
            {
                ResumeEvent();
            }
        }

        goto isr_end;
    }

    /* Endpoint's Slow Interrupt */
    if (disr & EP_SLOW_INT) {

        //while (USBEpIntSt)                   /* Endpoint Interrupt Status */
        {
            for (n = 0; n < USB_EP_NUM; n++)     /* Check All Endpoints */
            {
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
                            ProcessEP0(0, 1);// out setup
                            continue;
                        }
                        if ((n & 1) == 0)                /* OUT Endpoint */
                        {
                            ProcessEP0(0, 0);// out not setup
                        }
                        else {
                            ProcessEP0(1, 0);// in not setup
                        }

                        continue;
                    }
                    if ((n & 1) == 0)                /* OUT Endpoint */
                    {
                        ProcessEndPoint(m, 0);//out
                        //USBC_EPEvent(m, true);
                    }
                    else                           /* IN Endpoint */
                    {
                        ProcessEndPoint(m, 1);//in
                        //USBC_EPEvent(m, false);
                    }
                }
            }
        }
    }

isr_end:
    return;
}


void LPC17xx_USB_Driver::EP0_ISR(uint32_t Param) {
    while (1);//this is not used?????????
}
void LPC17xx_USB_Driver::ProcessEP0(int in, int setup) {
    uint32_t EP_INTR;
    int i;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // set up packet receive
    if (setup)//(EP_INTR & LPC17xx_USB::UDCICR__PCKT) && (USB.UDCCSRx[0] & LPC17xx_USB::UDCCSR__OPC))
    {
        USB_CONTROLLER_STATE *State = g_LPC17xx_USB_Driver.pUsbControllerState;

        uint8_t   len = 0;//USB.UDCBCRx[0] & LPC17xx_USB::UDCBCR_mask;        // Received packet length

        len = USB_ReadEP(0x00, g_LPC17xx_USB_Driver.ControlPacketBuffer);

        // special handling for the very first SETUP command - Getdescriptor[DeviceType], the host looks for 8 bytes data only
        USB_SETUP_PACKET* Setup = (USB_SETUP_PACKET*)&g_LPC17xx_USB_Driver.ControlPacketBuffer[0];
        if ((Setup->bRequest == USB_GET_DESCRIPTOR) && (((Setup->wValue & 0xFF00) >> 8) == USB_DEVICE_DESCRIPTOR_TYPE) && (Setup->wLength != 0x12))
            g_LPC17xx_USB_Driver.FirstDescriptorPacket = true;
        else
            g_LPC17xx_USB_Driver.FirstDescriptorPacket = false;

        // send it to the upper layer
        State->Data = &g_LPC17xx_USB_Driver.ControlPacketBuffer[0];
        State->DataSize = len;

        uint8_t result = LPC17_UsbClient_ControlCallback(State);

        switch (result) {
            case USB_STATE_DATA:
                // setup packet was handled and the upper layer has data to send
                break;

            case USB_STATE_ADDRESS:
                // upper layer needs us to change the address
                // address stage handles in hardware
                USB_DeviceAddress = State->Address | 0x80;
                break;

            case USB_STATE_DONE:
                State->DataCallback = nullptr;
                break;

            case USB_STATE_STALL:
                // since the setup command all handled in the hardware, should not have this state
                //
                // setup packet failed to process successfully
                // set stall condition on the default control
                // endpoint
                //
                USB_SetStallEP(0, 0);
                USB_SetStallEP(0, 1);
                break;

            case USB_STATE_STATUS:
                // handle by hardware
                break;
            case USB_STATE_CONFIGURATION:
                // handle partly by hardware and the GLOBAL_ISR will take care.
                // USB spec 9.4.5 SET_CONFIGURATION resets halt conditions, resets toggle bits

                //int USB_Configure( int Controller, const USB_DYNAMIC_CONFIGURATION *config )////////////////////////////////////////////////////////////////////////////////////////////
                USB_HW_Configure(true);

                // enable all endpoints // ignore EP0
                for (i = 1; i < 16; i++) {
                    // direction in
                    USB_ConfigEP(i, 1, 64);
                    USB_EnableEP(i, 1);
                    USB_ResetEP(i, 1);

                    // direction out
                    USB_ConfigEP(i, 0, 64);
                    USB_EnableEP(i, 0);
                    USB_ResetEP(i, 0);
                }

                break;

            case USB_STATE_REMOTE_WAKEUP:
                // It is not using currently as the device side won't go into SUSPEND mode unless
                // the PC is purposely to select it to SUSPEND, as there is always SOF in the bus
                // to keeping the device from SUSPEND.
                break;

            default:

                break;
                // the status change is only seen and taken care in hardware
        }
        if (result != USB_STATE_STALL) {
            ControlNext();

            // If the port is configured, then output any possible withheld data
            if (result == USB_STATE_CONFIGURATION) {
                for (int ep = 0; ep < c_Used_Endpoints; ep++) {
                    if (State->IsTxQueue[ep])
                        StartOutput(State, ep);
                }
            }
        }
    }
    else if (in)//(EP_INTR & LPC17xx_USB::UDCICR__PCKT) && (USB.UDCCSRx[0] & (LPC17xx_USB::UDCCSR__IPR | LPC17xx_USB::UDCCSR__SST)) == 0)
    {
        // If previous packet has been sent and UDC is ready for more
        ControlNext();      // See if there is more to send
        if (USB_DeviceAddress & 0x80) {
            USB_DeviceAddress &= 0x7F;
            USB_SetAddress(USB_DeviceAddress);
        }
    }
    else {
        // This is for usb CDC
    }

}

void LPC17xx_USB_Driver::EP_TxISR(uint32_t endpoint) {
    ASSERT_IRQ_MUST_BE_OFF();

    uint32_t EP_INTR;
    int val;

    if (USBEpIntSt & (1 << EPAdr(endpoint, 1)))//done sending?
    {
        //clear interrupt flag
        USBEpIntClr = 1 << EPAdr(endpoint, 1);
        while ((USBDevIntSt & CDFULL_INT) == 0);
        val = USBCmdData;

        // successfully transmitted packet, time to send the next one
        TxPacket(g_LPC17xx_USB_Driver.pUsbControllerState, endpoint);
    }

}

void LPC17xx_USB_Driver::EP_RxISR(uint32_t endpoint) {
    bool          DisableRx;
    USB_PACKET64* Packet64 = LPC17_UsbClient_RxEnqueue(g_LPC17xx_USB_Driver.pUsbControllerState, endpoint, DisableRx);

    /* copy packet in, making sure that Packet64->Buffer is never overflowed */
    if (Packet64) {
        uint8_t   len = 0;//USB.UDCBCRx[EPno] & LPC17xx_USB::UDCBCR_mask;
        uint32_t* packetBuffer = (uint32_t*)Packet64->Buffer;
        len = USB_ReadEP(endpoint, Packet64->Buffer);

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

void LPC17xx_USB_Driver::SuspendEvent() {
    //LPC17xx_USB& USB = LPC17xx::USB();
    USB_CONTROLLER_STATE *State = g_LPC17xx_USB_Driver.pUsbControllerState;

    // SUSPEND event only happened when Host(PC) set the device to SUSPEND
    // as there is always SOF every 1ms on the BUS to keep the device from
    // suspending. Therefore, the REMOTE wake up is not necessary at the ollie side

    g_LPC17xx_USB_Driver.PreviousDeviceState = State->DeviceState;

    State->DeviceState = USB_DEVICE_STATE_SUSPENDED;

    LPC17_UsbClient_StateCallback(State);
}


void LPC17xx_USB_Driver::ResumeEvent() {
    USB_CONTROLLER_STATE *State = g_LPC17xx_USB_Driver.pUsbControllerState;

    State->DeviceState = g_LPC17xx_USB_Driver.PreviousDeviceState;

    LPC17_UsbClient_StateCallback(State);

}



void LPC17xx_USB_Driver::ResetEvent() {
    USB_CONTROLLER_STATE *State = g_LPC17xx_USB_Driver.pUsbControllerState;

    USB_Reset();
    USB_DeviceAddress = 0;

    // clear all flags
    UsbClient_Driver::ClearEvent(0, 0xFFFFFFFF);

    for (int ep = 0; ep < c_Used_Endpoints; ep++) {
        g_LPC17xx_USB_Driver.TxRunning[ep] = false;
        g_LPC17xx_USB_Driver.TxNeedZLPS[ep] = false;
    }

    State->DeviceState = USB_DEVICE_STATE_DEFAULT;
    State->Address = 0;
    LPC17_UsbClient_StateCallback(State);

}

//--//

bool LPC17xx_USB_Driver::RxEnable(USB_CONTROLLER_STATE *State, int endpoint) {
    if (nullptr == State || endpoint >= c_Used_Endpoints)
        return false;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (nacking_rx_OUT_data[endpoint])
        EP_RxISR(endpoint);//force interrupt to read the pending EP

    return true;
}

bool LPC17xx_USB_Driver::ProtectPins(int Controller, bool On) {
    USB_CONTROLLER_STATE *State = g_LPC17xx_USB_Driver.pUsbControllerState;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Initialized yet?
    if (State) {
        if (On) {
            if (!g_LPC17xx_USB_Driver.PinsProtected) {
                // Disable the USB com, state change from Not protected to Protected
                g_LPC17xx_USB_Driver.PinsProtected = true;

                USB_Reset();
                USB_DeviceAddress = 0;

                StopHardware();
            }
        }
        else {
            if (g_LPC17xx_USB_Driver.PinsProtected) {
                // Ready for USB to enable, state change from Protected to Not protected
                g_LPC17xx_USB_Driver.PinsProtected = false;

                // enable the clock,
                // set USB to attached/powered
                // set the device to a known state- Attached before it is set to the powered state (USB specf 9.1.1)
                //CPU_GPIO_EnableInputPin(LPC17xx_USB::c_USBC_GPION_DET, false, nullptr, GPIO_INT_NONE, RESISTOR_DISABLED);
                //CPU_GPIO_EnableOutputPin(LPC17xx_USB::c_USBC_GPIOX_EN, false);       // Don't signal the host yet
                State->DeviceState = USB_DEVICE_STATE_ATTACHED;

                LPC17_UsbClient_StateCallback(State);

                StartHardware();
            }
        }

        return true;
    }
    else {
        return false;
    }
}

