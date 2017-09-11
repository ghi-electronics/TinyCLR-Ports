// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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
#include "STM32F4.h"
//--//
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

// USB 2.0 request packet from host
PACKED(struct) USB_SETUP_PACKET {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

// USB 2.0 response structure lengths
#define USB_STRING_DESCRIPTOR_MAX_LENGTH        126  // Maximum number of characters allowed in USB string descriptor
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

// Endpoint Attribute
#define USB_ENDPOINT_ATTRIBUTE_ISOCHRONOUS 1
#define USB_ENDPOINT_ATTRIBUTE_BULK 2
#define USB_ENDPOINT_ATTRIBUTE_INTERRUPT 3

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


PACKED(struct) USB_DYNAMIC_CONFIGURATION;

//--//


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
};

bool STM32F4_UsbClient_Initialize(int controller);
bool STM32F4_UsbClient_Uninitialize(int controller);
bool STM32F4_UsbClient_StartOutput(USB_CONTROLLER_STATE* State, int endpoint);
bool STM32F4_UsbClient_RxEnable(USB_CONTROLLER_STATE* State, int endpoint);
bool STM32F4_UsbClient_ProtectPins(int controller, bool On);

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

extern uint8_t STM32F4_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup, bool DataPhase);

extern USB_PACKET64* STM32F4_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* State, int queue, bool& DisableRx);
extern USB_PACKET64* STM32F4_UsbClient_TxDequeue(USB_CONTROLLER_STATE* State, int queue, bool  Done);

extern uint8_t STM32F4_UsbClient_ControlCallback(USB_CONTROLLER_STATE* State);
extern void  STM32F4_UsbClient_StateCallback(USB_CONTROLLER_STATE* State);

//--//

void USB_ClearQueues(USB_CONTROLLER_STATE *State, bool ClrRxQueue, bool ClrTxQueue);

const TinyCLR_UsbClient_DescriptorHeader * USB_FindRecord(USB_CONTROLLER_STATE* State, uint8_t marker, USB_SETUP_PACKET * iValue);

//--//

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

#define USB_OTG_NUM_FIFOS                8
#define USB_OTG_NUM_CHANNELS            16


typedef struct {
    // global registers
    __IO uint32_t GOTGCTL;
    __IO uint32_t GOTGINT;
    __IO uint32_t GAHBCFG;
    __IO uint32_t GUSBCFG;
    __IO uint32_t GRSTCTL;
    __IO uint32_t GINTSTS;
    __IO uint32_t GINTMSK;
    __IO uint32_t GRXSTSR;
    __IO uint32_t GRXSTSP;
    __IO uint32_t GRXFSIZ;
    union {
        __IO uint32_t HNPTXFSIZ;
        __IO uint32_t DIEPTXF0;
    };
    __IO uint32_t HNPTXSTS;
    uint32_t Res1[2];
    __IO uint32_t GCCFG;
    __IO uint32_t CID;
    uint32_t Res2[48];
    union {
        __IO uint32_t HPTXFSIZ;
        __IO uint32_t DIEPTXF[USB_OTG_NUM_FIFOS];
    };
    uint32_t Res3[184];
    // host mode registers
    __IO uint32_t HCFG;
    __IO uint32_t HFIR;
    __IO uint32_t HFNUM;
    uint32_t Res4;
    __IO uint32_t HPTXSTS;
    __IO uint32_t HAINT;
    __IO uint32_t HAINTMSK;
    uint32_t Res5[9];
    __IO uint32_t HPRT;
    uint32_t Res6[47];
    struct {
        __IO uint32_t CHAR;
        uint32_t Res7;
        __IO uint32_t INT;
        __IO uint32_t INTMSK;
        __IO uint32_t TSIZ;
        uint32_t Res8[3];
    } HC[USB_OTG_NUM_CHANNELS];
    uint32_t Res9[64];
    // device mode registers
    __IO uint32_t DCFG;
    __IO uint32_t DCTL;
    __IO uint32_t DSTS;
    uint32_t Res10;
    __IO uint32_t DIEPMSK;
    __IO uint32_t DOEPMSK;
    __IO uint32_t DAINT;
    __IO uint32_t DAINTMSK;
    uint32_t Res11[2];
    __IO uint32_t DVBUSDIS;
    __IO uint32_t DVBUSPULSE;
    uint32_t Res12;
    __IO uint32_t DIEPEMPMSK;
    uint32_t Res13[50];
    struct {
        __IO uint32_t CTL;
        uint32_t Res14;
        __IO uint32_t INT;
        uint32_t Res15;
        __IO uint32_t TSIZ;
        uint32_t Res16;
        __IO uint32_t TXFSTS;
        uint32_t Res17;
    } DIEP[USB_OTG_NUM_CHANNELS];
    struct {
        __IO uint32_t CTL;
        uint32_t Res18;
        __IO uint32_t INT;
        uint32_t Res19;
        __IO uint32_t TSIZ;
        uint32_t Res20[3];
    } DOEP[USB_OTG_NUM_CHANNELS];
    uint32_t Res21[64];
    // power and clock gating
    __IO uint32_t PCGCCTL;
    uint32_t Res22[127];
    // FIFO regions
    __IO uint32_t DFIFO[USB_OTG_NUM_FIFOS][1024];
}
OTG_TypeDef;


#define OTG_FS_BASE           (0x50000000)
#define OTG_FS                ((OTG_TypeDef *) OTG_FS_BASE)


#define OTG_GUSBCFG_PHYSEL    (1<<6)
#define OTG_GUSBCFG_PHYLPCS   (1<<15)
#define OTG_GUSBCFG_FDMOD     (1<<30)

#define OTG_GCCFG_PWRDWN      (1<<16)
#define OTG_GCCFG_VBUSASEN    (1<<18)
#define OTG_GCCFG_VBUSBSEN    (1<<19)
#define OTG_GCCFG_SOFOUTEN    (1<<20)
#define OTG_GCCFG_NOVBUSSENS  (1<<21)

#define OTG_GAHBCFG_GINTMSK   (1<<0)
#define OTG_GAHBCFG_TXFELVL   (1<<7)

#define OTG_GINTSTS_MMIS      (1<<1)
#define OTG_GINTSTS_OTGINT    (1<<2)
#define OTG_GINTSTS_RXFLVL    (1<<4)
#define OTG_GINTSTS_USBSUSP   (1<<11)
#define OTG_GINTSTS_USBRST    (1<<12)
#define OTG_GINTSTS_ENUMDNE   (1<<13)
#define OTG_GINTSTS_IEPINT    (1<<18)
#define OTG_GINTSTS_OEPINT    (1<<19)
#define OTG_GINTSTS_SRQINT    (1<<30)
#define OTG_GINTSTS_WKUPINT   (1U<<31)

#define OTG_GINTMSK_MMISM     (1<<1)
#define OTG_GINTMSK_OTGINT    (1<<2)
#define OTG_GINTMSK_RXFLVLM   (1<<4)
#define OTG_GINTMSK_USBSUSPM  (1<<11)
#define OTG_GINTMSK_USBRST    (1<<12)
#define OTG_GINTMSK_ENUMDNEM  (1<<13)
#define OTG_GINTMSK_IEPINT    (1<<18)
#define OTG_GINTMSK_OEPINT    (1<<19)
#define OTG_GINTMSK_SRQIM     (1<<30)
#define OTG_GINTMSK_WUIM      (1U<<31)

#define OTG_GRSTCTL_CSRST     (1<<0)
#define OTG_GRSTCTL_RXFFLSH   (1<<4)
#define OTG_GRSTCTL_TXFFLSH   (1<<5)
#define OTG_GRSTCTL_TXFNUM    (0x1F<<6)
#define OTG_GRSTCTL_TXF_ALL   (0x10<<6)

#define OTG_DCFG_DSPD         (0x3<<0)
#define OTG_DCFG_DAD          (0x7F<<4)

#define OTG_DCTL_RWUSIG       (1<<0)
#define OTG_DCTL_SDIS         (1<<1)
#define OTG_DCTL_POPRGDNE     (1<<11)

#define OTG_GRXSTSP_EPNUM     (0x0F<<0)
#define OTG_GRXSTSP_CHNUM     (0x0F<<0)
#define OTG_GRXSTSP_BCNT      (0x7FF<<4)
#define OTG_GRXSTSP_DPID      (0x03<<15)
#define OTG_GRXSTSP_PKTSTS    (0x0F<<17)
#define OTG_GRXSTSP_PKTSTS_GN (0x01<<17) // global OUT NAK
#define OTG_GRXSTSP_PKTSTS_PR (0x02<<17) // packet received
#define OTG_GRXSTSP_PKTSTS_DC (0x03<<17) // data transaction completed
#define OTG_GRXSTSP_PKTSTS_SC (0x04<<17) // setup stage completed
#define OTG_GRXSTSP_PKTSTS_TE (0x05<<17) // toggle error
#define OTG_GRXSTSP_PKTSTS_SR (0x06<<17) // setup data received
#define OTG_GRXSTSP_PKTSTS_CH (0x07<<17) // channel haltet
#define OTG_GRXSTSP_FRMNUM    (0x0F<<21)

#define OTG_DIEPMSK_XFRCM     (1<<0) // transfer completed
#define OTG_DIEPMSK_TOM       (1<<3) // timeout

#define OTG_DOEPMSK_XFRCM     (1<<0) // transfer completed
#define OTG_DOEPMSK_STUPM     (1<<3) // setup phase done

#define OTG_DIEPINT_XFRC      (1<<0) // transfer completed
#define OTG_DIEPINT_TOC       (1<<3) // timeout

#define OTG_DOEPINT_XFRC      (1<<0) // transfer completed
#define OTG_DOEPINT_STUP      (1<<3) // setup phase done

#define OTG_DIEPCTL_USBAEP    (1<<15)
#define OTG_DIEPCTL_STALL     (1<<21)
#define OTG_DIEPCTL_CNAK      (1<<26)
#define OTG_DIEPCTL_SNAK      (1<<27)
#define OTG_DIEPCTL_SD0PID    (1<<28)
#define OTG_DIEPCTL_EPDIS     (1<<30)
#define OTG_DIEPCTL_EPENA     (1U<<31)

#define OTG_DOEPCTL_USBAEP    (1<<15)
#define OTG_DOEPCTL_STALL     (1<<21)
#define OTG_DOEPCTL_CNAK      (1<<26)
#define OTG_DOEPCTL_SNAK      (1<<27)
#define OTG_DOEPCTL_SD0PID    (1<<28)
#define OTG_DOEPCTL_EPDIS     (1<<30)
#define OTG_DOEPCTL_EPENA     (1U<<31)

#define OTG_DIEPTSIZ_PKTCNT   (3<<19)
#define OTG_DIEPTSIZ_PKTCNT_1 (1<<19)

#define OTG_DOEPTSIZ_PKTCNT   (3<<19)
#define OTG_DOEPTSIZ_PKTCNT_1 (1<<19)
#define OTG_DOEPTSIZ_STUPCNT  (3<<29)

#ifdef DEBUG
#define USB_DEBUG_ASSERT(x) while(!(x))
#else
#define USB_DEBUG_ASSERT(x)
#endif



#define STM32F4_USB_FS_USE_ID_PIN 0
#define STM32F4_USB_FS_USE_VB_PIN 0



#define MAX_EP_SIZE            64     // maximum control channel packet size
#define MAX_EP0_SIZE            8      // default control channel packet size
#define STM32F4_USB_FS_MAX_EP   4 // OTG FS supports 4 endpoints

// use OTG Full Speed

#define STM32F4_USB_FS_ID 0

#define STM32F4_USB_DM_PIN(c) 11 // A11
#define STM32F4_USB_DP_PIN(c) 12 // A12
#define STM32F4_USB_VB_PIN(c)  9 // A9
#define STM32F4_USB_ID_PIN(c) 10 // A10

#define STM32F4_USB_USE_ID_PIN(c) STM32F4_USB_FS_USE_ID_PIN
#define STM32F4_USB_USE_VB_PIN(c) STM32F4_USB_FS_USE_VB_PIN
#define STM32F4_USB_ALT_MODE(c) (uint32_t)0x2A2; // AF10, 50MHz

#define USB_MAX_BUFFERS (STM32F4_USB_FS_MAX_EP - 1)

// FIFO sizes (in 32 bit words)
#define USB_RXFIFO_SIZE  64 // 256 bytes
#define USB_TX0FIFO_SIZE 64 // 256 bytes
#define USB_TXnFIFO_SIZE 64 // 256 bytes

// PHY turnaround time
// (4 AHB clocks + 1 Phy clock in Phy clocks)
#define STM32F4_USB_TRDT ((4 * 48000000 - 1) / SYSTEM_CYCLE_CLOCK_HZ + 2)

#define ENDPOINT_INUSED_MASK        0x01
#define ENDPOINT_DIR_IN_MASK        0x02
#define ENDPOINT_DIR_OUT_MASK       0x04


int8_t STM32F4_UsbClient_EndpointMap[] = { ENDPOINT_INUSED_MASK,                          // Endpoint 0
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 1
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK,  // Endpoint 2
                                                ENDPOINT_DIR_IN_MASK | ENDPOINT_DIR_OUT_MASK   // Endpoint 3
};

// State variables for one controller
typedef struct {
    USB_CONTROLLER_STATE state;

    uint8_t     ep0Buffer[MAX_EP_SIZE];
    uint16_t    endpointStatus[USB_MAX_QUEUES];
    uint16_t    endpointType;
    uint8_t     previousDeviceState;

} STM32F4_UsbClient_State;

/* State variables for the controllers */
static STM32F4_UsbClient_State STM32F4_UsbClient_ControllerState[TOTAL_USB_CONTROLLER];

/* Queues for all data endpoints */
static std::vector<USB_PACKET64> QueueBuffers[USB_MAX_BUFFERS];

/*
 * Suspend Event Interrupt Handler
 */
void STM32F4_UsbClient_SuspendEvent(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State) {
    // SUSPEND event only happened when Host(PC) set the device to SUSPEND
    // as there is always SOF every 1ms on the BUS to keep the device from
    // suspending. Therefore, the REMOTE wake up is not necessary at the device side
    ((STM32F4_UsbClient_State*)State)->previousDeviceState = State->DeviceState;
    State->DeviceState = USB_DEVICE_STATE_SUSPENDED;
    STM32F4_UsbClient_StateCallback(State);
}

/*
 * Resume Event Interrupt Handler
 */
void STM32F4_UsbClient_ResumeEvent(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State) {
    OTG->DCTL &= ~OTG_DCTL_RWUSIG; // remove remote wakeup signaling

    State->DeviceState = ((STM32F4_UsbClient_State*)State)->previousDeviceState;

    STM32F4_UsbClient_StateCallback(State);
}

/*
 * Reset Event Interrupt Handler
 */
void STM32F4_UsbClient_ResetEvent(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State) {
    // reset interrupts and FIFOs
    OTG->GINTSTS = 0xFFFFFFFF; // clear global interrupts
    OTG->GRXFSIZ = USB_RXFIFO_SIZE; // Rx Fifo
    OTG->DIEPTXF0 = (USB_TX0FIFO_SIZE << 16) | USB_RXFIFO_SIZE; // Tx Fifo 0
    uint32_t addr = USB_RXFIFO_SIZE + USB_TX0FIFO_SIZE;
    for (int i = 0; i < State->EndpointCount; i++) {
        OTG->DIEPTXF[i] = (USB_TXnFIFO_SIZE << 16) | addr; // Tx Fifo i
        addr += USB_TXnFIFO_SIZE;
        OTG->DIEP[i].INT = 0xFF; // clear endpoint interrupts
        OTG->DOEP[i].INT = 0xFF;
        OTG->DIEP[i].CTL = OTG_DIEPCTL_EPDIS; // deactivate endpoint
        OTG->DOEP[i].CTL = OTG_DOEPCTL_EPDIS;
    }

    // flush FIFOs
    OTG->GRSTCTL = OTG_GRSTCTL_RXFFLSH | OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXF_ALL;

    // configure control endpoint
    OTG->DIEP[0].CTL = OTG_DIEPCTL_USBAEP; // Tx FIFO num = 0, max packet size = 64
    OTG->DOEP[0].CTL = OTG_DOEPCTL_USBAEP;
    OTG->DIEP[0].TSIZ = 0;
    OTG->DOEP[0].TSIZ = OTG_DOEPTSIZ_STUPCNT; // up to 3 setup packets

    // configure data endpoints
    uint32_t intMask = 0x00010001; // ep0 interrupts;
    uint32_t eptype = ((STM32F4_UsbClient_State*)State)->endpointType >> 2; // endpoint types (2 bits / endpoint)
    uint32_t i = 1, bit = 2;
    while (eptype) {
        uint32_t type = eptype & 3;
        if (type != 0) { // data endpoint
            uint32_t ctrl = OTG_DIEPCTL_SD0PID | OTG_DIEPCTL_USBAEP;
            ctrl |= type << 18; // endpoint type
            ctrl |= State->MaxPacketSize[i]; // packet size
            if (State->IsTxQueue[i]) { // Tx (in) endpoint
                ctrl |= OTG_DIEPCTL_SNAK; // disable tx endpoint
                ctrl |= i << 22; // Tx FIFO number
                OTG->DIEP[i].CTL = ctrl; // configure in endpoint
                intMask |= bit; // enable in interrupt
            }
            else { // Rx (out) endpoint
                // Rx endpoints must be enabled here
                // Enabling after Set_Configuration does not work correctly
                OTG->DOEP[i].TSIZ = OTG_DOEPTSIZ_PKTCNT_1 | State->MaxPacketSize[i];
                ctrl |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK; // enable rx endpoint
                OTG->DOEP[i].CTL = ctrl; // configure out endpoint
                intMask |= bit << 16; // enable out interrupt
            }
        }
        i++;
        eptype >>= 2;
        bit <<= 1;
    }

    // enable interrupts
    OTG->DIEPMSK = OTG_DIEPMSK_XFRCM; // transfer complete
    OTG->DOEPMSK = OTG_DOEPMSK_XFRCM | OTG_DOEPMSK_STUPM; // setup stage done
    OTG->DAINTMSK = intMask;   // enable ep interrupts
    OTG->GINTMSK = OTG_GINTMSK_OEPINT | OTG_GINTMSK_IEPINT | OTG_GINTMSK_RXFLVLM
        | OTG_GINTMSK_USBRST | OTG_GINTMSK_USBSUSPM | OTG_GINTMSK_WUIM;

    OTG->DCFG &= ~OTG_DCFG_DAD; // reset device address

    /* clear all flags */
    UsbClient_Driver::ClearEvent(0, 0xFFFFFFFF); // clear all events on all endpoints

    State->FirstGetDescriptor = true;

    State->DeviceState = USB_DEVICE_STATE_DEFAULT;
    State->Address = 0;
    STM32F4_UsbClient_StateCallback(State);
}

/*
 * Data Endpoint Rx Interrupt Handler
 */
void STM32F4_UsbClient_EndpointRxInterrupt(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State, uint32_t ep, uint32_t count) {
    uint32_t* pd;

    if (ep == 0) { // control endpoint
        pd = (uint32_t*)((STM32F4_UsbClient_State*)State)->ep0Buffer;
        State->Data = (uint8_t*)pd;
        State->DataSize = count;
    }
    else { // data endpoint
        bool full;
        USB_PACKET64* Packet64 = STM32F4_UsbClient_RxEnqueue(State, ep, full);

        if (Packet64 == 0) {  // should not happen
            USB_DEBUG_ASSERT(0);
        }

        pd = (uint32_t*)Packet64->Buffer;
        Packet64->Size = count;
    }

    // read data
    uint32_t volatile* ps = OTG->DFIFO[ep];
    for (int c = count; c > 0; c -= 4) {
        *pd++ = *ps;
    }
    // data handling & Rx reenabling delayed to transfer completed interrupt
}

/*
 * Data In (Tx) Endpoint Interrupt Handler
 */
void STM32F4_UsbClient_EndpointInInterrupt(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State, uint32_t ep) {
    uint32_t bits = OTG->DIEP[ep].INT;
    if (bits & OTG_DIEPINT_XFRC) { // transfer completed

        OTG->DIEP[ep].INT = OTG_DIEPINT_XFRC; // clear interrupt
    }
    bool done = false;
    if (!(OTG->DIEP[ep].CTL & OTG_DIEPCTL_EPENA)) { // Tx idle

        uint32_t* ps = 0;
        uint32_t count;

        if (ep == 0) { // control endpoint
            if (State->DataCallback) { // data to send
                State->DataCallback(State);  // this call can't fail
                ps = (uint32_t*)State->Data;
                count = State->DataSize;


            }
        }
        else if (State->Queues[ep] != 0 && State->IsTxQueue[ep]) { // Tx data endpoint
            done = true;
            USB_PACKET64* Packet64 = STM32F4_UsbClient_TxDequeue(State, ep, done);

            if (Packet64) {  // data to send
                ps = (uint32_t*)Packet64->Buffer;
                count = Packet64->Size;
            }
        }

        if (ps) { // data to send
            // enable endpoint
            OTG->DIEP[ep].TSIZ = OTG_DIEPTSIZ_PKTCNT_1 | count;
            OTG->DIEP[ep].CTL |= OTG_DIEPCTL_EPENA | OTG_DIEPCTL_CNAK;

            // write data
            uint32_t volatile* pd = OTG->DFIFO[ep];
            for (int c = count; c > 0; c -= 4) {
                *pd = *ps++;
            }

            if (done) {
                State->Queues[ep]->erase(State->Queues[ep]->begin());
            }
        }
        else { // no data
            // disable endpoint
            OTG->DIEP[ep].CTL |= OTG_DIEPCTL_SNAK;
        }
    }
}

/*
 * Handle Setup Data Received on Control Endpoint
 */
void STM32F4_UsbClient_HandleSetup(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State) {
    /* send last setup packet to the upper layer */
    uint8_t result = STM32F4_UsbClient_ControlCallback(State);

    switch (result) {

    case USB_STATE_DATA:
        /* setup packet was handled and the upper layer has data to send */
        break;

    case USB_STATE_ADDRESS:
        /* upper layer needs us to change the address */

        OTG->DCFG |= State->Address << 4; // set device address
        break;

    case USB_STATE_DONE:
        State->DataCallback = 0;
        break;

    case USB_STATE_STALL:
        // setup packet failed to process successfully
        // set stall condition on the control endpoint
        OTG->DIEP[0].CTL |= OTG_DIEPCTL_STALL;
        OTG->DOEP[0].CTL |= OTG_DOEPCTL_STALL;

        // ********** skip rest of function **********
        return;

    case USB_STATE_STATUS:
        break;

    case USB_STATE_CONFIGURATION:
        break;

    case USB_STATE_REMOTE_WAKEUP:
        // It is not using currently as the device side won't go into SUSPEND mode unless
        // the PC is purposely to select it to SUSPEND, as there is always SOF in the bus
        // to keeping the device from SUSPEND.

        break;

    default:
        USB_DEBUG_ASSERT(0);
        break;
    }

    // check ep0 for replies
    STM32F4_UsbClient_EndpointInInterrupt(OTG, State, 0);

    // check all Tx endpoints after configuration setup
    if (result == USB_STATE_CONFIGURATION) {
        for (int ep = 1; ep < State->EndpointCount; ep++) {
            if (State->Queues[ep] && State->IsTxQueue[ep]) {
                STM32F4_UsbClient_EndpointInInterrupt(OTG, State, ep);
            }
        }
    }
}

/*
 * Data Out (Rx) Endpoint Interrupt Handler
 */
void STM32F4_UsbClient_EndpointOutInterrupt(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State, uint32_t ep) {
    uint32_t bits = OTG->DOEP[ep].INT;
    if (bits & OTG_DOEPINT_XFRC) { // transfer completed

        OTG->DOEP[ep].INT = OTG_DOEPINT_XFRC; // clear interrupt
    }

    if (bits & OTG_DOEPINT_STUP) { // setup phase done

        OTG->DOEP[ep].INT = OTG_DOEPINT_STUP; // clear interrupt
    }

    if (ep == 0) { // control endpoint

        // enable endpoint
        OTG->DOEP[0].TSIZ = OTG_DOEPTSIZ_STUPCNT | OTG_DOEPTSIZ_PKTCNT_1 | State->PacketSize;
        OTG->DOEP[0].CTL |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK;
        // Handle Setup data in upper layer
        STM32F4_UsbClient_HandleSetup(OTG, State);
    }
    else if ((int32_t)(State->Queues[ep]->size()) < ((int32_t)State->Queues[ep]->max_size())) {
        // enable endpoint
        OTG->DOEP[ep].TSIZ = OTG_DOEPTSIZ_PKTCNT_1 | State->MaxPacketSize[ep];
        OTG->DOEP[ep].CTL |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK;

    }
    else {
        // disable endpoint
        OTG->DOEP[ep].CTL |= OTG_DOEPCTL_SNAK;

    }
}


/*
 * Main Interrupt Handler
 */
void STM32F4_UsbClient_Interrupt(OTG_TypeDef* OTG, USB_CONTROLLER_STATE* State) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t intPend = OTG->GINTSTS; // get pending bits

    while (intPend & OTG_GINTSTS_RXFLVL) { // RxFifo non empty
        uint32_t status = OTG->GRXSTSP; // read and pop status word from fifo
        int ep = status & OTG_GRXSTSP_EPNUM;
        int count = (status & OTG_GRXSTSP_BCNT) >> 4;
        status &= OTG_GRXSTSP_PKTSTS;
        if (status == OTG_GRXSTSP_PKTSTS_PR // data received
            || status == OTG_GRXSTSP_PKTSTS_SR // setup received
            ) {
            STM32F4_UsbClient_EndpointRxInterrupt(OTG, State, ep, count);
        }
        else {
            // others: nothing to do
        }
        intPend = OTG->GINTSTS; // update pending bits
    }

    if (intPend & OTG_GINTSTS_IEPINT) { // IN endpoint
        uint32_t bits = OTG->DAINT & 0xFFFF; // pending IN endpoints
        int ep = 0;
        while (bits) {
            if (bits & 1)
                STM32F4_UsbClient_EndpointInInterrupt(OTG, State, ep);
            ep++;
            bits >>= 1;
        }
    }

    if (intPend & OTG_GINTSTS_OEPINT) { // OUT endpoint
        uint32_t bits = OTG->DAINT >> 16; // pending OUT endpoints
        int ep = 0;
        while (bits) {
            if (bits & 1)
                STM32F4_UsbClient_EndpointOutInterrupt(OTG, State, ep);

            ep++;
            bits >>= 1;
        }
    }

    if (intPend & OTG_GINTSTS_USBRST) { // reset
        STM32F4_UsbClient_ResetEvent(OTG, State);
        OTG->GINTSTS = OTG_GINTSTS_USBRST; // clear interrupt
    }
    else {
        if (intPend & OTG_GINTSTS_USBSUSP) { // suspend
            STM32F4_UsbClient_SuspendEvent(OTG, State);
            OTG->GINTSTS = OTG_GINTSTS_USBSUSP; // clear interrupt
        }

        if (intPend & OTG_GINTSTS_WKUPINT) { // wakeup
            STM32F4_UsbClient_ResumeEvent(OTG, State);
            OTG->GINTSTS = OTG_GINTSTS_WKUPINT; // clear interrupt
        }
    }
}

/*
 * OTG FS Interrupt Handler
 */
void STM32F4_UsbClient_FullspeedInterrupt(void* param) {
    STM32F4_UsbClient_Interrupt(OTG_FS, &STM32F4_UsbClient_ControllerState[STM32F4_USB_FS_ID].state);
}

bool STM32F4_UsbClient_Initialize(int controller) {
    if ((uint32_t)controller >= TOTAL_USB_CONTROLLER)
        return false;

    if (!STM32F4_Gpio_OpenPin(STM32F4_USB_DM_PIN(controller)) || !STM32F4_Gpio_OpenPin(STM32F4_USB_DP_PIN(controller)))
        return false;

    if (STM32F4_USB_USE_ID_PIN(controller) && !STM32F4_Gpio_OpenPin(STM32F4_USB_ID_PIN(controller)))
        return false;

    // enable USB clock
    // FS on AHB2
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

    USB_CONTROLLER_STATE *State = &STM32F4_UsbClient_ControllerState[controller].state;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Detach usb port for a while to enforce re-initialization
    OTG->DCTL = OTG_DCTL_SDIS; // soft disconnect

    OTG->GAHBCFG = OTG_GAHBCFG_TXFELVL;     // int on TxFifo completely empty, int off
    OTG->GUSBCFG = OTG_GUSBCFG_FDMOD        // force device mode
        | STM32F4_USB_TRDT << 10   // turnaround time
        | OTG_GUSBCFG_PHYSEL;      // internal PHY

    OTG->GCCFG = OTG_GCCFG_VBUSBSEN       // B device Vbus sensing
        | OTG_GCCFG_PWRDWN;        // transceiver enabled

    OTG->DCFG |= OTG_DCFG_DSPD;           // device speed = HS

    if (STM32F4_USB_USE_VB_PIN(controller) == 0) { // no Vbus pin
        OTG->GCCFG |= OTG_GCCFG_NOVBUSSENS; // disable vbus sense
    }

    STM32F4_Time_DelayNoInterrupt(nullptr, 1000); // asure host recognizes reattach

    // setup hardware
    STM32F4_UsbClient_ProtectPins(controller, true);

    STM32F4_Interrupt_Activate(OTG_FS_IRQn, (uint32_t*)&STM32F4_UsbClient_FullspeedInterrupt, 0);
    STM32F4_Interrupt_Activate(OTG_FS_WKUP_IRQn, (uint32_t*)&STM32F4_UsbClient_FullspeedInterrupt, 0);

    // allow interrupts
    OTG->GINTSTS = 0xFFFFFFFF;           // clear all interrupts
    OTG->GINTMSK = OTG_GINTMSK_USBRST;   // enable reset only
    OTG->DIEPEMPMSK = 0;                 // disable Tx FIFO empty interrupts
    OTG->GAHBCFG |= OTG_GAHBCFG_GINTMSK; // gloabl interrupt enable

    // rest of initializations done in reset interrupt handler
    return true;
}

bool STM32F4_UsbClient_Uninitialize(int controller) {
    STM32F4_Interrupt_Deactivate(OTG_FS_WKUP_IRQn);
    STM32F4_Interrupt_Deactivate(OTG_FS_IRQn);

    STM32F4_UsbClient_ProtectPins(controller, false);

    RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;

    STM32F4_Gpio_ClosePin(STM32F4_USB_DM_PIN(controller));
    STM32F4_Gpio_ClosePin(STM32F4_USB_DP_PIN(controller));

    if (STM32F4_USB_USE_ID_PIN(controller))
        STM32F4_Gpio_ClosePin(STM32F4_USB_ID_PIN(controller));

    return true;
}

bool STM32F4_UsbClient_StartOutput(USB_CONTROLLER_STATE* State, int ep) {
    if (State == 0 || ep >= State->EndpointCount)
        return false;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // If endpoint is not an output
    if (State->Queues[ep] == 0 || !State->IsTxQueue[ep])
        return false;

    /* if the halt feature for this endpoint is set, then just clear all the characters */
    if (State->EndpointStatus[ep] & USB_STATUS_ENDPOINT_HALT) {
        while (STM32F4_UsbClient_TxDequeue(State, ep, true) != 0) {
            State->Queues[ep]->erase(State->Queues[ep]->begin());
        } // clear TX queue


        return true;
    }

    if (irq.WasDisabled()) { // check all endpoints for pending actions
        STM32F4_UsbClient_Interrupt(OTG, State);
    }
    // write first packet if not done yet
    STM32F4_UsbClient_EndpointInInterrupt(OTG, State, ep);

    return true;
}

bool STM32F4_UsbClient_RxEnable(USB_CONTROLLER_STATE* State, int ep) {
    // If this is not a legal Rx queue
    if (State == 0 || State->Queues[ep] == 0 || State->IsTxQueue[ep])
        return false;

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // enable Rx
    if (!(OTG->DOEP[ep].CTL & OTG_DOEPCTL_EPENA)) {
        OTG->DOEP[ep].TSIZ = OTG_DOEPTSIZ_PKTCNT_1 | State->MaxPacketSize[ep];
        OTG->DOEP[ep].CTL |= OTG_DOEPCTL_EPENA | OTG_DOEPCTL_CNAK; // enable endpoint
    }

    return true;
}

bool STM32F4_UsbClient_ProtectPins(int controller, bool On) {
    USB_CONTROLLER_STATE *State = &STM32F4_UsbClient_ControllerState[controller].state;

    if (((STM32F4_UsbClient_State*)State)->endpointType == 0)
        return false;  // not yet initialized

    OTG_TypeDef* OTG = OTG_FS;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (On) {

        STM32F4_Gpio_ConfigurePin(STM32F4_USB_DM_PIN(controller), STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF10);
        STM32F4_Gpio_ConfigurePin(STM32F4_USB_DP_PIN(controller), STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF10);

        if (STM32F4_USB_USE_ID_PIN(controller)) {
            STM32F4_Gpio_ConfigurePin(STM32F4_USB_ID_PIN(controller), STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF10);
        }

        // attach usb port
        OTG->DCTL &= ~OTG_DCTL_SDIS; // remove soft disconnect

        State->DeviceState = USB_DEVICE_STATE_ATTACHED;
        STM32F4_UsbClient_StateCallback(State);
    }
    else {
        // detach usb port
        OTG->DCTL |= OTG_DCTL_SDIS; // soft disconnect

        // clear USB Txbuffer
        for (int ep = 1; ep < State->EndpointCount; ep++) {
            if (State->Queues[ep] && State->IsTxQueue[ep]) {
                while (STM32F4_UsbClient_TxDequeue(State, ep, true) != 0) {// clear TX queue
                    State->Queues[ep]->erase(State->Queues[ep]->begin());
                }

            }
        }

        State->DeviceState = USB_DEVICE_STATE_DETACHED;
        STM32F4_UsbClient_StateCallback(State);
    }

    return true;
}

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
    MAX_EP0_SIZE,                        // Endpoint 0 size
    USB_VENDOR_ID,                          // Vendor ID
    USB_PRODUCT_ID,                         // Product ID
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
    MAX_EP_SIZE,                                // Endpoint 1 packet size
    0 ,                                          // Endpoint 1 interval

    //Endpoint
    sizeof(TinyCLR_UsbClient_EndpointDescriptor),
    USB_ENDPOINT_DESCRIPTOR_TYPE,
    USB_ENDPOINT_DIRECTION_OUT,
    USB_ENDPOINT_ATTRIBUTE_BULK,
    MAX_EP_SIZE,                                // Endpoint 1 packet size
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
TinyCLR_UsbClient_OsStringDescriptor STM32F4_UsbClient_OsStringDescriptor;

// OS Extended Compatible ID for WinUSB
TinyCLR_UsbClient_XCompatibleOsId STM32F4_UsbClient_XCompatibleOsId;

// OS Extended Property
TinyCLR_UsbClient_XPropertiesOsWinUsb STM32F4_UsbClient_XPropertiesOsWinUsb;

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

    USB_CONTROLLER_STATE *State = &STM32F4_UsbClient_ControllerState[controller].state;

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

    UsbDefaultConfiguration.OS_String = (TinyCLR_UsbClient_OsStringDescriptor*)&STM32F4_UsbClient_OsStringDescriptor;
    UsbDefaultConfiguration.OS_XCompatible_ID = (TinyCLR_UsbClient_XCompatibleOsId*)&STM32F4_UsbClient_XCompatibleOsId;
    UsbDefaultConfiguration.OS_XProperty = (TinyCLR_UsbClient_XPropertiesOsWinUsb*)&STM32F4_UsbClient_XPropertiesOsWinUsb;

    UsbDefaultConfiguration.endList = (TinyCLR_UsbClient_DescriptorHeader*)&usbDescriptorHeader;

    // Init Usb State
    memset(State, 0, sizeof(USB_CONTROLLER_STATE));

    State->ControllerNum = controller;
    State->Configuration = &UsbDefaultConfiguration;
    State->CurrentState = USB_DEVICE_STATE_UNINITIALIZED;
    State->DeviceStatus = USB_STATUS_DEVICE_SELF_POWERED;
    State->EndpointCount = STM32F4_USB_FS_MAX_EP;
    State->PacketSize = MAX_EP0_SIZE;
    State->Initialized = true;

    for (auto i = 0; i < USB_MAX_QUEUES; i++) {
        State->streams[i].RxEP = USB_NULL_ENDPOINT;
        State->streams[i].TxEP = USB_NULL_ENDPOINT;
        State->MaxPacketSize[i] = MAX_EP_SIZE;
    }

    return State->Initialized;
}

bool UsbClient_Driver::Uninitialize(int controller) {
    USB_CONTROLLER_STATE *State = &STM32F4_UsbClient_ControllerState[controller].state;

    if (State == nullptr)
        return false;

    DISABLE_INTERRUPTS_SCOPED(irq);

    STM32F4_UsbClient_Uninitialize(controller);

    State->Initialized = false;

    // for soft reboot allow the USB to be off for at least 100ms
    STM32F4_Time_DelayNoInterrupt(nullptr, 100000); // 100ms

    return true;
}

bool UsbClient_Driver::OpenStream(int controller, int32_t& usbStream, TinyCLR_UsbClient_StreamMode mode) {
    USB_CONTROLLER_STATE * State = &STM32F4_UsbClient_ControllerState[controller].state;

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

        for (auto i = 0; i < SIZEOF_CONST_ARRAY(STM32F4_UsbClient_EndpointMap); i++) {
            if ((STM32F4_UsbClient_EndpointMap[i] & ENDPOINT_INUSED_MASK)) // in used
                continue;

            if (writeEp == USB_NULL_ENDPOINT && ((STM32F4_UsbClient_EndpointMap[i] & ENDPOINT_DIR_IN_MASK) == ENDPOINT_DIR_IN_MASK)) {
                writeEp = i;
                STM32F4_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

                continue;
            }

            if (readEp == USB_NULL_ENDPOINT && ((STM32F4_UsbClient_EndpointMap[i] & ENDPOINT_DIR_OUT_MASK) == ENDPOINT_DIR_OUT_MASK)) {
                readEp = i;
                STM32F4_UsbClient_EndpointMap[i] |= ENDPOINT_INUSED_MASK;

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
        uint32_t epType = ((STM32F4_UsbClient_State*)State)->endpointType;

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

                epType |= (ep->bmAttributes & 3) << (idx * 2);
                State->MaxPacketSize[idx] = ep->wMaxPacketSize;
            }

            ep = (TinyCLR_UsbClient_EndpointDescriptor  *)(((uint8_t *)ep) + ep->bLength);
        }

        ((STM32F4_UsbClient_State*)State)->endpointType = epType;

        break;
    }

    if (State->CurrentState == USB_DEVICE_STATE_UNINITIALIZED) {
        STM32F4_UsbClient_Initialize(controller);
    }

    return true;
}

bool UsbClient_Driver::CloseStream(int controller, int usbStream) {
    USB_CONTROLLER_STATE * State = &STM32F4_UsbClient_ControllerState[controller].state;

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
    STM32F4_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    // Close the TX stream
    endpoint = State->streams[usbStream].TxEP;
    if (endpoint != USB_NULL_ENDPOINT && State->Queues[endpoint]) {
        State->Queues[endpoint]->clear(); // Clear the queue
        QueueBuffers[endpoint - 1] = std::vector< USB_PACKET64>();
    }

    State->streams[usbStream].TxEP = USB_NULL_ENDPOINT;

    //Free endpoint
    STM32F4_UsbClient_EndpointMap[endpoint] &= ~ENDPOINT_INUSED_MASK;

    return true;
}

int UsbClient_Driver::Write(int controller, int usbStream, const char* Data, size_t size) {
    int endpoint;
    int totWrite = 0;
    USB_CONTROLLER_STATE * State = &STM32F4_UsbClient_ControllerState[controller].state;

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

                STM32F4_UsbClient_StartOutput(State, endpoint);

                irq.Release();
                //                lcd_printf("Looping in write\r\n");

                STM32F4_Time_Delay(nullptr, 50);

                irq.Acquire();
            }
        }

        // here we have a post-condition that IRQs are disabled for all paths through conditional block above

        if (State->DeviceState == USB_DEVICE_STATE_CONFIGURED) {
            STM32F4_UsbClient_StartOutput(State, endpoint);
        }

        return totWrite;
    }
}

int UsbClient_Driver::Read(int controller, int usbStream, char* Data, size_t size) {
    int endpoint;
    USB_CONTROLLER_STATE * State = &STM32F4_UsbClient_ControllerState[controller].state;

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

                STM32F4_UsbClient_RxEnable(State, endpoint);
            }
        }

        return count;
    }
}

bool UsbClient_Driver::Flush(int controller, int usbStream) {
    int endpoint;
    int retries = USB_FLUSH_RETRY_COUNT;
    int queueCnt;
    USB_CONTROLLER_STATE * State = &STM32F4_UsbClient_ControllerState[controller].state;

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
        STM32F4_UsbClient_StartOutput(State, endpoint);

        int cnt = (int32_t)State->Queues[endpoint]->size();

        if (queueCnt == cnt)
            STM32F4_Time_Delay(nullptr, 100); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == cnt) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = cnt;
    }

    if (retries <= 0)
        State->Queues[endpoint]->clear();

    return true;
}

uint32_t UsbClient_Driver::SetEvent(int controller, uint32_t Event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_CONTROLLER_STATE *State = &STM32F4_UsbClient_ControllerState[controller].state;

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

    USB_CONTROLLER_STATE *State = &STM32F4_UsbClient_ControllerState[controller].state;

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
            STM32F4_UsbClient_RxEnable(State, endpoint);
        }
    }

    if (ClrTxQueue) {
        for (int endpoint = 0; endpoint < USB_MAX_QUEUES; endpoint++) {
            if (State->Queues[endpoint] && State->IsTxQueue[endpoint])
                State->Queues[endpoint]->clear();
        }
    }
}

void STM32F4_UsbClient_StateCallback(USB_CONTROLLER_STATE* State) {
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

    STM32F4_UsbClient_StateCallback(State);

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

uint8_t STM32F4_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* State, USB_SETUP_PACKET* Setup, bool DataPhase) {
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

    STM32F4_UsbClient_StateCallback(State);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        State->ResidualCount = 0;
        State->DataCallback = USB_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

//--//

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

        //uint32_t *marker = (uint32_t*)*ptr;


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

uint8_t STM32F4_UsbClient_ControlCallback(USB_CONTROLLER_STATE* State) {
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
        return STM32F4_UsbClient_HandleSetConfiguration(State, Setup, true);
    default:
        return USB_HandleConfigurationRequests(State, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* STM32F4_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* State, int endpoint, bool& DisableRx) {
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

USB_PACKET64* STM32F4_UsbClient_TxDequeue(USB_CONTROLLER_STATE* State, int endpoint, bool Done) {
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
TinyCLR_Result STM32F4_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    uint8_t *osStringDescriptor = (uint8_t*)&STM32F4_UsbClient_OsStringDescriptor;
    uint8_t *xCompatibleOsId = (uint8_t*)&STM32F4_UsbClient_XCompatibleOsId;
    uint8_t *xPropertiesOsWinUsb = (uint8_t*)&STM32F4_UsbClient_XPropertiesOsWinUsb;

    UsbClient_Driver::OsExtendedPropertyHandler(self, osStringDescriptor, xCompatibleOsId, xPropertiesOsWinUsb);

    UsbClient_Driver::Initialize(controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_Release(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;
    UsbClient_Driver::Uninitialize(controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t& stream, TinyCLR_UsbClient_StreamMode mode) {
    int32_t controller = self->Index;
    int32_t availableStream;

    if (UsbClient_Driver::OpenStream(controller, availableStream, mode) == true) {
        stream = availableStream;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result STM32F4_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t stream) {
    int32_t controller = self->Index;

    UsbClient_Driver::CloseStream(controller, stream);
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t stream, const uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    length = UsbClient_Driver::Write(controller, stream, (const char*)data, length);
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t stream, uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

    length = UsbClient_Driver::Read(controller, stream, (char*)data, length);
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t stream) {
    int32_t controller = self->Index;

    UsbClient_Driver::Flush(controller, stream);
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    int32_t controller = self->Index;

    UsbClient_Driver::DataReceivedHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_SetOsExtendedProperty(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler) {
    int32_t controller = self->Index;

    UsbClient_Driver::OsExtendedPropertyHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length) {
    memcpy(&deviceDescriptor, descriptor, length);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length) {
    memcpy(&configDescriptor, descriptor, length);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value) {
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

void STM32F4_UsbClient_Reset() {
    for (auto controller = 0; controller < usbClientApi.Count; controller++) {
        // Close all stream if any opened
        for (auto stream = 0; stream < USB_MAX_QUEUES; stream++) {
            UsbClient_Driver::CloseStream(controller, stream);
        }

        // Close controller
        UsbClient_Driver::Uninitialize(controller);
    }
}

const TinyCLR_Api_Info* STM32F4_UsbClient_GetApi() {
    usbClientProvider.Parent = &usbClientApi;
    usbClientProvider.Index = 0;
    usbClientProvider.Acquire = &STM32F4_UsbClient_Acquire;
    usbClientProvider.Release = &STM32F4_UsbClient_Release;
    usbClientProvider.Open = &STM32F4_UsbClient_Open;
    usbClientProvider.Close = &STM32F4_UsbClient_Close;
    usbClientProvider.Write = &STM32F4_UsbClient_Write;
    usbClientProvider.Read = &STM32F4_UsbClient_Read;
    usbClientProvider.Flush = &STM32F4_UsbClient_Flush;
    usbClientProvider.SetDataReceivedHandler = &STM32F4_UsbClient_SetDataReceivedHandler;
    usbClientProvider.SetOsExtendedPropertyHandler = &STM32F4_UsbClient_SetOsExtendedProperty;
    usbClientProvider.SetDeviceDescriptor = &STM32F4_UsbClient_SetDeviceDescriptor;
    usbClientProvider.SetConfigDescriptor = &STM32F4_UsbClient_SetConfigDescriptor;
    usbClientProvider.SetStringDescriptor = &STM32F4_UsbClient_SetStringDescriptor;

    usbClientApi.Author = "GHI Electronics, LLC";
    usbClientApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.UsbClientProvider";
    usbClientApi.Type = TinyCLR_Api_Type::UsbClientProvider;
    usbClientApi.Version = 0;
    usbClientApi.Count = 1;
    usbClientApi.Implementation = &usbClientProvider;

    return &usbClientApi;
}
