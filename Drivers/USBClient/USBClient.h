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

#include <string.h>
#include <TinyCLR.h>
#include <Device.h>

#define __min(a,b)  (((a) < (b)) ? (a) : (b))

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

// Endpoint Direction
#define USB_ENDPOINT_DIRECTION_IN 0x80
#define USB_ENDPOINT_DIRECTION_OUT 0x00
#define USB_ENDPOINT_NULL 0xFF

// This version of the USB code supports only one language - which
// is not specified by USB configuration records - it is defined here.
// This is the String 0 descriptor.This array includes the String descriptor
// header and exactly one language.
#define USB_LANGUAGE_DESCRIPTOR_SIZE 4

// USB 2.0 defined descriptor types
#define USB_DEVICE_DESCRIPTOR_TYPE        1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE 2
#define USB_STRING_DESCRIPTOR_TYPE        3
#define USB_INTERFACE_DESCRIPTOR_TYPE     4
#define USB_ENDPOINT_DESCRIPTOR_TYPE      5

#define USB_END_DESCRIPTOR_MARKER           0x00
#define USB_DEVICE_DESCRIPTOR_MARKER        0x01
#define USB_CONFIGURATION_DESCRIPTOR_MARKER 0x02
#define USB_STRING_DESCRIPTOR_MARKER        0x03
#define USB_GENERIC_DESCRIPTOR_MARKER       0xFF

// configuration Descriptor
#define USB_ATTRIBUTE_REMOTE_WAKEUP    0x20
#define USB_ATTRIBUTE_SELF_POWER       0x40
#define USB_ATTRIBUTE_BASE             0x80

// Sideshow descriptor lengths
#define OS_DESCRIPTOR_STRING_SIZE                18
#define OS_DESCRIPTOR_STRING_LENGTH               7
#define USB_XCOMPATIBLE_OS_SIZE                  40
#define USB_XPROPERTY_OS_SIZE_WINUSB     0x0000008E  // Size of this descriptor (78 bytes for guid + 40 bytes for the property name + 24 bytes for other fields = 142 bytes)
#define USB_XCOMPATIBLE_OS_REQUEST                4
#define USB_XPROPERTY_OS_REQUEST                  5

#define OS_DESCRIPTOR_STRING_INDEX        0xEE
#define OS_DESCRIPTOR_STRING_VENDOR_CODE  0xA5

// Generic Descriptor Header
#define USB_REQUEST_TYPE_OUT       0x00
#define USB_REQUEST_TYPE_IN        0x80
#define USB_REQUEST_TYPE_STANDARD  0x00
#define USB_REQUEST_TYPE_CLASS     0x20
#define USB_REQUEST_TYPE_VENDOR    0x40
#define USB_REQUEST_TYPE_DEVICE    0x00
#define USB_REQUEST_TYPE_INTERFACE 0x01
#define USB_REQUEST_TYPE_ENDPOINT  0x02

struct USB_CONTROLLER_STATE {
    bool                                                        initialized;
    uint8_t                                                     currentState;
    uint8_t                                                     controllerNum;
    uint32_t                                                    event;

    TinyCLR_UsbClient_Configuration                             configuration;

    /* queues & maxPacketSize must be initialized by the HAL */
    USB_PACKET64**                                   	        queues;
    uint8_t*                                                    currentPacketOffset;
    uint8_t*                                                    maxPacketSize;
    bool*                                                       isTxQueue;

    /* Arbitrarily as many pipes as endpoints since that is the maximum number of pipes
       necessary to represent the maximum number of endpoints */
    USB_PIPE_MAP*                                               pipes;
    uint8_t                                                     totalPipesCount;

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

    uint16_t*                                                   endpointStatus;
    uint8_t                                                     totalEndpointsCount;
    uint8_t                                                     endpointStatusChange;

    /* callback function for getting next packet */
    USB_NEXT_CALLBACK                                           dataCallback;

    /* for helping out upper layer during callbacks */
    uint8_t*                                                    residualData;
    uint16_t                                                    residualCount;
    uint16_t                                                    expected;

    uint8_t*                                                    fifoPacketIn;
    uint8_t*                                                    fifoPacketOut;
    uint8_t*                                                    fifoPacketCount;
    uint8_t                                                     maxFifoPacketCount;

    uint8_t*                                                    controlEndpointBuffer;
};

const TinyCLR_Api_Info* TinyCLR_UsbClient_GetApi();
void TinyCLR_UsbClient_Reset();
TinyCLR_Result TinyCLR_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result TinyCLR_UsbClient_Release(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result TinyCLR_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t& pipe, TinyCLR_UsbClient_PipeMode mode);
TinyCLR_Result TinyCLR_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t pipe);
TinyCLR_Result TinyCLR_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t pipe, const uint8_t* data, size_t& length);
TinyCLR_Result TinyCLR_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t pipe, uint8_t* data, size_t& length);
TinyCLR_Result TinyCLR_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t pipe);
TinyCLR_Result TinyCLR_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler);
const TinyCLR_UsbClient_DescriptorHeader * TinyCLR_UsbClient_FindRecord(USB_CONTROLLER_STATE* usbState, uint8_t marker, USB_SETUP_PACKET * iValue);

bool CONCAT(DEVICE_TARGET, _UsbClient_Initialize(USB_CONTROLLER_STATE* usbState));
bool CONCAT(DEVICE_TARGET, _UsbClient_Uninitialize(USB_CONTROLLER_STATE* usbState));
bool CONCAT(DEVICE_TARGET, _UsbClient_StartOutput(USB_CONTROLLER_STATE* usbState, int32_t endpoint));
bool CONCAT(DEVICE_TARGET, _UsbClient_RxEnable(USB_CONTROLLER_STATE* usbState, int32_t endpoint));
