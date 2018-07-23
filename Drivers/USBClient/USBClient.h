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
/// USB Debugger state
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

// Endpoint type
#define USB_ENDPOINT_ATTRIBUTE_ISOCHRONOUS 1
#define USB_ENDPOINT_ATTRIBUTE_BULK 2
#define USB_ENDPOINT_ATTRIBUTE_INTERRUPT 3

// This version of the USB code supports only one language - which
// is not specified by USB configuration records - it is defined here.
// This is the String 0 descriptor.This array includes the String descriptor
// header and exactly one language.
#define USB_LANGUAGE_DESCRIPTOR_SIZE 4

// configuration Descriptor
#define USB_ATTRIBUTE_REMOTE_WAKEUP    0x20
#define USB_ATTRIBUTE_SELF_POWER       0x40
#define USB_ATTRIBUTE_BASE             0x80

// Generic Descriptor Header
#define USB_REQUEST_TYPE_OUT       0x00
#define USB_REQUEST_TYPE_IN        0x80
#define USB_REQUEST_TYPE_STANDARD  0x00
#define USB_REQUEST_TYPE_CLASS     0x20
#define USB_REQUEST_TYPE_VENDOR    0x40
#define USB_REQUEST_TYPE_DEVICE    0x00
#define USB_REQUEST_TYPE_INTERFACE 0x01
#define USB_REQUEST_TYPE_ENDPOINT  0x02

// Actual USB Client Structures size.
// The size send to host must be plus 2 bytes for lenght and type.
#define USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE      5
#define USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE     7
#define USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE 7
#define USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE        16

// This size must be large than WinUsb xproperty os size (0x8E)
#define USB_ENDPOINT_CONTROL_BUFFER_SIZE 256

struct USB_PACKET64 {
    uint32_t Size;
    uint8_t  Buffer[64];
};

struct USB_PIPE_MAP {
    uint8_t RxEP;
    uint8_t TxEP;
};

struct UsClientState {
    int32_t                                                     controllerIndex;
    bool                                                        initialized;
    uint8_t                                                     currentState;
    uint32_t                                                    event;

    TinyCLR_UsbClient_DeviceDescriptor                          deviceDescriptor;

    /* queues & maxPacketSize must be initialized by the HAL */
    USB_PACKET64**                                   	        queues;
    uint8_t*                                                    currentPacketOffset;
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
    uint8_t*                                                    maxEndpointsPacketSize;
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
void TinyCLR_UsbClient_Reset(int32_t controller);

TinyCLR_Result TinyCLR_UsbClient_Acquire(const TinyCLR_UsbClient_Controller* self);
TinyCLR_Result TinyCLR_UsbClient_Release(const TinyCLR_UsbClient_Controller* self);
TinyCLR_Result TinyCLR_UsbClient_OpenPipe(const TinyCLR_UsbClient_Controller* self, uint8_t writeEndpoint, uint8_t readEndpoint, uint32_t& pipe);
TinyCLR_Result TinyCLR_UsbClient_ClosePipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe);
TinyCLR_Result TinyCLR_UsbClient_WritePipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, const uint8_t* data, size_t& length);
TinyCLR_Result TinyCLR_UsbClient_ReadPipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, uint8_t* data, size_t& length);
TinyCLR_Result TinyCLR_UsbClient_FlushPipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe);
TinyCLR_Result TinyCLR_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_DataReceivedHandler handler);
TinyCLR_Result TinyCLR_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Controller* self, const TinyCLR_UsbClient_DeviceDescriptor* descriptor);
TinyCLR_Result TinyCLR_UsbClient_SetVendorClassRequestHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_RequestHandler handler);
const uint8_t* TinyCLR_UsbClient_FindRecord(UsClientState* usClientState, uint8_t marker, TinyCLR_UsbClient_SetupPacket * iValue);

bool TinyCLR_UsbClient_Initialize(UsClientState* usClientState);
bool TinyCLR_UsbClient_Uninitialize(UsClientState* usClientState);
bool TinyCLR_UsbClient_StartOutput(UsClientState* usClientState, int32_t endpoint);
bool TinyCLR_UsbClient_RxEnable(UsClientState* usClientState, int32_t endpoint);
void TinyCLR_UsbClient_Delay(uint64_t microseconds);
TinyCLR_Result TinyCLR_UsbClient_GetControllerCount(const TinyCLR_UsbClient_Controller* self, int32_t& count);

void TinyCLR_UsbClient_InitializeConfiguration(UsClientState *usClientState);
uint32_t TinyCLR_UsbClient_GetEndpointSize(int32_t endpoint);
