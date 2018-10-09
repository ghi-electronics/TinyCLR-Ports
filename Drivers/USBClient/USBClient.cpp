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
#include "USBClient.h"

#define __min(a,b)  (((a) < (b)) ? (a) : (b))

#define TOTAL_USBCLIENT_CONTROLLERS 1

static TinyCLR_UsbClient_Controller usbClientControllers[TOTAL_USBCLIENT_CONTROLLERS];
static TinyCLR_Api_Info usbClientApi[TOTAL_USBCLIENT_CONTROLLERS];
static UsClientState usbClientStates[TOTAL_USBCLIENT_CONTROLLERS];

TinyCLR_UsbClient_DataReceivedHandler TinyCLR_UsbClient_SetDataReceived;
TinyCLR_UsbClient_RequestHandler TinyCLR_UsbClient_ProcessVendorClassRequest = nullptr;
TinyCLR_UsbClient_RequestHandler TinyCLR_UsbClient_SetGetDescriptor = nullptr;

void TinyCLR_UsbClient_SetEvent(UsClientState *usClientState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t old_event = usClientState->event;

    usClientState->event |= event;

    if (old_event != usClientState->event) {
        TinyCLR_UsbClient_SetDataReceived(nullptr, 0);
    }
}

void TinyCLR_UsbClient_ClearEvent(UsClientState *usClientState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    usClientState->event &= ~event;
}

void TinyCLR_UsbClient_ClearQueues(UsClientState *usClientState, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto ifc = 0; ifc < usClientState->deviceDescriptor.Configurations->InterfaceCount; ifc++) {

        TinyCLR_UsbClient_InterfaceDescriptor* ifcx = (TinyCLR_UsbClient_InterfaceDescriptor*)&usClientState->deviceDescriptor.Configurations->Interfaces[ifc];

        if (ClrRxQueue) {
            for (auto endpoint = 0; endpoint < ifcx->EndpointCount; endpoint++) {
                if (usClientState->queues[endpoint] == nullptr || usClientState->isTxQueue[endpoint])
                    continue;

                TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);

                /* since this queue is now reset, we have room available for newly arrived packets */
                TinyCLR_UsbClient_RxEnable(usClientState, endpoint);
            }
        }

        if (ClrTxQueue) {
            for (auto endpoint = 0; endpoint < ifcx->EndpointCount; endpoint++) {
                if (usClientState->queues[endpoint] && usClientState->isTxQueue[endpoint])
                    TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);
            }
        }
    }
}

void TinyCLR_UsbClient_StateCallback(UsClientState* usClientState) {
    if (usClientState->currentState != usClientState->deviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
        //Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
        // The RxQueue is clear when it is configured.
        if (USB_DEVICE_STATE_CONFIGURED == usClientState->currentState) {
            TinyCLR_UsbClient_ClearQueues(usClientState, true, true);
        }

        usClientState->currentState = usClientState->deviceState;

        switch (usClientState->deviceState) {
        case USB_DEVICE_STATE_DETACHED:
            usClientState->residualCount = 0;
            usClientState->dataCallback = nullptr;

            break;
        case USB_DEVICE_STATE_CONFIGURED:
            /* whenever we enter the configured state, re-initialize all of the RxQueues */
            /* Txqueue has stored some data to be transmitted */
            TinyCLR_UsbClient_ClearQueues(usClientState, true, false);
            break;
        }
    }
}

void TinyCLR_UsbClient_DataCallback(UsClientState* usClientState) {
    uint32_t length = __min(usClientState->maxEndpointsPacketSize[0], usClientState->residualCount);

    memcpy(usClientState->ptrData, usClientState->residualData, length);

    usClientState->dataSize = length;
    usClientState->residualData += length;
    usClientState->residualCount -= length;

    if (length == usClientState->maxEndpointsPacketSize[0]) {
        usClientState->expected -= length;
    }
    else {
        usClientState->expected = 0;
    }

    if (usClientState->expected) {
        usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;
    }
    else {
        usClientState->dataCallback = nullptr;
    }
}

uint8_t TinyCLR_UsbClient_HandleGetStatus(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    uint16_t* status;
    uint16_t  zero = 0;

    /* validate setup packet */
    if (Setup->Value != 0 || Setup->Length != 2) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        status = &usClientState->deviceStatus;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        if (usClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
            return USB_STATE_STALL;
        }

        status = &zero;
        break;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usClientState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index >= usClientState->totalEndpointsCount) {
            return USB_STATE_STALL;
        }

        status = &usClientState->endpointStatus[Setup->Index];
        break;

    default:
        return USB_STATE_STALL;
    }


    /* send requested status to host */
    usClientState->residualData = reinterpret_cast<uint8_t*>(status);
    usClientState->residualCount = 2;
    usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleClearFeature(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->Value != USB_FEATURE_DEVICE_REMOTE_WAKEUP)
            return USB_STATE_STALL;

        // Locate the deviceDescriptor descriptor
        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)TinyCLR_UsbClient_FindRecord(usClientState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);

        if (Config && (Config->Attributes & USB_ATTRIBUTE_REMOTE_WAKEUP)) {
            usClientState->deviceStatus &= ~USB_STATUS_DEVICE_REMOTE_WAKEUP;
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
        if (usClientState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0)
            return USB_STATE_STALL;

        /* bit 0x80 designates direction, which we dont utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index == 0 || Setup->Index >= usClientState->totalEndpointsCount)
            return USB_STATE_STALL;

        if (Setup->Value != USB_FEATURE_ENDPOINT_HALT)
            return USB_STATE_STALL;

        /* clear the halt feature */
        usClientState->endpointStatus[Setup->Index] &= ~USB_STATUS_ENDPOINT_HALT;
        usClientState->endpointStatusChange = Setup->Index;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usClientState->residualCount = 0;
    usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t TinyCLR_UsbClient_HandleSetFeature(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->Value != USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
            return USB_STATE_STALL;
        }

        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)TinyCLR_UsbClient_FindRecord(usClientState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);
        if (Config == nullptr)        // If the deviceDescriptor record could not be found
            return USB_STATE_STALL; // Something pretty serious is wrong

        if (Config->Attributes & USB_ATTRIBUTE_REMOTE_WAKEUP) {
            usClientState->deviceStatus |= USB_STATUS_DEVICE_REMOTE_WAKEUP;
        }

        retState = USB_STATE_REMOTE_WAKEUP;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        /* there are no interface features to set */
        return USB_STATE_STALL;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usClientState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index == 0 || Setup->Index >= usClientState->totalEndpointsCount) {
            return USB_STATE_STALL;
        }

        if (Setup->Value != USB_FEATURE_ENDPOINT_HALT) {
            return USB_STATE_STALL;
        }

        /* set the halt feature */
        usClientState->endpointStatus[Setup->Index] |= USB_STATUS_ENDPOINT_HALT;
        usClientState->endpointStatusChange = Setup->Index;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usClientState->residualCount = 0;
    usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t TinyCLR_UsbClient_HandleSetAddress(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    /* validate setup packet */
    if (Setup->Value > 127 || Setup->Index != 0 || Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usClientState->deviceState >= USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    /* set address */
    usClientState->address = Setup->Value;

    /* catch state changes */
    if (usClientState->address == 0) {
        usClientState->deviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else {
        usClientState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }

    TinyCLR_UsbClient_StateCallback(usClientState);

    /* send zero-length packet to tell host we're done */
    usClientState->residualCount = 0;
    usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t TinyCLR_UsbClient_HandleConfigurationRequests(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    const uint8_t* header;
    uint8_t       type;
    uint8_t       DescriptorIndex;
    int32_t offset = 0;

    auto controllerIndex = usClientState->controllerIndex;

    /* this request is valid regardless of device state */
    type = ((Setup->Value & 0xFF00) >> 8);
    DescriptorIndex = (Setup->Value & 0x00FF);
    usClientState->expected = Setup->Length;

    if (usClientState->expected == 0) {
        // just return an empty Status packet
        usClientState->residualCount = 0;
        usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;
        return USB_STATE_DATA;
    }

    // The very first GET_DESCRIPTOR command out of reset should always return at most maxEndpointsPacketSize[0] bytes.
    // After that, you can return as many as the host has asked.
    if (usClientState->deviceState <= USB_DEVICE_STATE_DEFAULT) {
        if (usClientState->firstGetDescriptor) {
            usClientState->firstGetDescriptor = false;

            usClientState->expected = __min(usClientState->expected, usClientState->maxEndpointsPacketSize[0]);
        }
    }

    usClientState->residualData = nullptr;
    usClientState->residualCount = 0;

    if (Setup->Request == USB_GET_DESCRIPTOR) {
        switch (type) {
        case USB_DEVICE_DESCRIPTOR_TYPE:
            header = TinyCLR_UsbClient_FindRecord(usClientState, USB_DEVICE_DESCRIPTOR_TYPE, Setup);
            if (header) {
                auto size = 0;
                usClientState->controlEndpointBuffer[1] = USB_DEVICE_DESCRIPTOR_TYPE;

                size += 2;

                memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), header, USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE);

                size += USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE;

                usClientState->controlEndpointBuffer[0] = size;

                // ready to send
                usClientState->residualData = usClientState->controlEndpointBuffer;
                usClientState->residualCount = __min(usClientState->expected, size);
            }

            break;

        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            header = TinyCLR_UsbClient_FindRecord(usClientState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);

            if (header) {

                usClientState->controlEndpointBuffer[0] = (2 + USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE);
                usClientState->controlEndpointBuffer[1] = USB_CONFIGURATION_DESCRIPTOR_TYPE;

                auto size = 2;

                // Parse configuration
                memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), header, USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE);
                size += USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE;

                // Parse Vendor Class Descriptor
                for (auto vendor_id = 0; vendor_id < usClientState->deviceDescriptor.Configurations->VendorClassDescriptorCount; vendor_id++) {
                    auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&usClientState->deviceDescriptor.Configurations->VendorClassDescriptors[vendor_id];

                    usClientState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                    usClientState->controlEndpointBuffer[size + 1] = vendor->Type;

                    size += 2;
                    memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                    size += vendor->Length;
                }

                // Parse interfaces
                for (auto interface_id = 0; interface_id < usClientState->deviceDescriptor.Configurations->InterfaceCount; interface_id++) {
                    auto interface_size_index = size;

                    usClientState->controlEndpointBuffer[size + 0] = 2 + USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE;
                    usClientState->controlEndpointBuffer[size + 1] = USB_INTERFACE_DESCRIPTOR_TYPE;

                    size += 2;
                    memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_InterfaceDescriptor*>(&usClientState->deviceDescriptor.Configurations->Interfaces[interface_id])), USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE);
                    size += USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE;

                    auto ifc = (TinyCLR_UsbClient_InterfaceDescriptor*)&usClientState->deviceDescriptor.Configurations->Interfaces[interface_id];

                    // Parse Vendor Class Descriptor
                    for (auto vendor_id = 0; vendor_id < ifc->VendorClassDescriptorCount; vendor_id++) {
                        auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&ifc->VendorClassDescriptors[vendor_id];

                        usClientState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                        usClientState->controlEndpointBuffer[size + 1] = vendor->Type;

                        size += 2;
                        memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                        size += vendor->Length;
                    }

                    // Parse Endpoints
                    for (auto endpoint = 0; endpoint < ifc->EndpointCount; endpoint++) {
                        auto ep = (TinyCLR_UsbClient_EndpointDescriptor*)&ifc->Endpoints[endpoint];

                        usClientState->controlEndpointBuffer[size + 0] = USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE + 2;
                        usClientState->controlEndpointBuffer[size + 1] = USB_ENDPOINT_DESCRIPTOR_TYPE;

                        size += 2;
                        memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), reinterpret_cast<uint8_t*>(ep), USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE);
                        size += USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE;

                        // Parse Vendor Class Descriptor
                        for (auto vendor_id = 0; vendor_id < ep->VendorClassDescriptorCount; vendor_id++) {
                            auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&ep->VendorClassDescriptors[vendor_id];

                            usClientState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                            usClientState->controlEndpointBuffer[size + 1] = vendor->Type;

                            size += 2;
                            memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                            size += vendor->Length;
                        }
                    }
                }

                // Update size
                usClientState->controlEndpointBuffer[2] = (size >> 0) & 0xFF;
                usClientState->controlEndpointBuffer[3] = (size >> 8) & 0xFF;

                // ready to send
                usClientState->residualData = usClientState->controlEndpointBuffer;
                usClientState->residualCount = __min(usClientState->expected, size);
            }
            break;

        case USB_STRING_DESCRIPTOR_TYPE:

            if (nullptr != (header = TinyCLR_UsbClient_FindRecord(usClientState, USB_STRING_DESCRIPTOR_TYPE, Setup))) {
                TinyCLR_UsbClient_StringDescriptor* str = (TinyCLR_UsbClient_StringDescriptor*)header;

                auto size = 2 + str->Length * 2;

                usClientState->controlEndpointBuffer[0] = size;
                usClientState->controlEndpointBuffer[1] = USB_STRING_DESCRIPTOR_TYPE;

                memcpy(reinterpret_cast<uint8_t*>(&usClientState->controlEndpointBuffer[2]), reinterpret_cast<uint8_t*>(const_cast<wchar_t*>(str->Data)), str->Length * 2);

                usClientState->residualData = usClientState->controlEndpointBuffer;
                usClientState->residualCount = __min(usClientState->expected, size);
            }

            break;

        default:

            if (TinyCLR_UsbClient_SetGetDescriptor != nullptr) {

                const uint8_t* responsePayload;

                size_t responsePayloadLength = 0;

                if (TinyCLR_UsbClient_SetGetDescriptor(&usbClientControllers[controllerIndex], Setup, responsePayload, responsePayloadLength, TinyCLR_UsbClient_Now()) == TinyCLR_Result::Success) {
                    memcpy(usClientState->controlEndpointBuffer, reinterpret_cast<uint8_t*>(const_cast<uint8_t*>(responsePayload)), responsePayloadLength);
                    usClientState->residualData = usClientState->controlEndpointBuffer;
                    usClientState->residualCount = __min(usClientState->expected, responsePayloadLength);
                }
            }

            break;
        }
    }
    else {
        if (Setup->RequestType & (USB_REQUEST_TYPE_VENDOR | USB_REQUEST_TYPE_CLASS)) {
            const uint8_t* responsePayload;

            size_t responsePayloadLength = 0;

            if (TinyCLR_UsbClient_ProcessVendorClassRequest(&usbClientControllers[controllerIndex], Setup, responsePayload, responsePayloadLength, TinyCLR_UsbClient_Now()) == TinyCLR_Result::Success) {
                memcpy(usClientState->controlEndpointBuffer, reinterpret_cast<uint8_t*>(const_cast<uint8_t*>(responsePayload)), responsePayloadLength);

                usClientState->residualData = usClientState->controlEndpointBuffer;
                usClientState->residualCount = __min(usClientState->expected, responsePayloadLength);
            }

        }
    }

    // If the request was not recognized, the generic types should be searched
    if (usClientState->residualData == nullptr) {
        return USB_STATE_STALL;
    }

    usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleGetConfiguration(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    /* validate setup packet */
    if (Setup->Value != 0 || Setup->Index != 0 || Setup->Length != 1) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    usClientState->residualData = &usClientState->configurationNum;
    usClientState->residualCount = 1;
    usClientState->expected = 1;
    usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleSetConfiguration(UsClientState* usClientState, TinyCLR_UsbClient_SetupPacket* Setup, bool DataPhase) {
    /* validate setup packet */
    if (Setup->Index != 0 || Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    /* we only support one deviceDescriptor */
    if (Setup->Value > 1) {
        return USB_STATE_STALL;
    }

    usClientState->configurationNum = Setup->Value;

    /* catch state changes */
    if (usClientState->configurationNum == 0) {
        usClientState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }
    else {
        usClientState->deviceState = USB_DEVICE_STATE_CONFIGURED;
    }

    TinyCLR_UsbClient_StateCallback(usClientState);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        usClientState->residualCount = 0;
        usClientState->dataCallback = TinyCLR_UsbClient_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB deviceDescriptor records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const uint8_t* TinyCLR_UsbClient_FindRecord(UsClientState* usClientState, uint8_t marker, TinyCLR_UsbClient_SetupPacket * setup) {
    bool found = false;

    uint8_t* ptr = nullptr;

    switch (marker) {
    case USB_DEVICE_DESCRIPTOR_TYPE:
        ptr = reinterpret_cast<uint8_t*>(&usClientState->deviceDescriptor);
        found = true;
        break;

    case USB_CONFIGURATION_DESCRIPTOR_TYPE:
        ptr = reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_ConfigurationDescriptor*>(usClientState->deviceDescriptor.Configurations));
        found = true;
        break;
    case USB_STRING_DESCRIPTOR_TYPE:

        for (auto i = 0; i < usClientState->deviceDescriptor.StringCount; i++) {
            auto stringDescriptor = (TinyCLR_UsbClient_StringDescriptor*)&usClientState->deviceDescriptor.Strings[i];

            if (stringDescriptor->Index == (setup->Value & 0x00FF)) {
                ptr = reinterpret_cast<uint8_t*>(stringDescriptor);
                found = true;

                break;
            }

            stringDescriptor++;
        }

        break;
    }

    return found ? ptr : nullptr;
}

uint8_t TinyCLR_UsbClient_ControlCallback(UsClientState* usClientState) {
    TinyCLR_UsbClient_SetupPacket* Setup;

    if (usClientState->dataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (TinyCLR_UsbClient_SetupPacket*)usClientState->ptrData;

    switch (Setup->Request) {
    case USB_GET_STATUS:
        return TinyCLR_UsbClient_HandleGetStatus(usClientState, Setup);
    case USB_CLEAR_FEATURE:
        return TinyCLR_UsbClient_HandleClearFeature(usClientState, Setup);
    case USB_SET_FEATURE:
        return TinyCLR_UsbClient_HandleSetFeature(usClientState, Setup);
    case USB_SET_ADDRESS:
        return TinyCLR_UsbClient_HandleSetAddress(usClientState, Setup);
    case USB_GET_CONFIGURATION:
        return TinyCLR_UsbClient_HandleGetConfiguration(usClientState, Setup);
    case USB_SET_CONFIGURATION:
        return TinyCLR_UsbClient_HandleSetConfiguration(usClientState, Setup, true);
    default:
        return TinyCLR_UsbClient_HandleConfigurationRequests(usClientState, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(UsClientState* usClientState, int32_t endpoint, bool& disableRx) {
    USB_PACKET64* packet;

    if (usClientState->fifoPacketCount[endpoint] == usClientState->maxFifoPacketCount[endpoint]) {
        disableRx = true;

        return nullptr;
    }

    disableRx = false;

    packet = &usClientState->queues[endpoint][usClientState->fifoPacketIn[endpoint]];

    usClientState->fifoPacketIn[endpoint]++;
    usClientState->fifoPacketCount[endpoint]++;

    if (usClientState->fifoPacketIn[endpoint] == usClientState->maxFifoPacketCount[endpoint])
        usClientState->fifoPacketIn[endpoint] = 0;

    TinyCLR_UsbClient_SetEvent(usClientState, 1 << endpoint);

    return packet;
}

USB_PACKET64* TinyCLR_UsbClient_TxDequeue(UsClientState* usClientState, int32_t endpoint) {
    USB_PACKET64* packet;

    if (usClientState->fifoPacketCount[endpoint] == 0) {
        return nullptr;
    }

    packet = &usClientState->queues[endpoint][usClientState->fifoPacketOut[endpoint]];

    usClientState->fifoPacketCount[endpoint]--;
    usClientState->fifoPacketOut[endpoint]++;

    if (usClientState->fifoPacketOut[endpoint] == usClientState->maxFifoPacketCount[endpoint])
        usClientState->fifoPacketOut[endpoint] = 0;

    return packet;
}

void TinyCLR_UsbClient_ClearEndpoints(UsClientState* usClientState, int32_t endpoint) {
    usClientState->fifoPacketIn[endpoint] = usClientState->fifoPacketOut[endpoint] = usClientState->fifoPacketCount[endpoint] = 0;
}

bool TinyCLR_UsbClient_CanReceivePackage(UsClientState* usClientState, int32_t endpoint) {
    return usClientState->fifoPacketCount[endpoint] < usClientState->maxFifoPacketCount[endpoint];
}

///////////////////////////////////////////////////////////////////////////////////////////
/// TinyCLR USBClient API
///////////////////////////////////////////////////////////////////////////////////////////
const char* usbClientApiNames[TOTAL_USBCLIENT_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.STM32F4.UsbClientController\\0"
};

void TinyCLR_UsbClient_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_USBCLIENT_CONTROLLERS; i++) {
        if (usbClientStates[i].tableInitialized)
            continue;

        usbClientControllers[i].ApiInfo = &usbClientApi[i];
        usbClientControllers[i].Acquire = &TinyCLR_UsbClient_Acquire;
        usbClientControllers[i].Release = &TinyCLR_UsbClient_Release;
        usbClientControllers[i].OpenPipe = &TinyCLR_UsbClient_OpenPipe;
        usbClientControllers[i].ClosePipe = &TinyCLR_UsbClient_ClosePipe;
        usbClientControllers[i].WritePipe = &TinyCLR_UsbClient_WritePipe;
        usbClientControllers[i].ReadPipe = &TinyCLR_UsbClient_ReadPipe;
        usbClientControllers[i].FlushPipe = &TinyCLR_UsbClient_FlushPipe;
        usbClientControllers[i].SetDataReceivedHandler = &TinyCLR_UsbClient_SetDataReceivedHandler;
        usbClientControllers[i].SetVendorClassRequestHandler = &TinyCLR_UsbClient_SetVendorClassRequestHandler;
        usbClientControllers[i].SetDeviceDescriptor = &TinyCLR_UsbClient_SetDeviceDescriptor;
        usbClientControllers[i].GetBytesToWrite = &TinyCLR_UsbClient_GetBytesToWrite;
        usbClientControllers[i].GetBytesToRead = &TinyCLR_UsbClient_GetBytesToRead;
        usbClientControllers[i].ClearWriteBuffer = &TinyCLR_UsbClient_ClearWriteBuffer;
        usbClientControllers[i].ClearReadBuffer = &TinyCLR_UsbClient_ClearReadBuffer;
        usbClientControllers[i].GetWriteBufferSize = &TinyCLR_UsbClient_GetWriteBufferSize;
        usbClientControllers[i].GetReadBufferSize = &TinyCLR_UsbClient_GetReadBufferSize;
        usbClientControllers[i].SetWriteBufferSize = &TinyCLR_UsbClient_SetWriteBufferSize;
        usbClientControllers[i].SetReadBufferSize = &TinyCLR_UsbClient_SetReadBufferSize;

        usbClientApi[i].Author = "GHI Electronics, LLC";
        usbClientApi[i].Name = usbClientApiNames[i];
        usbClientApi[i].Type = TinyCLR_Api_Type::UsbClientController;
        usbClientApi[i].Version = 0;
        usbClientApi[i].Implementation = &usbClientControllers[i];
        usbClientApi[i].State = &usbClientStates[i];

        usbClientStates[i].controllerIndex = i;
        usbClientStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* TinyCLR_UsbClient_GetRequiredApi() {
    TinyCLR_UsbClient_EnsureTableInitialized();

    return &usbClientApi[0];
}

void TinyCLR_UsbClient_AddApi(const TinyCLR_Api_Manager* apiManager) {
    TinyCLR_UsbClient_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_USBCLIENT_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &usbClientApi[i]);
    }
}

TinyCLR_Result TinyCLR_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Controller* self, const TinyCLR_UsbClient_DeviceDescriptor* descriptor) {
    auto usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    memset(usClientState, 0, sizeof(UsClientState) * TOTAL_USBCLIENT_CONTROLLERS);

    memcpy(reinterpret_cast<uint8_t*>(&usClientState->deviceDescriptor), reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_DeviceDescriptor*>(descriptor)), sizeof(TinyCLR_UsbClient_DeviceDescriptor));

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Acquire(const TinyCLR_UsbClient_Controller* self) {
    auto usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (usClientState->initializeCount == 0) {
        TinyCLR_UsbClient_InitializeConfiguration(usClientState);

        usClientState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
        usClientState->deviceState = USB_DEVICE_STATE_UNINITIALIZED;
        usClientState->deviceStatus = USB_STATUS_DEVICE_SELF_POWERED;

        if (apiManager != nullptr) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

            usClientState->queues = reinterpret_cast<USB_PACKET64**>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint32_t)));
            usClientState->currentPacketOffset = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint8_t)));
            usClientState->isTxQueue = reinterpret_cast<bool*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(bool)));

            usClientState->fifoPacketIn = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint8_t)));
            usClientState->fifoPacketOut = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint8_t)));
            usClientState->fifoPacketCount = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint8_t)));
            usClientState->maxFifoPacketCount = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint8_t)));

            usClientState->pipes = reinterpret_cast<USB_PIPE_MAP*>(memoryManager->Allocate(memoryManager, usClientState->totalPipesCount * sizeof(USB_PIPE_MAP)));

            usClientState->controlEndpointBuffer = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, USB_ENDPOINT_CONTROL_BUFFER_SIZE));

            usClientState->endpointStatus = reinterpret_cast<uint16_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint16_t)));
            usClientState->maxEndpointsPacketSize = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usClientState->totalEndpointsCount * sizeof(uint8_t)));

            if (usClientState->queues == nullptr
                || usClientState->currentPacketOffset == nullptr
                || usClientState->isTxQueue == nullptr
                || usClientState->fifoPacketIn == nullptr
                || usClientState->fifoPacketOut == nullptr
                || usClientState->fifoPacketCount == nullptr
                || usClientState->maxFifoPacketCount == nullptr
                || usClientState->pipes == nullptr
                || usClientState->controlEndpointBuffer == nullptr
                || usClientState->maxEndpointsPacketSize == nullptr
                || usClientState->endpointStatus == nullptr) {
                if (usClientState->queues != nullptr)
                    memoryManager->Free(memoryManager, usClientState->queues);

                if (usClientState->currentPacketOffset != nullptr)
                    memoryManager->Free(memoryManager, usClientState->currentPacketOffset);

                if (usClientState->isTxQueue != nullptr)
                    memoryManager->Free(memoryManager, usClientState->isTxQueue);

                if (usClientState->fifoPacketIn != nullptr)
                    memoryManager->Free(memoryManager, usClientState->fifoPacketIn);

                if (usClientState->fifoPacketOut != nullptr)
                    memoryManager->Free(memoryManager, usClientState->fifoPacketOut);

                if (usClientState->fifoPacketCount != nullptr)
                    memoryManager->Free(memoryManager, usClientState->fifoPacketCount);

                if (usClientState->maxFifoPacketCount != nullptr)
                    memoryManager->Free(memoryManager, usClientState->maxFifoPacketCount);

                if (usClientState->pipes != nullptr)
                    memoryManager->Free(memoryManager, usClientState->pipes);

                if (usClientState->controlEndpointBuffer != nullptr)
                    memoryManager->Free(memoryManager, usClientState->controlEndpointBuffer);

                if (usClientState->endpointStatus != nullptr)
                    memoryManager->Free(memoryManager, usClientState->endpointStatus);

                if (usClientState->maxEndpointsPacketSize != nullptr)
                    memoryManager->Free(memoryManager, usClientState->maxEndpointsPacketSize);

                goto acquire_error;
            }

            // Reset buffer, make sure no random value in RAM after soft reset
            memset(reinterpret_cast<uint8_t*>(usClientState->queues), 0x00, usClientState->totalEndpointsCount * sizeof(uint32_t));
            memset(reinterpret_cast<uint8_t*>(usClientState->currentPacketOffset), 0x00, usClientState->totalEndpointsCount * sizeof(uint8_t));

            memset(reinterpret_cast<uint8_t*>(usClientState->fifoPacketIn), 0x00, usClientState->totalEndpointsCount * sizeof(uint8_t));
            memset(reinterpret_cast<uint8_t*>(usClientState->fifoPacketOut), 0x00, usClientState->totalEndpointsCount * sizeof(uint8_t));
            memset(reinterpret_cast<uint8_t*>(usClientState->fifoPacketCount), 0x00, usClientState->totalEndpointsCount * sizeof(uint8_t));
            memset(reinterpret_cast<uint8_t*>(usClientState->maxFifoPacketCount), 0x00, usClientState->totalEndpointsCount * sizeof(uint8_t));

            for (auto i = 0; i < usClientState->totalPipesCount; i++) {
                usClientState->pipes[i].RxEP = USB_ENDPOINT_NULL;
                usClientState->pipes[i].TxEP = USB_ENDPOINT_NULL;
            }

            for (auto i = 0; i < usClientState->totalEndpointsCount; i++) {
                usClientState->maxEndpointsPacketSize[i] = TinyCLR_UsbClient_GetEndpointSize(i);
                usClientState->maxFifoPacketCount[i] = usClientState->maxFifoPacketCountDefault;
            }

            usClientState->initialized = true;

            goto acquire_success;

        }

    acquire_error:
        return TinyCLR_Result::ArgumentNull;
    }

acquire_success:

    usClientState->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Release(const TinyCLR_UsbClient_Controller* self) {
    auto usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    if (usClientState->initializeCount == 0) return TinyCLR_Result::Success;

    usClientState->initializeCount--;

    if (usClientState->initializeCount == 0) {
        if (usClientState->initialized) {
            DISABLE_INTERRUPTS_SCOPED(irq);

            TinyCLR_UsbClient_Uninitialize(usClientState);

            usClientState->initialized = false;

            // for soft reboot allow the USB to be off for at least 100ms
            TinyCLR_UsbClient_Delay(100000); // 100ms

            if (apiManager != nullptr) {
                auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

                memoryManager->Free(memoryManager, usClientState->queues);
                memoryManager->Free(memoryManager, usClientState->currentPacketOffset);
                memoryManager->Free(memoryManager, usClientState->isTxQueue);

                memoryManager->Free(memoryManager, usClientState->fifoPacketIn);
                memoryManager->Free(memoryManager, usClientState->fifoPacketOut);
                memoryManager->Free(memoryManager, usClientState->fifoPacketCount);
                memoryManager->Free(memoryManager, usClientState->maxFifoPacketCount);

                memoryManager->Free(memoryManager, usClientState->pipes);

                memoryManager->Free(memoryManager, usClientState->controlEndpointBuffer);
                memoryManager->Free(memoryManager, usClientState->endpointStatus);
                memoryManager->Free(memoryManager, usClientState->maxEndpointsPacketSize);
            }
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_OpenPipe(const TinyCLR_UsbClient_Controller* self, uint8_t writeEndpoint, uint8_t readEndpoint, uint32_t& pipe) {
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    if (!usClientState->initialized)
        goto pipe_error;

    if (writeEndpoint < usClientState->totalEndpointsCount)
        usClientState->isTxQueue[writeEndpoint] = true;

    if (readEndpoint < usClientState->totalEndpointsCount)
        usClientState->isTxQueue[readEndpoint] = false;

    if (apiManager != nullptr) {
        for (auto i = 0; i < 2; i++) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));
            auto endpoint = (i == 0) ? writeEndpoint : readEndpoint;

            if (memoryManager != nullptr && endpoint < usClientState->totalEndpointsCount) {
                usClientState->queues[endpoint] = (USB_PACKET64*)memoryManager->Allocate(memoryManager, usClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

                memset(reinterpret_cast<uint8_t*>(usClientState->queues[endpoint]), 0x00, usClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

                TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);
            }
        }

        for (pipe = 0; pipe < usClientState->totalPipesCount; pipe++) {
            // The Pipe must be currently closed
            if (usClientState->pipes[pipe].RxEP == USB_ENDPOINT_NULL && usClientState->pipes[pipe].TxEP == USB_ENDPOINT_NULL)
                break;
        }

        if (pipe == usClientState->totalPipesCount)
            goto pipe_error;

        // All tests pass, assign the endpoints to the pipe
        usClientState->pipes[pipe].RxEP = readEndpoint;
        usClientState->pipes[pipe].TxEP = writeEndpoint;

        if (usClientState->currentState == USB_DEVICE_STATE_UNINITIALIZED) {
            TinyCLR_UsbClient_Initialize(usClientState);
        }

        return TinyCLR_Result::Success;
    }

pipe_error:
    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result TinyCLR_UsbClient_ClosePipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    if (!usClientState->initialized)
        return TinyCLR_Result::NotAvailable;

    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto i = 0; i < 2; i++) {
        // Close the Rx, Tx pipe
        int32_t endpoint = (i == 0) ? usClientState->pipes[pipe].RxEP : endpoint = usClientState->pipes[pipe].TxEP;

        if (endpoint != USB_ENDPOINT_NULL && usClientState->queues[endpoint] != nullptr) {
            TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);

            if (apiManager != nullptr) {
                auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

                if (usClientState->queues[endpoint] != nullptr)
                    memoryManager->Free(memoryManager, usClientState->queues[endpoint]);

                usClientState->queues[endpoint] = nullptr;
            }
        }
    }

    usClientState->pipes[pipe].RxEP = USB_ENDPOINT_NULL;
    usClientState->pipes[pipe].TxEP = USB_ENDPOINT_NULL;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_WritePipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, const uint8_t* data, size_t& length) {
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    if (!usClientState->initialized
        || usClientState->deviceState != USB_DEVICE_STATE_CONFIGURED
        || data == nullptr
        || length == 0) {
        return TinyCLR_Result::NotAvailable;
    }

    int32_t endpoint = usClientState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usClientState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    DISABLE_INTERRUPTS_SCOPED(irq);

    const uint8_t*      ptr = data;
    uint32_t            count = length;
    bool                Done = false;
    uint32_t            WaitLoopCnt = 0;
    int32_t             totWrite = 0;

    // This loop packetizes the data and sends it out.  All packets sent have
    // the maximum length for the given endpoint except for the last packet which
    // will always have less than the maximum length - even if the packet length
    // must be zero for this to occur.   This is done to comply with standard
    // USB bulk-mode transfers.
    while (!Done) {

        USB_PACKET64* Packet64 = nullptr;

        if (usClientState->fifoPacketCount[endpoint] < usClientState->maxFifoPacketCount[endpoint]) {
            Packet64 = &usClientState->queues[endpoint][usClientState->fifoPacketIn[endpoint]];

            usClientState->fifoPacketIn[endpoint]++;
            usClientState->fifoPacketCount[endpoint]++;

            if (usClientState->fifoPacketIn[endpoint] == usClientState->maxFifoPacketCount[endpoint])
                usClientState->fifoPacketIn[endpoint] = 0;
        }

        if (Packet64) {
            uint32_t max_move;

            if (count > usClientState->maxEndpointsPacketSize[endpoint])
                max_move = usClientState->maxEndpointsPacketSize[endpoint];
            else
                max_move = count;

            if (max_move) {
                memcpy(Packet64->Buffer, ptr, max_move);
            }

            // we are done when we send a non-full length packet
            if (max_move < usClientState->maxEndpointsPacketSize[endpoint]) {
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
                    TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);
                }

                goto done_write;
            }

            if (irq.IsDisabled()) // @todo - this really needs more checks to be totally valid
            {
                goto done_write;
            }

            if (usClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
                goto done_write;
            }

            TinyCLR_UsbClient_StartOutput(usClientState, endpoint);

            irq.Release();

            TinyCLR_UsbClient_Delay(50);

            irq.Acquire();
        }
    }

    // here we have a post-condition that IRQs are disabled for all paths through conditional block above
    if (usClientState->deviceState == USB_DEVICE_STATE_CONFIGURED) {
        TinyCLR_UsbClient_StartOutput(usClientState, endpoint);
    }

done_write:
    length = totWrite;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_ReadPipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, uint8_t* data, size_t& length) {
    int32_t endpoint;
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    if (!usClientState->initialized || usClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::NotAvailable;
    }

    endpoint = usClientState->pipes[pipe].RxEP;
    // If no Read side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usClientState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_PACKET64* Packet64 = nullptr;
    uint8_t*        ptr = reinterpret_cast<uint8_t*>(data);
    uint32_t        count = 0;
    uint32_t        remain = length;

    while (count < length) {
        uint32_t max_move;

        if (usClientState->fifoPacketCount[endpoint] > 0) {
            Packet64 = &usClientState->queues[endpoint][usClientState->fifoPacketOut[endpoint]];

            usClientState->fifoPacketCount[endpoint]--;
            usClientState->fifoPacketOut[endpoint]++;

            if (usClientState->fifoPacketOut[endpoint] == usClientState->maxFifoPacketCount[endpoint]) {
                usClientState->fifoPacketOut[endpoint] = 0;
            }
        }

        if (!Packet64) {
            TinyCLR_UsbClient_ClearEvent(usClientState, 1 << endpoint);
            break;
        }

        max_move = Packet64->Size - usClientState->currentPacketOffset[endpoint];
        if (remain < max_move) max_move = remain;

        memcpy(ptr, &Packet64->Buffer[usClientState->currentPacketOffset[endpoint]], max_move);

        usClientState->currentPacketOffset[endpoint] += max_move;
        ptr += max_move;
        count += max_move;
        remain -= max_move;

        /* if we're done with this packet, move onto the next */
        if (usClientState->currentPacketOffset[endpoint] == Packet64->Size) {
            usClientState->currentPacketOffset[endpoint] = 0;
            Packet64 = nullptr;

            TinyCLR_UsbClient_RxEnable(usClientState, endpoint);
        }
    }

    length = count;

    return TinyCLR_Result::Success;
}

#define USB_FLUSH_RETRY_COUNT 30
TinyCLR_Result TinyCLR_UsbClient_FlushPipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
    int32_t endpoint;
    int32_t retries = USB_FLUSH_RETRY_COUNT;
    int32_t queueCnt;
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    /* not configured, no data can go in or out */
    if (usClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::InvalidOperation;
    }

    endpoint = usClientState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usClientState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    queueCnt = usClientState->fifoPacketCount[endpoint];

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while (usClientState->fifoPacketCount[endpoint] > 0 && retries > 0) {
        TinyCLR_UsbClient_StartOutput(usClientState, endpoint);

        TinyCLR_UsbClient_Delay(queueCnt == usClientState->fifoPacketCount[endpoint] ? 100 : 0); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == usClientState->fifoPacketCount[endpoint]) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = usClientState->fifoPacketCount[endpoint];
    }

    if (retries <= 0)
        TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    TinyCLR_UsbClient_SetDataReceived = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetVendorClassRequestHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_RequestHandler handler) {
    TinyCLR_UsbClient_ProcessVendorClassRequest = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetGetDescriptorHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_RequestHandler handler) {
    TinyCLR_UsbClient_SetGetDescriptor = handler;

    return TinyCLR_Result::Success;
}

size_t TinyCLR_UsbClient_GetBytesToWrite(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].TxEP;

    return usClientState->fifoPacketCount[endpoint];
#else
    return 0;
#endif
}

size_t TinyCLR_UsbClient_GetBytesToRead(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].RxEP;

    return usClientState->fifoPacketCount[endpoint];
#else
    return 0;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_ClearWriteBuffer(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].TxEP;

    TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_ClearReadBuffer(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].RxEP;

    TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

size_t TinyCLR_UsbClient_GetWriteBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].TxEP;

    return usClientState->maxFifoPacketCount[endpoint];
#else
    return 0;
#endif
}

size_t TinyCLR_UsbClient_GetReadBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].RxEP;

    return usClientState->maxFifoPacketCount[endpoint];
#else
    return 0;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_SetWriteBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, size_t size) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].TxEP;

    if (usClientState->maxFifoPacketCount[endpoint] != size) {
        usClientState->maxFifoPacketCount[endpoint] = size;

        if (apiManager != nullptr) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

            if (usClientState->queues[endpoint] != nullptr) {
                // free if allocated
                memoryManager->Free(memoryManager, usClientState->queues[endpoint]);
            }

            // relocated
            usClientState->queues[endpoint] = (USB_PACKET64*)memoryManager->Allocate(memoryManager, usClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            memset(reinterpret_cast<uint8_t*>(usClientState->queues[endpoint]), 0x00, usClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);
        }
    }

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_SetReadBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, size_t size) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsClientState * usClientState = reinterpret_cast<UsClientState*>(self->ApiInfo->State);

    int32_t endpoint = usClientState->pipes[pipe].RxEP;

    if (usClientState->maxFifoPacketCount[endpoint] != size) {
        usClientState->maxFifoPacketCount[endpoint] = size;

        if (apiManager != nullptr) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

            if (usClientState->queues[endpoint] != nullptr) {
                // free if allocated
                memoryManager->Free(memoryManager, usClientState->queues[endpoint]);
            }

            // relocated
            usClientState->queues[endpoint] = (USB_PACKET64*)memoryManager->Allocate(memoryManager, usClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            memset(reinterpret_cast<uint8_t*>(usClientState->queues[endpoint]), 0x00, usClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            TinyCLR_UsbClient_ClearEndpoints(usClientState, endpoint);
        }
    }

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

void TinyCLR_UsbClient_Reset(int32_t controllerIndex) {
    UsClientState * usClientState = &usbClientStates[controllerIndex];

    for (auto pipe = 0; pipe < usClientState->totalPipesCount; pipe++) {
        TinyCLR_UsbClient_ClosePipe(&usbClientControllers[controllerIndex], pipe);
    }

    TinyCLR_UsbClient_Release(&usbClientControllers[controllerIndex]);

    usbClientStates[controllerIndex].tableInitialized = false;
    usbClientStates[controllerIndex].initializeCount = 0;
}