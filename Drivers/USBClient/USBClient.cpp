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

static TinyCLR_UsbClient_Controller usbClientProvider;
static TinyCLR_Api_Info usbClientApi;

TinyCLR_UsbClient_DataReceivedHandler TinyCLR_UsbClient_SetDataReceived;
TinyCLR_UsbClient_RequestHandler TinyCLR_UsbClient_ProcessVendorClassRequest = nullptr;
TinyCLR_UsbClient_RequestHandler TinyCLR_UsbClient_SetGetDescriptor = nullptr;

USB_CONTROLLER_STATE* usbClient_State;

void TinyCLR_UsbClient_SetEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t old_event = usbState->event;

    usbState->event |= event;

    if (old_event != usbState->event) {
        TinyCLR_UsbClient_SetDataReceived(nullptr, usbState->controllerNum);
    }
}

void TinyCLR_UsbClient_ClearEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    usbState->event &= ~event;
}

void TinyCLR_UsbClient_ClearQueues(USB_CONTROLLER_STATE *usbState, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto ifc = 0; ifc < usbState->deviceDescriptor.Configurations->InterfaceCount; ifc++) {

        TinyCLR_UsbClient_InterfaceDescriptor* ifcx = (TinyCLR_UsbClient_InterfaceDescriptor*)&usbState->deviceDescriptor.Configurations->Interfaces[ifc];

        if (ClrRxQueue) {
            for (auto endpoint = 0; endpoint < ifcx->EndpointCount; endpoint++) {
                if (usbState->queues[endpoint] == nullptr || usbState->isTxQueue[endpoint])
                    continue;

                TinyCLR_UsbClient_ClearEndpoints(usbState, endpoint);

                /* since this queue is now reset, we have room available for newly arrived packets */
                TinyCLR_UsbClient_RxEnable(usbState, endpoint);
            }
        }

        if (ClrTxQueue) {
            for (auto endpoint = 0; endpoint < ifcx->EndpointCount; endpoint++) {
                if (usbState->queues[endpoint] && usbState->isTxQueue[endpoint])
                    TinyCLR_UsbClient_ClearEndpoints(usbState, endpoint);
            }
        }
    }
}

void TinyCLR_UsbClient_StateCallback(USB_CONTROLLER_STATE* usbState) {
    if (usbState->currentState != usbState->deviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
        //Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
        // The RxQueue is clear when it is configured.
        if (USB_DEVICE_STATE_CONFIGURED == usbState->currentState) {
            TinyCLR_UsbClient_ClearQueues(usbState, true, true);
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
            TinyCLR_UsbClient_ClearQueues(usbState, true, false);
            break;
        }
    }
}

void TinyCLR_UsbClient_DataCallback(USB_CONTROLLER_STATE* usbState) {
    uint32_t length = __min(usbState->maxEndpointsPacketSize[0], usbState->residualCount);

    memcpy(usbState->ptrData, usbState->residualData, length);

    usbState->dataSize = length;
    usbState->residualData += length;
    usbState->residualCount -= length;

    if (length == usbState->maxEndpointsPacketSize[0]) {
        usbState->expected -= length;
    }
    else {
        usbState->expected = 0;
    }

    if (usbState->expected) {
        usbState->dataCallback = TinyCLR_UsbClient_DataCallback;
    }
    else {
        usbState->dataCallback = nullptr;
    }
}

uint8_t TinyCLR_UsbClient_HandleGetStatus(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup) {
    uint16_t* status;
    uint16_t  zero = 0;

    /* validate setup packet */
    if (Setup->Value != 0 || Setup->Length != 2) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
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
        if (usbState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index >= usbState->totalEndpointsCount) {
            return USB_STATE_STALL;
        }

        status = &usbState->endpointStatus[Setup->Index];
        break;

    default:
        return USB_STATE_STALL;
    }


    /* send requested status to host */
    usbState->residualData = reinterpret_cast<uint8_t*>(status);
    usbState->residualCount = 2;
    usbState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleClearFeature(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->Value != USB_FEATURE_DEVICE_REMOTE_WAKEUP)
            return USB_STATE_STALL;

        // Locate the deviceDescriptor descriptor
        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)TinyCLR_UsbClient_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);

        if (Config && (Config->Attributes & USB_ATTRIBUTE_REMOTE_WAKEUP)) {
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
        if (usbState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0)
            return USB_STATE_STALL;

        /* bit 0x80 designates direction, which we dont utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index == 0 || Setup->Index >= usbState->totalEndpointsCount)
            return USB_STATE_STALL;

        if (Setup->Value != USB_FEATURE_ENDPOINT_HALT)
            return USB_STATE_STALL;

        /* clear the halt feature */
        usbState->endpointStatus[Setup->Index] &= ~USB_STATUS_ENDPOINT_HALT;
        usbState->endpointStatusChange = Setup->Index;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t TinyCLR_UsbClient_HandleSetFeature(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->Value != USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
            return USB_STATE_STALL;
        }

        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)TinyCLR_UsbClient_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);
        if (Config == nullptr)        // If the deviceDescriptor record could not be found
            return USB_STATE_STALL; // Something pretty serious is wrong

        if (Config->Attributes & USB_ATTRIBUTE_REMOTE_WAKEUP) {
            usbState->deviceStatus |= USB_STATUS_DEVICE_REMOTE_WAKEUP;
        }

        retState = USB_STATE_REMOTE_WAKEUP;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        /* there are no interface features to set */
        return USB_STATE_STALL;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index == 0 || Setup->Index >= usbState->totalEndpointsCount) {
            return USB_STATE_STALL;
        }

        if (Setup->Value != USB_FEATURE_ENDPOINT_HALT) {
            return USB_STATE_STALL;
        }

        /* set the halt feature */
        usbState->endpointStatus[Setup->Index] |= USB_STATUS_ENDPOINT_HALT;
        usbState->endpointStatusChange = Setup->Index;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t TinyCLR_UsbClient_HandleSetAddress(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup) {
    /* validate setup packet */
    if (Setup->Value > 127 || Setup->Index != 0 || Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState >= USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    /* set address */
    usbState->address = Setup->Value;

    /* catch state changes */
    if (usbState->address == 0) {
        usbState->deviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else {
        usbState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }

    TinyCLR_UsbClient_StateCallback(usbState);

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t TinyCLR_UsbClient_HandleConfigurationRequests(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup) {
    const uint8_t* header;
    uint8_t       type;
    uint8_t       DescriptorIndex;
    int32_t offset = 0;

    /* this request is valid regardless of device state */
    type = ((Setup->Value & 0xFF00) >> 8);
    DescriptorIndex = (Setup->Value & 0x00FF);
    usbState->expected = Setup->Length;

    if (usbState->expected == 0) {
        // just return an empty Status packet
        usbState->residualCount = 0;
        usbState->dataCallback = TinyCLR_UsbClient_DataCallback;
        return USB_STATE_DATA;
    }

    // The very first GET_DESCRIPTOR command out of reset should always return at most maxEndpointsPacketSize[0] bytes.
    // After that, you can return as many as the host has asked.
    if (usbState->deviceState <= USB_DEVICE_STATE_DEFAULT) {
        if (usbState->firstGetDescriptor) {
            usbState->firstGetDescriptor = false;

            usbState->expected = __min(usbState->expected, usbState->maxEndpointsPacketSize[0]);
        }
    }

    usbState->residualData = nullptr;
    usbState->residualCount = 0;

    if (Setup->Request == USB_GET_DESCRIPTOR) {
        switch (type) {
        case USB_DEVICE_DESCRIPTOR_TYPE:
            header = TinyCLR_UsbClient_FindRecord(usbState, USB_DEVICE_DESCRIPTOR_TYPE, Setup);
            if (header) {
                auto size = 0;
                usbState->controlEndpointBuffer[1] = USB_DEVICE_DESCRIPTOR_TYPE;

                size += 2;

                memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), header, USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE);

                size += USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE;

                usbState->controlEndpointBuffer[0] = size;

                // ready to send
                usbState->residualData = usbState->controlEndpointBuffer;
                usbState->residualCount = __min(usbState->expected, size);
            }

            break;

        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            header = TinyCLR_UsbClient_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);

            if (header) {

                usbState->controlEndpointBuffer[0] = (2 + USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE);
                usbState->controlEndpointBuffer[1] = USB_CONFIGURATION_DESCRIPTOR_TYPE;

                auto size = 2;

                // Parse configuration
                memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), header, USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE);
                size += USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE;

                // Parse Vendor Class Descriptor
                for (auto vendor_id = 0; vendor_id < usbState->deviceDescriptor.Configurations->VendorClassDescriptorCount; vendor_id++) {
                    auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&usbState->deviceDescriptor.Configurations->VendorClassDescriptors[vendor_id];

                    usbState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                    usbState->controlEndpointBuffer[size + 1] = vendor->Type;

                    size += 2;
                    memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                    size += vendor->Length;
                }

                // Parse interfaces
                for (auto interface_id = 0; interface_id < usbState->deviceDescriptor.Configurations->InterfaceCount; interface_id++) {
                    auto interface_size_index = size;

                    usbState->controlEndpointBuffer[size + 0] = 2 + USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE;
                    usbState->controlEndpointBuffer[size + 1] = USB_INTERFACE_DESCRIPTOR_TYPE;

                    size += 2;
                    memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_InterfaceDescriptor*>(&usbState->deviceDescriptor.Configurations->Interfaces[interface_id])), USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE);
                    size += USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE;

                    auto ifc = (TinyCLR_UsbClient_InterfaceDescriptor*)&usbState->deviceDescriptor.Configurations->Interfaces[interface_id];

                    // Parse Vendor Class Descriptor
                    for (auto vendor_id = 0; vendor_id < ifc->VendorClassDescriptorCount; vendor_id++) {
                        auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&ifc->VendorClassDescriptors[vendor_id];

                        usbState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                        usbState->controlEndpointBuffer[size + 1] = vendor->Type;

                        size += 2;
                        memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                        size += vendor->Length;
                    }

                    // Parse Endpoints
                    for (auto endpoint = 0; endpoint < ifc->EndpointCount; endpoint++) {
                        auto ep = (TinyCLR_UsbClient_EndpointDescriptor*)&ifc->Endpoints[endpoint];

                        usbState->controlEndpointBuffer[size + 0] = USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE + 2;
                        usbState->controlEndpointBuffer[size + 1] = USB_ENDPOINT_DESCRIPTOR_TYPE;

                        size += 2;
                        memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), reinterpret_cast<uint8_t*>(ep), USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE);
                        size += USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE;

                        // Parse Vendor Class Descriptor
                        for (auto vendor_id = 0; vendor_id < ep->VendorClassDescriptorCount; vendor_id++) {
                            auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&ep->VendorClassDescriptors[vendor_id];

                            usbState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                            usbState->controlEndpointBuffer[size + 1] = vendor->Type;

                            size += 2;
                            memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                            size += vendor->Length;
                        }
                    }
                }

                // Update size
                usbState->controlEndpointBuffer[2] = (size >> 0) & 0xFF;
                usbState->controlEndpointBuffer[3] = (size >> 8) & 0xFF;

                // ready to send
                usbState->residualData = usbState->controlEndpointBuffer;
                usbState->residualCount = __min(usbState->expected, size);
            }
            break;

        case USB_STRING_DESCRIPTOR_TYPE:

            if (nullptr != (header = TinyCLR_UsbClient_FindRecord(usbState, USB_STRING_DESCRIPTOR_TYPE, Setup))) {
                TinyCLR_UsbClient_StringDescriptor* str = (TinyCLR_UsbClient_StringDescriptor*)header;

                auto size = 2 + str->Length * 2;

                usbState->controlEndpointBuffer[0] = size;
                usbState->controlEndpointBuffer[1] = USB_STRING_DESCRIPTOR_TYPE;

                memcpy(reinterpret_cast<uint8_t*>(&usbState->controlEndpointBuffer[2]), reinterpret_cast<uint8_t*>(const_cast<wchar_t*>(str->Data)), str->Length * 2);

                usbState->residualData = usbState->controlEndpointBuffer;
                usbState->residualCount = __min(usbState->expected, size);
            }

            break;

        default:

            if (TinyCLR_UsbClient_SetGetDescriptor != nullptr) {

                const uint8_t* responsePayload;

                size_t responsePayloadLength = 0;

                if (TinyCLR_UsbClient_SetGetDescriptor(&usbClientProvider, usbState->controllerNum, Setup, responsePayload, responsePayloadLength) == TinyCLR_Result::Success) {
                    memcpy(usbState->controlEndpointBuffer, reinterpret_cast<uint8_t*>(const_cast<uint8_t*>(responsePayload)), responsePayloadLength);
                    usbState->residualData = usbState->controlEndpointBuffer;
                    usbState->residualCount = __min(usbState->expected, responsePayloadLength);
                }
            }

            break;
        }
    }
    else {
        if (Setup->RequestType & (USB_REQUEST_TYPE_VENDOR | USB_REQUEST_TYPE_CLASS)) {
            const uint8_t* responsePayload;

            size_t responsePayloadLength = 0;

            if (TinyCLR_UsbClient_ProcessVendorClassRequest(&usbClientProvider, usbState->controllerNum, Setup, responsePayload, responsePayloadLength) == TinyCLR_Result::Success) {
                memcpy(usbState->controlEndpointBuffer, reinterpret_cast<uint8_t*>(const_cast<uint8_t*>(responsePayload)), responsePayloadLength);

                usbState->residualData = usbState->controlEndpointBuffer;
                usbState->residualCount = __min(usbState->expected, responsePayloadLength);
            }

        }
    }

    // If the request was not recognized, the generic types should be searched
    if (usbState->residualData == nullptr) {
        return USB_STATE_STALL;
    }

    usbState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleGetConfiguration(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup) {
    /* validate setup packet */
    if (Setup->Value != 0 || Setup->Index != 0 || Setup->Length != 1) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    usbState->residualData = &usbState->configurationNum;
    usbState->residualCount = 1;
    usbState->expected = 1;
    usbState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* usbState, TinyCLR_UsbClient_SetupPacket* Setup, bool DataPhase) {
    /* validate setup packet */
    if (Setup->Index != 0 || Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    /* we only support one deviceDescriptor */
    if (Setup->Value > 1) {
        return USB_STATE_STALL;
    }

    usbState->configurationNum = Setup->Value;

    /* catch state changes */
    if (usbState->configurationNum == 0) {
        usbState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }
    else {
        usbState->deviceState = USB_DEVICE_STATE_CONFIGURED;
    }

    TinyCLR_UsbClient_StateCallback(usbState);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        usbState->residualCount = 0;
        usbState->dataCallback = TinyCLR_UsbClient_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB deviceDescriptor records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const uint8_t* TinyCLR_UsbClient_FindRecord(USB_CONTROLLER_STATE* usbState, uint8_t marker, TinyCLR_UsbClient_SetupPacket * setup) {
    bool found = false;

    uint8_t* ptr = nullptr;

    switch (marker) {
    case USB_DEVICE_DESCRIPTOR_TYPE:
        ptr = reinterpret_cast<uint8_t*>(&usbState->deviceDescriptor);
        found = true;
        break;

    case USB_CONFIGURATION_DESCRIPTOR_TYPE:
        ptr = reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_ConfigurationDescriptor*>(usbState->deviceDescriptor.Configurations));
        found = true;
        break;
    case USB_STRING_DESCRIPTOR_TYPE:

        for (auto i = 0; i < usbState->deviceDescriptor.StringCount; i++) {
            auto stringDescriptor = (TinyCLR_UsbClient_StringDescriptor*)&usbState->deviceDescriptor.Strings[i];

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

uint8_t TinyCLR_UsbClient_ControlCallback(USB_CONTROLLER_STATE* usbState) {
    TinyCLR_UsbClient_SetupPacket* Setup;

    if (usbState->dataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (TinyCLR_UsbClient_SetupPacket*)usbState->ptrData;

    switch (Setup->Request) {
    case USB_GET_STATUS:
        return TinyCLR_UsbClient_HandleGetStatus(usbState, Setup);
    case USB_CLEAR_FEATURE:
        return TinyCLR_UsbClient_HandleClearFeature(usbState, Setup);
    case USB_SET_FEATURE:
        return TinyCLR_UsbClient_HandleSetFeature(usbState, Setup);
    case USB_SET_ADDRESS:
        return TinyCLR_UsbClient_HandleSetAddress(usbState, Setup);
    case USB_GET_CONFIGURATION:
        return TinyCLR_UsbClient_HandleGetConfiguration(usbState, Setup);
    case USB_SET_CONFIGURATION:
        return TinyCLR_UsbClient_HandleSetConfiguration(usbState, Setup, true);
    default:
        return TinyCLR_UsbClient_HandleConfigurationRequests(usbState, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint, bool& disableRx) {
    USB_PACKET64* packet;

    if (usbState->fifoPacketCount[endpoint] == usbState->maxFifoPacketCount) {
        disableRx = true;

        return nullptr;
    }

    disableRx = false;

    packet = &usbState->queues[endpoint][usbState->fifoPacketIn[endpoint]];

    usbState->fifoPacketIn[endpoint]++;
    usbState->fifoPacketCount[endpoint]++;

    if (usbState->fifoPacketIn[endpoint] == usbState->maxFifoPacketCount)
        usbState->fifoPacketIn[endpoint] = 0;

    TinyCLR_UsbClient_SetEvent(usbState, 1 << endpoint);

    return packet;
}

USB_PACKET64* TinyCLR_UsbClient_TxDequeue(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    USB_PACKET64* packet;

    if (usbState->fifoPacketCount[endpoint] == 0) {
        return nullptr;
    }

    packet = &usbState->queues[endpoint][usbState->fifoPacketOut[endpoint]];

    usbState->fifoPacketCount[endpoint]--;
    usbState->fifoPacketOut[endpoint]++;

    if (usbState->fifoPacketOut[endpoint] == usbState->maxFifoPacketCount)
        usbState->fifoPacketOut[endpoint] = 0;

    return packet;
}

void TinyCLR_UsbClient_ClearEndpoints(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    usbState->fifoPacketIn[endpoint] = usbState->fifoPacketOut[endpoint] = usbState->fifoPacketCount[endpoint] = 0;
}

bool TinyCLR_UsbClient_CanReceivePackage(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    return usbState->fifoPacketCount[endpoint] < usbState->maxFifoPacketCount;
}

///////////////////////////////////////////////////////////////////////////////////////////
/// TinyCLR USBClient API
///////////////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* TinyCLR_UsbClient_GetApi() {
    usbClientProvider.ApiInfo = &usbClientApi;
    usbClientProvider.Acquire = &TinyCLR_UsbClient_Acquire;
    usbClientProvider.Release = &TinyCLR_UsbClient_Release;
    usbClientProvider.Open = &TinyCLR_UsbClient_Open;
    usbClientProvider.Close = &TinyCLR_UsbClient_Close;
    usbClientProvider.Write = &TinyCLR_UsbClient_Write;
    usbClientProvider.Read = &TinyCLR_UsbClient_Read;
    usbClientProvider.Flush = &TinyCLR_UsbClient_Flush;
    usbClientProvider.SetDataReceivedHandler = &TinyCLR_UsbClient_SetDataReceivedHandler;
    usbClientProvider.SetVendorClassRequestHandler = &TinyCLR_UsbClient_SetVendorClassRequestHandler;
    usbClientProvider.SetDeviceDescriptor = &TinyCLR_UsbClient_SetDeviceDescriptor;
    usbClientProvider.GetControllerCount = &TinyCLR_UsbClient_GetControllerCount;

    usbClientApi.Author = "GHI Electronics, LLC";
    usbClientApi.Name = "GHIElectronics.TinyCLR.NativeApis.Drivers.UsbClientProvider";
    usbClientApi.Type = TinyCLR_Api_Type::UsbClientProvider;
    usbClientApi.Version = 0;
    usbClientApi.Implementation = &usbClientProvider;

    if (apiProvider != nullptr) {
        auto memoryProvider = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager));

        usbClient_State = reinterpret_cast<USB_CONTROLLER_STATE*>(memoryProvider->Allocate(memoryProvider, sizeof(USB_CONTROLLER_STATE)));
    }

    return &usbClientApi;
}

TinyCLR_Result TinyCLR_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Controller* self, int32_t controller, const TinyCLR_UsbClient_DeviceDescriptor* descriptor) {
    USB_CONTROLLER_STATE *usbState = &usbClient_State[controller];

    memset(usbState, 0, sizeof(USB_CONTROLLER_STATE));

    memcpy(reinterpret_cast<uint8_t*>(&usbState->deviceDescriptor), reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_DeviceDescriptor*>(descriptor)), sizeof(TinyCLR_UsbClient_DeviceDescriptor));

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Acquire(const TinyCLR_UsbClient_Controller* self, int32_t controller) {
    USB_CONTROLLER_STATE *usbState = &usbClient_State[controller];

    DISABLE_INTERRUPTS_SCOPED(irq);

    TinyCLR_UsbClient_InitializeConfiguration(usbState);

    usbState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
    usbState->deviceStatus = USB_STATUS_DEVICE_SELF_POWERED;

    if (apiProvider != nullptr) {
        auto memoryProvider = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager));

        usbState->queues = reinterpret_cast<USB_PACKET64**>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint32_t)));
        usbState->currentPacketOffset = reinterpret_cast<uint8_t*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint8_t)));
        usbState->isTxQueue = reinterpret_cast<bool*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(bool)));

        usbState->fifoPacketIn = reinterpret_cast<uint8_t*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint8_t)));
        usbState->fifoPacketOut = reinterpret_cast<uint8_t*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint8_t)));
        usbState->fifoPacketCount = reinterpret_cast<uint8_t*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint8_t)));

        usbState->pipes = reinterpret_cast<USB_PIPE_MAP*>(memoryProvider->Allocate(memoryProvider, usbState->totalPipesCount * sizeof(USB_PIPE_MAP)));

        usbState->controlEndpointBuffer = reinterpret_cast<uint8_t*>(memoryProvider->Allocate(memoryProvider, USB_ENDPOINT_CONTROL_BUFFER_SIZE));

        usbState->endpointStatus = reinterpret_cast<uint16_t*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint16_t)));
        usbState->maxEndpointsPacketSize = reinterpret_cast<uint8_t*>(memoryProvider->Allocate(memoryProvider, usbState->totalEndpointsCount * sizeof(uint8_t)));

        if (usbState->queues == nullptr
            || usbState->currentPacketOffset == nullptr
            || usbState->isTxQueue == nullptr
            || usbState->fifoPacketIn == nullptr
            || usbState->fifoPacketOut == nullptr
            || usbState->fifoPacketCount == nullptr
            || usbState->pipes == nullptr
            || usbState->controlEndpointBuffer == nullptr
            || usbState->maxEndpointsPacketSize == nullptr
            || usbState->endpointStatus == nullptr)
            goto acquire_error;

        // Reset buffer, make sure no random value in RAM after soft reset
        memset(reinterpret_cast<uint8_t*>(usbState->queues), 0x00, usbState->totalEndpointsCount * sizeof(uint32_t));
        memset(reinterpret_cast<uint8_t*>(usbState->currentPacketOffset), 0x00, usbState->totalEndpointsCount * sizeof(uint8_t));

        memset(reinterpret_cast<uint8_t*>(usbState->fifoPacketIn), 0x00, usbState->totalEndpointsCount * sizeof(uint8_t));
        memset(reinterpret_cast<uint8_t*>(usbState->fifoPacketOut), 0x00, usbState->totalEndpointsCount * sizeof(uint8_t));
        memset(reinterpret_cast<uint8_t*>(usbState->fifoPacketCount), 0x00, usbState->totalEndpointsCount * sizeof(uint8_t));

        for (auto i = 0; i < usbState->totalPipesCount; i++) {
            usbState->pipes[i].RxEP = USB_ENDPOINT_NULL;
            usbState->pipes[i].TxEP = USB_ENDPOINT_NULL;
        }

        for (auto i = 0; i < usbState->totalEndpointsCount; i++) {
            usbState->maxEndpointsPacketSize[i] = TinyCLR_UsbClient_GetEndpointSize(i);
        }

        usbState->initialized = true;

        return TinyCLR_Result::Success;

    }

acquire_error:
    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result TinyCLR_UsbClient_Release(const TinyCLR_UsbClient_Controller* self, int32_t controller) {
    USB_CONTROLLER_STATE *usbState = &usbClient_State[controller];

    if (usbState->initialized) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        TinyCLR_UsbClient_Uninitialize(usbState);

        usbState->initialized = false;

        // for soft reboot allow the USB to be off for at least 100ms
        TinyCLR_UsbClient_Delay(100000); // 100ms

        if (apiProvider != nullptr) {
            auto memoryProvider = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager));

            memoryProvider->Free(memoryProvider, usbState->queues);
            memoryProvider->Free(memoryProvider, usbState->currentPacketOffset);
            memoryProvider->Free(memoryProvider, usbState->isTxQueue);

            memoryProvider->Free(memoryProvider, usbState->fifoPacketIn);
            memoryProvider->Free(memoryProvider, usbState->fifoPacketOut);
            memoryProvider->Free(memoryProvider, usbState->fifoPacketCount);

            memoryProvider->Free(memoryProvider, usbState->pipes);

            memoryProvider->Free(memoryProvider, usbState->controlEndpointBuffer);
            memoryProvider->Free(memoryProvider, usbState->endpointStatus);
            memoryProvider->Free(memoryProvider, usbState->maxEndpointsPacketSize);
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Open(const TinyCLR_UsbClient_Controller* self, int32_t controller, int32_t& pipe, uint8_t writeEp, uint8_t readEp) {
    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    if (!usbState->initialized)
        goto pipe_error;

    if (writeEp < usbState->totalEndpointsCount)
        usbState->isTxQueue[writeEp] = true;

    if (readEp < usbState->totalEndpointsCount)
        usbState->isTxQueue[readEp] = false;

    if (apiProvider != nullptr) {
        for (auto i = 0; i < 2; i++) {
            auto memoryProvider = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager));
            auto endpoint = (i == 0) ? writeEp : readEp;

            if (memoryProvider != nullptr && endpoint < usbState->totalEndpointsCount) {
                usbState->queues[endpoint] = (USB_PACKET64*)memoryProvider->Allocate(memoryProvider, usbState->maxFifoPacketCount * sizeof(USB_PACKET64));

                memset(reinterpret_cast<uint8_t*>(usbState->queues[endpoint]), 0x00, usbState->maxFifoPacketCount * sizeof(USB_PACKET64));

                TinyCLR_UsbClient_ClearEndpoints(usbState, endpoint);
            }
        }

        for (pipe = 0; pipe < usbState->totalPipesCount; pipe++) {
            // The Pipe must be currently closed
            if (usbState->pipes[pipe].RxEP == USB_ENDPOINT_NULL && usbState->pipes[pipe].TxEP == USB_ENDPOINT_NULL)
                break;
        }

        if (pipe == usbState->totalPipesCount)
            goto pipe_error;

        // All tests pass, assign the endpoints to the pipe
        usbState->pipes[pipe].RxEP = readEp;
        usbState->pipes[pipe].TxEP = writeEp;

        if (usbState->currentState == USB_DEVICE_STATE_UNINITIALIZED) {
            TinyCLR_UsbClient_Initialize(usbState);
        }

        return TinyCLR_Result::Success;
    }

pipe_error:
    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result TinyCLR_UsbClient_Close(const TinyCLR_UsbClient_Controller* self, int32_t controller, int32_t pipe) {
    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    if (!usbState->initialized)
        return TinyCLR_Result::NotAvailable;

    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto i = 0; i < 2; i++) {
        // Close the Rx, Tx pipe
        int32_t endpoint = (i == 0) ? usbState->pipes[pipe].RxEP : endpoint = usbState->pipes[pipe].TxEP;

        if (endpoint != USB_ENDPOINT_NULL && usbState->queues[endpoint] != nullptr) {
            TinyCLR_UsbClient_ClearEndpoints(usbState, endpoint);

            if (apiProvider != nullptr) {
                auto memoryProvider = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager));

                if (usbState->queues[endpoint] != nullptr)
                    memoryProvider->Free(memoryProvider, usbState->queues[endpoint]);

                usbState->queues[endpoint] = nullptr;
            }
        }
    }

    usbState->pipes[pipe].RxEP = USB_ENDPOINT_NULL;
    usbState->pipes[pipe].TxEP = USB_ENDPOINT_NULL;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Write(const TinyCLR_UsbClient_Controller* self, int32_t controller, int32_t pipe, const uint8_t* data, size_t& length) {
    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    if (data == nullptr
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
    int32_t             totWrite = 0;

    // This loop packetizes the data and sends it out.  All packets sent have
    // the maximum length for the given endpoint except for the last packet which
    // will always have less than the maximum length - even if the packet length
    // must be zero for this to occur.   This is done to comply with standard
    // USB bulk-mode transfers.
    while (!Done) {

        USB_PACKET64* Packet64 = nullptr;

        if (usbState->fifoPacketCount[endpoint] < usbState->maxFifoPacketCount) {
            Packet64 = &usbState->queues[endpoint][usbState->fifoPacketIn[endpoint]];

            usbState->fifoPacketIn[endpoint]++;
            usbState->fifoPacketCount[endpoint]++;

            if (usbState->fifoPacketIn[endpoint] == usbState->maxFifoPacketCount)
                usbState->fifoPacketIn[endpoint] = 0;
        }

        if (Packet64) {
            uint32_t max_move;

            if (count > usbState->maxEndpointsPacketSize[endpoint])
                max_move = usbState->maxEndpointsPacketSize[endpoint];
            else
                max_move = count;

            if (max_move) {
                memcpy(Packet64->Buffer, ptr, max_move);
            }

            // we are done when we send a non-full length packet
            if (max_move < usbState->maxEndpointsPacketSize[endpoint]) {
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
                    TinyCLR_UsbClient_ClearEndpoints(usbState, endpoint);
                }

                goto done_write;
            }

            if (irq.IsDisabled()) // @todo - this really needs more checks to be totally valid
            {
                goto done_write;
            }

            if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
                goto done_write;
            }

            TinyCLR_UsbClient_StartOutput(usbState, endpoint);

            irq.Release();

            TinyCLR_UsbClient_Delay(50);

            irq.Acquire();
        }
    }

    // here we have a post-condition that IRQs are disabled for all paths through conditional block above
    if (usbState->deviceState == USB_DEVICE_STATE_CONFIGURED) {
        TinyCLR_UsbClient_StartOutput(usbState, endpoint);
    }

done_write:
    length = totWrite;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Read(const TinyCLR_UsbClient_Controller* self, int32_t controller, int32_t pipe, uint8_t* data, size_t& length) {
    int32_t endpoint;
    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    endpoint = usbState->pipes[pipe].RxEP;
    // If no Read side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_PACKET64* Packet64 = nullptr;
    uint8_t*        ptr = reinterpret_cast<uint8_t*>(data);
    uint32_t        count = 0;
    uint32_t        remain = length;

    while (count < length) {
        uint32_t max_move;

        if (usbState->fifoPacketCount[endpoint] > 0) {
            Packet64 = &usbState->queues[endpoint][usbState->fifoPacketOut[endpoint]];

            usbState->fifoPacketCount[endpoint]--;
            usbState->fifoPacketOut[endpoint]++;

            if (usbState->fifoPacketOut[endpoint] == usbState->maxFifoPacketCount) {
                usbState->fifoPacketOut[endpoint] = 0;
            }
        }

        if (!Packet64) {
            TinyCLR_UsbClient_ClearEvent(usbState, 1 << endpoint);
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

            TinyCLR_UsbClient_RxEnable(usbState, endpoint);
        }
    }

    length = count;

    return TinyCLR_Result::Success;
}

#define USB_FLUSH_RETRY_COUNT 30
TinyCLR_Result TinyCLR_UsbClient_Flush(const TinyCLR_UsbClient_Controller* self, int32_t controller, int32_t pipe) {
    int32_t endpoint;
    int32_t retries = USB_FLUSH_RETRY_COUNT;
    int32_t queueCnt;
    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    /* not configured, no data can go in or out */
    if (usbState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::InvalidOperation;
    }

    endpoint = usbState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    queueCnt = usbState->fifoPacketCount[endpoint];

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while (usbState->fifoPacketCount[endpoint] > 0 && retries > 0) {
        TinyCLR_UsbClient_StartOutput(usbState, endpoint);

        TinyCLR_UsbClient_Delay(queueCnt == usbState->fifoPacketCount[endpoint] ? 100 : 0); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == usbState->fifoPacketCount[endpoint]) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = usbState->fifoPacketCount[endpoint];
    }

    if (retries <= 0)
        TinyCLR_UsbClient_ClearEndpoints(usbState, endpoint);

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Controller* self, int32_t controller, TinyCLR_UsbClient_DataReceivedHandler handler) {
    TinyCLR_UsbClient_SetDataReceived = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetVendorClassRequestHandler(const TinyCLR_UsbClient_Controller* self, int32_t controller, TinyCLR_UsbClient_RequestHandler handler) {
    TinyCLR_UsbClient_ProcessVendorClassRequest = handler;

    return TinyCLR_Result::Success;
}


void TinyCLR_UsbClient_Reset(int32_t controller) {
    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    for (auto pipe = 0; pipe < usbState->totalPipesCount; pipe++) {
        TinyCLR_UsbClient_Close(&usbClientProvider, controller, pipe);
    }

    TinyCLR_UsbClient_Release(&usbClientProvider, controller);
}