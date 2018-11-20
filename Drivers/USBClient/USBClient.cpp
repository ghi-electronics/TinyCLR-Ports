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
static UsbClientState usbClientStates[TOTAL_USBCLIENT_CONTROLLERS];

TinyCLR_UsbClient_DataReceivedHandler TinyCLR_UsbClient_SetDataReceivedEvent = nullptr;
TinyCLR_UsbClient_RequestHandler TinyCLR_UsbClient_ProcessVendorClassRequestEvent = nullptr;
TinyCLR_UsbClient_RequestHandler TinyCLR_UsbClient_SetGetDescriptorEvent = nullptr;

void TinyCLR_UsbClient_SetEvent(UsbClientState *usbClientState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t old_event = usbClientState->event;

    usbClientState->event |= event;

    if (old_event != usbClientState->event) {
        TinyCLR_UsbClient_SetDataReceivedEvent(nullptr, 0);
    }
}

void TinyCLR_UsbClient_ClearEvent(UsbClientState *usbClientState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    usbClientState->event &= ~event;
}

void TinyCLR_UsbClient_ClearQueues(UsbClientState *usbClientState, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto ifc = 0; ifc < usbClientState->deviceDescriptor.Configurations->InterfaceCount; ifc++) {

        TinyCLR_UsbClient_InterfaceDescriptor* ifcx = (TinyCLR_UsbClient_InterfaceDescriptor*)&usbClientState->deviceDescriptor.Configurations->Interfaces[ifc];

        if (ClrRxQueue) {
            for (auto endpoint = 0; endpoint < ifcx->EndpointCount; endpoint++) {
                if (usbClientState->queues[endpoint] == nullptr || usbClientState->isTxQueue[endpoint])
                    continue;

                TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);

                /* since this queue is now reset, we have room available for newly arrived packets */
                TinyCLR_UsbClient_RxEnable(usbClientState, endpoint);
            }
        }

        if (ClrTxQueue) {
            for (auto endpoint = 0; endpoint < ifcx->EndpointCount; endpoint++) {
                if (usbClientState->queues[endpoint] && usbClientState->isTxQueue[endpoint])
                    TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);
            }
        }
    }
}

void TinyCLR_UsbClient_StateCallback(UsbClientState* usbClientState) {
    if (usbClientState->currentState != usbClientState->deviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
        //Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
        // The RxQueue is clear when it is configured.
        if (USB_DEVICE_STATE_CONFIGURED == usbClientState->currentState) {
            TinyCLR_UsbClient_ClearQueues(usbClientState, true, true);
        }

        usbClientState->currentState = usbClientState->deviceState;

        switch (usbClientState->deviceState) {
        case USB_DEVICE_STATE_DETACHED:
            usbClientState->residualCount = 0;
            usbClientState->dataCallback = nullptr;

            break;
        case USB_DEVICE_STATE_CONFIGURED:
            /* whenever we enter the configured state, re-initialize all of the RxQueues */
            /* Txqueue has stored some data to be transmitted */
            TinyCLR_UsbClient_ClearQueues(usbClientState, true, false);
            break;
        }
    }
}

void TinyCLR_UsbClient_DataCallback(UsbClientState* usbClientState) {
    uint32_t length = __min(usbClientState->maxEndpointsPacketSize[0], usbClientState->residualCount);

    memcpy(usbClientState->ptrData, usbClientState->residualData, length);

    usbClientState->dataSize = length;
    usbClientState->residualData += length;
    usbClientState->residualCount -= length;

    if (length == usbClientState->maxEndpointsPacketSize[0]) {
        usbClientState->expected -= length;
    }
    else {
        usbClientState->expected = 0;
    }

    if (usbClientState->expected) {
        usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;
    }
    else {
        usbClientState->dataCallback = nullptr;
    }
}

uint8_t TinyCLR_UsbClient_HandleGetStatus(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    uint16_t* status;
    uint16_t  zero = 0;

    /* validate setup packet */
    if (Setup->Value != 0 || Setup->Length != 2) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        status = &usbClientState->deviceStatus;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        if (usbClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
            return USB_STATE_STALL;
        }

        status = &zero;
        break;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbClientState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index >= usbClientState->totalEndpointsCount) {
            return USB_STATE_STALL;
        }

        status = &usbClientState->endpointStatus[Setup->Index];
        break;

    default:
        return USB_STATE_STALL;
    }


    /* send requested status to host */
    usbClientState->residualData = reinterpret_cast<uint8_t*>(status);
    usbClientState->residualCount = 2;
    usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleClearFeature(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->Value != USB_FEATURE_DEVICE_REMOTE_WAKEUP)
            return USB_STATE_STALL;

        // Locate the deviceDescriptor descriptor
        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)TinyCLR_UsbClient_FindRecord(usbClientState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);

        if (Config && (Config->Attributes & USB_ATTRIBUTE_REMOTE_WAKEUP)) {
            usbClientState->deviceStatus &= ~USB_STATUS_DEVICE_REMOTE_WAKEUP;
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
        if (usbClientState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0)
            return USB_STATE_STALL;

        /* bit 0x80 designates direction, which we dont utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index == 0 || Setup->Index >= usbClientState->totalEndpointsCount)
            return USB_STATE_STALL;

        if (Setup->Value != USB_FEATURE_ENDPOINT_HALT)
            return USB_STATE_STALL;

        /* clear the halt feature */
        usbClientState->endpointStatus[Setup->Index] &= ~USB_STATUS_ENDPOINT_HALT;
        usbClientState->endpointStatusChange = Setup->Index;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbClientState->residualCount = 0;
    usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t TinyCLR_UsbClient_HandleSetFeature(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    TinyCLR_UsbClient_ConfigurationDescriptor * Config;
    uint8_t       retState;

    /* validate setup packet */
    if (Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    switch (USB_SETUP_RECIPIENT(Setup->RequestType)) {
    case USB_SETUP_RECIPIENT_DEVICE:
        // only support Remote wakeup
        if (Setup->Value != USB_FEATURE_DEVICE_REMOTE_WAKEUP) {
            return USB_STATE_STALL;
        }

        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)TinyCLR_UsbClient_FindRecord(usbClientState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);
        if (Config == nullptr)        // If the deviceDescriptor record could not be found
            return USB_STATE_STALL; // Something pretty serious is wrong

        if (Config->Attributes & USB_ATTRIBUTE_REMOTE_WAKEUP) {
            usbClientState->deviceStatus |= USB_STATUS_DEVICE_REMOTE_WAKEUP;
        }

        retState = USB_STATE_REMOTE_WAKEUP;
        break;

    case USB_SETUP_RECIPIENT_INTERFACE:
        /* there are no interface features to set */
        return USB_STATE_STALL;

    case USB_SETUP_RECIPIENT_ENDPOINT:
        if (usbClientState->deviceState == USB_DEVICE_STATE_ADDRESS && Setup->Index != 0) {
            return USB_STATE_STALL;
        }

        /* bit 0x80 designates direction, which we don't utilize in this calculation */
        Setup->Index &= 0x7F;

        if (Setup->Index == 0 || Setup->Index >= usbClientState->totalEndpointsCount) {
            return USB_STATE_STALL;
        }

        if (Setup->Value != USB_FEATURE_ENDPOINT_HALT) {
            return USB_STATE_STALL;
        }

        /* set the halt feature */
        usbClientState->endpointStatus[Setup->Index] |= USB_STATUS_ENDPOINT_HALT;
        usbClientState->endpointStatusChange = Setup->Index;
        retState = USB_STATE_STATUS;
        break;

    default:
        return USB_STATE_STALL;
    }

    /* send zero-length packet to tell host we're done */
    usbClientState->residualCount = 0;
    usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t TinyCLR_UsbClient_HandleSetAddress(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    /* validate setup packet */
    if (Setup->Value > 127 || Setup->Index != 0 || Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbClientState->deviceState >= USB_DEVICE_STATE_CONFIGURED) {
        return USB_STATE_STALL;
    }

    /* set address */
    usbClientState->address = Setup->Value;

    /* catch state changes */
    if (usbClientState->address == 0) {
        usbClientState->deviceState = USB_DEVICE_STATE_DEFAULT;
    }
    else {
        usbClientState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }

    TinyCLR_UsbClient_StateCallback(usbClientState);

    /* send zero-length packet to tell host we're done */
    usbClientState->residualCount = 0;
    usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t TinyCLR_UsbClient_HandleConfigurationRequests(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    const uint8_t* header;
    uint8_t       type;
    uint8_t       DescriptorIndex;
    int32_t offset = 0;

    auto controllerIndex = usbClientState->controllerIndex;

    /* this request is valid regardless of device state */
    type = ((Setup->Value & 0xFF00) >> 8);
    DescriptorIndex = (Setup->Value & 0x00FF);
    usbClientState->expected = Setup->Length;

    if (usbClientState->expected == 0) {
        // just return an empty Status packet
        usbClientState->residualCount = 0;
        usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;
        return USB_STATE_DATA;
    }

    // The very first GET_DESCRIPTOR command out of reset should always return at most maxEndpointsPacketSize[0] bytes.
    // After that, you can return as many as the host has asked.
    if (usbClientState->deviceState <= USB_DEVICE_STATE_DEFAULT) {
        if (usbClientState->firstGetDescriptor) {
            usbClientState->firstGetDescriptor = false;

            usbClientState->expected = __min(usbClientState->expected, usbClientState->maxEndpointsPacketSize[0]);
        }
    }

    usbClientState->residualData = nullptr;
    usbClientState->residualCount = 0;

    if (Setup->Request == USB_GET_DESCRIPTOR) {
        switch (type) {
        case USB_DEVICE_DESCRIPTOR_TYPE:
            header = TinyCLR_UsbClient_FindRecord(usbClientState, USB_DEVICE_DESCRIPTOR_TYPE, Setup);
            if (header) {
                auto size = 0;
                usbClientState->controlEndpointBuffer[1] = USB_DEVICE_DESCRIPTOR_TYPE;

                size += 2;

                memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), header, USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE);

                size += USB_DEVICE_DESCRIPTOR_STRUCTURE_SIZE;

                usbClientState->controlEndpointBuffer[0] = size;

                // ready to send
                usbClientState->residualData = usbClientState->controlEndpointBuffer;
                usbClientState->residualCount = __min(usbClientState->expected, size);
            }

            break;

        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            header = TinyCLR_UsbClient_FindRecord(usbClientState, USB_CONFIGURATION_DESCRIPTOR_TYPE, Setup);

            if (header) {

                usbClientState->controlEndpointBuffer[0] = (2 + USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE);
                usbClientState->controlEndpointBuffer[1] = USB_CONFIGURATION_DESCRIPTOR_TYPE;

                auto size = 2;

                // Parse configuration
                memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), header, USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE);
                size += USB_CONFIGURATION_DESCRIPTOR_STRUCTURE_SIZE;

                // Parse Vendor Class Descriptor
                for (auto vendor_id = 0; vendor_id < usbClientState->deviceDescriptor.Configurations->VendorClassDescriptorCount; vendor_id++) {
                    auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&usbClientState->deviceDescriptor.Configurations->VendorClassDescriptors[vendor_id];

                    usbClientState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                    usbClientState->controlEndpointBuffer[size + 1] = vendor->Type;

                    size += 2;
                    memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                    size += vendor->Length;
                }

                // Parse interfaces
                for (auto interface_id = 0; interface_id < usbClientState->deviceDescriptor.Configurations->InterfaceCount; interface_id++) {
                    auto interface_size_index = size;

                    usbClientState->controlEndpointBuffer[size + 0] = 2 + USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE;
                    usbClientState->controlEndpointBuffer[size + 1] = USB_INTERFACE_DESCRIPTOR_TYPE;

                    size += 2;
                    memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_InterfaceDescriptor*>(&usbClientState->deviceDescriptor.Configurations->Interfaces[interface_id])), USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE);
                    size += USB_INTERFACE_DESCRIPTOR_STRUCTURE_SIZE;

                    auto ifc = (TinyCLR_UsbClient_InterfaceDescriptor*)&usbClientState->deviceDescriptor.Configurations->Interfaces[interface_id];

                    // Parse Vendor Class Descriptor
                    for (auto vendor_id = 0; vendor_id < ifc->VendorClassDescriptorCount; vendor_id++) {
                        auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&ifc->VendorClassDescriptors[vendor_id];

                        usbClientState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                        usbClientState->controlEndpointBuffer[size + 1] = vendor->Type;

                        size += 2;
                        memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                        size += vendor->Length;
                    }

                    // Parse Endpoints
                    for (auto endpoint = 0; endpoint < ifc->EndpointCount; endpoint++) {
                        auto ep = (TinyCLR_UsbClient_EndpointDescriptor*)&ifc->Endpoints[endpoint];

                        usbClientState->controlEndpointBuffer[size + 0] = USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE + 2;
                        usbClientState->controlEndpointBuffer[size + 1] = USB_ENDPOINT_DESCRIPTOR_TYPE;

                        size += 2;
                        memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), reinterpret_cast<uint8_t*>(ep), USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE);
                        size += USB_ENDPOINT_DESCRIPTOR_STRUCTURE_SIZE;

                        // Parse Vendor Class Descriptor
                        for (auto vendor_id = 0; vendor_id < ep->VendorClassDescriptorCount; vendor_id++) {
                            auto vendor = (TinyCLR_UsbClient_VendorClassDescriptor*)&ep->VendorClassDescriptors[vendor_id];

                            usbClientState->controlEndpointBuffer[size + 0] = 2 + vendor->Length;
                            usbClientState->controlEndpointBuffer[size + 1] = vendor->Type;

                            size += 2;
                            memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[size]), vendor->Payload, vendor->Length);
                            size += vendor->Length;
                        }
                    }
                }

                // Update size
                usbClientState->controlEndpointBuffer[2] = (size >> 0) & 0xFF;
                usbClientState->controlEndpointBuffer[3] = (size >> 8) & 0xFF;

                // ready to send
                usbClientState->residualData = usbClientState->controlEndpointBuffer;
                usbClientState->residualCount = __min(usbClientState->expected, size);
            }
            break;

        case USB_STRING_DESCRIPTOR_TYPE:

            if (nullptr != (header = TinyCLR_UsbClient_FindRecord(usbClientState, USB_STRING_DESCRIPTOR_TYPE, Setup))) {
                TinyCLR_UsbClient_StringDescriptor* str = (TinyCLR_UsbClient_StringDescriptor*)header;

                auto size = 2 + str->Length * 2;

                usbClientState->controlEndpointBuffer[0] = size;
                usbClientState->controlEndpointBuffer[1] = USB_STRING_DESCRIPTOR_TYPE;

                memcpy(reinterpret_cast<uint8_t*>(&usbClientState->controlEndpointBuffer[2]), reinterpret_cast<uint8_t*>(const_cast<wchar_t*>(str->Data)), str->Length * 2);

                usbClientState->residualData = usbClientState->controlEndpointBuffer;
                usbClientState->residualCount = __min(usbClientState->expected, size);
            }

            break;

        default:

            if (TinyCLR_UsbClient_SetGetDescriptorEvent != nullptr) {

                const uint8_t* responsePayload;

                size_t responsePayloadLength = 0;

                if (TinyCLR_UsbClient_SetGetDescriptorEvent(&usbClientControllers[controllerIndex], Setup, responsePayload, responsePayloadLength, TinyCLR_UsbClient_Now()) == TinyCLR_Result::Success) {
                    memcpy(usbClientState->controlEndpointBuffer, reinterpret_cast<uint8_t*>(const_cast<uint8_t*>(responsePayload)), responsePayloadLength);
                    usbClientState->residualData = usbClientState->controlEndpointBuffer;
                    usbClientState->residualCount = __min(usbClientState->expected, responsePayloadLength);
                }
            }

            break;
        }
    }
    else {
        if (Setup->RequestType & (USB_REQUEST_TYPE_VENDOR | USB_REQUEST_TYPE_CLASS)) {
            const uint8_t* responsePayload;

            size_t responsePayloadLength = 0;

            if (TinyCLR_UsbClient_ProcessVendorClassRequestEvent(&usbClientControllers[controllerIndex], Setup, responsePayload, responsePayloadLength, TinyCLR_UsbClient_Now()) == TinyCLR_Result::Success) {
                memcpy(usbClientState->controlEndpointBuffer, reinterpret_cast<uint8_t*>(const_cast<uint8_t*>(responsePayload)), responsePayloadLength);

                usbClientState->residualData = usbClientState->controlEndpointBuffer;
                usbClientState->residualCount = __min(usbClientState->expected, responsePayloadLength);
            }

        }
    }

    // If the request was not recognized, the generic types should be searched
    if (usbClientState->residualData == nullptr) {
        return USB_STATE_STALL;
    }

    usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleGetConfiguration(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup) {
    /* validate setup packet */
    if (Setup->Value != 0 || Setup->Index != 0 || Setup->Length != 1) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    usbClientState->residualData = &usbClientState->configurationNum;
    usbClientState->residualCount = 1;
    usbClientState->expected = 1;
    usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t TinyCLR_UsbClient_HandleSetConfiguration(UsbClientState* usbClientState, TinyCLR_UsbClient_SetupPacket* Setup, bool DataPhase) {
    /* validate setup packet */
    if (Setup->Index != 0 || Setup->Length != 0) {
        return USB_STATE_STALL;
    }

    /* validate based on device state */
    if (usbClientState->deviceState == USB_DEVICE_STATE_DEFAULT) {
        return USB_STATE_STALL;
    }

    /* we only support one deviceDescriptor */
    if (Setup->Value > 1) {
        return USB_STATE_STALL;
    }

    usbClientState->configurationNum = Setup->Value;

    /* catch state changes */
    if (usbClientState->configurationNum == 0) {
        usbClientState->deviceState = USB_DEVICE_STATE_ADDRESS;
    }
    else {
        usbClientState->deviceState = USB_DEVICE_STATE_CONFIGURED;
    }

    TinyCLR_UsbClient_StateCallback(usbClientState);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        usbClientState->residualCount = 0;
        usbClientState->dataCallback = TinyCLR_UsbClient_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB deviceDescriptor records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const uint8_t* TinyCLR_UsbClient_FindRecord(UsbClientState* usbClientState, uint8_t marker, TinyCLR_UsbClient_SetupPacket * setup) {
    bool found = false;

    uint8_t* ptr = nullptr;

    switch (marker) {
    case USB_DEVICE_DESCRIPTOR_TYPE:
        ptr = reinterpret_cast<uint8_t*>(&usbClientState->deviceDescriptor);
        found = true;
        break;

    case USB_CONFIGURATION_DESCRIPTOR_TYPE:
        ptr = reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_ConfigurationDescriptor*>(usbClientState->deviceDescriptor.Configurations));
        found = true;
        break;
    case USB_STRING_DESCRIPTOR_TYPE:

        for (auto i = 0; i < usbClientState->deviceDescriptor.StringCount; i++) {
            auto stringDescriptor = (TinyCLR_UsbClient_StringDescriptor*)&usbClientState->deviceDescriptor.Strings[i];

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

uint8_t TinyCLR_UsbClient_ControlCallback(UsbClientState* usbClientState) {
    TinyCLR_UsbClient_SetupPacket* Setup;

    if (usbClientState->dataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (TinyCLR_UsbClient_SetupPacket*)usbClientState->ptrData;

    switch (Setup->Request) {
    case USB_GET_STATUS:
        return TinyCLR_UsbClient_HandleGetStatus(usbClientState, Setup);
    case USB_CLEAR_FEATURE:
        return TinyCLR_UsbClient_HandleClearFeature(usbClientState, Setup);
    case USB_SET_FEATURE:
        return TinyCLR_UsbClient_HandleSetFeature(usbClientState, Setup);
    case USB_SET_ADDRESS:
        return TinyCLR_UsbClient_HandleSetAddress(usbClientState, Setup);
    case USB_GET_CONFIGURATION:
        return TinyCLR_UsbClient_HandleGetConfiguration(usbClientState, Setup);
    case USB_SET_CONFIGURATION:
        return TinyCLR_UsbClient_HandleSetConfiguration(usbClientState, Setup, true);
    default:
        return TinyCLR_UsbClient_HandleConfigurationRequests(usbClientState, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(UsbClientState* usbClientState, int32_t endpoint, bool& disableRx) {
    USB_PACKET64* packet;

    if (usbClientState->fifoPacketCount[endpoint] == usbClientState->maxFifoPacketCount[endpoint]) {
        disableRx = true;

        return nullptr;
    }

    disableRx = false;

    packet = &usbClientState->queues[endpoint][usbClientState->fifoPacketIn[endpoint]];

    usbClientState->fifoPacketIn[endpoint]++;
    usbClientState->fifoPacketCount[endpoint]++;

    if (usbClientState->fifoPacketIn[endpoint] == usbClientState->maxFifoPacketCount[endpoint])
        usbClientState->fifoPacketIn[endpoint] = 0;

    TinyCLR_UsbClient_SetEvent(usbClientState, 1 << endpoint);

    return packet;
}

USB_PACKET64* TinyCLR_UsbClient_TxDequeue(UsbClientState* usbClientState, int32_t endpoint) {
    USB_PACKET64* packet;

    if (usbClientState->fifoPacketCount[endpoint] == 0) {
        return nullptr;
    }

    packet = &usbClientState->queues[endpoint][usbClientState->fifoPacketOut[endpoint]];

    usbClientState->fifoPacketCount[endpoint]--;
    usbClientState->fifoPacketOut[endpoint]++;

    if (usbClientState->fifoPacketOut[endpoint] == usbClientState->maxFifoPacketCount[endpoint])
        usbClientState->fifoPacketOut[endpoint] = 0;

    return packet;
}

void TinyCLR_UsbClient_ClearEndpoints(UsbClientState* usbClientState, int32_t endpoint) {
    usbClientState->fifoPacketIn[endpoint] = usbClientState->fifoPacketOut[endpoint] = usbClientState->fifoPacketCount[endpoint] = 0;
}

bool TinyCLR_UsbClient_CanReceivePackage(UsbClientState* usbClientState, int32_t endpoint) {
    return usbClientState->fifoPacketCount[endpoint] < usbClientState->maxFifoPacketCount[endpoint];
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
    auto usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    memset(usbClientState, 0, sizeof(UsbClientState) * TOTAL_USBCLIENT_CONTROLLERS);

    memcpy(reinterpret_cast<uint8_t*>(&usbClientState->deviceDescriptor), reinterpret_cast<uint8_t*>(const_cast<TinyCLR_UsbClient_DeviceDescriptor*>(descriptor)), sizeof(TinyCLR_UsbClient_DeviceDescriptor));

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Acquire(const TinyCLR_UsbClient_Controller* self) {
    auto usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (usbClientState->initializeCount == 0) {
        TinyCLR_UsbClient_InitializeConfiguration(usbClientState);

        usbClientState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
        usbClientState->deviceState = USB_DEVICE_STATE_UNINITIALIZED;
        usbClientState->deviceStatus = USB_STATUS_DEVICE_SELF_POWERED;

        if (apiManager != nullptr) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

            usbClientState->queues = reinterpret_cast<USB_PACKET64**>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint32_t)));
            usbClientState->currentPacketOffset = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint8_t)));
            usbClientState->isTxQueue = reinterpret_cast<bool*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(bool)));

            usbClientState->fifoPacketIn = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint8_t)));
            usbClientState->fifoPacketOut = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint8_t)));
            usbClientState->fifoPacketCount = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint8_t)));
            usbClientState->maxFifoPacketCount = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint8_t)));

            usbClientState->pipes = reinterpret_cast<USB_PIPE_MAP*>(memoryManager->Allocate(memoryManager, usbClientState->totalPipesCount * sizeof(USB_PIPE_MAP)));

            usbClientState->controlEndpointBuffer = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, USB_ENDPOINT_CONTROL_BUFFER_SIZE));

            usbClientState->endpointStatus = reinterpret_cast<uint16_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint16_t)));
            usbClientState->maxEndpointsPacketSize = reinterpret_cast<uint8_t*>(memoryManager->Allocate(memoryManager, usbClientState->totalEndpointsCount * sizeof(uint8_t)));

            if (usbClientState->queues == nullptr
                || usbClientState->currentPacketOffset == nullptr
                || usbClientState->isTxQueue == nullptr
                || usbClientState->fifoPacketIn == nullptr
                || usbClientState->fifoPacketOut == nullptr
                || usbClientState->fifoPacketCount == nullptr
                || usbClientState->maxFifoPacketCount == nullptr
                || usbClientState->pipes == nullptr
                || usbClientState->controlEndpointBuffer == nullptr
                || usbClientState->maxEndpointsPacketSize == nullptr
                || usbClientState->endpointStatus == nullptr) {
                if (usbClientState->queues != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->queues);

                if (usbClientState->currentPacketOffset != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->currentPacketOffset);

                if (usbClientState->isTxQueue != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->isTxQueue);

                if (usbClientState->fifoPacketIn != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->fifoPacketIn);

                if (usbClientState->fifoPacketOut != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->fifoPacketOut);

                if (usbClientState->fifoPacketCount != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->fifoPacketCount);

                if (usbClientState->maxFifoPacketCount != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->maxFifoPacketCount);

                if (usbClientState->pipes != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->pipes);

                if (usbClientState->controlEndpointBuffer != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->controlEndpointBuffer);

                if (usbClientState->endpointStatus != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->endpointStatus);

                if (usbClientState->maxEndpointsPacketSize != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->maxEndpointsPacketSize);

                goto acquire_error;
            }

            // Reset buffer, make sure no random value in RAM after soft reset
            memset(reinterpret_cast<uint8_t*>(usbClientState->queues), 0x00, usbClientState->totalEndpointsCount * sizeof(uint32_t));
            memset(reinterpret_cast<uint8_t*>(usbClientState->currentPacketOffset), 0x00, usbClientState->totalEndpointsCount * sizeof(uint8_t));

            memset(reinterpret_cast<uint8_t*>(usbClientState->fifoPacketIn), 0x00, usbClientState->totalEndpointsCount * sizeof(uint8_t));
            memset(reinterpret_cast<uint8_t*>(usbClientState->fifoPacketOut), 0x00, usbClientState->totalEndpointsCount * sizeof(uint8_t));
            memset(reinterpret_cast<uint8_t*>(usbClientState->fifoPacketCount), 0x00, usbClientState->totalEndpointsCount * sizeof(uint8_t));
            memset(reinterpret_cast<uint8_t*>(usbClientState->maxFifoPacketCount), 0x00, usbClientState->totalEndpointsCount * sizeof(uint8_t));

            for (auto i = 0; i < usbClientState->totalPipesCount; i++) {
                usbClientState->pipes[i].RxEP = USB_ENDPOINT_NULL;
                usbClientState->pipes[i].TxEP = USB_ENDPOINT_NULL;
            }

            for (auto i = 0; i < usbClientState->totalEndpointsCount; i++) {
                usbClientState->maxEndpointsPacketSize[i] = TinyCLR_UsbClient_GetEndpointSize(i);
                usbClientState->maxFifoPacketCount[i] = usbClientState->maxFifoPacketCountDefault;
            }

            usbClientState->initialized = true;

            goto acquire_success;

        }

    acquire_error:
        return TinyCLR_Result::ArgumentNull;
    }

acquire_success:

    usbClientState->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_Release(const TinyCLR_UsbClient_Controller* self) {
    auto usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    if (usbClientState->initializeCount == 0) return TinyCLR_Result::Success;

    usbClientState->initializeCount--;

    if (usbClientState->initializeCount == 0) {
        if (usbClientState->initialized) {
            DISABLE_INTERRUPTS_SCOPED(irq);

            TinyCLR_UsbClient_Uninitialize(usbClientState);

            usbClientState->initialized = false;

            // for soft reboot allow the USB to be off for at least 100ms
            TinyCLR_UsbClient_Delay(100000); // 100ms

            if (apiManager != nullptr) {
                auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

                memoryManager->Free(memoryManager, usbClientState->queues);
                memoryManager->Free(memoryManager, usbClientState->currentPacketOffset);
                memoryManager->Free(memoryManager, usbClientState->isTxQueue);

                memoryManager->Free(memoryManager, usbClientState->fifoPacketIn);
                memoryManager->Free(memoryManager, usbClientState->fifoPacketOut);
                memoryManager->Free(memoryManager, usbClientState->fifoPacketCount);
                memoryManager->Free(memoryManager, usbClientState->maxFifoPacketCount);

                memoryManager->Free(memoryManager, usbClientState->pipes);

                memoryManager->Free(memoryManager, usbClientState->controlEndpointBuffer);
                memoryManager->Free(memoryManager, usbClientState->endpointStatus);
                memoryManager->Free(memoryManager, usbClientState->maxEndpointsPacketSize);
            }
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_OpenPipe(const TinyCLR_UsbClient_Controller* self, uint8_t writeEndpoint, uint8_t readEndpoint, uint32_t& pipe) {
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    if (!usbClientState->initialized)
        goto pipe_error;

    if (writeEndpoint < usbClientState->totalEndpointsCount)
        usbClientState->isTxQueue[writeEndpoint] = true;

    if (readEndpoint < usbClientState->totalEndpointsCount)
        usbClientState->isTxQueue[readEndpoint] = false;

    if (apiManager != nullptr) {
        for (auto i = 0; i < 2; i++) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));
            auto endpoint = (i == 0) ? writeEndpoint : readEndpoint;

            if (memoryManager != nullptr && endpoint < usbClientState->totalEndpointsCount) {
                usbClientState->queues[endpoint] = (USB_PACKET64*)memoryManager->Allocate(memoryManager, usbClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

                memset(reinterpret_cast<uint8_t*>(usbClientState->queues[endpoint]), 0x00, usbClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

                TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);
            }
        }

        for (pipe = 0; pipe < usbClientState->totalPipesCount; pipe++) {
            // The Pipe must be currently closed
            if (usbClientState->pipes[pipe].RxEP == USB_ENDPOINT_NULL && usbClientState->pipes[pipe].TxEP == USB_ENDPOINT_NULL)
                break;
        }

        if (pipe == usbClientState->totalPipesCount)
            goto pipe_error;

        // All tests pass, assign the endpoints to the pipe
        usbClientState->pipes[pipe].RxEP = readEndpoint;
        usbClientState->pipes[pipe].TxEP = writeEndpoint;

        if (usbClientState->currentState == USB_DEVICE_STATE_UNINITIALIZED) {
            TinyCLR_UsbClient_Initialize(usbClientState);
        }

        return TinyCLR_Result::Success;
    }

pipe_error:
    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result TinyCLR_UsbClient_ClosePipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    if (!usbClientState->initialized)
        return TinyCLR_Result::NotAvailable;

    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto i = 0; i < 2; i++) {
        // Close the Rx, Tx pipe
        int32_t endpoint = (i == 0) ? usbClientState->pipes[pipe].RxEP : endpoint = usbClientState->pipes[pipe].TxEP;

        if (endpoint != USB_ENDPOINT_NULL && usbClientState->queues[endpoint] != nullptr) {
            TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);

            if (apiManager != nullptr) {
                auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

                if (usbClientState->queues[endpoint] != nullptr)
                    memoryManager->Free(memoryManager, usbClientState->queues[endpoint]);

                usbClientState->queues[endpoint] = nullptr;
            }
        }
    }

    usbClientState->pipes[pipe].RxEP = USB_ENDPOINT_NULL;
    usbClientState->pipes[pipe].TxEP = USB_ENDPOINT_NULL;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_WritePipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, const uint8_t* data, size_t& length) {
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    if (!usbClientState->initialized
        || usbClientState->deviceState != USB_DEVICE_STATE_CONFIGURED
        || data == nullptr
        || length == 0) {
        return TinyCLR_Result::NotAvailable;
    }

    int32_t endpoint = usbClientState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbClientState->queues[endpoint] == nullptr) {
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

        if (usbClientState->fifoPacketCount[endpoint] < usbClientState->maxFifoPacketCount[endpoint]) {
            Packet64 = &usbClientState->queues[endpoint][usbClientState->fifoPacketIn[endpoint]];

            usbClientState->fifoPacketIn[endpoint]++;
            usbClientState->fifoPacketCount[endpoint]++;

            if (usbClientState->fifoPacketIn[endpoint] == usbClientState->maxFifoPacketCount[endpoint])
                usbClientState->fifoPacketIn[endpoint] = 0;
        }

        if (Packet64) {
            uint32_t max_move;

            if (count > usbClientState->maxEndpointsPacketSize[endpoint])
                max_move = usbClientState->maxEndpointsPacketSize[endpoint];
            else
                max_move = count;

            if (max_move) {
                memcpy(Packet64->Buffer, ptr, max_move);
            }

            // we are done when we send a non-full length packet
            if (max_move < usbClientState->maxEndpointsPacketSize[endpoint]) {
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
                    TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);
                }

                goto done_write;
            }

            if (irq.IsDisabled()) // @todo - this really needs more checks to be totally valid
            {
                goto done_write;
            }

            if (usbClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
                goto done_write;
            }

            TinyCLR_UsbClient_StartOutput(usbClientState, endpoint);

            irq.Release();

            TinyCLR_UsbClient_Delay(50);

            irq.Acquire();
        }
    }

    // here we have a post-condition that IRQs are disabled for all paths through conditional block above
    if (usbClientState->deviceState == USB_DEVICE_STATE_CONFIGURED) {
        TinyCLR_UsbClient_StartOutput(usbClientState, endpoint);
    }

done_write:
    length = totWrite;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_ReadPipe(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, uint8_t* data, size_t& length) {
    int32_t endpoint;
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    if (!usbClientState->initialized || usbClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::NotAvailable;
    }

    endpoint = usbClientState->pipes[pipe].RxEP;
    // If no Read side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbClientState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    DISABLE_INTERRUPTS_SCOPED(irq);

    USB_PACKET64* Packet64 = nullptr;
    uint8_t*        ptr = reinterpret_cast<uint8_t*>(data);
    uint32_t        count = 0;
    uint32_t        remain = length;

    while (count < length) {
        uint32_t max_move;

        if (usbClientState->fifoPacketCount[endpoint] > 0) {
            Packet64 = &usbClientState->queues[endpoint][usbClientState->fifoPacketOut[endpoint]];

            usbClientState->fifoPacketCount[endpoint]--;
            usbClientState->fifoPacketOut[endpoint]++;

            if (usbClientState->fifoPacketOut[endpoint] == usbClientState->maxFifoPacketCount[endpoint]) {
                usbClientState->fifoPacketOut[endpoint] = 0;
            }
        }

        if (!Packet64) {
            TinyCLR_UsbClient_ClearEvent(usbClientState, 1 << endpoint);
            break;
        }

        max_move = Packet64->Size - usbClientState->currentPacketOffset[endpoint];
        if (remain < max_move) max_move = remain;

        memcpy(ptr, &Packet64->Buffer[usbClientState->currentPacketOffset[endpoint]], max_move);

        usbClientState->currentPacketOffset[endpoint] += max_move;
        ptr += max_move;
        count += max_move;
        remain -= max_move;

        /* if we're done with this packet, move onto the next */
        if (usbClientState->currentPacketOffset[endpoint] == Packet64->Size) {
            usbClientState->currentPacketOffset[endpoint] = 0;
            Packet64 = nullptr;

            TinyCLR_UsbClient_RxEnable(usbClientState, endpoint);
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
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    /* not configured, no data can go in or out */
    if (usbClientState->deviceState != USB_DEVICE_STATE_CONFIGURED) {
        return TinyCLR_Result::InvalidOperation;
    }

    endpoint = usbClientState->pipes[pipe].TxEP;
    // If no Write side to pipe (or if not yet open)
    if (endpoint == USB_ENDPOINT_NULL || usbClientState->queues[endpoint] == nullptr) {
        return TinyCLR_Result::NotAvailable;
    }

    queueCnt = usbClientState->fifoPacketCount[endpoint];

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while (usbClientState->fifoPacketCount[endpoint] > 0 && retries > 0) {
        TinyCLR_UsbClient_StartOutput(usbClientState, endpoint);

        TinyCLR_UsbClient_Delay(queueCnt == usbClientState->fifoPacketCount[endpoint] ? 100 : 0); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == usbClientState->fifoPacketCount[endpoint]) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = usbClientState->fifoPacketCount[endpoint];
    }

    if (retries <= 0)
        TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    TinyCLR_UsbClient_SetDataReceivedEvent = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetVendorClassRequestHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_RequestHandler handler) {
    TinyCLR_UsbClient_ProcessVendorClassRequestEvent = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result TinyCLR_UsbClient_SetGetDescriptorHandler(const TinyCLR_UsbClient_Controller* self, TinyCLR_UsbClient_RequestHandler handler) {
    TinyCLR_UsbClient_SetGetDescriptorEvent = handler;

    return TinyCLR_Result::Success;
}

size_t TinyCLR_UsbClient_GetBytesToWrite(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].TxEP;

    return usbClientState->fifoPacketCount[endpoint];
#else
    return 0;
#endif
}

size_t TinyCLR_UsbClient_GetBytesToRead(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].RxEP;

    return usbClientState->fifoPacketCount[endpoint];
#else
    return 0;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_ClearWriteBuffer(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].TxEP;

    TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_ClearReadBuffer(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].RxEP;

    TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

size_t TinyCLR_UsbClient_GetWriteBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].TxEP;

    return usbClientState->maxFifoPacketCount[endpoint];
#else
    return 0;
#endif
}

size_t TinyCLR_UsbClient_GetReadBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].RxEP;

    return usbClientState->maxFifoPacketCount[endpoint];
#else
    return 0;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_SetWriteBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, size_t size) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].TxEP;

    if (usbClientState->maxFifoPacketCount[endpoint] != size) {
        usbClientState->maxFifoPacketCount[endpoint] = size;

        if (apiManager != nullptr) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

            if (usbClientState->queues[endpoint] != nullptr) {
                // free if allocated
                memoryManager->Free(memoryManager, usbClientState->queues[endpoint]);
            }

            // relocated
            usbClientState->queues[endpoint] = (USB_PACKET64*)memoryManager->Allocate(memoryManager, usbClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            memset(reinterpret_cast<uint8_t*>(usbClientState->queues[endpoint]), 0x00, usbClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);
        }
    }

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

TinyCLR_Result TinyCLR_UsbClient_SetReadBufferSize(const TinyCLR_UsbClient_Controller* self, uint32_t pipe, size_t size) {
#if DEVICE_MEMORY_PROFILE_FACTOR > 5
    UsbClientState * usbClientState = reinterpret_cast<UsbClientState*>(self->ApiInfo->State);

    int32_t endpoint = usbClientState->pipes[pipe].RxEP;

    if (usbClientState->maxFifoPacketCount[endpoint] != size) {
        usbClientState->maxFifoPacketCount[endpoint] = size;

        if (apiManager != nullptr) {
            auto memoryManager = reinterpret_cast<const TinyCLR_Memory_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager));

            if (usbClientState->queues[endpoint] != nullptr) {
                // free if allocated
                memoryManager->Free(memoryManager, usbClientState->queues[endpoint]);
            }

            // relocated
            usbClientState->queues[endpoint] = (USB_PACKET64*)memoryManager->Allocate(memoryManager, usbClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            memset(reinterpret_cast<uint8_t*>(usbClientState->queues[endpoint]), 0x00, usbClientState->maxFifoPacketCount[endpoint] * sizeof(USB_PACKET64));

            TinyCLR_UsbClient_ClearEndpoints(usbClientState, endpoint);
        }
    }

    return TinyCLR_Result::Success;
#else
    return TinyCLR_Result::NotSupported;
#endif
}

void TinyCLR_UsbClient_Reset(int32_t controllerIndex) {
    UsbClientState * usbClientState = &usbClientStates[controllerIndex];

    for (auto pipe = 0; pipe < usbClientState->totalPipesCount; pipe++) {
        TinyCLR_UsbClient_ClosePipe(&usbClientControllers[controllerIndex], pipe);
    }

    TinyCLR_UsbClient_Release(&usbClientControllers[controllerIndex]);

    usbClientStates[controllerIndex].tableInitialized = false;
    usbClientStates[controllerIndex].initializeCount = 0;
}