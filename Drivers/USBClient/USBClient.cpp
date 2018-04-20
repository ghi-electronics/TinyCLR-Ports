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

TinyCLR_UsbClient_DataReceivedHandler UsbClient_DataReceivedHandler;

// usb fifo buffer
static int32_t usb_packet_fifo_in[CONCAT(DEVICE_TARGET, _USB_ENDPOINT_COUNT)];
static int32_t usb_packet_fifo_out[CONCAT(DEVICE_TARGET, _USB_ENDPOINT_COUNT)];
static int32_t usb_packet_fifo_count[CONCAT(DEVICE_TARGET, _USB_ENDPOINT_COUNT)];

uint8_t UsbClient_ControlDataBuffer[256];

USB_CONTROLLER_STATE usbClient_State[CONCAT(DEVICE_TARGET, _TOTAL_USB_CONTROLLERS)];

uint8_t USB_LanguageDescriptor[USB_LANGUAGE_DESCRIPTOR_SIZE] =
{
    USB_LANGUAGE_DESCRIPTOR_SIZE,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09, 0x04                      // U.S. English
};

void UsbClient_SetEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t old_event = usbState->event;

    usbState->event |= event;

    if (old_event != usbState->event) {
        UsbClient_DataReceivedHandler(nullptr);
    }
}

void UsbClient_ClearEvent(USB_CONTROLLER_STATE *usbState, uint32_t event) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    usbState->event &= ~event;
}

void UsbClient_ClearQueues(USB_CONTROLLER_STATE *usbState, bool ClrRxQueue, bool ClrTxQueue) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (ClrRxQueue) {
        for (int32_t endpoint = 0; endpoint < usbState->configuration.interfaceDescriptor->bNumEndpoints; endpoint++) {
            if (usbState->queues[endpoint] == nullptr || usbState->isTxQueue[endpoint])
                continue;

            UsbClient_ClearEndpoints(endpoint);

            /* since this queue is now reset, we have room available for newly arrived packets */
            CONCAT(DEVICE_TARGET, _UsbClient_RxEnable(usbState, endpoint));
        }
    }

    if (ClrTxQueue) {
        for (int32_t endpoint = 0; endpoint < usbState->configuration.interfaceDescriptor->bNumEndpoints; endpoint++) {
            if (usbState->queues[endpoint] && usbState->isTxQueue[endpoint])
                UsbClient_ClearEndpoints(endpoint);
        }
    }
}

void UsbClient_StateCallback(USB_CONTROLLER_STATE* usbState) {
    if (usbState->currentState != usbState->deviceState) {
        /* whenever we leave the configured state, re-initialize all of the queues */
        //Not necessary, as TxBuffer may hold any data and then send them out when it is configured again.
        // The RxQueue is clear when it is configured.
        if (USB_DEVICE_STATE_CONFIGURED == usbState->currentState) {
            UsbClient_ClearQueues(usbState, true, true);
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
            UsbClient_ClearQueues(usbState, true, false);
            break;
        }
    }
}

void UsbClient_DataCallback(USB_CONTROLLER_STATE* usbState) {
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
        usbState->dataCallback = UsbClient_DataCallback;
    }
    else {
        usbState->dataCallback = nullptr;
    }
}

uint8_t UsbClient_HandleGetStatus(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
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
    usbState->dataCallback = UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t UsbClient_HandleClearFeature(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
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
        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)UsbClient_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);

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
    usbState->dataCallback = UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t UsbClient_HandleSetFeature(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
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

        Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)UsbClient_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
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
    usbState->dataCallback = UsbClient_DataCallback;

    /* notify lower layer of status change */
    return retState;
}

uint8_t UsbClient_HandleSetAddress(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
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

    UsbClient_StateCallback(usbState);

    /* send zero-length packet to tell host we're done */
    usbState->residualCount = 0;
    usbState->dataCallback = UsbClient_DataCallback;

    /* notify hardware of address change */
    return USB_STATE_ADDRESS;
}

uint8_t UsbClient_HandleConfigurationRequests(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
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
        usbState->dataCallback = UsbClient_DataCallback;
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
            header = UsbClient_FindRecord(usbState, USB_DEVICE_DESCRIPTOR_MARKER, Setup);
            if (header) {
                const TinyCLR_UsbClient_DeviceDescriptor * device = (TinyCLR_UsbClient_DeviceDescriptor *)header;
                usbState->residualData = (uint8_t *)&device->bLength;      // Start of the device descriptor
                usbState->residualCount = __min(usbState->expected, device->bLength);
            }
            break;

        case USB_CONFIGURATION_DESCRIPTOR_TYPE:
            header = UsbClient_FindRecord(usbState, USB_CONFIGURATION_DESCRIPTOR_MARKER, Setup);
            if (header) {
                const TinyCLR_UsbClient_ConfigurationDescriptor * Config = (TinyCLR_UsbClient_ConfigurationDescriptor *)header;
                int32_t offset = 9;
                //usbState->residualData = (uint8_t *)&Config->bLength;
                //usbState->residualCount = __min(usbState->expected, Config->wTotalLength);
                memcpy(UsbClient_ControlDataBuffer, (uint8_t *)&Config->bLength, sizeof(TinyCLR_UsbClient_DescriptorHeader) + offset);

                TinyCLR_UsbClient_InterfaceDescriptor* interface = usbState->configuration.interfaceDescriptor;


                memcpy((uint8_t *)&UsbClient_ControlDataBuffer[offset], (uint8_t *)usbState->configuration.interfaceDescriptor, sizeof(TinyCLR_UsbClient_InterfaceDescriptor));

                offset += sizeof(TinyCLR_UsbClient_InterfaceDescriptor);



                for (auto i = 0; i < usbState->configuration.interfaceDescriptor->bNumEndpoints; i++) {
                    memcpy((uint8_t *)&UsbClient_ControlDataBuffer[offset], (uint8_t *)&usbState->configuration.endpointDescriptor[i], sizeof(TinyCLR_UsbClient_EndpointDescriptor));
                    offset += sizeof(TinyCLR_UsbClient_EndpointDescriptor);
                }



                // UsbClient_ControlDataBuffer

                usbState->residualData = (uint8_t *)UsbClient_ControlDataBuffer;
                usbState->residualCount = __min(usbState->expected, Config->wTotalLength);
            }
            break;

        case USB_STRING_DESCRIPTOR_TYPE:
            if (DescriptorIndex == 0)        // If host is requesting the language list
            {
                usbState->residualData = USB_LanguageDescriptor;
                usbState->residualCount = __min(usbState->expected, USB_LANGUAGE_DESCRIPTOR_SIZE);
            }
            else {
                if (nullptr != (header = UsbClient_FindRecord(usbState, USB_STRING_DESCRIPTOR_MARKER, Setup))) {
                    const TinyCLR_UsbClient_StringDescriptorHeader * string = (TinyCLR_UsbClient_StringDescriptorHeader *)header;
                    usbState->residualData = (uint8_t *)&string->bLength;
                    usbState->residualCount = __min(usbState->expected, string->bLength);
                }
            }
            break;

        default:
            break;
        }
    }

    // If the request was not recognized, the generic types should be searched
    if (usbState->residualData == nullptr) {
        if (nullptr != (header = UsbClient_FindRecord(usbState, USB_GENERIC_DESCRIPTOR_MARKER, Setup))) {
            usbState->residualData = (uint8_t *)header;
            usbState->residualData += sizeof(TinyCLR_UsbClient_GenericDescriptorHeader);       // ptrData is located right after the header
            usbState->residualCount = __min(usbState->expected, header->size - sizeof(TinyCLR_UsbClient_GenericDescriptorHeader));
        }
        else
            return USB_STATE_STALL;
    }

    usbState->dataCallback = UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t UsbClient_HandleGetConfiguration(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup) {
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
    usbState->dataCallback = UsbClient_DataCallback;

    return USB_STATE_DATA;
}

uint8_t UsbClient_HandleSetConfiguration(USB_CONTROLLER_STATE* usbState, USB_SETUP_PACKET* Setup, bool DataPhase) {
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

    UsbClient_StateCallback(usbState);

    if (DataPhase) {
        /* send zero-length packet to tell host we're done */
        usbState->residualCount = 0;
        usbState->dataCallback = UsbClient_DataCallback;
    }

    return USB_STATE_CONFIGURATION;
}

// Searches through the USB configuration records for the requested type
// Returns a pointer to the header information if found and nullptr if not
const TinyCLR_UsbClient_DescriptorHeader * UsbClient_FindRecord(USB_CONTROLLER_STATE* usbState, uint8_t marker, USB_SETUP_PACKET * setup) {
    bool found = false;

    TinyCLR_UsbClient_DescriptorHeader* ptr = nullptr;

    switch (marker) {
    case USB_DEVICE_DESCRIPTOR_MARKER:
        ptr = (TinyCLR_UsbClient_DescriptorHeader*)usbState->configuration.deviceDescriptor;
        found = true;
        break;

    case USB_CONFIGURATION_DESCRIPTOR_MARKER:
        ptr = (TinyCLR_UsbClient_DescriptorHeader*)usbState->configuration.configurationDescriptor;
        found = true;
        break;
    case USB_STRING_DESCRIPTOR_MARKER:
        if ((setup->wValue & 0x00FF) == OS_DESCRIPTOR_STRING_INDEX)
            return  (TinyCLR_UsbClient_DescriptorHeader*)usbState->configuration.OsStringDescriptor;

        ptr = (TinyCLR_UsbClient_DescriptorHeader*)usbState->configuration.stringsDescriptor;

        while (ptr != nullptr && ptr->marker == USB_STRING_DESCRIPTOR_MARKER) {
            if (ptr->iValue == (setup->wValue & 0x00FF)) {
                found = true;
                break;
            }

            uint8_t* next = (uint8_t*)ptr;

            next += ptr->size;
            ptr = (TinyCLR_UsbClient_DescriptorHeader*)next;
        }        
        break;
    case USB_GENERIC_DESCRIPTOR_MARKER:
        ptr = (setup->wIndex == USB_XCOMPATIBLE_OS_REQUEST) ? (TinyCLR_UsbClient_DescriptorHeader*)usbState->configuration.OsXCompatibleId : (TinyCLR_UsbClient_DescriptorHeader*)usbState->configuration.OsXProperty;
        found = true;
        break;
    }

    return found ? ptr : nullptr;
}

uint8_t UsbClient_ControlCallback(USB_CONTROLLER_STATE* usbState) {
    USB_SETUP_PACKET* Setup;

    if (usbState->dataSize == 0) {
        return USB_STATE_DONE;
    }

    Setup = (USB_SETUP_PACKET*)usbState->ptrData;

    switch (Setup->bRequest) {
    case USB_GET_STATUS:
        return UsbClient_HandleGetStatus(usbState, Setup);
    case USB_CLEAR_FEATURE:
        return UsbClient_HandleClearFeature(usbState, Setup);
    case USB_SET_FEATURE:
        return UsbClient_HandleSetFeature(usbState, Setup);
    case USB_SET_ADDRESS:
        return UsbClient_HandleSetAddress(usbState, Setup);
    case USB_GET_CONFIGURATION:
        return UsbClient_HandleGetConfiguration(usbState, Setup);
    case USB_SET_CONFIGURATION:
        return UsbClient_HandleSetConfiguration(usbState, Setup, true);
    default:
        return UsbClient_HandleConfigurationRequests(usbState, Setup);
    }

    return USB_STATE_STALL;
}

USB_PACKET64* UsbClient_RxEnqueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint, bool& disableRx) {
    USB_PACKET64* packet;

    if (usb_packet_fifo_count[endpoint] == usbState->packetFifoCount) {
        disableRx = true;

        return nullptr;
    }

    disableRx = false;

    packet = &usbState->queues[endpoint][usb_packet_fifo_in[endpoint]];

    usb_packet_fifo_in[endpoint]++;
    usb_packet_fifo_count[endpoint]++;

    if (usb_packet_fifo_in[endpoint] == usbState->packetFifoCount)
        usb_packet_fifo_in[endpoint] = 0;

    UsbClient_SetEvent(usbState, 1 << endpoint);

    return packet;
}

USB_PACKET64* UsbClient_TxDequeue(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    USB_PACKET64* packet;

    if (usb_packet_fifo_count[endpoint] == 0) {
        return nullptr;
    }

    packet = &usbState->queues[endpoint][usb_packet_fifo_out[endpoint]];

    usb_packet_fifo_count[endpoint]--;
    usb_packet_fifo_out[endpoint]++;

    if (usb_packet_fifo_out[endpoint] == usbState->packetFifoCount)
        usb_packet_fifo_out[endpoint] = 0;

    return packet;
}

void UsbClient_ClearEndpoints(int32_t endpoint) {
    usb_packet_fifo_in[endpoint] = usb_packet_fifo_out[endpoint] = usb_packet_fifo_count[endpoint] = 0;
}

bool UsbClient_CanReceivePackage(USB_CONTROLLER_STATE* usbState, int32_t endpoint) {
    return usb_packet_fifo_count[endpoint] < usbState->packetFifoCount;
}

///////////////////////////////////////////////////////////////////////////////////////////
/// TinyCLR USBClient API
///////////////////////////////////////////////////////////////////////////////////////////
static TinyCLR_UsbClient_Provider usbClientProvider;
static TinyCLR_Api_Info usbClientApi;

const TinyCLR_Api_Info* UsbClient_GetApi() {
    usbClientProvider.Parent = &usbClientApi;
    usbClientProvider.Index = 0;
    usbClientProvider.Acquire = &UsbClient_Acquire;
    usbClientProvider.Release = &UsbClient_Release;
    usbClientProvider.Open = &UsbClient_Open;
    usbClientProvider.Close = &UsbClient_Close;
    usbClientProvider.Write = &UsbClient_Write;
    usbClientProvider.Read = &UsbClient_Read;
    usbClientProvider.Flush = &UsbClient_Flush;
    usbClientProvider.SetDataReceivedHandler = &UsbClient_SetDataReceivedHandler;

    usbClientApi.Author = "GHI Electronics, LLC";
    usbClientApi.Name = "GHIElectronics.TinyCLR.NativeApis.Drivers.UsbClientProvider";
    usbClientApi.Type = TinyCLR_Api_Type::UsbClientProvider;
    usbClientApi.Version = 0;
    usbClientApi.Count = 1;
    usbClientApi.Implementation = &usbClientProvider;

    return &usbClientApi;
}

TinyCLR_Result UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClient_State[controller];

    DISABLE_INTERRUPTS_SCOPED(irq);

    memset(usbState, 0, sizeof(USB_CONTROLLER_STATE));

    usbState->currentState = USB_DEVICE_STATE_UNINITIALIZED;
    usbState->deviceStatus = USB_STATUS_DEVICE_SELF_POWERED;
    usbState->initialized = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result UsbClient_Release(const TinyCLR_UsbClient_Provider* self) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE *usbState = &usbClient_State[controller];

    if (usbState->initialized) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CONCAT(DEVICE_TARGET, _UsbClient_Uninitialize(usbState));

        usbState->initialized = false;

        // for soft reboot allow the USB to be off for at least 100ms
        CONCAT(DEVICE_TARGET, _Time_Delay(nullptr, 100000)); // 100ms
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t& pipe, TinyCLR_UsbClient_PipeMode mode) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    if (!usbState->initialized)
        return TinyCLR_Result::NotAvailable;

    int32_t writeEp = USB_ENDPOINT_NULL;
    int32_t readEp = USB_ENDPOINT_NULL;

    if (mode != TinyCLR_UsbClient_PipeMode::InOut)
        return TinyCLR_Result::NotSupported;

    if (usbState->currentState == USB_DEVICE_STATE_UNINITIALIZED) {
        CONCAT(DEVICE_TARGET, _UsbClient_Initialize(usbState));

        usbState->endpointCount = usbState->configuration.interfaceDescriptor->bNumEndpoints;
        usbState->packetSize = usbState->configuration.deviceDescriptor->bMaxPacketSize0;

        for (auto i = 0; i < CONCAT(DEVICE_TARGET, _USB_PIPE_COUNT); i++) {
            usbState->pipes[i].RxEP = USB_ENDPOINT_NULL;
            usbState->pipes[i].TxEP = USB_ENDPOINT_NULL;
        }
    }

    for (auto i = 0; i < usbState->configuration.interfaceDescriptor->bNumEndpoints; i++) {
        TinyCLR_UsbClient_EndpointDescriptor  *ep = (TinyCLR_UsbClient_EndpointDescriptor*)&usbState->configuration.endpointDescriptor[i];

        auto idx = 0;

        if ((ep->bEndpointAddress & USB_ENDPOINT_DIRECTION_IN) == USB_ENDPOINT_DIRECTION_IN) {
            writeEp = (ep->bEndpointAddress & (~USB_ENDPOINT_DIRECTION_IN));
            idx = writeEp;
            usbState->isTxQueue[idx] = true;
        }
        else if ((ep->bEndpointAddress & USB_ENDPOINT_DIRECTION_OUT) == USB_ENDPOINT_DIRECTION_OUT) {
            readEp = (ep->bEndpointAddress & (~USB_ENDPOINT_DIRECTION_OUT));
            idx = readEp;
            usbState->isTxQueue[idx] = false;
        }

        if (idx > 0) {
            if (apiProvider != nullptr) {
                auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

                usbState->queues[idx] = (USB_PACKET64*)memoryProvider->Allocate(memoryProvider, usbState->packetFifoCount * sizeof(USB_PACKET64));

                if (usbState->queues[idx] == nullptr)
                    return TinyCLR_Result::ArgumentNull;;

                memset(reinterpret_cast<uint8_t*>(usbState->queues[idx]), 0x00, usbState->packetFifoCount * sizeof(USB_PACKET64));
            }

            UsbClient_ClearEndpoints(idx);
            usbState->maxPacketSize[idx] = ep->wMaxPacketSize;
        }
    }

    for (pipe = 0; pipe < CONCAT(DEVICE_TARGET, _USB_PIPE_COUNT); pipe++) {
        // The Pipe must be currently closed
        if (usbState->pipes[pipe].RxEP == USB_ENDPOINT_NULL && usbState->pipes[pipe].TxEP == USB_ENDPOINT_NULL)
            break;
    }

    if (pipe == CONCAT(DEVICE_TARGET, _USB_PIPE_COUNT))
        return TinyCLR_Result::NotAvailable;

    // All tests pass, assign the endpoints to the pipe
    usbState->pipes[pipe].RxEP = readEp;
    usbState->pipes[pipe].TxEP = writeEp;

    return TinyCLR_Result::Success;
}

TinyCLR_Result UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t pipe) {
    int32_t controller = self->Index;

    USB_CONTROLLER_STATE * usbState = &usbClient_State[controller];

    if (!usbState->initialized)
        return TinyCLR_Result::NotAvailable;

    DISABLE_INTERRUPTS_SCOPED(irq);

    // Close the Rx pipe
    int32_t endpoint = usbState->pipes[pipe].RxEP;
    if (endpoint != USB_ENDPOINT_NULL && usbState->queues[endpoint]) {
        UsbClient_ClearEndpoints(endpoint);
    }

    usbState->pipes[pipe].RxEP = USB_ENDPOINT_NULL;

    // Close the TX pipe
    endpoint = usbState->pipes[pipe].TxEP;
    if (endpoint != USB_ENDPOINT_NULL && usbState->queues[endpoint] != nullptr) {
        UsbClient_ClearEndpoints(endpoint);

        if (apiProvider != nullptr) {
            auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

            if (usbState->queues[endpoint] != nullptr)
                memoryProvider->Free(memoryProvider, usbState->queues[endpoint]);

            usbState->queues[endpoint] = nullptr;
        }
    }

    usbState->pipes[pipe].TxEP = USB_ENDPOINT_NULL;

    return TinyCLR_Result::Success;
}

TinyCLR_Result UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t pipe, const uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

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
    int32_t                 totWrite = 0;

    // This loop packetizes the data and sends it out.  All packets sent have
    // the maximum length for the given endpoint except for the last packet which
    // will always have less than the maximum length - even if the packet length
    // must be zero for this to occur.   This is done to comply with standard
    // USB bulk-mode transfers.
    while (!Done) {

        USB_PACKET64* Packet64 = nullptr;

        if (usb_packet_fifo_count[endpoint] < usbState->packetFifoCount) {
            Packet64 = &usbState->queues[endpoint][usb_packet_fifo_in[endpoint]];

            usb_packet_fifo_in[endpoint]++;
            usb_packet_fifo_count[endpoint]++;

            if (usb_packet_fifo_in[endpoint] == usbState->packetFifoCount)
                usb_packet_fifo_in[endpoint] = 0;
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
                    UsbClient_ClearEndpoints(endpoint);
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

            CONCAT(DEVICE_TARGET, _UsbClient_StartOutput(usbState, endpoint));

            irq.Release();

            CONCAT(DEVICE_TARGET, _Time_Delay(nullptr, 50));

            irq.Acquire();
        }
    }

    // here we have a post-condition that IRQs are disabled for all paths through conditional block above
    if (usbState->deviceState == USB_DEVICE_STATE_CONFIGURED) {
        CONCAT(DEVICE_TARGET, _UsbClient_StartOutput(usbState, endpoint));
    }

done_write:
    length = totWrite;

    return TinyCLR_Result::Success;
}

TinyCLR_Result UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t pipe, uint8_t* data, size_t& length) {
    int32_t controller = self->Index;

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
    uint8_t*        ptr = (uint8_t*)data;
    uint32_t        count = 0;
    uint32_t        remain = length;

    while (count < length) {
        uint32_t max_move;

        if (usb_packet_fifo_count[endpoint] > 0) {
            Packet64 = &usbState->queues[endpoint][usb_packet_fifo_out[endpoint]];

            usb_packet_fifo_count[endpoint]--;
            usb_packet_fifo_out[endpoint]++;

            if (usb_packet_fifo_out[endpoint] == usbState->packetFifoCount) {
                usb_packet_fifo_out[endpoint] = 0;
            }
        }

        if (!Packet64) {
            UsbClient_ClearEvent(usbState, 1 << endpoint);
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

            CONCAT(DEVICE_TARGET, _UsbClient_RxEnable(usbState, endpoint));
        }
    }

    length = count;

    return TinyCLR_Result::Success;
}

#define USB_FLUSH_RETRY_COUNT 30
TinyCLR_Result UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t pipe) {
    int32_t controller = self->Index;

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

    queueCnt = usb_packet_fifo_count[endpoint];

    // interrupts were disabled or USB interrupt was disabled for whatever reason, so force the flush
    while (usb_packet_fifo_count[endpoint] > 0 && retries > 0) {
        CONCAT(DEVICE_TARGET, _UsbClient_StartOutput(usbState, endpoint));

        CONCAT(DEVICE_TARGET, _Time_Delay(nullptr, queueCnt == usb_packet_fifo_count[endpoint] ? 100 : 0)); // don't call Events_WaitForEventsXXX because it will turn off interrupts

        retries = (queueCnt == usb_packet_fifo_count[endpoint]) ? retries - 1 : USB_FLUSH_RETRY_COUNT;

        queueCnt = usb_packet_fifo_count[endpoint];
    }

    if (retries <= 0)
        UsbClient_ClearEndpoints(endpoint);

    return TinyCLR_Result::Success;
}

TinyCLR_Result UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler) {
    int32_t controller = self->Index;

    UsbClient_DataReceivedHandler = handler;

    return TinyCLR_Result::Success;
}

void UsbClient_Reset() {
    for (auto pipe = 0; pipe < CONCAT(DEVICE_TARGET, _USB_ENDPOINT_COUNT); pipe++) {
        UsbClient_Close(&usbClientProvider, pipe);
    }

    UsbClient_Release(&usbClientProvider);
}
