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

#include <algorithm>
#include <string.h>
#include "AT91.h"

///////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN



struct AT91_CanData_T {
    uint32_t *matchFilters;
    uint32_t matchFiltersSize;

    uint32_t *lowerBoundFilters;
    uint32_t *upperBoundFilters;
    uint32_t groupFiltersSize;

}canData[2];


typedef struct {
    uint32_t timeStampL;
    uint32_t timeStampH;

    uint32_t msgId;

    bool extendedId;
    bool remoteTransmissionRequest;

    uint32_t dataA;
    uint32_t dataB;

    int32_t length;

} AT91_Can_Message;

struct AT91_Can_Controller {
    const TinyCLR_Can_Provider* provider;

    AT91_Can_Message *canRxMessagesFifo;

    TinyCLR_Can_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Can_MessageReceivedHandler    messageReceivedEventHandler;

    int32_t can_rx_count;
    int32_t can_rx_in;
    int32_t can_rx_out;

    int32_t can_max_messages_receiving;

    uint32_t baudrate;

};

static const AT91_Gpio_Pin g_AT91_Can_Tx_Pins[] = AT91_CAN_TX_PINS;
static const AT91_Gpio_Pin g_AT91_Can_Rx_Pins[] = AT91_CAN_RX_PINS;

static const int TOTAL_CAN_CONTROLLERS = SIZEOF_ARRAY(g_AT91_Can_Tx_Pins);

static AT91_Can_Controller canController[TOTAL_CAN_CONTROLLERS];

static TinyCLR_Can_Provider *canProvider[TOTAL_CAN_CONTROLLERS];
static TinyCLR_Api_Info canApi;

static uint8_t canProviderDefs[TOTAL_CAN_CONTROLLERS * sizeof(TinyCLR_Can_Provider)];

void CAN_DisableExplicitFilters(int32_t channel) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (canData[channel].matchFiltersSize && canData[channel].matchFilters != nullptr) {
        memoryProvider->Free(memoryProvider, canData[channel].matchFilters);

        canData[channel].matchFiltersSize = 0;
    }
}

void CAN_DisableGroupFilters(int32_t channel) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (canData[channel].groupFiltersSize) {
        if (canData[channel].lowerBoundFilters != nullptr)
            memoryProvider->Free(memoryProvider, canData[channel].lowerBoundFilters);

        if (canData[channel].upperBoundFilters != nullptr)
            memoryProvider->Free(memoryProvider, canData[channel].upperBoundFilters);

        canData[channel].groupFiltersSize = 0;
    }
}

/******************************************************************************
** Function name:        CAN_SetACCF_Lookup
**
** Descriptions:        Initialize CAN, install CAN interrupt handler
**
** parameters:            bitrate
** Returned value:        true or false, false if initialization failed.
**
******************************************************************************/
void CAN_SetACCF_Lookup(void) {
    
    return;
}

/******************************************************************************
** Function name:        CAN_SetACCF
**
** Descriptions:        Set acceptance filter and SRAM associated with
**
** parameters:            ACMF mode
** Returned value:        None
**
**
******************************************************************************/
void CAN_SetACCF(uint32_t ACCFMode) {
    
    return;
}

bool InsertionSort2CheckOverlap(uint32_t *lowerBounds, uint32_t *upperBounds, int32_t length) {

    uint32_t i, j, tmp, tmp2;

    for (i = 1; i < length; i++) {
        j = i;

        while (j > 0 && lowerBounds[j - 1] > lowerBounds[j]) {
            tmp = lowerBounds[j]; tmp2 = upperBounds[j];
            lowerBounds[j] = lowerBounds[j - 1];    upperBounds[j] = upperBounds[j - 1];
            lowerBounds[j - 1] = tmp;    upperBounds[j - 1] = tmp2;

            j--;
        }
    }

    // check for overlap
    if (lowerBounds[0] > upperBounds[0])
        return false;

    for (i = 1; i < length; i++) {
        if (lowerBounds[i] > upperBounds[i])
            return false;

        if (lowerBounds[i] <= upperBounds[i - 1])
            return false;
    }

    return true;
}

int32_t BinarySearch(uint32_t *sortedArray, int32_t first, int32_t last, uint32_t key) {
    int32_t mid;
    while (first <= last) {
        mid = (first + last) / 2;  // compute mid point.
        if (key > sortedArray[mid])
            first = mid + 1;  // repeat search in top half.
        else if (key < sortedArray[mid])
            last = mid - 1; // repeat search in bottom half.
        else
            return mid;     // found it. return position /////
    }

    return -1;    // failed to find key
}

int32_t BinarySearch2(uint32_t *lowerBounds, uint32_t *upperBounds, int32_t first, int32_t last, uint32_t key) {

    int32_t mid;

    while (first <= last) {
        mid = (first + last) / 2;  // compute mid point.

        if (key > upperBounds[mid])
            first = mid + 1;  // repeat search in top half.
        else if (key < lowerBounds[mid])
            last = mid - 1; // repeat search in bottom half.
        else
            return mid;     // found it. return position /////
    }

    return -1;    // failed to find key
}

const TinyCLR_Api_Info* AT91_Can_GetApi() {
    for (int i = 0; i < TOTAL_CAN_CONTROLLERS; i++) {
        canProvider[i] = (TinyCLR_Can_Provider*)(canProviderDefs + (i * sizeof(TinyCLR_Can_Provider)));
        canProvider[i]->Parent = &canApi;
        canProvider[i]->Index = i;
        canProvider[i]->Acquire = &AT91_Can_Acquire;
        canProvider[i]->Release = &AT91_Can_Release;
        canProvider[i]->Reset = &AT91_Can_Reset;
        canProvider[i]->WriteMessage = &AT91_Can_WriteMessage;
        canProvider[i]->ReadMessage = &AT91_Can_ReadMessage;
        canProvider[i]->SetTimings = &AT91_Can_SetTimings;
        canProvider[i]->GetUnReadMessageCount = &AT91_Can_GetUnReadMessageCount;
        canProvider[i]->SetMessageReceivedHandler = &AT91_Can_SetMessageReceivedHandler;
        canProvider[i]->SetErrorReceivedHandler = &AT91_Can_SetErrorReceivedHandler;
        canProvider[i]->SetExplicitFilters = &AT91_Can_SetExplicitFilters;
        canProvider[i]->SetGroupFilters = &AT91_Can_SetGroupFilters;
        canProvider[i]->DiscardUnreadMessages = &AT91_Can_DiscardUnreadMessages;
        canProvider[i]->IsSendingAllowed = &AT91_Can_IsSendingAllowed;
        canProvider[i]->GetReadErrorCount = &AT91_Can_GetReadErrorCount;
        canProvider[i]->GetWriteErrorCount = &AT91_Can_GetWriteErrorCount;
        canProvider[i]->GetSourceClock = &AT91_Can_GetSourceClock;
        canProvider[i]->SetReadBufferSize = &AT91_Can_SetReadBufferSize;
    }

    canApi.Author = "GHI Electronics, LLC";
    canApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.CanProvider";
    canApi.Type = TinyCLR_Api_Type::CanProvider;
    canApi.Version = 0;
    canApi.Count = TOTAL_CAN_CONTROLLERS;
    canApi.Implementation = canProvider;

    return &canApi;
}

uint32_t AT91_Can_GetLocalTime() {
    return AT91_Time_GetTimeForProcessorTicks(nullptr, AT91_Time_GetCurrentTicks(nullptr));
}

/******************************************************************************
** Function name:        CAN_ISR_Rx
**
** Descriptions:        CAN Rx1 interrupt handler
**
** parameters:            None
** Returned value:        None
**
******************************************************************************/
void CAN_ISR_Rx(int32_t channel) {
    // filter
    if (canData[channel].groupFiltersSize || canData[channel].matchFiltersSize) {
        
    }

    if (canController[channel].can_rx_count > (canController[channel].can_max_messages_receiving - 3)) {
        
    }

    // initialize destination pointer
    AT91_Can_Message *can_msg = &canController[channel].canRxMessagesFifo[canController[channel].can_rx_in];

    // timestamp
    uint64_t t = AT91_Can_GetLocalTime();

    can_msg->timeStampL = t & 0xFFFFFFFF;
    can_msg->timeStampH = t >> 32;

    uint32_t flag;
    uint32_t dataA;
    uint32_t dataB;
    uint32_t msgId;

    if (channel == 0) {
       
    }
    else {
        
    }

    can_msg->length = (flag >> 16) & 0x0F;

    can_msg->extendedId = ((flag & 0x80000000) != 0) ? true : false;

    can_msg->remoteTransmissionRequest = ((flag & 0x40000000) != 0) ? true : false;

    can_msg->msgId = msgId; // ID

    can_msg->dataA = dataA; // Data A

    can_msg->dataB = dataB; // Data B

    canController[channel].can_rx_count++;
    canController[channel].can_rx_in++;

    if (canController[channel].can_rx_in == canController[channel].can_max_messages_receiving) {
        canController[channel].can_rx_in = 0;
    }

    auto interop = (const TinyCLR_Interop_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider);

    canController[channel].messageReceivedEventHandler(interop, canController[channel].provider, canController[channel].can_rx_count);
}
void AT91_Can_RxInterruptHandler(void *param) {
    
}

TinyCLR_Result AT91_Can_Acquire(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index;

    if (!AT91_Gpio_OpenPin(g_AT91_Can_Tx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    if (!AT91_Gpio_OpenPin(g_AT91_Can_Rx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    // set pin as analog
    //AT91_Gpio_ConfigurePin(g_AT91_Can_Tx_Pins[channel].number, AT91_Gpio_Direction::Input, g_AT91_Can_Tx_Pins[channel].pinFunction, AT91_Gpio_ResistorMode::Inactive, AT91_Gpio_Hysteresis::Disable, AT91_Gpio_InputPolarity::NotInverted, AT91_Gpio_SlewRate::StandardMode, AT91_Gpio_OutputType::PushPull);
    //AT91_Gpio_ConfigurePin(g_AT91_Can_Rx_Pins[channel].number, AT91_Gpio_Direction::Input, g_AT91_Can_Rx_Pins[channel].pinFunction, AT91_Gpio_ResistorMode::Inactive, AT91_Gpio_Hysteresis::Disable, AT91_Gpio_InputPolarity::NotInverted, AT91_Gpio_SlewRate::StandardMode, AT91_Gpio_OutputType::PushPull);

    canController[channel].can_rx_count = 0;
    canController[channel].can_rx_in = 0;
    canController[channel].can_rx_out = 0;
    canController[channel].baudrate = 0;
    canController[channel].can_max_messages_receiving = AT91_CAN_RX_BUFFER_DEFAULT_SIZE;
    canController[channel].provider = self;

    canData[channel].matchFiltersSize = 0;
    canData[channel].groupFiltersSize = 0;

   

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_Release(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    TinyCLR_Result releasePin = AT91_Gpio_ReleasePin(nullptr, g_AT91_Can_Tx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    releasePin = AT91_Gpio_ReleasePin(nullptr, g_AT91_Can_Rx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    // free pin
    AT91_Gpio_ClosePin(g_AT91_Can_Tx_Pins[channel].number);
    AT91_Gpio_ClosePin(g_AT91_Can_Rx_Pins[channel].number);

    if (canController[channel].canRxMessagesFifo != nullptr)
        memoryProvider->Free(memoryProvider, canController[channel].canRxMessagesFifo);

    CAN_DisableExplicitFilters(channel);
    CAN_DisableGroupFilters(channel);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_Reset(const TinyCLR_Can_Provider* self) {
    int32_t channel = self->Index;

    canController[channel].can_rx_count = 0;
    canController[channel].can_rx_in = 0;
    canController[channel].can_rx_out = 0;

    // Reset CAN
    if (channel == 0) {
       
    }
    else {
       
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_WriteMessage(const TinyCLR_Can_Provider* self, uint32_t arbitrationId, bool extendedId, bool remoteTransmissionRequest, uint8_t* data, int32_t length) {

    uint32_t *canData = (uint32_t*)data;

    uint32_t flags = 0;
    uint32_t status;

    int32_t channel = self->Index;

    if (extendedId)
        flags |= 0x80000000;

    if (remoteTransmissionRequest)
        flags |= 0x40000000;

    flags |= (length & 0x0F) << 16;

    bool readyToSend = false;

    uint32_t timeout = CAN_TRANSFER_TIMEOUT;

    while (readyToSend == false && timeout > 0) {
        AT91_Can_IsSendingAllowed(self, readyToSend);
        timeout--;
    }

    if (timeout == 0)
        return TinyCLR_Result::Busy;

    if (channel == 0) {
        
    }
    else {
        
    }

    return TinyCLR_Result::Busy;
}

TinyCLR_Result AT91_Can_ReadMessage(const TinyCLR_Can_Provider* self, uint32_t& arbitrationId, bool& extendedId, bool& remoteTransmissionRequest, uint64_t& timestamp, uint8_t* data, int32_t& length) {
    AT91_Can_Message *can_msg;

    uint32_t *canData = (uint32_t*)data;

    int32_t channel = self->Index;

    if (canController[channel].can_rx_count) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        can_msg = &canController[channel].canRxMessagesFifo[canController[channel].can_rx_out];
        canController[channel].can_rx_out++;

        if (canController[channel].can_rx_out == canController[channel].can_max_messages_receiving)
            canController[channel].can_rx_out = 0;

        canController[channel].can_rx_count--;

        arbitrationId = can_msg->msgId;
        extendedId = can_msg->extendedId;
        remoteTransmissionRequest = can_msg->remoteTransmissionRequest;

        canData[0] = can_msg->dataA;
        canData[1] = can_msg->dataB;

        length = can_msg->length;

        timestamp = ((uint64_t)can_msg->timeStampL) | ((uint64_t)can_msg->timeStampH << 32);
    }

    return TinyCLR_Result::Success;

}

TinyCLR_Result AT91_Can_SetTimings(const TinyCLR_Can_Provider* self, int32_t propagation, int32_t phase1, int32_t phase2, int32_t brp, int32_t synchronizationJumpWidth, int8_t useMultiBitSampling) {
    int32_t channel = self->Index;

    AT91xx_SYSCON &SYSCON = *(AT91xx_SYSCON *)(size_t)(AT91xx_SYSCON::c_SYSCON_Base);

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    canController[channel].canRxMessagesFifo = (AT91_Can_Message*)memoryProvider->Allocate(memoryProvider, canController[channel].can_max_messages_receiving * sizeof(AT91_Can_Message));

    if (canController[channel].canRxMessagesFifo == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    canController[channel].baudrate = ((phase2 - 1) << 20) | ((phase1 - 1) << 16) | ((brp - 1) << 0);

    if (channel == 0) {
       
    }
    else {
       
    }

    AT91_Interrupt_Activate(0, (uint32_t*)&AT91_Can_RxInterruptHandler, 0);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_GetUnReadMessageCount(const TinyCLR_Can_Provider* self, size_t& count) {
    int32_t channel = self->Index;

    count = canController[channel].can_rx_count;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetMessageReceivedHandler(const TinyCLR_Can_Provider* self, TinyCLR_Can_MessageReceivedHandler handler) {
    int32_t channel = self->Index;

    canController[channel].messageReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetErrorReceivedHandler(const TinyCLR_Can_Provider* self, TinyCLR_Can_ErrorReceivedHandler handler) {
    int32_t channel = self->Index;

    canController[channel].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetExplicitFilters(const TinyCLR_Can_Provider* self, uint8_t* filters, int32_t length) {
    uint32_t *_matchFilters;
    uint32_t *filters32 = (uint32_t*)filters;

    int32_t channel = self->Index;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    _matchFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, length * sizeof(uint32_t));

    if (!_matchFilters)
        return TinyCLR_Result::OutOfMemory;

    memcpy(_matchFilters, filters32, length * sizeof(uint32_t));

    std::sort(_matchFilters, _matchFilters + length);

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CAN_DisableExplicitFilters(channel);

        canData[channel].matchFiltersSize = length;
        canData[channel].matchFilters = _matchFilters;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_SetGroupFilters(const TinyCLR_Can_Provider* self, uint8_t* lowerBounds, uint8_t* upperBounds, int32_t length) {
    uint32_t *_lowerBoundFilters, *_upperBoundFilters;
    uint32_t *lowerBounds32 = (uint32_t *)lowerBounds;
    uint32_t *upperBounds32 = (uint32_t *)upperBounds;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    int32_t channel = self->Index;

    _lowerBoundFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, length * sizeof(uint32_t));
    _upperBoundFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, length * sizeof(uint32_t));

    if (!_lowerBoundFilters || !_upperBoundFilters) {
        memoryProvider->Free(memoryProvider, _lowerBoundFilters);
        memoryProvider->Free(memoryProvider, _upperBoundFilters);

        return  TinyCLR_Result::OutOfMemory;
    }

    memcpy(_lowerBoundFilters, lowerBounds32, length * sizeof(uint32_t));
    memcpy(_upperBoundFilters, upperBounds32, length * sizeof(uint32_t));

    bool success = InsertionSort2CheckOverlap(_lowerBoundFilters, _upperBoundFilters, length);

    if (!success) {
        memoryProvider->Free(memoryProvider, _lowerBoundFilters);
        memoryProvider->Free(memoryProvider, _upperBoundFilters);

        return TinyCLR_Result::ArgumentInvalid;
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        CAN_DisableGroupFilters(channel);

        canData[channel].groupFiltersSize = length;
        canData[channel].lowerBoundFilters = _lowerBoundFilters;
        canData[channel].upperBoundFilters = _upperBoundFilters;
    }

    return TinyCLR_Result::Success;;
}

TinyCLR_Result AT91_Can_DiscardUnreadMessages(const TinyCLR_Can_Provider* self) {
    int32_t channel = self->Index;

    canController[channel].can_rx_count = 0;
    canController[channel].can_rx_in = 0;
    canController[channel].can_rx_out = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_IsSendingAllowed(const TinyCLR_Can_Provider* self, bool& allow) {
    int32_t channel = self->Index;

    uint32_t status = 0;

    allow = false;
   

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Can_GetReadErrorCount(const TinyCLR_Can_Provider* self, size_t& count) {
    int32_t channel = self->Index;

    count = channel == 0 ? 0 : 0;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result AT91_Can_GetWriteErrorCount(const TinyCLR_Can_Provider* self, size_t& count) {
    int32_t channel = self->Index;

     count = channel == 0 ? 0 : 0;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result AT91_Can_GetSourceClock(const TinyCLR_Can_Provider* self, uint32_t& sourceClock) {
    sourceClock = AT91_AHB_CLOCK_HZ / 2;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result AT91_Can_SetReadBufferSize(const TinyCLR_Can_Provider* self, size_t size) {
    int32_t channel = self->Index;

    if (size > 3) {
        canController[channel].can_max_messages_receiving = size;
        return TinyCLR_Result::Success;;
    }
    else {
        canController[channel].can_max_messages_receiving = AT91_CAN_RX_BUFFER_DEFAULT_SIZE;
        return TinyCLR_Result::ArgumentInvalid;;
    }
}

#endif // INCLUDE_CAN
