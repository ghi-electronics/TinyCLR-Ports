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
#include "LPC17.h"

///////////////////////////////////////////////////////////////////////////////

#ifdef INCLUDE_CAN

#define CAN_MESSAGES_MAX 128

#define CAN_TRANSFER_TIMEOUT 0xFFFF

struct LPC17_CanData_T {
    uint32_t *matchFilters;
    uint32_t matchFiltersSize;

    uint32_t *lowerBoundFilters;
    uint32_t *upperBoundFilters;
    uint32_t groupFiltersSize;

}canData[2];


typedef struct {
    uint32_t TimeStampL;
    uint32_t TimeStampH;

    uint32_t Frame; // Bits 16..19: DLC - Data Length Counter
                    // Bit 30: Set if this is a RTR message
                    // Bit 31: Set if this is a 29-bit ID message
    uint32_t MsgID;	// CAN Message ID (11-bit or 29-bit)
    uint32_t DataA;	// CAN Message Data Bytes 0-3
    uint32_t DataB;	// CAN Message Data Bytes 4-7

} LPC17_Can_Message;

enum class LPC17_Can_Error : uint32_t {
    OverRun,
    RxOver,
    BusOff,
    ErrorPassive,
    dataReveived = 0xFF,
};

struct LPC17_Can_Controller {
    const TinyCLR_Can_Provider* provider;

    LPC17_Can_Message *canRxMessagesFifo;

    TinyCLR_Can_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Can_MessageReceivedHandler    messageReceivedEventHandler;

    int32_t can_rx_count;
    int32_t can_rx_in;
    int32_t can_rx_out;

    int32_t can_max_messages_receiving;

    uint32_t baudrate;

};

static const LPC17_Gpio_Pin g_LPC17_Can_Tx_Pins[] = LPC17_CAN_TX_PINS;
static const LPC17_Gpio_Pin g_LPC17_Can_Rx_Pins[] = LPC17_CAN_RX_PINS;

static const int TOTAL_CAN_CONTROLLERS = SIZEOF_ARRAY(g_LPC17_Can_Tx_Pins);

static LPC17_Can_Controller canController[TOTAL_CAN_CONTROLLERS];

static TinyCLR_Can_Provider *canProvider[TOTAL_CAN_CONTROLLERS];
static TinyCLR_Api_Info canApi;

static uint8_t canProviderDefs[TOTAL_CAN_CONTROLLERS * sizeof(TinyCLR_Can_Provider)];

void InsertionSort(uint32_t *arr, int32_t length) {
    uint32_t i, j, tmp;

    for (i = 1; i < length; i++) {
        j = i;

        while (j > 0 && arr[j - 1] > arr[j]) {
            tmp = arr[j];
            arr[j] = arr[j - 1];
            arr[j - 1] = tmp;

            j--;
        }
    }
}

bool InsertionSort2CheckOverlap(uint32_t *lowerBounds, uint32_t *upperBounds, int32_t length) {

    uint32_t i, j, tmp, tmp2;

    for (i = 1; i < length; i++) {
        j = i;

        while (j > 0 && lowerBounds[j - 1] > lowerBounds[j]) {
            tmp = lowerBounds[j]; tmp2 = upperBounds[j];
            lowerBounds[j] = lowerBounds[j - 1];	upperBounds[j] = upperBounds[j - 1];
            lowerBounds[j - 1] = tmp;	upperBounds[j - 1] = tmp2;

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

const TinyCLR_Api_Info* LPC17_Can_GetApi() {
    for (int i = 0; i < TOTAL_CAN_CONTROLLERS; i++) {
        canProvider[i] = (TinyCLR_Can_Provider*)(canProviderDefs + (i * sizeof(TinyCLR_Can_Provider)));
        canProvider[i]->Parent = &canApi;
        canProvider[i]->Index = i;
        canProvider[i]->Acquire = &LPC17_Can_Acquire;
        canProvider[i]->Release = &LPC17_Can_Release;
        canProvider[i]->Reset = &LPC17_Can_Reset;
        canProvider[i]->PostMessage = &LPC17_Can_PostMessage;
        canProvider[i]->GetMessage = &LPC17_Can_GetMessage;
        canProvider[i]->SetSpeed = &LPC17_Can_SetSpeed;
        canProvider[i]->GetMessageCount = &LPC17_Can_GetMessageCount;
        canProvider[i]->SetMessageReceivedHandler = &LPC17_Can_SetMessageReceivedHandler;
        canProvider[i]->SetErrorReceivedHandler = &LPC17_Can_SetErrorReceivedHandler;
        canProvider[i]->SetExplicitFilters = &LPC17_Can_SetExplicitFilters;
        canProvider[i]->SetGroupFilters = &LPC17_Can_SetGroupFilters;
        canProvider[i]->DiscardIncomingMessages = &LPC17_Can_DiscardIncomingMessages;
        canProvider[i]->TransmissionAllowed = &LPC17_Can_TransmissionAllowed;
        canProvider[i]->ReceiveErrorCount = &LPC17_Can_ReceiveErrorCount;
        canProvider[i]->TransmitErrorCount = &LPC17_Can_TransmitErrorCount;
        canProvider[i]->GetSourceClock = &LPC17_Can_GetSourceClock;
        canProvider[i]->SetReceiveBufferSize = &LPC17_Can_SetReceiveBufferSize;
    }

    canApi.Author = "GHI Electronics, LLC";
    canApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.CanProvider";
    canApi.Type = TinyCLR_Api_Type::CanProvider;
    canApi.Version = 0;
    canApi.Count = TOTAL_CAN_CONTROLLERS;
    canApi.Implementation = canProvider;

    return &canApi;
}

uint32_t STM32_Can_GetLocalTime() {
    return LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentTicks(nullptr));
}
void STM32_Can_RxInterruptHandler(int32_t channel) {

}

void LPC17_Can_TxInterruptHandler0(void *param) {

}

void LPC17_Can_TxInterruptHandler1(void *param) {

}

void LPC17_Can_RxInterruptHandler0(void *param) {

}

void LPC17_Can_RxInterruptHandler1(void *param) {

}

TinyCLR_Result LPC17_Can_Acquire(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index;

    if (!LPC17_Gpio_OpenPin(g_LPC17_Can_Tx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    if (!LPC17_Gpio_OpenPin(g_LPC17_Can_Rx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    // set pin as analog
    LPC17_Gpio_ConfigurePin(g_LPC17_Can_Tx_Pins[channel].number, LPC17_Gpio_Direction::Input, g_LPC17_Can_Tx_Pins[channel].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(g_LPC17_Can_Rx_Pins[channel].number, LPC17_Gpio_Direction::Input, g_LPC17_Can_Rx_Pins[channel].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    canController[channel].can_rx_count = 0;
    canController[channel].can_rx_in = 0;
    canController[channel].can_rx_out = 0;
    canController[channel].baudrate = 0;
    canController[channel].can_max_messages_receiving = CAN_MESSAGES_MAX;
    canController[channel].provider = self;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_Release(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)globalApiProvider->FindDefault(globalApiProvider, TinyCLR_Api_Type::MemoryProvider);

    TinyCLR_Result releasePin = LPC17_Gpio_ReleasePin(nullptr, g_LPC17_Can_Tx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    releasePin = LPC17_Gpio_ReleasePin(nullptr, g_LPC17_Can_Rx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    // free pin
    LPC17_Gpio_ClosePin(g_LPC17_Can_Tx_Pins[channel].number);
    LPC17_Gpio_ClosePin(g_LPC17_Can_Rx_Pins[channel].number);

    if (canController[channel].canRxMessagesFifo != nullptr)
        memoryProvider->Free(memoryProvider, canController[channel].canRxMessagesFifo);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_Reset(const TinyCLR_Can_Provider* self) {
    int32_t channel = self->Index;

    canController[channel].can_rx_count = 0;
    canController[channel].can_rx_in = 0;
    canController[channel].can_rx_out = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_PostMessage(const TinyCLR_Can_Provider* self, uint32_t arbID, uint32_t flags, uint8_t *data) {

    uint32_t *canData = (uint32_t*)data;

    int32_t channel = self->Index;


    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_GetMessage(const TinyCLR_Can_Provider* self, uint32_t * arbID, uint32_t *flags, uint64_t *ts, uint8_t *data) {
    LPC17_Can_Message *can_msg;

    uint32_t *canData = (uint32_t*)data;

    int32_t channel = self->Index;

    if (canController[channel].can_rx_count) {
        DISABLE_INTERRUPTS_SCOPED(irq);

        can_msg = &canController[channel].canRxMessagesFifo[canController[channel].can_rx_out];
        canController[channel].can_rx_out++;

        if (canController[channel].can_rx_out == canController[channel].can_max_messages_receiving)
            canController[channel].can_rx_out = 0;

        canController[channel].can_rx_count--;
        //can1_send_error_event=TRUE;

        *flags = can_msg->Frame;
        *arbID = can_msg->MsgID; // CAN ID
        canData[0] = can_msg->DataA;
        canData[1] = can_msg->DataB;
        *ts = ((uint64_t)can_msg->TimeStampL) | ((uint64_t)can_msg->TimeStampH << 32);
    }

    return TinyCLR_Result::Success;

}

TinyCLR_Result LPC17_Can_SetSpeed(const TinyCLR_Can_Provider* self, int32_t propagation, int32_t phase1, int32_t phase2, int32_t brp, int32_t synchronizationJumpWidth, int8_t useMultiBitSampling) {
    int32_t channel = self->Index;


    auto memoryProvider = (const TinyCLR_Memory_Provider*)globalApiProvider->FindDefault(globalApiProvider, TinyCLR_Api_Type::MemoryProvider);

    canController[channel].canRxMessagesFifo = (LPC17_Can_Message*)memoryProvider->Allocate(memoryProvider, canController[channel].can_max_messages_receiving * sizeof(LPC17_Can_Message));

    if (canController[channel].canRxMessagesFifo == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    if (channel == 0) {

    }
    else {

    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_GetMessageCount(const TinyCLR_Can_Provider* self, int32_t &messageCount) {
    int32_t channel = self->Index;

    messageCount = canController[channel].can_rx_count;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_SetMessageReceivedHandler(const TinyCLR_Can_Provider* self, TinyCLR_Can_MessageReceivedHandler handler) {
    int32_t channel = self->Index;

    canController[channel].messageReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_SetErrorReceivedHandler(const TinyCLR_Can_Provider* self, TinyCLR_Can_ErrorReceivedHandler handler) {
    int32_t channel = self->Index;

    canController[channel].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_SetExplicitFilters(const TinyCLR_Can_Provider* self, uint8_t *filters, int32_t length) {
    uint32_t *_matchFilters;
    uint32_t *filters32 = (uint32_t*)filters;

    int32_t channel = self->Index;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)globalApiProvider->FindDefault(globalApiProvider, TinyCLR_Api_Type::MemoryProvider);

    _matchFilters = (uint32_t*)memoryProvider->Allocate(memoryProvider, length * sizeof(uint32_t));

    if (!_matchFilters)
        return TinyCLR_Result::OutOfMemory;

    memcpy(_matchFilters, filters32, length * sizeof(uint32_t));

    InsertionSort(_matchFilters, length);

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        canData[channel].matchFiltersSize = length;
        canData[channel].matchFilters = _matchFilters;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_SetGroupFilters(const TinyCLR_Can_Provider* self, uint8_t *lowerBounds, uint8_t *upperBounds, int32_t length) {
    uint32_t *_lowerBoundFilters, *_upperBoundFilters;
    uint32_t *lowerBounds32 = (uint32_t *)lowerBounds;
    uint32_t *upperBounds32 = (uint32_t *)upperBounds;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)globalApiProvider->FindDefault(globalApiProvider, TinyCLR_Api_Type::MemoryProvider);

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

        canData[channel].groupFiltersSize = length;
        canData[channel].lowerBoundFilters = _lowerBoundFilters;
        canData[channel].upperBoundFilters = _upperBoundFilters;
    }

    return TinyCLR_Result::Success;;
}

TinyCLR_Result LPC17_Can_DiscardIncomingMessages(const TinyCLR_Can_Provider* self) {
    int32_t channel = self->Index;

    canController[channel].can_rx_count = 0;
    canController[channel].can_rx_in = 0;
    canController[channel].can_rx_out = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Can_TransmissionAllowed(const TinyCLR_Can_Provider* self, bool &allow) {
    int32_t channel = self->Index;

    allow = false;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result LPC17_Can_ReceiveErrorCount(const TinyCLR_Can_Provider* self, int32_t &errorCount) {
    int32_t channel = self->Index;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result LPC17_Can_TransmitErrorCount(const TinyCLR_Can_Provider* self, int32_t &errorCount) {
    int32_t channel = self->Index;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result LPC17_Can_GetSourceClock(const TinyCLR_Can_Provider* self, uint32_t &sourceClock) {
    sourceClock = 0;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result LPC17_Can_SetReceiveBufferSize(const TinyCLR_Can_Provider* self, int32_t size) {
    int32_t channel = self->Index;

    if (size > 3) {
        canController[channel].can_max_messages_receiving = size;
        return TinyCLR_Result::Success;;
    }
    else {
        canController[channel].can_max_messages_receiving = CAN_MESSAGES_MAX;
        return TinyCLR_Result::ArgumentInvalid;;
    }
}

#endif // INCLUDE_CAN
