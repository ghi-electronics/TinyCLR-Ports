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

#include <algorithm>
#include "AT91SAM9Rx64.h"

#define USART_EVENT_POST_DEBOUNCE_TICKS (10 * 10000) // 10ms between each events
void AT91SAM9Rx64_Uart_EventCallback(const TinyCLR_Task_Manager* self, const TinyCLR_Api_Manager* apiManager, TinyCLR_Task_Reference task, void* arg);
static const uint32_t uartTxDefaultBuffersSize[] = AT91SAM9Rx64_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t uartRxDefaultBuffersSize[] = AT91SAM9Rx64_UART_DEFAULT_RX_BUFFER_SIZE;

struct UartState {
    int32_t controllerIndex;

    uint8_t *txBuffer;
    uint8_t *rxBuffer;
    size_t txBufferCount;
    size_t txBufferIn;
    size_t txBufferOut;
    size_t txBufferSize;

    size_t rxBufferCount;
    size_t rxBufferIn;
    size_t rxBufferOut;
    size_t rxBufferSize;

    bool handshaking;
    bool enable;

    TinyCLR_Uart_ErrorReceivedHandler errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler dataReceivedEventHandler;
    TinyCLR_Uart_ClearToSendChangedHandler cleartosendEventHandler;

    TinyCLR_Task_Reference dataReceivedCallbackTaskReference;
    TinyCLR_Task_Reference errorCallbackTaskReference;

    const TinyCLR_Task_Manager* taskManager;

    const TinyCLR_Uart_Controller* controller;

    bool tableInitialized;

    uint16_t initializeCount;

    size_t lastEventRxBufferCount;
    uint64_t lastRxTime;

    uint8_t errorEvent;
};

static UartState uartStates[TOTAL_UART_CONTROLLERS];
static TinyCLR_Uart_Controller uartControllers[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi[TOTAL_UART_CONTROLLERS];

const char* uartApiNames[] = {
#if TOTAL_UART_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.UartController\\0",
#if TOTAL_UART_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.UartController\\1",
#if TOTAL_UART_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.UartController\\2",
#if TOTAL_UART_CONTROLLERS > 3
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.UartController\\3",
#if TOTAL_UART_CONTROLLERS > 4
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.UartController\\4",
#if TOTAL_UART_CONTROLLERS > 5
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.UartController\\5",
#endif
#endif
#endif
#endif
#endif
#endif
};

void AT91SAM9Rx64_Uart_EnsureTableInitialized() {
    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        if (uartStates[i].tableInitialized)
            continue;

        uartControllers[i].ApiInfo = &uartApi[i];
        uartControllers[i].Acquire = &AT91SAM9Rx64_Uart_Acquire;
        uartControllers[i].Release = &AT91SAM9Rx64_Uart_Release;
        uartControllers[i].Enable = &AT91SAM9Rx64_Uart_Enable;
        uartControllers[i].Disable = &AT91SAM9Rx64_Uart_Disable;
        uartControllers[i].SetActiveSettings = &AT91SAM9Rx64_Uart_SetActiveSettings;
        uartControllers[i].Flush = &AT91SAM9Rx64_Uart_Flush;
        uartControllers[i].Read = &AT91SAM9Rx64_Uart_Read;
        uartControllers[i].Write = &AT91SAM9Rx64_Uart_Write;
        uartControllers[i].SetErrorReceivedHandler = &AT91SAM9Rx64_Uart_SetErrorReceivedHandler;
        uartControllers[i].SetDataReceivedHandler = &AT91SAM9Rx64_Uart_SetDataReceivedHandler;
        uartControllers[i].GetClearToSendState = &AT91SAM9Rx64_Uart_GetClearToSendState;
        uartControllers[i].SetClearToSendChangedHandler = &AT91SAM9Rx64_Uart_SetClearToSendChangedHandler;
        uartControllers[i].GetIsRequestToSendEnabled = &AT91SAM9Rx64_Uart_GetIsRequestToSendEnabled;
        uartControllers[i].SetIsRequestToSendEnabled = &AT91SAM9Rx64_Uart_SetIsRequestToSendEnabled;
        uartControllers[i].GetReadBufferSize = &AT91SAM9Rx64_Uart_GetReadBufferSize;
        uartControllers[i].SetReadBufferSize = &AT91SAM9Rx64_Uart_SetReadBufferSize;
        uartControllers[i].GetWriteBufferSize = &AT91SAM9Rx64_Uart_GetWriteBufferSize;
        uartControllers[i].SetWriteBufferSize = &AT91SAM9Rx64_Uart_SetWriteBufferSize;
        uartControllers[i].GetBytesToRead = &AT91SAM9Rx64_Uart_GetBytesToRead;
        uartControllers[i].GetBytesToWrite = &AT91SAM9Rx64_Uart_GetBytesToWrite;
        uartControllers[i].ClearReadBuffer = &AT91SAM9Rx64_Uart_ClearReadBuffer;
        uartControllers[i].ClearWriteBuffer = &AT91SAM9Rx64_Uart_ClearWriteBuffer;

        uartApi[i].Author = "GHI Electronics, LLC";
        uartApi[i].Name = uartApiNames[i];
        uartApi[i].Type = TinyCLR_Api_Type::UartController;
        uartApi[i].Version = 0;
        uartApi[i].Implementation = &uartControllers[i];
        uartApi[i].State = &uartStates[i];

        uartStates[i].controllerIndex = i;
        uartStates[i].initializeCount = 0;
        uartStates[i].txBuffer = nullptr;
        uartStates[i].txBuffer = nullptr;

        uartStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* AT91SAM9Rx64_Uart_GetRequiredApi() {
    AT91SAM9Rx64_Uart_EnsureTableInitialized();

    return &uartApi[UART_DEBUGGER_INDEX];
}

void AT91SAM9Rx64_Uart_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91SAM9Rx64_Uart_EnsureTableInitialized();

    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &uartApi[i]);
    }
}

#define UART_TXD_PIN 0
#define UART_RXD_PIN 1
#define UART_RTS_PIN 2
#define UART_CTS_PIN 3

static const AT91SAM9Rx64_Gpio_Pin uartPins[][4] = AT91SAM9Rx64_UART_PINS;

size_t AT91SAM9Rx64_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferSize;
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (state->rxBuffer) {
        memoryProvider->Free(memoryProvider, state->rxBuffer);
    }

    state->rxBufferSize = 0;

    state->rxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (state->rxBuffer == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    state->rxBufferSize = size;

    return TinyCLR_Result::Success;
}

size_t AT91SAM9Rx64_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferSize;
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (state->txBuffer) {
        memoryProvider->Free(memoryProvider, state->txBuffer);
    }

    state->txBufferSize = 0;

    state->txBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (state->txBuffer == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    state->txBufferSize = size;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_PinConfiguration(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = &uartStates[controllerIndex];

    AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(controllerIndex, enable);

    AT91SAM9Rx64_Uart_RxBufferFullInterruptEnable(controllerIndex, enable);

    if (enable) {
        // Connect pin to UART
        AT91SAM9Rx64_GpioInternal_ConfigurePin(uartPins[controllerIndex][UART_TXD_PIN].number, AT91SAM9Rx64_Gpio_Direction::Input, uartPins[controllerIndex][UART_TXD_PIN].peripheralSelection, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
        // Connect pin to UART
        AT91SAM9Rx64_GpioInternal_ConfigurePin(uartPins[controllerIndex][UART_RXD_PIN].number, AT91SAM9Rx64_Gpio_Direction::Input, uartPins[controllerIndex][UART_RXD_PIN].peripheralSelection, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);

        if (state->handshaking) {
            if (uartPins[controllerIndex][UART_CTS_PIN].number == PIN_NONE || uartPins[controllerIndex][UART_RTS_PIN].number == PIN_NONE)
                return TinyCLR_Result::NotSupported;

            if (!AT91SAM9Rx64_GpioInternal_OpenMultiPins(&uartPins[controllerIndex][UART_RTS_PIN], 2))
                return TinyCLR_Result::SharingViolation;

            AT91SAM9Rx64_GpioInternal_ConfigurePin(uartPins[controllerIndex][UART_CTS_PIN].number, AT91SAM9Rx64_Gpio_Direction::Input, uartPins[controllerIndex][UART_CTS_PIN].peripheralSelection, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
            AT91SAM9Rx64_GpioInternal_ConfigurePin(uartPins[controllerIndex][UART_RTS_PIN].number, AT91SAM9Rx64_Gpio_Direction::Input, uartPins[controllerIndex][UART_RTS_PIN].peripheralSelection, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
        }
    }
    else {
        AT91SAM9Rx64_GpioInternal_ClosePin(uartPins[controllerIndex][UART_TXD_PIN].number);
        AT91SAM9Rx64_GpioInternal_ClosePin(uartPins[controllerIndex][UART_RXD_PIN].number);

        if (state->handshaking) {
            AT91SAM9Rx64_GpioInternal_ClosePin(uartPins[controllerIndex][UART_CTS_PIN].number);
            AT91SAM9Rx64_GpioInternal_ClosePin(uartPins[controllerIndex][UART_RTS_PIN].number);
        }
    }

    return TinyCLR_Result::Success;
}

static inline void AT91SAM9Rx64_Uart_ReceiveData(int32_t controllerIndex, uint32_t sr) {
    AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);

    auto state = &uartStates[controllerIndex];
    bool error = ((sr & AT91SAM9Rx64_USART::US_OVRE) || (sr & AT91SAM9Rx64_USART::US_FRAME) || (sr & AT91SAM9Rx64_USART::US_PARE)) != 0;

    uint8_t data = usart.US_RHR;

    if (sr & AT91SAM9Rx64_USART::US_RXRDY) {
        state->rxBuffer[state->rxBufferIn++] = data;

        if (state->rxBufferCount < state->rxBufferSize) {
            state->rxBufferCount++;
        }

        if (state->rxBufferIn == state->rxBufferSize)
            state->rxBufferIn = 0;

        if (state->dataReceivedEventHandler != nullptr) {
            auto now = AT91SAM9Rx64_Time_GetSystemTime(nullptr);

            state->lastEventRxBufferCount++;

            if (now > (state->lastRxTime + USART_EVENT_POST_DEBOUNCE_TICKS)) {
                state->dataReceivedEventHandler(state->controller, state->lastEventRxBufferCount, now);
                state->lastEventRxBufferCount = 0;
            }

            state->lastRxTime = now;
        }
    }

    if (state->rxBufferCount == state->rxBufferSize) {
        state->errorEvent = 1 << (uint8_t)TinyCLR_Uart_Error::BufferFull;
    }
    else if (sr & AT91SAM9Rx64_USART::US_OVRE) {
        state->errorEvent = 1 << (uint8_t)TinyCLR_Uart_Error::Overrun;
    }
    else if (sr & AT91SAM9Rx64_USART::US_FRAME) {
        state->errorEvent = 1 << (uint8_t)TinyCLR_Uart_Error::Frame;
    }
    else if (sr & AT91SAM9Rx64_USART::US_PARE) {
        state->errorEvent = 1 << (uint8_t)TinyCLR_Uart_Error::ReceiveParity;
    }

    if (error) {
        // if error detected, clear status or reset CR.
        usart.US_CR = (AT91SAM9Rx64_USART::US_RSTRX | AT91SAM9Rx64_USART::US_RSTTX | AT91SAM9Rx64_USART::US_RXDIS | AT91SAM9Rx64_USART::US_TXDIS | AT91SAM9Rx64_USART::US_RSTSTA);

        usart.US_CR = AT91SAM9Rx64_USART::US_RXEN;
        usart.US_CR = AT91SAM9Rx64_USART::US_TXEN;
    }

    // Control rts by software - enable / disable when internal buffer reach 3/4
    if (state->handshaking && (state->rxBufferCount >= ((state->rxBufferSize * 3) / 4))) {
        usart.US_CR |= AT91SAM9Rx64_USART::US_RTSDIS;// Write rts to 1
    }
}

void AT91SAM9Rx64_Uart_TransmitData(int32_t controllerIndex) {
    AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);

    auto state = &uartStates[controllerIndex];

    if (state->txBufferCount > 0) {
        uint8_t txdata = state->txBuffer[state->txBufferOut++];

        state->txBufferCount--;

        if (state->txBufferOut == state->txBufferSize)
            state->txBufferOut = 0;

        usart.US_THR = txdata; // write TX data

    }
    else {
        AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false); // Disable interrupt when no more data to send.
    }

}
void AT91SAM9Rx64_Uart_InterruptHandler(void *param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t controllerIndex = *reinterpret_cast<uint32_t*>(param);

    AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);

    uint32_t sr = usart.US_CSR;

    if (sr & AT91SAM9Rx64_USART::US_RXRDY || sr & AT91SAM9Rx64_USART::US_OVRE || sr & AT91SAM9Rx64_USART::US_FRAME || sr & AT91SAM9Rx64_USART::US_PARE) {
        AT91SAM9Rx64_Uart_ReceiveData(controllerIndex, sr);
    }

    auto state = &uartStates[controllerIndex];

    if (state->handshaking) {
        bool ctsState = ((sr & AT91SAM9Rx64_USART::US_CTS) > 0) ? false : true;

        if (sr & AT91SAM9Rx64_USART::US_CTSIC) {
            if (state->cleartosendEventHandler != nullptr)
                state->cleartosendEventHandler(state->controller, ctsState, AT91SAM9Rx64_Time_GetSystemTime(nullptr));

            if (ctsState) {
                // If tx was disable to avoid locked up
                // Need Enable back if detected OK to send
                AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);
            }
        }

        if (!ctsState) {
            // Temporary disable tx during cts is high to avoild device lockup
            AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false);

            return;
        }
    }

    if (sr & AT91SAM9Rx64_USART::US_TXRDY) {
        AT91SAM9Rx64_Uart_TransmitData(controllerIndex);
    }

}
int32_t AT91SAM9Rx64_Uart_GetPeripheralId(int32_t controllerIndex) {
    int32_t usartId;

    if (controllerIndex == 0) {
        usartId = (AT91C_ID_SYS);
    }
    else if ((controllerIndex > 0) && (controllerIndex < 4)) {
        usartId = (AT91C_ID_US0 + (controllerIndex - 1));
    }
    else {
        usartId = (AT91C_ID_US0 + (controllerIndex - 4));
    }

    return usartId;
}

TinyCLR_Result AT91SAM9Rx64_Uart_Acquire(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;
        if (controllerIndex >= TOTAL_UART_CONTROLLERS)
            return TinyCLR_Result::ArgumentInvalid;

        if (!AT91SAM9Rx64_GpioInternal_OpenMultiPins(uartPins[controllerIndex], 2))
            return TinyCLR_Result::SharingViolation;

        state->txBufferCount = 0;
        state->txBufferIn = 0;
        state->txBufferOut = 0;

        state->rxBufferCount = 0;
        state->rxBufferIn = 0;
        state->rxBufferOut = 0;

        state->controller = self;
        state->handshaking = false;
        state->enable = false;

        state->lastEventRxBufferCount = 0;
        state->errorEvent = 0;
        state->lastRxTime = 0;

        state->txBuffer = nullptr;
        state->rxBuffer = nullptr;
        state->errorEventHandler = nullptr;
        state->dataReceivedEventHandler = nullptr;
        state->cleartosendEventHandler = nullptr;

        if (AT91SAM9Rx64_Uart_SetWriteBufferSize(self, uartTxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        if (AT91SAM9Rx64_Uart_SetReadBufferSize(self, uartRxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        AT91SAM9Rx64_PMC &pmc = AT91::PMC();

        int32_t uartId = AT91SAM9Rx64_Uart_GetPeripheralId(controllerIndex);

        pmc.EnablePeriphClock(uartId);
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, const TinyCLR_Uart_Settings* settings) {
    uint32_t baudRate = settings->BaudRate;
    uint32_t dataBits = settings->DataBits;
    TinyCLR_Uart_Parity parity = settings->Parity;
    TinyCLR_Uart_StopBitCount stopBits = settings->StopBits;
    TinyCLR_Uart_Handshake handshaking = settings->Handshaking;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    int32_t uartId = AT91SAM9Rx64_Uart_GetPeripheralId(controllerIndex);

    AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);

    // Disable interrupts
    usart.US_IDR = 0xFFFFFFFF;

    // Reset receiver and transmitter
    usart.US_CR = (AT91SAM9Rx64_USART::US_RSTRX | AT91SAM9Rx64_USART::US_RSTTX | AT91SAM9Rx64_USART::US_RXDIS | AT91SAM9Rx64_USART::US_TXDIS);

    // Define the baud rate divisor register
    {
        uint64_t dwMasterClock = AT91SAM9Rx64_SYSTEM_PERIPHERAL_CLOCK_HZ * 10;
        uint32_t baud_value = ((dwMasterClock) / (baudRate * 16));

        while ((baud_value > 0) && (baud_value * (baudRate * 16) > dwMasterClock)) {
            baud_value--;
        }

        if ((baud_value % 10) >= 5)
            baud_value = (baud_value / 10) + 1;
        else
            baud_value /= 10;

        usart.US_BRGR = baud_value;
    }

    // Write the Timeguard Register
    usart.US_TTGR = 0;

    state->controllerIndex = controllerIndex;
    AT91SAM9Rx64_InterruptInternal_Activate(uartId, (uint32_t*)&AT91SAM9Rx64_Uart_InterruptHandler, (void*)&state->controllerIndex);

    // Enable Transmitter
    uint32_t USMR = (AT91SAM9Rx64_USART::US_USMODE_NORMAL);

    switch (parity) {
    case TinyCLR_Uart_Parity::Odd:
        USMR |= AT91SAM9Rx64_USART::US_PAR_ODD;
        break;
    case TinyCLR_Uart_Parity::Even:
        USMR |= AT91SAM9Rx64_USART::US_PAR_EVEN;
        break;
    case TinyCLR_Uart_Parity::Mark:
        USMR |= AT91SAM9Rx64_USART::US_PAR_MARK;
        break;
    case TinyCLR_Uart_Parity::Space:
        USMR |= AT91SAM9Rx64_USART::US_PAR_SPACE;
        break;
    case TinyCLR_Uart_Parity::None:
        USMR |= AT91SAM9Rx64_USART::US_PAR_NONE;
        break;
    default:

        return TinyCLR_Result::NotSupported;
    }

    switch (dataBits) {
    case 5:
        USMR |= AT91SAM9Rx64_USART::US_CHRL_5_BITS;
        break;
    case 6:
        USMR |= AT91SAM9Rx64_USART::US_CHRL_6_BITS;
        break;
    case 7:
        USMR |= AT91SAM9Rx64_USART::US_CHRL_7_BITS;
        break;
    case 8:
        USMR |= AT91SAM9Rx64_USART::US_CHRL_8_BITS;
        break;
    default: // not supported
        return TinyCLR_Result::NotSupported;
    }

    switch (stopBits) {
    case TinyCLR_Uart_StopBitCount::One:
        // this board doesn't appear to work with 1 stop bits set
        USMR |= AT91SAM9Rx64_USART::US_NBSTOP_1_BIT;
        break;
    case TinyCLR_Uart_StopBitCount::Two:
        USMR |= AT91SAM9Rx64_USART::US_NBSTOP_2_BIT;
        break;
    case TinyCLR_Uart_StopBitCount::OnePointFive:
        USMR |= AT91SAM9Rx64_USART::US_NBSTOP_15_BIT;
        break;
    default: // not supported
        return TinyCLR_Result::NotSupported;
    }

    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        usart.US_IER = AT91SAM9Rx64_USART::US_CTSIC; // Enable cts interrupt
        usart.US_CR = AT91SAM9Rx64_USART::US_RTSEN; // Write rts to 0
        state->handshaking = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    usart.US_MR = USMR;

    usart.US_CR = AT91SAM9Rx64_USART::US_RXEN;
    usart.US_CR = AT91SAM9Rx64_USART::US_TXEN;

    return AT91SAM9Rx64_Uart_PinConfiguration(controllerIndex, true);
}

TinyCLR_Result AT91SAM9Rx64_Uart_Release(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        state->txBufferCount = 0;
        state->txBufferIn = 0;
        state->txBufferOut = 0;

        state->rxBufferCount = 0;
        state->rxBufferIn = 0;
        state->rxBufferOut = 0;

        state->enable = false;

        AT91SAM9Rx64_PMC &pmc = AT91::PMC();

        int32_t uartId = AT91SAM9Rx64_Uart_GetPeripheralId(controllerIndex);

        AT91SAM9Rx64_InterruptInternal_Deactivate(uartId);

        AT91SAM9Rx64_Uart_PinConfiguration(controllerIndex, false);

        pmc.DisablePeriphClock(uartId);

        // Release memory
        if (apiManager != nullptr) {
            auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

            if (state->txBuffer != nullptr) {
                memoryProvider->Free(memoryProvider, state->txBuffer);

                state->txBuffer = nullptr;
            }

            if (state->rxBuffer != nullptr) {
                memoryProvider->Free(memoryProvider, state->rxBuffer);

                state->rxBuffer = nullptr;
            }
        }

        AT91SAM9Rx64_Uart_SetErrorReceivedHandler(self, nullptr);
        AT91SAM9Rx64_Uart_SetDataReceivedHandler(self, nullptr);

        state->handshaking = false;
    }

    return TinyCLR_Result::Success;
}

void AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);

    if (enable) {
        usart.US_IER = AT91SAM9Rx64_USART::US_TXRDY;
    }
    else {
        usart.US_IDR = AT91SAM9Rx64_USART::US_TXRDY;
    }
}

void AT91SAM9Rx64_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);

    if (enable) {
        usart.US_IER = AT91SAM9Rx64_USART::US_RXRDY;
    }
    else {
        usart.US_IDR = AT91SAM9Rx64_USART::US_RXRDY;
    }
}

bool AT91SAM9Rx64_Uart_CanSend(int controllerIndex) {
    auto state = &uartStates[controllerIndex];
    bool value = true;

    AT91SAM9Rx64_Uart_GetClearToSendState(state->controller, value);

    return value;
}

TinyCLR_Result AT91SAM9Rx64_Uart_Flush(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount && !AT91SAM9Rx64_Interrupt_IsDisabled()) {
        AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(state->controllerIndex, true);

        while (state->txBufferCount > 0) {
            AT91SAM9Rx64_Time_Delay(nullptr, 1);
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    if (state->initializeCount == 0) {
        length = 0; // make sure length is updated

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(self->GetBytesToRead(self), length);

    size_t i = 0;

    while (i < length) {
        buffer[i++] = state->rxBuffer[state->rxBufferOut++];

        if (state->rxBufferOut == state->rxBufferSize)
            state->rxBufferOut = 0;

        // Control rts by software - enable / disable when internal buffer reach 3/4
        if (state->handshaking && (state->rxBufferCount < ((state->rxBufferSize * 3) / 4))) {
            AT91SAM9Rx64_USART &usart = AT91::USART(controllerIndex);
            usart.US_CR |= AT91SAM9Rx64_USART::US_RTSEN;// Write rts to 0
        }
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        state->rxBufferCount -= length;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->initializeCount == 0) {
        length = 0; // make sure length is updated

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(state->txBufferSize - state->txBufferCount, length);

    if (length == 0) return TinyCLR_Result::Success; // Return Success with nothing written;

    while (i < length) {

        state->txBuffer[state->txBufferIn++] = buffer[i++];

        if (state->txBufferIn == state->txBufferSize)
            state->txBufferIn = 0;
    }

    {
        DISABLE_INTERRUPTS_SCOPED(irq);

        state->txBufferCount += length;
    }

    if (length > 0) {
        AT91SAM9Rx64_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Uart_Error AT91SAM9Rx64_Uart_GetError(uint32_t error) {
    switch (error) {
    case 1:
        return TinyCLR_Uart_Error::Frame;

    case 2:
        return TinyCLR_Uart_Error::Overrun;

    case 8:
        return TinyCLR_Uart_Error::ReceiveParity;

    default:
        return TinyCLR_Uart_Error::BufferFull;
    }
}

void AT91SAM9Rx64_Uart_EventCallback(const TinyCLR_Task_Manager* self, const TinyCLR_Api_Manager* apiManager, TinyCLR_Task_Reference task, void* arg) {
    auto state = reinterpret_cast<UartState*>(arg);

    if (task == state->dataReceivedCallbackTaskReference) {
        size_t latestCount = 0;

        {
            DISABLE_INTERRUPTS_SCOPED(irq);
            latestCount = state->lastEventRxBufferCount;
            state->lastEventRxBufferCount = 0;
        }

        if (latestCount > 0 && state->dataReceivedEventHandler != nullptr) {
            state->dataReceivedEventHandler(state->controller, latestCount, AT91SAM9Rx64_Time_GetSystemTime(nullptr));
        }

        state->taskManager->Enqueue(state->taskManager, task, AT91SAM9Rx64_Time_GetProcessorTicksForTime(nullptr, USART_EVENT_POST_DEBOUNCE_TICKS));
    }
    else if (task == state->errorCallbackTaskReference) {
        uint8_t latestError = 0;

        {
            DISABLE_INTERRUPTS_SCOPED(irq);
            latestError = state->errorEvent;
            state->errorEvent = 0;
        }

        if ((latestError != 0) && state->errorEventHandler != nullptr) {
            state->errorEventHandler(state->controller, AT91SAM9Rx64_Uart_GetError(latestError), AT91SAM9Rx64_Time_GetSystemTime(nullptr));
        }
        state->taskManager->Enqueue(state->taskManager, task, AT91SAM9Rx64_Time_GetProcessorTicksForTime(nullptr, USART_EVENT_POST_DEBOUNCE_TICKS));
    }
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (handler != nullptr) {
        state->errorEventHandler = handler;
        state->taskManager = (const TinyCLR_Task_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::TaskManager);
        state->taskManager->Create(state->taskManager, AT91SAM9Rx64_Uart_EventCallback, (void*)state, false, state->errorCallbackTaskReference);
        state->taskManager->Enqueue(state->taskManager, state->errorCallbackTaskReference, AT91SAM9Rx64_Time_GetProcessorTicksForTime(nullptr, USART_EVENT_POST_DEBOUNCE_TICKS));
    }
    else {
        if (state->errorEventHandler != nullptr && state->taskManager != nullptr && state->errorCallbackTaskReference) {
            state->taskManager->Free(state->taskManager, state->errorCallbackTaskReference);

            state->errorEventHandler = nullptr;
            state->errorCallbackTaskReference = nullptr;
        }

    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (handler != nullptr) {
        state->dataReceivedEventHandler = handler;
        state->taskManager = (const TinyCLR_Task_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::TaskManager);
        state->taskManager->Create(state->taskManager, AT91SAM9Rx64_Uart_EventCallback, (void*)state, false, state->dataReceivedCallbackTaskReference);
        state->taskManager->Enqueue(state->taskManager, state->dataReceivedCallbackTaskReference, AT91SAM9Rx64_Time_GetProcessorTicksForTime(nullptr, USART_EVENT_POST_DEBOUNCE_TICKS));
    }

    else {
        if (state->dataReceivedEventHandler != nullptr && state->taskManager != nullptr && state->dataReceivedCallbackTaskReference != nullptr) {
            state->taskManager->Free(state->taskManager, state->dataReceivedCallbackTaskReference);

            state->dataReceivedEventHandler = nullptr;
            state->dataReceivedCallbackTaskReference = nullptr;
        }

    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = true;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;

        // Reading the pin state to protect values from register for inteterupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        AT91SAM9Rx64_Gpio_Read(nullptr, uartPins[controllerIndex][UART_CTS_PIN].number, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? false : true;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->cleartosendEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = false;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;

        // Reading the pin state to protect values from register for interrupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        AT91SAM9Rx64_Gpio_Read(nullptr, uartPins[controllerIndex][UART_RTS_PIN].number, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? true : false;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool value) {
    // Enable by hardware, no support by software.
    return TinyCLR_Result::NotSupported;
}

size_t AT91SAM9Rx64_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferCount;
}

size_t AT91SAM9Rx64_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferCount;
}

TinyCLR_Result AT91SAM9Rx64_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    uartTxDefaultBuffersSize ate->rxBufferIn = state->rxBufferOut = state->lastEventRxBufferCount = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->txBufferCount = state->txBufferIn = state->txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void AT91SAM9Rx64_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        AT91SAM9Rx64_Uart_Release(&uartControllers[i]);

        uartStates[i].tableInitialized = false;
        uartStates[i].initializeCount = 0;
        uartStates[i].txBuffer = nullptr;
        uartStates[i].txBuffer = nullptr;
    }
}

TinyCLR_Result AT91SAM9Rx64_Uart_Enable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Uart_Disable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = false;

    return TinyCLR_Result::Success;
}
