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
#include "AT91.h"

#define USART_EVENT_POST_DEBOUNCE_TICKS (10 * 10000) // 10ms between each events

static const uint32_t uartTxDefaultBuffersSize[] = AT91_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t uartRxDefaultBuffersSize[] = AT91_UART_DEFAULT_RX_BUFFER_SIZE;

struct UartState {
    int32_t controllerIndex;

    uint8_t *TxBuffer;
    uint8_t *RxBuffer;
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

    const TinyCLR_Uart_Controller* controller;

    bool tableInitialized = false;

    uint16_t initializeCount;

    uint32_t errorEvent;
    uint64_t lastEventTime;
    size_t lastEventRxBufferCount;
};

static UartState uartStates[TOTAL_UART_CONTROLLERS];
static TinyCLR_Uart_Controller uartControllers[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi[TOTAL_UART_CONTROLLERS];

const char* uartApiNames[] = {
#if TOTAL_UART_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.AT91.UartController\\0",
#if TOTAL_UART_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.AT91.UartController\\1",
#if TOTAL_UART_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.AT91.UartController\\2",
#if TOTAL_UART_CONTROLLERS > 3
"GHIElectronics.TinyCLR.NativeApis.AT91.UartController\\3",
#if TOTAL_UART_CONTROLLERS > 4
"GHIElectronics.TinyCLR.NativeApis.AT91.UartController\\4",
#if TOTAL_UART_CONTROLLERS > 5
"GHIElectronics.TinyCLR.NativeApis.AT91.UartController\\5",
#endif
#endif
#endif
#endif
#endif
#endif
};

void AT91_Uart_EnsureTableInitialized() {
    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        if (uartStates[i].tableInitialized)
            continue;

        uartControllers[i].ApiInfo = &uartApi[i];
        uartControllers[i].Acquire = &AT91_Uart_Acquire;
        uartControllers[i].Release = &AT91_Uart_Release;
        uartControllers[i].Enable = &AT91_Uart_Enable;
        uartControllers[i].Disable = &AT91_Uart_Disable;
        uartControllers[i].SetActiveSettings = &AT91_Uart_SetActiveSettings;
        uartControllers[i].Flush = &AT91_Uart_Flush;
        uartControllers[i].Read = &AT91_Uart_Read;
        uartControllers[i].Write = &AT91_Uart_Write;
        uartControllers[i].SetErrorReceivedHandler = &AT91_Uart_SetErrorReceivedHandler;
        uartControllers[i].SetDataReceivedHandler = &AT91_Uart_SetDataReceivedHandler;
        uartControllers[i].GetClearToSendState = &AT91_Uart_GetClearToSendState;
        uartControllers[i].SetClearToSendChangedHandler = &AT91_Uart_SetClearToSendChangedHandler;
        uartControllers[i].GetIsRequestToSendEnabled = &AT91_Uart_GetIsRequestToSendEnabled;
        uartControllers[i].SetIsRequestToSendEnabled = &AT91_Uart_SetIsRequestToSendEnabled;
        uartControllers[i].GetReadBufferSize = &AT91_Uart_GetReadBufferSize;
        uartControllers[i].SetReadBufferSize = &AT91_Uart_SetReadBufferSize;
        uartControllers[i].GetWriteBufferSize = &AT91_Uart_GetWriteBufferSize;
        uartControllers[i].SetWriteBufferSize = &AT91_Uart_SetWriteBufferSize;
        uartControllers[i].GetBytesToRead = &AT91_Uart_GetBytesToRead;
        uartControllers[i].GetBytesToWrite = &AT91_Uart_GetBytesToWrite;
        uartControllers[i].ClearReadBuffer = &AT91_Uart_ClearReadBuffer;
        uartControllers[i].ClearWriteBuffer = &AT91_Uart_ClearWriteBuffer;

        uartApi[i].Author = "GHI Electronics, LLC";
        uartApi[i].Name = uartApiNames[i];
        uartApi[i].Type = TinyCLR_Api_Type::UartController;
        uartApi[i].Version = 0;
        uartApi[i].Implementation = &uartControllers[i];
        uartApi[i].State = &uartStates[i];

        uartStates[i].controllerIndex = i;
        uartStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* AT91_Uart_GetRequiredApi() {
    AT91_Uart_EnsureTableInitialized();

    return &uartApi[0];
}

void AT91_Uart_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91_Uart_EnsureTableInitialized();

    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &uartApi[i]);
    }
}

static const AT91_Gpio_Pin uartTxPins[] = AT91_UART_TX_PINS;
static const AT91_Gpio_Pin uartRxPins[] = AT91_UART_RX_PINS;
static const AT91_Gpio_Pin uartRtsPins[] = AT91_UART_RTS_PINS;
static const AT91_Gpio_Pin uartCtsPins[] = AT91_UART_CTS_PINS;

int32_t AT91_Uart_GetTxPin(int32_t controllerIndex) {
    return uartTxPins[controllerIndex].number;
}

int32_t AT91_Uart_GetRxPin(int32_t controllerIndex) {
    return uartRxPins[controllerIndex].number;
}

int32_t AT91_Uart_GetRtsPin(int32_t controllerIndex) {
    return uartRtsPins[controllerIndex].number;
}

int32_t AT91_Uart_GetCtsPin(int32_t controllerIndex) {
    return uartCtsPins[controllerIndex].number;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetTxAlternateFunction(int32_t controllerIndex) {
    return uartTxPins[controllerIndex].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRxAlternateFunction(int32_t controllerIndex) {
    return uartRxPins[controllerIndex].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRtsAlternateFunction(int32_t controllerIndex) {
    return uartRtsPins[controllerIndex].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetCtsAlternateFunction(int32_t controllerIndex) {
    return uartCtsPins[controllerIndex].peripheralSelection;
}

size_t AT91_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferSize;
}

TinyCLR_Result AT91_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (state->RxBuffer) {
        memoryProvider->Free(memoryProvider, state->RxBuffer);
    }

    state->rxBufferSize = 0;

    state->RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (state->RxBuffer == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    state->rxBufferSize = size;

    return TinyCLR_Result::Success;
}

size_t AT91_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferSize;
}

TinyCLR_Result AT91_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (state->TxBuffer) {
        memoryProvider->Free(memoryProvider, state->TxBuffer);
    }

    state->txBufferSize = 0;

    state->TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (state->TxBuffer == nullptr) {
        return TinyCLR_Result::OutOfMemory;
    }

    state->txBufferSize = size;

    return TinyCLR_Result::Success;
}

bool AT91_Uart_CanPostEvent(int8_t controllerIndex) {
    auto state = reinterpret_cast<UartState*>(&uartStates[controllerIndex]);
    bool canPost = (AT91_Time_GetCurrentProcessorTime() - state->lastEventTime) > USART_EVENT_POST_DEBOUNCE_TICKS;

    state->lastEventTime = AT91_Time_GetCurrentProcessorTime();

    return canPost;
}

TinyCLR_Result AT91_Uart_PinConfiguration(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = &uartStates[controllerIndex];

    uint32_t txPin = AT91_Uart_GetTxPin(controllerIndex);
    uint32_t rxPin = AT91_Uart_GetRxPin(controllerIndex);
    uint32_t ctsPin = AT91_Uart_GetCtsPin(controllerIndex);
    uint32_t rtsPin = AT91_Uart_GetRtsPin(controllerIndex);

    AT91_Gpio_PeripheralSelection txPinMode = AT91_Uart_GetTxAlternateFunction(controllerIndex);
    AT91_Gpio_PeripheralSelection rxPinMode = AT91_Uart_GetRxAlternateFunction(controllerIndex);
    AT91_Gpio_PeripheralSelection ctsPinMode = AT91_Uart_GetCtsAlternateFunction(controllerIndex);
    AT91_Gpio_PeripheralSelection rtsPinMode = AT91_Uart_GetRtsAlternateFunction(controllerIndex);

    AT91_Uart_TxBufferEmptyInterruptEnable(controllerIndex, enable);

    AT91_Uart_RxBufferFullInterruptEnable(controllerIndex, enable);

    if (enable) {
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(txPin, AT91_Gpio_Direction::Input, txPinMode, AT91_Gpio_ResistorMode::Inactive);
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(rxPin, AT91_Gpio_Direction::Input, rxPinMode, AT91_Gpio_ResistorMode::Inactive);

        if (state->handshaking) {
            if (ctsPin == PIN_NONE || rtsPin == PIN_NONE)
                return TinyCLR_Result::NotSupported;

            if (!AT91_Gpio_OpenPin(ctsPin) || !AT91_Gpio_OpenPin(rtsPin))
                return TinyCLR_Result::SharingViolation;

            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, ctsPinMode, AT91_Gpio_ResistorMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, rtsPinMode, AT91_Gpio_ResistorMode::Inactive);
        }
    }
    else {
        AT91_Gpio_ClosePin(txPin);
        AT91_Gpio_ClosePin(rxPin);

        if (state->handshaking) {
            AT91_Gpio_ClosePin(ctsPin);
            AT91_Gpio_ClosePin(rtsPin);
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_SetErrorEvent(int32_t controllerIndex, TinyCLR_Uart_Error error) {

    auto state = &uartStates[controllerIndex];

    if (state->errorEventHandler != nullptr)
        state->errorEventHandler(state->controller, error, AT91_Time_GetCurrentProcessorTime());
}

void AT91_Uart_ReceiveData(int32_t controllerIndex, uint32_t sr) {
    AT91_USART &usart = AT91::USART(controllerIndex);

    uint8_t rxdata = usart.US_RHR;

    auto state = &uartStates[controllerIndex];
    auto canPostEvent = AT91_Uart_CanPostEvent(controllerIndex);
    bool error = (state->rxBufferCount == state->rxBufferSize) || (sr & AT91_USART::US_OVRE) || (sr & AT91_USART::US_FRAME) || (sr & AT91_USART::US_PARE);

    if (sr & AT91_USART::US_OVRE)
        if (canPostEvent) AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Overrun);

    if (sr & AT91_USART::US_FRAME)
        if (canPostEvent) AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Frame);

    if (sr & AT91_USART::US_PARE)
        if (canPostEvent) AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::ReceiveParity);

    if (state->rxBufferCount == state->rxBufferSize) {
        if (canPostEvent) AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferFull);
    }

    if (error) {
        // if error detected, clear status or reset CR.
        usart.US_CR = (AT91_USART::US_RSTRX | AT91_USART::US_RSTTX | AT91_USART::US_RXDIS | AT91_USART::US_TXDIS | AT91_USART::US_RSTSTA);

        usart.US_CR = AT91_USART::US_RXEN;
        usart.US_CR = AT91_USART::US_TXEN;
        return;
    }

    if (sr & AT91_USART::US_RXRDY) {
        state->RxBuffer[state->rxBufferIn++] = rxdata;

        state->rxBufferCount++;

        if (state->rxBufferIn == state->rxBufferSize)
            state->rxBufferIn = 0;

        if (state->dataReceivedEventHandler != nullptr) {
            if (canPostEvent) {
                if (state->rxBufferCount > state->lastEventRxBufferCount) {
                    // if driver hold event long enough that more than 1 byte
                    state->dataReceivedEventHandler(state->controller, state->rxBufferCount - state->lastEventRxBufferCount, AT91_Time_GetCurrentProcessorTime());
                }
                else {
                    // if user use poll to read data and rxBufferCount <= lastEventRxBufferCount, driver send at least 1 byte comming
                    state->dataReceivedEventHandler(state->controller, 1, AT91_Time_GetCurrentProcessorTime());
                }

                state->lastEventRxBufferCount = state->rxBufferCount;
            }
        }
    }

    // Control rts by software - enable / disable when internal buffer reach 3/4
    if (state->handshaking && (state->rxBufferCount >= ((state->rxBufferSize * 3) / 4))) {
        usart.US_CR |= AT91_USART::US_RTSDIS;// Write rts to 1
    }
}

void AT91_Uart_TransmitData(int32_t controllerIndex) {
    AT91_USART &usart = AT91::USART(controllerIndex);

    auto state = &uartStates[controllerIndex];

    if (state->txBufferCount > 0) {
        uint8_t txdata = state->TxBuffer[state->txBufferOut++];

        state->txBufferCount--;

        if (state->txBufferOut == state->txBufferSize)
            state->txBufferOut = 0;

        usart.US_THR = txdata; // write TX data

    }
    else {
        AT91_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false); // Disable interrupt when no more data to send.
    }

}
void AT91_Uart_InterruptHandler(void *param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t controllerIndex = *reinterpret_cast<uint32_t*>(param);

    AT91_USART &usart = AT91::USART(controllerIndex);

    uint32_t sr = usart.US_CSR;

    if (sr & AT91_USART::US_RXRDY || sr & AT91_USART::US_OVRE || sr & AT91_USART::US_FRAME || sr & AT91_USART::US_PARE) {
        AT91_Uart_ReceiveData(controllerIndex, sr);
    }

    auto state = &uartStates[controllerIndex];

    if (state->handshaking) {
        bool ctsState = ((sr & AT91_USART::US_CTS) > 0) ? false : true;

        if (sr & AT91_USART::US_CTSIC) {
            auto canPostEvent = AT91_Uart_CanPostEvent(controllerIndex);

            if (canPostEvent && state->cleartosendEventHandler != nullptr)
                state->cleartosendEventHandler(state->controller, ctsState, AT91_Time_GetCurrentProcessorTime());
        }

        if (!ctsState) {
            return;
        }
    }

    if (sr & AT91_USART::US_TXRDY) {
        AT91_Uart_TransmitData(controllerIndex);
    }

}
int32_t AT91_Uart_GetPeripheralId(int32_t controllerIndex) {
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

TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        int32_t txPin = AT91_Uart_GetTxPin(controllerIndex);
        int32_t rxPin = AT91_Uart_GetRxPin(controllerIndex);

        if (!AT91_Gpio_OpenPin(txPin) || !AT91_Gpio_OpenPin(rxPin))
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
        state->lastEventTime = AT91_Time_GetCurrentProcessorTime();

        state->TxBuffer = nullptr;
        state->RxBuffer = nullptr;

        if (AT91_Uart_SetWriteBufferSize(self, uartTxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        if (AT91_Uart_SetReadBufferSize(self, uartRxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        AT91_PMC &pmc = AT91::PMC();

        int32_t uartId = AT91_Uart_GetPeripheralId(controllerIndex);

        pmc.EnablePeriphClock(uartId);
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, const TinyCLR_Uart_Settings* settings) {
    uint32_t baudRate = settings->BaudRate;
    uint32_t dataBits = settings->DataBits;
    TinyCLR_Uart_Parity parity = settings->Parity;
    TinyCLR_Uart_StopBitCount stopBits = settings->StopBits;
    TinyCLR_Uart_Handshake handshaking = settings->Handshaking;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    int32_t uartId = AT91_Uart_GetPeripheralId(controllerIndex);

    AT91_USART &usart = AT91::USART(controllerIndex);

    // Disable interrupts
    usart.US_IDR = 0xFFFFFFFF;

    // Reset receiver and transmitter
    usart.US_CR = (AT91_USART::US_RSTRX | AT91_USART::US_RSTTX | AT91_USART::US_RXDIS | AT91_USART::US_TXDIS);

    // Define the baud rate divisor register
    {
        uint64_t dwMasterClock = AT91_SYSTEM_PERIPHERAL_CLOCK_HZ * 10;
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
    AT91_InterruptInternal_Activate(uartId, (uint32_t*)&AT91_Uart_InterruptHandler, (void*)&state->controllerIndex);

    // Enable Transmitter
    uint32_t USMR = (AT91_USART::US_USMODE_NORMAL);

    switch (parity) {
    case TinyCLR_Uart_Parity::Odd:
        USMR |= AT91_USART::US_PAR_ODD;
        break;
    case TinyCLR_Uart_Parity::Even:
        USMR |= AT91_USART::US_PAR_EVEN;
        break;
    case TinyCLR_Uart_Parity::Mark:
        USMR |= AT91_USART::US_PAR_MARK;
        break;
    case TinyCLR_Uart_Parity::Space:
        USMR |= AT91_USART::US_PAR_SPACE;
        break;
    case TinyCLR_Uart_Parity::None:
        USMR |= AT91_USART::US_PAR_NONE;
        break;
    default:

        return TinyCLR_Result::NotSupported;
    }

    switch (dataBits) {
    case 5:
        USMR |= AT91_USART::US_CHRL_5_BITS;
        break;
    case 6:
        USMR |= AT91_USART::US_CHRL_6_BITS;
        break;
    case 7:
        USMR |= AT91_USART::US_CHRL_7_BITS;
        break;
    case 8:
        USMR |= AT91_USART::US_CHRL_8_BITS;
        break;
    default: // not supported
        return TinyCLR_Result::NotSupported;
    }

    switch (stopBits) {
    case TinyCLR_Uart_StopBitCount::One:
        // this board doesn't appear to work with 1 stop bits set
        USMR |= AT91_USART::US_NBSTOP_1_BIT;
        break;
    case TinyCLR_Uart_StopBitCount::Two:
        USMR |= AT91_USART::US_NBSTOP_2_BIT;
        break;
    case TinyCLR_Uart_StopBitCount::OnePointFive:
        USMR |= AT91_USART::US_NBSTOP_15_BIT;
        break;
    default: // not supported
        return TinyCLR_Result::NotSupported;
    }

    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        usart.US_IER = AT91_USART::US_CTSIC; // Enable cts interrupt
        usart.US_CR = AT91_USART::US_RTSEN; // Write rts to 0
        state->handshaking = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    usart.US_MR = USMR;

    usart.US_CR = AT91_USART::US_RXEN;
    usart.US_CR = AT91_USART::US_TXEN;

    return AT91_Uart_PinConfiguration(controllerIndex, true);
}

TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Controller* self) {
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

        AT91_PMC &pmc = AT91::PMC();

        int32_t uartId = AT91_Uart_GetPeripheralId(controllerIndex);

        AT91_InterruptInternal_Deactivate(uartId);

        AT91_Uart_PinConfiguration(controllerIndex, false);

        pmc.DisablePeriphClock(uartId);

        if (apiManager != nullptr) {
            auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

            memoryProvider->Free(memoryProvider, state->TxBuffer);
            memoryProvider->Free(memoryProvider, state->RxBuffer);
        }

        state->handshaking = false;
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_USART &usart = AT91::USART(controllerIndex);

    if (enable) {
        usart.US_IER = AT91_USART::US_TXRDY;
    }
    else {
        usart.US_IDR = AT91_USART::US_TXRDY;
    }
}

void AT91_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_USART &usart = AT91::USART(controllerIndex);

    if (enable) {
        usart.US_IER = AT91_USART::US_RXRDY;
    }
    else {
        usart.US_IDR = AT91_USART::US_RXRDY;
    }
}

bool AT91_Uart_CanSend(int controllerIndex) {
    auto state = &uartStates[controllerIndex];
    bool value;

    AT91_Uart_GetClearToSendState(state->controller, value);

    return value;
}

TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount && !AT91_Interrupt_IsDisabled()) {
        AT91_Uart_TxBufferEmptyInterruptEnable(state->controllerIndex, true);

        while (state->txBufferCount > 0) {
            AT91_Time_Delay(nullptr, 1);
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    if (state->initializeCount == 0 || state->rxBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(self->GetBytesToRead(self), length);

    size_t i = 0;

    while (i < length) {
        buffer[i] = state->RxBuffer[state->rxBufferOut];

        state->rxBufferOut++;
        i++;
        state->rxBufferCount--;

        if (state->rxBufferOut == state->rxBufferSize)
            state->rxBufferOut = 0;

        // Control rts by software - enable / disable when internal buffer reach 3/4
        if (state->handshaking && (state->rxBufferCount < ((state->rxBufferSize * 3) / 4))) {
            AT91_USART &usart = AT91::USART(controllerIndex);
            usart.US_CR |= AT91_USART::US_RTSEN;// Write rts to 0
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->initializeCount == 0 || state->txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (state->txBufferCount == state->txBufferSize) {
        AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferFull);

        return TinyCLR_Result::Busy;
    }

    length = std::min(state->txBufferSize - state->txBufferCount, length);


    while (i < length) {

        state->TxBuffer[state->txBufferIn] = buffer[i];

        state->txBufferCount++;

        i++;

        state->txBufferIn++;

        if (state->txBufferIn == state->txBufferSize)
            state->txBufferIn = 0;
    }

    if (length > 0) {
        AT91_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = true;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;

        // Reading the pin state to protect values from register for inteterupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        AT91_Gpio_Read(nullptr, uartCtsPins[controllerIndex].number, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? false : true;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->cleartosendEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = false;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;

        // Reading the pin state to protect values from register for interrupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        AT91_Gpio_Read(nullptr, uartRtsPins[controllerIndex].number, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? true : false;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool value) {
    // Enable by hardware, no support by software.
    return TinyCLR_Result::NotSupported;
}

size_t AT91_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferCount;
}

size_t AT91_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferCount;
}

TinyCLR_Result AT91_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->rxBufferCount = state->rxBufferIn = state->rxBufferOut = state->lastEventRxBufferCount = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->txBufferCount = state->txBufferIn = state->txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void AT91_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartStates[i].initializeCount = 0;
        AT91_Uart_Release(&uartControllers[i]);

        uartStates[i].tableInitialized = false;
    }
}

TinyCLR_Result AT91_Uart_Enable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Disable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = false;

    return TinyCLR_Result::Success;
}
