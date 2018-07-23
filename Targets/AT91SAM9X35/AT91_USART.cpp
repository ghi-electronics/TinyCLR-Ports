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

    bool isOpened;
    bool handshakeEnable;

    TinyCLR_Uart_ErrorReceivedHandler errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler dataReceivedEventHandler;

    const TinyCLR_Uart_Controller* controller;
};


static UartState uartStates[TOTAL_UART_CONTROLLERS];
static TinyCLR_Uart_Controller uartControllers[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi[TOTAL_UART_CONTROLLERS];

const TinyCLR_Api_Info* AT91_Uart_GetApi() {
    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartControllers[i].ApiInfo = &uartApi[i];
        uartControllers[i].Acquire = &SAT91_Uart_Acquire;
        uartControllers[i].Release = &SAT91_Uart_Release;
        uartControllers[i].SetActiveSettings = &SAT91_Uart_SetActiveSettings;
        uartControllers[i].Flush = &SAT91_Uart_Flush;
        uartControllers[i].Read = &SAT91_Uart_Read;
        uartControllers[i].Write = &SAT91_Uart_Write;
        uartControllers[i].SetErrorReceivedHandler = &SAT91_Uart_SetErrorReceivedHandler;
        uartControllers[i].SetDataReceivedHandler = &SAT91_Uart_SetDataReceivedHandler;
        uartControllers[i].GetClearToSendState = &SAT91_Uart_GetClearToSendState;
        uartControllers[i].SetClearToSendChangedHandler = &SAT91_Uart_SetClearToSendChangedHandler;
        uartControllers[i].GetIsRequestToSendEnabled = &SAT91_Uart_GetIsRequestToSendEnabled;
        uartControllers[i].SetIsRequestToSendEnabled = &SAT91_Uart_SetIsRequestToSendEnabled;
        uartControllers[i].GetReadBufferSize = &SAT91_Uart_GetReadBufferSize;
        uartControllers[i].SetReadBufferSize = &SAT91_Uart_SetReadBufferSize;
        uartControllers[i].GetWriteBufferSize = &SAT91_Uart_GetWriteBufferSize;
        uartControllers[i].SetWriteBufferSize = &SAT91_Uart_SetWriteBufferSize;
        uartControllers[i].GetUnreadCount = &SAT91_Uart_GetUnreadCount;
        uartControllers[i].GetUnwrittenCount = &SAT91_Uart_GetUnwrittenCount;
        uartControllers[i].ClearReadBuffer = &SAT91_Uart_ClearReadBuffer;
        uartControllers[i].ClearWriteBuffer = &SAT91_Uart_ClearWriteBuffer;

        uartApi[i].Author = "GHI Electronics, LLC";
        uartApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.SAT91.UartController";
        uartApi[i].Type = TinyCLR_Api_Type::UartController;
        uartApi[i].Version = 0;
        uartApi[i].Implementation = &uartControllers[i];
        uartApi[i].State = &uartStates[i];

        uartStates[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&uartApi;
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

    auto controllerIndex = state->controllerIndex;

    return state->rxBufferSize == 0 ? uartRxDefaultBuffersSize[controllerIndex] : state->rxBufferSize;
}

TinyCLR_Result AT91_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->rxBufferSize) {
        memoryProvider->Free(memoryProvider, state->RxBuffer);
    }

    state->rxBufferSize = size;

    state->RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (state->RxBuffer == nullptr) {
        state->rxBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}

size_t AT91_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    return state->txBufferSize == 0 ? uartTxDefaultBuffersSize[controllerIndex] : state->txBufferSize;
}

TinyCLR_Result AT91_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (state->txBufferSize) {
        memoryProvider->Free(memoryProvider, state->TxBuffer);
    }

    state->txBufferSize = size;

    state->TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (state->TxBuffer == nullptr) {
        state->txBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
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

        if (state->handshakeEnable) {
            if (ctsPin == PIN_NONE || rtsPin == PIN_NONE)
                return TinyCLR_Result::NotSupported;

            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, ctsPinMode, AT91_Gpio_ResistorMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, rtsPinMode, AT91_Gpio_ResistorMode::Inactive);
        }
    }
    else {
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(txPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(rxPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);

        if (state->handshakeEnable) {
            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_SetErrorEvent(int32_t controllerIndex, TinyCLR_Uart_Error error) {

    auto state = &uartStates[controllerIndex];

    if (state->errorEventHandler != nullptr)
        state->errorEventHandler(state->controller, error);
}

void AT91_Uart_ReceiveData(int32_t controllerIndex, uint32_t sr) {
    AT91_USART &usart = AT91::USART(controllerIndex);

    uint8_t rxdata = usart.US_RHR;

    auto state = &uartStates[controllerIndex];

    if (state->rxBufferCount == state->rxBufferSize) {
        AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferFull);

        return;
    }

    state->RxBuffer[state->rxBufferIn++] = rxdata;

    state->rxBufferCount++;

    if (state->rxBufferIn == state->rxBufferSize)
        state->rxBufferIn = 0;

    if (state->dataReceivedEventHandler != nullptr)
        state->dataReceivedEventHandler(state->controller, 1);

    if (sr & AT91_USART::US_OVRE)
        AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Overrun);

    if (sr & AT91_USART::US_FRAME)
        AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Frame);

    if (sr & AT91_USART::US_PARE)
        AT91_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::ReceiveParity);

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

    uint32_t status = usart.US_CSR;

    if (status & AT91_USART::US_RXRDY) {
        AT91_Uart_ReceiveData(controllerIndex, status);
    }

    if (status & AT91_USART::US_TXRDY) {
        AT91_Uart_TransmitData(controllerIndex);
    }

}
int32_t AT91_Uart_GetPeripheralId(int32_t controllerIndex) {
    int32_t usartId;

    if (controllerIndex == 0) {
        usartId = (AT91C_ID_SYS);
    }
    else if ((controllerIndex > 0) && (controllerIndex < 4)) {
        usartId = (AT91C_ID_USART0 + (controllerIndex - 1));
    }
    else {
        usartId = (AT91C_ID_UART0 + (controllerIndex - 4));
    }

    return usartId;
}

TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    int32_t txPin = AT91_Uart_GetTxPin(controllerIndex);
    int32_t rxPin = AT91_Uart_GetRxPin(controllerIndex);

    if (state->isOpened || !AT91_Gpio_OpenPin(txPin) || !AT91_Gpio_OpenPin(rxPin))
        return TinyCLR_Result::SharingViolation;

    state->txBufferCount = 0;
    state->txBufferIn = 0;
    state->txBufferOut = 0;

    state->rxBufferCount = 0;
    state->rxBufferIn = 0;
    state->rxBufferOut = 0;

    state->controller = self;

    AT91_PMC &pmc = AT91::PMC();

    int32_t uartId = AT91_Uart_GetPeripheralId(controllerIndex);

    pmc.EnablePeriphClock(uartId);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

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
    AT91_Interrupt_Activate(uartId, (uint32_t*)&AT91_Uart_InterruptHandler, (void*)&state->controllerIndex);

    if (AT91_Uart_PinConfiguration(controllerIndex, true) == TinyCLR_Result::NotSupported)
        return TinyCLR_Result::NotSupported;

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

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    if (state->txBufferSize == 0) {
        state->txBufferSize = uartTxDefaultBuffersSize[controllerIndex];

        state->TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, state->txBufferSize);

        if (state->TxBuffer == nullptr) {
            state->txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (state->rxBufferSize == 0) {
        state->rxBufferSize = uartRxDefaultBuffersSize[controllerIndex];

        state->RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, state->rxBufferSize);

        if (state->RxBuffer == nullptr) {
            state->rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    usart.US_MR = USMR;

    usart.US_CR = AT91_USART::US_RXEN;
    usart.US_CR = AT91_USART::US_TXEN;

    state->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    state->txBufferCount = 0;
    state->txBufferIn = 0;
    state->txBufferOut = 0;

    state->rxBufferCount = 0;
    state->rxBufferIn = 0;
    state->rxBufferOut = 0;

    state->isOpened = false;
    state->handshakeEnable = false;

    AT91_PMC &pmc = AT91::PMC();

    int32_t uartId = AT91_Uart_GetPeripheralId(controllerIndex);

    AT91_Interrupt_Disable(uartId);

    if (state->isOpened)
        AT91_Uart_PinConfiguration(controllerIndex, false);

    pmc.DisablePeriphClock(uartId);

    if (apiManager != nullptr) {
        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

        if (state->txBufferSize != 0) {
            memoryProvider->Free(memoryProvider, state->TxBuffer);

            state->txBufferSize = 0;
        }

        if (state->rxBufferSize != 0) {
            memoryProvider->Free(memoryProvider, state->RxBuffer);

            state->rxBufferSize = 0;
        }
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

bool AT91_Uart_TxHandshakeEnabledState(int controllerIndex) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    AT91_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);

    while (state->txBufferCount > 0) {
        AT91_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length) {

    size_t i = 0;;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->isOpened == false || state->rxBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(state->rxBufferCount, length);

    while (i < length) {
        buffer[i] = state->RxBuffer[state->rxBufferOut];

        state->rxBufferOut++;
        i++;
        state->rxBufferCount--;

        if (state->rxBufferOut == state->rxBufferSize)
            state->rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->isOpened == false || state->txBufferSize == 0) {
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

TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

size_t AT91_Uart_GetUnreadCount(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferCount;
}

size_t AT91_Uart_GetUnwrittenCount(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferCount;
}

TinyCLR_Result AT91_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->rxBufferCount = state->rxBufferIn = state->rxBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->txBufferCount = state->txBufferIn = state->txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void AT91_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartStates[i].txBufferSize = 0;
        uartStates[i].rxBufferSize = 0;

        AT91_Uart_Release(&uartControllers[i]);

        uartStates[i].isOpened = false;
    }
}

TinyCLR_Result AT91_Uart_Enable(const TinyCLR_Uart_Controller* self) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_Disable(const TinyCLR_Uart_Controller* self) {
    return TinyCLR_Result::NotImplemented;
}
