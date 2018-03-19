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

static const uint32_t g_AT91_Uart_TxDefaultBuffersSize[] = AT91_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t g_AT91_Uart_RxDefaultBuffersSize[] = AT91_UART_DEFAULT_RX_BUFFER_SIZE;

struct AT91_Uart_Controller {


    uint8_t                             *TxBuffer;
    uint8_t                             *RxBuffer;
    size_t                              txBufferCount;
    size_t                              txBufferIn;
    size_t                              txBufferOut;
    size_t                              txBufferSize;

    size_t                              rxBufferCount;
    size_t                              rxBufferIn;
    size_t                              rxBufferOut;
    size_t                              rxBufferSize;

    bool                                isOpened;
    bool                                handshakeEnable;

    TinyCLR_Uart_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler    dataReceivedEventHandler;

    const TinyCLR_Uart_Provider*        provider;

};

static AT91_Uart_Controller g_UartController[TOTAL_UART_CONTROLLERS];

#define SET_BITS(Var,Shift,Mask,fieldsMask) {Var = setFieldValue(Var,Shift,Mask,fieldsMask);}

static uint8_t uartProviderDefs[TOTAL_UART_CONTROLLERS * sizeof(TinyCLR_Uart_Provider)];
static TinyCLR_Uart_Provider* uartProviders[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi;

const TinyCLR_Api_Info* AT91_Uart_GetApi() {

    for (int i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartProviders[i] = (TinyCLR_Uart_Provider*)(uartProviderDefs + (i * sizeof(TinyCLR_Uart_Provider)));
        uartProviders[i]->Parent = &uartApi;
        uartProviders[i]->Index = i;
        uartProviders[i]->Acquire = &AT91_Uart_Acquire;
        uartProviders[i]->Release = &AT91_Uart_Release;
        uartProviders[i]->SetActiveSettings = &AT91_Uart_SetActiveSettings;
        uartProviders[i]->Flush = &AT91_Uart_Flush;
        uartProviders[i]->Read = &AT91_Uart_Read;
        uartProviders[i]->Write = &AT91_Uart_Write;
        uartProviders[i]->SetPinChangedHandler = &AT91_Uart_SetPinChangedHandler;
        uartProviders[i]->SetErrorReceivedHandler = &AT91_Uart_SetErrorReceivedHandler;
        uartProviders[i]->SetDataReceivedHandler = &AT91_Uart_SetDataReceivedHandler;
        uartProviders[i]->GetBreakSignalState = AT91_Uart_GetBreakSignalState;
        uartProviders[i]->SetBreakSignalState = AT91_Uart_SetBreakSignalState;
        uartProviders[i]->GetCarrierDetectState = AT91_Uart_GetCarrierDetectState;
        uartProviders[i]->GetClearToSendState = AT91_Uart_GetClearToSendState;
        uartProviders[i]->GetDataReadyState = AT91_Uart_GetDataReadyState;
        uartProviders[i]->GetIsDataTerminalReadyEnabled = AT91_Uart_GetIsDataTerminalReadyEnabled;
        uartProviders[i]->SetIsDataTerminalReadyEnabled = AT91_Uart_SetIsDataTerminalReadyEnabled;
        uartProviders[i]->GetIsRequestToSendEnabled = AT91_Uart_GetIsRequestToSendEnabled;
        uartProviders[i]->SetIsRequestToSendEnabled = AT91_Uart_SetIsRequestToSendEnabled;
        uartProviders[i]->GetReadBufferSize = AT91_Uart_GetReadBufferSize;
        uartProviders[i]->SetReadBufferSize = AT91_Uart_SetReadBufferSize;
        uartProviders[i]->GetWriteBufferSize = AT91_Uart_GetWriteBufferSize;
        uartProviders[i]->SetWriteBufferSize = AT91_Uart_SetWriteBufferSize;
    }

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Count = TOTAL_UART_CONTROLLERS;
    uartApi.Implementation = uartProviders;

    return &uartApi;
}

static const AT91_Gpio_Pin g_at91_uart_tx_pins[] = AT91_UART_TX_PINS;
static const AT91_Gpio_Pin g_at91_uart_rx_pins[] = AT91_UART_RX_PINS;
static const AT91_Gpio_Pin g_at91_uart_rts_pins[] = AT91_UART_RTS_PINS;
static const AT91_Gpio_Pin g_at91_uart_cts_pins[] = AT91_UART_CTS_PINS;

int32_t AT91_Uart_GetTxPin(int32_t portNum) {
    return g_at91_uart_tx_pins[portNum].number;
}

int32_t AT91_Uart_GetRxPin(int32_t portNum) {
    return g_at91_uart_rx_pins[portNum].number;
}

int32_t AT91_Uart_GetRtsPin(int32_t portNum) {
    return g_at91_uart_rts_pins[portNum].number;
}

int32_t AT91_Uart_GetCtsPin(int32_t portNum) {
    return g_at91_uart_cts_pins[portNum].number;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetTxAlternateFunction(int32_t portNum) {
    return g_at91_uart_tx_pins[portNum].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRxAlternateFunction(int32_t portNum) {
    return g_at91_uart_rx_pins[portNum].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRtsAlternateFunction(int32_t portNum) {
    return g_at91_uart_rts_pins[portNum].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetCtsAlternateFunction(int32_t portNum) {
    return g_at91_uart_cts_pins[portNum].peripheralSelection;
}

TinyCLR_Result AT91_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].rxBufferSize == 0 ? g_AT91_Uart_RxDefaultBuffersSize[self->Index] : g_UartController[self->Index].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (g_UartController[self->Index].rxBufferSize) {
        memoryProvider->Free(memoryProvider, g_UartController[self->Index].RxBuffer);
    }

    g_UartController[self->Index].rxBufferSize = size;

    g_UartController[self->Index].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (g_UartController[self->Index].RxBuffer == nullptr) {
        g_UartController[self->Index].rxBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].txBufferSize == 0 ? g_AT91_Uart_TxDefaultBuffersSize[self->Index] : g_UartController[self->Index].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (g_UartController[self->Index].txBufferSize) {
        memoryProvider->Free(memoryProvider, g_UartController[self->Index].TxBuffer);
    }

    g_UartController[self->Index].txBufferSize = size;

    g_UartController[self->Index].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (g_UartController[self->Index].TxBuffer == nullptr) {
        g_UartController[self->Index].txBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_PinConfiguration(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t txPin = AT91_Uart_GetTxPin(portNum);
    uint32_t rxPin = AT91_Uart_GetRxPin(portNum);
    uint32_t ctsPin = AT91_Uart_GetCtsPin(portNum);
    uint32_t rtsPin = AT91_Uart_GetRtsPin(portNum);

    AT91_Gpio_PeripheralSelection txPinMode = AT91_Uart_GetTxAlternateFunction(portNum);
    AT91_Gpio_PeripheralSelection rxPinMode = AT91_Uart_GetRxAlternateFunction(portNum);
    AT91_Gpio_PeripheralSelection ctsPinMode = AT91_Uart_GetCtsAlternateFunction(portNum);
    AT91_Gpio_PeripheralSelection rtsPinMode = AT91_Uart_GetRtsAlternateFunction(portNum);

    AT91_Uart_TxBufferEmptyInterruptEnable(portNum, enable);

    AT91_Uart_RxBufferFullInterruptEnable(portNum, enable);

    if (enable) {
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(txPin, AT91_Gpio_Direction::Input, txPinMode, AT91_Gpio_ResistorMode::Inactive);
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(rxPin, AT91_Gpio_Direction::Input, rxPinMode, AT91_Gpio_ResistorMode::Inactive);

        if (g_UartController[portNum].handshakeEnable) {
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

        if (g_UartController[portNum].handshakeEnable) {
            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_SetErrorEvent(int32_t portNum, TinyCLR_Uart_Error error) {
    if (g_UartController[portNum].errorEventHandler != nullptr)
        g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, error);
}

void AT91_Uart_ReceiveData(int32_t portNum) {
    AT91_USART &usart = AT91::USART(portNum);

    uint8_t rxdata = usart.US_RHR;

    g_UartController[portNum].RxBuffer[g_UartController[portNum].rxBufferIn++] = rxdata;

    g_UartController[portNum].rxBufferCount++;

    if (g_UartController[portNum].rxBufferIn == g_UartController[portNum].rxBufferSize)
        g_UartController[portNum].rxBufferIn = 0;

    if (g_UartController[portNum].dataReceivedEventHandler != nullptr)
        g_UartController[portNum].dataReceivedEventHandler(g_UartController[portNum].provider, 1);

}

void AT91_Uart_TransmitData(int32_t portNum) {
    AT91_USART &usart = AT91::USART(portNum);

    if (g_UartController[portNum].txBufferCount > 0) {
        uint8_t txdata = g_UartController[portNum].TxBuffer[g_UartController[portNum].txBufferOut++];

        g_UartController[portNum].txBufferCount--;

        if (g_UartController[portNum].txBufferOut == g_UartController[portNum].txBufferSize)
            g_UartController[portNum].txBufferOut = 0;

        usart.US_THR = txdata; // write TX data

    }
    else {
        AT91_Uart_TxBufferEmptyInterruptEnable(portNum, false); // Disable interrupt when no more data to send.
    }

}
void AT91_Uart_InterruptHandler(void *param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t portNum = (uint32_t)param;

    AT91_USART &usart = AT91::USART(portNum);

    uint32_t status = usart.US_CSR;

    if (status & AT91_USART::US_RXRDY) {
        AT91_Uart_ReceiveData(portNum);
    }

    if (status & AT91_USART::US_TXRDY) {
        AT91_Uart_TransmitData(portNum);
    }

}
int32_t AT91_Uart_GetPeripheralId(int32_t portNum) {
    int32_t usartId;

    if (portNum == 0) {
        usartId = (AT91C_ID_SYS);
    }
    else if ((portNum > 0) && (portNum < 4)) {
        usartId = (AT91C_ID_US0 + (portNum - 1));
    }
    else {
        usartId = (AT91C_ID_US0 + (portNum - 4));
    }

    return usartId;
}

TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (portNum >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    DISABLE_INTERRUPTS_SCOPED(irq);

    g_UartController[portNum].txBufferCount = 0;
    g_UartController[portNum].txBufferIn = 0;
    g_UartController[portNum].txBufferOut = 0;

    g_UartController[portNum].rxBufferCount = 0;
    g_UartController[portNum].rxBufferIn = 0;
    g_UartController[portNum].rxBufferOut = 0;

    g_UartController[portNum].provider = self;

    AT91_PMC &pmc = AT91::PMC();

    int32_t uartId = AT91_Uart_GetPeripheralId(portNum);

    pmc.EnablePeriphClock(uartId);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t portNum = self->Index;

    int32_t uartId = AT91_Uart_GetPeripheralId(portNum);

    AT91_USART &usart = AT91::USART(portNum);

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

    AT91_Interrupt_Activate(uartId, (uint32_t*)&AT91_Uart_InterruptHandler, (void*)(size_t)self->Index);

    if (AT91_Uart_PinConfiguration(portNum, true) == TinyCLR_Result::NotSupported)
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

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (g_UartController[portNum].txBufferSize == 0) {
        g_UartController[portNum].txBufferSize = g_AT91_Uart_TxDefaultBuffersSize[portNum];

        g_UartController[self->Index].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[portNum].txBufferSize);

        if (g_UartController[self->Index].TxBuffer == nullptr) {
            g_UartController[self->Index].txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (g_UartController[portNum].rxBufferSize == 0) {
        g_UartController[portNum].rxBufferSize = g_AT91_Uart_RxDefaultBuffersSize[portNum];

        g_UartController[self->Index].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[portNum].rxBufferSize);

        if (g_UartController[self->Index].RxBuffer == nullptr) {
            g_UartController[self->Index].rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }
    usart.US_MR = USMR;

    usart.US_CR = AT91_USART::US_RXEN;
    usart.US_CR = AT91_USART::US_TXEN;

    g_UartController[portNum].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Provider* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t portNum = self->Index;

    g_UartController[portNum].txBufferCount = 0;
    g_UartController[portNum].txBufferIn = 0;
    g_UartController[portNum].txBufferOut = 0;

    g_UartController[portNum].rxBufferCount = 0;
    g_UartController[portNum].rxBufferIn = 0;
    g_UartController[portNum].rxBufferOut = 0;

    g_UartController[portNum].isOpened = false;
    g_UartController[portNum].handshakeEnable = false;

    AT91_PMC &pmc = AT91::PMC();

    int32_t uartId = AT91_Uart_GetPeripheralId(portNum);

    AT91_Interrupt_Disable(uartId);

    AT91_Uart_PinConfiguration(portNum, false);

    pmc.DisablePeriphClock(uartId);

    if (apiProvider != nullptr) {
        auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

        if (g_UartController[self->Index].txBufferSize != 0) {
            memoryProvider->Free(memoryProvider, g_UartController[self->Index].TxBuffer);

            g_UartController[self->Index].txBufferSize = 0;
        }

        if (g_UartController[self->Index].rxBufferSize != 0) {
            memoryProvider->Free(memoryProvider, g_UartController[self->Index].RxBuffer);

            g_UartController[self->Index].rxBufferSize = 0;
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_USART &usart = AT91::USART(portNum);

    if (enable) {
        usart.US_IER = AT91_USART::US_TXRDY;
    }
    else {
        usart.US_IDR = AT91_USART::US_TXRDY;
    }
}

void AT91_Uart_RxBufferFullInterruptEnable(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_USART &usart = AT91::USART(portNum);

    if (enable) {
        usart.US_IER = AT91_USART::US_RXRDY;
    }
    else {
        usart.US_IDR = AT91_USART::US_RXRDY;
    }
}

bool AT91_Uart_TxHandshakeEnabledState(int portNum) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (g_UartController[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    AT91_Uart_TxBufferEmptyInterruptEnable(portNum, true);

    while (g_UartController[portNum].txBufferCount > 0) {
        AT91_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    size_t i = 0;;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened == false || g_UartController[self->Index].rxBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(g_UartController[portNum].rxBufferCount, length);

    while (i < length) {
        buffer[i] = g_UartController[portNum].RxBuffer[g_UartController[portNum].rxBufferOut];

        g_UartController[portNum].rxBufferOut++;
        i++;
        g_UartController[portNum].rxBufferCount--;

        if (g_UartController[portNum].rxBufferOut == g_UartController[portNum].rxBufferSize)
            g_UartController[portNum].rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened == false || g_UartController[self->Index].txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (g_UartController[portNum].txBufferCount == g_UartController[portNum].txBufferSize) {
        AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::TransmitFull);

        return TinyCLR_Result::Busy;
    }

    length = std::min(g_UartController[portNum].txBufferSize - g_UartController[portNum].txBufferCount, length);


    while (i < length) {

        g_UartController[portNum].TxBuffer[g_UartController[portNum].txBufferIn] = buffer[i];

        g_UartController[portNum].txBufferCount++;

        i++;

        g_UartController[portNum].txBufferIn++;

        if (g_UartController[portNum].txBufferIn == g_UartController[portNum].txBufferSize)
            g_UartController[portNum].txBufferIn = 0;
    }

    if (length > 0) {
        AT91_Uart_TxBufferEmptyInterruptEnable(portNum, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result AT91_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

void AT91_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        AT91_Uart_Release(uartProviders[i]);
    }
}

