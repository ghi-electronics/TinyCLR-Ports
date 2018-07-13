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
    int32_t        controller;

};

static AT91_Uart_Controller g_UartController[TOTAL_UART_CONTROLLERS];

#define SET_BITS(Var,Shift,Mask,fieldsMask) {Var = setFieldValue(Var,Shift,Mask,fieldsMask);}

static TinyCLR_Uart_Provider uartProviders;
static TinyCLR_Api_Info uartApi;

const TinyCLR_Api_Info* AT91_Uart_GetApi() {
    uartProviders.ApiInfo = &uartApi;
    uartProviders.Acquire = &AT91_Uart_Acquire;
    uartProviders.Release = &AT91_Uart_Release;
    uartProviders.SetActiveSettings = &AT91_Uart_SetActiveSettings;
    uartProviders.Flush = &AT91_Uart_Flush;
    uartProviders.Read = &AT91_Uart_Read;
    uartProviders.Write = &AT91_Uart_Write;
    uartProviders.SetPinChangedHandler = &AT91_Uart_SetPinChangedHandler;
    uartProviders.SetErrorReceivedHandler = &AT91_Uart_SetErrorReceivedHandler;
    uartProviders.SetDataReceivedHandler = &AT91_Uart_SetDataReceivedHandler;
    uartProviders.GetBreakSignalState = AT91_Uart_GetBreakSignalState;
    uartProviders.SetBreakSignalState = AT91_Uart_SetBreakSignalState;
    uartProviders.GetCarrierDetectState = AT91_Uart_GetCarrierDetectState;
    uartProviders.GetClearToSendState = AT91_Uart_GetClearToSendState;
    uartProviders.GetDataReadyState = AT91_Uart_GetDataReadyState;
    uartProviders.GetIsDataTerminalReadyEnabled = AT91_Uart_GetIsDataTerminalReadyEnabled;
    uartProviders.SetIsDataTerminalReadyEnabled = AT91_Uart_SetIsDataTerminalReadyEnabled;
    uartProviders.GetIsRequestToSendEnabled = AT91_Uart_GetIsRequestToSendEnabled;
    uartProviders.SetIsRequestToSendEnabled = AT91_Uart_SetIsRequestToSendEnabled;
    uartProviders.GetReadBufferSize = AT91_Uart_GetReadBufferSize;
    uartProviders.SetReadBufferSize = AT91_Uart_SetReadBufferSize;
    uartProviders.GetWriteBufferSize = AT91_Uart_GetWriteBufferSize;
    uartProviders.SetWriteBufferSize = AT91_Uart_SetWriteBufferSize;
    uartProviders.GetUnreadCount = &AT91_Uart_GetUnreadCount;
    uartProviders.GetUnwrittenCount = &AT91_Uart_GetUnwrittenCount;
    uartProviders.ClearReadBuffer = &AT91_Uart_ClearReadBuffer;
    uartProviders.ClearWriteBuffer = &AT91_Uart_ClearWriteBuffer;
    uartProviders.GetControllerCount = &AT91_Uart_GetControllerCount;

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Implementation = &uartProviders;

    return &uartApi;
}

static const AT91_Gpio_Pin g_at91_uart_tx_pins[] = AT91_UART_TX_PINS;
static const AT91_Gpio_Pin g_at91_uart_rx_pins[] = AT91_UART_RX_PINS;
static const AT91_Gpio_Pin g_at91_uart_rts_pins[] = AT91_UART_RTS_PINS;
static const AT91_Gpio_Pin g_at91_uart_cts_pins[] = AT91_UART_CTS_PINS;

int32_t AT91_Uart_GetTxPin(int32_t controller) {
    return g_at91_uart_tx_pins[controller].number;
}

int32_t AT91_Uart_GetRxPin(int32_t controller) {
    return g_at91_uart_rx_pins[controller].number;
}

int32_t AT91_Uart_GetRtsPin(int32_t controller) {
    return g_at91_uart_rts_pins[controller].number;
}

int32_t AT91_Uart_GetCtsPin(int32_t controller) {
    return g_at91_uart_cts_pins[controller].number;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetTxAlternateFunction(int32_t controller) {
    return g_at91_uart_tx_pins[controller].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRxAlternateFunction(int32_t controller) {
    return g_at91_uart_rx_pins[controller].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRtsAlternateFunction(int32_t controller) {
    return g_at91_uart_rts_pins[controller].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetCtsAlternateFunction(int32_t controller) {
    return g_at91_uart_cts_pins[controller].peripheralSelection;
}

TinyCLR_Result AT91_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& size) {
    size = g_UartController[controller].rxBufferSize == 0 ? g_AT91_Uart_RxDefaultBuffersSize[controller] : g_UartController[controller].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (g_UartController[controller].rxBufferSize) {
        memoryProvider->Free(memoryProvider, g_UartController[controller].RxBuffer);
    }

    g_UartController[controller].rxBufferSize = size;

    g_UartController[controller].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (g_UartController[controller].RxBuffer == nullptr) {
        g_UartController[controller].rxBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& size) {
    size = g_UartController[controller].txBufferSize == 0 ? g_AT91_Uart_TxDefaultBuffersSize[controller] : g_UartController[controller].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (g_UartController[controller].txBufferSize) {
        memoryProvider->Free(memoryProvider, g_UartController[controller].TxBuffer);
    }

    g_UartController[controller].txBufferSize = size;

    g_UartController[controller].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (g_UartController[controller].TxBuffer == nullptr) {
        g_UartController[controller].txBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_PinConfiguration(int controller, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t txPin = AT91_Uart_GetTxPin(controller);
    uint32_t rxPin = AT91_Uart_GetRxPin(controller);
    uint32_t ctsPin = AT91_Uart_GetCtsPin(controller);
    uint32_t rtsPin = AT91_Uart_GetRtsPin(controller);

    AT91_Gpio_PeripheralSelection txPinMode = AT91_Uart_GetTxAlternateFunction(controller);
    AT91_Gpio_PeripheralSelection rxPinMode = AT91_Uart_GetRxAlternateFunction(controller);
    AT91_Gpio_PeripheralSelection ctsPinMode = AT91_Uart_GetCtsAlternateFunction(controller);
    AT91_Gpio_PeripheralSelection rtsPinMode = AT91_Uart_GetRtsAlternateFunction(controller);

    AT91_Uart_TxBufferEmptyInterruptEnable(controller, enable);

    AT91_Uart_RxBufferFullInterruptEnable(controller, enable);

    if (enable) {
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(txPin, AT91_Gpio_Direction::Input, txPinMode, AT91_Gpio_ResistorMode::Inactive);
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(rxPin, AT91_Gpio_Direction::Input, rxPinMode, AT91_Gpio_ResistorMode::Inactive);

        if (g_UartController[controller].handshakeEnable) {
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

        if (g_UartController[controller].handshakeEnable) {
            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_SetErrorEvent(int32_t controller, TinyCLR_Uart_Error error) {
    if (g_UartController[controller].errorEventHandler != nullptr)
        g_UartController[controller].errorEventHandler(g_UartController[controller].provider, controller, error);
}

void AT91_Uart_ReceiveData(int32_t controller, uint32_t sr) {
    AT91_USART &usart = AT91::USART(controller);

    uint8_t rxdata = usart.US_RHR;

    if (g_UartController[controller].rxBufferCount == g_UartController[controller].rxBufferSize) {
        AT91_Uart_SetErrorEvent(controller, TinyCLR_Uart_Error::ReceiveFull);

        return;
    }

    g_UartController[controller].RxBuffer[g_UartController[controller].rxBufferIn++] = rxdata;

    g_UartController[controller].rxBufferCount++;

    if (g_UartController[controller].rxBufferIn == g_UartController[controller].rxBufferSize)
        g_UartController[controller].rxBufferIn = 0;

    if (g_UartController[controller].dataReceivedEventHandler != nullptr)
        g_UartController[controller].dataReceivedEventHandler(g_UartController[controller].provider, controller, 1);

    if (sr & AT91_USART::US_OVRE)
        AT91_Uart_SetErrorEvent(controller, TinyCLR_Uart_Error::BufferOverrun);

    if (sr & AT91_USART::US_FRAME)
        AT91_Uart_SetErrorEvent(controller, TinyCLR_Uart_Error::Frame);

    if (sr & AT91_USART::US_PARE)
        AT91_Uart_SetErrorEvent(controller, TinyCLR_Uart_Error::ReceiveParity);

}

void AT91_Uart_TransmitData(int32_t controller) {
    AT91_USART &usart = AT91::USART(controller);

    if (g_UartController[controller].txBufferCount > 0) {
        uint8_t txdata = g_UartController[controller].TxBuffer[g_UartController[controller].txBufferOut++];

        g_UartController[controller].txBufferCount--;

        if (g_UartController[controller].txBufferOut == g_UartController[controller].txBufferSize)
            g_UartController[controller].txBufferOut = 0;

        usart.US_THR = txdata; // write TX data

    }
    else {
        AT91_Uart_TxBufferEmptyInterruptEnable(controller, false); // Disable interrupt when no more data to send.
    }

}
void AT91_Uart_InterruptHandler(void *param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t controller = *reinterpret_cast<uint32_t*>(param);

    AT91_USART &usart = AT91::USART(controller);

    uint32_t status = usart.US_CSR;

    if (status & AT91_USART::US_RXRDY) {
        AT91_Uart_ReceiveData(controller, status);
    }

    if (status & AT91_USART::US_TXRDY) {
        AT91_Uart_TransmitData(controller);
    }

}
int32_t AT91_Uart_GetPeripheralId(int32_t controller) {
    int32_t usartId;

    if (controller == 0) {
        usartId = (AT91C_ID_SYS);
    }
    else if ((controller > 0) && (controller < 4)) {
        usartId = (AT91C_ID_US0 + (controller - 1));
    }
    else {
        usartId = (AT91C_ID_US0 + (controller - 4));
    }

    return usartId;
}

TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Provider* self, int32_t controller) {
    if (controller >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t txPin = AT91_Uart_GetTxPin(controller);
    int32_t rxPin = AT91_Uart_GetRxPin(controller);

    if (g_UartController[controller].isOpened || !AT91_Gpio_OpenPin(txPin) || !AT91_Gpio_OpenPin(rxPin))
        return TinyCLR_Result::SharingViolation;

    g_UartController[controller].txBufferCount = 0;
    g_UartController[controller].txBufferIn = 0;
    g_UartController[controller].txBufferOut = 0;

    g_UartController[controller].rxBufferCount = 0;
    g_UartController[controller].rxBufferIn = 0;
    g_UartController[controller].rxBufferOut = 0;

    g_UartController[controller].provider = self;

    AT91_PMC &pmc = AT91::PMC();

    int32_t uartId = AT91_Uart_GetPeripheralId(controller);

    pmc.EnablePeriphClock(uartId);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, int32_t controller, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t uartId = AT91_Uart_GetPeripheralId(controller);

    AT91_USART &usart = AT91::USART(controller);

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

    g_UartController[controller].controller = controller;

    AT91_Interrupt_Activate(uartId, (uint32_t*)&AT91_Uart_InterruptHandler, (void*)(size_t)&g_UartController[controller].controller);

    if (AT91_Uart_PinConfiguration(controller, true) == TinyCLR_Result::NotSupported)
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

    if (g_UartController[controller].txBufferSize == 0) {
        g_UartController[controller].txBufferSize = g_AT91_Uart_TxDefaultBuffersSize[controller];

        g_UartController[controller].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[controller].txBufferSize);

        if (g_UartController[controller].TxBuffer == nullptr) {
            g_UartController[controller].txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (g_UartController[controller].rxBufferSize == 0) {
        g_UartController[controller].rxBufferSize = g_AT91_Uart_RxDefaultBuffersSize[controller];

        g_UartController[controller].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[controller].rxBufferSize);

        if (g_UartController[controller].RxBuffer == nullptr) {
            g_UartController[controller].rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }
    usart.US_MR = USMR;

    usart.US_CR = AT91_USART::US_RXEN;
    usart.US_CR = AT91_USART::US_TXEN;

    g_UartController[controller].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Provider* self, int32_t controller) {
    DISABLE_INTERRUPTS_SCOPED(irq);



    g_UartController[controller].txBufferCount = 0;
    g_UartController[controller].txBufferIn = 0;
    g_UartController[controller].txBufferOut = 0;

    g_UartController[controller].rxBufferCount = 0;
    g_UartController[controller].rxBufferIn = 0;
    g_UartController[controller].rxBufferOut = 0;

    g_UartController[controller].isOpened = false;
    g_UartController[controller].handshakeEnable = false;

    AT91_PMC &pmc = AT91::PMC();

    int32_t uartId = AT91_Uart_GetPeripheralId(controller);

    AT91_Interrupt_Disable(uartId);
    if (g_UartController[controller].isOpened) {
        AT91_Uart_PinConfiguration(controller, false);
    }

    pmc.DisablePeriphClock(uartId);

    if (apiProvider != nullptr) {
        auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

        if (g_UartController[controller].txBufferSize != 0) {
            memoryProvider->Free(memoryProvider, g_UartController[controller].TxBuffer);

            g_UartController[controller].txBufferSize = 0;
        }

        if (g_UartController[controller].rxBufferSize != 0) {
            memoryProvider->Free(memoryProvider, g_UartController[controller].RxBuffer);

            g_UartController[controller].rxBufferSize = 0;
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_TxBufferEmptyInterruptEnable(int controller, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_USART &usart = AT91::USART(controller);

    if (enable) {
        usart.US_IER = AT91_USART::US_TXRDY;
    }
    else {
        usart.US_IDR = AT91_USART::US_TXRDY;
    }
}

void AT91_Uart_RxBufferFullInterruptEnable(int controller, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    AT91_USART &usart = AT91::USART(controller);

    if (enable) {
        usart.US_IER = AT91_USART::US_RXRDY;
    }
    else {
        usart.US_IDR = AT91_USART::US_RXRDY;
    }
}

bool AT91_Uart_TxHandshakeEnabledState(int controller) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Provider* self, int32_t controller) {


    if (g_UartController[controller].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    AT91_Uart_TxBufferEmptyInterruptEnable(controller, true);

    while (g_UartController[controller].txBufferCount > 0) {
        AT91_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Provider* self, int32_t controller, uint8_t* buffer, size_t& length) {

    size_t i = 0;;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[controller].isOpened == false || g_UartController[controller].rxBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(g_UartController[controller].rxBufferCount, length);

    while (i < length) {
        buffer[i] = g_UartController[controller].RxBuffer[g_UartController[controller].rxBufferOut];

        g_UartController[controller].rxBufferOut++;
        i++;
        g_UartController[controller].rxBufferCount--;

        if (g_UartController[controller].rxBufferOut == g_UartController[controller].rxBufferSize)
            g_UartController[controller].rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Provider* self, int32_t controller, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[controller].isOpened == false || g_UartController[controller].txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (g_UartController[controller].txBufferCount == g_UartController[controller].txBufferSize) {
        AT91_Uart_SetErrorEvent(controller, TinyCLR_Uart_Error::TransmitFull);

        return TinyCLR_Result::Busy;
    }

    length = std::min(g_UartController[controller].txBufferSize - g_UartController[controller].txBufferCount, length);


    while (i < length) {

        g_UartController[controller].TxBuffer[g_UartController[controller].txBufferIn] = buffer[i];

        g_UartController[controller].txBufferCount++;

        i++;

        g_UartController[controller].txBufferIn++;

        if (g_UartController[controller].txBufferIn == g_UartController[controller].txBufferSize)
            g_UartController[controller].txBufferIn = 0;
    }

    if (length > 0) {
        AT91_Uart_TxBufferEmptyInterruptEnable(controller, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, int32_t controller, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result AT91_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, int32_t controller, TinyCLR_Uart_ErrorReceivedHandler handler) {


    g_UartController[controller].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, int32_t controller, TinyCLR_Uart_DataReceivedHandler handler) {


    g_UartController[controller].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_Uart_GetUnreadCount(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& count) {
    count = g_UartController[controller].rxBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_GetUnwrittenCount(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& count) {
    count = g_UartController[controller].txBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_ClearReadBuffer(const TinyCLR_Uart_Provider* self, int32_t controller) {
    g_UartController[controller].rxBufferCount = g_UartController[controller].rxBufferIn = g_UartController[controller].rxBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_ClearWriteBuffer(const TinyCLR_Uart_Provider* self, int32_t controller) {
    g_UartController[controller].txBufferCount = g_UartController[controller].txBufferIn = g_UartController[controller].txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void AT91_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        AT91_Uart_Release(&uartProviders, i);

        g_UartController[i].isOpened = false;
    }
}

TinyCLR_Result AT91_Uart_GetControllerCount(const TinyCLR_Uart_Provider* self, int32_t& count) {
    count = TOTAL_UART_CONTROLLERS;

    return TinyCLR_Result::Success;
}

