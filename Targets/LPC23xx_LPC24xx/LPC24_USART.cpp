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
#include "LPC24.h"

static const uint32_t g_LPC24_Uart_TxDefaultBuffersSize[] = LPC24_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t g_LPC24_Uart_RxDefaultBuffersSize[] = LPC24_UART_DEFAULT_RX_BUFFER_SIZE;

struct LPC24_Uart_Controller {

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

static LPC24_Uart_Controller g_UartController[TOTAL_UART_CONTROLLERS];

#define SET_BITS(Var,Shift,Mask,fieldsMask) {Var = setFieldValue(Var,Shift,Mask,fieldsMask);}

static uint8_t uartProviderDefs[TOTAL_UART_CONTROLLERS * sizeof(TinyCLR_Uart_Provider)];
static TinyCLR_Uart_Provider* uartProviders[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi;

uint32_t setFieldValue(volatile uint32_t oldVal, uint32_t shift, uint32_t mask, uint32_t val) {
    volatile uint32_t temp = oldVal;

    temp &= ~mask;
    temp |= val << shift;
    return temp;
}

const TinyCLR_Api_Info* LPC24_Uart_GetApi() {

    for (int i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartProviders[i] = (TinyCLR_Uart_Provider*)(uartProviderDefs + (i * sizeof(TinyCLR_Uart_Provider)));
        uartProviders[i]->Parent = &uartApi;
        uartProviders[i]->Index = i;
        uartProviders[i]->Acquire = &LPC24_Uart_Acquire;
        uartProviders[i]->Release = &LPC24_Uart_Release;
        uartProviders[i]->SetActiveSettings = &LPC24_Uart_SetActiveSettings;
        uartProviders[i]->Flush = &LPC24_Uart_Flush;
        uartProviders[i]->Read = &LPC24_Uart_Read;
        uartProviders[i]->Write = &LPC24_Uart_Write;
        uartProviders[i]->SetPinChangedHandler = &LPC24_Uart_SetPinChangedHandler;
        uartProviders[i]->SetErrorReceivedHandler = &LPC24_Uart_SetErrorReceivedHandler;
        uartProviders[i]->SetDataReceivedHandler = &LPC24_Uart_SetDataReceivedHandler;
        uartProviders[i]->GetBreakSignalState = LPC24_Uart_GetBreakSignalState;
        uartProviders[i]->SetBreakSignalState = LPC24_Uart_SetBreakSignalState;
        uartProviders[i]->GetCarrierDetectState = LPC24_Uart_GetCarrierDetectState;
        uartProviders[i]->GetClearToSendState = LPC24_Uart_GetClearToSendState;
        uartProviders[i]->GetDataReadyState = LPC24_Uart_GetDataReadyState;
        uartProviders[i]->GetIsDataTerminalReadyEnabled = LPC24_Uart_GetIsDataTerminalReadyEnabled;
        uartProviders[i]->SetIsDataTerminalReadyEnabled = LPC24_Uart_SetIsDataTerminalReadyEnabled;
        uartProviders[i]->GetIsRequestToSendEnabled = LPC24_Uart_GetIsRequestToSendEnabled;
        uartProviders[i]->SetIsRequestToSendEnabled = LPC24_Uart_SetIsRequestToSendEnabled;
        uartProviders[i]->GetReadBufferSize = LPC24_Uart_GetReadBufferSize;
        uartProviders[i]->SetReadBufferSize = LPC24_Uart_SetReadBufferSize;
        uartProviders[i]->GetWriteBufferSize = LPC24_Uart_GetWriteBufferSize;
        uartProviders[i]->SetWriteBufferSize = LPC24_Uart_SetWriteBufferSize;
    }

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Count = TOTAL_UART_CONTROLLERS;
    uartApi.Implementation = uartProviders;

    return &uartApi;
}

TinyCLR_Result LPC24_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].rxBufferSize == 0 ? g_LPC24_Uart_RxDefaultBuffersSize[self->Index] : g_UartController[self->Index].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
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

TinyCLR_Result LPC24_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].txBufferSize == 0 ? g_LPC24_Uart_TxDefaultBuffersSize[self->Index] : g_UartController[self->Index].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
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
void LPC24_Uart_PinConfiguration(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t txPin = LPC24_Uart_GetTxPin(portNum);
    uint32_t rxPin = LPC24_Uart_GetRxPin(portNum);
    uint32_t ctsPin = LPC24_Uart_GetCtsPin(portNum);
    uint32_t rtsPin = LPC24_Uart_GetRtsPin(portNum);

    LPC24_Gpio_PinFunction txPinMode = LPC24_Uart_GetTxAlternateFunction(portNum);
    LPC24_Gpio_PinFunction rxPinMode = LPC24_Uart_GetRxAlternateFunction(portNum);
    LPC24_Gpio_PinFunction ctsPinMode = LPC24_Uart_GetCtsAlternateFunction(portNum);
    LPC24_Gpio_PinFunction rtsPinMode = LPC24_Uart_GetRtsAlternateFunction(portNum);

    if (enable) {
        // Connect pin to UART
        LPC24_Gpio_ConfigurePin(txPin, LPC24_Gpio_Direction::Input, txPinMode, LPC24_Gpio_PinMode::Inactive);
        // Connect pin to UART
        LPC24_Gpio_ConfigurePin(rxPin, LPC24_Gpio_Direction::Input, rxPinMode, LPC24_Gpio_PinMode::Inactive);

        LPC24_Uart_TxBufferEmptyInterruptEnable(portNum, true);

        LPC24_Uart_RxBufferFullInterruptEnable(portNum, true);

        if (g_UartController[portNum].handshakeEnable) {
            LPC24_Gpio_ConfigurePin(ctsPin, LPC24_Gpio_Direction::Input, ctsPinMode, LPC24_Gpio_PinMode::Inactive);
            LPC24_Gpio_ConfigurePin(rtsPin, LPC24_Gpio_Direction::Input, rtsPinMode, LPC24_Gpio_PinMode::Inactive);
        }

    }
    else {

        LPC24_Uart_TxBufferEmptyInterruptEnable(portNum, false);
        // TODO Add config for uart pin protected state
        LPC24_Gpio_ConfigurePin(txPin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);

        LPC24_Uart_RxBufferFullInterruptEnable(portNum, false);
        // TODO Add config for uart pin protected state
        LPC24_Gpio_ConfigurePin(rxPin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);

        if (g_UartController[portNum].handshakeEnable) {
            LPC24_Gpio_ConfigurePin(ctsPin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
            LPC24_Gpio_ConfigurePin(rtsPin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
        }
    }
}

void LPC24_Uart_SetErrorEvent(int32_t portNum, TinyCLR_Uart_Error error) {
    if (g_UartController[portNum].errorEventHandler != nullptr)
        g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, error);
}

void LPC24_Uart_ReceiveData(int portNum, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);

    // Read data from Rx FIFO
    if (USARTC.SEL2.IER.UART_IER & (LPC24XX_USART::UART_IER_RDAIE)) {
        if ((LSR_Value & LPC24XX_USART::UART_LSR_RFDR) || (IIR_Value == LPC24XX_USART::UART_IIR_IID_Irpt_RDA) || (IIR_Value == LPC24XX_USART::UART_IIR_IID_Irpt_TOUT)) {
            do {
                uint8_t rxdata = (uint8_t)USARTC.SEL1.RBR.UART_RBR;

                if (0 == (LSR_Value & (LPC24XX_USART::UART_LSR_PEI | LPC24XX_USART::UART_LSR_OEI | LPC24XX_USART::UART_LSR_FEI))) {
                    if (g_UartController[portNum].rxBufferCount == g_UartController[portNum].rxBufferSize) {
                        LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveFull);

                        continue;
                    }

                    g_UartController[portNum].RxBuffer[g_UartController[portNum].rxBufferIn++] = rxdata;

                    g_UartController[portNum].rxBufferCount++;

                    if (g_UartController[portNum].rxBufferIn == g_UartController[portNum].rxBufferSize)
                        g_UartController[portNum].rxBufferIn = 0;

                    if (g_UartController[portNum].dataReceivedEventHandler != nullptr)
                        g_UartController[portNum].dataReceivedEventHandler(g_UartController[portNum].provider, 1);
                }

                LSR_Value = USARTC.UART_LSR;

                if (LSR_Value & 0x04) {
                    LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveParity);
                }
                else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
                    LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::Frame);
                }
                else if (LSR_Value & 0x02) {
                    LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::BufferOverrun);
                }
            } while (LSR_Value & LPC24XX_USART::UART_LSR_RFDR);
        }
    }
}
void LPC24_Uart_TransmitData(int portNum, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);

    // Send data
    if ((LSR_Value & LPC24XX_USART::UART_LSR_TE) || (IIR_Value == LPC24XX_USART::UART_IIR_IID_Irpt_THRE)) {
        // Check if CTS is high
        if (LPC24_Uart_TxHandshakeEnabledState(portNum)) {
            if (g_UartController[portNum].txBufferCount > 0) {
                uint8_t txdata = g_UartController[portNum].TxBuffer[g_UartController[portNum].txBufferOut++];

                g_UartController[portNum].txBufferCount--;

                if (g_UartController[portNum].txBufferOut == g_UartController[portNum].txBufferSize)
                    g_UartController[portNum].txBufferOut = 0;

                USARTC.SEL1.THR.UART_THR = txdata; // write TX data

            }
            else {
                LPC24_Uart_TxBufferEmptyInterruptEnable(portNum, false); // Disable interrupt when no more data to send.
            }
        }
    }
}

void LPC24_Uart_InterruptHandler(void *param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t portNum = (uint32_t)param;

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);
    volatile uint32_t LSR_Value = USARTC.UART_LSR;                     // Store LSR value since it's Read-to-Clear
    volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC24XX_USART::UART_IIR_IID_mask;

    if (g_UartController[portNum].handshakeEnable) {
        volatile bool dump = USARTC.UART_MSR; // Clr status register
    }

    if (LSR_Value & 0x04) {
        LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveParity);
    }
    else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
        LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::Frame);
    }
    else if (LSR_Value & 0x02) {
        LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::BufferOverrun);
    }

    LPC24_Uart_ReceiveData(portNum, LSR_Value, IIR_Value);

    LPC24_Uart_TransmitData(portNum, LSR_Value, IIR_Value);
}


TinyCLR_Result LPC24_Uart_Acquire(const TinyCLR_Uart_Provider* self) {
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

    switch (portNum) {
    case 0:
        LPC24XX::SYSCON().PCONP |= PCONP_PCUART0;
        break;

    case 1:
        LPC24XX::SYSCON().PCONP |= PCONP_PCUART1;
        break;

    case 2:
        LPC24XX::SYSCON().PCONP |= PCONP_PCUART2;
        break;

    case 3:
        LPC24XX::SYSCON().PCONP |= PCONP_PCUART3;
        break;
    }

    return TinyCLR_Result::Success;
}

void LPC24_Uart_SetClock(int32_t portNum, int32_t pclkSel) {
    pclkSel &= 0x03;

    switch (portNum) {
    case 0:

        LPC24XX::SYSCON().PCLKSEL0 &= ~(0x03 << 6);
        LPC24XX::SYSCON().PCLKSEL0 |= (pclkSel << 6);

        break;

    case 1:

        LPC24XX::SYSCON().PCLKSEL0 &= ~(0x03 << 8);
        LPC24XX::SYSCON().PCLKSEL0 |= (pclkSel << 8);

        break;

    case 2:
        LPC24XX::SYSCON().PCLKSEL1 &= ~(0x03 << 16);
        LPC24XX::SYSCON().PCLKSEL1 |= (pclkSel << 16);
        break;

    case 3:
        LPC24XX::SYSCON().PCLKSEL1 &= ~(0x03 << 18);
        LPC24XX::SYSCON().PCLKSEL1 |= (pclkSel << 18);
        break;

    }
}
TinyCLR_Result LPC24_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t portNum = self->Index;

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);

    uint32_t divisor;
    bool   fRet = true;
    uint32_t uart_clock;

    if (baudRate >= 460800) {
        uart_clock = LPC24XX_USART::c_ClockRate;
        LPC24_Uart_SetClock(portNum, 1);
    }
    else {
        uart_clock = LPC24XX_USART::c_ClockRate / 4;
        LPC24_Uart_SetClock(portNum, 0);
    }

    divisor = ((uart_clock / (baudRate * 16)));

    while ((uart_clock / (divisor * 16)) > baudRate) {
        divisor++;
    }

    // CWS: Disable interrupts
    USARTC.UART_LCR = 0; // prepare to Init UART
    USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_INTR_ALL_SET); // Disable all UART interrupts

    /* CWS: Set baud rate to baudRate bps */
    USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_DLAB; // prepare to access Divisor
    USARTC.SEL1.DLL.UART_DLL = divisor & 0xFF; //GET_LSB(divisor);
    USARTC.SEL2.DLM.UART_DLM = (divisor >> 8) & 0xFF; // GET_MSB(divisor);
    USARTC.UART_LCR &= ~LPC24XX_USART::UART_LCR_DLAB; // prepare to access RBR, THR, IER

    // CWS: Set port for 8 bit, 1 stop, no parity
    USARTC.UART_FDR = 0x10; // DIVADDVAL = 0, MULVAL = 1, DLM = 0;

    // DataBit range 5-8
    if (5 <= dataBits && dataBits <= 8) {
        SET_BITS(USARTC.UART_LCR,
            LPC24XX_USART::UART_LCR_WLS_shift,
            LPC24XX_USART::UART_LCR_WLS_mask,
            dataBits - 5);
    }
    else {   // not supported
     // set up 8 data bits incase return value is ignored

        return TinyCLR_Result::NotSupported;
    }

    switch (stopBits) {
    case TinyCLR_Uart_StopBitCount::Two:
        USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_NSB_15_STOPBITS;

        if (dataBits == 5)
            return TinyCLR_Result::NotSupported;

        break;

    case TinyCLR_Uart_StopBitCount::One:
        USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_NSB_1_STOPBITS;

        break;

    case TinyCLR_Uart_StopBitCount::OnePointFive:
        USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_NSB_15_STOPBITS;

        if (dataBits != 5)
            return TinyCLR_Result::NotSupported;

        break;

    default:

        return TinyCLR_Result::NotSupported;
    }

    switch (parity) {

    case TinyCLR_Uart_Parity::Space:
        USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_SPE;

    case TinyCLR_Uart_Parity::Even:
        USARTC.UART_LCR |= (LPC24XX_USART::UART_LCR_EPE | LPC24XX_USART::UART_LCR_PBE);
        break;

    case TinyCLR_Uart_Parity::Mark:
        USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_SPE;

    case  TinyCLR_Uart_Parity::Odd:
        USARTC.UART_LCR |= LPC24XX_USART::UART_LCR_PBE;
        break;

    case TinyCLR_Uart_Parity::None:
        USARTC.UART_LCR &= ~LPC24XX_USART::UART_LCR_PBE;
        break;

    default:

        return TinyCLR_Result::NotSupported;
    }

    if (handshaking != TinyCLR_Uart_Handshake::None && portNum != 1) // Only port 2 support handshaking
        return TinyCLR_Result::NotSupported;


    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        USARTC.UART_MCR |= (1 << 6) | (1 << 7);
        g_UartController[portNum].handshakeEnable = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    // CWS: Set the RX FIFO trigger level (to 8 bytes), reset RX, TX FIFO
    USARTC.SEL3.FCR.UART_FCR = (LPC24XX_USART::UART_FCR_RFITL_08 << LPC24XX_USART::UART_FCR_RFITL_shift) |
        LPC24XX_USART::UART_FCR_TFR |
        LPC24XX_USART::UART_FCR_RFR |
        LPC24XX_USART::UART_FCR_FME;

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (g_UartController[portNum].txBufferSize == 0) {
        g_UartController[portNum].txBufferSize = g_LPC24_Uart_TxDefaultBuffersSize[portNum];

        g_UartController[self->Index].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[portNum].txBufferSize);

        if (g_UartController[self->Index].TxBuffer == nullptr) {
            g_UartController[self->Index].txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (g_UartController[portNum].rxBufferSize == 0) {
        g_UartController[portNum].rxBufferSize = g_LPC24_Uart_RxDefaultBuffersSize[portNum];

        g_UartController[self->Index].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[portNum].rxBufferSize);

        if (g_UartController[self->Index].RxBuffer == nullptr) {
            g_UartController[self->Index].rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    LPC24_Interrupt_Activate(LPC24XX_USART::getIntNo(portNum), (uint32_t*)&LPC24_Uart_InterruptHandler, (void*)self->Index);
    LPC24_Interrupt_Enable(LPC24XX_USART::getIntNo(portNum));


    LPC24_Uart_PinConfiguration(portNum, true);

    g_UartController[portNum].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_Release(const TinyCLR_Uart_Provider* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t portNum = self->Index;

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);

    if (g_UartController[portNum].isOpened == true) {


        LPC24_Interrupt_Disable(LPC24XX_USART::getIntNo(portNum));

        LPC24_Uart_PinConfiguration(portNum, false);

        if (g_UartController[portNum].handshakeEnable) {
            USARTC.UART_MCR &= ~((1 << 6) | (1 << 7));
        }

        // CWS: Disable interrupts
        USARTC.UART_LCR = 0; // prepare to Init UART
        USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_INTR_ALL_SET);         // Disable all UART interrupt

    }

    g_UartController[portNum].txBufferCount = 0;
    g_UartController[portNum].txBufferIn = 0;
    g_UartController[portNum].txBufferOut = 0;

    g_UartController[portNum].rxBufferCount = 0;
    g_UartController[portNum].rxBufferIn = 0;
    g_UartController[portNum].rxBufferOut = 0;

    g_UartController[portNum].isOpened = false;
    g_UartController[portNum].handshakeEnable = false;

    switch (portNum) {
    case 0:
        LPC24XX::SYSCON().PCONP &= ~PCONP_PCUART0;
        break;

    case 1:
        LPC24XX::SYSCON().PCONP &= ~PCONP_PCUART1;
        break;

    case 2:
        LPC24XX::SYSCON().PCONP &= ~PCONP_PCUART2;
        break;

    case 3:
        LPC24XX::SYSCON().PCONP &= ~PCONP_PCUART3;
        break;
    }
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

void LPC24_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);

    if (enable) {
        LPC24XX::VIC().ForceInterrupt(LPC24XX_USART::getIntNo(portNum));// force interrupt as this chip has a bug????
        USARTC.SEL2.IER.UART_IER |= (LPC24XX_USART::UART_IER_THREIE);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_THREIE);
    }
}

void LPC24_Uart_RxBufferFullInterruptEnable(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(portNum);

    if (enable) {
        USARTC.SEL2.IER.UART_IER |= (LPC24XX_USART::UART_IER_RDAIE);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_RDAIE);
    }
}

bool LPC24_Uart_TxHandshakeEnabledState(int portNum) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result LPC24_Uart_Flush(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (g_UartController[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    LPC24_Uart_TxBufferEmptyInterruptEnable(portNum, true);

    while (g_UartController[portNum].txBufferCount > 0) {
        LPC24_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length) {
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

TinyCLR_Result LPC24_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened == false || g_UartController[self->Index].txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (g_UartController[portNum].txBufferCount == g_UartController[portNum].txBufferSize) {
        LPC24_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::TransmitFull);

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
        LPC24_Uart_TxBufferEmptyInterruptEnable(portNum, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result LPC24_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

void LPC24_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        if (g_UartController[i].isOpened)
            LPC24_Uart_Release(uartProviders[i]);

        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        g_UartController[i].isOpened = false;


    }
}

