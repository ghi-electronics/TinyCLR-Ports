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

static const uint32_t uartTxDefaultBuffersSize[] = LPC24_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t uartRxDefaultBuffersSize[] = LPC24_UART_DEFAULT_RX_BUFFER_SIZE;

struct UartState {
    int32_t controllerIndex;

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

    const TinyCLR_Uart_Controller*        controller;

};

#define SET_BITS(Var,Shift,Mask,fieldsMask) {Var = setFieldValue(Var,Shift,Mask,fieldsMask);}

uint32_t setFieldValue(volatile uint32_t oldVal, uint32_t shift, uint32_t mask, uint32_t val) {
    volatile uint32_t temp = oldVal;

    temp &= ~mask;
    temp |= val << shift;
    return temp;
}

static UartState uartStates[TOTAL_UART_CONTROLLERS];
static TinyCLR_Uart_Controller uartControllers[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi[TOTAL_UART_CONTROLLERS];

const TinyCLR_Api_Info* LPC24_Uart_GetApi() {
    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartControllers[i].ApiInfo = &uartApi[i];
        uartControllers[i].Acquire = &LPC24_Uart_Acquire;
        uartControllers[i].Release = &LPC24_Uart_Release;
        uartControllers[i].SetActiveSettings = &LPC24_Uart_SetActiveSettings;
        uartControllers[i].Flush = &LPC24_Uart_Flush;
        uartControllers[i].Read = &LPC24_Uart_Read;
        uartControllers[i].Write = &LPC24_Uart_Write;
        uartControllers[i].SetPinChangedHandler = &LPC24_Uart_SetPinChangedHandler;
        uartControllers[i].SetErrorReceivedHandler = &LPC24_Uart_SetErrorReceivedHandler;
        uartControllers[i].SetDataReceivedHandler = &LPC24_Uart_SetDataReceivedHandler;
        uartControllers[i].GetBreakSignalState = &LPC24_Uart_GetBreakSignalState;
        uartControllers[i].SetBreakSignalState = &LPC24_Uart_SetBreakSignalState;
        uartControllers[i].GetCarrierDetectState = &LPC24_Uart_GetCarrierDetectState;
        uartControllers[i].GetClearToSendState = &LPC24_Uart_GetClearToSendState;
        uartControllers[i].GetDataReadyState = &LPC24_Uart_GetDataReadyState;
        uartControllers[i].GetIsDataTerminalReadyEnabled = &LPC24_Uart_GetIsDataTerminalReadyEnabled;
        uartControllers[i].SetIsDataTerminalReadyEnabled = &LPC24_Uart_SetIsDataTerminalReadyEnabled;
        uartControllers[i].GetIsRequestToSendEnabled = &LPC24_Uart_GetIsRequestToSendEnabled;
        uartControllers[i].SetIsRequestToSendEnabled = &LPC24_Uart_SetIsRequestToSendEnabled;
        uartControllers[i].GetReadBufferSize = &LPC24_Uart_GetReadBufferSize;
        uartControllers[i].SetReadBufferSize = &LPC24_Uart_SetReadBufferSize;
        uartControllers[i].GetWriteBufferSize = &LPC24_Uart_GetWriteBufferSize;
        uartControllers[i].SetWriteBufferSize = &LPC24_Uart_SetWriteBufferSize;
        uartControllers[i].GetUnreadCount = &LPC24_Uart_GetUnreadCount;
        uartControllers[i].GetUnwrittenCount = &LPC24_Uart_GetUnwrittenCount;
        uartControllers[i].ClearReadBuffer = &LPC24_Uart_ClearReadBuffer;
        uartControllers[i].ClearWriteBuffer = &LPC24_Uart_ClearWriteBuffer;

        uartApi[i].Author = "GHI Electronics, LLC";
        uartApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.UartController";
        uartApi[i].Type = TinyCLR_Api_Type::UartController;
        uartApi[i].Version = 0;
        uartApi[i].Implementation = &uartControllers[i];
        uartApi[i].State = &uartStates[i];

        uartStates[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&uartApi;
}

TinyCLR_Result LPC24_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t& size) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    size = driver->rxBufferSize == 0 ? uartRxDefaultBuffersSize[controllerIndex] : driver->rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (driver->rxBufferSize) {
        memoryProvider->Free(memoryProvider, driver->RxBuffer);
    }

    driver->rxBufferSize = size;

    driver->RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (driver->RxBuffer == nullptr) {
        driver->rxBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t& size) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    size = driver->txBufferSize == 0 ? uartTxDefaultBuffersSize[controllerIndex] : driver->txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (size <= 0)
        return TinyCLR_Result::ArgumentInvalid;

    if (driver->txBufferSize) {
        memoryProvider->Free(memoryProvider, driver->TxBuffer);
    }

    driver->txBufferSize = size;

    driver->TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, size);

    if (driver->TxBuffer == nullptr) {
        driver->txBufferSize = 0;

        return TinyCLR_Result::OutOfMemory;
    }

    return TinyCLR_Result::Success;
}


void LPC24_Uart_PinConfiguration(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = &uartStates[controllerIndex];

    int32_t txPin = LPC24_Uart_GetTxPin(controllerIndex);
    int32_t rxPin = LPC24_Uart_GetRxPin(controllerIndex);
    int32_t ctsPin = LPC24_Uart_GetCtsPin(controllerIndex);
    int32_t rtsPin = LPC24_Uart_GetRtsPin(controllerIndex);

    LPC24_Gpio_PinFunction txPinMode = LPC24_Uart_GetTxAlternateFunction(controllerIndex);
    LPC24_Gpio_PinFunction rxPinMode = LPC24_Uart_GetRxAlternateFunction(controllerIndex);
    LPC24_Gpio_PinFunction ctsPinMode = LPC24_Uart_GetCtsAlternateFunction(controllerIndex);
    LPC24_Gpio_PinFunction rtsPinMode = LPC24_Uart_GetRtsAlternateFunction(controllerIndex);

    if (enable) {
        // Connect pin to UART
        LPC24_Gpio_ConfigurePin(txPin, LPC24_Gpio_Direction::Input, txPinMode, LPC24_Gpio_PinMode::Inactive);
        // Connect pin to UART
        LPC24_Gpio_ConfigurePin(rxPin, LPC24_Gpio_Direction::Input, rxPinMode, LPC24_Gpio_PinMode::Inactive);

        LPC24_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);

        LPC24_Uart_RxBufferFullInterruptEnable(controllerIndex, true);

        if (driver->handshakeEnable) {
            if (!LPC24_Gpio_OpenPin(ctsPin) || !LPC24_Gpio_OpenPin(rtsPin))
                return;
            LPC24_Gpio_ConfigurePin(ctsPin, LPC24_Gpio_Direction::Input, ctsPinMode, LPC24_Gpio_PinMode::Inactive);
            LPC24_Gpio_ConfigurePin(rtsPin, LPC24_Gpio_Direction::Input, rtsPinMode, LPC24_Gpio_PinMode::Inactive);
        }

    }
    else {

        LPC24_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false);
        // TODO Add config for uart pin protected state
        LPC24_Gpio_ClosePin(txPin);

        LPC24_Uart_RxBufferFullInterruptEnable(controllerIndex, false);
        // TODO Add config for uart pin protected state
        LPC24_Gpio_ClosePin(rxPin);

        if (driver->handshakeEnable) {
            LPC24_Gpio_ClosePin(ctsPin);
            LPC24_Gpio_ClosePin(rtsPin);
        }
    }
}

void LPC24_Uart_SetErrorEvent(int32_t controllerIndex, TinyCLR_Uart_Error error) {
    auto driver = &uartStates[controllerIndex];

    if (driver->errorEventHandler != nullptr)
        driver->errorEventHandler(driver->controller, error);
}

void LPC24_Uart_ReceiveData(int controllerIndex, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);

    auto driver = &uartStates[controllerIndex];

    // Read data from Rx FIFO
    if (USARTC.SEL2.IER.UART_IER & (LPC24XX_USART::UART_IER_RDAIE)) {
        if ((LSR_Value & LPC24XX_USART::UART_LSR_RFDR) || (IIR_Value == LPC24XX_USART::UART_IIR_IID_Irpt_RDA) || (IIR_Value == LPC24XX_USART::UART_IIR_IID_Irpt_TOUT)) {
            do {
                uint8_t rxdata = (uint8_t)USARTC.SEL1.RBR.UART_RBR;

                if (0 == (LSR_Value & (LPC24XX_USART::UART_LSR_PEI | LPC24XX_USART::UART_LSR_OEI | LPC24XX_USART::UART_LSR_FEI))) {
                    if (driver->rxBufferCount == driver->rxBufferSize) {
                        LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::ReceiveFull);

                        continue;
                    }

                    driver->RxBuffer[driver->rxBufferIn++] = rxdata;

                    driver->rxBufferCount++;

                    if (driver->rxBufferIn == driver->rxBufferSize)
                        driver->rxBufferIn = 0;

                    if (driver->dataReceivedEventHandler != nullptr)
                        driver->dataReceivedEventHandler(driver->controller, 1);
                }

                LSR_Value = USARTC.UART_LSR;

                if (LSR_Value & 0x04) {
                    LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::ReceiveParity);
                }
                else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
                    LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Frame);
                }
                else if (LSR_Value & 0x02) {
                    LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferOverrun);
                }
            } while (LSR_Value & LPC24XX_USART::UART_LSR_RFDR);
        }
    }
}
void LPC24_Uart_TransmitData(int controllerIndex, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);

    auto driver = &uartStates[controllerIndex];

    // Send data
    if ((LSR_Value & LPC24XX_USART::UART_LSR_TE) || (IIR_Value == LPC24XX_USART::UART_IIR_IID_Irpt_THRE)) {
        // Check if CTS is high
        if (LPC24_Uart_TxHandshakeEnabledState(controllerIndex)) {
            if (driver->txBufferCount > 0) {
                uint8_t txdata = driver->TxBuffer[driver->txBufferOut++];

                driver->txBufferCount--;

                if (driver->txBufferOut == driver->txBufferSize)
                    driver->txBufferOut = 0;

                USARTC.SEL1.THR.UART_THR = txdata; // write TX data

            }
            else {
                LPC24_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false); // Disable interrupt when no more data to send.
            }
        }
    }
}

void LPC24_Uart_InterruptHandler(void *param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t controllerIndex = *reinterpret_cast<uint32_t*>(param);

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);
    volatile uint32_t LSR_Value = USARTC.UART_LSR;                     // Store LSR value since it's Read-to-Clear
    volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC24XX_USART::UART_IIR_IID_mask;

    auto driver = &uartStates[controllerIndex];
    if (driver->handshakeEnable) {
        volatile bool dump = USARTC.UART_MSR; // Clr status register
    }

    if (LSR_Value & 0x04) {
        LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::ReceiveParity);
    }
    else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
        LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Frame);
    }
    else if (LSR_Value & 0x02) {
        LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferOverrun);
    }

    LPC24_Uart_ReceiveData(controllerIndex, LSR_Value, IIR_Value);

    LPC24_Uart_TransmitData(controllerIndex, LSR_Value, IIR_Value);
}


TinyCLR_Result LPC24_Uart_Acquire(const TinyCLR_Uart_Controller* self) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (controllerIndex >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t txPin = LPC24_Uart_GetTxPin(controllerIndex);
    int32_t rxPin = LPC24_Uart_GetRxPin(controllerIndex);

    if (driver->isOpened || !LPC24_Gpio_OpenPin(txPin) || !LPC24_Gpio_OpenPin(rxPin))
        return TinyCLR_Result::SharingViolation;

    driver->txBufferCount = 0;
    driver->txBufferIn = 0;
    driver->txBufferOut = 0;

    driver->rxBufferCount = 0;
    driver->rxBufferIn = 0;
    driver->rxBufferOut = 0;

    driver->controller = self;

    switch (controllerIndex) {
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

void LPC24_Uart_SetClock(int32_t controllerIndex, int32_t pclkSel) {
    pclkSel &= 0x03;

    switch (controllerIndex) {
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
TinyCLR_Result LPC24_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);

    uint32_t divisor;
    bool   fRet = true;
    uint32_t uart_clock;

    if (baudRate >= 460800) {
        uart_clock = LPC24XX_USART::c_ClockRate;
        LPC24_Uart_SetClock(controllerIndex, 1);
    }
    else {
        uart_clock = LPC24XX_USART::c_ClockRate / 4;
        LPC24_Uart_SetClock(controllerIndex, 0);
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

    if (handshaking != TinyCLR_Uart_Handshake::None && controllerIndex != 1) // Only port 2 support handshaking
        return TinyCLR_Result::NotSupported;


    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        USARTC.UART_MCR |= (1 << 6) | (1 << 7);
        driver->handshakeEnable = true;
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

    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

    if (driver->txBufferSize == 0) {
        driver->txBufferSize = uartTxDefaultBuffersSize[controllerIndex];

        driver->TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, driver->txBufferSize);

        if (driver->TxBuffer == nullptr) {
            driver->txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (driver->rxBufferSize == 0) {
        driver->rxBufferSize = uartRxDefaultBuffersSize[controllerIndex];

        driver->RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, driver->rxBufferSize);

        if (driver->RxBuffer == nullptr) {
            driver->rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }


    LPC24_Interrupt_Activate(LPC24XX_USART::getIntNo(controllerIndex), (uint32_t*)&LPC24_Uart_InterruptHandler, (void*)&driver->controllerIndex);
    LPC24_Interrupt_Enable(LPC24XX_USART::getIntNo(controllerIndex));

    LPC24_Uart_PinConfiguration(controllerIndex, true);

    driver->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_Release(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);

    LPC24_Interrupt_Disable(LPC24XX_USART::getIntNo(controllerIndex));

    if (driver->isOpened) {
        if (driver->handshakeEnable) {
            USARTC.UART_MCR &= ~((1 << 6) | (1 << 7));
        }

        // CWS: Disable interrupts
        USARTC.UART_LCR = 0; // prepare to Init UART
        USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_INTR_ALL_SET);         // Disable all UART interrupt

        LPC24_Uart_PinConfiguration(controllerIndex, false);
    }

    driver->txBufferCount = 0;
    driver->txBufferIn = 0;
    driver->txBufferOut = 0;

    driver->rxBufferCount = 0;
    driver->rxBufferIn = 0;
    driver->rxBufferOut = 0;

    driver->isOpened = false;
    driver->handshakeEnable = false;

    switch (controllerIndex) {
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
    if (apiManager != nullptr) {
        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

        if (driver->txBufferSize != 0) {
            memoryProvider->Free(memoryProvider, driver->TxBuffer);

            driver->txBufferSize = 0;
        }

        if (driver->rxBufferSize != 0) {
            memoryProvider->Free(memoryProvider, driver->RxBuffer);

            driver->rxBufferSize = 0;
        }
    }

    return TinyCLR_Result::Success;
}

void LPC24_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);

    if (enable) {
        LPC24XX::VIC().ForceInterrupt(LPC24XX_USART::getIntNo(controllerIndex));// force interrupt as this chip has a bug????
        USARTC.SEL2.IER.UART_IER |= (LPC24XX_USART::UART_IER_THREIE);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_THREIE);
    }
}

void LPC24_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC24XX_USART& USARTC = LPC24XX::UART(controllerIndex);

    if (enable) {
        USARTC.SEL2.IER.UART_IER |= (LPC24XX_USART::UART_IER_RDAIE);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC24XX_USART::UART_IER_RDAIE);
    }
}

bool LPC24_Uart_TxHandshakeEnabledState(int controllerIndex) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result LPC24_Uart_Flush(const TinyCLR_Uart_Controller* self) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (driver->isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    LPC24_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);

    while (driver->txBufferCount > 0) {
        LPC24_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    size_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (driver->isOpened == false || driver->rxBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(driver->rxBufferCount, length);

    while (i < length) {
        buffer[i] = driver->RxBuffer[driver->rxBufferOut];

        driver->rxBufferOut++;
        i++;
        driver->rxBufferCount--;

        if (driver->rxBufferOut == driver->rxBufferSize)
            driver->rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (driver->isOpened == false || driver->txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (driver->txBufferCount == driver->txBufferSize) {
        LPC24_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::TransmitFull);

        return TinyCLR_Result::Busy;
    }

    length = std::min(driver->txBufferSize - driver->txBufferCount, length);


    while (i < length) {

        driver->TxBuffer[driver->txBufferIn] = buffer[i];

        driver->txBufferCount++;

        i++;

        driver->txBufferIn++;

        if (driver->txBufferIn == driver->txBufferSize)
            driver->txBufferIn = 0;
    }

    if (length > 0) {
        LPC24_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetPinChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result LPC24_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    driver->errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    driver->dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_GetBreakSignalState(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_SetBreakSignalState(const TinyCLR_Uart_Controller* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetCarrierDetectState(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetDataReadyState(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Controller* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC24_Uart_GetUnreadCount(const TinyCLR_Uart_Controller* self, size_t& count) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    count = driver->rxBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_GetUnwrittenCount(const TinyCLR_Uart_Controller* self, size_t& count) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    count = driver->txBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    driver->rxBufferCount = driver->rxBufferIn = driver->rxBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self) {
    auto driver = reinterpret_cast<UartState*>(self->ApiInfo->State);

    driver->txBufferCount = driver->txBufferIn = driver->txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void LPC24_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartStates[i].txBufferSize = 0;
        uartStates[i].rxBufferSize = 0;

        LPC24_Uart_Release(&uartControllers[i]);

        uartStates[i].isOpened = false;
    }
}

TinyCLR_Result LPC24_Uart_GetControllerCount(const TinyCLR_Uart_Controller* self, int32_t& count) {
    count = TOTAL_UART_CONTROLLERS;

    return TinyCLR_Result::Success;
}
