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

struct AT91_Uart_Controller {
    uint8_t                             TxBuffer[AT91_UART_TX_BUFFER_SIZE];
    uint8_t                             RxBuffer[AT91_UART_RX_BUFFER_SIZE];

    size_t                              txBufferCount;
    size_t                              txBufferIn;
    size_t                              txBufferOut;

    size_t                              rxBufferCount;
    size_t                              rxBufferIn;
    size_t                              rxBufferOut;

    bool                                isOpened;
    bool                                handshakeEnable;

    TinyCLR_Uart_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler    dataReceivedEventHandler;

    const TinyCLR_Uart_Provider*        provider;

};

static AT91_Uart_Controller g_AT91_Uart_Controller[TOTAL_UART_CONTROLLERS];

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
    }

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Count = TOTAL_UART_CONTROLLERS;
    uartApi.Implementation = uartProviders;

    return &uartApi;
}


void AT91_Uart_PinConfiguration(int portNum, bool enable) {
    GLOBAL_LOCK(irq);

    uint32_t txPin = AT91_Uart_GetTxPin(portNum);
    uint32_t rxPin = AT91_Uart_GetRxPin(portNum);
    uint32_t ctsPin = AT91_Uart_GetCtsPin(portNum);
    uint32_t rtsPin = AT91_Uart_GetRtsPin(portNum);

    AT91_Gpio_PinFunction txPinMode = AT91_Uart_GetTxAlternateFunction(portNum);
    AT91_Gpio_PinFunction rxPinMode = AT91_Uart_GetRxAlternateFunction(portNum);
    AT91_Gpio_PinFunction ctsPinMode = AT91_Uart_GetCtsAlternateFunction(portNum);
    AT91_Gpio_PinFunction rtsPinMode = AT91_Uart_GetRtsAlternateFunction(portNum);

    if (enable) {
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(txPin, AT91_Gpio_Direction::Input, txPinMode, AT91_Gpio_PinMode::Inactive);
        // Connect pin to UART
        AT91_Gpio_ConfigurePin(rxPin, AT91_Gpio_Direction::Input, rxPinMode, AT91_Gpio_PinMode::Inactive);

        AT91_Uart_TxBufferEmptyInterruptEnable(portNum, true);

        AT91_Uart_RxBufferFullInterruptEnable(portNum, true);

        if (g_AT91_Uart_Controller[portNum].handshakeEnable) {
            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, ctsPinMode, AT91_Gpio_PinMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, rtsPinMode, AT91_Gpio_PinMode::Inactive);
        }

    }
    else {

        AT91_Uart_TxBufferEmptyInterruptEnable(portNum, false);
        // TODO Add config for uart pin protected state
        AT91_Gpio_ConfigurePin(txPin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);

        AT91_Uart_RxBufferFullInterruptEnable(portNum, false);
        // TODO Add config for uart pin protected state
        AT91_Gpio_ConfigurePin(rxPin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);

        if (g_AT91_Uart_Controller[portNum].handshakeEnable) {
            AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
            AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
        }
    }
}

void AT91_Uart_SetErrorEvent(int32_t portNum, TinyCLR_Uart_Error error) {
    if (g_AT91_Uart_Controller[portNum].errorEventHandler != nullptr)
        g_AT91_Uart_Controller[portNum].errorEventHandler(g_AT91_Uart_Controller[portNum].provider, error);
}

void AT91_Uart_ReceiveData(int portNum, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_START

        GLOBAL_LOCK(irq);

    AT91XX_USART& USARTC = AT91XX::UART(portNum);

    // Read data from Rx FIFO
    if (USARTC.SEL2.IER.UART_IER & (AT91XX_USART::UART_IER_RDAIE)) {
        if ((LSR_Value & AT91XX_USART::UART_LSR_RFDR) || (IIR_Value == AT91XX_USART::UART_IIR_IID_Irpt_RDA) || (IIR_Value == AT91XX_USART::UART_IIR_IID_Irpt_TOUT)) {
            do {
                uint8_t rxdata = (uint8_t)USARTC.SEL1.RBR.UART_RBR;

                if (0 == (LSR_Value & (AT91XX_USART::UART_LSR_PEI | AT91XX_USART::UART_LSR_OEI | AT91XX_USART::UART_LSR_FEI))) {
                    if (g_AT91_Uart_Controller[portNum].rxBufferCount == AT91_UART_RX_BUFFER_SIZE) {
                        AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveFull);

                        continue;
                    }

                    g_AT91_Uart_Controller[portNum].RxBuffer[g_AT91_Uart_Controller[portNum].rxBufferIn++] = rxdata;

                    g_AT91_Uart_Controller[portNum].rxBufferCount++;

                    if (g_AT91_Uart_Controller[portNum].rxBufferIn == AT91_UART_RX_BUFFER_SIZE)
                        g_AT91_Uart_Controller[portNum].rxBufferIn = 0;

                    if (g_AT91_Uart_Controller[portNum].dataReceivedEventHandler != nullptr)
                        g_AT91_Uart_Controller[portNum].dataReceivedEventHandler(g_AT91_Uart_Controller[portNum].provider, 1);
                }

                LSR_Value = USARTC.UART_LSR;

                if (LSR_Value & 0x04) {
                    AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveParity);
                }
                else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
                    AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::Frame);
                }
                else if (LSR_Value & 0x02) {
                    AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::BufferOverrun);
                }
            } while (LSR_Value & AT91XX_USART::UART_LSR_RFDR);
        }
    }


    INTERRUPT_END
}
void AT91_Uart_TransmitData(int portNum, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_START

        GLOBAL_LOCK(irq);

    AT91XX_USART& USARTC = AT91XX::UART(portNum);

    // Send data
    if ((LSR_Value & AT91XX_USART::UART_LSR_TE) || (IIR_Value == AT91XX_USART::UART_IIR_IID_Irpt_THRE)) {
        // Check if CTS is high
        if (AT91_Uart_TxHandshakeEnabledState(portNum)) {
            if (g_AT91_Uart_Controller[portNum].txBufferCount > 0) {
                uint8_t txdata = g_AT91_Uart_Controller[portNum].TxBuffer[g_AT91_Uart_Controller[portNum].txBufferOut++];

                g_AT91_Uart_Controller[portNum].txBufferCount--;

                if (g_AT91_Uart_Controller[portNum].txBufferOut == AT91_UART_TX_BUFFER_SIZE)
                    g_AT91_Uart_Controller[portNum].txBufferOut = 0;

                USARTC.SEL1.THR.UART_THR = txdata; // write TX data

            }
            else {
                AT91_Uart_TxBufferEmptyInterruptEnable(portNum, false); // Disable interrupt when no more data to send.
            }
        }
    }


    INTERRUPT_END
}

void AT91_Uart_InterruptHandler(void *param) {
    INTERRUPT_START

        GLOBAL_LOCK(irq);

    uint32_t portNum = (uint32_t)param;

    AT91XX_USART& USARTC = AT91XX::UART(portNum);
    volatile uint32_t LSR_Value = USARTC.UART_LSR;                     // Store LSR value since it's Read-to-Clear
    volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & AT91XX_USART::UART_IIR_IID_mask;

    if (LSR_Value & 0x04) {
        AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveParity);
    }
    else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
        AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::Frame);
    }
    else if (LSR_Value & 0x02) {
        AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::BufferOverrun);
    }

    AT91_Uart_ReceiveData(portNum, LSR_Value, IIR_Value);

    AT91_Uart_TransmitData(portNum, LSR_Value, IIR_Value);

    INTERRUPT_END
}


TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (portNum >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    GLOBAL_LOCK(irq);

    g_AT91_Uart_Controller[portNum].txBufferCount = 0;
    g_AT91_Uart_Controller[portNum].txBufferIn = 0;
    g_AT91_Uart_Controller[portNum].txBufferOut = 0;

    g_AT91_Uart_Controller[portNum].rxBufferCount = 0;
    g_AT91_Uart_Controller[portNum].rxBufferIn = 0;
    g_AT91_Uart_Controller[portNum].rxBufferOut = 0;

    g_AT91_Uart_Controller[portNum].provider = self;

    switch (portNum) {
        case 0:
            AT91XX::SYSCON().PCONP |= PCONP_PCUART0;
            break;

        case 1:
            AT91XX::SYSCON().PCONP |= PCONP_PCUART1;
            break;

        case 2:
            AT91XX::SYSCON().PCONP |= PCONP_PCUART2;
            break;

        case 3:
            AT91XX::SYSCON().PCONP |= PCONP_PCUART3;
            break;
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_SetClock(int32_t portNum, int32_t pclkSel) {
    pclkSel &= 0x03;

    switch (portNum) {
        case 0:

            AT91XX::SYSCON().PCLKSEL0 &= ~(0x03 << 6);
            AT91XX::SYSCON().PCLKSEL0 |= (pclkSel << 6);

            break;

        case 1:

            AT91XX::SYSCON().PCLKSEL0 &= ~(0x03 << 8);
            AT91XX::SYSCON().PCLKSEL0 |= (pclkSel << 8);

            break;

        case 2:
            AT91XX::SYSCON().PCLKSEL1 &= ~(0x03 << 16);
            AT91XX::SYSCON().PCLKSEL1 |= (pclkSel << 16);
            break;

        case 3:
            AT91XX::SYSCON().PCLKSEL1 &= ~(0x03 << 18);
            AT91XX::SYSCON().PCLKSEL1 |= (pclkSel << 18);
            break;

    }
}
TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    GLOBAL_LOCK(irq);

    int32_t portNum = self->Index;

    AT91XX_USART& USARTC = AT91XX::UART(portNum);

    uint32_t divisor;
    uint32_t fdr;
    bool   fRet = true;

    switch (baudRate) {

        case 2400: AT91_Uart_SetClock(portNum, 0); fdr = 0x41; divisor = 0x177; break;

        case 4800: AT91_Uart_SetClock(portNum, 0); fdr = 0xE3; divisor = 0xC1; break;

        case 9600: AT91_Uart_SetClock(portNum, 0); fdr = 0xC7; divisor = 0x4A; break;

        case 14400: AT91_Uart_SetClock(portNum, 0); fdr = 0xA1; divisor = 0x47; break;

        case 19200: AT91_Uart_SetClock(portNum, 0); fdr = 0xC7; divisor = 0x25; break;

        case 38400: AT91_Uart_SetClock(portNum, 0); fdr = 0xB3; divisor = 0x17; break;

        case 57600: AT91_Uart_SetClock(portNum, 0); fdr = 0x92; divisor = 0x10; break;

        case 115200: AT91_Uart_SetClock(portNum, 0); fdr = 0x92; divisor = 0x08; break;

        case 230400: AT91_Uart_SetClock(portNum, 0); fdr = 0x92; divisor = 0x04; break;

        case 460800: AT91_Uart_SetClock(portNum, 1); fdr = 0x92; divisor = 0x08; break;

        case 921600: AT91_Uart_SetClock(portNum, 1); fdr = 0x92; divisor = 0x04; break;

        default:
            AT91_Uart_SetClock(portNum, 1);
            divisor = ((AT91XX_USART::c_ClockRate / (baudRate * 16)));
            fdr = 0x10;

    }

    // CWS: Disable interrupts
    USARTC.UART_LCR = 0; // prepare to Init UART
    USARTC.SEL2.IER.UART_IER &= ~(AT91XX_USART::UART_IER_INTR_ALL_SET);          // Disable all UART interrupts
    /* CWS: Set baud rate to baudRate bps */
    USARTC.UART_LCR |= AT91XX_USART::UART_LCR_DLAB;                                          // prepare to access Divisor
    USARTC.SEL1.DLL.UART_DLL = divisor & 0xFF;      //GET_LSB(divisor);                                                      // Set baudrate.
    USARTC.SEL2.DLM.UART_DLM = (divisor >> 8) & 0xFF; // GET_MSB(divisor);
    USARTC.UART_LCR &= ~AT91XX_USART::UART_LCR_DLAB;                                              // prepare to access RBR, THR, IER
    // CWS: Set port for 8 bit, 1 stop, no parity

    USARTC.UART_FDR = fdr;

    // DataBit range 5-8
    if (5 <= dataBits && dataBits <= 8) {
        SET_BITS(USARTC.UART_LCR,
                 AT91XX_USART::UART_LCR_WLS_shift,
                 AT91XX_USART::UART_LCR_WLS_mask,
                 dataBits - 5);
    }
    else {   // not supported
     // set up 8 data bits incase return value is ignored

        return TinyCLR_Result::NotSupported;
    }

    switch (stopBits) {
        case TinyCLR_Uart_StopBitCount::Two:
            USARTC.UART_LCR |= AT91XX_USART::UART_LCR_NSB_15_STOPBITS;

            if (dataBits == 5)
                return TinyCLR_Result::NotSupported;

            break;

        case TinyCLR_Uart_StopBitCount::One:
            USARTC.UART_LCR |= AT91XX_USART::UART_LCR_NSB_1_STOPBITS;

            break;

        case TinyCLR_Uart_StopBitCount::OnePointFive:
            USARTC.UART_LCR |= AT91XX_USART::UART_LCR_NSB_15_STOPBITS;

            if (dataBits != 5)
                return TinyCLR_Result::NotSupported;

            break;

        default:

            return TinyCLR_Result::NotSupported;
    }

    switch (parity) {

        case TinyCLR_Uart_Parity::Space:
            USARTC.UART_LCR |= AT91XX_USART::UART_LCR_SPE;

        case TinyCLR_Uart_Parity::Even:
            USARTC.UART_LCR |= (AT91XX_USART::UART_LCR_EPE | AT91XX_USART::UART_LCR_PBE);
            break;

        case TinyCLR_Uart_Parity::Mark:
            USARTC.UART_LCR |= AT91XX_USART::UART_LCR_SPE;

        case  TinyCLR_Uart_Parity::Odd:
            USARTC.UART_LCR |= AT91XX_USART::UART_LCR_PBE;
            break;

        case TinyCLR_Uart_Parity::None:
            USARTC.UART_LCR &= ~AT91XX_USART::UART_LCR_PBE;
            break;

        default:

            return TinyCLR_Result::NotSupported;
    }

    if (handshaking != TinyCLR_Uart_Handshake::None && portNum != 2) // Only port 2 support handshaking
        return TinyCLR_Result::NotSupported;


    switch (handshaking) {
        case TinyCLR_Uart_Handshake::RequestToSend:
            USARTC.UART_MCR |= (1 << 6) | (1 << 7);
            g_AT91_Uart_Controller[portNum].handshakeEnable = true;
            break;

        case TinyCLR_Uart_Handshake::XOnXOff:
        case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
            return TinyCLR_Result::NotSupported;
    }

    // CWS: Set the RX FIFO trigger level (to 8 bytes), reset RX, TX FIFO
    USARTC.SEL3.FCR.UART_FCR = (AT91XX_USART::UART_FCR_RFITL_08 << AT91XX_USART::UART_FCR_RFITL_shift) |
        AT91XX_USART::UART_FCR_TFR |
        AT91XX_USART::UART_FCR_RFR |
        AT91XX_USART::UART_FCR_FME;


    AT91_Interrupt_Activate(AT91XX_USART::getIntNo(portNum), (uint32_t*)&AT91_Uart_InterruptHandler, (void*)self->Index);
    AT91_Interrupt_Enable(AT91XX_USART::getIntNo(portNum));

    AT91_Uart_PinConfiguration(portNum, true);

    g_AT91_Uart_Controller[portNum].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Provider* self) {
    GLOBAL_LOCK(irq);

    int32_t portNum = self->Index;

    AT91XX_USART& USARTC = AT91XX::UART(portNum);

    if (g_AT91_Uart_Controller[portNum].isOpened == true) {


        AT91_Interrupt_Disable(AT91XX_USART::getIntNo(portNum));

        AT91_Uart_PinConfiguration(portNum, false);

        if (g_AT91_Uart_Controller[portNum].handshakeEnable) {
            USARTC.UART_MCR &= ~((1 << 6) | (1 << 7));
        }

        // CWS: Disable interrupts
        USARTC.UART_LCR = 0; // prepare to Init UART
        USARTC.SEL2.IER.UART_IER &= ~(AT91XX_USART::UART_IER_INTR_ALL_SET);         // Disable all UART interrupt

    }

    g_AT91_Uart_Controller[portNum].txBufferCount = 0;
    g_AT91_Uart_Controller[portNum].txBufferIn = 0;
    g_AT91_Uart_Controller[portNum].txBufferOut = 0;

    g_AT91_Uart_Controller[portNum].rxBufferCount = 0;
    g_AT91_Uart_Controller[portNum].rxBufferIn = 0;
    g_AT91_Uart_Controller[portNum].rxBufferOut = 0;

    g_AT91_Uart_Controller[portNum].isOpened = false;
    g_AT91_Uart_Controller[portNum].handshakeEnable = false;

    switch (portNum) {
        case 0:
            AT91XX::SYSCON().PCONP &= ~PCONP_PCUART0;
            break;

        case 1:
            AT91XX::SYSCON().PCONP &= ~PCONP_PCUART1;
            break;

        case 2:
            AT91XX::SYSCON().PCONP &= ~PCONP_PCUART2;
            break;

        case 3:
            AT91XX::SYSCON().PCONP &= ~PCONP_PCUART3;
            break;
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable) {
    GLOBAL_LOCK(irq);

    AT91XX_USART& USARTC = AT91XX::UART(portNum);

    if (enable) {
        AT91XX::VIC().ForceInterrupt(AT91XX_USART::getIntNo(portNum));// force interrupt as this chip has a bug????
        USARTC.SEL2.IER.UART_IER |= (AT91XX_USART::UART_IER_THREIE);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(AT91XX_USART::UART_IER_THREIE);
    }
}

void AT91_Uart_RxBufferFullInterruptEnable(int portNum, bool enable) {
    GLOBAL_LOCK(irq);

    AT91XX_USART& USARTC = AT91XX::UART(portNum);

    if (enable) {
        USARTC.SEL2.IER.UART_IER |= (AT91XX_USART::UART_IER_RDAIE);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(AT91XX_USART::UART_IER_RDAIE);
    }
}

bool AT91_Uart_TxHandshakeEnabledState(int portNum) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (g_AT91_Uart_Controller[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    AT91_Uart_TxBufferEmptyInterruptEnable(portNum, true);

    while (g_AT91_Uart_Controller[portNum].txBufferCount > 0) {
        AT91_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    size_t i = 0;;

    GLOBAL_LOCK(irq);

    if (g_AT91_Uart_Controller[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    length = std::min(g_AT91_Uart_Controller[portNum].rxBufferCount, length);

    while (i < length) {
        buffer[i] = g_AT91_Uart_Controller[portNum].RxBuffer[g_AT91_Uart_Controller[portNum].rxBufferOut];

        g_AT91_Uart_Controller[portNum].rxBufferOut++;
        i++;
        g_AT91_Uart_Controller[portNum].rxBufferCount--;

        if (g_AT91_Uart_Controller[portNum].rxBufferOut == AT91_UART_RX_BUFFER_SIZE)
            g_AT91_Uart_Controller[portNum].rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    int32_t i = 0;

    GLOBAL_LOCK(irq);

    if (g_AT91_Uart_Controller[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    if (g_AT91_Uart_Controller[portNum].txBufferCount == AT91_UART_TX_BUFFER_SIZE) {
        AT91_Uart_SetErrorEvent(portNum, TinyCLR_Uart_Error::TransmitFull);

        return TinyCLR_Result::Busy;
    }

    length = std::min(AT91_UART_TX_BUFFER_SIZE - g_AT91_Uart_Controller[portNum].txBufferCount, length);


    while (i < length) {

        g_AT91_Uart_Controller[portNum].TxBuffer[g_AT91_Uart_Controller[portNum].txBufferIn] = buffer[i];

        g_AT91_Uart_Controller[portNum].txBufferCount++;

        i++;

        g_AT91_Uart_Controller[portNum].txBufferIn++;

        if (g_AT91_Uart_Controller[portNum].txBufferIn == AT91_UART_TX_BUFFER_SIZE)
            g_AT91_Uart_Controller[portNum].txBufferIn = 0;
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

    g_AT91_Uart_Controller[portNum].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_AT91_Uart_Controller[portNum].dataReceivedEventHandler = handler;

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
        AT91_Uart_Release(uartProviders[i]);
    }
}

