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
#include "LPC17.h"

struct LPC17xx_USART {
    static const uint32_t c_Uart_0 = 0;
    static const uint32_t c_Uart_1 = 1;
    static const uint32_t c_Uart_2 = 2;
    static const uint32_t c_Uart_3 = 3;
    static const uint32_t c_Uart_4 = 4;
    static const uint32_t c_MaxUart_no = 4;

    static const uint32_t c_UartBase_0 = 0x4000C000;
    static const uint32_t c_UartBase_1 = 0x40010000;
    static const uint32_t c_UartBase_2 = 0x40098000;
    static const uint32_t c_UartBase_3 = 0x4009C000;
    static const uint32_t c_UartBase_4 = 0x400A4000;

    static const uint32_t c_ClockRate = LPC17_SYSTEM_CLOCK_HZ / 2;

    static const uint32_t c_MAX_BAUDRATE = c_ClockRate / 16;
    static const uint32_t c_MIN_BAUDRATE = 0;

    //IER
    static const uint32_t UART_IER_RLSIE = 0x00000004;   //Receive line status interrupt enable.
    static const uint32_t UART_IER_THREIE = 0x00000002;   //transmit hold register empty interrupt enable.
    static const uint32_t UART_IER_RDAIE = 0x00000001;   //Receive data available interrupt enable.

    static const uint32_t UART_IER_INTR_ALL_SET = UART_IER_RLSIE | UART_IER_THREIE | UART_IER_RDAIE;

    //IID
    static const uint32_t UART_IIR_FMES = 0x00000080;   //FIFO MODE Enable status.
    static const uint32_t UART_IIR_RFTLS_mask = 0x00000060;   //RX FIFO threshold level status.
    static const uint32_t UART_IIR_RFTLS_shift = 5;
    static const uint32_t UART_IIR_IID_mask = 0x0000000F;   //Interrupt identification.
    // values.
    static const uint32_t UART_IIR_IID_Irpt_RLS = 0x00000006;   // Receiver line status interrupt (e.g. overrun error) .
    static const uint32_t UART_IIR_IID_Irpt_RDA = 0x00000004;   // Receive data ready interrupt.
    static const uint32_t UART_IIR_IID_Irpt_TOUT = 0x0000000C;   // Receive FIFO timeout interrupt.
    static const uint32_t UART_IIR_IID_Irpt_THRE = 0x00000002;   // Transmitter holding register empty.

    static const uint32_t UART_IIR_NIP = 0x00000001;   //There is no pending interrupt.

    //FCR
    static const uint32_t UART_FCR_RFITL_mask = 0x000000C0;     // Rx FIFO trigger level
    static const uint32_t UART_FCR_RFITL_shift = 6;

    static const uint32_t UART_FCR_RFITL_01 = 0x00000000;
    static const uint32_t UART_FCR_RFITL_04 = 0x00000001;
    static const uint32_t UART_FCR_RFITL_08 = 0x00000002;
    static const uint32_t UART_FCR_RFITL_14 = 0x00000003;

    static const uint32_t UART_FCR_TFR = 0x00000004;     // Tx FIFO reset
    static const uint32_t UART_FCR_RFR = 0x00000002;     // Rx FIFO reset
    static const uint32_t UART_FCR_FME = 0x00000001;     // FIFO Mode enable


    union {
        struct {
            /****/ volatile uint32_t UART_RBR;                             //receive data register
        } RBR;
        struct {
            /****/ volatile uint32_t UART_THR;                            //transmit data register.

        } THR;
        struct {
            /****/ volatile uint32_t UART_DLL;                            //Divisor Latch register. (LSB)
        } DLL;

    } SEL1;

    union {
        struct {
            /****/ volatile uint32_t UART_IER;                                //Interrupt enable register
        } IER;
        struct {
            /****/ volatile uint32_t UART_DLM;                               //Divisor Latch register.  (MSB)
        } DLM;
    } SEL2;

    union {
        struct {
            /****/ volatile uint32_t  UART_IIR;                                        //Interrupt identification register.
        } IIR;
        struct {
            /****/ volatile uint32_t  UART_FCR;
        } FCR;
    } SEL3;

    /****/ volatile uint32_t UART_LCR;                                   // line control register.
    //--//
    static const uint32_t UART_LCR_DLAB = 0x00000080;     // Dividor Latch Access bit.
    static const uint32_t UART_LCR_BCB = 0x00000040;     // Break control bit.
    static const uint32_t UART_LCR_SPE = 0x00000020;     // Stick parity enable.
    static const uint32_t UART_LCR_EPE = 0x00000010;     // Even  parity enable.
    static const uint32_t UART_LCR_PBE = 0x00000008;     // Parity bit enable.
    static const uint32_t UART_LCR_NSB_1_STOPBITS = 0x00000000;     // Number of stop bits (0 - 1 stop bit; 1 - 1.5 stop bits).
    static const uint32_t UART_LCR_NSB_15_STOPBITS = 0x00000004;     // Number of stop bits (0 - 1 stop bit; 1 - 1.5 stop bits).
    static const uint32_t UART_LCR_WLS_mask = 0x00000003;     // Word length select.
    static const uint32_t UART_LCR_WLS_shift = 0;
    static const uint32_t UART_LCR_WLS_5_BITS = 0x00000000;
    static const uint32_t UART_LCR_WLS_6_BITS = 0x00000001;
    static const uint32_t UART_LCR_WLS_7_BITS = 0x00000002;
    static const uint32_t UART_LCR_WLS_8_BITS = 0x00000003;

    /****/ volatile uint32_t UART_MCR;                        // modem control register.

    /****/ volatile uint32_t UART_LSR;                                   //line status register.
    static const uint32_t UART_LSR_ERR_RX = 0x00000080;     //Rx FIFO error
    static const uint32_t UART_LSR_TE = 0x00000040;     //Transmitter Empty.
    static const uint32_t UART_LSR_THRE = 0x00000020;     //Transmitter Holding register Empty.
    static const uint32_t UART_LSR_BII = 0x00000010;     //Break interrupt indicator.
    static const uint32_t UART_LSR_FEI = 0x00000008;     //Framing Error indicator.
    static const uint32_t UART_LSR_PEI = 0x00000004;     //Parity Error indicator.
    static const uint32_t UART_LSR_OEI = 0x00000002;     //Overrun Error indicator.
    static const uint32_t UART_LSR_RFDR = 0x00000001;     //RX FIFO data ready.

    /****/ volatile uint32_t UART_MSR;                        //Modem status register.

    /****/ volatile uint32_t UART_SCR;                                   //Scratch pad register.

    /****/ volatile uint32_t UART_ACR;                                   //Autobaud control register.
    static const uint32_t UART_ACR_START = 0x00000001;     // Start bit
    static const uint32_t UART_ACR_MODE1 = 0x00000002;     // Mode 1
    static const uint32_t UART_ACR_AUTO_RESTART = 0x00000004;     // Auto Restart
    static const uint32_t UART_ACR_AUTOBAUD_INT_CLR = 0x00000100;     // Autobaud interrupt clear
    static const uint32_t UART_ACR_TIMEOUT_INT_CLR = 0x00000200;     // Autobaud timeout interrupt clear

    /****/ volatile uint32_t UART_ICR;                                   //IrDA control register.

    /****/ volatile uint32_t UART_FDR;                                   //Fractional divider register.
    static const uint32_t UART_FDR_DIVADDVAL_mask = 0x0000000F;
    static const uint32_t UART_FDR_DIVADDVAL_shift = 0x00000000;
    static const uint32_t UART_FDR_MULVAL_mask = 0x000000F0;
    static const uint32_t UART_FDR_MULVAL_shift = 0x00000004;

    /****/ volatile uint32_t PADDING_3;

    /****/ volatile uint32_t UART_TER;                                   //Transmit Enable register.
    static const uint32_t UART_TER_TXEN = 0x00000080;  //TX Enable bit

    static LPC17xx_USART  & UART(int sel) {


        if (sel == LPC17xx_USART::c_Uart_0) {

            return *(LPC17xx_USART  *)(size_t)LPC17xx_USART::c_UartBase_0;

        }
        else if (sel == LPC17xx_USART::c_Uart_1) {

            return *(LPC17xx_USART  *)(size_t)LPC17xx_USART::c_UartBase_1;

        }
        else if (sel == LPC17xx_USART::c_Uart_2) {

            return *(LPC17xx_USART  *)(size_t)LPC17xx_USART::c_UartBase_2;

        }
        else if (sel == LPC17xx_USART::c_Uart_3) {

            return *(LPC17xx_USART  *)(size_t)LPC17xx_USART::c_UartBase_3;

        }
        else {

            return *(LPC17xx_USART  *)(size_t)LPC17xx_USART::c_UartBase_4;

        }
    }
};

static const uint32_t g_LPC17_Uart_TxDefaultBuffersSize[] = LPC17_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t g_LPC17_Uart_RxDefaultBuffersSize[] = LPC17_UART_DEFAULT_RX_BUFFER_SIZE;

struct UartController {
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

static UartController g_UartController[TOTAL_UART_CONTROLLERS];

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

const TinyCLR_Api_Info* LPC17_Uart_GetApi() {

    for (int i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartProviders[i] = (TinyCLR_Uart_Provider*)(uartProviderDefs + (i * sizeof(TinyCLR_Uart_Provider)));
        uartProviders[i]->Parent = &uartApi;
        uartProviders[i]->Index = i;
        uartProviders[i]->Acquire = &LPC17_Uart_Acquire;
        uartProviders[i]->Release = &LPC17_Uart_Release;
        uartProviders[i]->SetActiveSettings = &LPC17_Uart_SetActiveSettings;
        uartProviders[i]->Flush = &LPC17_Uart_Flush;
        uartProviders[i]->Read = &LPC17_Uart_Read;
        uartProviders[i]->Write = &LPC17_Uart_Write;
        uartProviders[i]->SetPinChangedHandler = &LPC17_Uart_SetPinChangedHandler;
        uartProviders[i]->SetErrorReceivedHandler = &LPC17_Uart_SetErrorReceivedHandler;
        uartProviders[i]->SetDataReceivedHandler = &LPC17_Uart_SetDataReceivedHandler;
        uartProviders[i]->GetBreakSignalState = LPC17_Uart_GetBreakSignalState;
        uartProviders[i]->SetBreakSignalState = LPC17_Uart_SetBreakSignalState;
        uartProviders[i]->GetCarrierDetectState = LPC17_Uart_GetCarrierDetectState;
        uartProviders[i]->GetClearToSendState = LPC17_Uart_GetClearToSendState;
        uartProviders[i]->GetDataReadyState = LPC17_Uart_GetDataReadyState;
        uartProviders[i]->GetIsDataTerminalReadyEnabled = LPC17_Uart_GetIsDataTerminalReadyEnabled;
        uartProviders[i]->SetIsDataTerminalReadyEnabled = LPC17_Uart_SetIsDataTerminalReadyEnabled;
        uartProviders[i]->GetIsRequestToSendEnabled = LPC17_Uart_GetIsRequestToSendEnabled;
        uartProviders[i]->SetIsRequestToSendEnabled = LPC17_Uart_SetIsRequestToSendEnabled;
        uartProviders[i]->GetReadBufferSize = LPC17_Uart_GetReadBufferSize;
        uartProviders[i]->SetReadBufferSize = LPC17_Uart_SetReadBufferSize;
        uartProviders[i]->GetWriteBufferSize = LPC17_Uart_GetWriteBufferSize;
        uartProviders[i]->SetWriteBufferSize = LPC17_Uart_SetWriteBufferSize;
    }

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Count = TOTAL_UART_CONTROLLERS;
    uartApi.Implementation = uartProviders;

    LPC17_Uart_Reset();

    return &uartApi;
}

TinyCLR_Result LPC17_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
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

TinyCLR_Result LPC17_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
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


void LPC17_Uart_PinConfiguration(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t txPin = LPC17_Uart_GetTxPin(portNum);
    int32_t rxPin = LPC17_Uart_GetRxPin(portNum);
    int32_t ctsPin = LPC17_Uart_GetCtsPin(portNum);
    int32_t rtsPin = LPC17_Uart_GetRtsPin(portNum);

    LPC17_Gpio_PinFunction txPinMode = LPC17_Uart_GetTxAlternateFunction(portNum);
    LPC17_Gpio_PinFunction rxPinMode = LPC17_Uart_GetRxAlternateFunction(portNum);
    LPC17_Gpio_PinFunction ctsPinMode = LPC17_Uart_GetCtsAlternateFunction(portNum);
    LPC17_Gpio_PinFunction rtsPinMode = LPC17_Uart_GetRtsAlternateFunction(portNum);

    if (enable) {
        // Connect pin to UART
        LPC17_Gpio_ConfigurePin(txPin, LPC17_Gpio_Direction::Input, txPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        // Connect pin to UART
        LPC17_Gpio_ConfigurePin(rxPin, LPC17_Gpio_Direction::Input, rxPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

        LPC17_Uart_TxBufferEmptyInterruptEnable(portNum, true);

        LPC17_Uart_RxBufferFullInterruptEnable(portNum, true);

        if (g_UartController[portNum].handshakeEnable) {
            if (!LPC17_Gpio_OpenPin(ctsPin) || !LPC17_Gpio_OpenPin(rtsPin))
                return;

            LPC17_Gpio_ConfigurePin(ctsPin, LPC17_Gpio_Direction::Input, ctsPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
            LPC17_Gpio_ConfigurePin(rtsPin, LPC17_Gpio_Direction::Input, rtsPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

        }

    }
    else {

        LPC17_Uart_TxBufferEmptyInterruptEnable(portNum, false);
        // TODO Add config for uart pin protected state
        LPC17_Gpio_ClosePin(txPin);

        LPC17_Uart_RxBufferFullInterruptEnable(portNum, false);
        // TODO Add config for uart pin protected state
        LPC17_Gpio_ClosePin(rxPin);

        if (g_UartController[portNum].handshakeEnable) {
            LPC17_Gpio_ClosePin(ctsPin);
            LPC17_Gpio_ClosePin(rtsPin);
        }
    }
}

void UART_SetErrorEvent(int32_t portNum, TinyCLR_Uart_Error error) {
    if (g_UartController[portNum].errorEventHandler != nullptr)
        g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, error);
}

void LPC17_Uart_ReceiveData(int portNum, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    // Read data from Rx FIFO
    if (USARTC.SEL2.IER.UART_IER & (LPC17xx_USART::UART_IER_RDAIE)) {
        if ((LSR_Value & LPC17xx_USART::UART_LSR_RFDR) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_RDA) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_TOUT)) {
            do {
                uint8_t rxdata = (uint8_t)USARTC.SEL1.RBR.UART_RBR;

                if (0 == (LSR_Value & (LPC17xx_USART::UART_LSR_PEI | LPC17xx_USART::UART_LSR_OEI | LPC17xx_USART::UART_LSR_FEI))) {
                    if (g_UartController[portNum].rxBufferCount == g_UartController[portNum].rxBufferSize) {
                        UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveFull);

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
                    UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveParity);
                }
                else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
                    UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::Frame);
                }
                else if (LSR_Value & 0x02) {
                    UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::BufferOverrun);
                }
            } while (LSR_Value & LPC17xx_USART::UART_LSR_RFDR);
        }
    }
}
void LPC17_Uart_TransmitData(int portNum, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    // Send data
    if ((LSR_Value & LPC17xx_USART::UART_LSR_TE) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_THRE)) {
        // Check if CTS is high
        if (LPC17_Uart_TxHandshakeEnabledState(portNum)) {
            if (g_UartController[portNum].txBufferCount > 0) {
                uint8_t txdata = g_UartController[portNum].TxBuffer[g_UartController[portNum].txBufferOut++];

                g_UartController[portNum].txBufferCount--;

                if (g_UartController[portNum].txBufferOut == g_UartController[portNum].txBufferSize)
                    g_UartController[portNum].txBufferOut = 0;

                USARTC.SEL1.THR.UART_THR = txdata; // write TX data

            }
            else {
                LPC17_Uart_TxBufferEmptyInterruptEnable(portNum, false); // Disable interrupt when no more data to send.
            }
        }
    }
}

void UART_IntHandler(int portNum) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    volatile uint32_t LSR_Value = USARTC.UART_LSR;           // Store LSR value since it's Read-to-Clear
    volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC17xx_USART::UART_IIR_IID_mask;

    if (g_UartController[portNum].handshakeEnable) {
        volatile bool dump = USARTC.UART_MSR; // Clr status register
    }

    if (LSR_Value & 0x04) {
        UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::ReceiveParity);
    }
    else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
        UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::Frame);
    }
    else if (LSR_Value & 0x02) {
        UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::BufferOverrun);
    }

    LPC17_Uart_ReceiveData(portNum, LSR_Value, IIR_Value);

    LPC17_Uart_TransmitData(portNum, LSR_Value, IIR_Value);
}
//--//
void UART0_IntHandler(void *param) {
    UART_IntHandler(0);
}

void UART1_IntHandler(void *param) {
    UART_IntHandler(1);
}

void UART2_IntHandler(void *param) {
    UART_IntHandler(2);
}

void UART3_IntHandler(void *param) {
    UART_IntHandler(3);
}
void UART4_IntHandler(void *param) {
    UART_IntHandler(4);
}

TinyCLR_Result LPC17_Uart_Acquire(const TinyCLR_Uart_Provider* self) {
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

    // Enable power config
    switch (portNum) {

    case 0: LPC_SC->PCONP |= PCONP_PCUART0; break;

    case 1: LPC_SC->PCONP |= PCONP_PCUART1; break;

    case 2: LPC_SC->PCONP |= PCONP_PCUART2; break;

    case 3: LPC_SC->PCONP |= PCONP_PCUART3; break;

    case 4: LPC_SC->PCONP |= PCONP_PCUART4; break;

    }

    int32_t txPin = LPC17_Uart_GetTxPin(portNum);
    int32_t rxPin = LPC17_Uart_GetRxPin(portNum);

    if (!LPC17_Gpio_OpenPin(txPin) || !LPC17_Gpio_OpenPin(rxPin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

void LPC17_Uart_SetClock(int32_t portNum, int32_t pclkSel) {
    pclkSel &= 0x03;
    LPC17xx_SYSCON &SYSCON = *(LPC17xx_SYSCON *)(size_t)(LPC17xx_SYSCON::c_SYSCON_Base);

    switch (portNum) {
    case 0:
        SYSCON.PCLKSEL0 &= ~(0x03 << 6);
        SYSCON.PCLKSEL0 |= (pclkSel << 6);
        break;

    case 1:
        SYSCON.PCLKSEL0 &= ~(0x03 << 8);
        SYSCON.PCLKSEL0 |= (pclkSel << 8);
        break;

    case 2:
        SYSCON.PCLKSEL1 &= ~(0x03 << 16);
        SYSCON.PCLKSEL1 |= (pclkSel << 16);
        break;

    case 3:
        SYSCON.PCLKSEL1 &= ~(0x03 << 18);
        SYSCON.PCLKSEL1 |= (pclkSel << 18);
        break;
    }
}
TinyCLR_Result LPC17_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t portNum = self->Index;

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    uint32_t     divisor;

    uint32_t fdr;

    switch (baudRate) {

    case 9600: LPC17_Uart_SetClock(portNum, 0); fdr = 0x54; divisor = 217; break;

    case 14400: LPC17_Uart_SetClock(portNum, 0); fdr = 81; divisor = 217; break;

    case 19200: LPC17_Uart_SetClock(portNum, 0); fdr = 177; divisor = 179; break;

    case 38400: LPC17_Uart_SetClock(portNum, 0); fdr = 131; divisor = 71; break;

    case 57600: LPC17_Uart_SetClock(portNum, 0); fdr = 213; divisor = 47; break;

    case 115200: LPC17_Uart_SetClock(portNum, 0); fdr = 117; divisor = 19; break;

    case 230400: LPC17_Uart_SetClock(portNum, 0); fdr = 229; divisor = 12; break;

    case 460800: LPC17_Uart_SetClock(portNum, 1); fdr = 229; divisor = 6; break;

    case 921600: LPC17_Uart_SetClock(portNum, 1); fdr = 229; divisor = 3; break;

    default:
        LPC17_Uart_SetClock(portNum, 1);
        divisor = ((LPC17xx_USART::c_ClockRate / (baudRate * 16)));
        fdr = 0x10;
        break;

    }

    // CWS: Disable interrupts
    USARTC.UART_LCR = 0; // prepare to Init UART
    USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_INTR_ALL_SET);          // Disable all UART interrupts
    /* CWS: Set baud rate to baudRate bps */
    USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_DLAB;                              // prepare to access Divisor
    USARTC.SEL1.DLL.UART_DLL = divisor & 0xFF;      //GET_LSB(divisor);                                                      // Set baudrate.
    USARTC.SEL2.DLM.UART_DLM = (divisor >> 8) & 0xFF; // GET_MSB(divisor);
    USARTC.UART_LCR &= ~LPC17xx_USART::UART_LCR_DLAB;                                              // prepare to access RBR, THR, IER
    // CWS: Set port for 8 bit, 1 stop, no parity
    USARTC.UART_FDR = fdr;

    // DataBit range 5-8
    if (5 <= dataBits && dataBits <= 8) {
        SET_BITS(USARTC.UART_LCR, LPC17xx_USART::UART_LCR_WLS_shift, LPC17xx_USART::UART_LCR_WLS_mask, dataBits - 5);
    }
    else {   // not supported
     // set up 8 data bits incase return value is ignored
        SET_BITS(USARTC.UART_LCR, LPC17xx_USART::UART_LCR_WLS_shift, LPC17xx_USART::UART_LCR_WLS_mask, LPC17xx_USART::UART_LCR_WLS_8_BITS);
        return TinyCLR_Result::NotSupported;
    }

    switch (stopBits) {
    case TinyCLR_Uart_StopBitCount::Two:
        USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_NSB_15_STOPBITS;

        if (dataBits == 5)
            return TinyCLR_Result::NotSupported;
        break;

    case TinyCLR_Uart_StopBitCount::One:
        USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_NSB_1_STOPBITS;
        break;

    case TinyCLR_Uart_StopBitCount::OnePointFive:
        USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_NSB_15_STOPBITS;

        if (dataBits != 5)
            return TinyCLR_Result::NotSupported;
        break;

    default:
        return TinyCLR_Result::NotSupported;
    }

    switch (parity) {

    case TinyCLR_Uart_Parity::Space:
        USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_SPE;

    case TinyCLR_Uart_Parity::Even:
        USARTC.UART_LCR |= (LPC17xx_USART::UART_LCR_EPE | LPC17xx_USART::UART_LCR_PBE);

        break;

    case TinyCLR_Uart_Parity::Mark:
        USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_SPE;

    case  TinyCLR_Uart_Parity::Odd:
        USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_PBE;

        break;

    case TinyCLR_Uart_Parity::None:

        USARTC.UART_LCR &= ~LPC17xx_USART::UART_LCR_PBE;

        break;

    default:

        return TinyCLR_Result::NotSupported;
    }

    if (handshaking != TinyCLR_Uart_Handshake::None && portNum != 1) // Only port 2 support handshaking
        return TinyCLR_Result::NotSupported;


    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        USARTC.UART_MCR |= (1 << 6) | (1 << 7);  // Enable CTS - RTS
        USARTC.SEL2.IER.UART_IER |= (1 << 7) | (1 << 3);    // Enable Interrupt CTS
        g_UartController[portNum].handshakeEnable = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    // CWS: Set the RX FIFO trigger level (to 8 bytes), reset RX, TX FIFO
    USARTC.SEL3.FCR.UART_FCR = (LPC17xx_USART::UART_FCR_RFITL_01 >> LPC17xx_USART::UART_FCR_RFITL_shift) |
        LPC17xx_USART::UART_FCR_TFR |
        LPC17xx_USART::UART_FCR_RFR |
        LPC17xx_USART::UART_FCR_FME;

    switch (portNum) {
    case 0:
        LPC17_Interrupt_Activate(UART0_IRQn, (uint32_t*)&UART0_IntHandler, 0);

        break;

    case 1:
        LPC17_Interrupt_Activate(UART1_IRQn, (uint32_t*)&UART1_IntHandler, 0);

        break;

    case 2:
        LPC17_Interrupt_Activate(UART2_IRQn, (uint32_t*)&UART2_IntHandler, 0);

        break;

    case 3:
        LPC17_Interrupt_Activate(UART3_IRQn, (uint32_t*)&UART3_IntHandler, 0);

        break;

    case 4:
        LPC17_Interrupt_Activate(UART4_IRQn, (uint32_t*)&UART4_IntHandler, 0);

        break;

    default:
        return TinyCLR_Result::ArgumentOutOfRange;
    }

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    if (g_UartController[portNum].txBufferSize == 0) {
        g_UartController[portNum].txBufferSize = g_LPC17_Uart_TxDefaultBuffersSize[portNum];

        g_UartController[self->Index].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[portNum].txBufferSize);

        if (g_UartController[self->Index].TxBuffer == nullptr) {
            g_UartController[self->Index].txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (g_UartController[portNum].rxBufferSize == 0) {
        g_UartController[portNum].rxBufferSize = g_LPC17_Uart_RxDefaultBuffersSize[portNum];

        g_UartController[self->Index].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[portNum].rxBufferSize);

        if (g_UartController[self->Index].RxBuffer == nullptr) {
            g_UartController[self->Index].rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    USARTC.UART_TER = LPC17xx_USART::UART_TER_TXEN;

    LPC17_Uart_PinConfiguration(portNum, true);

    g_UartController[portNum].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Release(const TinyCLR_Uart_Provider* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t portNum = self->Index;

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    if (g_UartController[portNum].isOpened == true) {

        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_INTR_ALL_SET);         // Disable all UART interrupt

        // CWS: Disable interrupts
        USARTC.SEL3.FCR.UART_FCR = 0;
        USARTC.UART_LCR = 0; // prepare to Init UART

        if (g_UartController[portNum].handshakeEnable) {
            USARTC.UART_MCR &= ~((1 << 6) | (1 << 7));
            USARTC.SEL2.IER.UART_IER &= ~((1 << 7) | (1 << 3));
        }

        LPC17_Uart_PinConfiguration(portNum, false);

        // Disable to save power
        switch (portNum) {

        case 0: LPC_SC->PCONP &= ~PCONP_PCUART0; break;

        case 1: LPC_SC->PCONP &= ~PCONP_PCUART1; break;

        case 2: LPC_SC->PCONP &= ~PCONP_PCUART2; break;

        case 3: LPC_SC->PCONP &= ~PCONP_PCUART3; break;

        case 4: LPC_SC->PCONP &= ~PCONP_PCUART4; break;
        }


    }

    g_UartController[portNum].txBufferCount = 0;
    g_UartController[portNum].txBufferIn = 0;
    g_UartController[portNum].txBufferOut = 0;

    g_UartController[portNum].rxBufferCount = 0;
    g_UartController[portNum].rxBufferIn = 0;
    g_UartController[portNum].rxBufferOut = 0;
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

    g_UartController[portNum].isOpened = false;
    g_UartController[portNum].handshakeEnable = false;

    return TinyCLR_Result::Success;
}

void LPC17_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    if (enable) {
        USARTC.UART_LCR &= (~LPC17xx_USART::UART_LCR_DLAB);
        USARTC.SEL2.IER.UART_IER |= (LPC17xx_USART::UART_IER_THREIE);

        LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

        volatile uint32_t LSR_Value = USARTC.UART_LSR;           // Store LSR value since it's Read-to-Clear
        volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC17xx_USART::UART_IIR_IID_mask;

        LPC17_Uart_TransmitData(portNum, LSR_Value, IIR_Value);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_THREIE);
    }
}

void LPC17_Uart_RxBufferFullInterruptEnable(int portNum, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(portNum);

    if (enable)
        USARTC.SEL2.IER.UART_IER |= (LPC17xx_USART::UART_IER_RDAIE);
    else
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_RDAIE);
}

bool LPC17_Uart_TxHandshakeEnabledState(int portNum) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result LPC17_Uart_Flush(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (g_UartController[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    LPC17_Uart_TxBufferEmptyInterruptEnable(portNum, true);

    while (g_UartController[portNum].txBufferCount > 0) {
        LPC17_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length) {
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

TinyCLR_Result LPC17_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened == false || g_UartController[self->Index].txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (g_UartController[portNum].txBufferCount == g_UartController[portNum].txBufferSize) {
        UART_SetErrorEvent(portNum, TinyCLR_Uart_Error::TransmitFull);

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
        LPC17_Uart_TxBufferEmptyInterruptEnable(portNum, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result LPC17_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

void LPC17_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        LPC17_Uart_Release(uartProviders[i]);
    }
}

