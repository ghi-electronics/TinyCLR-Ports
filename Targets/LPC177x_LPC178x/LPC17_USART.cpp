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

static TinyCLR_Uart_Provider uartProviders;
static TinyCLR_Api_Info uartApi;

uint32_t setFieldValue(volatile uint32_t oldVal, uint32_t shift, uint32_t mask, uint32_t val) {
    volatile uint32_t temp = oldVal;

    temp &= ~mask;
    temp |= val << shift;
    return temp;
}

const TinyCLR_Api_Info* LPC17_Uart_GetApi() {
    uartProviders.Parent = &uartApi;
    uartProviders.Acquire = &LPC17_Uart_Acquire;
    uartProviders.Release = &LPC17_Uart_Release;
    uartProviders.SetActiveSettings = &LPC17_Uart_SetActiveSettings;
    uartProviders.Flush = &LPC17_Uart_Flush;
    uartProviders.Read = &LPC17_Uart_Read;
    uartProviders.Write = &LPC17_Uart_Write;
    uartProviders.SetPinChangedHandler = &LPC17_Uart_SetPinChangedHandler;
    uartProviders.SetErrorReceivedHandler = &LPC17_Uart_SetErrorReceivedHandler;
    uartProviders.SetDataReceivedHandler = &LPC17_Uart_SetDataReceivedHandler;
    uartProviders.GetBreakSignalState = &LPC17_Uart_GetBreakSignalState;
    uartProviders.SetBreakSignalState = &LPC17_Uart_SetBreakSignalState;
    uartProviders.GetCarrierDetectState = &LPC17_Uart_GetCarrierDetectState;
    uartProviders.GetClearToSendState = &LPC17_Uart_GetClearToSendState;
    uartProviders.GetDataReadyState = &LPC17_Uart_GetDataReadyState;
    uartProviders.GetIsDataTerminalReadyEnabled = &LPC17_Uart_GetIsDataTerminalReadyEnabled;
    uartProviders.SetIsDataTerminalReadyEnabled = &LPC17_Uart_SetIsDataTerminalReadyEnabled;
    uartProviders.GetIsRequestToSendEnabled = &LPC17_Uart_GetIsRequestToSendEnabled;
    uartProviders.SetIsRequestToSendEnabled = &LPC17_Uart_SetIsRequestToSendEnabled;
    uartProviders.GetReadBufferSize = &LPC17_Uart_GetReadBufferSize;
    uartProviders.SetReadBufferSize = &LPC17_Uart_SetReadBufferSize;
    uartProviders.GetWriteBufferSize = &LPC17_Uart_GetWriteBufferSize;
    uartProviders.SetWriteBufferSize = &LPC17_Uart_SetWriteBufferSize;
    uartProviders.GetUnreadCount = &LPC17_Uart_GetUnreadCount;
    uartProviders.GetUnwrittenCount = &LPC17_Uart_GetUnwrittenCount;
    uartProviders.ClearReadBuffer = &LPC17_Uart_ClearReadBuffer;
    uartProviders.ClearWriteBuffer = &LPC17_Uart_ClearWriteBuffer;

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Count = TOTAL_UART_CONTROLLERS;
    uartApi.Implementation = &uartProviders;

    return &uartApi;
}

TinyCLR_Result LPC17_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& size) {
    size = g_UartController[controller].rxBufferSize == 0 ? g_LPC17_Uart_RxDefaultBuffersSize[controller] : g_UartController[controller].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t size) {
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

TinyCLR_Result LPC17_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& size) {
    size = g_UartController[controller].txBufferSize == 0 ? g_LPC17_Uart_TxDefaultBuffersSize[controller] : g_UartController[controller].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, int32_t controller, size_t size) {
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


void LPC17_Uart_PinConfiguration(int controller, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t txPin = LPC17_Uart_GetTxPin(controller);
    int32_t rxPin = LPC17_Uart_GetRxPin(controller);
    int32_t ctsPin = LPC17_Uart_GetCtsPin(controller);
    int32_t rtsPin = LPC17_Uart_GetRtsPin(controller);

    LPC17_Gpio_PinFunction txPinMode = LPC17_Uart_GetTxAlternateFunction(controller);
    LPC17_Gpio_PinFunction rxPinMode = LPC17_Uart_GetRxAlternateFunction(controller);
    LPC17_Gpio_PinFunction ctsPinMode = LPC17_Uart_GetCtsAlternateFunction(controller);
    LPC17_Gpio_PinFunction rtsPinMode = LPC17_Uart_GetRtsAlternateFunction(controller);

    if (enable) {
        // Connect pin to UART
        LPC17_Gpio_ConfigurePin(txPin, LPC17_Gpio_Direction::Input, txPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        // Connect pin to UART
        LPC17_Gpio_ConfigurePin(rxPin, LPC17_Gpio_Direction::Input, rxPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

        LPC17_Uart_TxBufferEmptyInterruptEnable(controller, true);

        LPC17_Uart_RxBufferFullInterruptEnable(controller, true);

        if (g_UartController[controller].handshakeEnable) {
            if (!LPC17_Gpio_OpenPin(ctsPin) || !LPC17_Gpio_OpenPin(rtsPin))
                return;

            LPC17_Gpio_ConfigurePin(ctsPin, LPC17_Gpio_Direction::Input, ctsPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
            LPC17_Gpio_ConfigurePin(rtsPin, LPC17_Gpio_Direction::Input, rtsPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

        }

    }
    else {

        LPC17_Uart_TxBufferEmptyInterruptEnable(controller, false);
        // TODO Add config for uart pin protected state
        LPC17_Gpio_ClosePin(txPin);

        LPC17_Uart_RxBufferFullInterruptEnable(controller, false);
        // TODO Add config for uart pin protected state
        LPC17_Gpio_ClosePin(rxPin);

        if (g_UartController[controller].handshakeEnable) {
            LPC17_Gpio_ClosePin(ctsPin);
            LPC17_Gpio_ClosePin(rtsPin);
        }
    }
}

void UART_SetErrorEvent(int32_t controller, TinyCLR_Uart_Error error) {
    if (g_UartController[controller].errorEventHandler != nullptr)
        g_UartController[controller].errorEventHandler(g_UartController[controller].provider, error);
}

void LPC17_Uart_ReceiveData(int controller, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

    // Read data from Rx FIFO
    if (USARTC.SEL2.IER.UART_IER & (LPC17xx_USART::UART_IER_RDAIE)) {
        if ((LSR_Value & LPC17xx_USART::UART_LSR_RFDR) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_RDA) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_TOUT)) {
            do {
                uint8_t rxdata = (uint8_t)USARTC.SEL1.RBR.UART_RBR;

                if (0 == (LSR_Value & (LPC17xx_USART::UART_LSR_PEI | LPC17xx_USART::UART_LSR_OEI | LPC17xx_USART::UART_LSR_FEI))) {
                    if (g_UartController[controller].rxBufferCount == g_UartController[controller].rxBufferSize) {
                        UART_SetErrorEvent(controller, TinyCLR_Uart_Error::ReceiveFull);

                        continue;
                    }

                    g_UartController[controller].RxBuffer[g_UartController[controller].rxBufferIn++] = rxdata;

                    g_UartController[controller].rxBufferCount++;

                    if (g_UartController[controller].rxBufferIn == g_UartController[controller].rxBufferSize)
                        g_UartController[controller].rxBufferIn = 0;

                    if (g_UartController[controller].dataReceivedEventHandler != nullptr)
                        g_UartController[controller].dataReceivedEventHandler(g_UartController[controller].provider, 1);
                }

                LSR_Value = USARTC.UART_LSR;

                if (LSR_Value & 0x04) {
                    UART_SetErrorEvent(controller, TinyCLR_Uart_Error::ReceiveParity);
                }
                else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
                    UART_SetErrorEvent(controller, TinyCLR_Uart_Error::Frame);
                }
                else if (LSR_Value & 0x02) {
                    UART_SetErrorEvent(controller, TinyCLR_Uart_Error::BufferOverrun);
                }
            } while (LSR_Value & LPC17xx_USART::UART_LSR_RFDR);
        }
    }
}
void LPC17_Uart_TransmitData(int controller, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

    // Send data
    if ((LSR_Value & LPC17xx_USART::UART_LSR_TE) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_THRE)) {
        // Check if CTS is high
        if (LPC17_Uart_TxHandshakeEnabledState(controller)) {
            if (g_UartController[controller].txBufferCount > 0) {
                uint8_t txdata = g_UartController[controller].TxBuffer[g_UartController[controller].txBufferOut++];

                g_UartController[controller].txBufferCount--;

                if (g_UartController[controller].txBufferOut == g_UartController[controller].txBufferSize)
                    g_UartController[controller].txBufferOut = 0;

                USARTC.SEL1.THR.UART_THR = txdata; // write TX data

            }
            else {
                LPC17_Uart_TxBufferEmptyInterruptEnable(controller, false); // Disable interrupt when no more data to send.
            }
        }
    }
}

void UART_IntHandler(int controller) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

    volatile uint32_t LSR_Value = USARTC.UART_LSR;           // Store LSR value since it's Read-to-Clear
    volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC17xx_USART::UART_IIR_IID_mask;

    if (g_UartController[controller].handshakeEnable) {
        volatile bool dump = USARTC.UART_MSR; // Clr status register
    }

    if (LSR_Value & 0x04) {
        UART_SetErrorEvent(controller, TinyCLR_Uart_Error::ReceiveParity);
    }
    else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
        UART_SetErrorEvent(controller, TinyCLR_Uart_Error::Frame);
    }
    else if (LSR_Value & 0x02) {
        UART_SetErrorEvent(controller, TinyCLR_Uart_Error::BufferOverrun);
    }

    LPC17_Uart_ReceiveData(controller, LSR_Value, IIR_Value);

    LPC17_Uart_TransmitData(controller, LSR_Value, IIR_Value);
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

TinyCLR_Result LPC17_Uart_Acquire(const TinyCLR_Uart_Provider* self, int32_t controller) {


    if (controller >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t txPin = LPC17_Uart_GetTxPin(controller);
    int32_t rxPin = LPC17_Uart_GetRxPin(controller);

    if (g_UartController[controller].isOpened || !LPC17_Gpio_OpenPin(txPin) || !LPC17_Gpio_OpenPin(rxPin))
        return TinyCLR_Result::SharingViolation;

    g_UartController[controller].txBufferCount = 0;
    g_UartController[controller].txBufferIn = 0;
    g_UartController[controller].txBufferOut = 0;

    g_UartController[controller].rxBufferCount = 0;
    g_UartController[controller].rxBufferIn = 0;
    g_UartController[controller].rxBufferOut = 0;

    g_UartController[controller].provider = self;

    // Enable power config
    switch (controller) {

    case 0: LPC_SC->PCONP |= PCONP_PCUART0; break;

    case 1: LPC_SC->PCONP |= PCONP_PCUART1; break;

    case 2: LPC_SC->PCONP |= PCONP_PCUART2; break;

    case 3: LPC_SC->PCONP |= PCONP_PCUART3; break;

    case 4: LPC_SC->PCONP |= PCONP_PCUART4; break;

    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, int32_t controller, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    DISABLE_INTERRUPTS_SCOPED(irq);



    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

    uint32_t     divisor;

    divisor = ((LPC17xx_USART::c_ClockRate / (baudRate * 16)));

    while (LPC17xx_USART::c_ClockRate / (divisor * 16) > baudRate) {
        divisor++;
    }

    // CWS: Disable interrupts
    USARTC.UART_LCR = 0; // prepare to Init UART
    USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_INTR_ALL_SET);          // Disable all UART interrupts
    /* CWS: Set baud rate to baudRate bps */
    USARTC.UART_LCR |= LPC17xx_USART::UART_LCR_DLAB;                              // prepare to access Divisor
    USARTC.SEL1.DLL.UART_DLL = divisor & 0xFF;      //GET_LSB(divisor);                                                      // Set baudrate.
    USARTC.SEL2.DLM.UART_DLM = (divisor >> 8) & 0xFF; // GET_MSB(divisor);
    USARTC.UART_LCR &= ~LPC17xx_USART::UART_LCR_DLAB; // prepare to access RBR, THR, IER
    // CWS: Set port for 8 bit, 1 stop, no parity
    USARTC.UART_FDR = 0x10; // DIVADDVAL = 0, MULVAL = 1, DLM = 0

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

    if (handshaking != TinyCLR_Uart_Handshake::None && controller != 1) // Only port 2 support handshaking
        return TinyCLR_Result::NotSupported;


    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        USARTC.UART_MCR |= (1 << 6) | (1 << 7);  // Enable CTS - RTS
        USARTC.SEL2.IER.UART_IER |= (1 << 7) | (1 << 3);    // Enable Interrupt CTS
        g_UartController[controller].handshakeEnable = true;
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

    switch (controller) {
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

    if (g_UartController[controller].txBufferSize == 0) {
        g_UartController[controller].txBufferSize = g_LPC17_Uart_TxDefaultBuffersSize[controller];

        g_UartController[controller].TxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[controller].txBufferSize);

        if (g_UartController[controller].TxBuffer == nullptr) {
            g_UartController[controller].txBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    if (g_UartController[controller].rxBufferSize == 0) {
        g_UartController[controller].rxBufferSize = g_LPC17_Uart_RxDefaultBuffersSize[controller];

        g_UartController[controller].RxBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, g_UartController[controller].rxBufferSize);

        if (g_UartController[controller].RxBuffer == nullptr) {
            g_UartController[controller].rxBufferSize = 0;

            return TinyCLR_Result::OutOfMemory;
        }
    }

    USARTC.UART_TER = LPC17xx_USART::UART_TER_TXEN;

    LPC17_Uart_PinConfiguration(controller, true);

    g_UartController[controller].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Release(const TinyCLR_Uart_Provider* self, int32_t controller) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;



    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);



    if (g_UartController[controller].isOpened == true) {
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_INTR_ALL_SET);         // Disable all UART interrupt
                // CWS: Disable interrupts
        USARTC.SEL3.FCR.UART_FCR = 0;
        USARTC.UART_LCR = 0; // prepare to Init UART

        if (g_UartController[controller].handshakeEnable) {
            USARTC.UART_MCR &= ~((1 << 6) | (1 << 7));
            USARTC.SEL2.IER.UART_IER &= ~((1 << 7) | (1 << 3));
        }

        g_UartController[controller].txBufferCount = 0;
        g_UartController[controller].txBufferIn = 0;
        g_UartController[controller].txBufferOut = 0;

        g_UartController[controller].rxBufferCount = 0;
        g_UartController[controller].rxBufferIn = 0;
        g_UartController[controller].rxBufferOut = 0;
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

        LPC17_Uart_PinConfiguration(controller, false);
    }

    // Disable to save power
    switch (controller) {

    case 0: LPC_SC->PCONP &= ~PCONP_PCUART0; break;

    case 1: LPC_SC->PCONP &= ~PCONP_PCUART1; break;

    case 2: LPC_SC->PCONP &= ~PCONP_PCUART2; break;

    case 3: LPC_SC->PCONP &= ~PCONP_PCUART3; break;

    case 4: LPC_SC->PCONP &= ~PCONP_PCUART4; break;
    }

    g_UartController[controller].isOpened = false;
    g_UartController[controller].handshakeEnable = false;

    return TinyCLR_Result::Success;
}

void LPC17_Uart_TxBufferEmptyInterruptEnable(int controller, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

    if (enable) {
        USARTC.UART_LCR &= (~LPC17xx_USART::UART_LCR_DLAB);
        USARTC.SEL2.IER.UART_IER |= (LPC17xx_USART::UART_IER_THREIE);

        LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

        volatile uint32_t LSR_Value = USARTC.UART_LSR;           // Store LSR value since it's Read-to-Clear
        volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC17xx_USART::UART_IIR_IID_mask;

        LPC17_Uart_TransmitData(controller, LSR_Value, IIR_Value);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_THREIE);
    }
}

void LPC17_Uart_RxBufferFullInterruptEnable(int controller, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controller);

    if (enable)
        USARTC.SEL2.IER.UART_IER |= (LPC17xx_USART::UART_IER_RDAIE);
    else
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_RDAIE);
}

bool LPC17_Uart_TxHandshakeEnabledState(int controller) {
    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result LPC17_Uart_Flush(const TinyCLR_Uart_Provider* self, int32_t controller) {


    if (g_UartController[controller].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    // Make sute interrupt is enable
    LPC17_Uart_TxBufferEmptyInterruptEnable(controller, true);

    while (g_UartController[controller].txBufferCount > 0) {
        LPC17_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Read(const TinyCLR_Uart_Provider* self, int32_t controller, uint8_t* buffer, size_t& length) {

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

TinyCLR_Result LPC17_Uart_Write(const TinyCLR_Uart_Provider* self, int32_t controller, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[controller].isOpened == false || g_UartController[controller].txBufferSize == 0) {
        length = 0;

        return TinyCLR_Result::NotAvailable;
    }

    if (g_UartController[controller].txBufferCount == g_UartController[controller].txBufferSize) {
        UART_SetErrorEvent(controller, TinyCLR_Uart_Error::TransmitFull);

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
        LPC17_Uart_TxBufferEmptyInterruptEnable(controller, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, int32_t controller, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result LPC17_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, int32_t controller, TinyCLR_Uart_ErrorReceivedHandler handler) {


    g_UartController[controller].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, int32_t controller, TinyCLR_Uart_DataReceivedHandler handler) {


    g_UartController[controller].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result LPC17_Uart_GetUnreadCount(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& count) {
    count = g_UartController[controller].rxBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_GetUnwrittenCount(const TinyCLR_Uart_Provider* self, int32_t controller, size_t& count) {
    count = g_UartController[controller].txBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_ClearReadBuffer(const TinyCLR_Uart_Provider* self, int32_t controller) {
    g_UartController[controller].rxBufferCount = g_UartController[controller].rxBufferIn = g_UartController[controller].rxBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_ClearWriteBuffer(const TinyCLR_Uart_Provider* self, int32_t controller) {
    g_UartController[controller].txBufferCount = g_UartController[controller].txBufferIn = g_UartController[controller].txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void LPC17_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        LPC17_Uart_Release(&uartProviders, i);

        g_UartController[i].isOpened = false;
    }
}

