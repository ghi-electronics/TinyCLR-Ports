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

#define USART_EVENT_POST_DEBOUNCE_TICKS (10 * 10000) // 10ms between each events

static const uint32_t uartTxDefaultBuffersSize[] = LPC17_UART_DEFAULT_TX_BUFFER_SIZE;
static const uint32_t uartRxDefaultBuffersSize[] = LPC17_UART_DEFAULT_RX_BUFFER_SIZE;

struct UartState {
    int32_t controllerIndex;

    uint8_t                             *txBuffer;
    uint8_t                             *rxBuffer;

    size_t                              txBufferCount;
    size_t                              txBufferIn;
    size_t                              txBufferOut;
    size_t                              txBufferSize;

    size_t                              rxBufferCount;
    size_t                              rxBufferIn;
    size_t                              rxBufferOut;
    size_t                              rxBufferSize;

    bool                                handshaking;
    bool                                enable;

    TinyCLR_Uart_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler    dataReceivedEventHandler;
    TinyCLR_Uart_ClearToSendChangedHandler cleartosendEventHandler;

    const TinyCLR_Uart_Controller*        controller;

    bool tableInitialized;

    uint16_t initializeCount;

    uint32_t errorEvent;
    uint64_t lastEventTime;
    size_t lastEventRxBufferCount;
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

const char* uartApiNames[] = {
#if TOTAL_UART_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.LPC17.UartController\\0",
#if TOTAL_UART_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.LPC17.UartController\\1",
#if TOTAL_UART_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.LPC17.UartController\\2",
#if TOTAL_UART_CONTROLLERS > 3
"GHIElectronics.TinyCLR.NativeApis.LPC17.UartController\\3",
#if TOTAL_UART_CONTROLLERS > 4
"GHIElectronics.TinyCLR.NativeApis.LPC17.UartController\\4",
#endif
#endif
#endif
#endif
#endif
};

void LPC17_Uart_EnsureTableInitialized() {
    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        if (uartStates[i].tableInitialized)
            continue;

        uartControllers[i].ApiInfo = &uartApi[i];
        uartControllers[i].Acquire = &LPC17_Uart_Acquire;
        uartControllers[i].Release = &LPC17_Uart_Release;
        uartControllers[i].Enable = &LPC17_Uart_Enable;
        uartControllers[i].Disable = &LPC17_Uart_Disable;
        uartControllers[i].SetActiveSettings = &LPC17_Uart_SetActiveSettings;
        uartControllers[i].Flush = &LPC17_Uart_Flush;
        uartControllers[i].Read = &LPC17_Uart_Read;
        uartControllers[i].Write = &LPC17_Uart_Write;
        uartControllers[i].SetErrorReceivedHandler = &LPC17_Uart_SetErrorReceivedHandler;
        uartControllers[i].SetDataReceivedHandler = &LPC17_Uart_SetDataReceivedHandler;
        uartControllers[i].GetClearToSendState = &LPC17_Uart_GetClearToSendState;
        uartControllers[i].SetClearToSendChangedHandler = &LPC17_Uart_SetClearToSendChangedHandler;
        uartControllers[i].GetIsRequestToSendEnabled = &LPC17_Uart_GetIsRequestToSendEnabled;
        uartControllers[i].SetIsRequestToSendEnabled = &LPC17_Uart_SetIsRequestToSendEnabled;
        uartControllers[i].GetReadBufferSize = &LPC17_Uart_GetReadBufferSize;
        uartControllers[i].SetReadBufferSize = &LPC17_Uart_SetReadBufferSize;
        uartControllers[i].GetWriteBufferSize = &LPC17_Uart_GetWriteBufferSize;
        uartControllers[i].SetWriteBufferSize = &LPC17_Uart_SetWriteBufferSize;
        uartControllers[i].GetBytesToRead = &LPC17_Uart_GetBytesToRead;
        uartControllers[i].GetBytesToWrite = &LPC17_Uart_GetBytesToWrite;
        uartControllers[i].ClearReadBuffer = &LPC17_Uart_ClearReadBuffer;
        uartControllers[i].ClearWriteBuffer = &LPC17_Uart_ClearWriteBuffer;

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

const TinyCLR_Api_Info* LPC17_Uart_GetRequiredApi() {
    LPC17_Uart_EnsureTableInitialized();

    return &uartApi[UART_DEBUGGER_INDEX];
}

void LPC17_Uart_AddApi(const TinyCLR_Api_Manager* apiManager) {
    LPC17_Uart_EnsureTableInitialized();

    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &uartApi[i]);
    }
}

size_t LPC17_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferSize;
}

TinyCLR_Result LPC17_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
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

size_t LPC17_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferSize;
}

TinyCLR_Result LPC17_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
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

bool LPC17_Uart_CanPostEvent(int8_t controllerIndex) {
    auto state = reinterpret_cast<UartState*>(&uartStates[controllerIndex]);
    bool canPost = (LPC17_Time_GetCurrentProcessorTime() - state->lastEventTime) > USART_EVENT_POST_DEBOUNCE_TICKS;

    state->lastEventTime = LPC17_Time_GetCurrentProcessorTime();

    return canPost;
}

TinyCLR_Result LPC17_Uart_PinConfiguration(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = &uartStates[controllerIndex];

    int32_t txPin = LPC17_Uart_GetTxPin(controllerIndex);
    int32_t rxPin = LPC17_Uart_GetRxPin(controllerIndex);
    int32_t ctsPin = LPC17_Uart_GetCtsPin(controllerIndex);
    int32_t rtsPin = LPC17_Uart_GetRtsPin(controllerIndex);

    LPC17_Gpio_PinFunction txPinMode = LPC17_Uart_GetTxAlternateFunction(controllerIndex);
    LPC17_Gpio_PinFunction rxPinMode = LPC17_Uart_GetRxAlternateFunction(controllerIndex);
    LPC17_Gpio_PinFunction ctsPinMode = LPC17_Uart_GetCtsAlternateFunction(controllerIndex);
    LPC17_Gpio_PinFunction rtsPinMode = LPC17_Uart_GetRtsAlternateFunction(controllerIndex);

    if (enable) {
        // Connect pin to UART
        LPC17_GpioInternal_ConfigurePin(txPin, LPC17_Gpio_Direction::Input, txPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        // Connect pin to UART
        LPC17_GpioInternal_ConfigurePin(rxPin, LPC17_Gpio_Direction::Input, rxPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

        LPC17_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);

        LPC17_Uart_RxBufferFullInterruptEnable(controllerIndex, true);

        if (state->handshaking) {
            if (ctsPin == PIN_NONE || rtsPin == PIN_NONE)
                return TinyCLR_Result::NotSupported;

            if (!LPC17_GpioInternal_OpenPin(ctsPin) || !LPC17_GpioInternal_OpenPin(rtsPin)) {
                LPC17_Uart_PinConfiguration(controllerIndex, false); // Release all pins included tx, rx

                return TinyCLR_Result::SharingViolation;
            }

            LPC17_GpioInternal_ConfigurePin(ctsPin, LPC17_Gpio_Direction::Input, ctsPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
            LPC17_GpioInternal_ConfigurePin(rtsPin, LPC17_Gpio_Direction::Input, rtsPinMode, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        }
    }
    else {

        LPC17_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false);
        // TODO Add config for uart pin protected state
        LPC17_GpioInternal_ClosePin(txPin);

        LPC17_Uart_RxBufferFullInterruptEnable(controllerIndex, false);
        // TODO Add config for uart pin protected state
        LPC17_GpioInternal_ClosePin(rxPin);

        if (state->handshaking) {
            LPC17_GpioInternal_ClosePin(ctsPin);
            LPC17_GpioInternal_ClosePin(rtsPin);
        }
    }

    return TinyCLR_Result::Success;
}

void LPC17_Uart_SetErrorEvent(int32_t controllerIndex, TinyCLR_Uart_Error error) {
    auto state = &uartStates[controllerIndex];

    if (state->errorEventHandler != nullptr)
        state->errorEventHandler(state->controller, error, LPC17_Time_GetSystemTime());
}

void LPC17_Uart_ReceiveData(int controllerIndex, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

    auto state = &uartStates[controllerIndex];
    bool error = (LSR_Value & (LPC17xx_USART::UART_LSR_PEI | LPC17xx_USART::UART_LSR_OEI | LPC17xx_USART::UART_LSR_FEI));

    // Read data from Rx FIFO
    if ((USARTC.SEL2.IER.UART_IER & (LPC17xx_USART::UART_IER_RDAIE)) || error) {
        if ((LSR_Value & LPC17xx_USART::UART_LSR_RFDR) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_RDA) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_TOUT) || (error)) {
            do {
                uint8_t rxdata = (uint8_t)USARTC.SEL1.RBR.UART_RBR;

                auto canPostEvent = LPC17_Uart_CanPostEvent(controllerIndex);

                if (!error) {
                    if (state->rxBufferCount == state->rxBufferSize) {
                        if (canPostEvent) LPC17_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferFull);

                        goto clear_status;
                    }

                    state->rxBuffer[state->rxBufferIn++] = rxdata;

                    state->rxBufferCount++;

                    if (state->rxBufferIn == state->rxBufferSize)
                        state->rxBufferIn = 0;

                    if (state->dataReceivedEventHandler != nullptr)
                        if (canPostEvent) {
                            if (state->rxBufferCount > state->lastEventRxBufferCount) {
                                // if driver hold event long enough that more than 1 byte
                                state->dataReceivedEventHandler(state->controller, state->rxBufferCount - state->lastEventRxBufferCount, LPC17_Time_GetSystemTime());
                            }
                            else {
                                // if user use poll to read data and rxBufferCount <= lastEventRxBufferCount, driver send at least 1 byte comming
                                state->dataReceivedEventHandler(state->controller, 1, LPC17_Time_GetSystemTime());
                            }

                            state->lastEventRxBufferCount = state->rxBufferCount;
                        }
                }

            clear_status:
                LSR_Value = USARTC.UART_LSR;

                if (LSR_Value & 0x04) {
                    if (canPostEvent) LPC17_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::ReceiveParity);
                }
                else if ((LSR_Value & 0x08) || (LSR_Value & 0x80)) {
                    if (canPostEvent) LPC17_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Frame);
                }
                else if (LSR_Value & 0x02) {
                    if (canPostEvent) LPC17_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::Overrun);
                }

                error = (LSR_Value & (LPC17xx_USART::UART_LSR_PEI | LPC17xx_USART::UART_LSR_OEI | LPC17xx_USART::UART_LSR_FEI));

            } while ((LSR_Value & LPC17xx_USART::UART_LSR_RFDR) || error);
        }
    }
}
void LPC17_Uart_TransmitData(int controllerIndex, uint32_t LSR_Value, uint32_t IIR_Value) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

    auto state = &uartStates[controllerIndex];

    // Send data
    if ((LSR_Value & LPC17xx_USART::UART_LSR_TE) || (IIR_Value == LPC17xx_USART::UART_IIR_IID_Irpt_THRE)) {
        // Check if CTS is high
        if (LPC17_Uart_CanSend(controllerIndex)) {
            if (state->txBufferCount > 0) {
                uint8_t txdata = state->txBuffer[state->txBufferOut++];

                state->txBufferCount--;

                if (state->txBufferOut == state->txBufferSize)
                    state->txBufferOut = 0;

                USARTC.SEL1.THR.UART_THR = txdata; // write TX data

            }
            else {
                LPC17_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false); // Disable interrupt when no more data to send.
            }
        }
        else {
            // Temporary disable tx during cts is high to avoild device lockup
            LPC17_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false);
        }
    }
}

void LPC17_UART_IntHandler(int controllerIndex) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

    volatile uint32_t LSR_Value = USARTC.UART_LSR;           // Store LSR value since it's Read-to-Clear
    volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC17xx_USART::UART_IIR_IID_mask;

    auto state = &uartStates[controllerIndex];

    LPC17_Uart_ReceiveData(controllerIndex, LSR_Value, IIR_Value);

    LPC17_Uart_TransmitData(controllerIndex, LSR_Value, IIR_Value);

    if (state->handshaking) {
        volatile uint32_t msr = USARTC.UART_MSR; // clear cts interrupt

        if (msr & 0x1) {  // detect cts changed bit
            bool ctsState = true;

            LPC17_Uart_GetClearToSendState(state->controller, ctsState);

            if (state->cleartosendEventHandler != nullptr)
                state->cleartosendEventHandler(state->controller, ctsState, LPC17_Time_GetSystemTime());

            // If tx was disable to avoid locked up
            // Need Enable back if detected OK to send
            if (ctsState)
                LPC17_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);
        }
    }
}
//--//
void LPC17_UART0_IntHandler(void *param) {
    LPC17_UART_IntHandler(0);
}

void LPC17_UART1_IntHandler(void *param) {
    LPC17_UART_IntHandler(1);
}

void LPC17_UART2_IntHandler(void *param) {
    LPC17_UART_IntHandler(2);
}

void LPC17_UART3_IntHandler(void *param) {
    LPC17_UART_IntHandler(3);
}
void LPC17_UART4_IntHandler(void *param) {
    LPC17_UART_IntHandler(4);
}

TinyCLR_Result LPC17_Uart_Acquire(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        if (controllerIndex >= TOTAL_UART_CONTROLLERS)
            return TinyCLR_Result::ArgumentInvalid;

        DISABLE_INTERRUPTS_SCOPED(irq);

        int32_t txPin = LPC17_Uart_GetTxPin(controllerIndex);
        int32_t rxPin = LPC17_Uart_GetRxPin(controllerIndex);

        if (!LPC17_GpioInternal_OpenPin(txPin))
            return TinyCLR_Result::SharingViolation;

        if (!LPC17_GpioInternal_OpenPin(rxPin)) {
            LPC17_GpioInternal_ClosePin(txPin); // Close tx if openning rx failed

            return TinyCLR_Result::SharingViolation;
        }

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
        state->lastEventTime = LPC17_Time_GetCurrentProcessorTime();

        state->txBuffer = nullptr;
        state->rxBuffer = nullptr;

        if (LPC17_Uart_SetWriteBufferSize(self, uartTxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        if (LPC17_Uart_SetReadBufferSize(self, uartRxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        // Enable power config
        switch (controllerIndex) {

        case 0: LPC_SC->PCONP |= PCONP_PCUART0; break;

        case 1: LPC_SC->PCONP |= PCONP_PCUART1; break;

        case 2: LPC_SC->PCONP |= PCONP_PCUART2; break;

        case 3: LPC_SC->PCONP |= PCONP_PCUART3; break;

        case 4: LPC_SC->PCONP |= PCONP_PCUART4; break;

        }
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, const TinyCLR_Uart_Settings* settings) {
    uint32_t baudRate = settings->BaudRate;
    uint32_t dataBits = settings->DataBits;
    TinyCLR_Uart_Parity parity = settings->Parity;
    TinyCLR_Uart_StopBitCount stopBits = settings->StopBits;
    TinyCLR_Uart_Handshake handshaking = settings->Handshaking;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

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

    if (handshaking != TinyCLR_Uart_Handshake::None && controllerIndex != 1) // Only port 2 support handshaking
        return TinyCLR_Result::NotSupported;


    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        USARTC.UART_MCR |= (1 << 6) | (1 << 7);  // Enable CTS - RTS
        USARTC.SEL2.IER.UART_IER |= (1 << 7) | (1 << 3);    // Enable Interrupt CTS
        state->handshaking = true;
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

    switch (controllerIndex) {
    case 0:
        LPC17_InterruptInternal_Activate(UART0_IRQn, (uint32_t*)&LPC17_UART0_IntHandler, 0);

        break;

    case 1:
        LPC17_InterruptInternal_Activate(UART1_IRQn, (uint32_t*)&LPC17_UART1_IntHandler, 0);

        break;

    case 2:
        LPC17_InterruptInternal_Activate(UART2_IRQn, (uint32_t*)&LPC17_UART2_IntHandler, 0);

        break;

    case 3:
        LPC17_InterruptInternal_Activate(UART3_IRQn, (uint32_t*)&LPC17_UART3_IntHandler, 0);

        break;

    case 4:
        LPC17_InterruptInternal_Activate(UART4_IRQn, (uint32_t*)&LPC17_UART4_IntHandler, 0);

        break;

    default:
        return TinyCLR_Result::ArgumentOutOfRange;
    }

    USARTC.UART_TER = LPC17xx_USART::UART_TER_TXEN;

    LPC17_Uart_PinConfiguration(controllerIndex, true);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Release(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {

        auto controllerIndex = state->controllerIndex;

        LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_INTR_ALL_SET);         // Disable all UART interrupt
                // CWS: Disable interrupts
        USARTC.SEL3.FCR.UART_FCR = 0;
        USARTC.UART_LCR = 0; // prepare to Init UART

        if (state->handshaking) {
            USARTC.UART_MCR &= ~((1 << 6) | (1 << 7));
            USARTC.SEL2.IER.UART_IER &= ~((1 << 7) | (1 << 3));
        }

        state->txBufferCount = 0;
        state->txBufferIn = 0;
        state->txBufferOut = 0;

        state->rxBufferCount = 0;
        state->rxBufferIn = 0;
        state->rxBufferOut = 0;
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

        LPC17_Uart_PinConfiguration(controllerIndex, false);

        // Disable to save power
        switch (controllerIndex) {

        case 0: LPC_SC->PCONP &= ~PCONP_PCUART0; break;

        case 1: LPC_SC->PCONP &= ~PCONP_PCUART1; break;

        case 2: LPC_SC->PCONP &= ~PCONP_PCUART2; break;

        case 3: LPC_SC->PCONP &= ~PCONP_PCUART3; break;

        case 4: LPC_SC->PCONP &= ~PCONP_PCUART4; break;
        }

        state->handshaking = false;
    }

    return TinyCLR_Result::Success;
}

void LPC17_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

    if (enable) {
        USARTC.UART_LCR &= (~LPC17xx_USART::UART_LCR_DLAB);
        USARTC.SEL2.IER.UART_IER |= (LPC17xx_USART::UART_IER_THREIE);

        LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

        volatile uint32_t LSR_Value = USARTC.UART_LSR;           // Store LSR value since it's Read-to-Clear
        volatile uint32_t IIR_Value = USARTC.SEL3.IIR.UART_IIR & LPC17xx_USART::UART_IIR_IID_mask;

        LPC17_Uart_TransmitData(controllerIndex, LSR_Value, IIR_Value);
    }
    else {
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_THREIE);
    }
}

void LPC17_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    LPC17xx_USART& USARTC = LPC17xx_USART::UART(controllerIndex);

    if (enable)
        USARTC.SEL2.IER.UART_IER |= (LPC17xx_USART::UART_IER_RDAIE);
    else
        USARTC.SEL2.IER.UART_IER &= ~(LPC17xx_USART::UART_IER_RDAIE);
}

bool LPC17_Uart_CanSend(int controllerIndex) {
    auto state = &uartStates[controllerIndex];
    bool value = true;

    LPC17_Uart_GetClearToSendState(state->controller, value);

    return value;
}

TinyCLR_Result LPC17_Uart_Flush(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount && !LPC17_Interrupt_IsDisabled()) {
        LPC17_Uart_TxBufferEmptyInterruptEnable(state->controllerIndex, true);

        while (state->txBufferCount > 0) {
            LPC17_Time_Delay(nullptr, 1);
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        length = 0; // make sure length is updated

        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(self->GetBytesToRead(self), length);

    size_t i = 0;

    while (i < length) {
        buffer[i] = state->rxBuffer[state->rxBufferOut];

        state->rxBufferOut++;
        i++;
        state->rxBufferCount--;

        if (state->rxBufferOut == state->rxBufferSize)
            state->rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->initializeCount == 0) {
        length = 0; // make sure length is updated

        return TinyCLR_Result::NotAvailable;
    }

    if (state->txBufferCount == state->txBufferSize) {
        LPC17_Uart_SetErrorEvent(controllerIndex, TinyCLR_Uart_Error::BufferFull);

        return TinyCLR_Result::Busy;
    }

    length = std::min(state->txBufferSize - state->txBufferCount, length);


    while (i < length) {

        state->txBuffer[state->txBufferIn] = buffer[i];

        state->txBufferCount++;

        i++;

        state->txBufferIn++;

        if (state->txBufferIn == state->txBufferSize)
            state->txBufferIn = 0;
    }

    if (length > 0) {
        LPC17_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = true;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;
        auto ctsPin = LPC17_Uart_GetCtsPin(controllerIndex);

        // Reading the pin state to protect values from register for inteterupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        LPC17_Gpio_Read(nullptr, ctsPin, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? false : true;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->cleartosendEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = false;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;
        auto rtsPin = LPC17_Uart_GetRtsPin(controllerIndex);

        // Reading the pin state to protect values from register for inteterupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        LPC17_Gpio_Read(nullptr, rtsPin, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? true : false;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool value) {
    // Enable by hardware, no support by software.
    return TinyCLR_Result::NotSupported;
}

size_t LPC17_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferCount;
}

size_t LPC17_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferCount;
}

TinyCLR_Result LPC17_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->rxBufferCount = state->rxBufferIn = state->rxBufferOut = state->lastEventRxBufferCount = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->txBufferCount = state->txBufferIn = state->txBufferOut = 0;

    return TinyCLR_Result::Success;
}

void LPC17_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        LPC17_Uart_Release(&uartControllers[i]);

        uartStates[i].tableInitialized = false;
        uartStates[i].initializeCount = 0;
        uartStates[i].txBuffer = nullptr;
        uartStates[i].txBuffer = nullptr;
    }
}

TinyCLR_Result LPC17_Uart_Enable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Uart_Disable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = false;

    return TinyCLR_Result::Success;
}

