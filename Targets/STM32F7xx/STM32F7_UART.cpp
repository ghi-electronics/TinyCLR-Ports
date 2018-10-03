// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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
#include "STM32F7.h"

#define USART_EVENT_POST_DEBOUNCE_TICKS (10 * 10000) // 10ms between each events
// StopBits
#define USART_STOP_BITS_NONE          0
#define USART_STOP_BITS_ONE           1
#define USART_STOP_BITS_TWO           2
#define USART_STOP_BITS_ONEPOINTFIVE  3

//Bit data length
#define STM32F7_UART_DATA_BIT_LENGTH_8    8
#define STM32F7_UART_DATA_BIT_LENGTH_9    9

bool STM32F7_Uart_CanSend(int controllerIndex);
void STM32F7_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable);
void STM32F7_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable);
void STM32F7_Uart_Reset();

typedef  USART_TypeDef* USART_TypeDef_Ptr;

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

    USART_TypeDef_Ptr portReg;

    bool handshaking;
    bool enable;

    TinyCLR_Uart_ErrorReceivedHandler errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler dataReceivedEventHandler;
    TinyCLR_Uart_ClearToSendChangedHandler cleartosendEventHandler;

    const TinyCLR_Uart_Controller* controller;

    bool tableInitialized;

    uint16_t initializeCount;

    uint64_t lastEventTime;
    size_t lastEventRxBufferCount;
};

static const STM32F7_Gpio_Pin uartTxPins[] = STM32F7_UART_TX_PINS;
static const STM32F7_Gpio_Pin uartRxPins[] = STM32F7_UART_RX_PINS;
static const STM32F7_Gpio_Pin uartCtsPins[] = STM32F7_UART_CTS_PINS;
static const STM32F7_Gpio_Pin uartRtsPins[] = STM32F7_UART_RTS_PINS;
static const uint32_t uartRxDefaultBuffersSize[] = STM32F7_UART_DEFAULT_RX_BUFFER_SIZE;
static const uint32_t uartTxDefaultBuffersSize[] = STM32F7_UART_DEFAULT_TX_BUFFER_SIZE;

static UartState uartStates[TOTAL_UART_CONTROLLERS];
static TinyCLR_Uart_Controller uartControllers[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi[TOTAL_UART_CONTROLLERS];

const char* uartApiNames[] = {
#if TOTAL_UART_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\0",
#if TOTAL_UART_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\1",
#if TOTAL_UART_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\2",
#if TOTAL_UART_CONTROLLERS > 3
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\3",
#if TOTAL_UART_CONTROLLERS > 4
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\4",
#if TOTAL_UART_CONTROLLERS > 5
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\5",
#if TOTAL_UART_CONTROLLERS > 6
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\6",
#if TOTAL_UART_CONTROLLERS > 7
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\7",
#if TOTAL_UART_CONTROLLERS > 8
"GHIElectronics.TinyCLR.NativeApis.STM32F7.UartController\\8",
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
};

void STM32F7_Uart_EnsureTableInitialized() {
    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        if (uartStates[i].tableInitialized)
            continue;

        uartControllers[i].ApiInfo = &uartApi[i];
        uartControllers[i].Acquire = &STM32F7_Uart_Acquire;
        uartControllers[i].Release = &STM32F7_Uart_Release;
        uartControllers[i].Enable = &STM32F7_Uart_Enable;
        uartControllers[i].Disable = &STM32F7_Uart_Disable;
        uartControllers[i].SetActiveSettings = &STM32F7_Uart_SetActiveSettings;
        uartControllers[i].Flush = &STM32F7_Uart_Flush;
        uartControllers[i].Read = &STM32F7_Uart_Read;
        uartControllers[i].Write = &STM32F7_Uart_Write;
        uartControllers[i].SetErrorReceivedHandler = &STM32F7_Uart_SetErrorReceivedHandler;
        uartControllers[i].SetDataReceivedHandler = &STM32F7_Uart_SetDataReceivedHandler;
        uartControllers[i].GetClearToSendState = &STM32F7_Uart_GetClearToSendState;
        uartControllers[i].SetClearToSendChangedHandler = &STM32F7_Uart_SetClearToSendChangedHandler;
        uartControllers[i].GetIsRequestToSendEnabled = &STM32F7_Uart_GetIsRequestToSendEnabled;
        uartControllers[i].SetIsRequestToSendEnabled = &STM32F7_Uart_SetIsRequestToSendEnabled;
        uartControllers[i].GetReadBufferSize = &STM32F7_Uart_GetReadBufferSize;
        uartControllers[i].SetReadBufferSize = &STM32F7_Uart_SetReadBufferSize;
        uartControllers[i].GetWriteBufferSize = &STM32F7_Uart_GetWriteBufferSize;
        uartControllers[i].SetWriteBufferSize = &STM32F7_Uart_SetWriteBufferSize;
        uartControllers[i].GetBytesToRead = &STM32F7_Uart_GetBytesToRead;
        uartControllers[i].GetBytesToWrite = &STM32F7_Uart_GetBytesToWrite;
        uartControllers[i].ClearReadBuffer = &STM32F7_Uart_ClearReadBuffer;
        uartControllers[i].ClearWriteBuffer = &STM32F7_Uart_ClearWriteBuffer;

        uartApi[i].Author = "GHI Electronics, LLC";
        uartApi[i].Name = uartApiNames[i];
        uartApi[i].Type = TinyCLR_Api_Type::UartController;
        uartApi[i].Version = 0;
        uartApi[i].Implementation = &uartControllers[i];
        uartApi[i].State = &uartStates[i];

        uartStates[i].controllerIndex = i;
        uartStates[i].tableInitialized = true;
    }

    if (TOTAL_UART_CONTROLLERS > 0) uartStates[0].portReg = USART1;
    if (TOTAL_UART_CONTROLLERS > 1) uartStates[1].portReg = USART2;
    if (TOTAL_UART_CONTROLLERS > 2) uartStates[2].portReg = USART3;
    if (TOTAL_UART_CONTROLLERS > 3) uartStates[3].portReg = UART4;
    if (TOTAL_UART_CONTROLLERS > 4) uartStates[4].portReg = UART5;
    if (TOTAL_UART_CONTROLLERS > 5) uartStates[5].portReg = USART6;
#ifdef UART7
    if (TOTAL_UART_CONTROLLERS > 6) uartStates[6].portReg = UART7;
#ifdef UART8
    if (TOTAL_UART_CONTROLLERS > 7) uartStates[7].portReg = UART8;
#ifdef UART9
    if (TOTAL_UART_CONTROLLERS > 8) uartStates[8].portReg = UART9;
#endif
#endif
#endif

}

const TinyCLR_Api_Info* STM32F7_Uart_GetRequiredApi() {
    STM32F7_Uart_EnsureTableInitialized();

    return &uartApi[0];
}


void STM32F7_Uart_AddApi(const TinyCLR_Api_Manager* apiManager) {
    STM32F7_Uart_EnsureTableInitialized();

    for (int32_t i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &uartApi[i]);
    }
}

size_t STM32F7_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferSize;
}

TinyCLR_Result STM32F7_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
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

size_t STM32F7_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferSize;
}

TinyCLR_Result STM32F7_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size) {
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

bool STM32F7_Uart_CanPostEvent(int8_t controllerIndex) {
    auto state = reinterpret_cast<UartState*>(&uartStates[controllerIndex]);
    bool canPost = (STM32F7_Time_GetCurrentProcessorTime() - state->lastEventTime) > USART_EVENT_POST_DEBOUNCE_TICKS;

    if (canPost) // only update new time if system accepts to post event!
        state->lastEventTime = STM32F7_Time_GetCurrentProcessorTime();

    return canPost;
}

void STM32F7_Uart_InterruptHandler(int8_t controllerIndex) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(&uartStates[controllerIndex]);
    auto sr = (uint16_t)(state->portReg->ISR);
    auto canPostEvent = STM32F7_Uart_CanPostEvent(controllerIndex);
    bool error = (state->rxBufferCount == state->rxBufferSize) || (sr & USART_ISR_ORE) || (sr & USART_ISR_FE) || (sr & USART_ISR_PE);

    if (sr & USART_ISR_RXNE || sr & USART_ISR_ORE || sr & USART_ISR_FE || sr & USART_ISR_PE) {
        uint8_t data = (uint8_t)(state->portReg->RDR); // read RX data

        if (state->errorEventHandler != nullptr && canPostEvent) {
            if (state->rxBufferCount == state->rxBufferSize) {
                state->errorEventHandler(state->controller, TinyCLR_Uart_Error::BufferFull, STM32F7_Time_GetCurrentProcessorTime());
            }

            if (sr & USART_ISR_ORE) {
                state->portReg->ICR |= USART_ISR_ORE;
                state->errorEventHandler(state->controller, TinyCLR_Uart_Error::Overrun, STM32F7_Time_GetCurrentProcessorTime());
            }

            if (sr & USART_ISR_FE) {
                state->portReg->ICR |= USART_ISR_FE;
                state->errorEventHandler(state->controller, TinyCLR_Uart_Error::Frame, STM32F7_Time_GetCurrentProcessorTime());
            }

            if (sr & USART_ISR_PE) {
                state->portReg->ICR |= USART_ISR_PE;
                state->errorEventHandler(state->controller, TinyCLR_Uart_Error::ReceiveParity, STM32F7_Time_GetCurrentProcessorTime());
            }
        }

        if (error)
            return;

        if (sr & USART_ISR_RXNE) {
            state->RxBuffer[state->rxBufferIn++] = data;

            state->rxBufferCount++;

            if (state->rxBufferIn == state->rxBufferSize)
                state->rxBufferIn = 0;

            if (state->dataReceivedEventHandler != nullptr) {
                if (canPostEvent) {
                    if (state->rxBufferCount > state->lastEventRxBufferCount) {
                        // if driver hold event long enough that more than 1 byte
                        state->dataReceivedEventHandler(state->controller, state->rxBufferCount - state->lastEventRxBufferCount, STM32F7_Time_GetCurrentProcessorTime());
                    }
                    else {
                        // if user use poll to read data and rxBufferCount <= lastEventRxBufferCount, driver send at least 1 byte comming
                        state->dataReceivedEventHandler(state->controller, 1, STM32F7_Time_GetCurrentProcessorTime());
                    }

                    state->lastEventRxBufferCount = state->rxBufferCount;
                }
            }
        }
    }

    if (sr & USART_ISR_TXE) {
        if (STM32F7_Uart_CanSend(controllerIndex)) {
            if (state->txBufferCount > 0) {
                uint8_t data = state->TxBuffer[state->txBufferOut++];

                state->txBufferCount--;

                if (state->txBufferOut == state->txBufferSize)
                    state->txBufferOut = 0;

                state->portReg->TDR = data; // write TX data

            }
            else {
                STM32F7_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false); // Disable interrupt when no more data to send.
            }
        }
    }

    if (state->handshaking && (sr & USART_ISR_CTSIF)) {
        auto ctsActive = (sr & USART_ISR_CTS) ? true : false;

        // STM32F7 write 1 to clear CTS interrupt
        state->portReg->ICR |= USART_ISR_CTSIF;

        if (canPostEvent && state->cleartosendEventHandler != nullptr)
            state->cleartosendEventHandler(state->controller, ctsActive, STM32F7_Time_GetCurrentProcessorTime());
    }
}

void STM32F7_Uart_Interrupt0(void* param) {
    STM32F7_Uart_InterruptHandler(0);
}

void STM32F7_Uart_Interrupt1(void* param) {
    STM32F7_Uart_InterruptHandler(1);
}

void STM32F7_Uart_Interrupt2(void* param) {
    STM32F7_Uart_InterruptHandler(2);
}

void STM32F7_Uart_Interrupt3(void* param) {
    STM32F7_Uart_InterruptHandler(3);
}

void STM32F7_Uart_Interrupt4(void* param) {
    STM32F7_Uart_InterruptHandler(4);
}

void STM32F7_Uart_Interrupt5(void* param) {
    STM32F7_Uart_InterruptHandler(5);
}
#ifdef UART7
void STM32F7_Uart_Interrupt6(void* param) {
    STM32F7_Uart_InterruptHandler(6);
}
#ifdef UART8
void STM32F7_Uart_Interrupt7(void* param) {
    STM32F7_Uart_InterruptHandler(7);
}
#ifdef UART9
void STM32F7_Uart_Interrupt8(void* param) {
    STM32F7_Uart_InterruptHandler(8);
}
#endif
#endif
#endif

TinyCLR_Result STM32F7_Uart_Acquire(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        int32_t controllerIndex = state->controllerIndex;

        if (controllerIndex >= TOTAL_UART_CONTROLLERS)
            return TinyCLR_Result::ArgumentInvalid;


        if (!STM32F7_GpioInternal_OpenPin(uartRxPins[controllerIndex].number) || !STM32F7_GpioInternal_OpenPin(uartTxPins[controllerIndex].number))
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
        state->lastEventTime = STM32F7_Time_GetCurrentProcessorTime();

        state->TxBuffer = nullptr;
        state->RxBuffer = nullptr;

        if (STM32F7_Uart_SetWriteBufferSize(self, uartTxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;

        if (STM32F7_Uart_SetReadBufferSize(self, uartRxDefaultBuffersSize[controllerIndex]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, const TinyCLR_Uart_Settings* settings) {
    uint32_t baudRate = settings->BaudRate;
    uint32_t dataBits = settings->DataBits;
    TinyCLR_Uart_Parity parity = settings->Parity;
    TinyCLR_Uart_StopBitCount stopBits = settings->StopBits;
    TinyCLR_Uart_Handshake handshaking = settings->Handshaking;
    uint32_t clk;

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    int32_t controllerIndex = state->controllerIndex;

    // enable UART clock
    if (controllerIndex == 5) { // COM6 on APB2
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
    else if (controllerIndex == 0) { // COM1 on APB2
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
    else if (controllerIndex < 5) { // COM2-5 on APB1
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN >> 1 << controllerIndex;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#ifdef UART7
    else if (controllerIndex == 6) {
        RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#ifdef UART8
    else if (controllerIndex == 7) {
        RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#ifdef UART9
    else if (controllerIndex == 8) {
        RCC->APB2ENR |= RCC_APB2ENR_UART9EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
#endif
#endif
#endif
    //  baudrate
    uint16_t div = (uint16_t)((clk + (baudRate >> 1)) / baudRate); // rounded

    while ((clk / div) > baudRate)
        div++;

    state->portReg->BRR = div;

    // control
    uint16_t ctrl_cr1 = USART_CR1_TE | USART_CR1_RE;

    if (parity != TinyCLR_Uart_Parity::None) {
        ctrl_cr1 |= USART_CR1_PCE;
        dataBits++;
    }

    if (parity == TinyCLR_Uart_Parity::Odd)
        ctrl_cr1 |= USART_CR1_PS;

    if (dataBits == STM32F7_UART_DATA_BIT_LENGTH_9)
        ctrl_cr1 |= USART_CR1_M;
    else {
        if (dataBits != STM32F7_UART_DATA_BIT_LENGTH_8)
            return TinyCLR_Result::ArgumentInvalid;
    }

    state->portReg->CR1 = ctrl_cr1;


    uint32_t stopbit = USART_STOP_BITS_NONE;

    switch (stopBits) {
    case TinyCLR_Uart_StopBitCount::OnePointFive:
        stopbit = USART_STOP_BITS_ONEPOINTFIVE;
        break;

    case TinyCLR_Uart_StopBitCount::Two:
        stopbit = USART_STOP_BITS_TWO;

    case TinyCLR_Uart_StopBitCount::One:
    default:
        break;
    }

    state->portReg->CR2 = (uint16_t)(stopbit << 12);

    uint16_t ctrl_cr3 = 0;

    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        ctrl_cr3 = USART_CR3_CTSE | USART_CR3_RTSE | USART_CR3_CTSIE;

        state->handshaking = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    state->portReg->CR3 = ctrl_cr3;

    auto& tx = uartTxPins[controllerIndex];
    auto& rx = uartRxPins[controllerIndex];
    auto& cts = uartCtsPins[controllerIndex];
    auto& rts = uartRtsPins[controllerIndex];

    STM32F7_GpioInternal_ConfigurePin(rx.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, rx.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(tx.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, tx.alternateFunction);

    if (handshaking == TinyCLR_Uart_Handshake::RequestToSend) {
        if (!STM32F7_GpioInternal_OpenPin(cts.number) || !STM32F7_GpioInternal_OpenPin(rts.number))
            return TinyCLR_Result::SharingViolation;

        STM32F7_GpioInternal_ConfigurePin(cts.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, cts.alternateFunction);
        STM32F7_GpioInternal_ConfigurePin(rts.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, rts.alternateFunction);
    }

    switch (controllerIndex) {
    case 0:
        STM32F7_InterruptInternal_Activate(USART1_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt0, 0);
        break;

    case 1:
        STM32F7_InterruptInternal_Activate(USART2_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt1, 0);
        break;
    case 2:
        STM32F7_InterruptInternal_Activate(USART3_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt2, 0);
        break;

    case 3:
        STM32F7_InterruptInternal_Activate(UART4_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt3, 0);
        break;

    case 4:
        STM32F7_InterruptInternal_Activate(UART5_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt4, 0);
        break;

    case 5:
        STM32F7_InterruptInternal_Activate(USART6_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt5, 0);
        break;

#ifdef UART7
    case 6:
        STM32F7_InterruptInternal_Activate(UART7_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt6, 0);
        break;

#ifdef UART8
    case 7:
        STM32F7_InterruptInternal_Activate(UART8_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt7, 0);
        break;

#ifdef UART9
    case 8:
        STM32F7_InterruptInternal_Activate(UART9_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt8, 0);
        break;
#endif
#endif
#endif
    }

    STM32F7_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true);
    STM32F7_Uart_RxBufferFullInterruptEnable(controllerIndex, true);

    state->portReg->CR1 |= USART_CR1_UE; // start uart

    return ((cts.number == PIN_NONE || rts.number == PIN_NONE) && handshaking == TinyCLR_Uart_Handshake::RequestToSend) ? TinyCLR_Result::NotSupported : TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Release(const TinyCLR_Uart_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        int32_t controllerIndex = state->controllerIndex;

        state->portReg->CR1 = 0; // stop uart

        switch (controllerIndex) {
        case 0:
            STM32F7_InterruptInternal_Deactivate(USART1_IRQn);
            break;

        case 1:
            STM32F7_InterruptInternal_Deactivate(USART2_IRQn);
            break;
        case 2:
            STM32F7_InterruptInternal_Deactivate(USART3_IRQn);
            break;

        case 3:
            STM32F7_InterruptInternal_Deactivate(UART4_IRQn);
            break;

        case 4:
            STM32F7_InterruptInternal_Deactivate(UART5_IRQn);
            break;

        case 5:
            STM32F7_InterruptInternal_Deactivate(USART6_IRQn);
            break;

#ifdef UART7
        case 6:
            STM32F7_InterruptInternal_Deactivate(UART7_IRQn);
            break;

#ifdef UART8
        case 7:
            STM32F7_InterruptInternal_Deactivate(UART8_IRQn);
            break;

#ifdef UART9
        case 8:
            STM32F7_InterruptInternal_Deactivate(UART9_IRQn);
            break;
#endif
#endif
#endif
        }

        STM32F7_Uart_RxBufferFullInterruptEnable(controllerIndex, false);
        STM32F7_Uart_TxBufferEmptyInterruptEnable(controllerIndex, false);

        // disable UART clock
        if (controllerIndex == 5) { // COM6 on APB2
            RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
        }
        else if (controllerIndex == 0) { // COM1 on APB2
            RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
        }
        else if (controllerIndex < 5) { // COM2-5 on APB1
            RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN >> 1 << controllerIndex);
        }
#ifdef UART7
        else if (controllerIndex == 6) {
            RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
        }

#ifdef UART8
        else if (controllerIndex == 7) {
            RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
        }

#ifdef UART9
        else if (controllerIndex == 8) {
            RCC->APB2ENR &= ~RCC_APB2ENR_UART9EN;
        }
#endif
#endif
#endif
        if (apiManager != nullptr) {
            auto memoryProvider = (const TinyCLR_Memory_Manager*)apiManager->FindDefault(apiManager, TinyCLR_Api_Type::MemoryManager);

            memoryProvider->Free(memoryProvider, state->TxBuffer);
            memoryProvider->Free(memoryProvider, state->RxBuffer);
        }

        STM32F7_GpioInternal_ClosePin(uartRxPins[controllerIndex].number);
        STM32F7_GpioInternal_ClosePin(uartTxPins[controllerIndex].number);

        if (state->handshaking) {
            state->portReg->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

            STM32F7_GpioInternal_ClosePin(uartCtsPins[controllerIndex].number);
            STM32F7_GpioInternal_ClosePin(uartRtsPins[controllerIndex].number);
        }
    }

    return TinyCLR_Result::Success;
}

void STM32F7_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartStates[i].initializeCount = 0;

        STM32F7_Uart_Release(&uartControllers[i]);

        uartStates[i].tableInitialized = false;

    }
}

void STM32F7_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable) {
    auto state = &uartStates[controllerIndex];

    if (enable) {
        state->portReg->CR1 |= USART_CR1_TXEIE;  // tx int enable
    }
    else {
        state->portReg->CR1 &= ~USART_CR1_TXEIE; // tx int disable
    }
}

void STM32F7_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable) {
    auto state = &uartStates[controllerIndex];

    if (enable) {
        state->portReg->CR1 |= USART_CR1_RXNEIE;  // rx int enable
    }
    else {
        state->portReg->CR1 &= ~USART_CR1_RXNEIE; // rx int disable
    }
}

bool STM32F7_Uart_CanSend(int controllerIndex) {
    auto state = &uartStates[controllerIndex];
    bool value;

    STM32F7_Uart_GetClearToSendState(state->controller, value);

    return value;
}

TinyCLR_Result STM32F7_Uart_Flush(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount && !STM32F7_Interrupt_IsDisabled()) {
        STM32F7_Uart_TxBufferEmptyInterruptEnable(state->controllerIndex, true);

        while (state->txBufferCount > 0) {
            STM32F7_Time_Delay(nullptr, 1);
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(self->GetBytesToRead(self), length);

    size_t i = 0;

    while (i < length) {
        buffer[i++] = state->RxBuffer[state->rxBufferOut];

        state->rxBufferOut++;

        state->rxBufferCount--;

        if (state->rxBufferOut == state->rxBufferSize)
            state->rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length) {

    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    int32_t controllerIndex = state->controllerIndex;

    if (state->initializeCount == 0) {
        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(state->txBufferSize - state->txBufferCount, length);

    if (state->txBufferCount == state->txBufferSize) {
        if (state->errorEventHandler != nullptr)
            state->errorEventHandler(state->controller, TinyCLR_Uart_Error::BufferFull, STM32F7_Time_GetCurrentProcessorTime());

        return TinyCLR_Result::Success;
    }

    while (i < length) {

        state->TxBuffer[state->txBufferIn] = buffer[i++];

        state->txBufferCount++;

        state->txBufferIn++;

        if (state->txBufferIn == state->txBufferSize)
            state->txBufferIn = 0;
    }

    if (length > 0) {
        STM32F7_Uart_TxBufferEmptyInterruptEnable(controllerIndex, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = true;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;

        // Reading the pin state to protect values from register for inteterupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        STM32F7_Gpio_Read(nullptr, uartCtsPins[controllerIndex].number, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? false : true;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->cleartosendEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& value) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    value = false;

    if (state->handshaking) {
        auto controllerIndex = state->controllerIndex;

        // Reading the pin state to protect values from register for inteterupt which is higher priority (some bits are clear once read)
        TinyCLR_Gpio_PinValue pinState;
        STM32F7_Gpio_Read(nullptr, uartRtsPins[controllerIndex].number, pinState);

        value = (pinState == TinyCLR_Gpio_PinValue::High) ? true : false;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool value) {
    // Enable by hardware, no support by software.
    return TinyCLR_Result::NotSupported;
}

size_t STM32F7_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->rxBufferCount;
}

size_t STM32F7_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    return state->txBufferCount;
}

TinyCLR_Result STM32F7_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->rxBufferCount = state->rxBufferIn = state->rxBufferOut = state->lastEventRxBufferCount = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);

    state->txBufferCount = state->txBufferIn = state->txBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Enable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Disable(const TinyCLR_Uart_Controller* self) {
    auto state = reinterpret_cast<UartState*>(self->ApiInfo->State);
    state->enable = false;

    return TinyCLR_Result::Success;
}