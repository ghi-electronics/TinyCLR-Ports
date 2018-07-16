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

// StopBits
#define USART_STOP_BITS_NONE          0
#define USART_STOP_BITS_ONE           1
#define USART_STOP_BITS_TWO           2
#define USART_STOP_BITS_ONEPOINTFIVE  3

//Bit data length
#define STM32F7_UART_DATA_BIT_LENGTH_8    8
#define STM32F7_UART_DATA_BIT_LENGTH_9    9

bool STM32F7_Uart_TxHandshakeEnabledState(int controller);
void STM32F7_Uart_TxBufferEmptyInterruptEnable(int controller, bool enable);
void STM32F7_Uart_RxBufferFullInterruptEnable(int controller, bool enable);
void STM32F7_Uart_Reset();

typedef  USART_TypeDef* USART_TypeDef_Ptr;

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

    USART_TypeDef_Ptr                   portPtr;

    bool                                isOpened;
    bool                                handshaking;

    TinyCLR_Uart_ErrorReceivedHandler   errorEventHandler;
    TinyCLR_Uart_DataReceivedHandler    dataReceivedEventHandler;

    const TinyCLR_Uart_Controller*        provider;

};

static const STM32F7_Gpio_Pin g_STM32F7_Uart_Tx_Pins[] = STM32F7_UART_TX_PINS;
static const STM32F7_Gpio_Pin g_STM32F7_Uart_Rx_Pins[] = STM32F7_UART_RX_PINS;
static const STM32F7_Gpio_Pin g_STM32F7_Uart_Cts_Pins[] = STM32F7_UART_CTS_PINS;
static const STM32F7_Gpio_Pin g_STM32F7_Uart_Rts_Pins[] = STM32F7_UART_RTS_PINS;
static const uint32_t g_STM32F7_Uart_RxDefaultBuffersSize[] = STM32F7_UART_DEFAULT_RX_BUFFER_SIZE;
static const uint32_t g_STM32F7_Uart_TxDefaultBuffersSize[] = STM32F7_UART_DEFAULT_TX_BUFFER_SIZE;

static const int TOTAL_UART_CONTROLLERS = SIZEOF_ARRAY(g_STM32F7_Uart_Tx_Pins);

static UartController g_UartController[TOTAL_UART_CONTROLLERS];

static USART_TypeDef_Ptr g_STM32F7_Uart_Ports[TOTAL_UART_CONTROLLERS];

static TinyCLR_Uart_Controller uartProviders;
static TinyCLR_Api_Info uartApi;

const TinyCLR_Api_Info* STM32F7_Uart_GetApi() {
    uartProviders.ApiInfo = &uartApi;
    uartProviders.Acquire = &STM32F7_Uart_Acquire;
    uartProviders.Release = &STM32F7_Uart_Release;
    uartProviders.SetActiveSettings = &STM32F7_Uart_SetActiveSettings;
    uartProviders.Flush = &STM32F7_Uart_Flush;
    uartProviders.Read = &STM32F7_Uart_Read;
    uartProviders.Write = &STM32F7_Uart_Write;
    uartProviders.SetPinChangedHandler = &STM32F7_Uart_SetPinChangedHandler;
    uartProviders.SetErrorReceivedHandler = &STM32F7_Uart_SetErrorReceivedHandler;
    uartProviders.SetDataReceivedHandler = &STM32F7_Uart_SetDataReceivedHandler;
    uartProviders.GetBreakSignalState = STM32F7_Uart_GetBreakSignalState;
    uartProviders.SetBreakSignalState = STM32F7_Uart_SetBreakSignalState;
    uartProviders.GetCarrierDetectState = STM32F7_Uart_GetCarrierDetectState;
    uartProviders.GetClearToSendState = STM32F7_Uart_GetClearToSendState;
    uartProviders.GetDataReadyState = STM32F7_Uart_GetDataReadyState;
    uartProviders.GetIsDataTerminalReadyEnabled = STM32F7_Uart_GetIsDataTerminalReadyEnabled;
    uartProviders.SetIsDataTerminalReadyEnabled = STM32F7_Uart_SetIsDataTerminalReadyEnabled;
    uartProviders.GetIsRequestToSendEnabled = STM32F7_Uart_GetIsRequestToSendEnabled;
    uartProviders.SetIsRequestToSendEnabled = STM32F7_Uart_SetIsRequestToSendEnabled;
    uartProviders.GetReadBufferSize = STM32F7_Uart_GetReadBufferSize;
    uartProviders.SetReadBufferSize = STM32F7_Uart_SetReadBufferSize;
    uartProviders.GetWriteBufferSize = STM32F7_Uart_GetWriteBufferSize;
    uartProviders.SetWriteBufferSize = STM32F7_Uart_SetWriteBufferSize;
    uartProviders.GetUnreadCount = &STM32F7_Uart_GetUnreadCount;
    uartProviders.GetUnwrittenCount = &STM32F7_Uart_GetUnwrittenCount;
    uartProviders.ClearReadBuffer = &STM32F7_Uart_ClearReadBuffer;
    uartProviders.ClearWriteBuffer = &STM32F7_Uart_ClearWriteBuffer;
    uartProviders.GetControllerCount = &STM32F7_Uart_GetControllerCount;


    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Implementation = &uartProviders;

    if (TOTAL_UART_CONTROLLERS > 0) g_STM32F7_Uart_Ports[0] = USART1;
    if (TOTAL_UART_CONTROLLERS > 1) g_STM32F7_Uart_Ports[1] = USART2;
#if !defined(STM32F401xE) && !defined(STM32F411xE)
    if (TOTAL_UART_CONTROLLERS > 2) g_STM32F7_Uart_Ports[2] = USART3;
    if (TOTAL_UART_CONTROLLERS > 3) g_STM32F7_Uart_Ports[3] = UART4;
    if (TOTAL_UART_CONTROLLERS > 4) g_STM32F7_Uart_Ports[4] = UART5;
    if (TOTAL_UART_CONTROLLERS > 5) g_STM32F7_Uart_Ports[5] = USART6;
#ifdef UART7
    if (TOTAL_UART_CONTROLLERS > 6) g_STM32F7_Uart_Ports[6] = UART7;
#ifdef UART8
    if (TOTAL_UART_CONTROLLERS > 7) g_STM32F7_Uart_Ports[7] = UART8;
#endif
#endif
#endif
    return &uartApi;
}

TinyCLR_Result STM32F7_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self, int32_t controller, size_t& size) {
    size = g_UartController[controller].rxBufferSize == 0 ? g_STM32F7_Uart_RxDefaultBuffersSize[controller] : g_UartController[controller].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, int32_t controller, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

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

TinyCLR_Result STM32F7_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self, int32_t controller, size_t& size) {
    size = g_UartController[controller].txBufferSize == 0 ? g_STM32F7_Uart_TxDefaultBuffersSize[controller] : g_UartController[controller].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, int32_t controller, size_t size) {
    auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

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

void STM32F7_Uart_IrqRx(int controller, uint16_t sr) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint8_t data = (uint8_t)(g_UartController[controller].portPtr->RDR); // read RX data

    if (g_UartController[controller].rxBufferCount == g_UartController[controller].rxBufferSize) {
        if (g_UartController[controller].errorEventHandler != nullptr)
            g_UartController[controller].errorEventHandler(g_UartController[controller].provider, controller, TinyCLR_Uart_Error::ReceiveFull);

        return;
    }

    g_UartController[controller].RxBuffer[g_UartController[controller].rxBufferIn++] = data;

    g_UartController[controller].rxBufferCount++;

    if (g_UartController[controller].rxBufferIn == g_UartController[controller].rxBufferSize)
        g_UartController[controller].rxBufferIn = 0;

    if (g_UartController[controller].dataReceivedEventHandler != nullptr)
        g_UartController[controller].dataReceivedEventHandler(g_UartController[controller].provider, controller, 1);

    if (g_UartController[controller].errorEventHandler != nullptr) {
        if (sr & USART_ISR_ORE)
            g_UartController[controller].errorEventHandler(g_UartController[controller].provider, controller, TinyCLR_Uart_Error::BufferOverrun);

        if (sr & USART_ISR_FE)
            g_UartController[controller].errorEventHandler(g_UartController[controller].provider, controller, TinyCLR_Uart_Error::Frame);

        if (sr & USART_ISR_PE)
            g_UartController[controller].errorEventHandler(g_UartController[controller].provider, controller, TinyCLR_Uart_Error::ReceiveParity);
    }
}

void STM32F7_Uart_IrqTx(int controller) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (STM32F7_Uart_TxHandshakeEnabledState(controller)) {
        if (g_UartController[controller].txBufferCount > 0) {
            uint8_t data = g_UartController[controller].TxBuffer[g_UartController[controller].txBufferOut++];

            g_UartController[controller].txBufferCount--;

            if (g_UartController[controller].txBufferOut == g_UartController[controller].txBufferSize)
                g_UartController[controller].txBufferOut = 0;

            g_UartController[controller].portPtr->TDR = data; // write TX data

        }
        else {
            STM32F7_Uart_TxBufferEmptyInterruptEnable(controller, false); // Disable interrupt when no more data to send.
        }
    }
}

void STM32F7_Uart_InterruptHandler(int8_t controller, uint16_t sr) {
    if (sr & USART_ISR_RXNE)
        STM32F7_Uart_IrqRx(controller, sr);

    if (sr & USART_ISR_TXE)
        STM32F7_Uart_IrqTx(controller);
}

void STM32F7_Uart_Interrupt0(void* param) {
    uint16_t sr = USART1->ISR;

    STM32F7_Uart_InterruptHandler(0, sr);
}

void STM32F7_Uart_Interrupt1(void* param) {
    uint16_t sr = USART2->ISR;

    STM32F7_Uart_InterruptHandler(1, sr);
}

#if !defined(STM32F401xE) && !defined(STM32F411xE)
void STM32F7_Uart_Interrupt2(void* param) {
    uint16_t sr = USART3->ISR;

    STM32F7_Uart_InterruptHandler(2, sr);
}

void STM32F7_Uart_Interrupt3(void* param) {
    uint16_t sr = UART4->ISR;

    STM32F7_Uart_InterruptHandler(3, sr);
}

void STM32F7_Uart_Interrupt4(void* param) {
    uint16_t sr = UART5->ISR;

    STM32F7_Uart_InterruptHandler(4, sr);
}

void STM32F7_Uart_Interrupt5(void* param) {
    uint16_t sr = USART6->ISR;

    STM32F7_Uart_InterruptHandler(5, sr);
}

void STM32F7_Uart_Interrupt6(void* param) {
    uint16_t sr = UART7->ISR;

    STM32F7_Uart_InterruptHandler(6, sr);
}

void STM32F7_Uart_Interrupt7(void* param) {
    uint16_t sr = UART8->ISR;

    STM32F7_Uart_InterruptHandler(7, sr);
}
#endif

TinyCLR_Result STM32F7_Uart_Acquire(const TinyCLR_Uart_Controller* self, int32_t controller) {
    if (controller >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[controller].isOpened || !STM32F7_GpioInternal_OpenPin(g_STM32F7_Uart_Rx_Pins[controller].number) || !STM32F7_GpioInternal_OpenPin(g_STM32F7_Uart_Tx_Pins[controller].number))
        return TinyCLR_Result::SharingViolation;

    g_UartController[controller].txBufferCount = 0;
    g_UartController[controller].txBufferIn = 0;
    g_UartController[controller].txBufferOut = 0;

    g_UartController[controller].rxBufferCount = 0;
    g_UartController[controller].rxBufferIn = 0;
    g_UartController[controller].rxBufferOut = 0;

    g_UartController[controller].portPtr = g_STM32F7_Uart_Ports[controller];
    g_UartController[controller].provider = self;

    g_UartController[controller].handshaking = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, int32_t controller, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {
    uint32_t clk;

    // enable UART clock
    if (controller == 5) { // COM6 on APB2
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
    else if (controller == 0) { // COM1 on APB2
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
    else if (controller < 5) { // COM2-5 on APB1
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN >> 1 << controller;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#if !defined(STM32F401xE) && !defined(STM32F411xE)
#ifdef UART7
    else if (controller == 6) {
        RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#endif
#ifdef UART8
    else if (controller == 7) {
        RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#endif
#endif
    //  baudrate
    uint16_t div = (uint16_t)((clk + (baudRate >> 1)) / baudRate); // rounded

    while ((clk / div) > baudRate)
        div++;

    g_UartController[controller].portPtr->BRR = div;

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

    g_UartController[controller].portPtr->CR1 = ctrl_cr1;


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

    g_UartController[controller].portPtr->CR2 = (uint16_t)(stopbit << 12);

    uint16_t ctrl_cr3 = 0;

    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        ctrl_cr3 = USART_CR3_CTSE | USART_CR3_RTSE;

        g_UartController[controller].handshaking = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    g_UartController[controller].portPtr->CR3 = ctrl_cr3;

    auto& tx = g_STM32F7_Uart_Tx_Pins[controller];
    auto& rx = g_STM32F7_Uart_Rx_Pins[controller];
    auto& cts = g_STM32F7_Uart_Cts_Pins[controller];
    auto& rts = g_STM32F7_Uart_Rts_Pins[controller];

    STM32F7_GpioInternal_ConfigurePin(rx.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, rx.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(tx.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, tx.alternateFunction);

    if (handshaking == TinyCLR_Uart_Handshake::RequestToSend) {
        if (!STM32F7_GpioInternal_OpenPin(cts.number) || !STM32F7_GpioInternal_OpenPin(rts.number))
            return TinyCLR_Result::SharingViolation;

        STM32F7_GpioInternal_ConfigurePin(cts.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, cts.alternateFunction);
        STM32F7_GpioInternal_ConfigurePin(rts.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, rts.alternateFunction);
    }

    switch (controller) {
    case 0:
        STM32F7_InterruptInternal_Activate(USART1_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt0, 0);
        break;

    case 1:
        STM32F7_InterruptInternal_Activate(USART2_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt1, 0);
        break;
#if !defined(STM32F401xE) && !defined(STM32F411xE)
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
#endif

#ifdef UART8
    case 7:
        STM32F7_InterruptInternal_Activate(UART8_IRQn, (uint32_t*)&STM32F7_Uart_Interrupt7, 0);
        break;
#endif

#endif
    }

    g_UartController[controller].isOpened = true;

    if (g_UartController[controller].txBufferSize == 0) {
        if (STM32F7_Uart_SetWriteBufferSize(self, controller, g_STM32F7_Uart_TxDefaultBuffersSize[controller]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;
    }

    if (g_UartController[controller].rxBufferSize == 0) {
        if (STM32F7_Uart_SetReadBufferSize(self, controller, g_STM32F7_Uart_RxDefaultBuffersSize[controller]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;
    }

    STM32F7_Uart_TxBufferEmptyInterruptEnable(controller, true);
    STM32F7_Uart_RxBufferFullInterruptEnable(controller, true);

    g_UartController[controller].portPtr->CR1 |= USART_CR1_UE; // start uart

    return ((cts.number == PIN_NONE || rts.number == PIN_NONE) && handshaking == TinyCLR_Uart_Handshake::RequestToSend) ? TinyCLR_Result::NotSupported : TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Release(const TinyCLR_Uart_Controller* self, int32_t controller) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    g_UartController[controller].portPtr->CR1 = 0; // stop uart

    switch (controller) {
    case 0:
        STM32F7_InterruptInternal_Deactivate(USART1_IRQn);
        break;

    case 1:
        STM32F7_InterruptInternal_Deactivate(USART2_IRQn);
        break;
#if !defined(STM32F401xE) && !defined(STM32F411xE)
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
#endif

#ifdef UART7
    case 7:
        STM32F7_InterruptInternal_Deactivate(UART8_IRQn);
        break;
#endif

#endif
    }

    STM32F7_Uart_RxBufferFullInterruptEnable(controller, false);
    STM32F7_Uart_TxBufferEmptyInterruptEnable(controller, false);

    // disable UART clock
    if (controller == 5) { // COM6 on APB2
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
    }
    else if (controller == 0) { // COM1 on APB2
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    }
    else if (controller < 5) { // COM2-5 on APB1
        RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN >> 1 << controller);
    }
#if !defined(STM32F401xE) && !defined(STM32F411xE)
#ifdef UART7
    else if (controller == 6) {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
    }
#endif
#ifdef UART8
    else if (controller == 7) {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
    }
#endif
#endif

    if (apiProvider != nullptr) {
        auto memoryProvider = (const TinyCLR_Memory_Manager*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryManager);

        if (g_UartController[controller].txBufferSize != 0) {
            memoryProvider->Free(memoryProvider, g_UartController[controller].TxBuffer);

            g_UartController[controller].txBufferSize = 0;
        }

        if (g_UartController[controller].rxBufferSize != 0) {
            memoryProvider->Free(memoryProvider, g_UartController[controller].RxBuffer);

            g_UartController[controller].rxBufferSize = 0;
        }
    }

    if (g_UartController[controller].isOpened) {
        STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Rx_Pins[controller].number);
        STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Tx_Pins[controller].number);

        if (g_UartController[controller].handshaking) {
            STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Cts_Pins[controller].number);
            STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Rts_Pins[controller].number);
        }
    }

    g_UartController[controller].isOpened = false;

    return TinyCLR_Result::Success;
}

void STM32F7_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        STM32F7_Uart_Release(&uartProviders, i);

        g_UartController[i].isOpened = false;
    }
}

void STM32F7_Uart_TxBufferEmptyInterruptEnable(int controller, bool enable) {
    if (enable) {
        g_UartController[controller].portPtr->CR1 |= USART_CR1_TXEIE;  // tx int enable
    }
    else {
        g_UartController[controller].portPtr->CR1 &= ~USART_CR1_TXEIE; // tx int disable
    }
}

void STM32F7_Uart_RxBufferFullInterruptEnable(int controller, bool enable) {
    if (enable) {
        g_UartController[controller].portPtr->CR1 |= USART_CR1_RXNEIE;  // rx int enable
    }
    else {
        g_UartController[controller].portPtr->CR1 &= ~USART_CR1_RXNEIE; // rx int disable
    }
}

bool STM32F7_Uart_TxHandshakeEnabledState(int controller) {
    // The state of the CTS input only matters if Flow Control is enabled
    if (g_UartController[controller].portPtr->CR3 & USART_CR3_CTSE) {
        TinyCLR_Gpio_PinValue value;

        auto gpioController = 0; //TODO Temporary set to 0

        STM32F7_Gpio_Read(nullptr, gpioController, g_STM32F7_Uart_Cts_Pins[controller].number, value);

        return !(value == TinyCLR_Gpio_PinValue::High);
    }

    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result STM32F7_Uart_Flush(const TinyCLR_Uart_Controller* self, int32_t controller) {
    if (g_UartController[controller].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    while (g_UartController[controller].txBufferCount > 0) {
        STM32F7_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Read(const TinyCLR_Uart_Controller* self, int32_t controller, uint8_t* buffer, size_t& length) {

    size_t i = 0;;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[controller].isOpened == false) {
        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(g_UartController[controller].rxBufferCount, length);

    while (i < length) {
        buffer[i++] = g_UartController[controller].RxBuffer[g_UartController[controller].rxBufferOut];

        g_UartController[controller].rxBufferOut++;

        g_UartController[controller].rxBufferCount--;

        if (g_UartController[controller].rxBufferOut == g_UartController[controller].rxBufferSize)
            g_UartController[controller].rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Write(const TinyCLR_Uart_Controller* self, int32_t controller, const uint8_t* buffer, size_t& length) {
    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[controller].isOpened == false) {
        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(g_UartController[controller].txBufferSize - g_UartController[controller].txBufferCount, length);

    if (g_UartController[controller].txBufferCount == g_UartController[controller].txBufferSize) {
        if (g_UartController[controller].errorEventHandler != nullptr)
            g_UartController[controller].errorEventHandler(g_UartController[controller].provider, controller, TinyCLR_Uart_Error::TransmitFull);

        return TinyCLR_Result::Success;
    }

    while (i < length) {

        g_UartController[controller].TxBuffer[g_UartController[controller].txBufferIn] = buffer[i++];

        g_UartController[controller].txBufferCount++;

        g_UartController[controller].txBufferIn++;

        if (g_UartController[controller].txBufferIn == g_UartController[controller].txBufferSize)
            g_UartController[controller].txBufferIn = 0;
    }

    if (length > 0) {
        STM32F7_Uart_TxBufferEmptyInterruptEnable(controller, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetPinChangedHandler(const TinyCLR_Uart_Controller* self, int32_t controller, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result STM32F7_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, int32_t controller, TinyCLR_Uart_ErrorReceivedHandler handler) {
    g_UartController[controller].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, int32_t controller, TinyCLR_Uart_DataReceivedHandler handler) {
    g_UartController[controller].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetBreakSignalState(const TinyCLR_Uart_Controller* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_SetBreakSignalState(const TinyCLR_Uart_Controller* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetCarrierDetectState(const TinyCLR_Uart_Controller* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetDataReadyState(const TinyCLR_Uart_Controller* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Controller* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Controller* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, int32_t controller, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, int32_t controller, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetUnreadCount(const TinyCLR_Uart_Controller* self, int32_t controller, size_t& count) {
    count = g_UartController[controller].rxBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetUnwrittenCount(const TinyCLR_Uart_Controller* self, int32_t controller, size_t& count) {
    count = g_UartController[controller].txBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self, int32_t controller) {
    g_UartController[controller].rxBufferCount = g_UartController[controller].rxBufferIn = g_UartController[controller].rxBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self, int32_t controller) {
    g_UartController[controller].txBufferCount = g_UartController[controller].txBufferIn = g_UartController[controller].txBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetControllerCount(const TinyCLR_Uart_Controller* self, int32_t& count) {
    count = TOTAL_UART_CONTROLLERS;

    return TinyCLR_Result::Success;
}