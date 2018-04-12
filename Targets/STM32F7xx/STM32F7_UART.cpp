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

bool STM32F7_Uart_TxHandshakeEnabledState(int portNum);
void STM32F7_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable);
void STM32F7_Uart_RxBufferFullInterruptEnable(int portNum, bool enable);
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

    const TinyCLR_Uart_Provider*        provider;

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

static uint8_t uartProviderDefs[TOTAL_UART_CONTROLLERS * sizeof(TinyCLR_Uart_Provider)];
static TinyCLR_Uart_Provider* uartProviders[TOTAL_UART_CONTROLLERS];
static TinyCLR_Api_Info uartApi;

const TinyCLR_Api_Info* STM32F7_Uart_GetApi() {
    for (int i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        uartProviders[i] = (TinyCLR_Uart_Provider*)(uartProviderDefs + (i * sizeof(TinyCLR_Uart_Provider)));
        uartProviders[i]->Parent = &uartApi;
        uartProviders[i]->Index = i;
        uartProviders[i]->Acquire = &STM32F7_Uart_Acquire;
        uartProviders[i]->Release = &STM32F7_Uart_Release;
        uartProviders[i]->SetActiveSettings = &STM32F7_Uart_SetActiveSettings;
        uartProviders[i]->Flush = &STM32F7_Uart_Flush;
        uartProviders[i]->Read = &STM32F7_Uart_Read;
        uartProviders[i]->Write = &STM32F7_Uart_Write;
        uartProviders[i]->SetPinChangedHandler = &STM32F7_Uart_SetPinChangedHandler;
        uartProviders[i]->SetErrorReceivedHandler = &STM32F7_Uart_SetErrorReceivedHandler;
        uartProviders[i]->SetDataReceivedHandler = &STM32F7_Uart_SetDataReceivedHandler;
        uartProviders[i]->GetBreakSignalState = STM32F7_Uart_GetBreakSignalState;
        uartProviders[i]->SetBreakSignalState = STM32F7_Uart_SetBreakSignalState;
        uartProviders[i]->GetCarrierDetectState = STM32F7_Uart_GetCarrierDetectState;
        uartProviders[i]->GetClearToSendState = STM32F7_Uart_GetClearToSendState;
        uartProviders[i]->GetDataReadyState = STM32F7_Uart_GetDataReadyState;
        uartProviders[i]->GetIsDataTerminalReadyEnabled = STM32F7_Uart_GetIsDataTerminalReadyEnabled;
        uartProviders[i]->SetIsDataTerminalReadyEnabled = STM32F7_Uart_SetIsDataTerminalReadyEnabled;
        uartProviders[i]->GetIsRequestToSendEnabled = STM32F7_Uart_GetIsRequestToSendEnabled;
        uartProviders[i]->SetIsRequestToSendEnabled = STM32F7_Uart_SetIsRequestToSendEnabled;
        uartProviders[i]->GetReadBufferSize = STM32F7_Uart_GetReadBufferSize;
        uartProviders[i]->SetReadBufferSize = STM32F7_Uart_SetReadBufferSize;
        uartProviders[i]->GetWriteBufferSize = STM32F7_Uart_GetWriteBufferSize;
        uartProviders[i]->SetWriteBufferSize = STM32F7_Uart_SetWriteBufferSize;
        uartProviders[i]->GetUnreadDataCount = &STM32F7_Uart_GetUnreadDataCount;
        uartProviders[i]->GetUnsendDataCount = &STM32F7_Uart_GetUnsendDataCount;
        uartProviders[i]->ClearReadBuffer = &STM32F7_Uart_ClearReadBuffer;
        uartProviders[i]->ClearSendBuffer = &STM32F7_Uart_ClearSendBuffer;
    }

    uartApi.Author = "GHI Electronics, LLC";
    uartApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.UartProvider";
    uartApi.Type = TinyCLR_Api_Type::UartProvider;
    uartApi.Version = 0;
    uartApi.Count = TOTAL_UART_CONTROLLERS;
    uartApi.Implementation = uartProviders;

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

TinyCLR_Result STM32F7_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].rxBufferSize == 0 ? g_STM32F7_Uart_RxDefaultBuffersSize[self->Index] : g_UartController[self->Index].rxBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
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

TinyCLR_Result STM32F7_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t& size) {
    size = g_UartController[self->Index].txBufferSize == 0 ? g_STM32F7_Uart_TxDefaultBuffersSize[self->Index] : g_UartController[self->Index].txBufferSize;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t size) {
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

void STM32F7_Uart_IrqRx(int portNum, uint16_t sr) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint8_t data = (uint8_t)(g_UartController[portNum].portPtr->RDR); // read RX data

    if (g_UartController[portNum].rxBufferCount == g_UartController[portNum].rxBufferSize) {
        if (g_UartController[portNum].errorEventHandler != nullptr)
            g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, TinyCLR_Uart_Error::ReceiveFull);

        return;
    }

    g_UartController[portNum].RxBuffer[g_UartController[portNum].rxBufferIn++] = data;

    g_UartController[portNum].rxBufferCount++;

    if (g_UartController[portNum].rxBufferIn == g_UartController[portNum].rxBufferSize)
        g_UartController[portNum].rxBufferIn = 0;

    if (g_UartController[portNum].dataReceivedEventHandler != nullptr)
        g_UartController[portNum].dataReceivedEventHandler(g_UartController[portNum].provider, 1);

    if (g_UartController[portNum].errorEventHandler != nullptr) {
        if (sr & USART_ISR_ORE)
            g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, TinyCLR_Uart_Error::BufferOverrun);

        if (sr & USART_ISR_FE)
            g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, TinyCLR_Uart_Error::Frame);

        if (sr & USART_ISR_PE)
            g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, TinyCLR_Uart_Error::ReceiveParity);
    }
}

void STM32F7_Uart_IrqTx(int portNum) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (STM32F7_Uart_TxHandshakeEnabledState(portNum)) {
        if (g_UartController[portNum].txBufferCount > 0) {
            uint8_t data = g_UartController[portNum].TxBuffer[g_UartController[portNum].txBufferOut++];

            g_UartController[portNum].txBufferCount--;

            if (g_UartController[portNum].txBufferOut == g_UartController[portNum].txBufferSize)
                g_UartController[portNum].txBufferOut = 0;

            g_UartController[portNum].portPtr->TDR = data; // write TX data

        }
        else {
            STM32F7_Uart_TxBufferEmptyInterruptEnable(portNum, false); // Disable interrupt when no more data to send.
        }
    }
}

void STM32F7_Uart_InterruptHandler(int8_t portNum, uint16_t sr) {
    if (sr & USART_ISR_RXNE)
        STM32F7_Uart_IrqRx(portNum, sr);

    if (sr & USART_ISR_TXE)
        STM32F7_Uart_IrqTx(portNum);
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

TinyCLR_Result STM32F7_Uart_Acquire(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (portNum >= TOTAL_UART_CONTROLLERS)
        return TinyCLR_Result::ArgumentInvalid;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened || !STM32F7_GpioInternal_OpenPin(g_STM32F7_Uart_Rx_Pins[portNum].number) || !STM32F7_GpioInternal_OpenPin(g_STM32F7_Uart_Tx_Pins[portNum].number))
        return TinyCLR_Result::SharingViolation;

    g_UartController[portNum].txBufferCount = 0;
    g_UartController[portNum].txBufferIn = 0;
    g_UartController[portNum].txBufferOut = 0;

    g_UartController[portNum].rxBufferCount = 0;
    g_UartController[portNum].rxBufferIn = 0;
    g_UartController[portNum].rxBufferOut = 0;

    g_UartController[portNum].portPtr = g_STM32F7_Uart_Ports[portNum];
    g_UartController[portNum].provider = self;

    g_UartController[portNum].handshaking = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {
    int32_t portNum = self->Index;
    uint32_t clk;

    // enable UART clock
    if (portNum == 5) { // COM6 on APB2
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
    else if (portNum == 0) { // COM1 on APB2
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        clk = STM32F7_APB2_CLOCK_HZ;
    }
    else if (portNum < 5) { // COM2-5 on APB1
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN >> 1 << portNum;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#if !defined(STM32F401xE) && !defined(STM32F411xE)
#ifdef UART7
    else if (portNum == 6) {
        RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#endif
#ifdef UART8
    else if (portNum == 7) {
        RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
        clk = STM32F7_APB1_CLOCK_HZ;
    }
#endif
#endif
    //  baudrate
    uint16_t div = (uint16_t)((clk + (baudRate >> 1)) / baudRate); // rounded

    while ((clk / div) > baudRate)
        div++;

    g_UartController[portNum].portPtr->BRR = div;

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

    g_UartController[portNum].portPtr->CR1 = ctrl_cr1;


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

    g_UartController[portNum].portPtr->CR2 = (uint16_t)(stopbit << 12);

    uint16_t ctrl_cr3 = 0;

    switch (handshaking) {
    case TinyCLR_Uart_Handshake::RequestToSend:
        ctrl_cr3 = USART_CR3_CTSE | USART_CR3_RTSE;

        g_UartController[portNum].handshaking = true;
        break;

    case TinyCLR_Uart_Handshake::XOnXOff:
    case TinyCLR_Uart_Handshake::RequestToSendXOnXOff:
        return TinyCLR_Result::NotSupported;
    }

    g_UartController[portNum].portPtr->CR3 = ctrl_cr3;

    auto& tx = g_STM32F7_Uart_Tx_Pins[portNum];
    auto& rx = g_STM32F7_Uart_Rx_Pins[portNum];
    auto& cts = g_STM32F7_Uart_Cts_Pins[portNum];
    auto& rts = g_STM32F7_Uart_Rts_Pins[portNum];

    STM32F7_GpioInternal_ConfigurePin(rx.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, rx.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(tx.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, tx.alternateFunction);

    if (handshaking == TinyCLR_Uart_Handshake::RequestToSend) {
        if (!STM32F7_GpioInternal_OpenPin(cts.number) || !STM32F7_GpioInternal_OpenPin(rts.number))
            return TinyCLR_Result::SharingViolation;

        STM32F7_GpioInternal_ConfigurePin(cts.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, cts.alternateFunction);
        STM32F7_GpioInternal_ConfigurePin(rts.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, rts.alternateFunction);
    }

    switch (portNum) {
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


    g_UartController[portNum].isOpened = true;

    if (g_UartController[portNum].txBufferSize == 0) {
        if (STM32F7_Uart_SetWriteBufferSize(self, g_STM32F7_Uart_TxDefaultBuffersSize[portNum]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;
    }

    if (g_UartController[portNum].rxBufferSize == 0) {
        if (STM32F7_Uart_SetReadBufferSize(self, g_STM32F7_Uart_RxDefaultBuffersSize[portNum]) != TinyCLR_Result::Success)
            return TinyCLR_Result::OutOfMemory;
    }

    STM32F7_Uart_TxBufferEmptyInterruptEnable(portNum, true);
    STM32F7_Uart_RxBufferFullInterruptEnable(portNum, true);

    g_UartController[portNum].portPtr->CR1 |= USART_CR1_UE; // start uart

    return ((cts.number == PIN_NONE || rts.number == PIN_NONE) && handshaking == TinyCLR_Uart_Handshake::RequestToSend) ? TinyCLR_Result::NotSupported : TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Release(const TinyCLR_Uart_Provider* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t portNum = self->Index;

    g_UartController[portNum].portPtr->CR1 = 0; // stop uart

    switch (portNum) {
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

    STM32F7_Uart_RxBufferFullInterruptEnable(portNum, false);
    STM32F7_Uart_TxBufferEmptyInterruptEnable(portNum, false);

    // disable UART clock
    if (portNum == 5) { // COM6 on APB2
        RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
    }
    else if (portNum == 0) { // COM1 on APB2
        RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    }
    else if (portNum < 5) { // COM2-5 on APB1
        RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN >> 1 << portNum);
    }
#if !defined(STM32F401xE) && !defined(STM32F411xE)
#ifdef UART7
    else if (portNum == 6) {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
    }
#endif
#ifdef UART8
    else if (portNum == 7) {
        RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
    }
#endif
#endif

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

    if (g_UartController[portNum].isOpened) {
        STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Rx_Pins[portNum].number);
        STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Tx_Pins[portNum].number);

        if (g_UartController[portNum].handshaking) {
            STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Cts_Pins[portNum].number);
            STM32F7_GpioInternal_ClosePin(g_STM32F7_Uart_Rts_Pins[portNum].number);
        }
    }

    g_UartController[portNum].isOpened = false;

    return TinyCLR_Result::Success;
}

void STM32F7_Uart_Reset() {
    for (auto i = 0; i < TOTAL_UART_CONTROLLERS; i++) {
        g_UartController[i].txBufferSize = 0;
        g_UartController[i].rxBufferSize = 0;

        STM32F7_Uart_Release(uartProviders[i]);

        g_UartController[i].isOpened = false;
    }
}

void STM32F7_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable) {
    if (enable) {
        g_UartController[portNum].portPtr->CR1 |= USART_CR1_TXEIE;  // tx int enable
    }
    else {
        g_UartController[portNum].portPtr->CR1 &= ~USART_CR1_TXEIE; // tx int disable
    }
}

void STM32F7_Uart_RxBufferFullInterruptEnable(int portNum, bool enable) {
    if (enable) {
        g_UartController[portNum].portPtr->CR1 |= USART_CR1_RXNEIE;  // rx int enable
    }
    else {
        g_UartController[portNum].portPtr->CR1 &= ~USART_CR1_RXNEIE; // rx int disable
    }
}

bool STM32F7_Uart_TxHandshakeEnabledState(int portNum) {
    // The state of the CTS input only matters if Flow Control is enabled
    if (g_UartController[portNum].portPtr->CR3 & USART_CR3_CTSE) {
        TinyCLR_Gpio_PinValue value;

        STM32F7_Gpio_Read(nullptr, g_STM32F7_Uart_Cts_Pins[portNum].number, value);

        return !(value == TinyCLR_Gpio_PinValue::High);
    }

    return true; // If this handshake input is not being used, it is assumed to be good
}

TinyCLR_Result STM32F7_Uart_Flush(const TinyCLR_Uart_Provider* self) {
    int32_t portNum = self->Index;

    if (g_UartController[portNum].isOpened == false)
        return TinyCLR_Result::NotAvailable;

    while (g_UartController[portNum].txBufferCount > 0) {
        STM32F7_Time_Delay(nullptr, 1);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    size_t i = 0;;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened == false) {
        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(g_UartController[portNum].rxBufferCount, length);

    while (i < length) {
        buffer[i++] = g_UartController[portNum].RxBuffer[g_UartController[portNum].rxBufferOut];

        g_UartController[portNum].rxBufferOut++;

        g_UartController[portNum].rxBufferCount--;

        if (g_UartController[portNum].rxBufferOut == g_UartController[portNum].rxBufferSize)
            g_UartController[portNum].rxBufferOut = 0;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t portNum = self->Index;
    int32_t i = 0;

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_UartController[portNum].isOpened == false) {
        return TinyCLR_Result::NotAvailable;
    }

    length = std::min(g_UartController[portNum].txBufferSize - g_UartController[portNum].txBufferCount, length);

    if (g_UartController[portNum].txBufferCount == g_UartController[portNum].txBufferSize) {
        if (g_UartController[portNum].errorEventHandler != nullptr)
            g_UartController[portNum].errorEventHandler(g_UartController[portNum].provider, TinyCLR_Uart_Error::TransmitFull);

        return TinyCLR_Result::Success;
    }

    while (i < length) {

        g_UartController[portNum].TxBuffer[g_UartController[portNum].txBufferIn] = buffer[i++];

        g_UartController[portNum].txBufferCount++;

        g_UartController[portNum].txBufferIn++;

        if (g_UartController[portNum].txBufferIn == g_UartController[portNum].txBufferSize)
            g_UartController[portNum].txBufferIn = 0;
    }

    if (length > 0) {
        STM32F7_Uart_TxBufferEmptyInterruptEnable(portNum, true); // Enable Tx to start transfer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler) {
    //TODO
    return TinyCLR_Result::Success;
}
TinyCLR_Result STM32F7_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].errorEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler) {
    int32_t portNum = self->Index;

    g_UartController[portNum].dataReceivedEventHandler = handler;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result STM32F7_Uart_GetUnreadDataCount(const TinyCLR_Uart_Provider* self, size_t& count) {
    count = g_UartController[self->Index].rxBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_GetUnsendDataCount(const TinyCLR_Uart_Provider* self, size_t& count) {
    count = g_UartController[self->Index].txBufferCount;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_ClearReadBuffer(const TinyCLR_Uart_Provider* self) {
    g_UartController[self->Index].rxBufferCount = g_UartController[self->Index].rxBufferIn = g_UartController[self->Index].rxBufferOut = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Uart_ClearSendBuffer(const TinyCLR_Uart_Provider* self) {
    g_UartController[self->Index].txBufferCount = g_UartController[self->Index].txBufferIn = g_UartController[self->Index].txBufferOut = 0;

    return TinyCLR_Result::Success;
}