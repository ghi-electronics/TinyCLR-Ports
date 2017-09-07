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

    AT91_Gpio_PeripheralSelection txPinMode = AT91_Uart_GetTxAlternateFunction(portNum);
    AT91_Gpio_PeripheralSelection rxPinMode = AT91_Uart_GetRxAlternateFunction(portNum);
    AT91_Gpio_PeripheralSelection ctsPinMode = AT91_Uart_GetCtsAlternateFunction(portNum);
    AT91_Gpio_PeripheralSelection rtsPinMode = AT91_Uart_GetRtsAlternateFunction(portNum);

    if (enable) {        
        if (g_AT91_Uart_Controller[portNum].handshakeEnable) {
            //AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, ctsPinMode, AT91_Gpio_PinMode::Inactive);
            //AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, rtsPinMode, AT91_Gpio_PinMode::Inactive);
        }

    }
    else {
        
        if (g_AT91_Uart_Controller[portNum].handshakeEnable) {
            //AT91_Gpio_ConfigurePin(ctsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
            //AT91_Gpio_ConfigurePin(rtsPin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
        }
    }
}

void AT91_Uart_SetErrorEvent(int32_t portNum, TinyCLR_Uart_Error error) {
    if (g_AT91_Uart_Controller[portNum].errorEventHandler != nullptr)
        g_AT91_Uart_Controller[portNum].errorEventHandler(g_AT91_Uart_Controller[portNum].provider, error);
}

void AT91_Uart_InterruptHandler(void *param) {
    INTERRUPT_START

    GLOBAL_LOCK(irq);

    uint32_t portNum = (uint32_t)param;

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
            
            break;

        case 1:
           
            break;

        case 2:
           
            break;

        case 3:
            
            break;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking) {

    GLOBAL_LOCK(irq);

    int32_t portNum = self->Index;

    AT91_Uart_PinConfiguration(portNum, true);

    g_AT91_Uart_Controller[portNum].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Provider* self) {
    GLOBAL_LOCK(irq);

    int32_t portNum = self->Index;

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
            
            break;

        case 1:
            
            break;

        case 2:
            
            break;

        case 3:
            
            break;
    }

    return TinyCLR_Result::Success;
}

void AT91_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable) {
    GLOBAL_LOCK(irq);

    if (enable) {
        
    }
    else {
        
    }
}

void AT91_Uart_RxBufferFullInterruptEnable(int portNum, bool enable) {
    GLOBAL_LOCK(irq);

    if (enable) {
    }
    else {
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

