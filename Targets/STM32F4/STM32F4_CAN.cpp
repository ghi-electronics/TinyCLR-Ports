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
#include "STM32F4.h"

///////////////////////////////////////////////////////////////////////////////

#define INCLUDE_CAN

static TinyCLR_Can_Provider canProvider;
static TinyCLR_Api_Info canApi;

static const STM32F4_Gpio_Pin g_STM32F4_Can_Tx_Pins[] = STM32F4_CAN_TX_PINS;
static const STM32F4_Gpio_Pin g_STM32F4_Can_Rx_Pins[] = STM32F4_CAN_RX_PINS;
static const int TOTAL_CAN_CONTROLLERS = SIZEOF_ARRAY(g_STM32F4_Can_Tx_Pins);

const TinyCLR_Api_Info* STM32F4_Can_GetApi() {
   
    return nullptr;
}

TinyCLR_Result STM32F4_Can_Acquire(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;
        
    int32_t channel = self->Index; 
        
    if (!STM32F4_GpioInternal_OpenPin(g_STM32F4_Can_Tx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    if (!STM32F4_GpioInternal_OpenPin(g_STM32F4_Can_Rx_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;
    // set pin as analog
    STM32F4_GpioInternal_ConfigurePin(g_STM32F4_Can_Tx_Pins[channel].number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, g_STM32F4_Can_Tx_Pins[channel].alternateFunction);
    STM32F4_GpioInternal_ConfigurePin(g_STM32F4_Can_Rx_Pins[channel].number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, g_STM32F4_Can_Rx_Pins[channel].alternateFunction);
    
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_Release(const TinyCLR_Can_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t channel = self->Index; 

    TinyCLR_Result releasePin = STM32F4_Gpio_ReleasePin(nullptr, g_STM32F4_Can_Tx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;
        
    releasePin = STM32F4_Gpio_ReleasePin(nullptr, g_STM32F4_Can_Rx_Pins[channel].number);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    // free pin
    STM32F4_GpioInternal_ClosePin(g_STM32F4_Can_Tx_Pins[channel].number);
    STM32F4_GpioInternal_ClosePin(g_STM32F4_Can_Rx_Pins[channel].number);

    return TinyCLR_Result::Success;
}
TinyCLR_Result STM32F4_Can_Reset(const TinyCLR_Can_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_PostMessage(const TinyCLR_Can_Provider* self, uint32_t arbID, uint32_t flags, uint8_t *data) {
    
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Can_GetMessage(const TinyCLR_Can_Provider* self, uint32_t * arbID, uint32_t *flags, uint64_t *ts, uint8_t *data) {
    
    return TinyCLR_Result::Success;
}
#endif // INCLUDE_CAN
