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

#include "STM32F4.h"

#ifdef INCLUDE_DAC
///////////////////////////////////////////////////////////////////////////////

#define TOTAL_DAC_CONTROLLERS 1
#define STM32F4_DAC_CHANNEL_NUMS 2
#define STM32F4_DAC_FIRST_PIN 4
#define STM32F4_DAC_RESOLUTION_INT_BIT 12

static TinyCLR_Dac_Controller dacControllers[TOTAL_DAC_CONTROLLERS];
static TinyCLR_Api_Info dacApi[TOTAL_DAC_CONTROLLERS];

struct DacDriver {
    bool isOpened[STM32F4_DAC_CHANNEL_NUMS];
}

static DacDriver dacDriver[TOTAL_DAC_CONTROLLERS];

const TinyCLR_Api_Info* STM32F4_Dac_GetApi() {
    for (int32_t i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        dacControllers[i].ApiInfo = &dacApi[i];
        dacControllers[i].Acquire = &STM32F4_Dac_Acquire;
        dacControllers[i].Release = &STM32F4_Dac_Release;
        dacControllers[i].AcquireChannel = &STM32F4_Dac_AcquireChannel;
        dacControllers[i].ReleaseChannel = &STM32F4_Dac_ReleaseChannel;
        dacControllers[i].WriteValue = &STM32F4_Dac_WriteValue;
        dacControllers[i].GetMinValue = &STM32F4_Dac_GetMinValue;
        dacControllers[i].GetMaxValue = &STM32F4_Dac_GetMaxValue;
        dacControllers[i].GetResolutionInBits = &STM32F4_Dac_GetResolutionInBits;
        dacControllers[i].GetChannelCount = &STM32F4_Dac_GetChannelCount;

        dacApi[i].Author = "GHI Electronics, LLC";
        dacApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.DacProvider";
        dacApi[i].Type = TinyCLR_Api_Type::DacProvider;
        dacApi[i].Version = 0;
        dacApi[i].Implementation = &dacControllers[i];
        dacApi[i].State = &dacDriver[i];
    }

    return &dacApi;
}

TinyCLR_Result STM32F4_Dac_Acquire(const TinyCLR_Dac_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_Release(const TinyCLR_Dac_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_AcquireChannel(const TinyCLR_Dac_Controller* self, int32_t channel) {
    if (!STM32F4_GpioInternal_OpenPin(STM32F4_DAC_FIRST_PIN + channel))
        return TinyCLR_Result::SharingViolation;

    auto driver = reinterpret_cast<DacDriver*>(self->ApiInfo->State);

    // enable DA clock
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // set pin as analog
    STM32F4_GpioInternal_ConfigurePin(STM32F4_DAC_FIRST_PIN + channel, STM32F4_Gpio_PortMode::Analog, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);

    if (channel) {
        DAC->CR |= DAC_CR_EN2; // enable channel 2
    }
    else {
        DAC->CR |= DAC_CR_EN1; // enable channel 1
    }

    driver->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_ReleaseChannel(const TinyCLR_Dac_Controller* self, int32_t channel) {
    auto driver = reinterpret_cast<DacDriver*>(self->ApiInfo->State);

    if (channel) {
        DAC->CR &= ~DAC_CR_EN2; // disable channel 2
    }
    else {
        DAC->CR &= ~DAC_CR_EN1; // disable channel 1
    }

    if ((DAC->CR & (DAC_CR_EN1 | DAC_CR_EN2)) == 0) { // all channels off
        // disable DA clock
        RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
    }

    if (driver->isOpened[channel])
        STM32F4_GpioInternal_ClosePin(STM32F4_DAC_FIRST_PIN + channel);

    driver->isOpened[channel] = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_WriteValue(const TinyCLR_Dac_Controller* self, int32_t channel, int32_t value) {
    value &= 0x00000FFF;

    if (channel)
        DAC->DHR12R2 = value;
    else
        DAC->DHR12R1 = value;

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self) {
    return STM32F4_DAC_CHANNEL_NUMS;
}

int32_t STM32F4_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self) {
    return STM32F4_DAC_RESOLUTION_INT_BIT;
}

int32_t STM32F4_Dac_GetMinValue(const TinyCLR_Dac_Controller* self) {
    return 0;
}

int32_t STM32F4_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self) {
    return ((1 << STM32F4_DAC_RESOLUTION_INT_BIT) - 1);
}

void STM32F4_Dac_Reset() {
    for (auto c = 0; c < TOTAL_DAC_CONTROLLERS; c++) {
        for (auto i = 0; i < STM32F4_Dac_GetChannelCount(&dacControllers[c], 0); i++) {
            STM32F4_Dac_ReleaseChannel(&dacControllers[c], i);

            dacDriver[c].isOpened[i] = false;
        }
    }
}
#endif