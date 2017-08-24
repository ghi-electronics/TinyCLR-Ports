// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
// Copyright 2017 GHI Electronics, LLC
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

#ifdef STM32F427xx
///////////////////////////////////////////////////////////////////////////////

#define STM32F4_DAC_CHANNELS             2       // number of channels
#define STM32F4_DAC_FIRST_PIN           4       // channel 0 pin (A4)
#define STM32F4_DAC_RESOLUTION_INT_BIT    12      // max resolution in bit

static TinyCLR_Dac_Provider dacProvider;
static TinyCLR_Api_Info dacApi;

const TinyCLR_Api_Info* STM32F4_Dac_GetApi() {
    dacProvider.Parent = &dacApi;
    dacProvider.Index = 0;
    dacProvider.Acquire = &STM32F4_Dac_Acquire;
    dacProvider.Release = &STM32F4_Dac_Release;
    dacProvider.AcquireChannel = &STM32F4_Dac_AcquireChannel;
    dacProvider.ReleaseChannel = &STM32F4_Dac_ReleaseChannel;
    dacProvider.WriteValue = &STM32F4_Dac_WriteValue;
    dacProvider.GetChannelCount = &STM32F4_Dac_GetChannelCount;
    dacProvider.GetResolutionInBits = &STM32F4_Dac_GetResolutionInBits;
    dacProvider.GetMinValue = &STM32F4_Dac_GetMinValue;
    dacProvider.GetMaxValue = &STM32F4_Dac_GetMaxValue;

    dacApi.Author = "GHI Electronics, LLC";
    dacApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.DacProvider";
    dacApi.Type = TinyCLR_Api_Type::DacProvider;
    dacApi.Version = 0;
    dacApi.Count = 1;
    dacApi.Implementation = &dacProvider;

    return &dacApi;
}

TinyCLR_Result STM32F4_Dac_Acquire(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_Release(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (!STM32F4_Gpio_OpenPin(STM32F4_DAC_FIRST_PIN + channel))
        return TinyCLR_Result::SharingViolation;

    // enable DA clock
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // set pin as analog
    STM32F4_Gpio_ConfigurePin(STM32F4_DAC_FIRST_PIN + channel, STM32F4_Gpio_PortMode::Analog, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);

    if (channel) {
        DAC->CR |= DAC_CR_EN2; // enable channel 2
    }
    else {
        DAC->CR |= DAC_CR_EN1; // enable channel 1
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    TinyCLR_Result releasePin = STM32F4_Gpio_ReleasePin(nullptr, STM32F4_DAC_FIRST_PIN + channel);

    if (releasePin != TinyCLR_Result::Success)
        return releasePin;

    if (channel) {
        DAC->CR &= ~DAC_CR_EN2; // disable channel 2
    }
    else {
        DAC->CR &= ~DAC_CR_EN1; // disable channel 1
    }

    // free pin
    STM32F4_Gpio_ClosePin(STM32F4_DAC_FIRST_PIN + channel);

    if ((DAC->CR & (DAC_CR_EN1 | DAC_CR_EN2)) == 0) { // all channels off
        // disable DA clock
        RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value) {
    value &= 0x00000FFF;

    if (channel)
        DAC->DHR12R2 = value;
    else
        DAC->DHR12R1 = value;

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self) {
    return STM32F4_DAC_CHANNELS;
}

int32_t STM32F4_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self) {
    return STM32F4_DAC_RESOLUTION_INT_BIT;
}

int32_t STM32F4_Dac_GetMinValue(const TinyCLR_Dac_Provider* self) {
    return 0;
}

int32_t STM32F4_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self) {
    return ((1 << STM32F4_DAC_RESOLUTION_INT_BIT) - 1);
}

void STM32F4_Dac_Reset() {
    for (auto i = 0; i < STM32F4_Dac_GetChannelCount(&dacProvider); i++) {
        STM32F4_Dac_ReleaseChannel(&dacProvider, i);
    }
}

#endif