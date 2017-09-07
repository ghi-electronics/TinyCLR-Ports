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

#include "AT91.h"

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

const TinyCLR_Api_Info* AT91_Adc_GetApi() {
    adcProvider.Parent = &adcApi;
    adcProvider.Index = 0;
    adcProvider.Acquire = &AT91_Adc_Acquire;
    adcProvider.Release = &AT91_Adc_Release;
    adcProvider.AcquireChannel = &AT91_Adc_AcquireChannel;
    adcProvider.ReleaseChannel = &AT91_Adc_ReleaseChannel;
    adcProvider.IsChannelModeSupported = &AT91_Adc_IsChannelModeSupported;
    adcProvider.ReadValue = &AT91_Adc_ReadValue;
    adcProvider.GetMinValue = &AT91_Adc_GetMinValue;
    adcProvider.GetMaxValue = &AT91_Adc_GetMaxValue;
    adcProvider.GetResolutionInBits = &AT91_Adc_GetResolutionInBits;
    adcProvider.GetChannelCount = &AT91_Adc_GetChannelCount;
    adcProvider.GetChannelMode = &AT91_Adc_GetChannelMode;
    adcProvider.SetChannelMode = &AT91_Adc_SetChannelMode;

    adcApi.Author = "GHI Electronics, LLC";
    adcApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.AdcProvider";
    adcApi.Type = TinyCLR_Api_Type::AdcProvider;
    adcApi.Version = 0;
    adcApi.Count = 1;
    adcApi.Implementation = &adcProvider;

    return &adcApi;
}

TinyCLR_Result AT91_Adc_Acquire(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_Release(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetPinForChannel(int32_t channel) {
    if ((uint32_t)channel >= AT91_Adc_GetControllerCount())
        return GPIO_PIN_NONE;

    return AT91_Adc_GetPin(channel);
}

TinyCLR_Result AT91_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    if (channel >= AT91_Adc_GetControllerCount())
        return TinyCLR_Result::ArgumentOutOfRange;

    if (AT91_Adc_GetPin(channel) == _P_NONE_)
        return TinyCLR_Result::ArgumentInvalid;

    //AT91_Gpio_ConfigurePin(AT91_Adc_GetPin(channel), AT91_Gpio_Direction::Input, AT91_Adc_GetPinFunction(channel), AT91_Gpio_PinMode::Inactive);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    //AT91_Gpio_ConfigurePin(AT91_Adc_GetPin(channel), AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value) {
    uint32_t result = 0;

    if (channel >= AT91_Adc_GetControllerCount())
        return TinyCLR_Result::ArgumentOutOfRange;

    value = 0;

    // channel not available
    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self) {
    return AT91_Adc_GetControllerCount();
}

int32_t AT91_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self) {
    return 10;
}

int32_t AT91_Adc_GetMinValue(const TinyCLR_Adc_Provider* self) {
    return 0;
}

int32_t AT91_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self) {
    return (1 << AT91_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode AT91_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result AT91_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool AT91_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void AT91_Adc_Reset() {
    for (auto ch = 0; ch < AT91_Adc_GetControllerCount(); ch++) {
        AT91_Adc_ReleaseChannel(&adcProvider, ch);
    }  
}
