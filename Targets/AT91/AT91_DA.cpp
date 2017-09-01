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

#define AT91_DAC_PRECISION_BITS 	10	// Number of Bits in the DAC Convertion
#define AT91_DAC_MAX_VALUE 	(1<<AT91_DAC_PRECISION_BITS)

///////////////////////////////////////////////////////////////////////////////

static TinyCLR_Dac_Provider dacProvider;
static TinyCLR_Api_Info dacApi;

static const uint32_t g_AT91_Dac_Pins[] = AT91_DAC_PINS;
static const AT91_Gpio_PinFunction g_AT91_Dac_altMode[] = AT91_DAC_ALT_MODE;

const TinyCLR_Api_Info* AT91_Dac_GetApi() {
    dacProvider.Parent = &dacApi;
    dacProvider.Index = 0;
    dacProvider.Acquire = &AT91_Dac_Acquire;
    dacProvider.Release = &AT91_Dac_Release;
    dacProvider.AcquireChannel = &AT91_Dac_AcquireChannel;
    dacProvider.ReleaseChannel = &AT91_Dac_ReleaseChannel;
    dacProvider.WriteValue = &AT91_Dac_WriteValue;
    dacProvider.GetChannelCount = &AT91_Dac_GetChannelCount;
    dacProvider.GetResolutionInBits = &AT91_Dac_GetResolutionInBits;
    dacProvider.GetMinValue = &AT91_Dac_GetMinValue;
    dacProvider.GetMaxValue = &AT91_Dac_GetMaxValue;

    dacApi.Author = "GHI Electronics, LLC";
    dacApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.DacProvider";
    dacApi.Type = TinyCLR_Api_Type::DacProvider;
    dacApi.Version = 0;
    dacApi.Count = 1;
    dacApi.Implementation = &dacProvider;

    return &dacApi;
}

TinyCLR_Result AT91_Dac_Acquire(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Dac_Release(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (channel >= TOTAL_DAC_CONTROLLERS)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!AT91_Gpio_OpenPin(g_AT91_Dac_Pins[channel]))
        return TinyCLR_Result::SharingViolation;

    AT91_Gpio_ConfigurePin(g_AT91_Dac_Pins[channel], AT91_Gpio_Direction::Input, g_AT91_Dac_altMode[channel], AT91_Gpio_PinMode::Inactive);   

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (channel >= TOTAL_DAC_CONTROLLERS)
        return TinyCLR_Result::ArgumentOutOfRange;

    AT91_Gpio_ClosePin(g_AT91_Dac_Pins[channel]);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value) {
    if (channel >= TOTAL_DAC_CONTROLLERS)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (value > AT91_DAC_MAX_VALUE) {
        value = AT91_DAC_MAX_VALUE;
    }

    if (value < 1) {
        value = 1;
    }

    return TinyCLR_Result::Success;
}

int32_t AT91_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self) {
    return TOTAL_DAC_CONTROLLERS;
}

int32_t AT91_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self) {
    return AT91_DAC_PRECISION_BITS;
}

int32_t AT91_Dac_GetMinValue(const TinyCLR_Dac_Provider* self) {
    return 0;
}

int32_t AT91_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self) {
    return ((1 << AT91_DAC_PRECISION_BITS) - 1);
}

void AT91_Dac_Reset() {
    for (auto ch = 0; ch < AT91_Dac_GetChannelCount(&dacProvider); ch++) {
        AT91_Dac_ReleaseChannel(&dacProvider, ch);
    }
}

