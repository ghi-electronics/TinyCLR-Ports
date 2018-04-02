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

#include "LPC24.h"

#define DACR (*(volatile unsigned long *)0xE006C000)


#define LPC24_DAC_PRECISION_BITS 	10	// Number of Bits in the DAC Convertion
#define LPC24_DAC_MAX_VALUE 	(1<<LPC24_DAC_PRECISION_BITS)

///////////////////////////////////////////////////////////////////////////////

static TinyCLR_Dac_Provider dacProvider;
static TinyCLR_Api_Info dacApi;

static const LPC24_Gpio_Pin g_LPC24_Dac_Pins[] = LPC24_DAC_PINS;

bool g_LPC24_Dac_IsOpened[SIZEOF_ARRAY(g_LPC24_Dac_Pins)];

const TinyCLR_Api_Info* LPC24_Dac_GetApi() {
    dacProvider.Parent = &dacApi;
    dacProvider.Index = 0;
    dacProvider.Acquire = &LPC24_Dac_Acquire;
    dacProvider.Release = &LPC24_Dac_Release;
    dacProvider.AcquireChannel = &LPC24_Dac_AcquireChannel;
    dacProvider.ReleaseChannel = &LPC24_Dac_ReleaseChannel;
    dacProvider.WriteValue = &LPC24_Dac_WriteValue;
    dacProvider.GetChannelCount = &LPC24_Dac_GetChannelCount;
    dacProvider.GetResolutionInBits = &LPC24_Dac_GetResolutionInBits;
    dacProvider.GetMinValue = &LPC24_Dac_GetMinValue;
    dacProvider.GetMaxValue = &LPC24_Dac_GetMaxValue;

    dacApi.Author = "GHI Electronics, LLC";
    dacApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.DacProvider";
    dacApi.Type = TinyCLR_Api_Type::DacProvider;
    dacApi.Version = 0;
    dacApi.Count = 1;
    dacApi.Implementation = &dacProvider;

    return &dacApi;
}

TinyCLR_Result LPC24_Dac_Acquire(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_Release(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (channel >= LPC24_Dac_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC24_Gpio_OpenPin(g_LPC24_Dac_Pins[channel].number))
        return TinyCLR_Result::SharingViolation;

    LPC24_Gpio_ConfigurePin(g_LPC24_Dac_Pins[channel].number, LPC24_Gpio_Direction::Input, g_LPC24_Dac_Pins[channel].pinFunction, LPC24_Gpio_PinMode::Inactive);

    DACR = (0 << 6); // This sets the initial starting voltage at 0

    g_LPC24_Dac_IsOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (channel >= LPC24_Dac_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (g_LPC24_Dac_IsOpened[channel])
        LPC24_Gpio_ClosePin(g_LPC24_Dac_Pins[channel].number);

    g_LPC24_Dac_IsOpened[channel] = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value) {
    if (channel >= LPC24_Dac_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (value > LPC24_DAC_MAX_VALUE) {
        value = LPC24_DAC_MAX_VALUE;
    }

    if (value < 1) {
        value = 1;
    }

    DACR = ((value - 1) << 6); // Sets voltage level between 0 and 1023.

    return TinyCLR_Result::Success;
}

int32_t LPC24_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self) {
    return SIZEOF_ARRAY(g_LPC24_Dac_Pins);
}

int32_t LPC24_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self) {
    return LPC24_DAC_PRECISION_BITS;
}

int32_t LPC24_Dac_GetMinValue(const TinyCLR_Dac_Provider* self) {
    return 0;
}

int32_t LPC24_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self) {
    return ((1 << LPC24_DAC_PRECISION_BITS) - 1);
}

void LPC24_Dac_Reset() {
    for (auto ch = 0; ch < LPC24_Dac_GetChannelCount(&dacProvider); ch++) {
        LPC24_Dac_ReleaseChannel(&dacProvider, ch);

        g_LPC24_Dac_IsOpened[ch] = false;
    }
}

