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

#include "LPC17.h"

#define DAC_BASE 0x4008C000

#define DACR (*(volatile unsigned long *)0x4008C000)
#define DACR_OFFSET 0x0
#define DACR_VALUE_MASK 0xFFC0
#define DACR_VALUE_BIT 6
#define DACR_BIAS_MASK 0x10000
#define DACR_BIAS 0x10000
#define DACR_BIAS_BIT 16

#define LPC17_DAC_PRECISION_BITS 	10	// Number of Bits in the DAC Convertion
#define LPC17_DAC_MAX_VALUE 	(1<<LPC17_DAC_PRECISION_BITS)

///////////////////////////////////////////////////////////////////////////////

static TinyCLR_Dac_Provider dacProvider;
static TinyCLR_Api_Info dacApi;

static const LPC17_Gpio_Pin g_lpc17_dac_pins[] = LPC17_DAC_PINS;

bool g_lpc17_dac_isOpened[SIZEOF_ARRAY(g_lpc17_dac_pins)];

const TinyCLR_Api_Info* LPC17_Dac_GetApi() {
    dacProvider.Parent = &dacApi;
    dacProvider.Acquire = &LPC17_Dac_Acquire;
    dacProvider.Release = &LPC17_Dac_Release;
    dacProvider.AcquireChannel = &LPC17_Dac_AcquireChannel;
    dacProvider.ReleaseChannel = &LPC17_Dac_ReleaseChannel;
    dacProvider.WriteValue = &LPC17_Dac_WriteValue;
    dacProvider.GetChannelCount = &LPC17_Dac_GetChannelCount;
    dacProvider.GetResolutionInBits = &LPC17_Dac_GetResolutionInBits;
    dacProvider.GetMinValue = &LPC17_Dac_GetMinValue;
    dacProvider.GetMaxValue = &LPC17_Dac_GetMaxValue;

    dacApi.Author = "GHI Electronics, LLC";
    dacApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.DacProvider";
    dacApi.Type = TinyCLR_Api_Type::DacProvider;
    dacApi.Version = 0;
    dacApi.Implementation = &dacProvider;

    return &dacApi;
}

TinyCLR_Result LPC17_Dac_Acquire(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_Release(const TinyCLR_Dac_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (channel >= SIZEOF_ARRAY(g_lpc17_dac_pins))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC17_Gpio_OpenPin(g_lpc17_dac_pins[channel].number))
        return  TinyCLR_Result::SharingViolation;

    LPC17_Gpio_ConfigurePin(g_lpc17_dac_pins[channel].number, LPC17_Gpio_Direction::Output, g_lpc17_dac_pins[channel].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    DACR = (0 << 6); // This sets the initial starting voltage at 0

    g_lpc17_dac_isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel) {
    if (channel >= SIZEOF_ARRAY(g_lpc17_dac_pins))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (g_lpc17_dac_isOpened[channel]) {
        DACR = (0 << 6); // This sets the initial starting voltage at 0

        LPC17_Gpio_ClosePin(g_lpc17_dac_pins[channel].number);
    }

    g_lpc17_dac_isOpened[channel] = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value) {
    if (channel >= SIZEOF_ARRAY(g_lpc17_dac_pins))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (value > LPC17_DAC_MAX_VALUE) {
        value = LPC17_DAC_MAX_VALUE;
    }

    if (value < 1) {
        value = 1;
    }

    DACR = ((value - 1) << 6); // Sets voltage level between 0 and 1023.

    return TinyCLR_Result::Success;
}

int32_t LPC17_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self) {
    return SIZEOF_ARRAY(g_lpc17_dac_pins);
}

int32_t LPC17_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self) {
    return LPC17_DAC_PRECISION_BITS;
}

int32_t LPC17_Dac_GetMinValue(const TinyCLR_Dac_Provider* self) {
    return 0;
}

int32_t LPC17_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self) {
    return ((1 << LPC17_DAC_PRECISION_BITS) - 1);
}

void LPC17_Dac_Reset() {
    for (auto ch = 0; ch < LPC17_Dac_GetChannelCount(&dacProvider); ch++) {
        LPC17_Dac_ReleaseChannel(&dacProvider, ch);

        g_lpc17_dac_isOpened[ch] = false;
    }
}

TinyCLR_Result LPC17_Dac_GetControllerCount(const TinyCLR_Dac_Provider* self, int32_t& count) {
    count = 1;

    return TinyCLR_Result::Success;
}

