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
#define TOTAL_DAC_CONTROLLERS 1

static TinyCLR_Dac_Controller dacControllers[TOTAL_DAC_CONTROLLERS];
static TinyCLR_Api_Info dacApi[TOTAL_DAC_CONTROLLERS];

static const LPC17_Gpio_Pin dacPins[] = LPC17_DAC_PINS;

struct DacState {
    bool isOpened[SIZEOF_ARRAY(dacPins)];
};

static DacState dacStates[TOTAL_DAC_CONTROLLERS];

const char* dacApiNames[TOTAL_DAC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC17.DacController\\0"
};

void LPC17_Dac_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_DAC_CONTROLLERS; i++) {
        dacControllers[i].ApiInfo = &dacApi[i];
        dacControllers[i].Acquire = &LPC17_Dac_Acquire;
        dacControllers[i].Release = &LPC17_Dac_Release;
        dacControllers[i].OpenChannel = &LPC17_Dac_OpenChannel;
        dacControllers[i].CloseChannel = &LPC17_Dac_CloseChannel;
        dacControllers[i].WriteValue = &LPC17_Dac_WriteValue;
        dacControllers[i].GetMinValue = &LPC17_Dac_GetMinValue;
        dacControllers[i].GetMaxValue = &LPC17_Dac_GetMaxValue;
        dacControllers[i].GetResolutionInBits = &LPC17_Dac_GetResolutionInBits;
        dacControllers[i].GetChannelCount = &LPC17_Dac_GetChannelCount;

        dacApi[i].Author = "GHI Electronics, LLC";
        dacApi[i].Name = dacApiNames[i];
        dacApi[i].Type = TinyCLR_Api_Type::DacController;
        dacApi[i].Version = 0;
        dacApi[i].Implementation = &dacControllers[i];
        dacApi[i].State = &dacStates[i];

        apiManager->Add(apiManager, &dacApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::DacController, dacApi[0].Name);
}

TinyCLR_Result LPC17_Dac_Acquire(const TinyCLR_Dac_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_Release(const TinyCLR_Dac_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_OpenChannel(const TinyCLR_Dac_Controller* self, uint32_t channel) {
    if (channel >= SIZEOF_ARRAY(dacPins))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC17_Gpio_OpenPin(dacPins[channel].number))
        return  TinyCLR_Result::SharingViolation;

    auto state = reinterpret_cast<DacState*>(self->ApiInfo->State);

    LPC17_Gpio_ConfigurePin(dacPins[channel].number, LPC17_Gpio_Direction::Output, dacPins[channel].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    DACR = (0 << 6); // This sets the initial starting voltage at 0

    state->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_CloseChannel(const TinyCLR_Dac_Controller* self, uint32_t channel) {
    if (channel >= SIZEOF_ARRAY(dacPins))
        return TinyCLR_Result::ArgumentOutOfRange;

    auto state = reinterpret_cast<DacState*>(self->ApiInfo->State);

    if (state->isOpened[channel]) {
        DACR = (0 << 6); // This sets the initial starting voltage at 0

        LPC17_Gpio_ClosePin(dacPins[channel].number);
    }

    state->isOpened[channel] = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Dac_WriteValue(const TinyCLR_Dac_Controller* self, uint32_t channel, int32_t value) {
    if (channel >= SIZEOF_ARRAY(dacPins))
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

uint32_t LPC17_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self) {
    return SIZEOF_ARRAY(dacPins);
}

uint32_t LPC17_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self) {
    return LPC17_DAC_PRECISION_BITS;
}

int32_t LPC17_Dac_GetMinValue(const TinyCLR_Dac_Controller* self) {
    return 0;
}

int32_t LPC17_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self) {
    return ((1 << LPC17_DAC_PRECISION_BITS) - 1);
}

void LPC17_Dac_Reset() {
    for (auto c = 0; c < TOTAL_DAC_CONTROLLERS; c++) {
        for (auto ch = 0; ch < LPC17_Dac_GetChannelCount(&dacControllers[c]); ch++) {
            LPC17_Dac_CloseChannel(&dacControllers[c], ch);

            dacStates[c].isOpened[ch] = false;
        }
    }
}
