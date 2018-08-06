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
#define TOTAL_DAC_CONTROLLERS 1

static TinyCLR_Dac_Controller dacControllers[TOTAL_DAC_CONTROLLERS];
static TinyCLR_Api_Info dacApi[TOTAL_DAC_CONTROLLERS];

static const LPC24_Gpio_Pin dacPins[] = LPC24_DAC_PINS;

struct DacState {
    bool isOpened[SIZEOF_ARRAY(dacPins)];
};

static DacState dacStates[TOTAL_DAC_CONTROLLERS];

const char* dacApiNames[TOTAL_DAC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.DacController\\0"
};

void LPC24_Dac_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_DAC_CONTROLLERS; i++) {
        dacControllers[i].ApiInfo = &dacApi[i];
        dacControllers[i].Acquire = &LPC24_Dac_Acquire;
        dacControllers[i].Release = &LPC24_Dac_Release;
        dacControllers[i].OpenChannel = &LPC24_Dac_OpenChannel;
        dacControllers[i].CloseChannel = &LPC24_Dac_CloseChannel;
        dacControllers[i].WriteValue = &LPC24_Dac_WriteValue;
        dacControllers[i].GetMinValue = &LPC24_Dac_GetMinValue;
        dacControllers[i].GetMaxValue = &LPC24_Dac_GetMaxValue;
        dacControllers[i].GetResolutionInBits = &LPC24_Dac_GetResolutionInBits;
        dacControllers[i].GetChannelCount = &LPC24_Dac_GetChannelCount;

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

TinyCLR_Result LPC24_Dac_Acquire(const TinyCLR_Dac_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_Release(const TinyCLR_Dac_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_OpenChannel(const TinyCLR_Dac_Controller* self, uint32_t channel) {
    if (channel >= LPC24_Dac_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC24_Gpio_OpenPin(dacPins[channel].number))
        return TinyCLR_Result::SharingViolation;

    auto state = reinterpret_cast<DacState*>(self->ApiInfo->State);

    LPC24_Gpio_ConfigurePin(dacPins[channel].number, LPC24_Gpio_Direction::Input, dacPins[channel].pinFunction, LPC24_Gpio_PinMode::Inactive);

    DACR = (0 << 6); // This sets the initial starting voltage at 0

    state->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_CloseChannel(const TinyCLR_Dac_Controller* self, uint32_t channel) {
    if (channel >= LPC24_Dac_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    auto state = reinterpret_cast<DacState*>(self->ApiInfo->State);
    if (state->isOpened[channel])
        LPC24_Gpio_ClosePin(dacPins[channel].number);

    state->isOpened[channel] = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Dac_WriteValue(const TinyCLR_Dac_Controller* self, uint32_t channel, int32_t value) {
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

uint32_t LPC24_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self) {
    return SIZEOF_ARRAY(dacPins);
}

uint32_t LPC24_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self) {
    return LPC24_DAC_PRECISION_BITS;
}

int32_t LPC24_Dac_GetMinValue(const TinyCLR_Dac_Controller* self) {
    return 0;
}

int32_t LPC24_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self) {
    return ((1 << LPC24_DAC_PRECISION_BITS) - 1);
}

void LPC24_Dac_Reset() {
    for (auto c = 0; c < TOTAL_DAC_CONTROLLERS; c++) {
        for (auto ch = 0; ch < LPC24_Dac_GetChannelCount(&dacControllers[c]); ch++) {
            LPC24_Dac_CloseChannel(&dacControllers[c], ch);

            dacStates[c].isOpened[ch] = false;
        }
    }
}

