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

#define AD0CR (*(volatile unsigned *)0xE0034000)

#define LPC24xx_ADC_DataRegisterShiftBits	4
#define LPC24xx_ADC_BitRegisterMask			0xFFF

#define ADC_DATA_BASE_ADDRESS	0xE0034010
#define ADC_GLOBAR_DATA_ADDRESS 0xE0034004

#define TOTAL_ADC_CONTROLLERS 1

static TinyCLR_Adc_Controller adcControllers[TOTAL_ADC_CONTROLLERS];
static TinyCLR_Api_Info adcApi[TOTAL_ADC_CONTROLLERS];

const char* adcApiNames[TOTAL_ADC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.AdcController\\0"
};

struct AdcState {
    static uint8_t isOpen;
};

uint8_t AdcState::isOpen;

void LPC24_Adc_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_ADC_CONTROLLERS; i++) {
        adcControllers[i].ApiInfo = &adcApi[i];
        adcControllers[i].Acquire = &LPC24_Adc_Acquire;
        adcControllers[i].Release = &LPC24_Adc_Release;
        adcControllers[i].OpenChannel = &LPC24_Adc_OpenChannel;
        adcControllers[i].CloseChannel = &LPC24_Adc_CloseChannel;
        adcControllers[i].ReadChannel = &LPC24_Adc_ReadChannel;
        adcControllers[i].SetChannelMode = &LPC24_Adc_SetChannelMode;
        adcControllers[i].GetChannelMode = &LPC24_Adc_GetChannelMode;
        adcControllers[i].IsChannelModeSupported = &LPC24_Adc_IsChannelModeSupported;
        adcControllers[i].GetMinValue = &LPC24_Adc_GetMinValue;
        adcControllers[i].GetMaxValue = &LPC24_Adc_GetMaxValue;
        adcControllers[i].GetResolutionInBits = &LPC24_Adc_GetResolutionInBits;
        adcControllers[i].GetChannelCount = &LPC24_Adc_GetChannelCount;

        adcApi[i].Author = "GHI Electronics, LLC";
        adcApi[i].Name = adcApiNames[i];
        adcApi[i].Type = TinyCLR_Api_Type::AdcController;
        adcApi[i].Version = 0;
        adcApi[i].Implementation = &adcControllers[i];
        adcApi[i].State = &adcStates[i];
        
        apiManager->Add(apiManager, &adcApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::AdcController, adcApi[0].Name);
}

TinyCLR_Result LPC24_Adc_Acquire(const TinyCLR_Adc_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Adc_Release(const TinyCLR_Adc_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

uint32_t LPC24_Adc_GetPinForChannel(uint32_t channel) {
    if ((uint32_t)channel >= LPC24_Adc_GetChannelCount(&adcControllers[0]))
        return PIN_NONE;

    return LPC24_Adc_GetPin(channel);
}

TinyCLR_Result LPC24_Adc_OpenChannel(const TinyCLR_Adc_Controller* self, uint32_t channel) {
    if (channel >= LPC24_Adc_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (LPC24_Adc_GetPin(channel) == PIN_NONE)
        return TinyCLR_Result::ArgumentInvalid;

    if (!LPC24_Gpio_OpenPin(LPC24_Adc_GetPin(channel)))
        return  TinyCLR_Result::SharingViolation;

    LPC24XX::SYSCON().PCONP |= PCONP_PCAD;

    LPC24_Gpio_ConfigurePin(LPC24_Adc_GetPin(channel), LPC24_Gpio_Direction::Input, LPC24_Adc_GetPinFunction(channel), LPC24_Gpio_PinMode::Inactive);

    AD0CR |= (1 << channel) |// sample one of the pins
        ((3 - 1) << 8) |//devide the clock by 14 60/14= 4.3 (must be <4.5)
        (1 << 16) |//burst mode
        (0 << 17) |//10 bits
        (1 << 21);//operational

    auto state = reinterpret_cast<AdcState*>(self->ApiInfo->State);

    state->isOpen |= (1 << channel);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Adc_CloseChannel(const TinyCLR_Adc_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<AdcState*>(self->ApiInfo->State);

    if (state->isOpen & (1 << channel)) {
        LPC24_Gpio_ClosePin(LPC24_Adc_GetPin(channel));

    }

    state->isOpen &= ~(1 << channel);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Adc_ReadChannel(const TinyCLR_Adc_Controller* self, uint32_t channel, int32_t& value) {
    uint32_t result = 0;

    if (channel >= LPC24_Adc_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    value = 0;

    // get the values
    for (auto i = 0; i < 5; i++) {
        LPC24_Time_Delay(nullptr, 5);

        result = ((*((uint32_t*)(ADC_DATA_BASE_ADDRESS)+channel)) >> 6) & 0x3FF;

        value += result;
    }

    value /= 5;

    // channel not available
    return TinyCLR_Result::Success;
}

uint32_t LPC24_Adc_GetResolutionInBits(const TinyCLR_Adc_Controller* self) {
    return 10;
}

int32_t LPC24_Adc_GetMinValue(const TinyCLR_Adc_Controller* self) {
    return 0;
}

int32_t LPC24_Adc_GetMaxValue(const TinyCLR_Adc_Controller* self) {
    return (1 << LPC24_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode LPC24_Adc_GetChannelMode(const TinyCLR_Adc_Controller* self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result LPC24_Adc_SetChannelMode(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool LPC24_Adc_IsChannelModeSupported(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void LPC24_Adc_Reset() {
    for (auto c = 0; c < TOTAL_ADC_CONTROLLERS; c++) {
        for (auto ch = 0; ch < LPC24_Adc_GetChannelCount(&adcControllers[c]); ch++) {
            LPC24_Adc_CloseChannel(&adcControllers[c], ch);
        }

        AdcState::isOpen = 0;
    }

    LPC24XX::SYSCON().PCONP &= ~(PCONP_PCAD);
}
