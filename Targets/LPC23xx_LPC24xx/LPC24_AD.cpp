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

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

uint32_t g_lpc24_adc_isOpened;

const TinyCLR_Api_Info* LPC24_Adc_GetApi() {
    adcProvider.Parent = &adcApi;
    adcProvider.Acquire = &LPC24_Adc_Acquire;
    adcProvider.Release = &LPC24_Adc_Release;
    adcProvider.AcquireChannel = &LPC24_Adc_AcquireChannel;
    adcProvider.ReleaseChannel = &LPC24_Adc_ReleaseChannel;
    adcProvider.IsChannelModeSupported = &LPC24_Adc_IsChannelModeSupported;
    adcProvider.ReadValue = &LPC24_Adc_ReadValue;
    adcProvider.GetMinValue = &LPC24_Adc_GetMinValue;
    adcProvider.GetMaxValue = &LPC24_Adc_GetMaxValue;
    adcProvider.GetResolutionInBits = &LPC24_Adc_GetResolutionInBits;
    adcProvider.GetChannelCount = &LPC24_Adc_GetControllerCount;
    adcProvider.GetChannelMode = &LPC24_Adc_GetChannelMode;
    adcProvider.SetChannelMode = &LPC24_Adc_SetChannelMode;

    adcApi.Author = "GHI Electronics, LLC";
    adcApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.AdcProvider";
    adcApi.Type = TinyCLR_Api_Type::AdcProvider;
    adcApi.Version = 0;
    adcApi.Implementation = &adcProvider;

    return &adcApi;
}

TinyCLR_Result LPC24_Adc_Acquire(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Adc_Release(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t LPC24_Adc_GetPinForChannel(int32_t channel) {
    if ((uint32_t)channel >= LPC24_Adc_GetControllerCount())
        return PIN_NONE;

    return LPC24_Adc_GetPin(channel);
}

TinyCLR_Result LPC24_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    if (channel >= LPC24_Adc_GetControllerCount())
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

    g_lpc24_adc_isOpened |= (1 << channel);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    if (g_lpc24_adc_isOpened & (1 << channel)) {
        LPC24_Gpio_ClosePin(LPC24_Adc_GetPin(channel));

    }

    g_lpc24_adc_isOpened &= ~(1 << channel);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value) {
    uint32_t result = 0;

    if (channel >= LPC24_Adc_GetControllerCount())
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

int32_t LPC24_Adc_GetControllerCount(const TinyCLR_Adc_Provider* self) {
    return LPC24_Adc_GetControllerCount();
}

int32_t LPC24_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self) {
    return 10;
}

int32_t LPC24_Adc_GetMinValue(const TinyCLR_Adc_Provider* self) {
    return 0;
}

int32_t LPC24_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self) {
    return (1 << LPC24_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode LPC24_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result LPC24_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool LPC24_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void LPC24_Adc_Reset() {
    for (auto ch = 0; ch < LPC24_Adc_GetControllerCount(); ch++) {
        LPC24_Adc_ReleaseChannel(&adcProvider, ch);
    }

    g_lpc24_adc_isOpened = 0;

    LPC24XX::SYSCON().PCONP &= ~(PCONP_PCAD);
}
