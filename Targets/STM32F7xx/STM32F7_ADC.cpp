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

#include "STM32F7.h"

#define STM32F7_AD_SAMPLE_TIME 2   // sample time = 28 cycles
#define ADCx ADC1
#define RCC_APB2ENR_ADCxEN RCC_APB2ENR_ADC1EN
#define STM32F7_ADC_CHANNEL_NONE    0xFF
#define STM32F7_ADC_PINS { PIN(A, 0), PIN(A, 1), PIN(A, 2), PIN(A, 3), PIN(A, 4), PIN(A, 5), PIN(A, 6), PIN(A, 7), PIN(B, 0), PIN(B, 1), PIN(C, 0), PIN(C, 1), PIN(C, 2),PIN(C, 3), PIN(C, 4), PIN(C, 5), PIN_NONE, PIN_NONE}
#define STM32F7_AD_NUM SIZEOF_ARRAY(g_STM32F7_AD_Channel)

static const uint8_t g_STM32F7_AD_Channel[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 12, 13, 14, 15, 16, 17 };
static const uint32_t g_STM32F7_AD_Pins[] = STM32F7_ADC_PINS;

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

bool g_STM32F7_AD_IsOpened[STM32F7_AD_NUM];

const TinyCLR_Api_Info* STM32F7_Adc_GetApi() {
    adcProvider.Parent = &adcApi;
    adcProvider.Index = 0;
    adcProvider.Acquire = &STM32F7_Adc_Acquire;
    adcProvider.Release = &STM32F7_Adc_Release;
    adcProvider.AcquireChannel = &STM32F7_Adc_AcquireChannel;
    adcProvider.ReleaseChannel = &STM32F7_Adc_ReleaseChannel;
    adcProvider.ReadValue = &STM32F7_Adc_ReadValue;
    adcProvider.SetChannelMode = &STM32F7_Adc_SetChannelMode;
    adcProvider.GetChannelMode = &STM32F7_Adc_GetChannelMode;
    adcProvider.IsChannelModeSupported = &STM32F7_Adc_IsChannelModeSupported;
    adcProvider.GetMinValue = &STM32F7_Adc_GetMinValue;
    adcProvider.GetMaxValue = &STM32F7_Adc_GetMaxValue;
    adcProvider.GetResolutionInBits = &STM32F7_Adc_GetResolutionInBits;
    adcProvider.GetChannelCount = &STM32F7_Adc_GetChannelCount;

    adcApi.Author = "GHI Electronics, LLC";
    adcApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.AdcProvider";
    adcApi.Type = TinyCLR_Api_Type::AdcProvider;
    adcApi.Version = 0;
    adcApi.Count = 1;
    adcApi.Implementation = &adcProvider;

    return &adcApi;
}

TinyCLR_Result STM32F7_Adc_Acquire(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Adc_Release(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t STM32F7_Adc_GetPinForChannel(int32_t channel) {
    // return GPIO pin
    // for internally connected channels this is PIN_NONE as these don't take any GPIO pins
    int chNum = g_STM32F7_AD_Channel[channel];

    if (chNum >= STM32F7_AD_NUM)
        return PIN_NONE;

    for (int i = 0; i < STM32F7_AD_NUM; i++) {
        if (g_STM32F7_AD_Channel[i] == chNum) {
            return (int32_t)g_STM32F7_AD_Pins[chNum];
        }
    }

    // channel not available
    return PIN_NONE;
}

TinyCLR_Result STM32F7_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    int32_t chNum = g_STM32F7_AD_Channel[channel];

    if (chNum >= STM32F7_AD_NUM)
        return TinyCLR_Result::NotAvailable;

    if (!STM32F7_GpioInternal_OpenPin(STM32F7_Adc_GetPinForChannel(channel)))
        return TinyCLR_Result::SharingViolation;

    // init this channel if it's listed in the STM32F7_AD_CHANNELS array
    for (int i = 0; i < STM32F7_AD_NUM; i++) {
        if (g_STM32F7_AD_Channel[i] == chNum) {
            // valid channel
            if (!(RCC->APB2ENR & RCC_APB2ENR_ADCxEN)) { // not yet initialized
                RCC->APB2ENR |= RCC_APB2ENR_ADCxEN; // enable AD clock
                ADC->CCR = 0; // ADCCLK = PB2CLK / 2;
                ADCx->SQR1 = 0; // 1 conversion
                ADCx->CR1 = 0;
                ADCx->CR2 = ADC_CR2_ADON; // AD on
                ADCx->SMPR1 = 0x01249249 * STM32F7_AD_SAMPLE_TIME;
                ADCx->SMPR2 = 0x09249249 * STM32F7_AD_SAMPLE_TIME;
            }

            // set pin as analog input if channel is not one of the internally connected
            if (chNum <= 15) {
                STM32F7_GpioInternal_ConfigurePin(STM32F7_Adc_GetPinForChannel(channel), STM32F7_Gpio_PortMode::Analog, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, STM32F7_Gpio_AlternateFunction::AF0);

                g_STM32F7_AD_IsOpened[chNum] = true;

                return TinyCLR_Result::Success;
            }
        }
    }

    // channel not available
    return TinyCLR_Result::ArgumentOutOfRange;
}

TinyCLR_Result STM32F7_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    int chNum = g_STM32F7_AD_Channel[channel];

    // free GPIO pin if this channel is listed in the STM32F7_AD_CHANNELS array
    // and if it's not one of the internally connected ones as these channels don't take any GPIO pins
    if (chNum < STM32F7_AD_NUM && g_STM32F7_AD_IsOpened[chNum])
        STM32F7_GpioInternal_ClosePin(STM32F7_Adc_GetPinForChannel(channel));

    g_STM32F7_AD_IsOpened[chNum] = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value) {
    int chNum = g_STM32F7_AD_Channel[channel];

    // check if this channel is listed in the STM32F7_AD_CHANNELS array
    for (int i = 0; i < STM32F7_AD_NUM; i++) {
        if (g_STM32F7_AD_Channel[i] == chNum) {
            // valid channel
            int x = ADCx->DR; // clear EOC flag

            ADCx->SQR3 = chNum; // select channel

            // need to enable internal reference at ADC->CCR register to work with internally connected channels
            if (chNum == 16 || chNum == 17) {
                ADC->CCR |= ADC_CCR_TSVREFE; // Enable internal reference to work with temperature sensor and VREFINT channels
            }

            ADCx->CR2 |= ADC_CR2_SWSTART; // start AD
            while (!(ADCx->SR & ADC_SR_EOC)); // wait for completion

            // disable internally reference
            if (chNum == 16 || chNum == 17) {
                ADC->CCR &= ~ADC_CCR_TSVREFE;
            }

            value = ADCx->DR; // read result

            return TinyCLR_Result::Success;
        }
    }

    // channel not available
    return TinyCLR_Result::ArgumentOutOfRange;
}

int32_t STM32F7_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self) {
    return STM32F7_AD_NUM;
}

int32_t STM32F7_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self) {
    return 12;
}

int32_t STM32F7_Adc_GetMinValue(const TinyCLR_Adc_Provider* self) {
    return 0;
}

int32_t STM32F7_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self) {
    return (1 << STM32F7_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode STM32F7_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result STM32F7_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool STM32F7_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void STM32F7_Adc_Reset() {
    for (auto i = 0; i < STM32F7_AD_NUM; i++) {
        STM32F7_Adc_ReleaseChannel(&adcProvider, i);

        g_STM32F7_AD_IsOpened[i] = false;
    }
}