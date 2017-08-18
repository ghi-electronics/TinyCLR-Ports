// Copyright Microsoft Corporation
// Implementation for STM32F4: Copyright Oberon microsystems, Inc
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

#define STM32F4_AD_SAMPLE_TIME 2   // sample time = 28 cycles

#if STM32F4_ADC == 1
#define ADCx ADC1
#define RCC_APB2ENR_ADCxEN RCC_APB2ENR_ADC1EN
// ADC1 pins plus two internally connected channels thus the 0 for 'no pin'
// Vsense for temperature sensor @ ADC1_IN16
// Vrefubt for internal voltage reference (1.21V) @ ADC1_IN17
// to access the internal channels need to include '16' and/or '17' at the STM32F4_AD_CHANNELS array in 'platform_selector.h'
#define STM32F4_ADC_PINS {0,1,2,3,4,5,6,7,16,17,32,33,34,35,36,37,0,0}
#elif STM32F4_ADC == 3
#define ADCx ADC3
#define RCC_APB2ENR_ADCxEN RCC_APB2ENR_ADC3EN
#define STM32F4_ADC_PINS {0,1,2,3,86,87,88,89,90,83,32,33,34,35,84,85} // ADC3 pins
#else
#error wrong STM32F4_ADC value (1 or 3)
#endif

// Channels
static const uint8_t g_STM32F4_AD_Channel[] = STM32F4_AD_CHANNELS;
static const uint8_t g_STM32F4_AD_Pins[] = STM32F4_ADC_PINS;
#define STM32F4_AD_NUM SIZEOF_CONST_ARRAY(g_STM32F4_AD_Channel)  // number of channels

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

const TinyCLR_Api_Info* STM32F4_Adc_GetApi() {
    adcProvider.Parent = &adcApi;
    adcProvider.Index = 0;
    adcProvider.Acquire = &STM32F4_Adc_Acquire;
    adcProvider.Release = &STM32F4_Adc_Release;
    adcProvider.AcquireChannel = &STM32F4_Adc_AcquireChannel;
    adcProvider.ReleaseChannel = &STM32F4_Adc_ReleaseChannel;
    adcProvider.IsChannelModeSupported = &STM32F4_Adc_IsChannelModeSupported;
    adcProvider.ReadValue = &STM32F4_Adc_ReadValue;
    adcProvider.GetMinValue = &STM32F4_Adc_GetMinValue;
    adcProvider.GetMaxValue = &STM32F4_Adc_GetMaxValue;
    adcProvider.GetResolutionInBits = &STM32F4_Adc_GetResolutionInBits;
    adcProvider.GetChannelCount = &STM32F4_Adc_GetChannelCount;
    adcProvider.GetChannelMode = &STM32F4_Adc_GetChannelMode;
    adcProvider.SetChannelMode = &STM32F4_Adc_SetChannelMode;

    adcApi.Author = "GHI Electronics, LLC";
    adcApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.AdcProvider";
    adcApi.Type = TinyCLR_Api_Type::AdcProvider;
    adcApi.Version = 0;
    adcApi.Count = 1;
    adcApi.Implementation = &adcProvider;

    return &adcApi;
}

TinyCLR_Result STM32F4_Adc_Acquire(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Adc_Release(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Adc_GetPinForChannel(int32_t channel) {
    // return GPIO pin
    // for internally connected channels this is GPIO_PIN_NONE as these don't take any GPIO pins
    int chNum = g_STM32F4_AD_Channel[channel];

    for (int i = 0; i < STM32F4_AD_NUM; i++) {
        if (g_STM32F4_AD_Channel[i] == chNum) {
            return (int32_t)g_STM32F4_AD_Pins[chNum];
        }
    }

    // channel not available
    return GPIO_PIN_NONE;
}

TinyCLR_Result STM32F4_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    int chNum = g_STM32F4_AD_Channel[channel];

    if (!STM32F4_Gpio_OpenPin(STM32F4_Adc_GetPinForChannel(channel)))
        return TinyCLR_Result::SharingViolation;

    // init this channel if it's listed in the STM32F4_AD_CHANNELS array
    for (int i = 0; i < STM32F4_AD_NUM; i++) {
        if (g_STM32F4_AD_Channel[i] == chNum) {
            // valid channel
            if (!(RCC->APB2ENR & RCC_APB2ENR_ADCxEN)) { // not yet initialized
                RCC->APB2ENR |= RCC_APB2ENR_ADCxEN; // enable AD clock
                ADC->CCR = 0; // ADCCLK = PB2CLK / 2;
                ADCx->SQR1 = 0; // 1 conversion
                ADCx->CR1 = 0;
                ADCx->CR2 = ADC_CR2_ADON; // AD on
                ADCx->SMPR1 = 0x01249249 * STM32F4_AD_SAMPLE_TIME;
                ADCx->SMPR2 = 0x09249249 * STM32F4_AD_SAMPLE_TIME;
            }

            // set pin as analog input if channel is not one of the internally connected
            if (chNum <= 15) {
                STM32F4_Gpio_ConfigurePin(STM32F4_Adc_GetPinForChannel(channel), STM32F4_Gpio_PortMode::Analog, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);
                return TinyCLR_Result::Success;
            }
        }
    }

    // channel not available
    return TinyCLR_Result::ArgumentOutOfRange;
}

TinyCLR_Result STM32F4_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    int chNum = g_STM32F4_AD_Channel[channel];

    // free GPIO pin if this channel is listed in the STM32F4_AD_CHANNELS array
    // and if it's not one of the internally connected ones as these channels don't take any GPIO pins
    if (chNum <= 15)
        STM32F4_Gpio_ClosePin(STM32F4_Adc_GetPinForChannel(channel));

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value) {
    int chNum = g_STM32F4_AD_Channel[channel];

    // check if this channel is listed in the STM32F4_AD_CHANNELS array
    for (int i = 0; i < STM32F4_AD_NUM; i++) {
        if (g_STM32F4_AD_Channel[i] == chNum) {
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

int32_t STM32F4_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self) {
    return STM32F4_AD_NUM;
}

int32_t STM32F4_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self) {
    return 12;
}

int32_t STM32F4_Adc_GetMinValue(const TinyCLR_Adc_Provider* self) {
    return 0;
}

int32_t STM32F4_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self) {
    return (1 << STM32F4_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode STM32F4_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result STM32F4_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool STM32F4_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void STM32F4_Adc_Reset() {
    for (auto i = 0; i < STM32F4_AD_NUM; i++) {
        STM32F4_Adc_ReleaseChannel(&adcProvider, i);
    }
}
