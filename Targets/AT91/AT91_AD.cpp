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

#define MAX_AVERAGE_AMOUNT 5

//registers
#define ADC_CONTROLLER_REGISTER_BASE 0xF804C000
#define ADC_CONTROLLER_CHANNEL_DATA_REGISTER_BASE (ADC_CONTROLLER_REGISTER_BASE + 0x50)               // ADC_CDR0 ~ ADC_CDR11
#define ADC_CONTROLLER_CONTROL_REGISTER (*(volatile uint32_t *)(ADC_CONTROLLER_REGISTER_BASE + 0x00)) // ADC_CR
#define ADC_CONTROLLER_MODE_REGISTER (*(volatile uint32_t *)(ADC_CONTROLLER_REGISTER_BASE + 0x04))    // ADC_MR
#define ADC_CONTROLLER_CHANNEL_ENABLE (*(volatile uint32_t *)(ADC_CONTROLLER_REGISTER_BASE + 0x10))   // ADC_CHER
#define ADC_CONTROLLER_TRIGGER_REGISTER (*(volatile uint32_t *)(ADC_CONTROLLER_REGISTER_BASE + 0xC0)) // ADC_TRGR

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

const TinyCLR_Api_Info *AT91_Adc_GetApi() {
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

TinyCLR_Result AT91_Adc_Acquire(const TinyCLR_Adc_Provider *self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_ADC);
    pmc.EnablePeriphClock(AT91C_ID_PIOA_PIOB);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_Release(const TinyCLR_Adc_Provider *self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetPinForChannel(int32_t channel) {
    if ((uint32_t)channel >= AT91_Adc_GetControllerCount())
        return PIN_NONE;

    return AT91_Adc_GetPin(channel);
}

TinyCLR_Result AT91_Adc_AcquireChannel(const TinyCLR_Adc_Provider *self, int32_t channel) {
    if (channel >= AT91_Adc_GetControllerCount())
        return TinyCLR_Result::ArgumentOutOfRange;

    if (AT91_Adc_GetPin(channel) == PIN_NONE)
        return TinyCLR_Result::ArgumentInvalid;

    if (channel >= 0 && channel <= AT91_Adc_GetControllerCount() && AT91_Gpio_OpenPin(AT91_Adc_GetPin(channel))) {
        AT91_Gpio_ConfigurePin(AT91_Adc_GetPin(channel), AT91_Gpio_Direction::Input, AT91_Adc_GetPeripheralSelection(channel), AT91_Gpio_ResistorMode::Inactive);
    }
    else {
        return TinyCLR_Result::SharingViolation;
    }

    ADC_CONTROLLER_CHANNEL_ENABLE = (1 << channel);
    ADC_CONTROLLER_MODE_REGISTER = (63 << 8);
    ADC_CONTROLLER_TRIGGER_REGISTER = 6;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_ReleaseChannel(const TinyCLR_Adc_Provider *self, int32_t channel) {
    AT91_Gpio_ClosePin(AT91_Adc_GetPin(channel));

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_AverageMiddleThreeValues(uint32_t *arrayToSort) {
    uint32_t valueToReturn;

    int32_t middleArrayPossition = MAX_AVERAGE_AMOUNT / 2;
    valueToReturn = arrayToSort[middleArrayPossition - 1];
    valueToReturn += arrayToSort[middleArrayPossition];
    valueToReturn += arrayToSort[middleArrayPossition + 1];

    return (valueToReturn / 3);
}

void AT91_Adc_SwapArrayValues_Ascending(uint32_t *arrayToSort, int32_t smallNumberArrayPosition, int32_t largeNumberArrayPosition) {
    uint32_t smallNumber = arrayToSort[smallNumberArrayPosition];
    uint32_t largeNumber = arrayToSort[largeNumberArrayPosition];
    arrayToSort[smallNumberArrayPosition] = largeNumber;
    arrayToSort[largeNumberArrayPosition] = smallNumber;
}

void AT91_Adc_SortArray(uint32_t *arrayToSort) {
    bool isArraySorted = false;
    int32_t arrayCount;

    while (!isArraySorted) {
        arrayCount = 0;

        for (int32_t x = 0; x < (MAX_AVERAGE_AMOUNT - 1); x++) {
            if (arrayToSort[x] < arrayToSort[x + 1])
                AT91_Adc_SwapArrayValues_Ascending(arrayToSort, x, (x + 1));
            else
                arrayCount++;
        }

        if (arrayCount == (MAX_AVERAGE_AMOUNT - 1))
            isArraySorted = true;
    }
}

TinyCLR_Result AT91_Adc_ReadValue(const TinyCLR_Adc_Provider *self, int32_t channel, int32_t &value) {
    uint32_t arrayValuesToSort[MAX_AVERAGE_AMOUNT];
    uint32_t valueToStoreInArray;

    if (channel >= AT91_Adc_GetControllerCount())
        return TinyCLR_Result::ArgumentOutOfRange;

    value = 0;

    ADC_CONTROLLER_CONTROL_REGISTER = (1 << 1); // Starts the convertion process.
    // get the values
    for (auto i = 0; i < MAX_AVERAGE_AMOUNT; i++) {
        AT91_Time_Delay(nullptr, 5);
        valueToStoreInArray = *((volatile uint32_t *)(ADC_CONTROLLER_CHANNEL_DATA_REGISTER_BASE + (channel * 0x4)));
        arrayValuesToSort[i] = valueToStoreInArray;
    }

    AT91_Adc_SortArray(arrayValuesToSort);

    value = AT91_Adc_AverageMiddleThreeValues(arrayValuesToSort);

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetChannelCount(const TinyCLR_Adc_Provider *self) {
    return AT91_Adc_GetControllerCount();
}

int32_t AT91_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider *self) {
    return 10;
}

int32_t AT91_Adc_GetMinValue(const TinyCLR_Adc_Provider *self) {
    return 0;
}

int32_t AT91_Adc_GetMaxValue(const TinyCLR_Adc_Provider *self) {
    return (1 << AT91_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode AT91_Adc_GetChannelMode(const TinyCLR_Adc_Provider *self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result AT91_Adc_SetChannelMode(const TinyCLR_Adc_Provider *self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool AT91_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider *self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void AT91_Adc_Reset() {
    for (auto ch = 0; ch < AT91_Adc_GetControllerCount(); ch++) {
        AT91_Adc_ReleaseChannel(&adcProvider, ch);
    }
}
