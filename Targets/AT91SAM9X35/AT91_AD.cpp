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

#define TOTAL_ADC_CONTROLLERS 1

static TinyCLR_Adc_Controller adcControllers[TOTAL_ADC_CONTROLLERS];
static TinyCLR_Api_Info adcApi[TOTAL_ADC_CONTROLLERS];

static const AT91_Gpio_Pin adcPins[] = AT91_ADC_PINS;

const char* adcApiNames[TOTAL_ADC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91.AdcController\\0"
};

struct AdcState {
    bool isOpened[SIZEOF_ARRAY(adcPins)];
};

static AdcState adcStates[TOTAL_ADC_CONTROLLERS];

void AT91_Adc_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_ADC_CONTROLLERS; i++) {
        adcControllers[i].ApiInfo = &adcApi[i];
        adcControllers[i].Acquire = &AT91_Adc_Acquire;
        adcControllers[i].Release = &AT91_Adc_Release;
        adcControllers[i].OpenChannel = &AT91_Adc_OpenChannel;
        adcControllers[i].CloseChannel = &AT91_Adc_CloseChannel;
        adcControllers[i].ReadChannel = &AT91_Adc_ReadChannel;
        adcControllers[i].SetChannelMode = &AT91_Adc_SetChannelMode;
        adcControllers[i].GetChannelMode = &AT91_Adc_GetChannelMode;
        adcControllers[i].IsChannelModeSupported = &AT91_Adc_IsChannelModeSupported;
        adcControllers[i].GetMinValue = &AT91_Adc_GetMinValue;
        adcControllers[i].GetMaxValue = &AT91_Adc_GetMaxValue;
        adcControllers[i].GetResolutionInBits = &AT91_Adc_GetResolutionInBits;
        adcControllers[i].GetChannelCount = &AT91_Adc_GetChannelCount;

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

int32_t AT91_Adc_GetPin(int32_t channel) {
    return  adcPins[channel].number;
}

AT91_Gpio_PeripheralSelection AT91_Adc_GetPeripheralSelection(int32_t channel) {
    return  adcPins[channel].peripheralSelection;
}

TinyCLR_Result AT91_Adc_Acquire(const TinyCLR_Adc_Controller *self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_ADC);
    pmc.EnablePeriphClock(AT91C_ID_PIOA_PIOB);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_Release(const TinyCLR_Adc_Controller *self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetPinForChannel(int32_t channel) {
    if ((uint32_t)channel >= AT91_Adc_GetChannelCount(&adcControllers[0]))
        return PIN_NONE;

    return AT91_Adc_GetPin(channel);
}

TinyCLR_Result AT91_Adc_OpenChannel(const TinyCLR_Adc_Controller *self, uint32_t channel) {
    if (channel >= AT91_Adc_GetChannelCount(self))
        return TinyCLR_Result::ArgumentOutOfRange;

    if (AT91_Adc_GetPin(channel) == PIN_NONE)
        return TinyCLR_Result::ArgumentInvalid;

    if (channel >= 0 && channel <= AT91_Adc_GetChannelCount(self) && AT91_GpioInternal_OpenPin(AT91_Adc_GetPin(channel))) {
        AT91_GpioInternal_ConfigurePin(AT91_Adc_GetPin(channel), AT91_Gpio_Direction::Input, AT91_Adc_GetPeripheralSelection(channel), AT91_Gpio_ResistorMode::Inactive);
    }
    else {
        return TinyCLR_Result::SharingViolation;
    }

    ADC_CONTROLLER_CHANNEL_ENABLE = (1 << channel);
    ADC_CONTROLLER_MODE_REGISTER = (63 << 8);
    ADC_CONTROLLER_TRIGGER_REGISTER = 6;

    auto state = reinterpret_cast<AdcState*>(self->ApiInfo->State);

    state->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_CloseChannel(const TinyCLR_Adc_Controller *self, uint32_t channel) {
    auto state = reinterpret_cast<AdcState*>(self->ApiInfo->State);

    if (state->isOpened[channel])
        AT91_GpioInternal_ClosePin(AT91_Adc_GetPin(channel));

    state->isOpened[channel] = false;

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

TinyCLR_Result AT91_Adc_ReadChannel(const TinyCLR_Adc_Controller *self, uint32_t channel, int32_t &value) {
    uint32_t arrayValuesToSort[MAX_AVERAGE_AMOUNT];
    uint32_t valueToStoreInArray;

    if (channel >= AT91_Adc_GetChannelCount(self))
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

uint32_t AT91_Adc_GetChannelCount(const TinyCLR_Adc_Controller *self) {
    return SIZEOF_ARRAY(adcPins);
}

uint32_t AT91_Adc_GetResolutionInBits(const TinyCLR_Adc_Controller *self) {
    return 10;
}

int32_t AT91_Adc_GetMinValue(const TinyCLR_Adc_Controller *self) {
    return 0;
}

int32_t AT91_Adc_GetMaxValue(const TinyCLR_Adc_Controller *self) {
    return (1 << AT91_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode AT91_Adc_GetChannelMode(const TinyCLR_Adc_Controller *self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result AT91_Adc_SetChannelMode(const TinyCLR_Adc_Controller *self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool AT91_Adc_IsChannelModeSupported(const TinyCLR_Adc_Controller *self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void AT91_Adc_Reset() {
    for (auto c = 0; c < TOTAL_ADC_CONTROLLERS; c++) {
        for (uint32_t ch = 0; ch < AT91_Adc_GetChannelCount(&adcControllers[c]); ch++) {
            AT91_Adc_CloseChannel(&adcControllers[c], ch);

            adcStates[c].isOpened[ch] = false;
        }
    }
}
