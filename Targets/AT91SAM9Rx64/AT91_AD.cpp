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
#define TOUCHSCREEN_ADC_CONTROLLER_CHANNEL_SELECT                          (*reinterpret_cast<volatile unsigned long *>(0xFFFD0010)) // TSADCC_CHER
#define TOUCHSCREEN_ADC_CONTROLLER_CHANNEL_DISABLE_REGISTER                (*reinterpret_cast<volatile unsigned long *>(0xFFFD0014)) // TSADCC_MR
#define TOUCHSCREEN_ADC_CONTROLLER_MODE_REGISTER                           (*reinterpret_cast<volatile unsigned long *>(0xFFFD0004)) // TSADCC_MR
#define TOUCHSCREEN_ADC_CONTROLLER_TRIGGER_REGISTER                        (*reinterpret_cast<volatile unsigned long *>(0xFFFD0008)) // TSADCC_TRGR
#define TOUCHSCREEN_ADC_CONTROLLER_CHANNEL_DATA_REGISTER_BASE_ADDRESS      0xFFFD0030 // TSADCC_TRGR

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

const TinyCLR_Api_Info *AT91_Adc_GetApi() {
    adcProvider.Parent = &adcApi;
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
    adcProvider.GetControllerCount = &AT91_Adc_GetControllerCount;

    adcApi.Author = "GHI Electronics, LLC";
    adcApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.AdcProvider";
    adcApi.Type = TinyCLR_Api_Type::AdcProvider;
    adcApi.Version = 0;
    adcApi.Implementation = &adcProvider;

    return &adcApi;
}

static const AT91_Gpio_Pin g_at91_adc_pins[] = AT91_ADC_PINS;

bool g_at91_adc_isOpened[SIZEOF_ARRAY(g_at91_adc_pins)];

int32_t AT91_Adc_GetChannelCount() {
    return SIZEOF_ARRAY(g_at91_adc_pins);
}

int32_t AT91_Adc_GetPin(int32_t channel) {
    return  g_at91_adc_pins[channel].number;
}

AT91_Gpio_PeripheralSelection AT91_Adc_GetPeripheralSelection(int32_t channel) {
    return  g_at91_adc_pins[channel].peripheralSelection;
}

TinyCLR_Result AT91_Adc_Acquire(const TinyCLR_Adc_Provider *self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_TSADCC);
    pmc.EnablePeriphClock(AT91C_ID_PIOA);
    pmc.EnablePeriphClock(AT91C_ID_PIOB);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_Release(const TinyCLR_Adc_Provider *self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetPinForChannel(int32_t channel) {
    if ((uint32_t)channel >= AT91_Adc_GetChannelCount())
        return PIN_NONE;

    return AT91_Adc_GetPin(channel);
}

TinyCLR_Result AT91_Adc_AcquireChannel(const TinyCLR_Adc_Provider *self, int32_t controller, int32_t channel) {
    if (channel >= AT91_Adc_GetChannelCount())
        return TinyCLR_Result::ArgumentOutOfRange;

    if (AT91_Adc_GetPin(channel) == PIN_NONE)
        return TinyCLR_Result::ArgumentInvalid;

    if (channel >= 0 && channel <= AT91_Adc_GetChannelCount() && AT91_Gpio_OpenPin(AT91_Adc_GetPin(channel))) {
        AT91_Gpio_ConfigurePin(AT91_Adc_GetPin(channel), AT91_Gpio_Direction::Input, AT91_Adc_GetPeripheralSelection(channel), AT91_Gpio_ResistorMode::Inactive);
    }
    else {
        return TinyCLR_Result::SharingViolation;
    }

    TOUCHSCREEN_ADC_CONTROLLER_CHANNEL_SELECT = (1 << channel);
    TOUCHSCREEN_ADC_CONTROLLER_MODE_REGISTER = (1 << 6) | (63 << 8) | (0xf << 24);
    TOUCHSCREEN_ADC_CONTROLLER_TRIGGER_REGISTER = 6;

    g_at91_adc_isOpened[channel] = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Adc_ReleaseChannel(const TinyCLR_Adc_Provider *self, int32_t controller, int32_t channel) {
    if (g_at91_adc_isOpened[channel])
        AT91_Gpio_ClosePin(AT91_Adc_GetPin(channel));

    g_at91_adc_isOpened[channel] = false;

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

TinyCLR_Result AT91_Adc_ReadValue(const TinyCLR_Adc_Provider *self, int32_t controller, int32_t channel, int32_t &value) {
    uint32_t arrayValuesToSort[MAX_AVERAGE_AMOUNT];
    uint32_t valueToStoreInArray;

    if (channel >= AT91_Adc_GetChannelCount())
        return TinyCLR_Result::ArgumentOutOfRange;

    value = 0;

    // get the values
    for (auto i = 0; i < MAX_AVERAGE_AMOUNT; i++) {
        AT91_Time_Delay(nullptr, 5);
        valueToStoreInArray = *(reinterpret_cast<volatile uint32_t *>(TOUCHSCREEN_ADC_CONTROLLER_CHANNEL_DATA_REGISTER_BASE_ADDRESS + (channel * 0x4)));
        arrayValuesToSort[i] = valueToStoreInArray;
    }

    AT91_Adc_SortArray(arrayValuesToSort);

    value = AT91_Adc_AverageMiddleThreeValues(arrayValuesToSort);

    return TinyCLR_Result::Success;
}

int32_t AT91_Adc_GetChannelCount(const TinyCLR_Adc_Provider *self, int32_t controller) {
    return AT91_Adc_GetChannelCount();
}

int32_t AT91_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider *self, int32_t controller) {
    return 10;
}

int32_t AT91_Adc_GetMinValue(const TinyCLR_Adc_Provider *self, int32_t controller) {
    return 0;
}

int32_t AT91_Adc_GetMaxValue(const TinyCLR_Adc_Provider *self, int32_t controller) {
    return (1 << AT91_Adc_GetResolutionInBits(self, 0)) - 1;
}

TinyCLR_Adc_ChannelMode AT91_Adc_GetChannelMode(const TinyCLR_Adc_Provider *self, int32_t controller) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result AT91_Adc_SetChannelMode(const TinyCLR_Adc_Provider *self, int32_t controller, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool AT91_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider *self, int32_t controller, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void AT91_Adc_Reset() {
    for (auto ch = 0; ch < AT91_Adc_GetChannelCount(); ch++) {
        AT91_Adc_ReleaseChannel(&adcProvider, 0, ch);

        g_at91_adc_isOpened[ch] = false;
    }
}

TinyCLR_Result AT91_Adc_GetControllerCount(const TinyCLR_Adc_Provider* self, int32_t& count) {
    count = 1;

    return TinyCLR_Result::Success;
}