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


#define AD0_BASE 0x40034000

#define AD0CR (*(volatile unsigned *)0x40034000)
#define AD0CR_OFFSET 0x0
#define AD0CR_SEL_MASK 0xFF
#define AD0CR_SEL_BIT 0
#define AD0CR_CLKDIV_MASK 0xFF00
#define AD0CR_CLKDIV_BIT 8
#define AD0CR_BURST_MASK 0x10000
#define AD0CR_BURST 0x10000
#define AD0CR_BURST_BIT 16
#define AD0CR_CLKS_MASK 0xE0000
#define AD0CR_CLKS_BIT 17
#define AD0CR_PDN_MASK 0x200000
#define AD0CR_PDN 0x200000
#define AD0CR_PDN_BIT 21
#define AD0CR_START_MASK 0x7000000
#define AD0CR_START_BIT 24
#define AD0CR_EDGE_MASK 0x8000000
#define AD0CR_EDGE 0x8000000
#define AD0CR_EDGE_BIT 27

#define AD0GDR (*(volatile unsigned *)0x40034004)
#define AD0GDR_OFFSET 0x4
#define AD0GDR_RESULT_MASK 0xFFC0
#define AD0GDR_RESULT_BIT 6
#define AD0GDR_CHN_MASK 0x7000000
#define AD0GDR_CHN_BIT 24
#define AD0GDR_OVERUN_MASK 0x40000000
#define AD0GDR_OVERUN 0x40000000
#define AD0GDR_OVERUN_BIT 30
#define AD0GDR_DONE_MASK 0x80000000
#define AD0GDR_DONE 0x80000000
#define AD0GDR_DONE_BIT 31

#define AD0INTEN (*(volatile unsigned *)0x4003400C)
#define AD0INTEN_OFFSET 0xC
#define AD0INTEN_ADINTEN0_MASK 0x1
#define AD0INTEN_ADINTEN0 0x1
#define AD0INTEN_ADINTEN0_BIT 0
#define AD0INTEN_ADINTEN1_MASK 0x2
#define AD0INTEN_ADINTEN1 0x2
#define AD0INTEN_ADINTEN1_BIT 1
#define AD0INTEN_ADINTEN2_MASK 0x4
#define AD0INTEN_ADINTEN2 0x4
#define AD0INTEN_ADINTEN2_BIT 2
#define AD0INTEN_ADINTEN3_MASK 0x8
#define AD0INTEN_ADINTEN3 0x8
#define AD0INTEN_ADINTEN3_BIT 3
#define AD0INTEN_ADINTEN4_MASK 0x10
#define AD0INTEN_ADINTEN4 0x10
#define AD0INTEN_ADINTEN4_BIT 4
#define AD0INTEN_ADINTEN5_MASK 0x20
#define AD0INTEN_ADINTEN5 0x20
#define AD0INTEN_ADINTEN5_BIT 5
#define AD0INTEN_ADINTEN6_MASK 0x40
#define AD0INTEN_ADINTEN6 0x40
#define AD0INTEN_ADINTEN6_BIT 6
#define AD0INTEN_ADINTEN7_MASK 0x80
#define AD0INTEN_ADINTEN7 0x80
#define AD0INTEN_ADINTEN7_BIT 7
#define AD0INTEN_ADGINTEN_MASK 0x100
#define AD0INTEN_ADGINTEN 0x100
#define AD0INTEN_ADGINTEN_BIT 8

#define AD0DR0 (*(volatile unsigned *)0x40034010)
#define AD0DR0_OFFSET 0x10
#define AD0DR0_RESULT_MASK 0xFFC0
#define AD0DR0_RESULT_BIT 6
#define AD0DR0_OVERRUN_MASK 0x40000000
#define AD0DR0_OVERRUN 0x40000000
#define AD0DR0_OVERRUN_BIT 30
#define AD0DR0_DONE_MASK 0x80000000
#define AD0DR0_DONE 0x80000000
#define AD0DR0_DONE_BIT 31

#define AD0DR1 (*(volatile unsigned *)0x40034014)
#define AD0DR1_OFFSET 0x14
#define AD0DR1_RESULT_MASK 0xFFC0
#define AD0DR1_RESULT_BIT 6
#define AD0DR1_OVERRUN_MASK 0x40000000
#define AD0DR1_OVERRUN 0x40000000
#define AD0DR1_OVERRUN_BIT 30
#define AD0DR1_DONE_MASK 0x80000000
#define AD0DR1_DONE 0x80000000
#define AD0DR1_DONE_BIT 31

#define AD0DR2 (*(volatile unsigned *)0x40034018)
#define AD0DR2_OFFSET 0x18
#define AD0DR2_RESULT_MASK 0xFFC0
#define AD0DR2_RESULT_BIT 6
#define AD0DR2_OVERRUN_MASK 0x40000000
#define AD0DR2_OVERRUN 0x40000000
#define AD0DR2_OVERRUN_BIT 30
#define AD0DR2_DONE_MASK 0x80000000
#define AD0DR2_DONE 0x80000000
#define AD0DR2_DONE_BIT 31

#define AD0DR3 (*(volatile unsigned *)0x4003401C)
#define AD0DR3_OFFSET 0x1C
#define AD0DR3_RESULT_MASK 0xFFC0
#define AD0DR3_RESULT_BIT 6
#define AD0DR3_OVERRUN_MASK 0x40000000
#define AD0DR3_OVERRUN 0x40000000
#define AD0DR3_OVERRUN_BIT 30
#define AD0DR3_DONE_MASK 0x80000000
#define AD0DR3_DONE 0x80000000
#define AD0DR3_DONE_BIT 31

#define AD0DR4 (*(volatile unsigned *)0x40034020)
#define AD0DR4_OFFSET 0x20
#define AD0DR4_RESULT_MASK 0xFFC0
#define AD0DR4_RESULT_BIT 6
#define AD0DR4_OVERRUN_MASK 0x40000000
#define AD0DR4_OVERRUN 0x40000000
#define AD0DR4_OVERRUN_BIT 30
#define AD0DR4_DONE_MASK 0x80000000
#define AD0DR4_DONE 0x80000000
#define AD0DR4_DONE_BIT 31

#define AD0DR5 (*(volatile unsigned *)0x40034024)
#define AD0DR5_OFFSET 0x24
#define AD0DR5_RESULT_MASK 0xFFC0
#define AD0DR5_RESULT_BIT 6
#define AD0DR5_OVERRUN_MASK 0x40000000
#define AD0DR5_OVERRUN 0x40000000
#define AD0DR5_OVERRUN_BIT 30
#define AD0DR5_DONE_MASK 0x80000000
#define AD0DR5_DONE 0x80000000
#define AD0DR5_DONE_BIT 31

#define AD0DR6 (*(volatile unsigned *)0x40034028)
#define AD0DR6_OFFSET 0x28
#define AD0DR6_RESULT_MASK 0xFFC0
#define AD0DR6_RESULT_BIT 6
#define AD0DR6_OVERRUN_MASK 0x40000000
#define AD0DR6_OVERRUN 0x40000000
#define AD0DR6_OVERRUN_BIT 30
#define AD0DR6_DONE_MASK 0x80000000
#define AD0DR6_DONE 0x80000000
#define AD0DR6_DONE_BIT 31

#define AD0DR7 (*(volatile unsigned *)0x4003402C)
#define AD0DR7_OFFSET 0x2C
#define AD0DR7_RESULT_MASK 0xFFC0
#define AD0DR7_RESULT_BIT 6
#define AD0DR7_OVERRUN_MASK 0x40000000
#define AD0DR7_OVERRUN 0x40000000
#define AD0DR7_OVERRUN_BIT 30
#define AD0DR7_DONE_MASK 0x80000000
#define AD0DR7_DONE 0x80000000
#define AD0DR7_DONE_BIT 31

#define AD0STAT (*(volatile unsigned *)0x40034030)
#define AD0STAT_OFFSET 0x30
#define AD0STAT_DONE0_MASK 0x1
#define AD0STAT_DONE0 0x1
#define AD0STAT_DONE0_BIT 0
#define AD0STAT_DONE1_MASK 0x2
#define AD0STAT_DONE1 0x2
#define AD0STAT_DONE1_BIT 1
#define AD0STAT_DONE2_MASK 0x4
#define AD0STAT_DONE2 0x4
#define AD0STAT_DONE2_BIT 2
#define AD0STAT_DONE3_MASK 0x8
#define AD0STAT_DONE3 0x8
#define AD0STAT_DONE3_BIT 3
#define AD0STAT_DONE4_MASK 0x10
#define AD0STAT_DONE4 0x10
#define AD0STAT_DONE4_BIT 4
#define AD0STAT_DONE5_MASK 0x20
#define AD0STAT_DONE5 0x20
#define AD0STAT_DONE5_BIT 5
#define AD0STAT_DONE6_MASK 0x40
#define AD0STAT_DONE6 0x40
#define AD0STAT_DONE6_BIT 6
#define AD0STAT_DONE7_MASK 0x80
#define AD0STAT_DONE7 0x80
#define AD0STAT_DONE7_BIT 7
#define AD0STAT_OVERRUN0_MASK 0x100
#define AD0STAT_OVERRUN0 0x100
#define AD0STAT_OVERRUN0_BIT 8
#define AD0STAT_OVERRUN1_MASK 0x200
#define AD0STAT_OVERRUN1 0x200
#define AD0STAT_OVERRUN1_BIT 9
#define AD0STAT_OVERRUN2_MASK 0x400
#define AD0STAT_OVERRUN2 0x400
#define AD0STAT_OVERRUN2_BIT 10
#define AD0STAT_OVERRUN3_MASK 0x800
#define AD0STAT_OVERRUN3 0x800
#define AD0STAT_OVERRUN3_BIT 11
#define AD0STAT_OVERRUN4_MASK 0x1000
#define AD0STAT_OVERRUN4 0x1000
#define AD0STAT_OVERRUN4_BIT 12
#define AD0STAT_OVERRUN5_MASK 0x2000
#define AD0STAT_OVERRUN5 0x2000
#define AD0STAT_OVERRUN5_BIT 13
#define AD0STAT_OVERRUN6_MASK 0x4000
#define AD0STAT_OVERRUN6 0x4000
#define AD0STAT_OVERRUN6_BIT 14
#define AD0STAT_OVERRUN7_MASK 0x8000
#define AD0STAT_OVERRUN7 0x8000
#define AD0STAT_OVERRUN7_BIT 15
#define AD0STAT_ADINT_MASK 0x10000
#define AD0STAT_ADINT 0x10000
#define AD0STAT_ADINT_BIT 16

#define LPC17xx_ADC_DataRegisterShiftBits	4
#define LPC17xx_ADC_BitRegisterMask			0xFFF

struct LPC17xx_ADC {
    static const uint32_t c_ADC_Base = 0x40034010;

};

static const int32_t g_LPC17_Adc_Pins[] = LPC17_ADC_PINS;
static const LPC17_Gpio_PinFunction g_LPC17_Adc_altMode[] = LPC17_ADC_ALT_MODE;

static TinyCLR_Adc_Provider adcProvider;
static TinyCLR_Api_Info adcApi;

const TinyCLR_Api_Info* LPC17_Adc_GetApi() {
    adcProvider.Parent = &adcApi;
    adcProvider.Index = 0;
    adcProvider.Acquire = &LPC17_Adc_Acquire;
    adcProvider.Release = &LPC17_Adc_Release;
    adcProvider.AcquireChannel = &LPC17_Adc_AcquireChannel;
    adcProvider.ReleaseChannel = &LPC17_Adc_ReleaseChannel;
    adcProvider.IsChannelModeSupported = &LPC17_Adc_IsChannelModeSupported;
    adcProvider.ReadValue = &LPC17_Adc_ReadValue;
    adcProvider.GetMinValue = &LPC17_Adc_GetMinValue;
    adcProvider.GetMaxValue = &LPC17_Adc_GetMaxValue;
    adcProvider.GetResolutionInBits = &LPC17_Adc_GetResolutionInBits;
    adcProvider.GetChannelCount = &LPC17_Adc_GetChannelCount;
    adcProvider.GetChannelMode = &LPC17_Adc_GetChannelMode;
    adcProvider.SetChannelMode = &LPC17_Adc_SetChannelMode;

    adcApi.Author = "GHI Electronics, LLC";
    adcApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.AdcProvider";
    adcApi.Type = TinyCLR_Api_Type::AdcProvider;
    adcApi.Version = 0;
    adcApi.Count = 1;
    adcApi.Implementation = &adcProvider;

    return &adcApi;
}

TinyCLR_Result LPC17_Adc_Acquire(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Adc_Release(const TinyCLR_Adc_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    if (channel >= TOTAL_ADC_CONTROLLERS)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC17_Gpio_OpenPin(g_LPC17_Adc_Pins[channel]))
        return  TinyCLR_Result::SharingViolation;

    LPC_SC->PCONP |= PCONP_PCAD; // To enable power on ADC  Possibly add a check to see if power is enabled before setting and for the ability to check if any ADC are active in uninitialize

    LPC17_Gpio_ConfigurePin(g_LPC17_Adc_Pins[channel], LPC17_Gpio_Direction::Input, g_LPC17_Adc_altMode[channel], LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    AD0CR |= (1 << channel) | // Selects this channel in the register to initialize
        ((5 - 1) << 8) | // Divide the clock (60 MHz) by 5 (60 / (4 + 1) = 12) <-- must be < 12.5
        (1 << 16) | // Burst Mode set
        (1 << 21); // Set convertion operation to opperational

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel) {
    LPC17_Gpio_ClosePin(g_LPC17_Adc_Pins[channel]);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value) {
    uint32_t result = 0;

    if (channel >= TOTAL_ADC_CONTROLLERS)
        return TinyCLR_Result::ArgumentOutOfRange;

    value = 0;

    // get the values
    for (auto i = 0; i < 5; i++) {
        LPC17_Time_Delay(nullptr, 5);

        result = ((*((uint32_t*)(LPC17xx_ADC::c_ADC_Base) + channel)) >> LPC17xx_ADC_DataRegisterShiftBits) & LPC17xx_ADC_BitRegisterMask;

        value += result;
    }

    value /= 5;

    // channel not available
    return TinyCLR_Result::Success;
}

int32_t LPC17_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self) {
    return TOTAL_ADC_CONTROLLERS;
}

int32_t LPC17_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self) {
    return 12;
}

int32_t LPC17_Adc_GetMinValue(const TinyCLR_Adc_Provider* self) {
    return 0;
}

int32_t LPC17_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self) {
    return (1 << LPC17_Adc_GetResolutionInBits(self)) - 1;
}

TinyCLR_Adc_ChannelMode LPC17_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self) {
    return TinyCLR_Adc_ChannelMode::SingleEnded;
}

TinyCLR_Result LPC17_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded ? TinyCLR_Result::Success : TinyCLR_Result::NotSupported;
}

bool LPC17_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode) {
    return mode == TinyCLR_Adc_ChannelMode::SingleEnded;
}

void LPC17_Adc_Reset() {
    LPC_SC->PCONP &= ~PCONP_PCAD;
    for (auto ch = 0; ch < TOTAL_ADC_CONTROLLERS; ch++) {
        LPC17_Adc_ReleaseChannel(&adcProvider, ch);
    }

}
