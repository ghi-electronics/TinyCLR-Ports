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

#if STM32F7_APB1_CLOCK_HZ == STM32F7_AHB_CLOCK_HZ
#define PWM1_CLK_HZ (STM32F7_APB1_CLOCK_HZ)
#else
#define PWM1_CLK_HZ (STM32F7_APB1_CLOCK_HZ * 2)
#endif
#define PWM1_CLK_MHZ (PWM1_CLK_HZ / 1000000)

#if STM32F7_APB2_CLOCK_HZ == STM32F7_AHB_CLOCK_HZ
#define PWM2_CLK_HZ (STM32F7_APB2_CLOCK_HZ)
#else
#define PWM2_CLK_HZ (STM32F7_APB2_CLOCK_HZ * 2)
#endif
#define PWM2_CLK_MHZ (PWM2_CLK_HZ / 1000000)

#if PWM2_CLK_MHZ > PWM1_CLK_MHZ
#define PWM_MAX_CLK_MHZ PWM2_CLK_MHZ
#else
#define PWM_MAX_CLK_MHZ PWM1_CLK_MHZ
#endif

#define PWM_MILLISECONDS  1000
#define PWM_MICROSECONDS  1000000
#define PWM_NANOSECONDS   1000000000
#define PWM_PER_CONTROLLER 4

#define STM32F7_MIN_PWM_FREQUENCY 1

typedef  TIM_TypeDef* ptr_TIM_TypeDef;

struct PwmController {
    ptr_TIM_TypeDef     timerdef;
    STM32F7_Gpio_Pin         gpioPin[PWM_PER_CONTROLLER];

    bool                invert[PWM_PER_CONTROLLER];
    bool                isOpened[PWM_PER_CONTROLLER];

    double              actualFreq;
    double              theoryFreq;
    double              dutyCycle[PWM_PER_CONTROLLER];

    uint32_t            period;
    uint32_t            presc;
    uint32_t            timer;

};

void STM32F7_Pwm_ResetController(int32_t controller);
STM32F7_Gpio_Pin* STM32F7_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin);

static STM32F7_Gpio_Pin pwmPins[][PWM_PER_CONTROLLER] = STM32F7_PWM_PINS;

static const int TOTAL_PWM_CONTROLLER = SIZEOF_ARRAY(pwmPins);

static PwmController g_PwmController[TOTAL_PWM_CONTROLLER];

static TinyCLR_Pwm_Provider pwmProviders;
static TinyCLR_Api_Info pwmApi;

const TinyCLR_Api_Info* STM32F7_Pwm_GetApi() {
    pwmProviders.Parent = &pwmApi;
    pwmProviders.Acquire = &STM32F7_Pwm_Acquire;
    pwmProviders.Release = &STM32F7_Pwm_Release;
    pwmProviders.AcquirePin = &STM32F7_Pwm_AcquirePin;
    pwmProviders.ReleasePin = &STM32F7_Pwm_ReleasePin;
    pwmProviders.EnablePin = &STM32F7_Pwm_EnablePin;
    pwmProviders.DisablePin = &STM32F7_Pwm_DisablePin;
    pwmProviders.SetPulseParameters = &STM32F7_Pwm_SetPulseParameters;
    pwmProviders.SetDesiredFrequency = &STM32F7_Pwm_SetDesiredFrequency;
    pwmProviders.GetMinFrequency = &STM32F7_Pwm_GetMinFrequency;
    pwmProviders.GetMaxFrequency = &STM32F7_Pwm_GetMaxFrequency;
    pwmProviders.GetPinCount = &STM32F7_Pwm_GetPinCount;

    pwmApi.Author = "GHI Electronics, LLC";
    pwmApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.PwmProvider";
    pwmApi.Type = TinyCLR_Api_Type::PwmProvider;
    pwmApi.Version = 0;
    pwmApi.Implementation = &pwmProviders;

    return &pwmApi;
}

//--//

TinyCLR_Result STM32F7_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin) {
    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    auto actualPin = STM32F7_Pwm_GetGpioPinForChannel(self, controller, pin);

    if (!STM32F7_GpioInternal_OpenPin(actualPin->number))
        return TinyCLR_Result::SharingViolation;

    // relevant RCC register & bit
    __IO uint32_t* enReg = &RCC->APB1ENR;
    if ((uint32_t)treg & 0x10000) enReg = &RCC->APB2ENR;
    int enBit = 1 << (((uint32_t)treg >> 10) & 0x1F);

    if (!(*enReg & enBit)) { // not yet initialized
        *enReg |= enBit; // enable timer clock
        treg->CR1 = TIM_CR1_URS | TIM_CR1_ARPE; // double buffered update
        treg->EGR = TIM_EGR_UG; // enforce first update
        if (g_PwmController[controller].timer == 1 || g_PwmController[controller].timer == 8) {
            treg->BDTR |= TIM_BDTR_MOE; // main output enable (timer 1 & 8 only)
        }
    }

    *(__IO uint16_t*)&((uint32_t*)&treg->CCR1)[pin] = 0; // reset compare register

    // enable PWM channel
    uint32_t mode = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // PWM1 mode, double buffered
    if (pin & 1) mode <<= 8; // 1 or 3
    __IO uint32_t* reg = &treg->CCMR1;
    if (pin & 2) reg = &treg->CCMR2; // 2 or 3
    *reg |= mode;

    g_PwmController[controller].isOpened[pin] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin) {
    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    auto actualPin = STM32F7_Pwm_GetGpioPinForChannel(self, controller, pin);

    uint32_t mask = 0xFF; // disable PWM channel
    if (pin & 1) mask = 0xFF00; // 1 or 3
    __IO uint32_t* reg = &treg->CCMR1;
    if (pin & 2) reg = &treg->CCMR2; // 2 or 3
    *reg &= ~mask;

    if ((treg->CCMR1 | treg->CCMR2) == 0) { // no channel active
        __IO uint32_t* enReg = &RCC->APB1ENR;
        if ((uint32_t)treg & 0x10000) enReg = &RCC->APB2ENR;
        int enBit = 1 << (((uint32_t)treg >> 10) & 0x1F);
        *enReg &= ~enBit; // disable timer clock
    }

    STM32F7_GpioInternal_ClosePin(actualPin->number);

    g_PwmController[controller].isOpened[pin] = false;

    return TinyCLR_Result::Success;
}

void STM32F7_Pwm_GetScaleFactor(double frequency, uint32_t& period, uint32_t& scale) {
    if (frequency >= 1000.0) {
        period = (uint32_t)(((uint32_t)PWM_NANOSECONDS / frequency) + 0.5);
        scale = PWM_NANOSECONDS;
    }
    else if (frequency >= 1.0) {
        period = (uint32_t)(((uint32_t)PWM_MICROSECONDS / frequency) + 0.5);
        scale = PWM_MICROSECONDS;
    }
    else {
        period = (uint32_t)(((uint32_t)PWM_MILLISECONDS / frequency) + 0.5);
        scale = PWM_MILLISECONDS;
    }
}

double STM32F7_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self, int32_t controller) {
    uint32_t period = 0;
    uint32_t scale = 0;

    double freq = g_PwmController[controller].theoryFreq;

    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    STM32F7_Pwm_GetScaleFactor(freq, period, scale);

    uint32_t p = period, s = scale;

    // set pre, p, & d such that:
    // pre * p = PWM_CLK * period / scale
    // pre * d = PWM_CLK * duration / scale

    uint32_t clk = PWM1_CLK_HZ;

    if ((uint32_t)treg & 0x10000) clk = PWM2_CLK_HZ; // APB2

    uint32_t pre = clk / s; // prescaler

    if (pre == 0) { // s > PWM_CLK
        uint32_t sm = s / 1000000; // scale in MHz
        clk = PWM1_CLK_MHZ;      // clock in MHz
        if ((uint32_t)treg & 0x10000) clk = PWM2_CLK_MHZ; // APB2
        if (p > 0xFFFFFFFF / PWM_MAX_CLK_MHZ) { // avoid overflow
            pre = clk;
            p /= sm;
        }
        else {
            pre = 1;
            p = p * clk / sm;
        }
    }
    else {
        while (pre > 0x10000) { // prescaler too large
            if (p >= 0x80000000) return 0;
            pre >>= 1;
            p <<= 1;
        }
    }

    if (g_PwmController[controller].timer != 2 && g_PwmController[controller].timer != 5) { // 16 bit timer
        while (p >= 0x10000) { // period too large
            if (pre > 0x8000) return 0;
            pre <<= 1;
            p >>= 1;
        }
    }

    clk = PWM1_CLK_HZ;

    if ((uint32_t)treg & 0x10000)
        clk = PWM2_CLK_HZ; // APB2

    if (p == 0 || pre == 0)
        return 0;

    uint32_t period2 = scale / ((clk / pre) / p);
    double actualFreq = (double)(scale / period2);

    while (period2 < period || actualFreq > freq) {
        p++;

        period2 = scale / ((clk / pre) / p);

        if (period2 == 0)
            return 0;

        actualFreq = (double)(scale / period2);
    }

    g_PwmController[controller].actualFreq = actualFreq;
    g_PwmController[controller].period = p;
    g_PwmController[controller].presc = pre;

    return actualFreq;
}

TinyCLR_Result STM32F7_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin) {
    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    auto actualPin = STM32F7_Pwm_GetGpioPinForChannel(self, controller, pin);

    STM32F7_GpioInternal_ConfigurePin(actualPin->number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, actualPin->alternateFunction);

    uint16_t enBit = TIM_CCER_CC1E << (4 * pin);

    treg->CCER |= enBit; // enable output

    uint16_t cr1 = treg->CR1;

    if ((cr1 & TIM_CR1_CEN) == 0) { // timer stopped
        treg->EGR = TIM_EGR_UG; // enforce register update
        treg->CR1 = cr1 | TIM_CR1_CEN; // start timer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin) {
    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    auto actualPin = STM32F7_Pwm_GetGpioPinForChannel(self, controller, pin);

    uint16_t ccer = treg->CCER;

    ccer &= ~(TIM_CCER_CC1E << (4 * pin));
    treg->CCER = ccer; // disable output

    STM32F7_GpioInternal_ConfigurePin(actualPin->number, STM32F7_Gpio_PortMode::Input, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, STM32F7_Gpio_AlternateFunction::AF0);

    if ((ccer & (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)) == 0) { // idle
        treg->CR1 &= ~TIM_CR1_CEN; // stop timer
    }

    return TinyCLR_Result::Success;
}

int32_t STM32F7_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self, int32_t controller) {
    return PWM_PER_CONTROLLER;
}

STM32F7_Gpio_Pin* STM32F7_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin) {
    return &(g_PwmController[controller].gpioPin[pin]);
}

double STM32F7_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self, int32_t controller) {
    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    if ((uint32_t)treg & 0x10000)
        return STM32F7_APB2_CLOCK_HZ; // max can be Systemclock / 2 MHz on some PWM

    return STM32F7_APB1_CLOCK_HZ; //max can be Systemclock / 4 MHz on some PWM
}

double STM32F7_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self, int32_t controller) {
    return STM32F7_MIN_PWM_FREQUENCY;
}

TinyCLR_Result STM32F7_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t controller, int32_t pin, double dutyCycle, bool invertPolarity) {
    ptr_TIM_TypeDef treg = g_PwmController[controller].timerdef;

    uint32_t duration = (uint32_t)(dutyCycle * g_PwmController[controller].period);

    if (duration > g_PwmController[controller].period)
        duration = g_PwmController[controller].period;

    treg->PSC = g_PwmController[controller].presc - 1;
    treg->ARR = g_PwmController[controller].period - 1;

    if (g_PwmController[controller].timer == 2 || g_PwmController[controller].timer == 5) {
        if (pin == 0)
            treg->CCR1 = duration;
        else if (pin == 1)
            treg->CCR2 = duration;
        else if (pin == 2)
            treg->CCR3 = duration;
        else if (pin == 3)
            treg->CCR4 = duration;
    }
    else {
        *(__IO uint16_t*)&((uint32_t*)&treg->CCR1)[pin] = duration;
    }

    uint32_t invBit = TIM_CCER_CC1P << (4 * pin);

    if (invertPolarity) {
        treg->CCER |= invBit;
    }
    else {
        treg->CCER &= ~invBit;
    }

    if (duration != (uint32_t)(g_PwmController[controller].dutyCycle[pin] * g_PwmController[controller].period))
        treg->EGR = TIM_EGR_UG; // enforce register update - update immidiately any changes

    g_PwmController[controller].invert[pin] = invertPolarity;
    g_PwmController[controller].dutyCycle[pin] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result STM32F7_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, int32_t controller, double& frequency) {
    // If current frequency is same with desired frequency, no need to re-calculate
    if (g_PwmController[controller].theoryFreq == frequency) {
        return TinyCLR_Result::Success;
    }

    // If detected a different, save desired frequency
    g_PwmController[controller].theoryFreq = frequency;

    // Calculate actual frequency base on desired frequency
    frequency = STM32F7_Pwm_GetActualFrequency(self, controller);

    // Update channel if frequency had different
    for (int p = 0; p < PWM_PER_CONTROLLER; p++)
        if (g_PwmController[controller].gpioPin[p].number != PIN_NONE)
            if (STM32F7_Pwm_SetPulseParameters(self, controller, p, g_PwmController[controller].dutyCycle[p], g_PwmController[controller].invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Pwm_Acquire(const TinyCLR_Pwm_Provider* self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    STM32F7_Pwm_ResetController(controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Pwm_Release(const TinyCLR_Pwm_Provider* self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    STM32F7_Pwm_ResetController(controller);

    return TinyCLR_Result::Success;
}

void STM32F7_Pwm_Reset() {
    if (TOTAL_PWM_CONTROLLER > 0) g_PwmController[0].timerdef = TIM1;
    if (TOTAL_PWM_CONTROLLER > 1) g_PwmController[1].timerdef = TIM2;
    if (TOTAL_PWM_CONTROLLER > 2) g_PwmController[2].timerdef = TIM3;
    if (TOTAL_PWM_CONTROLLER > 3) g_PwmController[3].timerdef = TIM4;
#if !defined(STM32F401xE) && !defined(STM32F411xE)
    if (TOTAL_PWM_CONTROLLER > 4) g_PwmController[4].timerdef = TIM5;
    if (TOTAL_PWM_CONTROLLER > 5) g_PwmController[5].timerdef = TIM6;
    if (TOTAL_PWM_CONTROLLER > 6) g_PwmController[6].timerdef = TIM7;
    if (TOTAL_PWM_CONTROLLER > 7) g_PwmController[7].timerdef = TIM8;
    if (TOTAL_PWM_CONTROLLER > 8) g_PwmController[8].timerdef = TIM9;
    if (TOTAL_PWM_CONTROLLER > 9) g_PwmController[9].timerdef = TIM10;
    if (TOTAL_PWM_CONTROLLER > 10) g_PwmController[10].timerdef = TIM11;
    if (TOTAL_PWM_CONTROLLER > 11) g_PwmController[11].timerdef = TIM12;
    if (TOTAL_PWM_CONTROLLER > 12) g_PwmController[12].timerdef = TIM13;
    if (TOTAL_PWM_CONTROLLER > 13) g_PwmController[13].timerdef = TIM14;
#endif

    for (auto controller = 0u; controller < TOTAL_PWM_CONTROLLER; controller++) {
        STM32F7_Pwm_ResetController(controller);
    }
}

void STM32F7_Pwm_ResetController(int32_t controller) {
    for (int p = 0; p < PWM_PER_CONTROLLER; p++) {
        g_PwmController[controller].gpioPin[p].number = pwmPins[controller][p].number;
        g_PwmController[controller].gpioPin[p].alternateFunction = pwmPins[controller][p].alternateFunction;

        if (g_PwmController[controller].gpioPin[p].number != PIN_NONE) {
            g_PwmController[controller].dutyCycle[p] = 0;
            g_PwmController[controller].invert[p] = false;
        }

        if (g_PwmController[controller].isOpened[p] == true) {
            STM32F7_Pwm_DisablePin(&pwmProviders, controller, p);
            STM32F7_Pwm_ReleasePin(&pwmProviders, controller, p);
        }

        g_PwmController[controller].isOpened[p] = false;
    }

    g_PwmController[controller].theoryFreq = 0.0;
    g_PwmController[controller].actualFreq = 0.0;
    g_PwmController[controller].period = 0;
    g_PwmController[controller].presc = 0;
    g_PwmController[controller].timer = controller + 1;
}

TinyCLR_Result STM32F7_Pwm_GetControllerCount(const TinyCLR_Pwm_Provider* self, int32_t& count) {
    count = TOTAL_PWM_CONTROLLER;

    return TinyCLR_Result::Success;
}
