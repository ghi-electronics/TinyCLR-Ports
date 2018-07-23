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

#include "STM32F4.h"

#if STM32F4_APB1_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ
#define PWM1_CLK_HZ (STM32F4_APB1_CLOCK_HZ)
#else
#define PWM1_CLK_HZ (STM32F4_APB1_CLOCK_HZ * 2)
#endif
#define PWM1_CLK_MHZ (PWM1_CLK_HZ / 1000000)

#if STM32F4_APB2_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ
#define PWM2_CLK_HZ (STM32F4_APB2_CLOCK_HZ)
#else
#define PWM2_CLK_HZ (STM32F4_APB2_CLOCK_HZ * 2)
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

#define STM32F4_MIN_PWM_FREQUENCY 1

typedef  TIM_TypeDef* ptr_TIM_TypeDef;

struct PwmState {
    int32_t controllerIndex;

    ptr_TIM_TypeDef     timReg;
    STM32F4_Gpio_Pin    gpioPin[PWM_PER_CONTROLLER];

    TinyCLR_Pwm_PulsePolarity invert[PWM_PER_CONTROLLER];
    bool                isOpened[PWM_PER_CONTROLLER];

    double              actualFreq;
    double              theoryFreq;
    double              dutyCycle[PWM_PER_CONTROLLER];

    uint32_t            period;
    uint32_t            presc;
    uint32_t            timer;

};

void STM32F4_Pwm_ResetController(int32_t controllerIndex);
STM32F4_Gpio_Pin* STM32F4_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);

static STM32F4_Gpio_Pin pwmPins[][PWM_PER_CONTROLLER] = STM32F4_PWM_PINS;

#define TOTAL_PWM_CONTROLLERS SIZEOF_ARRAY(pwmPins)

static PwmState pwmStates[TOTAL_PWM_CONTROLLERS];

static TinyCLR_Pwm_Controller pwmControllers[TOTAL_PWM_CONTROLLERS];
static TinyCLR_Api_Info pwmApi[TOTAL_PWM_CONTROLLERS];

const TinyCLR_Api_Info* STM32F4_Pwm_GetApi() {
    for (auto i = 0; i < TOTAL_PWM_CONTROLLERS; i++) {
        pwmControllers[i].ApiInfo = &pwmApi[i];
        pwmControllers[i].Acquire = &STM32F4_Pwm_Acquire;
        pwmControllers[i].Release = &STM32F4_Pwm_Release;
        pwmControllers[i].OpenChannel = &STM32F4_Pwm_OpenChannel;
        pwmControllers[i].CloseChannel = &STM32F4_Pwm_CloseChannel;
        pwmControllers[i].EnableChannel = &STM32F4_Pwm_EnableChannel;
        pwmControllers[i].DisableChannel = &STM32F4_Pwm_DisableChannel;
        pwmControllers[i].SetPulseParameters = &STM32F4_Pwm_SetPulseParameters;
        pwmControllers[i].SetDesiredFrequency = &STM32F4_Pwm_SetDesiredFrequency;
        pwmControllers[i].GetMinFrequency = &STM32F4_Pwm_GetMinFrequency;
        pwmControllers[i].GetMaxFrequency = &STM32F4_Pwm_GetMaxFrequency;
        pwmControllers[i].GetChannelCount = &STM32F4_Pwm_GetChannelCount;

        pwmApi[i].Author = "GHI Electronics, LLC";
        pwmApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.PwmController";
        pwmApi[i].Type = TinyCLR_Api_Type::PwmController;
        pwmApi[i].Version = 0;
        pwmApi[i].Implementation = &pwmControllers[i];
        pwmApi[i].State = &pwmStates[i];

        pwmStates[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&pwmApi;
}

//--//

TinyCLR_Result STM32F4_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    ptr_TIM_TypeDef treg = state->timReg;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, channel);

    if (!STM32F4_GpioInternal_OpenPin(actualPin->number))
        return TinyCLR_Result::SharingViolation;

    // relevant RCC register & bit
    __IO uint32_t* enReg = &RCC->APB1ENR;
    if ((uint32_t)treg & 0x10000) enReg = &RCC->APB2ENR;
    int enBit = 1 << (((uint32_t)treg >> 10) & 0x1F);

    if (!(*enReg & enBit)) { // not yet initialized
        *enReg |= enBit; // enable timer clock
        treg->CR1 = TIM_CR1_URS | TIM_CR1_ARPE; // double buffered update
        treg->EGR = TIM_EGR_UG; // enforce first update
        if (state->timer == 1 || state->timer == 8) {
            treg->BDTR |= TIM_BDTR_MOE; // main output enable (timer 1 & 8 only)
        }
    }

    *(__IO uint16_t*)&((uint32_t*)&treg->CCR1)[channel] = 0; // reset compare register

    // enable PWM channel
    uint32_t mode = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // PWM1 mode, double buffered
    if (channel & 1) mode <<= 8; // 1 or 3
    __IO uint32_t* reg = &treg->CCMR1;
    if (channel & 2) reg = &treg->CCMR2; // 2 or 3
    *reg |= mode;

    state->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    ptr_TIM_TypeDef treg = state->timReg;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, channel);

    uint32_t mask = 0xFF; // disable PWM channel
    if (channel & 1) mask = 0xFF00; // 1 or 3
    __IO uint32_t* reg = &treg->CCMR1;
    if (channel & 2) reg = &treg->CCMR2; // 2 or 3
    *reg &= ~mask;

    if ((treg->CCMR1 | treg->CCMR2) == 0) { // no channel active
        __IO uint32_t* enReg = &RCC->APB1ENR;
        if ((uint32_t)treg & 0x10000) enReg = &RCC->APB2ENR;
        int enBit = 1 << (((uint32_t)treg >> 10) & 0x1F);
        *enReg &= ~enBit; // disable timer clock
    }

    STM32F4_GpioInternal_ClosePin(actualPin->number);

    state->isOpened[channel] = false;

    return TinyCLR_Result::Success;
}

void STM32F4_Pwm_GetScaleFactor(double frequency, uint32_t& period, uint32_t& scale) {
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

double STM32F4_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    double freq = state->theoryFreq;

    ptr_TIM_TypeDef treg = state->timReg;

    STM32F4_Pwm_GetScaleFactor(freq, period, scale);

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

    if (state->timer != 2 && state->timer != 5) { // 16 bit timer
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

    state->actualFreq = actualFreq;
    state->period = p;
    state->presc = pre;

    return actualFreq;
}

TinyCLR_Result STM32F4_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    ptr_TIM_TypeDef treg = state->timReg;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, channel);

    STM32F4_GpioInternal_ConfigurePin(actualPin->number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, actualPin->alternateFunction);

    uint16_t enBit = TIM_CCER_CC1E << (4 * channel);

    treg->CCER |= enBit; // enable output

    uint16_t cr1 = treg->CR1;

    if ((cr1 & TIM_CR1_CEN) == 0) { // timer stopped
        treg->EGR = TIM_EGR_UG; // enforce register update
        treg->CR1 = cr1 | TIM_CR1_CEN; // start timer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    ptr_TIM_TypeDef treg = state->timReg;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, channel);

    uint16_t ccer = treg->CCER;

    ccer &= ~(TIM_CCER_CC1E << (4 * channel));
    treg->CCER = ccer; // disable output

    STM32F4_GpioInternal_ConfigurePin(actualPin->number, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);

    if ((ccer & (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)) == 0) { // idle
        treg->CR1 &= ~TIM_CR1_CEN; // stop timer
    }

    return TinyCLR_Result::Success;
}

uint32_t STM32F4_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self) {
    return PWM_PER_CONTROLLER;
}

STM32F4_Gpio_Pin* STM32F4_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    return &(state->gpioPin[channel]);
}

double STM32F4_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    ptr_TIM_TypeDef treg = state->timReg;

    if ((uint32_t)treg & 0x10000)
        return STM32F4_APB2_CLOCK_HZ; // max can be Systemclock / 2 MHz on some PWM

    return STM32F4_APB1_CLOCK_HZ; //max can be Systemclock / 4 MHz on some PWM
}

double STM32F4_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self) {
    return STM32F4_MIN_PWM_FREQUENCY;
}

TinyCLR_Result STM32F4_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    ptr_TIM_TypeDef treg = state->timReg;

    uint32_t duration = (uint32_t)(dutyCycle * state->period);

    if (duration > state->period)
        duration = state->period;

    treg->PSC = state->presc - 1;
    treg->ARR = state->period - 1;

    if (state->timer == 2 || state->timer == 5) {
        if (channel == 0)
            treg->CCR1 = duration;
        else if (channel == 1)
            treg->CCR2 = duration;
        else if (channel == 2)
            treg->CCR3 = duration;
        else if (channel == 3)
            treg->CCR4 = duration;
    }
    else {
        *(__IO uint16_t*)&((uint32_t*)&treg->CCR1)[channel] = duration;
    }

    uint32_t invBit = TIM_CCER_CC1P << (4 * channel);

    if (polarity == TinyCLR_Pwm_PulsePolarity::ActiveHigh) {
        treg->CCER |= invBit;
    }
    else {
        treg->CCER &= ~invBit;
    }

    if (duration != (uint32_t)(state->dutyCycle[channel] * state->period))
        treg->EGR = TIM_EGR_UG; // enforce register update - update immidiately any changes

    state->invert[channel] = polarity;
    state->dutyCycle[channel] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result STM32F4_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    // If current frequency is same with desired frequency, no need to re-calculate
    if (state->theoryFreq == frequency) {
        return TinyCLR_Result::Success;
    }

    // If detected a different, save desired frequency
    state->theoryFreq = frequency;

    // Calculate actual frequency base on desired frequency
    frequency = STM32F4_Pwm_GetActualFrequency(self);

    // Update channel if frequency had different
    for (int p = 0; p < PWM_PER_CONTROLLER; p++)
        if (state->gpioPin[p].number != PIN_NONE)
            if (STM32F4_Pwm_SetPulseParameters(self, p, state->dutyCycle[p], state->invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_Acquire(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    STM32F4_Pwm_ResetController(controllerIndex);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_Release(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    STM32F4_Pwm_ResetController(controllerIndex);

    return TinyCLR_Result::Success;
}

void STM32F4_Pwm_Reset() {
    if (TOTAL_PWM_CONTROLLERS > 0) pwmStates[0].timReg = TIM1;
    if (TOTAL_PWM_CONTROLLERS > 1) pwmStates[1].timReg = TIM2;
    if (TOTAL_PWM_CONTROLLERS > 2) pwmStates[2].timReg = TIM3;
    if (TOTAL_PWM_CONTROLLERS > 3) pwmStates[3].timReg = TIM4;
#if !defined(STM32F401xE) && !defined(STM32F411xE)
    if (TOTAL_PWM_CONTROLLERS > 4) pwmStates[4].timReg = TIM5;
    if (TOTAL_PWM_CONTROLLERS > 5) pwmStates[5].timReg = TIM6;
    if (TOTAL_PWM_CONTROLLERS > 6) pwmStates[6].timReg = TIM7;
    if (TOTAL_PWM_CONTROLLERS > 7) pwmStates[7].timReg = TIM8;
    if (TOTAL_PWM_CONTROLLERS > 8) pwmStates[8].timReg = TIM9;
    if (TOTAL_PWM_CONTROLLERS > 9) pwmStates[9].timReg = TIM10;
    if (TOTAL_PWM_CONTROLLERS > 10) pwmStates[10].timReg = TIM11;
    if (TOTAL_PWM_CONTROLLERS > 11) pwmStates[11].timReg = TIM12;
    if (TOTAL_PWM_CONTROLLERS > 12) pwmStates[12].timReg = TIM13;
    if (TOTAL_PWM_CONTROLLERS > 13) pwmStates[13].timReg = TIM14;
#endif

    for (auto controllerIndex = 0u; controllerIndex < TOTAL_PWM_CONTROLLERS; controllerIndex++) {
        STM32F4_Pwm_ResetController(controllerIndex);
    }
}

void STM32F4_Pwm_ResetController(int32_t controllerIndex) {
    auto state = &pwmStates[controllerIndex];

    for (int p = 0; p < PWM_PER_CONTROLLER; p++) {
        state->gpioPin[p].number = pwmPins[controllerIndex][p].number;
        state->gpioPin[p].alternateFunction = pwmPins[controllerIndex][p].alternateFunction;

        if (state->gpioPin[p].number != PIN_NONE) {
            state->dutyCycle[p] = 0;
            state->invert[p] = TinyCLR_Pwm_PulsePolarity::ActiveLow;
        }

        if (state->isOpened[p] == true) {
            STM32F4_Pwm_DisableChannel(&pwmControllers[controllerIndex], p);
            STM32F4_Pwm_CloseChannel(&pwmControllers[controllerIndex], p);
        }

        state->isOpened[p] = false;
    }

    state->theoryFreq = 0.0;
    state->actualFreq = 0.0;
    state->period = 0;
    state->presc = 0;
    state->timer = controllerIndex + 1;
}

