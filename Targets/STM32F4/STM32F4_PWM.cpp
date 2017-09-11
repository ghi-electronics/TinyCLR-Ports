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


#if SYSTEM_APB1_CLOCK_HZ == SYSTEM_CYCLE_CLOCK_HZ
#define PWM1_CLK_HZ (SYSTEM_APB1_CLOCK_HZ)
#else
#define PWM1_CLK_HZ (SYSTEM_APB1_CLOCK_HZ * 2)
#endif
#define PWM1_CLK_MHZ (PWM1_CLK_HZ / ONE_MHZ)

#if SYSTEM_APB2_CLOCK_HZ == SYSTEM_CYCLE_CLOCK_HZ
#define PWM2_CLK_HZ (SYSTEM_APB2_CLOCK_HZ)
#else
#define PWM2_CLK_HZ (SYSTEM_APB2_CLOCK_HZ * 2)
#endif
#define PWM2_CLK_MHZ (PWM2_CLK_HZ / ONE_MHZ)

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

#define pwmController(x)  g_PwmController[x]


typedef  TIM_TypeDef* ptr_TIM_TypeDef;

struct PwmController {
    ptr_TIM_TypeDef     timerdef;
    STM32F4_Pin         gpioPin[PWM_PER_CONTROLLER];

    bool                invert[PWM_PER_CONTROLLER];
    double              actualFreq;
    double              theoryFreq;
    double              dutyCycle[PWM_PER_CONTROLLER];

    uint32_t            period;
    uint32_t            presc;
    uint32_t            timer;

};

static STM32F4_Pin pwmPins[][PWM_PER_CONTROLLER] = STM32F4_PWM_PINS;

static const int TOTAL_PWM_CONTROLLER = SIZEOF_CONST_ARRAY(pwmPins);

static PwmController g_PwmController[TOTAL_PWM_CONTROLLER];

static uint8_t pwmProviderDefs[TOTAL_PWM_CONTROLLER * sizeof(TinyCLR_Pwm_Provider)];
static TinyCLR_Pwm_Provider* pwmProviders[TOTAL_PWM_CONTROLLER];
static TinyCLR_Api_Info pwmApi;

const TinyCLR_Api_Info* STM32F4_Pwm_GetApi() {
    for (int i = 0; i < TOTAL_PWM_CONTROLLER; i++) {
        pwmProviders[i] = (TinyCLR_Pwm_Provider*)(pwmProviderDefs + (i * sizeof(TinyCLR_Pwm_Provider)));
        pwmProviders[i]->Parent = &pwmApi;
        pwmProviders[i]->Index = i;
        pwmProviders[i]->Acquire = &STM32F4_Pwm_Acquire;
        pwmProviders[i]->Release = &STM32F4_Pwm_Release;
        pwmProviders[i]->SetDesiredFrequency = &STM32F4_Pwm_SetDesiredFrequency;
        pwmProviders[i]->AcquirePin = &STM32F4_Pwm_AcquirePin;
        pwmProviders[i]->ReleasePin = &STM32F4_Pwm_ReleasePin;
        pwmProviders[i]->EnablePin = &STM32F4_Pwm_EnablePin;
        pwmProviders[i]->DisablePin = &STM32F4_Pwm_DisablePin;
        pwmProviders[i]->SetPulseParameters = &STM32F4_Pwm_SetPulseParameters;
        pwmProviders[i]->GetMinFrequency = &STM32F4_Pwm_GetMinFrequency;
        pwmProviders[i]->GetMaxFrequency = &STM32F4_Pwm_GetMaxFrequency;
        pwmProviders[i]->GetActualFrequency = &STM32F4_Pwm_GetActualFrequency;
        pwmProviders[i]->GetPinCount = &STM32F4_Pwm_GetPinCount;
    }

    pwmApi.Author = "GHI Electronics, LLC";
    pwmApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.PwmProvider";
    pwmApi.Type = TinyCLR_Api_Type::PwmProvider;
    pwmApi.Version = 0;
    pwmApi.Count = TOTAL_PWM_CONTROLLER;
    pwmApi.Implementation = pwmProviders;

    return &pwmApi;
}

//--//

TinyCLR_Result STM32F4_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, pin);

    if (!STM32F4_Gpio_OpenPin(actualPin->number))
        return TinyCLR_Result::SharingViolation;

    // relevant RCC register & bit
    __IO uint32_t* enReg = &RCC->APB1ENR;
    if ((uint32_t)treg & 0x10000) enReg = &RCC->APB2ENR;
    int enBit = 1 << (((uint32_t)treg >> 10) & 0x1F);

    if (!(*enReg & enBit)) { // not yet initialized
        *enReg |= enBit; // enable timer clock
        treg->CR1 = TIM_CR1_URS | TIM_CR1_ARPE; // double buffered update
        treg->EGR = TIM_EGR_UG; // enforce first update
        if (pwmController(self->Index).timer == 1 || pwmController(self->Index).timer == 8) {
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

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, pin);

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

    STM32F4_Gpio_ClosePin(actualPin->number);

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

double STM32F4_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    double freq = pwmController(self->Index).theoryFreq;

    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    STM32F4_Pwm_GetScaleFactor(freq, period, scale);

    uint32_t p = period, s = scale;

    // set pre, p, & d such that:
    // pre * p = PWM_CLK * period / scale
    // pre * d = PWM_CLK * duration / scale

    uint32_t clk = PWM1_CLK_HZ;

    if ((uint32_t)treg & 0x10000) clk = PWM2_CLK_HZ; // APB2

    uint32_t pre = clk / s; // prescaler

    if (pre == 0) { // s > PWM_CLK
        uint32_t sm = s / ONE_MHZ; // scale in MHz
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

    if (pwmController(self->Index).timer != 2 && pwmController(self->Index).timer != 5) { // 16 bit timer
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

    pwmController(self->Index).actualFreq = actualFreq;
    pwmController(self->Index).period = p;
    pwmController(self->Index).presc = pre;

    return actualFreq;
}

TinyCLR_Result STM32F4_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, pin);

    STM32F4_Gpio_ConfigurePin(actualPin->number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, actualPin->alternateFunction);

    uint16_t enBit = TIM_CCER_CC1E << (4 * pin);

    treg->CCER |= enBit; // enable output

    uint16_t cr1 = treg->CR1;

    if ((cr1 & TIM_CR1_CEN) == 0) { // timer stopped
        treg->EGR = TIM_EGR_UG; // enforce register update
        treg->CR1 = cr1 | TIM_CR1_CEN; // start timer
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    auto actualPin = STM32F4_Pwm_GetGpioPinForChannel(self, pin);

    uint16_t ccer = treg->CCER;

    ccer &= ~(TIM_CCER_CC1E << (4 * pin));
    treg->CCER = ccer; // disable output

    STM32F4_Gpio_ConfigurePin(actualPin->number, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);

    if ((ccer & (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E)) == 0) { // idle
        treg->CR1 &= ~TIM_CR1_CEN; // stop timer
    }

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self) {
    int chnlCnt = 0;

    for (int p = 0; p < PWM_PER_CONTROLLER; p++) {
        if (pwmController(self->Index).gpioPin[p].number != PIN_NONE) {
            chnlCnt++;
        }
    }

    return chnlCnt;
}

STM32F4_Pin* STM32F4_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    return &(pwmController(self->Index).gpioPin[pin]);
}

double STM32F4_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self) {
    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    if ((uint32_t)treg & 0x10000)
        return SYSTEM_APB2_CLOCK_HZ; // max can be Systemclock / 2 MHz on some PWM

    return SYSTEM_APB1_CLOCK_HZ; //max can be Systemclock / 4 MHz on some PWM
}

double STM32F4_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self) {
    return STM32F4_MIN_PWM_FREQUENCY;
}

TinyCLR_Result STM32F4_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity) {
    ptr_TIM_TypeDef treg = pwmController(self->Index).timerdef;

    uint32_t duration = (uint32_t)(dutyCycle * pwmController(self->Index).period);

    if (duration > pwmController(self->Index).period)
        duration = pwmController(self->Index).period;

    treg->PSC = pwmController(self->Index).presc - 1;
    treg->ARR = pwmController(self->Index).period - 1;

    if (pwmController(self->Index).timer == 2) {
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

    if (duration != (uint32_t)(pwmController(self->Index).dutyCycle[pin] * pwmController(self->Index).period))
        treg->EGR = TIM_EGR_UG; // enforce register update - update immidiately any changes

    pwmController(self->Index).invert[pin] = invertPolarity;
    pwmController(self->Index).dutyCycle[pin] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result STM32F4_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency) {
    // If current frequency is same with desired frequency, no need to re-calculate
    if (pwmController(self->Index).theoryFreq == frequency) {
        return TinyCLR_Result::Success;
    }

    // If detected a different, save desired frequency
    pwmController(self->Index).theoryFreq = frequency;

    // Calculate actual frequency base on desired frequency
    frequency = STM32F4_Pwm_GetActualFrequency(self);

    // Update channel if frequency had different
    for (int p = 0; p < PWM_PER_CONTROLLER; p++)
        if (pwmController(self->Index).gpioPin[p].number != PIN_NONE)
            if (STM32F4_Pwm_SetPulseParameters(self, p, pwmController(self->Index).dutyCycle[p], pwmController(self->Index).invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_Acquire(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    STM32F4_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Pwm_Release(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    STM32F4_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

void STM32F4_Pwm_Reset() {
    if (TOTAL_PWM_CONTROLLER > 0) g_PwmController[0].timerdef = TIM1;
    if (TOTAL_PWM_CONTROLLER > 1) g_PwmController[1].timerdef = TIM2;
    if (TOTAL_PWM_CONTROLLER > 2) g_PwmController[2].timerdef = TIM3;
    if (TOTAL_PWM_CONTROLLER > 3) g_PwmController[3].timerdef = TIM4;
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

    for (auto controller = 0u; controller < TOTAL_PWM_CONTROLLER; controller++) {
        auto& c = g_PwmController[controller];

        c.actualFreq = 0.0;
        c.theoryFreq = 0.0;
        c.period = 0;
        c.presc = 0;
        c.timer = controller + 1;

        for (auto i = 0; i < PWM_PER_CONTROLLER; i++) {
            c.invert[0] = false;
            c.dutyCycle[0] = 0.0;

            c.gpioPin[i].number = pwmPins[controller]->number;
            c.gpioPin[i].alternateFunction = pwmPins[controller]->alternateFunction;
        }

        STM32F4_Pwm_ResetController(controller);

        for (int p = 0; p < PWM_PER_CONTROLLER; p++) {
            if (pwmController(controller).gpioPin[p].number != PIN_NONE) {
                STM32F4_Pwm_DisablePin(pwmProviders[controller], p);
                STM32F4_Pwm_ReleasePin(pwmProviders[controller], p);
            }
        }
    }
}

void STM32F4_Pwm_ResetController(int32_t controller) {
    for (int p = 0; p < PWM_PER_CONTROLLER; p++) {
        if (pwmController(controller).gpioPin[p].number != PIN_NONE) {
            pwmController(controller).dutyCycle[p] = 0;
            pwmController(controller).invert[p] = false;
        }
    }

    pwmController(controller).theoryFreq = 0;
    pwmController(controller).actualFreq = 0;
    pwmController(controller).period = 0;
    pwmController(controller).presc = 0;
}

