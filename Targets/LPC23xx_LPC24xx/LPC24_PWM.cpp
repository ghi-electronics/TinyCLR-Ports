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

#define PWM0_BASE 0xE0014000

#define PWM0IR (*(volatile unsigned long *)0xE0014000)
#define PWM0TCR (*(volatile unsigned long *)0xE0014004)
#define PWM0TC (*(volatile unsigned long *)0xE0014008)
#define PWM0PR (*(volatile unsigned long *)0xE001400C)
#define PWM0PC (*(volatile unsigned long *)0xE0014010)
#define PWM0MCR (*(volatile unsigned long *)0xE0014014)
#define PWM0PCR (*(volatile unsigned long *)0xE001404C)
#define PWM0LER (*(volatile unsigned long *)0xE0014050)

#define PWM1_BASE 0xE0018000

#define PWM1IR (*(volatile unsigned long *)0xE0018000)
#define PWM1TCR (*(volatile unsigned long *)0xE0018004)
#define PWM1TC (*(volatile unsigned long *)0xE0018008)
#define PWM1PR (*(volatile unsigned long *)0xE001800C)
#define PWM1PC (*(volatile unsigned long *)0xE0018010)
#define PWM1MCR (*(volatile unsigned long *)0xE0018014)
#define PWM1CCR (*(volatile unsigned long *)0xE0018028)
#define PWM1CR0 (*(volatile unsigned long *)0xE001802C)
#define PWM1CR1 (*(volatile unsigned long *)0xE0018030)
#define PWM1CR2 (*(volatile unsigned long *)0xE0018034)
#define PWM1CR3 (*(volatile unsigned long *)0xE0018038)
#define PWM1EMR (*(volatile unsigned long *)0xE001803C)
#define PWM1PCR (*(volatile unsigned long *)0xE001804C)
#define PWM1LER (*(volatile unsigned long *)0xE0018050)
#define PWM1CTCR (*(volatile unsigned long *)0xE0018070)

// PWM
#define PWM0MR0 (*(volatile unsigned long *)0xE0014018)
#define PWM0MR1 ((uint32_t)0xE001401C)
#define PWM0MR2 ((uint32_t)0xE0014020)
#define PWM0MR3 ((uint32_t)0xE0014024)
#define PWM0MR4 ((uint32_t)0xE0014040)
#define PWM0MR5 ((uint32_t)0xE0014044)
#define PWM0MR6 ((uint32_t)0xE0014048)

#define PWM1MR0 (*(volatile unsigned long *)0xE0018018)
#define PWM1MR1 ((uint32_t)0xE001801C)
#define PWM1MR2 ((uint32_t)0xE0018020)
#define PWM1MR3 ((uint32_t)0xE0018024)
#define PWM1MR4 ((uint32_t)0xE0018040)
#define PWM1MR5 ((uint32_t)0xE0018044)
#define PWM1MR6 ((uint32_t)0xE0018048)

#define PCONP_PCPWM0 0x20
#define PCONP_PCPWM1 0x40

#define LPC24_MAX_PWM_FREQUENCY (SYSTEM_CLOCK_HZ)
#define LPC24_MIN_PWM_FREQUENCY 1

#define PWM_MILLISECONDS  1000
#define PWM_MICROSECONDS  1000000
#define PWM_NANOSECONDS   1000000000

static PwmController g_PwmController[TOTAL_PWM_CONTROLLER];

static TinyCLR_Pwm_Controller pwmProviders;
static TinyCLR_Api_Info pwmApi;

const TinyCLR_Api_Info* LPC24_Pwm_GetApi() {
    pwmProviders.ApiInfo = &pwmApi;
    pwmProviders.Acquire = &LPC24_Pwm_Acquire;
    pwmProviders.Release = &LPC24_Pwm_Release;
    pwmProviders.SetDesiredFrequency = &LPC24_Pwm_SetDesiredFrequency;
    pwmProviders.AcquirePin = &LPC24_Pwm_AcquirePin;
    pwmProviders.ReleasePin = &LPC24_Pwm_ReleasePin;
    pwmProviders.EnablePin = &LPC24_Pwm_EnablePin;
    pwmProviders.DisablePin = &LPC24_Pwm_DisablePin;
    pwmProviders.SetPulseParameters = &LPC24_Pwm_SetPulseParameters;
    pwmProviders.GetMinFrequency = &LPC24_Pwm_GetMinFrequency;
    pwmProviders.GetMaxFrequency = &LPC24_Pwm_GetMaxFrequency;
    pwmProviders.GetPinCount = &LPC24_Pwm_GetPinCount;
    pwmProviders.GetControllerCount = &LPC24_Pwm_GetControllerCount;

    pwmApi.Author = "GHI Electronics, LLC";
    pwmApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.PwmProvider";
    pwmApi.Type = TinyCLR_Api_Type::PwmProvider;
    pwmApi.Version = 0;
    pwmApi.Implementation = &pwmProviders;

    return &pwmApi;
}

TinyCLR_Result LPC24_Pwm_AcquirePin(const TinyCLR_Pwm_Controller* self, int32_t pin) {
    int32_t actualPin = LPC24_Pwm_GetGpioPinForChannel(self, controller, pin);

    if (!LPC24_Gpio_OpenPin(actualPin))
        return TinyCLR_Result::SharingViolation;

    // enable PWM output
    if (g_PwmController[controller].channel[pin] == 0) {

        // Enable PWMs controller channel 0
        LPC24XX::SYSCON().PCONP |= PCONP_PCPWM0;

        // Reset Timer Counter
        PWM0TCR |= (1 << 1);
        *g_PwmController[controller].matchAddress[pin] = 0;
        PWM0MCR = (1 << 1); // Reset on MAT0
        PWM0TCR = 1; // Enable
        PWM0PCR |= (1 << (9 + g_PwmController[controller].match[pin])); // To enable output on the proper channel
    }
    else if (g_PwmController[controller].channel[pin] == 1) {

        // Enable PWMs controller channel 1
        LPC24XX::SYSCON().PCONP |= PCONP_PCPWM1;

        // Reset Timer Counter
        PWM1TCR |= (1 << 1);
        *g_PwmController[controller].matchAddress[pin] = 0;
        PWM1MCR = (1 << 1); // Reset on MAT0
        PWM1TCR = 1; // Enable
        PWM1PCR |= (1 << (9 + (g_PwmController[controller].match[pin]))); // To enable output on the proper channel
    }

    g_PwmController[controller].isOpened[pin] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Pwm_ReleasePin(const TinyCLR_Pwm_Controller* self, int32_t pin) {
    int32_t actualPin = LPC24_Pwm_GetGpioPinForChannel(self, controller, pin);

    LPC24_Gpio_ClosePin(actualPin);

    g_PwmController[controller].isOpened[pin] = false;

    return TinyCLR_Result::Success;
}

void LPC24_Pwm_GetScaleFactor(double frequency, uint32_t& period, uint32_t& scale) {
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

double LPC24_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    uint64_t periodInNanoSeconds = 0;

    double frequency = g_PwmController[controller].frequency;

    LPC24_Pwm_GetScaleFactor(frequency, period, scale);

    switch (scale) {
    case PWM_MILLISECONDS:
        periodInNanoSeconds = (period * 1000000);
        break;

    case PWM_MICROSECONDS:
        periodInNanoSeconds = (period * 1000);
        break;

    case PWM_NANOSECONDS:
        periodInNanoSeconds = period;
        break;

    default:
        return 0;
    }

    uint64_t periodTicks = (uint64_t)((SYSTEM_CLOCK_HZ / 1000000)) * periodInNanoSeconds / 1000;

    // update actual period and duration, after few boudary changes base on current system clock
    periodInNanoSeconds = ((uint64_t)(periodTicks * 1000)) / ((uint64_t)((SYSTEM_CLOCK_HZ / 1000000)));

    // make sure out frequency <= in frequency
    if (periodInNanoSeconds > 0) {
        double freq_out = (double)(1000000000 / periodInNanoSeconds);

        while (freq_out > frequency) {
            periodInNanoSeconds++;
            freq_out = (double)(1000000000 / periodInNanoSeconds);
        }
    }

    switch (scale) {
    case PWM_MILLISECONDS:
        period = periodInNanoSeconds / 1000000;
        break;

    case PWM_MICROSECONDS:
        period = periodInNanoSeconds / 1000;
        break;

    case PWM_NANOSECONDS:
        period = periodInNanoSeconds;
        break;
    }

    frequency = (double)(scale / period);

    return frequency;

}

TinyCLR_Result LPC24_Pwm_EnablePin(const TinyCLR_Pwm_Controller* self, int32_t pin) {
    int32_t actualPin = LPC24_Pwm_GetGpioPinForChannel(self, controller, pin);

    LPC24_Gpio_ConfigurePin(actualPin, LPC24_Gpio_Direction::Input, g_PwmController[controller].gpioPin[pin].pinFunction, LPC24_Gpio_PinMode::Inactive);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Pwm_DisablePin(const TinyCLR_Pwm_Controller* self, int32_t pin) {
    int32_t actualPin = LPC24_Pwm_GetGpioPinForChannel(self, controller, pin);

    LPC24_Gpio_ConfigurePin(actualPin, LPC24_Gpio_Direction::Output, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);

    return TinyCLR_Result::Success;
}

int32_t LPC24_Pwm_GetPinCount(const TinyCLR_Pwm_Controller* self) {
    return MAX_PWM_PER_CONTROLLER;
}

int32_t LPC24_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, int32_t pin) {
    return g_PwmController[controller].gpioPin[pin].number;
}

double LPC24_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self) {
    return LPC24_MAX_PWM_FREQUENCY;
}

double LPC24_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self) {
    return LPC24_MIN_PWM_FREQUENCY;
}

TinyCLR_Result LPC24_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, int32_t pin, double dutyCycle, bool invertPolarity) {

    uint32_t period = 0;
    uint32_t duration = 0;
    uint32_t scale = 0;

    uint32_t periodInNanoSeconds = 0;
    uint32_t durationInNanoSeconds = 0;

    double frequency = g_PwmController[controller].frequency;

    LPC24_Pwm_GetScaleFactor(frequency, period, scale);

    duration = (uint32_t)(dutyCycle * period);

    // Work on actual period and duration after update.
    // Repeat calculation again to genarate periodTicks and highTicks. G120 convert from period and duration to ticks.
    switch (scale) {
    case PWM_MILLISECONDS:
        periodInNanoSeconds = (period * 1000000);
        durationInNanoSeconds = (duration * 1000000);
        break;

    case PWM_MICROSECONDS:
        periodInNanoSeconds = (period * 1000);
        durationInNanoSeconds = (duration * 1000);
        break;

    case PWM_NANOSECONDS:
        periodInNanoSeconds = period;
        durationInNanoSeconds = duration;
        break;

    default:
        return TinyCLR_Result::InvalidOperation;
    }

    // make sure out frequency <= in frequency
    if (periodInNanoSeconds > 0) {
        double freq_out = (double)(1000000000 / periodInNanoSeconds);

        while (freq_out > frequency) {
            periodInNanoSeconds++;
            freq_out = (double)(1000000000 / periodInNanoSeconds);
        }
    }

    uint32_t periodTicks = (uint64_t)((SYSTEM_CLOCK_HZ / 1000000)) * periodInNanoSeconds / 1000;
    uint32_t highTicks = (uint64_t)((SYSTEM_CLOCK_HZ / 1000000)) * durationInNanoSeconds / 1000;

    // A strange instance where if the periodInNanoSeconds would have ended in infinite 3 (calculated with float in NETMF), periodTicks are not computed correctly
    if (0 == ((periodInNanoSeconds - 3) % 10))
        periodTicks += 1;

    if (0 == ((highTicks - 3) % 10))
        highTicks += 1;

    if ((int)periodTicks < 0)
        periodTicks = 0;

    if ((int)highTicks < 0)
        highTicks = 0;

    if (highTicks > periodTicks)
        highTicks = periodTicks;

    if (invertPolarity)
        highTicks = periodTicks - highTicks;

    if (periodInNanoSeconds == 0 || durationInNanoSeconds == 0) {
        LPC24_Gpio_EnableOutputPin(g_PwmController[controller].gpioPin[pin].number, false);
        g_PwmController[controller].outputEnabled[pin] = true;

        return TinyCLR_Result::Success;
    }
    else if (durationInNanoSeconds >= periodInNanoSeconds) {
        LPC24_Gpio_EnableOutputPin(g_PwmController[controller].gpioPin[pin].number, true);
        g_PwmController[controller].outputEnabled[pin] = true;

        return TinyCLR_Result::Success;
    }
    else {
        if (g_PwmController[controller].channel[pin] == 0) {
            // Re-scale with new frequency!
            if ((PWM0MR0 != periodTicks)) {

                // Reset Timer Counter
                PWM0TCR |= (1 << 1);
                PWM0MR0 = periodTicks;
                PWM0MCR = (1 << 1); // Reset on MAT0
                PWM0TCR = 1; // Enable
            }

            *g_PwmController[controller].matchAddress[pin] = highTicks;
        }
        else if (g_PwmController[controller].channel[pin] == 1) {
            // Re-scale with new frequency!
            if ((PWM1MR0 != periodTicks)) {
                // Reset Timer Counter
                PWM1TCR |= (1 << 1);
                PWM1MR0 = periodTicks;
                PWM1MCR = (1 << 1); // Reset on MAT0
                PWM1TCR = 1; // Enable
            }

            *g_PwmController[controller].matchAddress[pin] = highTicks;
        }

        if (g_PwmController[controller].outputEnabled[pin] == true) {
            LPC24_Pwm_EnablePin(self, controller, pin);

            g_PwmController[controller].outputEnabled[pin] = false;
        }
    }

    g_PwmController[controller].invert[pin] = invertPolarity;
    g_PwmController[controller].dutyCycle[pin] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result LPC24_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency) {
    g_PwmController[controller].frequency = frequency;

    // Calculate actual frequency
    frequency = LPC24_Pwm_GetActualFrequency(self, controller);


    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++)
        if (g_PwmController[controller].gpioPin[p].number != PIN_NONE)
            if (LPC24_Pwm_SetPulseParameters(self, controller, p, g_PwmController[controller].dutyCycle[p], g_PwmController[controller].invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Pwm_Acquire(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    LPC24_Pwm_ResetController(controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Pwm_Release(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    LPC24_Pwm_ResetController(controller);

    return TinyCLR_Result::Success;
}

void LPC24_Pwm_Reset() {

    for (auto controller = 0; controller < TOTAL_PWM_CONTROLLER; controller++) {
        LPC24_Pwm_ResetController(controller);
    }
}

void LPC24_Pwm_ResetController(int32_t controller) {
    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        g_PwmController[controller].gpioPin[p] = LPC24_Pwm_GetPins(controller, p);

        if (g_PwmController[controller].gpioPin[p].number != PIN_NONE) {
            // Reset values
            g_PwmController[controller].channel[p] = controller;
            g_PwmController[controller].match[p] = p;
#if defined(LPC2388) || defined(LPC2387)
            if (p < 3)
                g_PwmController[controller].matchAddress[p] = (uint32_t*)(PWM1MR1 + (p * 4));
            else
                g_PwmController[controller].matchAddress[p] = (uint32_t*)(PWM1MR4 + ((p - 3) * 4));
#else
            if (p < 3)
                g_PwmController[controller].matchAddress[p] = controller == 0 ? (uint32_t*)(PWM0MR1 + (p * 4)) : (uint32_t*)(PWM1MR1 + (p * 4));
            else
                g_PwmController[controller].matchAddress[p] = controller == 0 ? (uint32_t*)(PWM0MR4 + ((p - 3) * 4)) : (uint32_t*)(PWM1MR4 + ((p - 3) * 4));
#endif
            g_PwmController[controller].outputEnabled[p] = false;
            g_PwmController[controller].invert[p] = false;
            g_PwmController[controller].frequency = 0.0;
            g_PwmController[controller].dutyCycle[p] = 0.0;

            if (g_PwmController[controller].isOpened[p] == true) {
                if (controller == 0)
                    PWM0PCR &= ~(1 << (9 + (g_PwmController[controller].match[p])));
                if (controller == 1)
                    PWM1PCR &= ~(1 << (9 + (g_PwmController[controller].match[p])));

                LPC24_Pwm_DisablePin(&pwmProviders, controller, p);
                LPC24_Pwm_ReleasePin(&pwmProviders, controller, p);
            }

            g_PwmController[controller].isOpened[p] = false;
        }
    }
}

TinyCLR_Result LPC24_Pwm_GetControllerCount(const TinyCLR_Pwm_Controller* self, int32_t& count) {
    count = TOTAL_PWM_CONTROLLER;

    return TinyCLR_Result::Success;
}
