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

#define PWM0_BASE 0x40014000

#define PWM0IR (*(volatile unsigned long *)0x40014000)
#define PWM0TCR (*(volatile unsigned long *)0x40014004)
#define PWM0TC (*(volatile unsigned long *)0x40014008)
#define PWM0PR (*(volatile unsigned long *)0x4001400C)
#define PWM0PC (*(volatile unsigned long *)0x40014010)
#define PWM0MCR (*(volatile unsigned long *)0x40014014)
#define PWM0PCR (*(volatile unsigned long *)0x4001404C)
#define PWM0LER (*(volatile unsigned long *)0x40014050)

#define PWM1_BASE 0x40018000

#define PWM1IR (*(volatile unsigned long *)0x40018000)
#define PWM1TCR (*(volatile unsigned long *)0x40018004)
#define PWM1TC (*(volatile unsigned long *)0x40018008)
#define PWM1PR (*(volatile unsigned long *)0x4001800C)
#define PWM1PC (*(volatile unsigned long *)0x40018010)
#define PWM1MCR (*(volatile unsigned long *)0x40018014)
#define PWM1CCR (*(volatile unsigned long *)0x40018028)
#define PWM1CR0 (*(volatile unsigned long *)0x4001802C)
#define PWM1CR1 (*(volatile unsigned long *)0x40018030)
#define PWM1CR2 (*(volatile unsigned long *)0x40018034)
#define PWM1CR3 (*(volatile unsigned long *)0x40018038)
#define PWM1EMR (*(volatile unsigned long *)0x4001803C)
#define PWM1PCR (*(volatile unsigned long *)0x4001804C)
#define PWM1LER (*(volatile unsigned long *)0x40018050)
#define PWM1CTCR (*(volatile unsigned long *)0x40018070)

#define PWM_MILLISECONDS  1000
#define PWM_MICROSECONDS  1000000
#define PWM_NANOSECONDS   1000000000

static PwmController g_PwmController[TOTAL_PWM_CONTROLLER];

static uint8_t pwmProviderDefs[TOTAL_PWM_CONTROLLER * sizeof(TinyCLR_Pwm_Provider)];
static TinyCLR_Pwm_Provider* pwmProviders[TOTAL_PWM_CONTROLLER];
static TinyCLR_Api_Info pwmApi;

const TinyCLR_Api_Info* LPC17_Pwm_GetApi() {
    for (int i = 0; i < TOTAL_PWM_CONTROLLER; i++) {
        pwmProviders[i] = (TinyCLR_Pwm_Provider*)(pwmProviderDefs + (i * sizeof(TinyCLR_Pwm_Provider)));
        pwmProviders[i]->Parent = &pwmApi;
        pwmProviders[i]->Index = i;
        pwmProviders[i]->Acquire = &LPC17_Pwm_Acquire;
        pwmProviders[i]->Release = &LPC17_Pwm_Release;
        pwmProviders[i]->SetDesiredFrequency = &LPC17_Pwm_SetDesiredFrequency;
        pwmProviders[i]->AcquirePin = &LPC17_Pwm_AcquirePin;
        pwmProviders[i]->ReleasePin = &LPC17_Pwm_ReleasePin;
        pwmProviders[i]->EnablePin = &LPC17_Pwm_EnablePin;
        pwmProviders[i]->DisablePin = &LPC17_Pwm_DisablePin;
        pwmProviders[i]->SetPulseParameters = &LPC17_Pwm_SetPulseParameters;
        pwmProviders[i]->GetMinFrequency = &LPC17_Pwm_GetMinFrequency;
        pwmProviders[i]->GetMaxFrequency = &LPC17_Pwm_GetMaxFrequency;
        pwmProviders[i]->GetActualFrequency = &LPC17_Pwm_GetActualFrequency;
        pwmProviders[i]->GetPinCount = &LPC17_Pwm_GetPinCount;
    }

    pwmApi.Author = "GHI Electronics, LLC";
    pwmApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.PwmProvider";
    pwmApi.Type = TinyCLR_Api_Type::PwmProvider;
    pwmApi.Version = 0;
    pwmApi.Count = TOTAL_PWM_CONTROLLER;
    pwmApi.Implementation = pwmProviders;

    return &pwmApi;
}

TinyCLR_Result LPC17_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, pin);

    if (!LPC17_Gpio_OpenPin(actualPin))
        return TinyCLR_Result::SharingViolation;

    // enable PWM output
    if (g_PwmController[self->Index].channel[pin] == 0) {

        // Enable PWMs controller channel 0
        LPC_SC->PCONP |= PCONP_PCPWM0;

        // Reset Timer Counter
        PWM0TCR |= (1 << 1);
        *g_PwmController[self->Index].matchAddress[pin] = 0;
        PWM0MCR = (1 << 1); // Reset on MAT0
        PWM0TCR = 1; // Enable
        PWM0PCR |= (1 << (9 + g_PwmController[self->Index].match[pin])); // To enable output on the proper channel
    }
    else if (g_PwmController[self->Index].channel[pin] == 1) {

        // Enable PWMs controller channel 1
        LPC_SC->PCONP |= PCONP_PCPWM1;

        // Reset Timer Counter
        PWM1TCR |= (1 << 1);
        *g_PwmController[self->Index].matchAddress[pin] = 0;
        PWM1MCR = (1 << 1); // Reset on MAT0
        PWM1TCR = 1; // Enable
        PWM1PCR |= (1 << (9 + (g_PwmController[self->Index].match[pin]))); // To enable output on the proper channel
    }

    return TinyCLR_Result::Success;;
}

TinyCLR_Result LPC17_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, pin);

    LPC17_Gpio_ClosePin(actualPin);

    return TinyCLR_Result::Success;
}

void LPC17_Pwm_GetScaleFactor(double frequency, uint32_t& period, uint32_t& scale) {
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

double LPC17_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    uint64_t periodInNanoSeconds = 0;

    double frequency = g_PwmController[self->Index].frequency;

    LPC17_Pwm_GetScaleFactor(frequency, period, scale);

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

    uint64_t periodTicks = (uint64_t)(((LPC17_SYSTEM_CLOCK_HZ / 2) / 1000000)) * periodInNanoSeconds / 1000;

    // update actual period and duration, after few boudary changes base on current system clock
    periodInNanoSeconds = ((uint64_t)(periodTicks * 1000)) / ((uint64_t)(((LPC17_SYSTEM_CLOCK_HZ / 2) / 1000000)));

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

TinyCLR_Result LPC17_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, pin);

    LPC17_Gpio_ConfigurePin(actualPin, LPC17_Gpio_Direction::Input, g_PwmController[self->Index].gpioPin[pin].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, pin);

    LPC17_Gpio_ConfigurePin(actualPin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    return TinyCLR_Result::Success;
}

int32_t LPC17_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self) {
    uint32_t chnlCnt = 0;

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        if (g_PwmController[self->Index].gpioPin[p].number != PIN_NONE) {
            chnlCnt++;
        }
    }

    return chnlCnt;
}

int32_t LPC17_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    return g_PwmController[self->Index].gpioPin[pin].number;
}

double LPC17_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self) {
    return (LPC17_SYSTEM_CLOCK_HZ / 4);
}

double LPC17_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self) {
    return 1;
}

TinyCLR_Result LPC17_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity) {

    uint32_t period = 0;
    uint32_t duration = 0;
    uint32_t scale = 0;

    uint32_t periodInNanoSeconds = 0;
    uint32_t durationInNanoSeconds = 0;

    double frequency = g_PwmController[self->Index].frequency;

    LPC17_Pwm_GetScaleFactor(frequency, period, scale);

    duration = (uint32_t)(dutyCycle * period);

    // Work on actual period and duration after update.
    // Repeat calculation again to genarate periodTicks and highTicks. LPC17 convert from period and duration to ticks.
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

    // 18M/M = 18 * period / 1000 to get legal value.
    uint32_t periodTicks = (uint64_t)(((LPC17_SYSTEM_CLOCK_HZ / 2) / 1000000)) * periodInNanoSeconds / 1000;
    uint32_t highTicks = (uint64_t)(((LPC17_SYSTEM_CLOCK_HZ / 2) / 1000000)) * durationInNanoSeconds / 1000;

    // A strange instance where if the periodInNanoSeconds would have ended in infinite 3 (calculated with float in NETMF), periodTicks are not computed correctly
    if (0 == ((periodInNanoSeconds - 3) % 10))
        periodTicks += 1;

    if (0 == ((highTicks - 3) % 10))
        highTicks += 1;

    periodTicks -= 1;
    highTicks -= 1;

    if ((int)periodTicks < 0)
        periodTicks = 0;

    if ((int)highTicks < 0)
        highTicks = 0;

    if (highTicks > periodTicks)
        highTicks = periodTicks;

    if (invertPolarity)
        highTicks = periodTicks - highTicks;

    if (period == 0 || duration == 0) {
        LPC17_Gpio_EnableOutputPin(g_PwmController[self->Index].gpioPin[pin].number, false);
        g_PwmController[self->Index].outputEnabled[pin] = true;

        return TinyCLR_Result::Success;
    }
    else if (duration >= period) {
        LPC17_Gpio_EnableOutputPin(g_PwmController[self->Index].gpioPin[pin].number, true);
        g_PwmController[self->Index].outputEnabled[pin] = true;

        return TinyCLR_Result::Success;
    }
    else {
        if (g_PwmController[self->Index].channel[pin] == 0) {
            // Re-scale with new frequency!
            if ((PWM0MR0 != periodTicks)) {

                // Reset Timer Counter
                PWM0TCR |= (1 << 1);
                PWM0MR0 = periodTicks;
                PWM0MCR = (1 << 1); // Reset on MAT0
                PWM0TCR = 1; // Enable
            }

            *g_PwmController[self->Index].matchAddress[pin] = highTicks;
        }
        else if (g_PwmController[self->Index].channel[pin] == 1) {
            // Re-scale with new frequency!
            if ((PWM1MR0 != periodTicks)) {
                // Reset Timer Counter
                PWM1TCR |= (1 << 1);
                PWM1MR0 = periodTicks;
                PWM1MCR = (1 << 1); // Reset on MAT0
                PWM1TCR = 1; // Enable
            }

            *g_PwmController[self->Index].matchAddress[pin] = highTicks;
        }

        if (g_PwmController[self->Index].outputEnabled[pin] == true) {
            LPC17_Pwm_EnablePin(self, pin);

            g_PwmController[self->Index].outputEnabled[pin] = false;
        }
    }

    g_PwmController[self->Index].invert[pin] = invertPolarity;
    g_PwmController[self->Index].dutyCycle[pin] = dutyCycle;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency) {
    g_PwmController[self->Index].frequency = frequency;

    // Calculate actual frequency
    frequency = LPC17_Pwm_GetActualFrequency(self);

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++)
        if (g_PwmController[self->Index].gpioPin[p].number != PIN_NONE)
            if (LPC17_Pwm_SetPulseParameters(self, p, g_PwmController[self->Index].dutyCycle[p], g_PwmController[self->Index].invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_Acquire(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    LPC17_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_Release(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    LPC17_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

void LPC17_Pwm_Reset() {

    for (auto controller = 0; controller < TOTAL_PWM_CONTROLLER; controller++) {
        LPC17_Pwm_ResetController(controller);

        for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
            if (g_PwmController[pwmProviders[controller]->Index].gpioPin[p].number != PIN_NONE) {
                // Reset PWM and close pin
                LPC17_Pwm_DisablePin(pwmProviders[controller], p);
                LPC17_Pwm_ReleasePin(pwmProviders[controller], p);
            }
        }
    }
}

void LPC17_Pwm_ResetController(int32_t controller) {
    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        g_PwmController[pwmProviders[controller]->Index].gpioPin[p] = LPC17_Pwm_GetPins(controller, p);

        if (g_PwmController[pwmProviders[controller]->Index].gpioPin[p].number != PIN_NONE) {
            // Reset values
            g_PwmController[pwmProviders[controller]->Index].channel[p] = controller;
            g_PwmController[pwmProviders[controller]->Index].match[p] = p;
            g_PwmController[pwmProviders[controller]->Index].matchAddress[p] = pwmProviders[controller]->Index == 0 ? (uint32_t*)(PWM0MR1 + (p * 4)) : (uint32_t*)(PWM1MR1 + (p * 4));
            g_PwmController[pwmProviders[controller]->Index].outputEnabled[p] = false;
            g_PwmController[pwmProviders[controller]->Index].invert[p] = false;
            g_PwmController[pwmProviders[controller]->Index].frequency = 0.0;
            g_PwmController[pwmProviders[controller]->Index].dutyCycle[p] = 0.0;
        }
    }
}
