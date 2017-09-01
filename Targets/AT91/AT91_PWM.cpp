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

#define AT91_MAX_PWM_FREQUENCY (SYSTEM_CLOCK_HZ)
#define AT91_MIN_PWM_FREQUENCY 1

#define PWM_MILLISECONDS  1000
#define PWM_MICROSECONDS  1000000
#define PWM_NANOSECONDS   1000000000

static PwmController* g_PwmController;

static uint8_t pwmProviderDefs[TOTAL_PWM_CONTROLLER * sizeof(TinyCLR_Pwm_Provider)];
static TinyCLR_Pwm_Provider* pwmProviders[TOTAL_PWM_CONTROLLER];
static TinyCLR_Api_Info pwmApi;

const TinyCLR_Api_Info* AT91_Pwm_GetApi() {
    for (int i = 0; i < TOTAL_PWM_CONTROLLER; i++) {
        pwmProviders[i] = (TinyCLR_Pwm_Provider*)(pwmProviderDefs + (i * sizeof(TinyCLR_Pwm_Provider)));
        pwmProviders[i]->Parent = &pwmApi;
        pwmProviders[i]->Index = i;
        pwmProviders[i]->Acquire = &AT91_Pwm_Acquire;
        pwmProviders[i]->Release = &AT91_Pwm_Release;
        pwmProviders[i]->SetDesiredFrequency = &AT91_Pwm_SetDesiredFrequency;
        pwmProviders[i]->AcquirePin = &AT91_Pwm_AcquirePin;
        pwmProviders[i]->ReleasePin = &AT91_Pwm_ReleasePin;
        pwmProviders[i]->EnablePin = &AT91_Pwm_EnablePin;
        pwmProviders[i]->DisablePin = &AT91_Pwm_DisablePin;
        pwmProviders[i]->SetPulseParameters = &AT91_Pwm_SetPulseParameters;
        pwmProviders[i]->GetMinFrequency = &AT91_Pwm_GetMinFrequency;
        pwmProviders[i]->GetMaxFrequency = &AT91_Pwm_GetMaxFrequency;
        pwmProviders[i]->GetActualFrequency = &AT91_Pwm_GetActualFrequency;
        pwmProviders[i]->GetPinCount = &AT91_Pwm_GetPinCount;
    }

    pwmApi.Author = "GHI Electronics, LLC";
    pwmApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.PwmProvider";
    pwmApi.Type = TinyCLR_Api_Type::PwmProvider;
    pwmApi.Version = 0;
    pwmApi.Count = TOTAL_PWM_CONTROLLER;
    pwmApi.Implementation = (pwmApi.Count > 1) ? pwmProviders : (TinyCLR_Pwm_Provider**)&pwmProviderDefs;

    return &pwmApi;
}

TinyCLR_Result AT91_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    if (!AT91_Gpio_OpenPin(actualPin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;;
}

TinyCLR_Result AT91_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    AT91_Gpio_ClosePin(actualPin);

    return TinyCLR_Result::Success;
}

void AT91_Pwm_GetScaleFactor(double frequency, uint32_t& period, uint32_t& scale) {
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

double AT91_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    uint64_t periodInNanoSeconds = 0;

    double frequency = g_PwmController[self->Index].frequency;

    AT91_Pwm_GetScaleFactor(frequency, period, scale);

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

    uint64_t periodTicks = 0;

    // update actual period and duration, after few boudary changes base on current system clock
    periodInNanoSeconds = 0;

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

TinyCLR_Result AT91_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    AT91_Gpio_ConfigurePin(actualPin, AT91_Gpio_Direction::Input, g_PwmController[self->Index].gpioAlternateFunction[pin], AT91_Gpio_PinMode::Inactive);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    AT91_Gpio_ConfigurePin(actualPin, AT91_Gpio_Direction::Output, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);

    return TinyCLR_Result::Success;
}

int32_t AT91_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self) {
    return MAX_PWM_PER_CONTROLLER;
}

int32_t AT91_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    return g_PwmController[self->Index].gpioPin[pin];
}

double AT91_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self) {
    return AT91_MAX_PWM_FREQUENCY;
}

double AT91_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self) {
    return AT91_MIN_PWM_FREQUENCY;
}

TinyCLR_Result AT91_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity) {

    uint32_t period = 0;
    uint32_t duration = 0;
    uint32_t scale = 0;

    uint32_t periodInNanoSeconds = 0;
    uint32_t durationInNanoSeconds = 0;

    double frequency = g_PwmController[self->Index].frequency;

    AT91_Pwm_GetScaleFactor(frequency, period, scale);

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

    // 18M/M = 18 * period / 1000 to get legal value.
    uint32_t periodTicks = 0;
    uint32_t highTicks = 0; 

    if ((int)periodTicks < 0)
        periodTicks = 0;

    if ((int)highTicks < 0)
        highTicks = 0;

    if (highTicks > periodTicks)
        highTicks = periodTicks;

    if (invertPolarity)
        highTicks = periodTicks - highTicks;

    g_PwmController[self->Index].invert[pin] = invertPolarity;
    g_PwmController[self->Index].dutyCycle[pin] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result AT91_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency) {
    g_PwmController[self->Index].frequency = frequency;

    // Calculate actual frequency
    frequency = AT91_Pwm_GetActualFrequency(self);


    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++)
        if (g_PwmController[self->Index].gpioPin[p] != GPIO_PIN_NONE)
            if (AT91_Pwm_SetPulseParameters(self, p, g_PwmController[self->Index].dutyCycle[p], g_PwmController[self->Index].invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Pwm_Acquire(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    AT91_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Pwm_Release(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    AT91_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

void AT91_Pwm_Reset() {
    g_PwmController = AT91_Pwm_GetControllers();

    for (auto controller = 0; controller < TOTAL_PWM_CONTROLLER; controller++) {
        AT91_Pwm_ResetController(controller);

        for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
            if (g_PwmController[pwmProviders[controller]->Index].gpioPin[p] != GPIO_PIN_NONE) {
                // Reset PWM and close pin
                AT91_Pwm_DisablePin(pwmProviders[controller], p);
                AT91_Pwm_ReleasePin(pwmProviders[controller], p);
            }
        }
    }
}

void AT91_Pwm_ResetController(int32_t controller) {
    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        if (g_PwmController[pwmProviders[controller]->Index].gpioPin[p] != GPIO_PIN_NONE) {
            // Reset values
            g_PwmController[pwmProviders[controller]->Index].outputEnabled[p] = false;
            g_PwmController[pwmProviders[controller]->Index].invert[p] = false;
            g_PwmController[pwmProviders[controller]->Index].frequency = 0.0;
            g_PwmController[pwmProviders[controller]->Index].dutyCycle[p] = 0.0;
        }
    }
}