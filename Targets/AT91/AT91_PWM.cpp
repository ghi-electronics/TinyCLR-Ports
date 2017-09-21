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

#define PWM_MODE_REGISTER				(*(uint32_t *)(AT91C_BASE_PWMC + 0x00))
#define PWM_ENABLE_REGISTER				(*(uint32_t *)(AT91C_BASE_PWMC + 0x04))
#define PWM_DISABLE_REGISTER			(*(uint32_t *)(AT91C_BASE_PWMC + 0x08))
#define PWM_INTERUPT_ENABLE_REGISTER	(*(uint32_t *)(AT91C_BASE_PWMC + 0x10))
#define PWM_INTERUPT_DISABLE_REGISTER	(*(uint32_t *)(AT91C_BASE_PWMC + 0x14))

#define PWM_CHANNEL_MODE_REGISTER(x)	(uint32_t *)(AT91C_BASE_PWMC + (0x200 + (x * 0x20) + 0x00))
#define PWM_DUTY_REGISTER(x)            (uint32_t *)(AT91C_BASE_PWMC + (0x200 + (x * 0x20) + 0x04))
#define PWM_CHANNEL_UPDATE_REGISTER(x)  (uint32_t *)(AT91C_BASE_PWMC + (0x200 + (x * 0x10) + ((x + 1) * 0x10)))

#define PWM_MILLISECONDS                1000
#define PWM_MICROSECONDS                1000000
#define PWM_NANOSECONDS                 1000000000

#define AT91_MAX_PWM_FREQUENCY          25000000
#define AT91_MIN_PWM_FREQUENCY          1

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

static const AT91_Gpio_Pin g_at91_pwm_pins[TOTAL_PWM_CONTROLLER][MAX_PWM_PER_CONTROLLER] = AT91_PWM_PINS;

AT91_Gpio_Pin AT91_Pwm_GetPins(int32_t controller, int32_t channel) {
    return g_at91_pwm_pins[controller][channel];
}

bool AT91_Pwm_SetPinState(const TinyCLR_Pwm_Provider* self, int32_t pin, bool state) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    AT91_Gpio_EnableOutputPin(actualPin, state);

    return true;
}

TinyCLR_Result AT91_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);
    int32_t controller = self->Index;

    if (!AT91_Gpio_OpenPin(actualPin))
        return TinyCLR_Result::SharingViolation;

    PWM_ENABLE_REGISTER |= (1 << controller);
    PWM_INTERUPT_ENABLE_REGISTER |= (1 << controller);

    AT91_Pwm_SetPinState(self, pin, false);

    return TinyCLR_Result::Success;;
}

TinyCLR_Result AT91_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);
    int32_t controller = self->Index;

    PWM_DISABLE_REGISTER |= (1 << controller);
    PWM_INTERUPT_DISABLE_REGISTER |= (1 << controller);

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

    uint32_t convertedPeriod = 0;

    double frequency = g_PwmController[self->Index].frequency;

    AT91_Pwm_GetScaleFactor(frequency, period, scale);

    switch (scale) {
    case PWM_MILLISECONDS:
        convertedPeriod = (period * 1000000);

        break;

    case PWM_MICROSECONDS:
        convertedPeriod = (period * 1000);

        break;

    case PWM_NANOSECONDS:
        convertedPeriod = period;

        break;
    }

    if (convertedPeriod > 503308801)
        convertedPeriod = 503308801; // max period
    else if (convertedPeriod < 40)
        convertedPeriod = 40; // min period

    switch (scale) {
    case PWM_MILLISECONDS:
        period = (convertedPeriod / 1000000);

        break;

    case PWM_MICROSECONDS:
        period = (convertedPeriod / 1000);

        break;

    case PWM_NANOSECONDS:
        convertedPeriod = period;

        break;
    }

    frequency = (double)(scale / period);

    return frequency;

}

TinyCLR_Result AT91_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    AT91_Gpio_ConfigurePin(actualPin, AT91_Gpio_Direction::Input, g_PwmController[self->Index].gpioPin[pin].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    int32_t actualPin = AT91_Pwm_GetGpioPinForChannel(self, pin);

    AT91_Pwm_SetPinState(self, pin, false);

    return TinyCLR_Result::Success;
}

int32_t AT91_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self) {
    return MAX_PWM_PER_CONTROLLER;
}

int32_t AT91_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin) {
    return g_PwmController[self->Index].gpioPin[pin].number;
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

    uint32_t convertedPeriod = 0;
    uint32_t convertedDuration = 0;
    uint32_t pulseBeginsOnHighEdge = 1; // Default Pulse starts on High Edge.

    uint32_t divider = 4; // Sets the default period/duration clock divider to 4 --> (MCK / divider) // MCK = 100 MHz
    uint32_t registerDividerFlag = 0x2; // Sets the default channel clock divider to 4 --> (MCK / divider) // MCK = 100 MHz

    double frequency = g_PwmController[self->Index].frequency;

    AT91_Pwm_GetScaleFactor(frequency, period, scale);

    duration = (uint32_t)(dutyCycle * period);

    switch (scale) {
    case PWM_MILLISECONDS:
        convertedPeriod = (period * 1000000);
        convertedDuration = (duration * 1000000);
        break;

    case PWM_MICROSECONDS:
        convertedPeriod = (period * 1000);
        convertedDuration = (duration * 1000);
        break;

    case PWM_NANOSECONDS:
        convertedPeriod = period;
        convertedDuration = duration;
        break;
    }

    if (convertedPeriod > 503308801) {
        return TinyCLR_Result::InvalidOperation;
    }
    else if (convertedPeriod > 251654401) {
        divider = 1024;
        registerDividerFlag = 0xA;
    }
    else if (convertedPeriod > 125827200) {
        divider = 512;
        registerDividerFlag = 0x9;
    }
    else if (convertedPeriod > 62913600) {
        divider = 256;
        registerDividerFlag = 0x8;
    }
    else if (convertedPeriod > 31456800) {
        divider = 128;
        registerDividerFlag = 0x7;
    }
    else if (convertedPeriod > 15728400) {
        divider = 64;
        registerDividerFlag = 0x6;
    }
    else if (convertedPeriod > 7864200) {
        divider = 32;
        registerDividerFlag = 0x5;
    }
    else if (convertedPeriod > 3932100) {
        divider = 16;
        registerDividerFlag = 0x4;
    }
    else if (convertedPeriod > 1966050) {
        divider = 8;
        registerDividerFlag = 0x3;
    }
    else if (convertedPeriod > 983025) {
        divider = 4;
        registerDividerFlag = 0x2;
    }
    else if (convertedPeriod > 491513) {
        divider = 2;
        registerDividerFlag = 0x1;
    }
    else if (convertedPeriod >= 40) // Absoulute minimum for the PWM Counter
    {
        divider = 1;
        registerDividerFlag = 0x0;
    }
    else {
        return TinyCLR_Result::InvalidOperation;
    }

    // Flips the pulse
    if (invertPolarity == 0)
        pulseBeginsOnHighEdge = 1;
    else
        pulseBeginsOnHighEdge = 0;

    *g_PwmController[self->Index].channelModeReg = (volatile unsigned long)(registerDividerFlag | (pulseBeginsOnHighEdge << 9) | (1 << 10));
    *g_PwmController[self->Index].channelUpdateReg = (volatile unsigned long)(convertedPeriod / (divider * 7.5));
    *g_PwmController[self->Index].dutyCycleReg = (volatile unsigned long)(convertedDuration / (divider * 7.5));

    g_PwmController[self->Index].invert[pin] = invertPolarity;
    g_PwmController[self->Index].dutyCycle[pin] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result AT91_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency) {
    g_PwmController[self->Index].frequency = frequency;

    // Calculate actual frequency
    frequency = AT91_Pwm_GetActualFrequency(self);

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++)
        if (g_PwmController[self->Index].gpioPin[p].number != PIN_NONE)
            if (AT91_Pwm_SetPulseParameters(self, p, g_PwmController[self->Index].dutyCycle[p], g_PwmController[self->Index].invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Pwm_Acquire(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    AT91_Pwm_ResetController(self->Index);

    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_PWM);

    if (PWM_MODE_REGISTER != PWM_MODE_REGISTER & 0x100) // Checks if clock has been set
        PWM_MODE_REGISTER = (1 << 16);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Pwm_Release(const TinyCLR_Pwm_Provider* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    AT91_Pwm_ResetController(self->Index);

    return TinyCLR_Result::Success;
}

void AT91_Pwm_Reset() {
    for (auto controller = 0; controller < TOTAL_PWM_CONTROLLER; controller++) {
        AT91_Pwm_ResetController(controller);

        for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
            if (g_PwmController[pwmProviders[controller]->Index].gpioPin[p].number != PIN_NONE) {
                // Reset PWM and close pin
                AT91_Pwm_DisablePin(pwmProviders[controller], p);
                AT91_Pwm_ReleasePin(pwmProviders[controller], p);
            }
        }
    }
}

void AT91_Pwm_ResetController(int32_t controller) {
    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        g_PwmController[pwmProviders[controller]->Index].gpioPin[p] = AT91_Pwm_GetPins(controller, p);

        if (g_PwmController[pwmProviders[controller]->Index].gpioPin[p].number != PIN_NONE) {
            // Reset values
            g_PwmController[pwmProviders[controller]->Index].channelModeReg = PWM_CHANNEL_MODE_REGISTER(controller);
            g_PwmController[pwmProviders[controller]->Index].dutyCycleReg = PWM_DUTY_REGISTER(controller);
            g_PwmController[pwmProviders[controller]->Index].channelUpdateReg = PWM_CHANNEL_UPDATE_REGISTER(controller);
            g_PwmController[pwmProviders[controller]->Index].invert[p] = false;
            g_PwmController[pwmProviders[controller]->Index].frequency = 0.0;
            g_PwmController[pwmProviders[controller]->Index].dutyCycle[p] = 0.0;
        }
    }
}
