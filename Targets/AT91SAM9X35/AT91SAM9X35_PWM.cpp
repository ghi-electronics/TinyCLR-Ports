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

#include "AT91SAM9X35.h"

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

#define AT91SAM9X35_MAX_PWM_FREQUENCY          25000000
#define AT91SAM9X35_MIN_PWM_FREQUENCY          1

#define PWM_MILLISECONDS  1000
#define PWM_MICROSECONDS  1000000
#define PWM_NANOSECONDS   1000000000

const char* PwmApiNames[] = {
#if TOTAL_PWM_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9X35.PwmController\\0",
#if TOTAL_PWM_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9X35.PwmController\\1",
#if TOTAL_PWM_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9X35.PwmController\\2",
#if TOTAL_PWM_CONTROLLERS > 3
"GHIElectronics.TinyCLR.NativeApis.AT91SAM9X35.PwmController\\3",
#endif
#endif
#endif
#endif
};

static PwmState pwmStates[TOTAL_PWM_CONTROLLERS];

static TinyCLR_Pwm_Controller pwmControllers[TOTAL_PWM_CONTROLLERS];
static TinyCLR_Api_Info pwmApi[TOTAL_PWM_CONTROLLERS];

void AT91SAM9X35_Pwm_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_PWM_CONTROLLERS; i++) {
        pwmControllers[i].ApiInfo = &pwmApi[i];
        pwmControllers[i].Acquire = &AT91SAM9X35_Pwm_Acquire;
        pwmControllers[i].Release = &AT91SAM9X35_Pwm_Release;
        pwmControllers[i].OpenChannel = &AT91SAM9X35_Pwm_OpenChannel;
        pwmControllers[i].CloseChannel = &AT91SAM9X35_Pwm_CloseChannel;
        pwmControllers[i].EnableChannel = &AT91SAM9X35_Pwm_EnableChannel;
        pwmControllers[i].DisableChannel = &AT91SAM9X35_Pwm_DisableChannel;
        pwmControllers[i].SetPulseParameters = &AT91SAM9X35_Pwm_SetPulseParameters;
        pwmControllers[i].SetDesiredFrequency = &AT91SAM9X35_Pwm_SetDesiredFrequency;
        pwmControllers[i].GetMinFrequency = &AT91SAM9X35_Pwm_GetMinFrequency;
        pwmControllers[i].GetMaxFrequency = &AT91SAM9X35_Pwm_GetMaxFrequency;
        pwmControllers[i].GetChannelCount = &AT91SAM9X35_Pwm_GetChannelCount;

        pwmApi[i].Author = "GHI Electronics, LLC";
        pwmApi[i].Name = PwmApiNames[i];
        pwmApi[i].Type = TinyCLR_Api_Type::PwmController;
        pwmApi[i].Version = 0;
        pwmApi[i].Implementation = &pwmControllers[i];
        pwmApi[i].State = &pwmStates[i];

        pwmStates[i].controllerIndex = i;

        apiManager->Add(apiManager, &pwmApi[i]);
    }
}

static const AT91SAM9X35_Gpio_Pin pwmPins[TOTAL_PWM_CONTROLLERS][MAX_PWM_PER_CONTROLLER] = AT91SAM9X35_PWM_PINS;

AT91SAM9X35_Gpio_Pin AT91SAM9X35_Pwm_GetPins(int32_t controllerIndex, int32_t channel) {
    return pwmPins[controllerIndex][channel];
}

bool AT91SAM9X35_Pwm_SetPinState(const TinyCLR_Pwm_Controller* self, uint32_t channel, bool state) {
    int32_t actualPin = AT91SAM9X35_Pwm_GetGpioPinForChannel(self, channel);

    AT91SAM9X35_GpioInternal_EnableOutputPin(actualPin, state);

    return true;
}

TinyCLR_Result AT91SAM9X35_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = AT91SAM9X35_Pwm_GetGpioPinForChannel(self, channel);

    if (!AT91SAM9X35_GpioInternal_OpenPin(actualPin))
        return TinyCLR_Result::SharingViolation;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    PWM_ENABLE_REGISTER |= (1 << controllerIndex);
    PWM_INTERUPT_ENABLE_REGISTER |= (1 << controllerIndex);

    AT91SAM9X35_Pwm_SetPinState(self, channel, false);

    state->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9X35_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = AT91SAM9X35_Pwm_GetGpioPinForChannel(self, channel);

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    PWM_DISABLE_REGISTER |= (1 << controllerIndex);
    PWM_INTERUPT_DISABLE_REGISTER |= (1 << controllerIndex);

    AT91SAM9X35_GpioInternal_ClosePin(actualPin);

    state->isOpened[channel] = false;

    return TinyCLR_Result::Success;
}

void AT91SAM9X35_Pwm_GetScaleFactor(double frequency, uint32_t& period, uint32_t& scale) {
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

double AT91SAM9X35_Pwm_GetMaxPeriod(uint32_t divider) {
    switch (divider) {
    case 1024:
        return 503308801.0;
    case 512:
        return 251654401.0;
    case 256:
        return 125827200.0;
    case 128:
        return 62913600.0;
    case 64:
        return 31456800.0;
    case 32:
        return 15728400.0;
    case 16:
        return 7864200.0;
    case 8:
        return 3932100.0;
    case 4:
        return 1966050.0;
    case 2:
        return 983025.0;
    case 1:
        return 491513.0;
    }
}

void AT91SAM9X35_Pwm_GetDivider(uint32_t period, uint32_t& divider, uint32_t& registerDividerFlag) {
    if (period > 503308801) {
        divider = 0;
        registerDividerFlag = 0;
    }
    else if (period > 251654401) {
        divider = 1024;
        registerDividerFlag = 0xA;
    }
    else if (period > 125827200) {
        divider = 512;
        registerDividerFlag = 0x9;
    }
    else if (period > 62913600) {
        divider = 256;
        registerDividerFlag = 0x8;
    }
    else if (period > 31456800) {
        divider = 128;
        registerDividerFlag = 0x7;
    }
    else if (period > 15728400) {
        divider = 64;
        registerDividerFlag = 0x6;
    }
    else if (period > 7864200) {
        divider = 32;
        registerDividerFlag = 0x5;
    }
    else if (period > 3932100) {
        divider = 16;
        registerDividerFlag = 0x4;
    }
    else if (period > 1966050) {
        divider = 8;
        registerDividerFlag = 0x3;
    }
    else if (period > 983025) {
        divider = 4;
        registerDividerFlag = 0x2;
    }
    else if (period > 491513) {
        divider = 2;
        registerDividerFlag = 0x1;
    }
    else if (period >= 40) // Absoulute minimum for the PWM Counter
    {
        divider = 1;
        registerDividerFlag = 0x0;
    }
    else {
        divider = 0;
        registerDividerFlag = 0;
    }
}
uint32_t AT91SAM9X35_Pwm_GetPeriod(const TinyCLR_Pwm_Controller* self, uint32_t period, uint32_t divider) {
    // make sure out frequency <= in frequency
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    double d = divider * 7.6;
    double p = (double)period;
    double mck_clk = (double)AT91SAM9X35_SYSTEM_PERIPHERAL_CLOCK_HZ;
    double freq_out = (double)(mck_clk / (p / (d)));
    double frequency = state->frequency;

    if ((p > 0) && (freq_out > frequency)) {

        while (freq_out > frequency) {
            p += d;

            if (p >= AT91SAM9X35_Pwm_GetMaxPeriod(divider)) {
                break;
            }

            freq_out = (double)(mck_clk / (p / (d)));
        }

        // Update new period
        period = (volatile unsigned long)p;
    }

    return period;
}

double AT91SAM9X35_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    uint32_t convertedPeriod = 0;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    double frequency = state->frequency;

    AT91SAM9X35_Pwm_GetScaleFactor(frequency, period, scale);

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

    // make sure out frequency <= in frequency
    uint32_t divider;
    uint32_t registerDividerFlag;

    AT91SAM9X35_Pwm_GetDivider(convertedPeriod, divider, registerDividerFlag);

    if (divider > 0)
        convertedPeriod = AT91SAM9X35_Pwm_GetPeriod(self, convertedPeriod, divider);

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
        period = convertedPeriod;
        break;
    }

    frequency = (double)(scale / period);

    return frequency;
}

TinyCLR_Result AT91SAM9X35_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = AT91SAM9X35_Pwm_GetGpioPinForChannel(self, channel);

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    AT91SAM9X35_GpioInternal_ConfigurePin(actualPin, AT91SAM9X35_Gpio_Direction::Input, state->gpioPin[channel].peripheralSelection, AT91SAM9X35_Gpio_ResistorMode::Inactive);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9X35_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = AT91SAM9X35_Pwm_GetGpioPinForChannel(self, channel);

    AT91SAM9X35_Pwm_SetPinState(self, channel, false);

    return TinyCLR_Result::Success;
}

uint32_t AT91SAM9X35_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self) {
    return MAX_PWM_PER_CONTROLLER;
}

uint32_t AT91SAM9X35_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    return state->gpioPin[channel].number;
}

double AT91SAM9X35_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self) {
    return AT91SAM9X35_MAX_PWM_FREQUENCY;
}

double AT91SAM9X35_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self) {
    return AT91SAM9X35_MIN_PWM_FREQUENCY;
}

TinyCLR_Result AT91SAM9X35_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity) {
    uint32_t period = 0;
    uint32_t scale = 0;

    uint32_t convertedPeriod = 0;
    uint32_t convertedDuration = 0;
    uint32_t pulseBeginsOnHighEdge = 1; // Default Pulse starts on High Edge.

    uint32_t divider = 4; // Sets the default period/duration clock divider to 4 --> (MCK / divider) // MCK = 100 MHz
    uint32_t registerDividerFlag = 0x2; // Sets the default channel clock divider to 4 --> (MCK / divider) // MCK = 100 MHz

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    double frequency = state->frequency;

    // We save dutyCycle and polarity to state's variables,
    // but don't store to PWM registers if frequency is not initialize yet (freq == 0.0)
    if (frequency > 0.0) {
        AT91SAM9X35_Pwm_GetScaleFactor(frequency, period, scale);

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

        AT91SAM9X35_Pwm_GetDivider(convertedPeriod, divider, registerDividerFlag);

        if (divider > 0)
            convertedPeriod = AT91SAM9X35_Pwm_GetPeriod(self, convertedPeriod, divider);

        convertedDuration = (uint32_t)(dutyCycle * convertedPeriod);

        // Flips the pulse
        if (polarity == TinyCLR_Pwm_PulsePolarity::ActiveLow)
            pulseBeginsOnHighEdge = 1;
        else
            pulseBeginsOnHighEdge = 0;

        *state->channelModeReg = (volatile unsigned long)(registerDividerFlag | (pulseBeginsOnHighEdge << 9) | (1 << 10));
        *state->channelUpdateReg = (volatile unsigned long)(convertedPeriod / (divider * 7.5));
        *state->dutyCycleReg = (volatile unsigned long)(convertedDuration / (divider * 7.5));
    }

    state->invert[channel] = polarity;
    state->dutyCycle[channel] = dutyCycle;

    return TinyCLR_Result::Success;

}

TinyCLR_Result AT91SAM9X35_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    state->frequency = frequency;

    // Calculate actual frequency
    frequency = AT91SAM9X35_Pwm_GetActualFrequency(self);

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++)
        if (state->gpioPin[p].number != PIN_NONE)
            if (AT91SAM9X35_Pwm_SetPulseParameters(self, p, state->dutyCycle[p], state->invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9X35_Pwm_Acquire(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        AT91SAM9X35_Pwm_ResetController(state->controllerIndex);

        AT91SAM9X35_PMC &pmc = AT91::PMC();
        pmc.EnablePeriphClock(AT91C_ID_PWM);

        if (PWM_MODE_REGISTER != PWM_MODE_REGISTER & 0x100) // Checks if clock has been set
            PWM_MODE_REGISTER = (1 << 16);
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9X35_Pwm_Release(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0)
        AT91SAM9X35_Pwm_ResetController(state->controllerIndex);

    return TinyCLR_Result::Success;
}

void AT91SAM9X35_Pwm_Reset() {
    for (auto controllerIndex = 0; controllerIndex < TOTAL_PWM_CONTROLLERS; controllerIndex++) {
        AT91SAM9X35_Pwm_ResetController(controllerIndex);
        pwmStates[controllerIndex].initializeCount = 0;
    }
}

void AT91SAM9X35_Pwm_ResetController(int32_t controllerIndex) {
    auto state = &pwmStates[controllerIndex];

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        state->gpioPin[p] = AT91SAM9X35_Pwm_GetPins(controllerIndex, p);

        if (state->gpioPin[p].number != PIN_NONE) {
            // Reset values
            state->channelModeReg = PWM_CHANNEL_MODE_REGISTER(controllerIndex);
            state->dutyCycleReg = PWM_DUTY_REGISTER(controllerIndex);
            state->channelUpdateReg = PWM_CHANNEL_UPDATE_REGISTER(controllerIndex);
            state->invert[p] = TinyCLR_Pwm_PulsePolarity::ActiveLow;
            state->frequency = 0.0;
            state->dutyCycle[p] = 0.0;

            if (state->isOpened[p] == true) {
                AT91SAM9X35_Pwm_DisableChannel(&pwmControllers[controllerIndex], p);
                AT91SAM9X35_Pwm_CloseChannel(&pwmControllers[controllerIndex], p);
            }

            state->isOpened[p] = false;
        }
    }
}
