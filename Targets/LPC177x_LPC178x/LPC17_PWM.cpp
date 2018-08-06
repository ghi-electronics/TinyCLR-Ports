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

#define PWM0MR0 (*(volatile unsigned long *)0x40014018)
#define PWM0MR1 ((uint32_t )0x4001401C)
#define PWM0MR2 ((uint32_t )0x40014020)
#define PWM0MR3 ((uint32_t )0x40014024)
#define PWM0MR4 ((uint32_t )0x40014040)
#define PWM0MR5 ((uint32_t )0x40014044)
#define PWM0MR6 ((uint32_t )0x40014048)

#define PWM1MR0 (*(volatile unsigned long *)0x40018018)
#define PWM1MR1 ((uint32_t )0x4001801C)
#define PWM1MR2 ((uint32_t )0x40018020)
#define PWM1MR3 ((uint32_t )0x40018024)
#define PWM1MR4 ((uint32_t )0x40018040)
#define PWM1MR5 ((uint32_t )0x40018044)
#define PWM1MR6 ((uint32_t )0x40018048)

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

const char* PwmApiNames[] = {
#if TOTAL_PWM_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.LPC17.PwmController\\0",
#if TOTAL_PWM_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.LPC17.PwmController\\1",
#endif
#endif
};

static PwmState pwmStates[TOTAL_PWM_CONTROLLERS];

static TinyCLR_Pwm_Controller pwmControllers[TOTAL_PWM_CONTROLLERS];
static TinyCLR_Api_Info pwmApi[TOTAL_PWM_CONTROLLERS];

void LPC17_Pwm_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_PWM_CONTROLLERS; i++) {
        pwmControllers[i].ApiInfo = &pwmApi[i];
        pwmControllers[i].Acquire = &LPC17_Pwm_Acquire;
        pwmControllers[i].Release = &LPC17_Pwm_Release;
        pwmControllers[i].OpenChannel = &LPC17_Pwm_OpenChannel;
        pwmControllers[i].CloseChannel = &LPC17_Pwm_CloseChannel;
        pwmControllers[i].EnableChannel = &LPC17_Pwm_EnableChannel;
        pwmControllers[i].DisableChannel = &LPC17_Pwm_DisableChannel;
        pwmControllers[i].SetPulseParameters = &LPC17_Pwm_SetPulseParameters;
        pwmControllers[i].SetDesiredFrequency = &LPC17_Pwm_SetDesiredFrequency;
        pwmControllers[i].GetMinFrequency = &LPC17_Pwm_GetMinFrequency;
        pwmControllers[i].GetMaxFrequency = &LPC17_Pwm_GetMaxFrequency;
        pwmControllers[i].GetChannelCount = &LPC17_Pwm_GetChannelCount;

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

TinyCLR_Result LPC17_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, channel);

    if (!LPC17_Gpio_OpenPin(actualPin))
        return TinyCLR_Result::SharingViolation;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    // enable PWM output
    if (state->channel[channel] == 0) {

        // Enable PWMs controllerIndex channel 0
        LPC_SC->PCONP |= PCONP_PCPWM0;

        // Reset Timer Counter
        PWM0TCR |= (1 << 1);
        *state->matchAddress[channel] = 0;
        PWM0MCR = (1 << 1); // Reset on MAT0
        PWM0TCR = 1; // Enable
        PWM0PCR |= (1 << (9 + state->match[channel])); // To enable output on the proper channel
    }
    else if (state->channel[channel] == 1) {

        // Enable PWMs controllerIndex channel 1
        LPC_SC->PCONP |= PCONP_PCPWM1;

        // Reset Timer Counter
        PWM1TCR |= (1 << 1);
        *state->matchAddress[channel] = 0;
        PWM1MCR = (1 << 1); // Reset on MAT0
        PWM1TCR = 1; // Enable
        PWM1PCR |= (1 << (9 + (state->match[channel]))); // To enable output on the proper channel
    }

    state->isOpened[channel] = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, channel);

    LPC17_Gpio_ClosePin(actualPin);

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    state->isOpened[channel] = false;

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

double LPC17_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self) {
    uint32_t period = 0;
    uint32_t scale = 0;

    uint64_t periodInNanoSeconds = 0;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    double frequency = state->frequency;

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

TinyCLR_Result LPC17_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, channel);

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    LPC17_Gpio_ConfigurePin(actualPin, LPC17_Gpio_Direction::Input, state->gpioPin[channel].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    int32_t actualPin = LPC17_Pwm_GetGpioPinForChannel(self, channel);

    LPC17_Gpio_ConfigurePin(actualPin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    return TinyCLR_Result::Success;
}

uint32_t LPC17_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self) {
    return MAX_PWM_PER_CONTROLLER;
}

uint32_t LPC17_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    return state->gpioPin[channel].number;
}

double LPC17_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self) {
    return (LPC17_SYSTEM_CLOCK_HZ / 4);
}

double LPC17_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self) {
    return 1;
}

TinyCLR_Result LPC17_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity) {

    uint32_t period = 0;
    uint32_t duration = 0;
    uint32_t scale = 0;

    uint32_t periodInNanoSeconds = 0;
    uint32_t durationInNanoSeconds = 0;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    double frequency = state->frequency;

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

    // make sure out frequency <= in frequency
    if (periodInNanoSeconds > 0) {
        double freq_out = (double)(1000000000 / periodInNanoSeconds);

        while (freq_out > frequency) {
            periodInNanoSeconds++;
            freq_out = (double)(1000000000 / periodInNanoSeconds);
        }
    }

    uint32_t periodTicks = (uint64_t)(((LPC17_SYSTEM_CLOCK_HZ / 2) / 1000000)) * periodInNanoSeconds / 1000;
    uint32_t highTicks = (uint64_t)(((LPC17_SYSTEM_CLOCK_HZ / 2) / 1000000)) * durationInNanoSeconds / 1000;

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

    if (polarity == TinyCLR_Pwm_PulsePolarity::ActiveLow)
        highTicks = periodTicks - highTicks;

    if (periodInNanoSeconds == 0 || durationInNanoSeconds == 0) {
        LPC17_Gpio_EnableOutputPin(state->gpioPin[channel].number, false);
        state->outputEnabled[channel] = true;

        return TinyCLR_Result::Success;
    }
    else if (durationInNanoSeconds >= periodInNanoSeconds) {
        LPC17_Gpio_EnableOutputPin(state->gpioPin[channel].number, true);
        state->outputEnabled[channel] = true;

        return TinyCLR_Result::Success;
    }
    else {
        if (state->channel[channel] == 0) {
            // Re-scale with new frequency!
            if ((PWM0MR0 != periodTicks)) {

                // Reset Timer Counter
                PWM0TCR |= (1 << 1);
                PWM0MR0 = periodTicks;
                PWM0MCR = (1 << 1); // Reset on MAT0
                PWM0TCR = 1; // Enable
            }

            *state->matchAddress[channel] = highTicks;
        }
        else if (state->channel[channel] == 1) {
            // Re-scale with new frequency!
            if ((PWM1MR0 != periodTicks)) {
                // Reset Timer Counter
                PWM1TCR |= (1 << 1);
                PWM1MR0 = periodTicks;
                PWM1MCR = (1 << 1); // Reset on MAT0
                PWM1TCR = 1; // Enable
            }

            *state->matchAddress[channel] = highTicks;
        }

        if (state->outputEnabled[channel] == true) {
            LPC17_Pwm_EnableChannel(self, channel);

            state->outputEnabled[channel] = false;
        }
    }

    state->invert[channel] = polarity;
    state->dutyCycle[channel] = dutyCycle;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency) {
    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);

    state->frequency = frequency;

    // Calculate actual frequency
    frequency = LPC17_Pwm_GetActualFrequency(self);

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++)
        if (state->gpioPin[p].number != PIN_NONE)
            if (LPC17_Pwm_SetPulseParameters(self, p, state->dutyCycle[p], state->invert[p]) != TinyCLR_Result::Success)
                return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_Acquire(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    LPC17_Pwm_ResetController(controllerIndex);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Pwm_Release(const TinyCLR_Pwm_Controller* self) {
    if (self == nullptr) return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<PwmState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    LPC17_Pwm_ResetController(controllerIndex);

    return TinyCLR_Result::Success;
}

void LPC17_Pwm_Reset() {
    for (auto controllerIndex = 0; controllerIndex < TOTAL_PWM_CONTROLLERS; controllerIndex++) {
        LPC17_Pwm_ResetController(controllerIndex);
    }
}
void LPC17_Pwm_ResetController(int32_t controllerIndex) {
    auto state = &pwmStates[controllerIndex];

    for (int p = 0; p < MAX_PWM_PER_CONTROLLER; p++) {
        state->gpioPin[p] = LPC17_Pwm_GetPins(controllerIndex, p);

        if (state->gpioPin[p].number != PIN_NONE) {
            // Reset values
            state->channel[p] = controllerIndex;
            state->match[p] = p;
            if (p < 3)
                state->matchAddress[p] = controllerIndex == 0 ? (uint32_t*)(PWM0MR1 + (p * 4)) : (uint32_t*)(PWM1MR1 + (p * 4));
            else
                state->matchAddress[p] = controllerIndex == 0 ? (uint32_t*)(PWM0MR4 + ((p - 3) * 4)) : (uint32_t*)(PWM1MR4 + ((p - 3) * 4));

            state->outputEnabled[p] = false;
            state->invert[p] = TinyCLR_Pwm_PulsePolarity::ActiveLow;
            state->frequency = 0.0;
            state->dutyCycle[p] = 0.0;

            if (state->isOpened[p] == true) {
                if (controllerIndex == 0)
                    PWM0PCR &= ~(1 << (9 + (state->match[p])));
                if (controllerIndex == 1)
                    PWM1PCR &= ~(1 << (9 + (state->match[p])));

                LPC17_Pwm_DisableChannel(&pwmControllers[controllerIndex], p);
                LPC17_Pwm_CloseChannel(&pwmControllers[controllerIndex], p);
            }

            state->isOpened[p] = false;
        }
    }
}
