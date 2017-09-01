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

// Driver
#define AT91_Gpio_DebounceDefaultMilisecond   20
#define AT91_Gpio_MaxPins                     TOTAL_GPIO_PINS


struct AT91_Int_State {
    uint8_t                                     pin;      // pin number
    uint32_t                                    debounce; // debounce
    uint64_t                                    lastDebounceTicks;

    const TinyCLR_Gpio_Provider*                controller; // controller
    TinyCLR_Gpio_ValueChangedHandler            ISR; // interrupt handler
    TinyCLR_Gpio_PinValue                       currentValue;
};

static bool                     g_pinReserved[AT91_Gpio_MaxPins];
static uint64_t                     g_debounceTicksPin[AT91_Gpio_MaxPins];
static AT91_Int_State              g_int_state[AT91_Gpio_MaxPins]; // interrupt state
static TinyCLR_Gpio_PinDriveMode    g_pinDriveMode[AT91_Gpio_MaxPins];

static TinyCLR_Gpio_Provider gpioProvider;
static TinyCLR_Api_Info gpioApi;

const TinyCLR_Api_Info* AT91_Gpio_GetApi() {
    gpioProvider.Parent = &gpioApi;
    gpioProvider.Index = 0;
    gpioProvider.Acquire = &AT91_Gpio_Acquire;
    gpioProvider.Release = &AT91_Gpio_Release;
    gpioProvider.AcquirePin = &AT91_Gpio_AcquirePin;
    gpioProvider.ReleasePin = &AT91_Gpio_ReleasePin;
    gpioProvider.IsDriveModeSupported = &AT91_Gpio_IsDriveModeSupported;
    gpioProvider.Read = &AT91_Gpio_Read;
    gpioProvider.Write = &AT91_Gpio_Write;
    gpioProvider.GetDriveMode = &AT91_Gpio_GetDriveMode;
    gpioProvider.SetDriveMode = &AT91_Gpio_SetDriveMode;
    gpioProvider.GetDebounceTimeout = &AT91_Gpio_GetDebounceTimeout;
    gpioProvider.SetDebounceTimeout = &AT91_Gpio_SetDebounceTimeout;
    gpioProvider.SetValueChangedHandler = &AT91_Gpio_SetValueChangedHandler;
    gpioProvider.GetPinCount = &AT91_Gpio_GetPinCount;

    gpioApi.Author = "GHI Electronics, LLC";
    gpioApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.GpioProvider";
    gpioApi.Type = TinyCLR_Api_Type::GpioProvider;
    gpioApi.Version = 0;
    gpioApi.Count = 1;
    gpioApi.Implementation = &gpioProvider;

    return &gpioApi;
}

TinyCLR_Result AT91_Gpio_Acquire(const TinyCLR_Gpio_Provider* self) {
    AT91_Gpio_Reset();

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Gpio_Release(const TinyCLR_Gpio_Provider* self) {
    AT91_Gpio_Reset();

    return TinyCLR_Result::Success;
}

void AT91_Gpio_InterruptHandler(void* param) {
    INTERRUPT_START

    GLOBAL_LOCK(irq);

    bool executeIsr = true;

    for (auto port = 0; port <= 2; port += 2) { // Only port 0 and port 2 support interrupts
        auto status_mask_register = 1 << port;

        for (auto pin = 0; ((*GPIO_INTERRUPT_STATUS_REG) & status_mask_register) && (pin < 32); pin++) {

            if (!(GET_PIN_INTERRUPT_RISING_EDGE_STATUS(port, pin)) && !(GET_PIN_INTERRUPT_FALLING_EDGE_STATUS(port, pin))) // If this is not the Pin, skip to next pin
                continue;

            AT91_Int_State* state = &g_int_state[pin + port * 32];

            CLEAR_PIN_INTERRUPT(port, pin); // Clear this pin's IRQ

            if (state->debounce) {
                if ((AT91_Time_GetCurrentTicks(nullptr) - state->lastDebounceTicks) >= g_debounceTicksPin[state->pin]) {
                    state->lastDebounceTicks = AT91_Time_GetCurrentTicks(nullptr);
                }
                else {
                    executeIsr = false;
                }
            }

            if (executeIsr)
                state->ISR(state->controller, state->pin, state->currentValue);
        }
    }

    INTERRUPT_END
}

TinyCLR_Result AT91_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR) {
    AT91_Int_State* state = &g_int_state[pin];

    GLOBAL_LOCK(irq);

    uint32_t port = GET_PORT(pin);
    uint32_t pinMask = GET_PIN_MASK(pin);
    
    AT91_Gpio_EnableInputPin(pin, g_pinDriveMode[pin]);

    if (ISR) {
        state->controller = &gpioProvider;
        state->pin = (uint8_t)pin;
        state->debounce = AT91_Gpio_GetDebounceTimeout(self, pin);
        state->ISR = ISR;
        state->lastDebounceTicks = AT91_Time_GetCurrentTicks(nullptr);

    }
    else {

    }

    return TinyCLR_Result::Success;
}
bool AT91_Gpio_Disable_Interrupt(uint32_t pin) {
    return true;
}

bool AT91_Gpio_OpenPin(int32_t pin) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return false;

    if (g_pinReserved[pin])
        return false;

    g_pinReserved[pin] = true;

    return true;
}

bool AT91_Gpio_ClosePin(int32_t pin) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return false;

    g_pinReserved[pin] = false;

    // reset to default state
    return AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
}

bool AT91_Gpio_ReadPin(int32_t pin) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return false;

    return true;
}

void AT91_Gpio_WritePin(int32_t pin, bool value) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return;

    if (value){
	
    }    
    else {
    
	}
}

bool AT91_Gpio_ConfigurePin(int32_t pin, AT91_Gpio_Direction pinDir, AT91_Gpio_PinFunction alternateFunction, AT91_Gpio_PinMode pullResistor) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return false;

    switch (alternateFunction) {
        case AT91_Gpio_PinFunction::PinFunction0:
            if (pinDir == AT91_Gpio_Direction::Output) {
    
    }
            else {
                switch (pullResistor) {
                    case AT91_Gpio_PinMode::Inactive:

                    break;

                    case AT91_Gpio_PinMode::PullUp:

                    break;

                    case AT91_Gpio_PinMode::PullDown:

                    break;

                }
            }

            break;

        case AT91_Gpio_PinFunction::PinFunction1:

            break;

        case AT91_Gpio_PinFunction::PinFunction2:
            
            break;

        case AT91_Gpio_PinFunction::PinFunction3:
            
            break;
    }

    return true;
}

void AT91_Gpio_EnableOutputPin(int32_t pin, bool initialState) {
    AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Output, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
    AT91_Gpio_WritePin(pin, initialState);
}

void AT91_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {
    switch (mode) {
        case TinyCLR_Gpio_PinDriveMode::Input:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
            break;
        case TinyCLR_Gpio_PinDriveMode::InputPullUp:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::PullUp);
            break;
        case TinyCLR_Gpio_PinDriveMode::InputPullDown:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::PullDown);
            break;
    }
}

TinyCLR_Result AT91_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue& value) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    value = AT91_Gpio_ReadPin(pin) ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue value) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    AT91_Gpio_WritePin(pin, value == TinyCLR_Gpio_PinValue::High ? true : false);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t pin) {

    GLOBAL_LOCK(irq);

    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!AT91_Gpio_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin) {

    GLOBAL_LOCK(irq);

    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    AT91_Gpio_ClosePin(pin);

    return TinyCLR_Result::Success;
}

bool AT91_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {

    switch (mode) {
        case TinyCLR_Gpio_PinDriveMode::Output:
        case TinyCLR_Gpio_PinDriveMode::Input:
        case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        case TinyCLR_Gpio_PinDriveMode::InputPullDown:
            return true;
    }

    return false;
}

TinyCLR_Gpio_PinDriveMode AT91_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin) {
    return g_pinDriveMode[pin];
}

TinyCLR_Result AT91_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    switch (driveMode) {
        case TinyCLR_Gpio_PinDriveMode::Output:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Output, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
            break;

        case TinyCLR_Gpio_PinDriveMode::Input:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::Inactive);
            break;

        case TinyCLR_Gpio_PinDriveMode::InputPullUp:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::PullUp);
            break;

        case TinyCLR_Gpio_PinDriveMode::InputPullDown:
            AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinMode::PullDown);
            break;

        case TinyCLR_Gpio_PinDriveMode::OutputOpenDrain:
        case TinyCLR_Gpio_PinDriveMode::OutputOpenDrainPullUp:
        case TinyCLR_Gpio_PinDriveMode::OutputOpenSource:
        case TinyCLR_Gpio_PinDriveMode::OutputOpenSourcePullDown:
        default:
            return  TinyCLR_Result::NotSupported;
    }

    g_pinDriveMode[pin] = driveMode;

    return TinyCLR_Result::Success;
}

int32_t AT91_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin) {
    return (int32_t)(g_debounceTicksPin[pin] / (SLOW_CLOCKS_PER_SECOND / 1000)); // ticks -> ms
}

TinyCLR_Result AT91_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, int32_t debounceTime) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (debounceTime > 0 && debounceTime < 10000) {
        g_debounceTicksPin[pin] = AT91_Time_MillisecondsToTicks(nullptr, (uint64_t)debounceTime);
        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::WrongType;
}

int32_t AT91_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self) {
    return AT91_Gpio_MaxPins;
}

void AT91_Gpio_Reset() {

    for (auto pin = 0; pin < AT91_Gpio_GetPinCount(&gpioProvider); pin++) {
        g_pinReserved[pin] = false;
        AT91_Gpio_SetDebounceTimeout(&gpioProvider, pin, AT91_Gpio_DebounceDefaultMilisecond);
    }
}
