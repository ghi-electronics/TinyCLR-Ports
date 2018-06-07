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

#define GET_PORT(x)                     (x / 32)
#define GET_PIN(x)                      (x % 32)
#define GET_PIN_MASK(x)                 (1<<(x % 32))

#define IOCON_BASE                  ((uint32_t*)0x4002C000)
#define FIODIR(x)                   ((uint32_t*)(0x20098000 + (0x20 * GET_PORT(x))))
#define FIOSET(x)                   ((uint32_t*)(0x20098018 + (0x20 * GET_PORT(x))))
#define FIOCLR(x)                   ((uint32_t*)(0x2009801C + (0x20 * GET_PORT(x))))
#define FIOPIN(x)                   ((uint32_t*)(0x20098014 + (0x20 * GET_PORT(x))))

#define GPIO_INT_RisingEdge(port)               ((uint32_t*)(0X40028090 + (0x10 * port)))
#define GPIO_INT_FallingEdge(port)              ((uint32_t*)(0X40028094 + (0x10 * port)))
#define GPIO_INT_RisingEdge_Status(port)        ((uint32_t*)(0X40028084 + (0x10 * port)))
#define GPIO_INT_FallingEdge_Status(port)       ((uint32_t*)(0X40028088 + (0x10 * port)))
#define GPIO_INT_Clear(port)                    ((uint32_t*)(0X4002808C + (0x10 * port)))
#define GPIO_INT_Overall_IO_Status              ((uint32_t*)0x40028080)


                   // pin  00   01   02   03   04   05   06   07   08   09   10   11   12   13   14   15   16   17   18   19   20   21   22   23   24   25   26   27   28   29   30   31    port
const char IOCON_Type[] = { 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'W', 'W', 'W', 'D', 'D', 'A', 'A', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'A', 'A', 'A', 'A', 'I', 'I', 'U', 'U', 'U', // 0
                            'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'A', 'A', // 1
                            'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', // 2
                            'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', // 3
                            'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', 'D', // 4
                            'D', 'D', 'I', 'I', 'D' };


static const LPC17_Gpio_PinConfiguration g_lpc17_pins[] = LPC17_GPIO_PINS;

#define LPC17_Gpio_PinReserved                 1
#define LPC17_Gpio_DebounceDefaultMilisecond   20
#define LPC17_Gpio_MaxPins                     SIZEOF_ARRAY(g_lpc17_pins)

#define LPC17_GPIO_DEFAULT_CONTROLLER 0

struct LPC17_Int_State {
    uint8_t                                     pin;      // pin number
    int64_t                                    debounce; // debounce
    uint64_t                                    lastDebounceTicks;

    const TinyCLR_Gpio_Provider*                controller; // controller
    TinyCLR_Gpio_ValueChangedHandler            ISR; // interrupt handler
    TinyCLR_Gpio_PinValue                       currentValue;
};

static bool                     g_pinReserved[LPC17_Gpio_MaxPins];
static int64_t                     g_debounceTicksPin[LPC17_Gpio_MaxPins];
static LPC17_Int_State             g_int_state[LPC17_Gpio_MaxPins]; // interrupt state
static TinyCLR_Gpio_PinDriveMode    g_pinDriveMode[LPC17_Gpio_MaxPins];

static TinyCLR_Gpio_Provider gpioProvider;
static TinyCLR_Api_Info gpioApi;

const TinyCLR_Api_Info* LPC17_Gpio_GetApi() {
    gpioProvider.Parent = &gpioApi;
    gpioProvider.Acquire = &LPC17_Gpio_Acquire;
    gpioProvider.Release = &LPC17_Gpio_Release;
    gpioProvider.AcquirePin = &LPC17_Gpio_AcquirePin;
    gpioProvider.ReleasePin = &LPC17_Gpio_ReleasePin;
    gpioProvider.IsDriveModeSupported = &LPC17_Gpio_IsDriveModeSupported;
    gpioProvider.Read = &LPC17_Gpio_Read;
    gpioProvider.Write = &LPC17_Gpio_Write;
    gpioProvider.GetDriveMode = &LPC17_Gpio_GetDriveMode;
    gpioProvider.SetDriveMode = &LPC17_Gpio_SetDriveMode;
    gpioProvider.GetDebounceTimeout = &LPC17_Gpio_GetDebounceTimeout;
    gpioProvider.SetDebounceTimeout = &LPC17_Gpio_SetDebounceTimeout;
    gpioProvider.SetValueChangedHandler = &LPC17_Gpio_SetValueChangedHandler;
    gpioProvider.GetPinCount = &LPC17_Gpio_GetPinCount;

    gpioApi.Author = "GHI Electronics, LLC";
    gpioApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.GpioProvider";
    gpioApi.Type = TinyCLR_Api_Type::GpioProvider;
    gpioApi.Version = 0;
    gpioApi.Implementation = &gpioProvider;

    return &gpioApi;
}

TinyCLR_Result LPC17_Gpio_Acquire(const TinyCLR_Gpio_Provider* self, int32_t controller) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_Release(const TinyCLR_Gpio_Provider* self, int32_t controller) {
    return TinyCLR_Result::Success;
}

void LPC17_Gpio_InterruptHandler(void* param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t* GPIO_INT_Overall_IO_Status_Register = GPIO_INT_Overall_IO_Status;

    bool executeIsr = true;

    for (auto port = 0; port <= 2; port += 2) { // Only port 0 and port 2 support interrupts
        auto status_mask_register = 1 << port;

        uint32_t* GPIO_Port_Interrupt_ClearRegister = GPIO_INT_Clear(port);
        uint32_t* GPIO_Port_INT_RisingEdge_Status_Register = GPIO_INT_RisingEdge_Status(port);
        uint32_t* GPIO_Port_INT_FallingEdge_Status_Register = GPIO_INT_FallingEdge_Status(port);

        for (auto pin = 0; ((*GPIO_INT_Overall_IO_Status_Register) & status_mask_register) && (pin < 32); pin++) {

            if (!(*GPIO_Port_INT_RisingEdge_Status_Register & (0x1 << pin)) && !(*GPIO_Port_INT_FallingEdge_Status_Register & (0x1 << pin))) // If this is not the Pin, skip to next pin
                continue;

            LPC17_Int_State* state = &g_int_state[pin + port * 32];

            *GPIO_Port_Interrupt_ClearRegister |= (0x1 << pin); // Clear this pin's IRQ

            if (state->debounce) {
                if ((LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentProcessorTicks(nullptr)) - state->lastDebounceTicks) >= g_debounceTicksPin[state->pin]) {
                    state->lastDebounceTicks = LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentProcessorTicks(nullptr));
                }
                else {
                    executeIsr = false;
                }
            }

            if (executeIsr) {
                auto gpioController = 0; //TODO Temporary set to 0

                LPC17_Gpio_Read(&gpioProvider, gpioController, state->pin, state->currentValue); // read value as soon as possible

                state->ISR(state->controller, gpioController, state->pin, state->currentValue);
            }
        }
    }
}

TinyCLR_Result LPC17_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR) {
    LPC17_Int_State* state = &g_int_state[pin];

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t port = GET_PORT(pin);
    uint32_t pinMask = GET_PIN_MASK(pin);
    uint32_t* FIODIR_Register = FIODIR(pin);

    uint32_t* GPIO_Port_X_Interrupt_RisingEdgeRegister = GPIO_INT_RisingEdge(port);
    uint32_t* GPIO_Port_X_Interrupt_FallingEdgeRegister = GPIO_INT_FallingEdge(port);

    if (ISR && ((port != 0) && (port != 2))) // If interrupt is called on a non interrupt capable pin return false
        return TinyCLR_Result::ArgumentInvalid;

    LPC17_Gpio_EnableInputPin(pin, g_pinDriveMode[pin]);
    auto gpioController = 0; //TODO Temporary set to 0

    if (ISR) {
        state->controller = &gpioProvider;
        state->pin = (uint8_t)pin;
        state->debounce = LPC17_Gpio_GetDebounceTimeout(self, gpioController, pin);
        state->ISR = ISR;
        state->lastDebounceTicks = LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentProcessorTicks(nullptr));

        *GPIO_Port_X_Interrupt_RisingEdgeRegister |= pinMask;
        *GPIO_Port_X_Interrupt_FallingEdgeRegister |= pinMask;

        LPC17_Interrupt_Activate(GPIO_IRQn, (uint32_t*)&LPC17_Gpio_InterruptHandler, 0);
        LPC17_Interrupt_Enable(GPIO_IRQn);
    }
    else {
        LPC17_Interrupt_Disable(GPIO_IRQn);
    }

    return TinyCLR_Result::Success;
}
bool LPC17_Gpio_Disable_Interrupt(uint32_t pin) {
    return true;
}

bool LPC17_Gpio_OpenPin(int32_t pin) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return false;

    if (g_pinReserved[pin])
        return false;

    g_pinReserved[pin] = true;

    return true;
}

bool LPC17_Gpio_ClosePin(int32_t pin) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return false;

    g_pinReserved[pin] = false;

    // reset to default state
    return LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
}

bool LPC17_Gpio_ConfigurePin(int32_t pin, LPC17_Gpio_Direction pinDir, LPC17_Gpio_PinFunction alternateFunction, LPC17_Gpio_ResistorMode pullResistor, LPC17_Gpio_Hysteresis hysteresis, LPC17_Gpio_InputPolarity inputPolarity, LPC17_Gpio_SlewRate slewRate, LPC17_Gpio_OutputType outputType) {
    uint32_t* FIODIR_Register = FIODIR(pin);

    switch (pinDir) {
    case LPC17_Gpio_Direction::Output:
        *FIODIR_Register |= GET_PIN_MASK(pin);
        break;

    default:
        *FIODIR_Register &= ~GET_PIN_MASK(pin);
        break;
    }

    uint32_t* IOCON_Register = IOCON_BASE;

    uint32_t digitalMode = 0;
    uint32_t analogMode = 0; // Default is False; Only one AnalogOut pin on whole processor.

    IOCON_Register += pin;

    switch (IOCON_Type[pin]) {
    case 'D':
        *IOCON_Register &= 0xFFFFFFE0; // Clear mask to clear pullResistor and Alt Function before resetting
        *IOCON_Register |= ((uint8_t)pullResistor << 3) | ((uint8_t)alternateFunction);

        if (pinDir == LPC17_Gpio_Direction::Input) {
            *IOCON_Register |= ((uint8_t)hysteresis << 5);
            *IOCON_Register |= ((uint8_t)inputPolarity << 6);
        }

        if (pinDir == LPC17_Gpio_Direction::Output) {
            *IOCON_Register |= ((uint8_t)slewRate << 9);
            *IOCON_Register |= ((uint8_t)outputType << 10);
        }

        break;
    case 'A':
        *IOCON_Register &= 0xFFFEFF60; // Clear mask to clear pullResistor, Alt Function, Digital Status, and ADC/DAC Mode before resetting

        // If pin is GPIO then the pin must be set to Digital Mode Vs. Analog Mode
        if (alternateFunction == LPC17_Gpio_PinFunction::PinFunction0 || (alternateFunction == LPC17_Gpio_PinFunction::PinFunction3 && (pin == PIN(0, 25) || pin == PIN(0, 26))))
            digitalMode = 1; // pin is set Digital Mode
        else
            digitalMode = 0; // pin is set to Analog Mode

        // If pin is to be Digital to Analog Converter, then the pin must be set to Digital to Analog Mode Vs. Analog to Digital Mode
        if (alternateFunction == LPC17_Gpio_PinFunction::PinFunction2)
            analogMode = 1; // pin is set Digital to Analog Mode
        else
            analogMode = 0; // pin is set to Analog to Digital Mode

        *IOCON_Register |= (analogMode << 16) | (digitalMode << 7) | ((uint8_t)pullResistor << 3) | ((uint8_t)alternateFunction);
        break;
    case 'U':
        *IOCON_Register &= 0xFFFFFFF8; // Clear mask to clear Alt Function before resetting
        *IOCON_Register |= ((uint8_t)alternateFunction);
        break;
    case 'I':
        *IOCON_Register &= 0xFFFFFFF8; // Clear mask to clear Alt Function before resetting
        *IOCON_Register |= ((uint8_t)alternateFunction);
        break;
    case 'W':
        *IOCON_Register &= 0xFFFFFFE0; // Clear mask to clear pullResistor and Alt Function before resetting
        *IOCON_Register |= ((uint8_t)pullResistor << 3) | ((uint8_t)alternateFunction);
        break;
    }
}

bool LPC17_Gpio_ReadPin(int32_t pin) {
    uint32_t* FIOPIN_Register = FIOPIN(pin);

    if ((*FIOPIN_Register & GET_PIN_MASK(pin)) != 0)
        return true;

    return false;
}
void LPC17_Gpio_WritePin(int32_t pin, bool value) {
    uint32_t* FIOSET_Register = FIOSET(pin);
    uint32_t* FIOCLR_Register = FIOCLR(pin);

    if (value) {
        *FIOSET_Register |= GET_PIN_MASK(pin);
    }
    else {
        *FIOCLR_Register |= GET_PIN_MASK(pin);
    }
}

void LPC17_Gpio_EnableOutputPin(int32_t pin, bool initialState) {
    LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Output, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_WritePin(pin, initialState);
}

void LPC17_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {
    LPC17_Gpio_ResistorMode resistorMode = LPC17_Gpio_ResistorMode::Inactive;

    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        resistorMode = LPC17_Gpio_ResistorMode::PullUp;
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        resistorMode = LPC17_Gpio_ResistorMode::PullDown;
        break;
    }

    LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, resistorMode, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

}

TinyCLR_Result LPC17_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin, TinyCLR_Gpio_PinValue& value) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (LPC17_Gpio_ReadPin(pin))
        value = TinyCLR_Gpio_PinValue::High;
    else
        value = TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin, TinyCLR_Gpio_PinValue value) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (value == TinyCLR_Gpio_PinValue::High)
        LPC17_Gpio_WritePin(pin, true);
    else
        LPC17_Gpio_WritePin(pin, false);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC17_Gpio_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    LPC17_Gpio_ClosePin(pin);

    return TinyCLR_Result::Success;
}

bool LPC17_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {

    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
    case TinyCLR_Gpio_PinDriveMode::Input:
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        return true;
    }

    return false;
}

TinyCLR_Gpio_PinDriveMode LPC17_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin) {
    return g_pinDriveMode[pin];
}

TinyCLR_Result LPC17_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    if (pin >= LPC17_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    switch (driveMode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
        LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Output, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        break;

    case TinyCLR_Gpio_PinDriveMode::Input:
        LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::PullUp, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        LPC17_Gpio_ConfigurePin(pin, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction0, LPC17_Gpio_ResistorMode::PullDown, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
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

uint64_t LPC17_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin) {
    return g_debounceTicksPin[pin];
}

TinyCLR_Result LPC17_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t controller, int32_t pin, uint64_t debounceTicks) {
    g_debounceTicksPin[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

int32_t LPC17_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self, int32_t controller) {
    return LPC17_Gpio_MaxPins;
}

void LPC17_Gpio_Reset() {
    uint32_t* GPIO_Port_0_INT_RisingEdge_Register = GPIO_INT_RisingEdge(0);
    uint32_t* GPIO_Port_0_INT_FallingEdge_Register = GPIO_INT_FallingEdge(0);
    uint32_t* GPIO_Port_2_INT_RisingEdge_Register = GPIO_INT_RisingEdge(2);
    uint32_t* GPIO_Port_2_INT_FallingEdge_Register = GPIO_INT_FallingEdge(2);

    auto gpioController = 0; //TODO Temporary set to 0

    for (auto pin = 0; pin < LPC17_Gpio_GetPinCount(&gpioProvider, gpioController); pin++) {
        auto& p = g_lpc17_pins[pin];

        g_pinReserved[pin] = 0;
        LPC17_Gpio_SetDebounceTimeout(&gpioProvider, gpioController, pin, LPC17_Gpio_DebounceDefaultMilisecond);

        if (p.apply) {
            LPC17_Gpio_ConfigurePin(pin, p.direction, p.pinFunction, p.resistorMode, p.hysteresis, p.inputPolarity, p.slewRate, p.outputType);

            if (p.direction == LPC17_Gpio_Direction::Output)
                LPC17_Gpio_WritePin(pin, p.outputDirection);
        }
    }

    *GPIO_Port_0_INT_RisingEdge_Register = 0x0;
    *GPIO_Port_0_INT_FallingEdge_Register = 0x0;
    *GPIO_Port_2_INT_RisingEdge_Register = 0x0;
    *GPIO_Port_2_INT_FallingEdge_Register = 0x0;
}

TinyCLR_Result LPC17_Gpio_GetControllerCount(const TinyCLR_Gpio_Provider* self, int32_t& count) {
    count = 1;

    return TinyCLR_Result::Success;
}
