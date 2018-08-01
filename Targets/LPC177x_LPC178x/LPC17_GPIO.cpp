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


static const LPC17_Gpio_PinConfiguration gpioPins[] = LPC17_GPIO_PINS;

#define TOTAL_GPIO_CONTROLLERS 1

#define TOTAL_GPIO_PINS SIZEOF_ARRAY(gpioPins)

#define TOTAL_GPIO_INTERRUPT_PINS TOTAL_GPIO_PINS

#define DEBOUNCE_DEFAULT_TICKS     (20*10000) // 20ms in ticks

#define PIN_RESERVED 1

struct GpioInterruptState {
    uint8_t pin;
    int64_t debounce;
    uint64_t  lastDebounceTicks;

    const TinyCLR_Gpio_Controller* controller;
    TinyCLR_Gpio_PinChangedHandler handler;
    TinyCLR_Gpio_PinValue currentValue;
};

struct GpioState {
    int32_t controllerIndex;
    bool tableInitialized = false;
};

static GpioState gpioStates[TOTAL_GPIO_CONTROLLERS];

static bool pinReserved[TOTAL_GPIO_PINS];
static int64_t gpioDebounceInTicks[TOTAL_GPIO_PINS];
static GpioInterruptState gpioInterruptState[TOTAL_GPIO_INTERRUPT_PINS];
static TinyCLR_Gpio_PinDriveMode pinDriveMode[TOTAL_GPIO_PINS];

static TinyCLR_Gpio_Controller gpioControllers[TOTAL_GPIO_CONTROLLERS];
static TinyCLR_Api_Info gpioApi[TOTAL_GPIO_CONTROLLERS];

const char* GpioApiNames[TOTAL_GPIO_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC17.GpioController\\0"
};

void LPC17_Gpio_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_GPIO_CONTROLLERS; i++) {
        if (gpioStates[i].tableInitialized)
            continue;

        gpioControllers[i].ApiInfo = &gpioApi[i];
        gpioControllers[i].Acquire = &LPC17_Gpio_Acquire;
        gpioControllers[i].Release = &LPC17_Gpio_Release;
        gpioControllers[i].OpenPin = &LPC17_Gpio_OpenPin;
        gpioControllers[i].ClosePin = &LPC17_Gpio_ClosePin;
        gpioControllers[i].Read = &LPC17_Gpio_Read;
        gpioControllers[i].Write = &LPC17_Gpio_Write;
        gpioControllers[i].IsDriveModeSupported = &LPC17_Gpio_IsDriveModeSupported;
        gpioControllers[i].GetDriveMode = &LPC17_Gpio_GetDriveMode;
        gpioControllers[i].SetDriveMode = &LPC17_Gpio_SetDriveMode;
        gpioControllers[i].GetDebounceTimeout = &LPC17_Gpio_GetDebounceTimeout;
        gpioControllers[i].SetDebounceTimeout = &LPC17_Gpio_SetDebounceTimeout;
        gpioControllers[i].SetPinChangedHandler = &LPC17_Gpio_SetPinChangedHandler;
        gpioControllers[i].GetPinCount = &LPC17_Gpio_GetPinCount;

        gpioApi[i].Author = "GHI Electronics, LLC";
        gpioApi[i].Name = GpioApiNames[i];
        gpioApi[i].Type = TinyCLR_Api_Type::GpioController;
        gpioApi[i].Version = 0;
        gpioApi[i].Implementation = &gpioControllers[i];
        gpioApi[i].State = &gpioStates[i];

        gpioStates[i].controllerIndex = i;
        gpioStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* LPC17_Gpio_GetRequiredApi() {
    LPC17_Gpio_EnsureTableInitialized();

    return &gpioApi[0];
}

void LPC17_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager) {
    LPC17_Gpio_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_GPIO_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &gpioApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::GpioController, LPC17_Gpio_GetRequiredApi()->Name);
}

TinyCLR_Result LPC17_Gpio_Acquire(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_Release(const TinyCLR_Gpio_Controller* self) {
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

            GpioInterruptState* interruptState = &gpioInterruptState[pin + port * 32];

            *GPIO_Port_Interrupt_ClearRegister |= (0x1 << pin); // Clear this pin's IRQ

            if (interruptState->debounce) {
                if ((LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentProcessorTicks(nullptr)) - interruptState->lastDebounceTicks) >= gpioDebounceInTicks[interruptState->pin]) {
                    interruptState->lastDebounceTicks = LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentProcessorTicks(nullptr));
                }
                else {
                    executeIsr = false;
                }
            }

            if (executeIsr) {
                LPC17_Gpio_Read(interruptState->controller, interruptState->pin, interruptState->currentValue); // read value as soon as possible

                auto edge = interruptState->currentValue == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinChangeEdge::RisingEdge : TinyCLR_Gpio_PinChangeEdge::FallingEdge;

                interruptState->handler(interruptState->controller, interruptState->pin, edge);
            }
        }
    }
}

TinyCLR_Result LPC17_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler) {
    GpioInterruptState* interruptState = &gpioInterruptState[pin];

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t port = GET_PORT(pin);
    uint32_t pinMask = GET_PIN_MASK(pin);
    uint32_t* FIODIR_Register = FIODIR(pin);

    uint32_t* GPIO_Port_X_Interrupt_RisingEdgeRegister = GPIO_INT_RisingEdge(port);
    uint32_t* GPIO_Port_X_Interrupt_FallingEdgeRegister = GPIO_INT_FallingEdge(port);

    if (handler && ((port != 0) && (port != 2))) // If interrupt is called on a non interrupt capable pin return false
        return TinyCLR_Result::ArgumentInvalid;

    LPC17_Gpio_EnableInputPin(pin, pinDriveMode[pin]);

    auto state = reinterpret_cast<GpioState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (handler) {
        interruptState->controller = &gpioControllers[controllerIndex];
        interruptState->pin = (uint8_t)pin;
        interruptState->debounce = LPC17_Gpio_GetDebounceTimeout(self, pin);
        interruptState->handler = handler;
        interruptState->lastDebounceTicks = LPC17_Time_GetTimeForProcessorTicks(nullptr, LPC17_Time_GetCurrentProcessorTicks(nullptr));

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
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    if (pinReserved[pin])
        return false;

    pinReserved[pin] = true;

    return true;
}

bool LPC17_Gpio_ClosePin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    pinReserved[pin] = false;

    // reset to default interruptState
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

TinyCLR_Result LPC17_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (LPC17_Gpio_ReadPin(pin))
        value = TinyCLR_Gpio_PinValue::High;
    else
        value = TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (value == TinyCLR_Gpio_PinValue::High)
        LPC17_Gpio_WritePin(pin, true);
    else
        LPC17_Gpio_WritePin(pin, false);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC17_Gpio_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    LPC17_Gpio_ClosePin(pin);

    return TinyCLR_Result::Success;
}

bool LPC17_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode) {

    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
    case TinyCLR_Gpio_PinDriveMode::Input:
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        return true;
    }

    return false;
}

TinyCLR_Gpio_PinDriveMode LPC17_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    return pinDriveMode[pin];
}

TinyCLR_Result LPC17_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
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

    pinDriveMode[pin] = driveMode;

    return TinyCLR_Result::Success;
}

uint64_t LPC17_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    return gpioDebounceInTicks[pin];
}

TinyCLR_Result LPC17_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks) {
    gpioDebounceInTicks[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

uint32_t LPC17_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self) {
    return TOTAL_GPIO_PINS;
}

void LPC17_Gpio_Reset() {
    uint32_t* GPIO_Port_0_INT_RisingEdge_Register = GPIO_INT_RisingEdge(0);
    uint32_t* GPIO_Port_0_INT_FallingEdge_Register = GPIO_INT_FallingEdge(0);
    uint32_t* GPIO_Port_2_INT_RisingEdge_Register = GPIO_INT_RisingEdge(2);
    uint32_t* GPIO_Port_2_INT_FallingEdge_Register = GPIO_INT_FallingEdge(2);

    for (auto c = 0; c < TOTAL_GPIO_CONTROLLERS; c++) {
        for (auto pin = 0; pin < LPC17_Gpio_GetPinCount(&gpioControllers[c]); pin++) {
            auto& p = gpioPins[pin];

            pinReserved[pin] = 0;
            LPC17_Gpio_SetDebounceTimeout(&gpioControllers[c], pin, DEBOUNCE_DEFAULT_TICKS);

            if (p.apply) {
                LPC17_Gpio_ConfigurePin(pin, p.direction, p.pinFunction, p.resistorMode, p.hysteresis, p.inputPolarity, p.slewRate, p.outputType);

                if (p.direction == LPC17_Gpio_Direction::Output)
                    LPC17_Gpio_WritePin(pin, p.outputDirection);
            }
        }
    }

    *GPIO_Port_0_INT_RisingEdge_Register = 0x0;
    *GPIO_Port_0_INT_FallingEdge_Register = 0x0;
    *GPIO_Port_2_INT_RisingEdge_Register = 0x0;
    *GPIO_Port_2_INT_FallingEdge_Register = 0x0;
}
