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

#define SCS_BASE (*(volatile unsigned long *)0xE01FC1A0)

#define GPIO_BASE 0xE0028000
#define IO0IntStatR_OFFSET 0x84
#define IO0IntStatF_OFFSET 0x88
#define IO0IntClr_OFFSET 0x8C
#define IO0IntEnR_OFFSET 0x90
#define IO0IntEnF_OFFSET 0x94

#define PCB_BASE 0xE002C000

#define PINSEL0 (*(volatile unsigned long *)0xE002C000)
#define PINSEL0_OFFSET 0x0

#define PINMODE0 (*(volatile unsigned long *)0xE002C040)
#define PINMODE0_OFFSET 0x40

#define FIO_BASE 0x3FFFC000

#define FIO0DIR (*(volatile unsigned long *)0x3FFFC000)
#define FIO0DIR_OFFSET 0x0

#define FIO0PIN (*(volatile unsigned long *)0x3FFFC014)
#define FIO0PIN_OFFSET 0x14

#define FIO0PIN0 (*(volatile unsigned char *)0x3FFFC014)
#define FIO0PIN0_OFFSET 0x14

#define FIO0SET (*(volatile unsigned long *)0x3FFFC018)
#define FIO0SET_OFFSET 0x18

#define FIO0SET0 (*(volatile unsigned char *)0x3FFFC018)
#define FIO0SET0_OFFSET 0x18

#define FIO0CLR (*(volatile unsigned long *)0x3FFFC01C)
#define FIO0CLR_OFFSET 0x1C

// Primary functions
#define GET_PORT(pin)                       (pin / 32)
#define GET_PIN(pin)                        (pin % 32)
#define GET_PIN_MASK(pin)                   (1<<(pin % 32))

#define SET_PINSEL_ALTERNATE_0(port, pin)   *((volatile uint32_t *)(PCB_BASE+PINSEL0_OFFSET  + (2*port+((pin>>4) != 0))*0x4)) &= ~(3u<<((2*pin)&0x1F))
#define SET_PINSEL_ALTERNATE_1(port, pin)   *((volatile uint32_t *)(PCB_BASE+PINSEL0_OFFSET  + (2*port+((pin>>4) != 0))*0x4)) |=  (1u<<((2*pin)&0x1F));\
                                            *((volatile uint32_t *)(PCB_BASE+PINSEL0_OFFSET  + (2*port+((pin>>4) != 0))*0x4)) &= ~(2u<<((2*pin)&0x1F))
#define SET_PINSEL_ALTERNATE_2(port, pin)   *((volatile uint32_t *)(PCB_BASE+PINSEL0_OFFSET  + (2*port+((pin>>4) != 0))*0x4)) &= ~(1u<<((2*pin)&0x1F));\
                                            *((volatile uint32_t *)(PCB_BASE+PINSEL0_OFFSET  + (2*port+((pin>>4) != 0))*0x4)) |=  (2u<<((2*pin)&0x1F))
#define SET_PINSEL_ALTERNATE_3(port, pin)   *((volatile uint32_t *)(PCB_BASE+PINSEL0_OFFSET  + (2*port+((pin>>4) != 0))*0x4)) |=  (3u<<((2*pin)&0x1F))

#define SET_PINMODE_PULLUP(port, pin)       *((volatile uint32_t *)(PCB_BASE+PINMODE0_OFFSET + (2*port+((pin>>4) != 0))*0x4)) &= ~(3u<<((2*pin)&0x1F))
#define SET_PINMODE_PULLDOWN(port, pin)     *((volatile uint32_t *)(PCB_BASE+PINMODE0_OFFSET + (2*port+((pin>>4) != 0))*0x4)) |=  (3u<<((2*pin)&0x1F))
#define SET_PINMODE_NOPULL(port, pin)       *((volatile uint32_t *)(PCB_BASE+PINMODE0_OFFSET + (2*port+((pin>>4) != 0))*0x4)) &= ~(1u<<((2*pin)&0x1F));\
                                            *((volatile uint32_t *)(PCB_BASE+PINMODE0_OFFSET + (2*port+((pin>>4) != 0))*0x4)) |=  (2u<<((2*pin)&0x1F))
#define SET_PINDIR_OUTPUT(port, pin)        *((volatile uint32_t *)(FIO_BASE+FIO0DIR_OFFSET + port*0x20 )) |=  (1u<<pin)
#define SET_PINDIR_INPUT(port, pin)         *((volatile uint32_t *)(FIO_BASE+FIO0DIR_OFFSET + port*0x20 )) &= ~(1u<<pin)

#define SET_PIN_HIGH(port, pin)             *((volatile uint32_t *)(FIO_BASE+FIO0SET_OFFSET + port*0x20 )) =   (1u<<pin)
#define SET_PIN_LOW(port, pin)              *((volatile uint32_t *)(FIO_BASE+FIO0CLR_OFFSET + port*0x20 )) =   (1u<<pin)

#define GET_PIN_STATUS(port, pin)           ((*((volatile uint32_t *)(FIO_BASE+FIO0PIN_OFFSET + port*0x20))&(1u<<pin)) == (1u<<pin))

// Interrupt
#define GPIO_INTERRUPT_STATUS_REG                           ((volatile uint32_t *)0xE0028080)

#define SET_PIN_INTERRUPT_RISING_EDGE(port, pin)		    *((volatile unsigned long *)(GPIO_BASE + IO0IntEnR_OFFSET + port*0x10 )) |=  (1u<<pin)
#define DISABLE_PIN_INTERRUPT_RISING_EDGE(port, pin)	    *((volatile unsigned long *)(GPIO_BASE + IO0IntEnR_OFFSET + port*0x10 )) &= ~(1u<<pin)

#define SET_PIN_INTERRUPT_FALLING_EDGE(port, pin)   	    *((volatile unsigned long *)(GPIO_BASE + IO0IntEnF_OFFSET + port*0x10 )) |=  (1u<<pin)
#define DISABLE_PIN_INTERRUPT_FALLING_EDGE(port, pin)	    *((volatile unsigned long *)(GPIO_BASE + IO0IntEnF_OFFSET + port*0x10 )) &= ~(1u<<pin)

#define GET_PIN_INTERRUPT_RISING_EDGE_STATUS(port, pin)	    ((*((volatile unsigned long *)(GPIO_BASE + IO0IntStatR_OFFSET + port*0x10 ))&(1u<<pin)) == (1u<<pin))
#define GET_PIN_INTERRUPT_FALLING_EDGE_STATUS(port, pin)    ((*((volatile unsigned long *)(GPIO_BASE + IO0IntStatF_OFFSET + port*0x10 ))&(1u<<pin)) == (1u<<pin))

#define CLEAR_PIN_INTERRUPT(port, pin)                      *((volatile unsigned long *)(GPIO_BASE + IO0IntClr_OFFSET + port*0x10 )) =  (1u<<pin)

// Driver
static const LPC24_Gpio_PinConfiguration gpioPins[] = LPC24_GPIO_PINS;

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
};

static GpioState gpioStates[TOTAL_GPIO_CONTROLLERS];

static bool pinReserved[TOTAL_GPIO_PINS] __attribute__((section(".bss2.pinReserved")));
static int64_t gpioDebounceInTicks[TOTAL_GPIO_PINS] __attribute__((section(".bss2.gpioDebounceInTicks")));
static GpioInterruptState gpioInterruptState[TOTAL_GPIO_INTERRUPT_PINS] __attribute__((section(".bss2.gpioInterruptState")));
static TinyCLR_Gpio_PinDriveMode pinDriveMode[TOTAL_GPIO_PINS] __attribute__((section(".bss2.pinDriveMode")));

static TinyCLR_Gpio_Controller gpioControllers[TOTAL_GPIO_CONTROLLERS];
static TinyCLR_Api_Info gpioApi[TOTAL_GPIO_CONTROLLERS];

void LPC24_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_GPIO_CONTROLLERS; i++) {
        gpioControllers[i].ApiInfo = &gpioApi[i];
        gpioControllers[i].Acquire = &LPC24_Gpio_Acquire;
        gpioControllers[i].Release = &LPC24_Gpio_Release;
        gpioControllers[i].OpenPin = &LPC24_Gpio_OpenPin;
        gpioControllers[i].ClosePin = &LPC24_Gpio_ClosePin;
        gpioControllers[i].Read = &LPC24_Gpio_Read;
        gpioControllers[i].Write = &LPC24_Gpio_Write;
        gpioControllers[i].IsDriveModeSupported = &LPC24_Gpio_IsDriveModeSupported;
        gpioControllers[i].GetDriveMode = &LPC24_Gpio_GetDriveMode;
        gpioControllers[i].SetDriveMode = &LPC24_Gpio_SetDriveMode;
        gpioControllers[i].GetDebounceTimeout = &LPC24_Gpio_GetDebounceTimeout;
        gpioControllers[i].SetDebounceTimeout = &LPC24_Gpio_SetDebounceTimeout;
        gpioControllers[i].SetPinChangedHandler = &LPC24_Gpio_SetPinChangedHandler;
        gpioControllers[i].GetPinCount = &LPC24_Gpio_GetPinCount;

        gpioApi[i].Author = "GHI Electronics, LLC";
        gpioApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.GpioController";
        gpioApi[i].Type = TinyCLR_Api_Type::GpioController;
        gpioApi[i].Version = 0;
        gpioApi[i].Implementation = &gpioControllers[i];
        gpioApi[i].State = &gpioStates[i];

        gpioStates[i].controllerIndex = i;
    }

    
}

TinyCLR_Result LPC24_Gpio_Acquire(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_Release(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

void LPC24_Gpio_InterruptHandler(void* param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    bool executeIsr = true;

    for (auto port = 0; port <= 2; port += 2) { // Only port 0 and port 2 support interrupts
        auto status_mask_register = 1 << port;

        for (auto pin = 0; ((*GPIO_INTERRUPT_STATUS_REG) & status_mask_register) && (pin < 32); pin++) {

            if (!(GET_PIN_INTERRUPT_RISING_EDGE_STATUS(port, pin)) && !(GET_PIN_INTERRUPT_FALLING_EDGE_STATUS(port, pin))) // If this is not the Pin, skip to next pin
                continue;

            GpioInterruptState* interruptState = &gpioInterruptState[pin + port * 32];

            CLEAR_PIN_INTERRUPT(port, pin); // Clear this pin's IRQ

            if (interruptState->debounce) {
                if ((LPC24_Time_GetTimeForProcessorTicks(nullptr, LPC24_Time_GetCurrentProcessorTicks(nullptr)) - interruptState->lastDebounceTicks) >= gpioDebounceInTicks[interruptState->pin]) {
                    interruptState->lastDebounceTicks = LPC24_Time_GetTimeForProcessorTicks(nullptr, LPC24_Time_GetCurrentProcessorTicks(nullptr));
                }
                else {
                    executeIsr = false;
                }
            }

            if (executeIsr) {
                LPC24_Gpio_Read(interruptState->controller, interruptState->pin, interruptState->currentValue); // read value as soon as possible

                auto edge = interruptState->currentValue == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinChangeEdge::RisingEdge : TinyCLR_Gpio_PinChangeEdge::FallingEdge;

                interruptState->handler(interruptState->controller, interruptState->pin, edge);
            }
        }
    }
}

TinyCLR_Result LPC24_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler) {
    GpioInterruptState* interruptState = &gpioInterruptState[pin];

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t port = GET_PORT(pin);
    uint32_t pinMask = GET_PIN_MASK(pin);

    if (handler && ((port != 0) && (port != 2))) // If interrupt is called on a non interrupt capable pin return false
        return TinyCLR_Result::ArgumentInvalid;

    LPC24_Gpio_EnableInputPin(pin, pinDriveMode[pin]);

    auto state = reinterpret_cast<GpioState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (handler) {
        interruptState->controller = &gpioControllers[controllerIndex];
        interruptState->pin = (uint8_t)pin;
        interruptState->debounce = LPC24_Gpio_GetDebounceTimeout(self, pin);
        interruptState->handler = handler;
        interruptState->lastDebounceTicks = LPC24_Time_GetTimeForProcessorTicks(nullptr, LPC24_Time_GetCurrentProcessorTicks(nullptr));

        SET_PIN_INTERRUPT_RISING_EDGE(GET_PORT(pin), GET_PIN(pin));
        SET_PIN_INTERRUPT_FALLING_EDGE(GET_PORT(pin), GET_PIN(pin));

        LPC24_Interrupt_Activate(LPC24XX_VIC::c_IRQ_INDEX_EINT3, (uint32_t*)&LPC24_Gpio_InterruptHandler, 0);
    }
    else {
        DISABLE_PIN_INTERRUPT_RISING_EDGE(GET_PORT(pin), GET_PIN(pin));
        DISABLE_PIN_INTERRUPT_FALLING_EDGE(GET_PORT(pin), GET_PIN(pin));

        LPC24_Interrupt_Disable(LPC24XX_VIC::c_IRQ_INDEX_EINT3);
    }

    return TinyCLR_Result::Success;
}
bool LPC24_Gpio_Disable_Interrupt(uint32_t pin) {
    return true;
}

bool LPC24_Gpio_OpenPin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    if (pinReserved[pin])
        return false;

    pinReserved[pin] = true;

    return true;
}

bool LPC24_Gpio_ClosePin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    pinReserved[pin] = false;

    // reset to default interruptState
    return LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
}

bool LPC24_Gpio_ReadPin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    return GET_PIN_STATUS(GET_PORT(pin), GET_PIN(pin));
}

void LPC24_Gpio_WritePin(int32_t pin, bool value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return;

    if (value)
        SET_PIN_HIGH(GET_PORT(pin), GET_PIN(pin));
    else
        SET_PIN_LOW(GET_PORT(pin), GET_PIN(pin));
}

bool LPC24_Gpio_ConfigurePin(int32_t pin, LPC24_Gpio_Direction pinDir, LPC24_Gpio_PinFunction alternateFunction, LPC24_Gpio_PinMode pullResistor) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    switch (alternateFunction) {
    case LPC24_Gpio_PinFunction::PinFunction0:
        if (pinDir == LPC24_Gpio_Direction::Output) {
            SET_PINDIR_OUTPUT(GET_PORT(pin), GET_PIN(pin));
            SET_PINMODE_NOPULL(GET_PORT(pin), GET_PIN(pin));
            SET_PINSEL_ALTERNATE_0(GET_PORT(pin), GET_PIN(pin));
        }
        else {
            switch (pullResistor) {
            case LPC24_Gpio_PinMode::Inactive:
                SET_PINDIR_INPUT(GET_PORT(pin), GET_PIN(pin));
                SET_PINMODE_NOPULL(GET_PORT(pin), GET_PIN(pin));
                SET_PINSEL_ALTERNATE_0(GET_PORT(pin), GET_PIN(pin));
                break;

            case LPC24_Gpio_PinMode::PullUp:
                SET_PINSEL_ALTERNATE_0(GET_PORT(pin), GET_PIN(pin));
                SET_PINDIR_INPUT(GET_PORT(pin), GET_PIN(pin));
                SET_PINMODE_PULLUP(GET_PORT(pin), GET_PIN(pin));
                break;

            case LPC24_Gpio_PinMode::PullDown:
                SET_PINSEL_ALTERNATE_0(GET_PORT(pin), GET_PIN(pin));
                SET_PINDIR_INPUT(GET_PORT(pin), GET_PIN(pin));
                SET_PINMODE_PULLDOWN(GET_PORT(pin), GET_PIN(pin));
                break;

            }
        }

        break;

    case LPC24_Gpio_PinFunction::PinFunction1:
        SET_PINDIR_INPUT(GET_PORT(pin), GET_PIN(pin));
        SET_PINMODE_NOPULL(GET_PORT(pin), GET_PIN(pin));
        SET_PINSEL_ALTERNATE_1(GET_PORT(pin), GET_PIN(pin));

        break;

    case LPC24_Gpio_PinFunction::PinFunction2:
        SET_PINDIR_INPUT(GET_PORT(pin), GET_PIN(pin));
        SET_PINMODE_NOPULL(GET_PORT(pin), GET_PIN(pin));
        SET_PINSEL_ALTERNATE_2(GET_PORT(pin), GET_PIN(pin));

        break;

    case LPC24_Gpio_PinFunction::PinFunction3:
        SET_PINDIR_INPUT(GET_PORT(pin), GET_PIN(pin));
        SET_PINMODE_NOPULL(GET_PORT(pin), GET_PIN(pin));
        SET_PINSEL_ALTERNATE_3(GET_PORT(pin), GET_PIN(pin));
        break;
    }

    return true;
}

void LPC24_Gpio_EnableOutputPin(int32_t pin, bool initialState) {
    LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Output, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
    LPC24_Gpio_WritePin(pin, initialState);
}

void LPC24_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {
    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Input:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
        break;
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::PullUp);
        break;
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::PullDown);
        break;
    }
}

TinyCLR_Result LPC24_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    value = LPC24_Gpio_ReadPin(pin) ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    LPC24_Gpio_WritePin(pin, value == TinyCLR_Gpio_PinValue::High ? true : false);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC24_Gpio_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    LPC24_Gpio_ClosePin(pin);

    return TinyCLR_Result::Success;
}

bool LPC24_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode) {

    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
    case TinyCLR_Gpio_PinDriveMode::Input:
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        return true;
    }

    return false;
}

TinyCLR_Gpio_PinDriveMode LPC24_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    return pinDriveMode[pin];
}

TinyCLR_Result LPC24_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    switch (driveMode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Output, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
        break;

    case TinyCLR_Gpio_PinDriveMode::Input:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::PullUp);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::PullDown);
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

uint64_t LPC24_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    return gpioDebounceInTicks[pin];
}

TinyCLR_Result LPC24_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks) {
    gpioDebounceInTicks[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

uint32_t LPC24_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self) {
    return TOTAL_GPIO_PINS;
}

void LPC24_Gpio_Reset() {
    SCS_BASE |= (1 << 0); // Enable for port 0 and 1

    for (auto c = 0; c < TOTAL_GPIO_CONTROLLERS; c++) {
        for (auto pin = 0; pin < LPC24_Gpio_GetPinCount(&gpioControllers[c]); pin++) {
            auto& p = gpioPins[pin];

            pinReserved[pin] = false;
            LPC24_Gpio_SetDebounceTimeout(&gpioControllers[c], pin, DEBOUNCE_DEFAULT_TICKS);

            if (p.apply) {
                LPC24_Gpio_ConfigurePin(pin, p.pinDirection, p.pinFunction, p.pinMode);

                if (p.pinDirection == LPC24_Gpio_Direction::Output)
                    LPC24_Gpio_WritePin(pin, p.outputDirection);
            }
        }
    }
}
