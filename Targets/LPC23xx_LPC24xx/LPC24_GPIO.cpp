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
#define LPC24_Gpio_DebounceDefaultTicks   (20*10000) // 20ms in ticks
#define LPC24_Gpio_MaxPins                     SIZEOF_ARRAY(g_lpc24_pins)

static const LPC24_Gpio_PinConfiguration g_lpc24_pins[] = LPC24_GPIO_PINS;
struct LPC24_Int_State {
    uint8_t                                     pin;      // pin number
    int64_t                                    debounce; // debounce
    uint64_t                                    lastDebounceTicks;

    const TinyCLR_Gpio_Controller*                controller; // controller
    TinyCLR_Gpio_ValueChangedHandler            ISR; // interrupt handler
    TinyCLR_Gpio_PinValue                       currentValue;
};

static bool                     g_pinReserved[LPC24_Gpio_MaxPins] __attribute__((section(".bss2.g_pinReserved")));
static int64_t                     g_debounceTicksPin[LPC24_Gpio_MaxPins] __attribute__((section(".bss2.g_debounceTicksPin")));
static LPC24_Int_State              g_int_state[LPC24_Gpio_MaxPins] __attribute__((section(".bss2.g_int_state"))); // interrupt state
static TinyCLR_Gpio_PinDriveMode    g_pinDriveMode[LPC24_Gpio_MaxPins] __attribute__((section(".bss2.g_pinDriveMode")));

static TinyCLR_Gpio_Controller gpioProvider;
static TinyCLR_Api_Info gpioApi;

#define LPC24_GPIO_DEFAULT_CONTROLLER 0

const TinyCLR_Api_Info* LPC24_Gpio_GetApi() {
    gpioProvider.ApiInfo = &gpioApi;
    gpioProvider.Acquire = &LPC24_Gpio_Acquire;
    gpioProvider.Release = &LPC24_Gpio_Release;
    gpioProvider.AcquirePin = &LPC24_Gpio_AcquirePin;
    gpioProvider.ReleasePin = &LPC24_Gpio_ReleasePin;
    gpioProvider.IsDriveModeSupported = &LPC24_Gpio_IsDriveModeSupported;
    gpioProvider.Read = &LPC24_Gpio_Read;
    gpioProvider.Write = &LPC24_Gpio_Write;
    gpioProvider.GetDriveMode = &LPC24_Gpio_GetDriveMode;
    gpioProvider.SetDriveMode = &LPC24_Gpio_SetDriveMode;
    gpioProvider.GetDebounceTimeout = &LPC24_Gpio_GetDebounceTimeout;
    gpioProvider.SetDebounceTimeout = &LPC24_Gpio_SetDebounceTimeout;
    gpioProvider.SetValueChangedHandler = &LPC24_Gpio_SetValueChangedHandler;
    gpioProvider.GetPinCount = &LPC24_Gpio_GetPinCount;
    gpioProvider.GetControllerCount = &LPC24_Gpio_GetControllerCount;

    gpioApi.Author = "GHI Electronics, LLC";
    gpioApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.GpioProvider";
    gpioApi.Type = TinyCLR_Api_Type::GpioProvider;
    gpioApi.Version = 0;
    gpioApi.Implementation = &gpioProvider;

    return &gpioApi;
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

            LPC24_Int_State* state = &g_int_state[pin + port * 32];

            CLEAR_PIN_INTERRUPT(port, pin); // Clear this pin's IRQ

            if (state->debounce) {
                if ((LPC24_Time_GetTimeForProcessorTicks(nullptr, LPC24_Time_GetCurrentProcessorTicks(nullptr)) - state->lastDebounceTicks) >= g_debounceTicksPin[state->pin]) {
                    state->lastDebounceTicks = LPC24_Time_GetTimeForProcessorTicks(nullptr, LPC24_Time_GetCurrentProcessorTicks(nullptr));
                }
                else {
                    executeIsr = false;
                }
            }

            if (executeIsr) {
                auto gpioController = 0; //TODO Temporary set to 0

                LPC24_Gpio_Read(&gpioProvider, gpioController, state->pin, state->currentValue); // read value as soon as possible

                state->ISR(state->controller, gpioController, state->pin, state->currentValue);
            }
        }
    }
}

TinyCLR_Result LPC24_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR) {
    LPC24_Int_State* state = &g_int_state[pin];

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t port = GET_PORT(pin);
    uint32_t pinMask = GET_PIN_MASK(pin);

    if (ISR && ((port != 0) && (port != 2))) // If interrupt is called on a non interrupt capable pin return false
        return TinyCLR_Result::ArgumentInvalid;

    LPC24_Gpio_EnableInputPin(pin, g_pinDriveMode[pin]);

    auto gpioController = 0; //TODO Temporary set to 0

    if (ISR) {
        state->controller = &gpioProvider;
        state->pin = (uint8_t)pin;
        state->debounce = LPC24_Gpio_GetDebounceTimeout(self, gpioController, pin);
        state->ISR = ISR;
        state->lastDebounceTicks = LPC24_Time_GetTimeForProcessorTicks(nullptr, LPC24_Time_GetCurrentProcessorTicks(nullptr));

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
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return false;

    if (g_pinReserved[pin])
        return false;

    g_pinReserved[pin] = true;

    return true;
}

bool LPC24_Gpio_ClosePin(int32_t pin) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return false;

    g_pinReserved[pin] = false;

    // reset to default state
    return LPC24_Gpio_ConfigurePin(pin, LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction0, LPC24_Gpio_PinMode::Inactive);
}

bool LPC24_Gpio_ReadPin(int32_t pin) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return false;

    return GET_PIN_STATUS(GET_PORT(pin), GET_PIN(pin));
}

void LPC24_Gpio_WritePin(int32_t pin, bool value) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return;

    if (value)
        SET_PIN_HIGH(GET_PORT(pin), GET_PIN(pin));
    else
        SET_PIN_LOW(GET_PORT(pin), GET_PIN(pin));
}

bool LPC24_Gpio_ConfigurePin(int32_t pin, LPC24_Gpio_Direction pinDir, LPC24_Gpio_PinFunction alternateFunction, LPC24_Gpio_PinMode pullResistor) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
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

TinyCLR_Result LPC24_Gpio_Read(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinValue& value) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    value = LPC24_Gpio_ReadPin(pin) ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_Write(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinValue value) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    LPC24_Gpio_WritePin(pin, value == TinyCLR_Gpio_PinValue::High ? true : false);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_AcquirePin(const TinyCLR_Gpio_Controller* self, int32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!LPC24_Gpio_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Gpio_ReleasePin(const TinyCLR_Gpio_Controller* self, int32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    LPC24_Gpio_ClosePin(pin);

    return TinyCLR_Result::Success;
}

bool LPC24_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {

    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
    case TinyCLR_Gpio_PinDriveMode::Input:
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        return true;
    }

    return false;
}

TinyCLR_Gpio_PinDriveMode LPC24_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, int32_t pin) {
    return g_pinDriveMode[pin];
}

TinyCLR_Result LPC24_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    if (pin >= LPC24_Gpio_MaxPins || pin < 0)
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

    g_pinDriveMode[pin] = driveMode;

    return TinyCLR_Result::Success;
}

uint64_t LPC24_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, int32_t pin) {
    return g_debounceTicksPin[pin];
}

TinyCLR_Result LPC24_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, int32_t pin, uint64_t debounceTicks) {
    g_debounceTicksPin[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

int32_t LPC24_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self) {
    return LPC24_Gpio_MaxPins;
}

void LPC24_Gpio_Reset() {
    SCS_BASE |= (1 << 0); // Enable for port 0 and 1

    auto gpioController = 0; //TODO Temporary set to 0

    for (auto pin = 0; pin < LPC24_Gpio_GetPinCount(&gpioProvider, gpioController); pin++) {
        auto& p = g_lpc24_pins[pin];

        g_pinReserved[pin] = false;
        LPC24_Gpio_SetDebounceTimeout(&gpioProvider, gpioController, pin, LPC24_Gpio_DebounceDefaultTicks);

        if (p.apply) {
            LPC24_Gpio_ConfigurePin(pin, p.pinDirection, p.pinFunction, p.pinMode);

            if (p.pinDirection == LPC24_Gpio_Direction::Output)
                LPC24_Gpio_WritePin(pin, p.outputDirection);
        }
    }
}

TinyCLR_Result LPC24_Gpio_GetControllerCount(const TinyCLR_Gpio_Controller* self, int32_t& count) {
    count = 1;

    return TinyCLR_Result::Success;
}
