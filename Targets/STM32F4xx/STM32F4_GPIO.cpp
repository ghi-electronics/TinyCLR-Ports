// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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

#include "STM32F4.h"

#define TOTAL_GPIO_CONTROLLERS 1

#define TOTAL_GPIO_PINS SIZEOF_ARRAY(stm32f4_pins)

static const STM32F4_Gpio_PinConfiguration stm32f4_pins[] = STM32F4_GPIO_PINS;

#define TOTAL_GPIO_INTERRUPT_PINS 16

#define DEBOUNCE_DEFAULT_TICKS     (20*10000) // 20ms in ticks

#define PIN_RESERVED 1

// indexed port configuration access
#define Port(port) ((GPIO_TypeDef *) (GPIOA_BASE + (port << 10)))

struct STM32F4_Interrupt_State {
    uint8_t pin;
    int64_t debounce;
    uint64_t  lastDebounceTicks;

    const TinyCLR_Gpio_Controller* controller;
    TinyCLR_Gpio_ValueChangedHandler ISR;
    TinyCLR_Gpio_PinValue currentValue;
};

struct GpioDriver {
    int32_t controllerIndex;
};

static GpioDriver gpioDrivers[TOTAL_GPIO_CONTROLLERS];

static bool g_pinReserved[TOTAL_GPIO_PINS];
static int64_t g_debounceTicksPin[TOTAL_GPIO_PINS];
static STM32F4_Interrupt_State g_int_state[TOTAL_GPIO_INTERRUPT_PINS];
static TinyCLR_Gpio_PinDriveMode g_pinDriveMode[TOTAL_GPIO_PINS];

static TinyCLR_Gpio_Controller gpioControllers[TOTAL_GPIO_CONTROLLERS];
static TinyCLR_Api_Info gpioApi[TOTAL_GPIO_CONTROLLERS];

const TinyCLR_Api_Info* STM32F4_Gpio_GetApi() {
    for (int32_t i = 0; i < TOTAL_GPIO_CONTROLLERS; i++) {
        gpioControllers[i].ApiInfo = &gpioApi[i];
        gpioControllers[i].Acquire = &STM32F4_Gpio_Acquire;
        gpioControllers[i].Release = &STM32F4_Gpio_Release;
        gpioControllers[i].AcquirePin = &STM32F4_Gpio_AcquirePin;
        gpioControllers[i].ReleasePin = &STM32F4_Gpio_ReleasePin;
        gpioControllers[i].Read = &STM32F4_Gpio_Read;
        gpioControllers[i].Write = &STM32F4_Gpio_Write;
        gpioControllers[i].IsDriveModeSupported = &STM32F4_Gpio_IsDriveModeSupported;
        gpioControllers[i].GetDriveMode = &STM32F4_Gpio_GetDriveMode;
        gpioControllers[i].SetDriveMode = &STM32F4_Gpio_SetDriveMode;
        gpioControllers[i].GetDebounceTimeout = &STM32F4_Gpio_GetDebounceTimeout;
        gpioControllers[i].SetDebounceTimeout = &STM32F4_Gpio_SetDebounceTimeout;
        gpioControllers[i].SetValueChangedHandler = &STM32F4_Gpio_SetValueChangedHandler;
        gpioControllers[i].GetPinCount = &STM32F4_Gpio_GetPinCount;

        gpioApi[i].Author = "GHI Electronics, LLC";
        gpioApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.GpioController";
        gpioApi[i].Type = TinyCLR_Api_Type::GpioController;
        gpioApi[i].Version = 0;
        gpioApi[i].Implementation = &gpioControllers[i];
        gpioApi[i].State = &gpioDrivers[i];

        gpioDrivers[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&gpioApi;
}

TinyCLR_Result STM32F4_Gpio_Acquire(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Gpio_Release(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

/*
 * Interrupt Handler
 */
void STM32F4_Gpio_ISR(int num)  // 0 <= num <= 15
{
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    bool executeIsr = true;

    STM32F4_Interrupt_State* interruptState = &g_int_state[num];

    uint32_t bit = 1 << num;

    STM32F4_Gpio_Read(nullptr, interruptState->pin, interruptState->currentValue); // read value as soon as possible

    EXTI->PR = bit;   // reset pending bit

    if (interruptState->ISR) {
        if (interruptState->debounce) {   // debounce enabled
            if ((STM32F4_Time_GetTimeForProcessorTicks(nullptr, STM32F4_Time_GetCurrentProcessorTicks(nullptr)) - interruptState->lastDebounceTicks) >= g_debounceTicksPin[interruptState->pin]) {
                interruptState->lastDebounceTicks = STM32F4_Time_GetTimeForProcessorTicks(nullptr, STM32F4_Time_GetCurrentProcessorTicks(nullptr));
            }
            else {
                executeIsr = false;
            }

        }

        if (executeIsr)
            interruptState->ISR(interruptState->controller, interruptState->pin, interruptState->currentValue);
    }
}

void STM32F4_Gpio_Interrupt0(void* param) // EXTI0
{
    STM32F4_Gpio_ISR(0);
}

void STM32F4_Gpio_Interrupt1(void* param) // EXTI1
{
    STM32F4_Gpio_ISR(1);
}

void STM32F4_Gpio_Interrupt2(void* param) // EXTI2
{
    STM32F4_Gpio_ISR(2);
}

void STM32F4_Gpio_Interrupt3(void* param) // EXTI3
{
    STM32F4_Gpio_ISR(3);
}

void STM32F4_Gpio_Interrupt4(void* param) // EXTI4
{
    STM32F4_Gpio_ISR(4);
}

void STM32F4_Gpio_Interrupt5(void* param) // EXTI5 - EXTI9
{
    uint32_t pending = EXTI->PR & EXTI->IMR & 0x03E0; // pending bits 5..9
    int num = 5; pending >>= 5;
    do {
        if (pending & 1) STM32F4_Gpio_ISR(num);
        num++; pending >>= 1;
    } while (pending);
}

void STM32F4_Gpio_Interrupt10(void* param) // EXTI10 - EXTI15
{
    uint32_t pending = EXTI->PR & EXTI->IMR & 0xFC00; // pending bits 10..15
    int num = 10; pending >>= 10;
    do {
        if (pending & 1) STM32F4_Gpio_ISR(num);
        num++; pending >>= 1;
    } while (pending);
}

TinyCLR_Result STM32F4_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler isr) {
    uint32_t num = pin & 0x0F;
    uint32_t bit = 1 << num;
    uint32_t shift = (num & 0x3) << 2; // 4 bit fields
    uint32_t idx = num >> 2;
    uint32_t mask = 0xF << shift;
    uint32_t config = (pin >> 4) << shift; // port number configuration

    STM32F4_Interrupt_State* interruptState = &g_int_state[num];

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = reinterpret_cast<GpioDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (isr) {
        if ((SYSCFG->EXTICR[idx] & mask) != config) {
            if (EXTI->IMR & bit)
                return TinyCLR_Result::SharingViolation; // interrupt in use

            SYSCFG->EXTICR[idx] = SYSCFG->EXTICR[idx] & ~mask | config;
        }
        interruptState->controller = &gpioControllers[controllerIndex];
        interruptState->pin = (uint8_t)pin;
        interruptState->debounce = STM32F4_Gpio_GetDebounceTimeout(self, pin);
        interruptState->ISR = isr;
        interruptState->lastDebounceTicks = STM32F4_Time_GetTimeForProcessorTicks(nullptr, STM32F4_Time_GetCurrentProcessorTicks(nullptr));

        EXTI->RTSR &= ~bit;
        EXTI->FTSR &= ~bit;

        EXTI->FTSR |= bit;
        EXTI->RTSR |= bit;

        do {
            EXTI->PR = bit; // remove pending interrupt
        } while (EXTI->PR & bit); // repeat if pending again

        EXTI->IMR |= bit; // enable interrupt
    }
    else if ((SYSCFG->EXTICR[idx] & mask) == config) {
        EXTI->IMR &= ~bit; // disable interrupt
        interruptState->ISR = 0;
    }
    return TinyCLR_Result::Success;
}
bool STM32F4_Gpio_DisableInterrupt(uint32_t pin) {
    uint32_t num = pin & 0x0F;
    uint32_t bit = 1 << num;
    uint32_t shift = (num & 0x3) << 2; // 4 bit fields
    uint32_t idx = num >> 2;
    uint32_t mask = 0xF << shift;
    uint32_t config = (pin >> 4) << shift; // port number configuration

    STM32F4_Interrupt_State* interruptState = &g_int_state[num];
    if ((SYSCFG->EXTICR[idx] & mask) == config) {
        EXTI->IMR &= ~bit; // disable interrupt
        interruptState->ISR = 0;
    }
    return true;
}

bool STM32F4_GpioInternal_OpenPin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin == PIN_NONE || g_pinReserved[pin])
        return false;

    g_pinReserved[pin] = true;

    return true;
}

bool STM32F4_GpioInternal_ClosePin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin == PIN_NONE)
        return false;

    g_pinReserved[pin] = false;

    // reset to default state
    return STM32F4_GpioInternal_ConfigurePin(pin, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);
}

bool STM32F4_GpioInternal_ConfigurePin(int32_t pin, STM32F4_Gpio_PortMode portMode, STM32F4_Gpio_OutputType outputType, STM32F4_Gpio_OutputSpeed outputSpeed, STM32F4_Gpio_PullDirection pullDirection, STM32F4_Gpio_AlternateFunction alternateFunction) {
    if (pin >= TOTAL_GPIO_PINS || pin == PIN_NONE)
        return false;

    GPIO_TypeDef* port = Port(pin >> 4);

    pin &= 0x0F; // bit number

    uint32_t bit = 1 << pin;
    uint32_t shift = pin << 1; // 2 bits / pin
    uint32_t mask = 0x3 << shift;
    uint32_t pullDir = (uint8_t)pullDirection;
    uint32_t mode = (uint8_t)portMode;
    uint32_t alternate = (uint8_t)alternateFunction;
    uint32_t speed = (uint8_t)outputSpeed; // Force all ports to 100 MHz High speed on 30 pF (80 MHz Output max speed on 15 pF)
    uint32_t altSh = (pin & 0x7) << 2; // 4 bits / pin
    uint32_t altMsk = 0xF << altSh;
    uint32_t idx = pin >> 3;
    uint32_t af = alternate << altSh;

    mode <<= shift;
    pullDir <<= shift;
    speed <<= shift;

    // Write to register
    DISABLE_INTERRUPTS_SCOPED(irq);

    port->MODER = port->MODER & ~mask | mode;
    port->PUPDR = port->PUPDR & ~mask | pullDir;
    port->OSPEEDR = port->OSPEEDR & ~mask | speed;
    port->AFR[idx] = port->AFR[idx] & ~altMsk | af;

    if (outputType == STM32F4_Gpio_OutputType::OpenDrain) {
        // open drain
        port->OTYPER |= bit;
    }
    else {
        port->OTYPER &= ~bit;
    }

    return true;
}

bool STM32F4_GpioInternal_ReadPin(int32_t pin) {
    GPIO_TypeDef* port = Port(pin >> 4); // pointer to the actual port registers

    return ((port->IDR >> (pin & 0xF)) & 1) > 0 ? true : false;

}
void STM32F4_GpioInternal_WritePin(int32_t pin, bool value) {
    GPIO_TypeDef* port = Port(pin >> 4); // pointer to the actual port registers

    uint16_t bit = 1 << (pin & 0x0F);

    if (value)
        port->BSRR = bit; // set bit
    else
        port->BSRR = (bit << 16); // reset bit
}

TinyCLR_Result STM32F4_Gpio_Read(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinValue& value) {
    value = STM32F4_GpioInternal_ReadPin(pin) ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Gpio_Write(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinValue value) {
    STM32F4_GpioInternal_WritePin(pin, value == TinyCLR_Gpio_PinValue::High ? true : false);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Gpio_AcquirePin(const TinyCLR_Gpio_Controller* self, int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin == PIN_NONE)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (g_pinReserved[pin] == PIN_RESERVED) {
        return TinyCLR_Result::SharingViolation;
    }

    if (STM32F4_GpioInternal_OpenPin(pin))
        return TinyCLR_Result::Success;

    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result STM32F4_Gpio_ReleasePin(const TinyCLR_Gpio_Controller* self, int32_t pin) {
    return STM32F4_GpioInternal_ClosePin(pin) == true ? TinyCLR_Result::Success : TinyCLR_Result::NotAvailable;
}

bool STM32F4_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {
    return (mode == TinyCLR_Gpio_PinDriveMode::Output || mode == TinyCLR_Gpio_PinDriveMode::Input || mode == TinyCLR_Gpio_PinDriveMode::InputPullUp || mode == TinyCLR_Gpio_PinDriveMode::InputPullDown) ? true : false;
}

TinyCLR_Gpio_PinDriveMode STM32F4_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, int32_t pin) {
    return g_pinDriveMode[pin];
}

TinyCLR_Result STM32F4_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, int32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    switch (driveMode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
    case TinyCLR_Gpio_PinDriveMode::Input:
        STM32F4_GpioInternal_ConfigurePin(pin, driveMode == TinyCLR_Gpio_PinDriveMode::Output ? STM32F4_Gpio_PortMode::GeneralPurposeOutput : STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        STM32F4_GpioInternal_ConfigurePin(pin, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, driveMode == TinyCLR_Gpio_PinDriveMode::InputPullUp ? STM32F4_Gpio_PullDirection::PullUp : STM32F4_Gpio_PullDirection::PullDown, STM32F4_Gpio_AlternateFunction::AF0);
        break;

    case TinyCLR_Gpio_PinDriveMode::OutputOpenDrain:
        STM32F4_GpioInternal_ConfigurePin(pin, STM32F4_Gpio_PortMode::GeneralPurposeOutput, STM32F4_Gpio_OutputType::OpenDrain, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);
        break;

    case TinyCLR_Gpio_PinDriveMode::OutputOpenDrainPullUp:
        STM32F4_GpioInternal_ConfigurePin(pin, STM32F4_Gpio_PortMode::GeneralPurposeOutput, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, STM32F4_Gpio_AlternateFunction::AF0);
        break;

    default:
        return TinyCLR_Result::NotSupported;
    }

    g_pinDriveMode[pin] = driveMode;

    return TinyCLR_Result::Success;
}

uint64_t STM32F4_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, int32_t pin) {
    return g_debounceTicksPin[pin];
}

TinyCLR_Result STM32F4_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, int32_t pin, uint64_t debounceTicks) {
    g_debounceTicksPin[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self) {
    return TOTAL_GPIO_PINS;
}

void STM32F4_Gpio_Reset() {
    for (int i = 0; i < TOTAL_GPIO_PINS; i++) {
        auto& p = stm32f4_pins[i];

        g_pinReserved[i] = 0;
        STM32F4_Gpio_SetDebounceTimeout(nullptr, i, DEBOUNCE_DEFAULT_TICKS);
        STM32F4_Gpio_DisableInterrupt(i);

        if (p.apply) {
            STM32F4_GpioInternal_ConfigurePin(i, p.portMode, p.outputType, p.outputSpeed, p.pullDirection, p.alternateFunction);

            if (p.portMode == STM32F4_Gpio_PortMode::GeneralPurposeOutput)
                STM32F4_GpioInternal_WritePin(i, p.outputDirection);
        }
    }

    EXTI->IMR = 0; // disable all external interrups;
    STM32F4_InterruptInternal_Activate(EXTI0_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt0, 0);
    STM32F4_InterruptInternal_Activate(EXTI1_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt1, 0);
    STM32F4_InterruptInternal_Activate(EXTI2_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt2, 0);
    STM32F4_InterruptInternal_Activate(EXTI3_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt3, 0);
    STM32F4_InterruptInternal_Activate(EXTI4_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt4, 0);
    STM32F4_InterruptInternal_Activate(EXTI9_5_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt5, 0);
    STM32F4_InterruptInternal_Activate(EXTI15_10_IRQn, (uint32_t*)&STM32F4_Gpio_Interrupt10, 0);
}
