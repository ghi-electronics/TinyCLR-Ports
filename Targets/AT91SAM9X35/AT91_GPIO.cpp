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

static const AT91_Gpio_PinConfiguration g_at91_pins[] = AT91_GPIO_PINS;

#define AT91_Gpio_DebounceDefaultMilisecond   20
#define AT91_Gpio_MaxPins                     SIZEOF_ARRAY(g_at91_pins)

#define PIO_PPDDR(x)	(*(volatile unsigned long *)(0xFFFFF490 + (x * 0x200))) // Pull-down Disable Resistor Register -- Write Only
#define PIO_PPDER(x)	(*(volatile unsigned long *)(0xFFFFF494 + (x * 0x200))) // Pull-down Enable Resistor Register -- Write Only

#define GETPORT(pin)    (pin/32)
#define GETBIT(pin)     (pin%32)

#define MAX_PORT (AT91_Gpio_MaxPins/32)

struct AT91_Int_State {
    uint8_t                                     pin;      // pin number
    int64_t                                    debounce; // debounce
    uint64_t                                    lastDebounceTicks;

    const TinyCLR_Gpio_Provider*                controller; // controller
    TinyCLR_Gpio_ValueChangedHandler            ISR; // interrupt handler
    TinyCLR_Gpio_PinValue                       currentValue;
};

static bool                     	g_pinReserved[AT91_Gpio_MaxPins];
static int64_t                     g_debounceTicksPin[AT91_Gpio_MaxPins];
static AT91_Int_State              	g_int_state[AT91_Gpio_MaxPins]; // interrupt state
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
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Gpio_Release(const TinyCLR_Gpio_Provider* self) {
    return TinyCLR_Result::Success;
}

void AT91_Gpio_InterruptHandler(void* param) {
    INTERRUPT_STARTED_SCOPED(isr);

    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto port = 0; port < MAX_PORT; port++) {

        AT91_PIO &pioX = AT91::PIO(port);

        uint32_t bitMask = 0x00000001;
        uint32_t bitIndex = 0;

        uint32_t interruptsActive = pioX.PIO_ISR;

        interruptsActive &= pioX.PIO_IMR;

        while (interruptsActive) {
            while ((interruptsActive & bitMask) == 0) {
                bitMask <<= 1;
                bitIndex++;
            }

            bool executeIsr = true;

            AT91_Int_State* state = &g_int_state[bitIndex + port * 32];;

            if (state->debounce) {
                if ((AT91_Time_GetTimeForProcessorTicks(nullptr, AT91_Time_GetCurrentProcessorTicks(nullptr)) - state->lastDebounceTicks) >= g_debounceTicksPin[state->pin]) {
                    state->lastDebounceTicks = AT91_Time_GetTimeForProcessorTicks(nullptr, AT91_Time_GetCurrentProcessorTicks(nullptr));
                }
                else {
                    executeIsr = false;
                }
            }

            if (executeIsr) {
                AT91_Gpio_Read(&gpioProvider, state->pin, state->currentValue); // read value as soon as possible

                state->ISR(state->controller, state->pin, state->currentValue);
            }

            interruptsActive ^= bitMask;
        }
    }

}

TinyCLR_Result AT91_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR) {
    AT91_Int_State* state = &g_int_state[pin];

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t port = GETPORT(pin);
    uint32_t bitmask = 1 << GETBIT(pin);

    AT91_PIO &pioX = AT91::PIO(port);

    AT91_Gpio_EnableInputPin(pin, g_pinDriveMode[pin]);

    if (ISR) {
        state->controller = &gpioProvider;
        state->pin = (uint8_t)pin;
        state->debounce = AT91_Gpio_GetDebounceTimeout(self, pin);
        state->ISR = ISR;
        state->lastDebounceTicks = AT91_Time_GetTimeForProcessorTicks(nullptr, AT91_Time_GetCurrentProcessorTicks(nullptr));

        pioX.PIO_IER = bitmask; // Enable interrupt
    }
    else {
        pioX.PIO_IDR = bitmask; // Disable interrupt
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
    return AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
}

bool AT91_Gpio_ReadPin(int32_t pin) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return false;

    uint32_t  bit = GETBIT(pin);
    uint32_t  port = GETPORT(pin);

    AT91_PIO &pioX = AT91::PIO(port);

    return (((pioX.PIO_PDSR) >> bit) & 1) ? true : false;
}

void AT91_Gpio_WritePin(int32_t pin, bool value) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return;

    uint32_t  port = GETPORT(pin);
    uint32_t  bit = GETBIT(pin);

    AT91_PIO &pioX = AT91::PIO(port);

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (value) {
        pioX.PIO_SODR = 1 << bit;
    }
    else {
        pioX.PIO_CODR = 1 << bit;
    }
}

bool AT91_Gpio_ConfigurePin(int32_t pin, AT91_Gpio_Direction pinDir, AT91_Gpio_PeripheralSelection peripheralSelection, AT91_Gpio_ResistorMode resistorMode, AT91_Gpio_MultiDriver multiDrive, AT91_Gpio_Filter filter, AT91_Gpio_FilterSlowClock filterSlowClock, AT91_Gpio_Schmitt schmitt, AT91_Gpio_DriveSpeed driveSpeed) {
    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return false;

    uint32_t bitmask = 1 << GETBIT(pin);
    int32_t port = GETPORT(pin);

    AT91_PIO &pioX = AT91::PIO(port);

    if (peripheralSelection == AT91_Gpio_PeripheralSelection::None) {
        pioX.PIO_PER = bitmask; // Enable PIO function
        switch (pinDir) {
        case AT91_Gpio_Direction::Output:

            pioX.PIO_PUDR = bitmask;	// Disable the pull up resistor
            PIO_PPDDR(port) = bitmask;	// Disable the pull-down resistor

            pioX.PIO_OER = bitmask; // Enable Output

            break;

        case AT91_Gpio_Direction::Input:
            pioX.PIO_ODR = bitmask; // Disable Output

            if (filter == AT91_Gpio_Filter::Enable) {
                pioX.PIO_IFER = bitmask;
            }
            else {
                pioX.PIO_IFDR = bitmask;
            }

            if (filterSlowClock == AT91_Gpio_FilterSlowClock::Enable) {
                pioX.PIO_IFSCER = bitmask;
            }
            else {
                pioX.PIO_IFSCDR = bitmask;
            }

            switch (resistorMode) {
            case AT91_Gpio_ResistorMode::Inactive:
                pioX.PIO_PUDR = bitmask;	// Disable the pull up resistor
                PIO_PPDDR(port) = bitmask;	// Disable the pull-down resistor
                break;

            case AT91_Gpio_ResistorMode::PullUp:
                PIO_PPDDR(port) = bitmask;	// Disable the pull-down resistor first.
                pioX.PIO_PUER = bitmask;	// Enable the pull up resistor
                break;

            case AT91_Gpio_ResistorMode::PullDown:
                pioX.PIO_PUDR = bitmask;	// Disable the pull up resistor first.
                PIO_PPDER(port) = bitmask;	// Enables the pull-down resistor
                break;
            }
            break;
        }
    }
    else {
        if (resistorMode == AT91_Gpio_ResistorMode::Inactive)
            pioX.PIO_PUDR = bitmask;         // Disable the pull up resistor
        else if (resistorMode == AT91_Gpio_ResistorMode::PullUp)
            pioX.PIO_PUER = bitmask;            // Enable the pull up resistor

        switch (peripheralSelection) {
        case AT91_Gpio_PeripheralSelection::PeripheralA:
            pioX.PIO_PDR = bitmask;
            pioX.PIO_ABCDSR[0] &= ~bitmask; // Writes a zero to bit that is masked
            pioX.PIO_ABCDSR[1] &= ~bitmask; // Writes a zero to bit that is masked
            break;

        case AT91_Gpio_PeripheralSelection::PeripheralB:
            pioX.PIO_PDR = bitmask;
            pioX.PIO_ABCDSR[0] |= bitmask;
            pioX.PIO_ABCDSR[1] &= ~bitmask;
            break;

        case AT91_Gpio_PeripheralSelection::PeripheralC:
            pioX.PIO_PDR = bitmask;
            pioX.PIO_ABCDSR[0] &= ~bitmask;
            pioX.PIO_ABCDSR[1] |= bitmask;
            break;

        case AT91_Gpio_PeripheralSelection::PeripheralD:
            pioX.PIO_PDR = bitmask;
            pioX.PIO_ABCDSR[0] |= bitmask;
            pioX.PIO_ABCDSR[1] |= bitmask;
            break;
        }

        pioX.PIO_IDR = bitmask; // Disable the Input Change Interrupt
    }

    return true;
}


bool AT91_Gpio_ConfigurePin(int32_t pin, AT91_Gpio_Direction pinDir, AT91_Gpio_PeripheralSelection peripheralSelection, AT91_Gpio_ResistorMode resistorMode) {
    return AT91_Gpio_ConfigurePin(pin, pinDir, peripheralSelection, resistorMode, AT91_Gpio_MultiDriver::Disable, AT91_Gpio_Filter::Enable, AT91_Gpio_FilterSlowClock::Disable, AT91_Gpio_Schmitt::Disable, AT91_Gpio_DriveSpeed::Reserved);
}

void AT91_Gpio_EnableOutputPin(int32_t pin, bool initialState) {
    AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Output, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_WritePin(pin, initialState);
}

void AT91_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {
    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Input:
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        break;
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullUp);
        break;
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullDown);
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

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= AT91_Gpio_MaxPins || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!AT91_Gpio_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

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
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Output, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        break;

    case TinyCLR_Gpio_PinDriveMode::Input:
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::Inactive);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullUp);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        AT91_Gpio_ConfigurePin(pin, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullDown);
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

uint64_t AT91_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin) {
    return g_debounceTicksPin[pin];
}

TinyCLR_Result AT91_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, uint64_t debounceTicks) {
    g_debounceTicksPin[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

int32_t AT91_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self) {
    return AT91_Gpio_MaxPins;
}

void AT91_Gpio_Reset() {
    AT91_PMC &pmc = AT91::PMC();

    pmc.PMC_PCER = (1 << 2); // Enable Power Control Port A, B
    pmc.PMC_PCER = (1 << 3); // Enable Power Control Port C, D

    for (auto port = 0; port < MAX_PORT; port++) {
        // initialize pins as free
        AT91_PIO &pioX = AT91::PIO(port);

        pioX.PIO_IDR = 0xffffffff;                // Disable all interrupts
        pioX.PIO_ISR ^= 0xffffffff;
    }

    for (auto pin = 0; pin < AT91_Gpio_GetPinCount(&gpioProvider); pin++) {
        auto& p = g_at91_pins[pin];

        g_pinReserved[pin] = false;
        AT91_Gpio_SetDebounceTimeout(&gpioProvider, pin, AT91_Gpio_DebounceDefaultMilisecond);

        if (p.apply) {
            AT91_Gpio_ConfigurePin(pin, p.direction, p.peripheralSelection, p.resistorMode);

            if (p.direction == AT91_Gpio_Direction::Output)
                AT91_Gpio_WritePin(pin, p.outputDirection);
        }
    }

    AT91_Interrupt_Activate(AT91C_ID_PIOA_PIOB, (uint32_t*)&AT91_Gpio_InterruptHandler, nullptr);
    AT91_Interrupt_Activate(AT91C_ID_PIOC_PIOD, (uint32_t*)&AT91_Gpio_InterruptHandler, nullptr);
}
