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

#include "AT91SAM9Rx64.h"
#define PIO_PPDDR(x)	(*(volatile unsigned long *)(0xFFFFF490 + (x * 0x200))) // Pull-down Disable Resistor Register -- Write Only
#define PIO_PPDER(x)	(*(volatile unsigned long *)(0xFFFFF494 + (x * 0x200))) // Pull-down Enable Resistor Register -- Write Only

#define GETPORT(pin)    (pin/32)
#define GETBIT(pin)     (pin%32)

#define TOTAL_GPIO_CONTROLLERS 1
#define MAX_PORT (TOTAL_GPIO_PINS/32)

#define DEBOUNCE_DEFAULT_TICKS (20*10000) // 20ms in ticks

#define TOTAL_GPIO_PINS SIZEOF_ARRAY(gpioPins)

#define TOTAL_GPIO_INTERRUPT_PINS TOTAL_GPIO_PINS

static const AT91SAM9Rx64_Gpio_PinConfiguration gpioPins[] = AT91SAM9Rx64_GPIO_PINS;

struct GpioInterruptState {
    uint8_t pin;
    int64_t debounce;
    uint64_t  lastDebounceTicks;

    const TinyCLR_Gpio_Controller* controller;
    TinyCLR_Gpio_PinChangedHandler handler;
    TinyCLR_Gpio_PinValue currentValue;
    TinyCLR_Gpio_PinChangeEdge edge;
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
static TinyCLR_Gpio_PinValue previousOutputValue[TOTAL_GPIO_PINS];

static TinyCLR_Gpio_Controller gpioControllers[TOTAL_GPIO_PINS];
static TinyCLR_Api_Info gpioApi[TOTAL_GPIO_PINS];

const char* GpioApiNames[TOTAL_GPIO_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.GpioController\\0"
};

void AT91SAM9Rx64_Gpio_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_GPIO_CONTROLLERS; i++) {
        if (gpioStates[i].tableInitialized)
            continue;

        gpioControllers[i].ApiInfo = &gpioApi[i];
        gpioControllers[i].Acquire = &AT91SAM9Rx64_Gpio_Acquire;
        gpioControllers[i].Release = &AT91SAM9Rx64_Gpio_Release;
        gpioControllers[i].OpenPin = &AT91SAM9Rx64_Gpio_OpenPin;
        gpioControllers[i].ClosePin = &AT91SAM9Rx64_Gpio_ClosePin;
        gpioControllers[i].Read = &AT91SAM9Rx64_Gpio_Read;
        gpioControllers[i].Write = &AT91SAM9Rx64_Gpio_Write;
        gpioControllers[i].IsDriveModeSupported = &AT91SAM9Rx64_Gpio_IsDriveModeSupported;
        gpioControllers[i].GetDriveMode = &AT91SAM9Rx64_Gpio_GetDriveMode;
        gpioControllers[i].SetDriveMode = &AT91SAM9Rx64_Gpio_SetDriveMode;
        gpioControllers[i].GetDebounceTimeout = &AT91SAM9Rx64_Gpio_GetDebounceTimeout;
        gpioControllers[i].SetDebounceTimeout = &AT91SAM9Rx64_Gpio_SetDebounceTimeout;
        gpioControllers[i].SetPinChangedHandler = &AT91SAM9Rx64_Gpio_SetPinChangedHandler;
        gpioControllers[i].GetPinCount = &AT91SAM9Rx64_Gpio_GetPinCount;

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

const TinyCLR_Api_Info* AT91SAM9Rx64_Gpio_GetRequiredApi() {
    AT91SAM9Rx64_Gpio_EnsureTableInitialized();

    return &gpioApi[0];
}

void AT91SAM9Rx64_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91SAM9Rx64_Gpio_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_GPIO_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &gpioApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::GpioController, AT91SAM9Rx64_Gpio_GetRequiredApi()->Name);
}

TinyCLR_Result AT91SAM9Rx64_Gpio_Acquire(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Gpio_Release(const TinyCLR_Gpio_Controller* self) {
    return TinyCLR_Result::Success;
}

void AT91SAM9Rx64_Gpio_InterruptHandler(void* param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    for (auto port = 0; port < MAX_PORT; port++) {

        AT91SAM9Rx64_PIO &pioX = AT91::PIO(port);

        uint32_t bitMask = 0x00000001;
        uint32_t bitIndex = 0;

        uint32_t interruptsActive = pioX.PIO_ISR;

        interruptsActive &= pioX.PIO_IMR;

        while (interruptsActive) {
            while ((interruptsActive & bitMask) == 0) {
                bitMask <<= 1;
                bitIndex++;
            }

            bool executeIsr = false;

            GpioInterruptState* interruptState = &gpioInterruptState[bitIndex + port * 32];

            AT91SAM9Rx64_Gpio_Read(interruptState->controller, interruptState->pin, interruptState->currentValue); // read value as soon as possible

            auto edge = interruptState->currentValue == TinyCLR_Gpio_PinValue::High ? TinyCLR_Gpio_PinChangeEdge::RisingEdge : TinyCLR_Gpio_PinChangeEdge::FallingEdge;
            auto expectedEdgeInterger = static_cast<uint32_t>(interruptState->edge);
            auto currentEdgeInterger = static_cast<uint32_t>(edge);

            if (interruptState->handler && ((expectedEdgeInterger & currentEdgeInterger) || (expectedEdgeInterger == 0))) {
                if (interruptState->debounce) {
                    if ((AT91SAM9Rx64_Time_GetTimeForProcessorTicks(nullptr, AT91SAM9Rx64_Time_GetCurrentProcessorTicks(nullptr)) - interruptState->lastDebounceTicks) >= gpioDebounceInTicks[interruptState->pin]) {
                        executeIsr = true;
                    }

                    interruptState->lastDebounceTicks = AT91SAM9Rx64_Time_GetTimeForProcessorTicks(nullptr, AT91SAM9Rx64_Time_GetCurrentProcessorTicks(nullptr));
                }

                if (executeIsr)
                    interruptState->handler(interruptState->controller, interruptState->pin, edge, AT91SAM9Rx64_Time_GetSystemTime(nullptr));
            }

            interruptsActive ^= bitMask;
        }
    }
}


TinyCLR_Result AT91SAM9Rx64_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler) {
    GpioInterruptState* interruptState = &gpioInterruptState[pin];

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<GpioState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    uint32_t port = GETPORT(pin);
    uint32_t bitmask = 1 << GETBIT(pin);

    AT91SAM9Rx64_PIO &pioX = AT91::PIO(port);

    AT91SAM9Rx64_GpioInternal_EnableInputPin(pin, pinDriveMode[pin]);

    if (handler) {
        interruptState->controller = &gpioControllers[controllerIndex];
        interruptState->pin = (uint8_t)pin;
        interruptState->debounce = AT91SAM9Rx64_Gpio_GetDebounceTimeout(self, pin);
        interruptState->handler = handler;
        interruptState->lastDebounceTicks = AT91SAM9Rx64_Time_GetTimeForProcessorTicks(nullptr, AT91SAM9Rx64_Time_GetCurrentProcessorTicks(nullptr));
        interruptState->edge = edge;

        pioX.PIO_IER = bitmask; // Enable interrupt
    }
    else {
        pioX.PIO_IDR = bitmask; // Disable interrupt
    }

    return TinyCLR_Result::Success;
}
bool AT91SAM9Rx64_Gpio_Disable_Interrupt(uint32_t pin) {
    return true;
}

bool AT91SAM9Rx64_GpioInternal_OpenPin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    if (pinReserved[pin])
        return false;

    pinReserved[pin] = true;

    return true;
}

bool AT91SAM9Rx64_GpioInternal_ClosePin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    pinReserved[pin] = false;

    // reset to default interruptState
    return AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
}

bool AT91SAM9Rx64_GpioInternal_OpenMultiPins(const AT91SAM9Rx64_Gpio_Pin* pins, size_t count) {
    for (auto i = 0; i < count; i++) {
        if (!AT91SAM9Rx64_GpioInternal_OpenPin(pins[i].number)) {
            for (auto ii = 0; ii < i; ii++) {
                AT91SAM9Rx64_GpioInternal_ClosePin(pins[ii].number);
            }

            return false;
        }
    }

    return true;
}

bool AT91SAM9Rx64_GpioInternal_ReadPin(int32_t pin) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    uint32_t  bit = GETBIT(pin);
    uint32_t  port = GETPORT(pin);

    AT91SAM9Rx64_PIO &pioX = AT91::PIO(port);

    return (((pioX.PIO_PDSR) >> bit) & 1) ? true : false;
}

void AT91SAM9Rx64_GpioInternal_WritePin(int32_t pin, bool value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return;

    uint32_t  port = GETPORT(pin);
    uint32_t  bit = GETBIT(pin);

    AT91SAM9Rx64_PIO &pioX = AT91::PIO(port);

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (value) {
        pioX.PIO_SODR = 1 << bit;
    }
    else {
        pioX.PIO_CODR = 1 << bit;
    }
}

bool AT91SAM9Rx64_GpioInternal_ConfigurePin(int32_t pin, AT91SAM9Rx64_Gpio_Direction pinDir, AT91SAM9Rx64_Gpio_PeripheralSelection peripheralSelection, AT91SAM9Rx64_Gpio_ResistorMode resistorMode, AT91SAM9Rx64_Gpio_MultiDriver multiDrive, AT91SAM9Rx64_Gpio_Filter filter, AT91SAM9Rx64_Gpio_FilterSlowClock filterSlowClock, AT91SAM9Rx64_Gpio_Schmitt schmitt, AT91SAM9Rx64_Gpio_DriveSpeed driveSpeed) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return false;

    uint32_t bitmask = 1 << GETBIT(pin);
    int32_t port = GETPORT(pin);

    AT91SAM9Rx64_PIO &pioX = AT91::PIO(port);

    if (peripheralSelection == AT91SAM9Rx64_Gpio_PeripheralSelection::None) {
        pioX.PIO_PER = bitmask; // Enable PIO function
        switch (pinDir) {
        case AT91SAM9Rx64_Gpio_Direction::Output:

            pioX.PIO_PPUDR = bitmask;	// Disable the pull up resistor
            PIO_PPDDR(port) = bitmask;	// Disable the pull-down resistor

            pioX.PIO_OER = bitmask; // Enable Output

            break;

        case AT91SAM9Rx64_Gpio_Direction::Input:
            pioX.PIO_ODR = bitmask; // Disable Output

            if (filter == AT91SAM9Rx64_Gpio_Filter::Enable) {
                pioX.PIO_IFER = bitmask;
            }
            else {
                pioX.PIO_IFDR = bitmask;
            }



            switch (resistorMode) {
            case AT91SAM9Rx64_Gpio_ResistorMode::Inactive:
                pioX.PIO_PPUDR = bitmask;	// Disable the pull up resistor
                PIO_PPDDR(port) = bitmask;	// Disable the pull-down resistor
                break;

            case AT91SAM9Rx64_Gpio_ResistorMode::PullUp:
                PIO_PPDDR(port) = bitmask;	// Disable the pull-down resistor first.
                pioX.PIO_PPUER = bitmask;	// Enable the pull up resistor
                break;

            case AT91SAM9Rx64_Gpio_ResistorMode::PullDown:
                pioX.PIO_PPUDR = bitmask;	// Disable the pull up resistor first.
                PIO_PPDER(port) = bitmask;	// Enables the pull-down resistor
                break;
            }
            break;
        }
    }
    else {
        if (resistorMode == AT91SAM9Rx64_Gpio_ResistorMode::Inactive)
            pioX.PIO_PPUDR = bitmask;         // Disable the pull up resistor
        else if (resistorMode == AT91SAM9Rx64_Gpio_ResistorMode::PullUp)
            pioX.PIO_PPUER = bitmask;            // Enable the pull up resistor

        switch (peripheralSelection) {
        case AT91SAM9Rx64_Gpio_PeripheralSelection::PeripheralA:
            pioX.PIO_PDR = bitmask;
            pioX.PIO_ASR = bitmask; // Writes a zero to bit that is masked

            break;

        case AT91SAM9Rx64_Gpio_PeripheralSelection::PeripheralB:
            pioX.PIO_PDR = bitmask;
            pioX.PIO_BSR = bitmask;
            break;

        }

        pioX.PIO_IDR = bitmask; // Disable the Input Change Interrupt
    }

    return true;
}


bool AT91SAM9Rx64_GpioInternal_ConfigurePin(int32_t pin, AT91SAM9Rx64_Gpio_Direction pinDir, AT91SAM9Rx64_Gpio_PeripheralSelection peripheralSelection, AT91SAM9Rx64_Gpio_ResistorMode resistorMode) {
    return AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, pinDir, peripheralSelection, resistorMode, AT91SAM9Rx64_Gpio_MultiDriver::Disable, AT91SAM9Rx64_Gpio_Filter::Enable, AT91SAM9Rx64_Gpio_FilterSlowClock::Disable, AT91SAM9Rx64_Gpio_Schmitt::Disable, AT91SAM9Rx64_Gpio_DriveSpeed::Reserved);
}

void AT91SAM9Rx64_GpioInternal_EnableOutputPin(int32_t pin, bool initialState) {
    AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Output, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
    AT91SAM9Rx64_GpioInternal_WritePin(pin, initialState);
}

void AT91SAM9Rx64_GpioInternal_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode mode) {
    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Input:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
        break;
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::PullUp);
        break;
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::PullDown);
        break;
    }
}

TinyCLR_Result AT91SAM9Rx64_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    value = AT91SAM9Rx64_GpioInternal_ReadPin(pin) ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    AT91SAM9Rx64_GpioInternal_WritePin(pin, value == TinyCLR_Gpio_PinValue::High ? true : false);

    previousOutputValue[pin] = value;
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    if (!AT91SAM9Rx64_GpioInternal_OpenPin(pin))
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    AT91SAM9Rx64_GpioInternal_ClosePin(pin);

    return TinyCLR_Result::Success;
}

bool AT91SAM9Rx64_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode) {

    switch (mode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
    case TinyCLR_Gpio_PinDriveMode::Input:
    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        return true;
    }

    return false;
}

TinyCLR_Gpio_PinDriveMode AT91SAM9Rx64_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    return pinDriveMode[pin];
}

TinyCLR_Result AT91SAM9Rx64_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode driveMode) {
    if (pin >= TOTAL_GPIO_PINS || pin < 0)
        return TinyCLR_Result::ArgumentOutOfRange;

    switch (driveMode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Output, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);

        AT91SAM9Rx64_Gpio_Write(self, pin, previousOutputValue[pin]);
        break;

    case TinyCLR_Gpio_PinDriveMode::Input:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::Inactive);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::PullUp);
        break;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, AT91SAM9Rx64_Gpio_Direction::Input, AT91SAM9Rx64_Gpio_PeripheralSelection::None, AT91SAM9Rx64_Gpio_ResistorMode::PullDown);
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

uint64_t AT91SAM9Rx64_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin) {
    return gpioDebounceInTicks[pin];
}

TinyCLR_Result AT91SAM9Rx64_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks) {
    gpioDebounceInTicks[pin] = debounceTicks;

    return TinyCLR_Result::Success;
}

uint32_t AT91SAM9Rx64_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self) {
    return TOTAL_GPIO_PINS;
}

void AT91SAM9Rx64_Gpio_Reset() {
    AT91SAM9Rx64_PMC &pmc = AT91::PMC();

    pmc.EnablePeriphClock(AT91C_ID_PIOA);
    pmc.EnablePeriphClock(AT91C_ID_PIOB);
    pmc.EnablePeriphClock(AT91C_ID_PIOC);
    pmc.EnablePeriphClock(AT91C_ID_PIOD);

    for (auto port = 0; port < MAX_PORT; port++) {
        // initialize pins as free
        AT91SAM9Rx64_PIO &pioX = AT91::PIO(port);

        pioX.PIO_IDR = 0xffffffff;                // Disable all interrupts
        pioX.PIO_ISR ^= 0xffffffff;
    }

    for (auto pin = 0; pin < AT91SAM9Rx64_Gpio_GetPinCount(nullptr); pin++) {
        auto& p = gpioPins[pin];

        pinReserved[pin] = false;
        previousOutputValue[pin] = TinyCLR_Gpio_PinValue::Low;

        AT91SAM9Rx64_Gpio_SetDebounceTimeout(nullptr, pin, DEBOUNCE_DEFAULT_TICKS);

        if (p.apply) {
            AT91SAM9Rx64_GpioInternal_ConfigurePin(pin, p.direction, p.peripheralSelection, p.resistorMode);

            if (p.direction == AT91SAM9Rx64_Gpio_Direction::Output)
                AT91SAM9Rx64_GpioInternal_WritePin(pin, p.outputDirection);
        }
    }

    AT91SAM9Rx64_InterruptInternal_Activate(AT91C_ID_PIOA, (uint32_t*)&AT91SAM9Rx64_Gpio_InterruptHandler, nullptr);
    AT91SAM9Rx64_InterruptInternal_Activate(AT91C_ID_PIOB, (uint32_t*)&AT91SAM9Rx64_Gpio_InterruptHandler, nullptr);
    AT91SAM9Rx64_InterruptInternal_Activate(AT91C_ID_PIOC, (uint32_t*)&AT91SAM9Rx64_Gpio_InterruptHandler, nullptr);
    AT91SAM9Rx64_InterruptInternal_Activate(AT91C_ID_PIOD, (uint32_t*)&AT91SAM9Rx64_Gpio_InterruptHandler, nullptr);

    gpioStates[0].tableInitialized = false;
}

