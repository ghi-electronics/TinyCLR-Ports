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

#include <TinyCLR.h>
#include <LPC17.h>
#include <Device.h>

#include "../../Drivers/SPIDisplay/SPIDisplay.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_Devices.h"

// G120, G120E
#define LPC17_G120                         1
#define LPC17_G120E                        2

#define G120E_DETECT1_PIN PIN(0,19)
#define G120E_DETECT2_PIN PIN(0,20)
#define G120E_DETECT3_PIN PIN(0,21)

#define G120E_DETECT1_STATE TinyCLR_Gpio_PinValue::Low
#define G120E_DETECT2_STATE TinyCLR_Gpio_PinValue::Low
#define G120E_DETECT3_STATE TinyCLR_Gpio_PinValue::Low

void LPC17_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    SPIDisplay_AddApi(apiManager);
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices);
}

static int32_t lpc178_deviceId = -1;

int32_t LPC17_Startup_GetDeviceId() {
    TinyCLR_Gpio_Controller* provider = nullptr;

    auto gpioController = 0; //TODO Temporary set to 0

    if (lpc178_deviceId < 0) {
        TinyCLR_Gpio_PinValue value1, value2, value3;

        LPC17_Gpio_EnableInputPin(G120E_DETECT1_PIN, TinyCLR_Gpio_PinDriveMode::InputPullUp);
        LPC17_Gpio_EnableInputPin(G120E_DETECT2_PIN, TinyCLR_Gpio_PinDriveMode::InputPullUp);
        LPC17_Gpio_EnableInputPin(G120E_DETECT3_PIN, TinyCLR_Gpio_PinDriveMode::InputPullUp);

        LPC17_Gpio_Read(provider, G120E_DETECT1_PIN, value1);
        LPC17_Gpio_Read(provider, G120E_DETECT2_PIN, value2);
        LPC17_Gpio_Read(provider, G120E_DETECT3_PIN, value3);

        if ((G120E_DETECT1_STATE == value1) && (G120E_DETECT2_STATE == value2) && (G120E_DETECT3_STATE == value3))
            lpc178_deviceId = LPC17_G120E;
        else
            lpc178_deviceId = LPC17_G120;
    }

    return lpc178_deviceId;
}

int32_t LPC17_Startup_GetDebuggerSelectorPin() {
    return DEBUGGER_SELECTOR_PIN;
}

TinyCLR_Gpio_PinDriveMode LPC17_Startup_GetDebuggerSelectorPull() {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return G120_DEBUGGER_SELECTOR_PULL;
    else
        return G120E_DEBUGGER_SELECTOR_PULL;
}


TinyCLR_Gpio_PinValue LPC17_Startup_GetDebuggerSelectorUsbState() {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return G120_DEBUGGER_SELECTOR_USB_STATE;
    else
        return G120E_DEBUGGER_SELECTOR_USB_STATE;
}

const TinyCLR_Startup_UsbDebuggerConfiguration LPC17_Startup_UsbDebuggerConfiguration = {
    USB_DEBUGGER_VENDOR_ID,
    USB_DEBUGGER_PRODUCT_ID,
    CONCAT(L,DEVICE_MANUFACTURER),
    CONCAT(L,DEVICE_NAME),
    0
};

void LPC17_Startup_GetDebuggerTransportApi(const TinyCLR_Api_Info*& api, const void*& configuration) {
#if defined(DEBUGGER_SELECTOR_PIN)
    TinyCLR_Gpio_PinValue value, valueUsbActive;
    auto provider = static_cast<const TinyCLR_Gpio_Controller*>(LPC17_Gpio_GetRequiredApi()->Implementation);

    provider->OpenPin(provider, LPC17_Startup_GetDebuggerSelectorPin());
    provider->SetDriveMode(provider, LPC17_Startup_GetDebuggerSelectorPin(), LPC17_Startup_GetDebuggerSelectorPull());
    provider->Read(provider, LPC17_Startup_GetDebuggerSelectorPin(), value);
    provider->ClosePin(provider, LPC17_Startup_GetDebuggerSelectorPin());

    valueUsbActive = LPC17_Startup_GetDebuggerSelectorUsbState();

    if (value == valueUsbActive) {
        api = LPC17_UsbDevice_GetRequiredApi();
        configuration = (const void*)&LPC17_Startup_UsbDebuggerConfiguration;
    }
    else {
        api = LPC17_Uart_GetRequiredApi();        
    }
#elif defined(DEBUGGER_FORCE_API) && defined(DEBUGGER_FORCE_INDEX)
    api = DEBUGGER_FORCE_API;
    index = DEBUGGER_FORCE_INDEX;
#else
#error You must specify a debugger mode pin or specify the API explicitly.
#endif
}

void LPC17_Startup_GetRunApp(bool& runApp) {
#if defined(RUN_APP_PIN)
    TinyCLR_Gpio_PinValue value;
    auto provider = static_cast<const TinyCLR_Gpio_Controller*>(LPC17_Gpio_GetRequiredApi()->Implementation);

    provider->OpenPin(provider, RUN_APP_PIN);
    provider->SetDriveMode(provider, RUN_APP_PIN, RUN_APP_PULL);
    provider->Read(provider, RUN_APP_PIN, value);
    provider->ClosePin(provider, RUN_APP_PIN);

    runApp = value == RUN_APP_STATE;
#elif defined(RUN_APP_FORCE_STATE)
    runApp = RUN_APP_FORCE_STATE;
#else
    runApp = true;
#endif
}

// PWM
const LPC17_Gpio_Pin g120PwmPins[TOTAL_PWM_CONTROLLERS][MAX_PWM_PER_CONTROLLER] = LPC17_G120_PWM_PINS;
const LPC17_Gpio_Pin g120ePwmPins[TOTAL_PWM_CONTROLLERS][MAX_PWM_PER_CONTROLLER] = LPC17_G120E_PWM_PINS;

LPC17_Gpio_Pin LPC17_Pwm_GetPins(int32_t controller, int32_t channel) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120) {
        return g120PwmPins[controller][channel];
    }
    else {
        return g120ePwmPins[controller][channel];
    }
}

// Uart
static const LPC17_Gpio_Pin g120UartTxPins[] = LPC17_G120_UART_TX_PINS;
static const LPC17_Gpio_Pin g120UartRxPins[] = LPC17_G120_UART_RX_PINS;
static const LPC17_Gpio_Pin g120UartRtsPins[] = LPC17_G120_UART_RTS_PINS;
static const LPC17_Gpio_Pin g120UartCtsPins[] = LPC17_G120_UART_CTS_PINS;

static const LPC17_Gpio_Pin g120eUartTxPins[] = LPC17_G120E_UART_TX_PINS;
static const LPC17_Gpio_Pin g120eUartRxPins[] = LPC17_G120E_UART_RX_PINS;
static const LPC17_Gpio_Pin g120eUartRtsPins[] = LPC17_G120E_UART_RTS_PINS;
static const LPC17_Gpio_Pin g120eUartCtsPins[] = LPC17_G120E_UART_CTS_PINS;

int32_t LPC17_Uart_GetTxPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartTxPins[portNum].number;
    else
        return g120eUartTxPins[portNum].number;
}

int32_t LPC17_Uart_GetRxPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartRxPins[portNum].number;
    else
        return g120eUartRxPins[portNum].number;
}

int32_t LPC17_Uart_GetRtsPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartRtsPins[portNum].number;
    else
        return g120eUartRtsPins[portNum].number;
}

int32_t LPC17_Uart_GetCtsPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartCtsPins[portNum].number;
    else
        return g120eUartCtsPins[portNum].number;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetTxAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartTxPins[portNum].pinFunction;
    else
        return g120eUartTxPins[portNum].pinFunction;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetRxAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartRxPins[portNum].pinFunction;
    else
        return g120eUartRxPins[portNum].pinFunction;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetRtsAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartRtsPins[portNum].pinFunction;
    else
        return g120eUartRtsPins[portNum].pinFunction;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetCtsAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g120UartCtsPins[portNum].pinFunction;
    else
        return g120eUartCtsPins[portNum].pinFunction;
}
