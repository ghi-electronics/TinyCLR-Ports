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

// G120, G120E
#define LPC17_G120                         1
#define LPC17_G120E                        2

#define G120E_DETECT1_PIN PIN(0,19)
#define G120E_DETECT2_PIN PIN(0,20)
#define G120E_DETECT3_PIN PIN(0,21)

#define G120E_DETECT1_STATE TinyCLR_Gpio_PinValue::Low
#define G120E_DETECT2_STATE TinyCLR_Gpio_PinValue::Low
#define G120E_DETECT3_STATE TinyCLR_Gpio_PinValue::Low


static int32_t lpc178_deviceId = -1;

int32_t LPC17_Startup_GetDeviceId() {
    TinyCLR_Gpio_Provider* provider = nullptr;

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


void LPC17_Startup_GetDebugger(const TinyCLR_Api_Info*& api, size_t& index) {
    TinyCLR_Gpio_PinValue value, valueUsbActive;
    auto controller = static_cast<const TinyCLR_Gpio_Provider*>(LPC17_Gpio_GetApi()->Implementation);

    controller->AcquirePin(controller, LPC17_Startup_GetDebuggerSelectorPin());
    controller->SetDriveMode(controller, LPC17_Startup_GetDebuggerSelectorPin(), LPC17_Startup_GetDebuggerSelectorPull());
    controller->Read(controller, LPC17_Startup_GetDebuggerSelectorPin(), value);
    controller->ReleasePin(controller, LPC17_Startup_GetDebuggerSelectorPin());

    valueUsbActive = LPC17_Startup_GetDebuggerSelectorUsbState();

    if (value == valueUsbActive) {
        api = LPC17_UsbClient_GetApi();
        index = USB_DEBUGGER_INDEX;
    }
    else {
        api = LPC17_Uart_GetApi();
        index = UART_DEBUGGER_INDEX;
    }
}

void LPC17_Startup_GetRunApp(bool& runApp) {
    TinyCLR_Gpio_PinValue value;
    auto controller = static_cast<const TinyCLR_Gpio_Provider*>(LPC17_Gpio_GetApi()->Implementation);
    controller->AcquirePin(controller, RUN_APP_PIN);
    controller->SetDriveMode(controller, RUN_APP_PIN, RUN_APP_PULL);
    controller->Read(controller, RUN_APP_PIN, value);
    controller->ReleasePin(controller, RUN_APP_PIN);

    runApp = value == RUN_APP_STATE;
}

// PWM
const LPC17_Gpio_Pin g_lpc17_g120_pwm_pins[TOTAL_PWM_CONTROLLER][MAX_PWM_PER_CONTROLLER] = LPC17_G120_PWM_PINS;
const LPC17_Gpio_Pin g_lpc17_g120e_pwm_pins[TOTAL_PWM_CONTROLLER][MAX_PWM_PER_CONTROLLER] = LPC17_G120E_PWM_PINS;

LPC17_Gpio_Pin LPC17_Pwm_GetPins(int32_t controller, int32_t channel) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120) {
        return g_lpc17_g120_pwm_pins[controller][channel];
    }
    else {
        return g_lpc17_g120e_pwm_pins[controller][channel];
    }
}

// Uart
static const LPC17_Gpio_Pin g_lpc17_g120_uart_tx_pins[] = LPC17_G120_UART_TX_PINS;
static const LPC17_Gpio_Pin g_lpc17_g120_uart_rx_pins[] = LPC17_G120_UART_RX_PINS;
static const LPC17_Gpio_Pin g_lpc17_g120_uart_rts_pins[] = LPC17_G120_UART_RTS_PINS;
static const LPC17_Gpio_Pin g_lpc17_g120_uart_cts_pins[] = LPC17_G120_UART_CTS_PINS;

static const LPC17_Gpio_Pin g_lpc17_g120e_uart_tx_pins[] = LPC17_G120E_UART_TX_PINS;
static const LPC17_Gpio_Pin g_lpc17_g120e_uart_rx_pins[] = LPC17_G120E_UART_RX_PINS;
static const LPC17_Gpio_Pin g_lpc17_g120e_uart_rts_pins[] = LPC17_G120E_UART_RTS_PINS;
static const LPC17_Gpio_Pin g_lpc17_g120e_uart_cts_pins[] = LPC17_G120E_UART_CTS_PINS;

int32_t LPC17_Uart_GetTxPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_tx_pins[portNum].number;
    else
        return g_lpc17_g120e_uart_tx_pins[portNum].number;
}

int32_t LPC17_Uart_GetRxPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_rx_pins[portNum].number;
    else
        return g_lpc17_g120e_uart_rx_pins[portNum].number;
}

int32_t LPC17_Uart_GetRtsPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_rts_pins[portNum].number;
    else
        return g_lpc17_g120e_uart_rts_pins[portNum].number;
}

int32_t LPC17_Uart_GetCtsPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_cts_pins[portNum].number;
    else
        return g_lpc17_g120e_uart_cts_pins[portNum].number;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetTxAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_tx_pins[portNum].pinFunction;
    else
        return g_lpc17_g120e_uart_tx_pins[portNum].pinFunction;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetRxAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_rx_pins[portNum].pinFunction;
    else
        return g_lpc17_g120e_uart_rx_pins[portNum].pinFunction;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetRtsAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_rts_pins[portNum].pinFunction;
    else
        return g_lpc17_g120e_uart_rts_pins[portNum].pinFunction;
}

LPC17_Gpio_PinFunction LPC17_Uart_GetCtsAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_lpc17_g120_uart_cts_pins[portNum].pinFunction;
    else
        return g_lpc17_g120e_uart_cts_pins[portNum].pinFunction;
}
