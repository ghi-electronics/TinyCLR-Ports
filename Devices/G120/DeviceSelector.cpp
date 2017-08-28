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
#include <DeviceSelector.h>

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

        if ((G120E_DETECT1_STATE == (uint32_t)value1) && (G120E_DETECT2_STATE == (uint32_t)value2) && (G120E_DETECT3_STATE == (uint32_t)value3))
            lpc178_deviceId = LPC17_G120E;
        else
            lpc178_deviceId = LPC17_G120;
    }

    return lpc178_deviceId;
}

int32_t LPC17_Startup_GetLModePin() {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return G120_LMODE_PIN;
    else
        return G120E_LMODE_PIN;
}

TinyCLR_Gpio_PinValue LPC17_Startup_GetLModeUsbState() {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return G120_LMODE_USB_STATE;
    else
        return G120E_LMODE_USB_STATE;
}

// PWM
static PwmController g_LPC17_G120_PWM[TOTAL_PWM_CONTROLLER] = LPC17_G120_PWM;
static PwmController g_LPC17_G120E_PWM[TOTAL_PWM_CONTROLLER] = LPC17_G120E_PWM;

PwmController* LPC17_Pwm_GetControllers() {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return (PwmController*)&g_LPC17_G120_PWM;
    else
        return (PwmController*)&g_LPC17_G120E_PWM;
}

// Uart
static const uint32_t g_LPC17_G120_UART_TX_PINS[] = LPC17_G120_UART_TX_PINS;
static const uint32_t g_LPC17_G120_UART_RX_PINS[] = LPC17_G120_UART_RX_PINS;
static const uint32_t g_LPC17_G120_UART_RTS_PINS[] = LPC17_G120_UART_RTS_PINS;
static const uint32_t g_LPC17_G120_UART_CTS_PINS[] = LPC17_G120_UART_CTS_PINS;

static const uint32_t g_LPC17_G120E_UART_TX_PINS[] = LPC17_G120E_UART_TX_PINS;
static const uint32_t g_LPC17_G120E_UART_RX_PINS[] = LPC17_G120E_UART_RX_PINS;
static const uint32_t g_LPC17_G120E_UART_RTS_PINS[] = LPC17_G120E_UART_RTS_PINS;
static const uint32_t g_LPC17_G120E_UART_CTS_PINS[] = LPC17_G120E_UART_CTS_PINS;

static const LPC17_Gpio_PinFunction g_LPC17_G120_UART_TX_ALT_MODE[] = LPC17_G120_UART_TX_ALT_MODE;
static const LPC17_Gpio_PinFunction g_LPC17_G120_UART_RX_ALT_MODE[] = LPC17_G120_UART_RX_ALT_MODE;
static const LPC17_Gpio_PinFunction g_LPC17_G120_UART_RTS_ALT_MODE[] = LPC17_G120_UART_RTS_ALT_MODE;
static const LPC17_Gpio_PinFunction g_LPC17_G120_UART_CTS_ALT_MODE[] = LPC17_G120_UART_CTS_ALT_MODE;

static const LPC17_Gpio_PinFunction g_LPC17_G120E_UART_TX_ALT_MODE[] = LPC17_G120E_UART_TX_ALT_MODE;
static const LPC17_Gpio_PinFunction g_LPC17_G120E_UART_RX_ALT_MODE[] = LPC17_G120E_UART_RX_ALT_MODE;
static const LPC17_Gpio_PinFunction g_LPC17_G120E_UART_RTS_ALT_MODE[] = LPC17_G120E_UART_RTS_ALT_MODE;
static const LPC17_Gpio_PinFunction g_LPC17_G120E_UART_CTS_ALT_MODE[] = LPC17_G120E_UART_CTS_ALT_MODE;

int32_t LPC17_Uart_GetTxPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_TX_PINS[portNum];
    else
        return g_LPC17_G120E_UART_TX_PINS[portNum];
}

int32_t LPC17_Uart_GetRxPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_RX_PINS[portNum];
    else
        return g_LPC17_G120E_UART_RX_PINS[portNum];
}

int32_t LPC17_Uart_GetRtsPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_RTS_PINS[portNum];
    else
        return g_LPC17_G120E_UART_RTS_PINS[portNum];
}

int32_t LPC17_Uart_GetCtsPin(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_CTS_PINS[portNum];
    else
        return g_LPC17_G120E_UART_CTS_PINS[portNum];
}

LPC17_Gpio_PinFunction LPC17_Uart_GetTxAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_TX_ALT_MODE[portNum];
    else
        return g_LPC17_G120E_UART_TX_ALT_MODE[portNum];
}

LPC17_Gpio_PinFunction LPC17_Uart_GetRxAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_RX_ALT_MODE[portNum];
    else
        return g_LPC17_G120E_UART_RX_ALT_MODE[portNum];
}

LPC17_Gpio_PinFunction LPC17_Uart_GetRtsAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_RTS_ALT_MODE[portNum];
    else
        return g_LPC17_G120E_UART_RTS_ALT_MODE[portNum];
}

LPC17_Gpio_PinFunction LPC17_Uart_GetCtsAlternateFunction(int32_t portNum) {
    if (LPC17_Startup_GetDeviceId() == LPC17_G120)
        return g_LPC17_G120_UART_CTS_ALT_MODE[portNum];
    else
        return g_LPC17_G120E_UART_CTS_ALT_MODE[portNum];
}