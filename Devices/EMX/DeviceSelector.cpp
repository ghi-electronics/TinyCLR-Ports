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

int32_t LPC24_Startup_GetLModePin() {
    return LMODE_PIN;
}

TinyCLR_Gpio_PinValue LPC24_Startup_GetLModeUsbState() {
    return LMODE_USB_STATE;
}

static int32_t lpc24_deviceId = 0;

int32_t LPC24_Startup_GetDeviceId() {

    if (lpc24_deviceId == 0) {
        lpc24_deviceId = LPC24_Flash_GetPartId();
    }

    return lpc24_deviceId;
}

// UsbClient
void LPC24_UsbClient_PinConfiguration() {
    OTGClkCtrl = 0x1F;
    while ((OTGClkSt & 0x1F) != 0x1F);

    LPC24_Gpio_ConfigurePin(_P(0, 14), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction2, LPC24_Gpio_PinMode::Inactive); // connect pin
    LPC24_Gpio_ConfigurePin(_P(0, 31), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction1, LPC24_Gpio_PinMode::Inactive); // D2+ pin. D2- has only USBD- function. no need to config

    OTGStCtrl |= 3;
}

// Uart
static const uint32_t g_LPC24_UART_TX_PINS[] = LPC24_UART_TX_PINS;
static const uint32_t g_LPC24_UART_RX_PINS[] = LPC24_UART_RX_PINS;
static const uint32_t g_LPC24_UART_RTS_PINS[] = LPC24_UART_RTS_PINS;
static const uint32_t g_LPC24_UART_CTS_PINS[] = LPC24_UART_CTS_PINS;

static const LPC24_Gpio_PinFunction g_LPC24_UART_TX_ALT_MODE[] = LPC24_UART_TX_ALT_MODE;
static const LPC24_Gpio_PinFunction g_LPC24_UART_RX_ALT_MODE[] = LPC24_UART_RX_ALT_MODE;
static const LPC24_Gpio_PinFunction g_LPC24_UART_RTS_ALT_MODE[] = LPC24_UART_RTS_ALT_MODE;
static const LPC24_Gpio_PinFunction g_LPC24_UART_CTS_ALT_MODE[] = LPC24_UART_CTS_ALT_MODE;

int32_t LPC24_Uart_GetTxPin(int32_t portNum) {
    return g_LPC24_UART_TX_PINS[portNum];
}

int32_t LPC24_Uart_GetRxPin(int32_t portNum) {
    return g_LPC24_UART_RX_PINS[portNum];
}

int32_t LPC24_Uart_GetRtsPin(int32_t portNum) {
    return g_LPC24_UART_RTS_PINS[portNum];
}

int32_t LPC24_Uart_GetCtsPin(int32_t portNum) {
    return g_LPC24_UART_CTS_PINS[portNum];
}

LPC24_Gpio_PinFunction LPC24_Uart_GetTxAlternateFunction(int32_t portNum) {
    return g_LPC24_UART_TX_ALT_MODE[portNum];
}

LPC24_Gpio_PinFunction LPC24_Uart_GetRxAlternateFunction(int32_t portNum) {
    return g_LPC24_UART_RX_ALT_MODE[portNum];
}

LPC24_Gpio_PinFunction LPC24_Uart_GetRtsAlternateFunction(int32_t portNum) {
    return g_LPC24_UART_RTS_ALT_MODE[portNum];
}

LPC24_Gpio_PinFunction LPC24_Uart_GetCtsAlternateFunction(int32_t portNum) {
    return g_LPC24_UART_CTS_ALT_MODE[portNum];
}

// ADC
static const uint32_t g_LPC24_Adc_Pins[] = LPC24_ADC_PINS;
static const LPC24_Gpio_PinFunction g_LPC24_Adc_altMode[] = LPC24_ADC_ALT_MODE;

int32_t LPC24_Adc_GetControllerCount() {
    return TOTAL_ADC_CONTROLLERS;
}

int32_t LPC24_Adc_GetPin(int32_t channel) {
    return  g_LPC24_Adc_Pins[channel];
}

LPC24_Gpio_PinFunction LPC24_Adc_GetPinFunction(int32_t channel) {
    return  g_LPC24_Adc_altMode[channel];
}

//PWM
static PwmController g_LPC24_PWM[TOTAL_PWM_CONTROLLER] = LPC24_PWM;

PwmController* LPC24_Pwm_GetControllers() {
    return (PwmController*)&g_LPC24_PWM;
}

