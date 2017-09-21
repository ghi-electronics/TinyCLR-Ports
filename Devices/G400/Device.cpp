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
#include <Device.h>

// UsbClient
void AT91_UsbClient_PinConfiguration() {    
    
}

// Uart
static const uint32_t g_AT91_UART_TX_PINS[] = AT91_UART_TX_PINS;
static const uint32_t g_AT91_UART_RX_PINS[] = AT91_UART_RX_PINS;
static const uint32_t g_AT91_UART_RTS_PINS[] = AT91_UART_RTS_PINS;
static const uint32_t g_AT91_UART_CTS_PINS[] = AT91_UART_CTS_PINS;

static const AT91_Gpio_PeripheralSelection g_AT91_UART_TX_ALT_MODE[] = AT91_UART_TX_ALT_MODE;
static const AT91_Gpio_PeripheralSelection g_AT91_UART_RX_ALT_MODE[] = AT91_UART_RX_ALT_MODE;
static const AT91_Gpio_PeripheralSelection g_AT91_UART_RTS_ALT_MODE[] = AT91_UART_RTS_ALT_MODE;
static const AT91_Gpio_PeripheralSelection g_AT91_UART_CTS_ALT_MODE[] =  AT91_UART_CTS_ALT_MODE;

int32_t AT91_Uart_GetTxPin(int32_t portNum) {
    return g_AT91_UART_TX_PINS[portNum];
}

int32_t AT91_Uart_GetRxPin(int32_t portNum) {
    return g_AT91_UART_RX_PINS[portNum];
}

int32_t AT91_Uart_GetRtsPin(int32_t portNum) {
    return g_AT91_UART_RTS_PINS[portNum];
}

int32_t AT91_Uart_GetCtsPin(int32_t portNum) {
    return g_AT91_UART_CTS_PINS[portNum];
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetTxAlternateFunction(int32_t portNum) {
    return g_AT91_UART_TX_ALT_MODE[portNum];
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRxAlternateFunction(int32_t portNum) {
    return g_AT91_UART_RX_ALT_MODE[portNum];
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRtsAlternateFunction(int32_t portNum) {
    return g_AT91_UART_RTS_ALT_MODE[portNum];
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetCtsAlternateFunction(int32_t portNum) {
    return g_AT91_UART_CTS_ALT_MODE[portNum];
}

// ADC
static const uint32_t g_AT91_Adc_Pins[] = AT91_ADC_PINS;
static const AT91_Gpio_PeripheralSelection g_AT91_ADC_ALT_MODE[] = AT91_ADC_ALT_MODE ;

int32_t AT91_Adc_GetControllerCount() {
    return TOTAL_ADC_CONTROLLERS;
}

int32_t AT91_Adc_GetPin(int32_t channel) {
    return  g_AT91_Adc_Pins[channel];
}

AT91_Gpio_PeripheralSelection AT91_Adc_GetPeripheralSelection(int32_t channel) {
    return  g_AT91_ADC_ALT_MODE[channel];
}

//PWM
static PwmController g_AT91_PWM[] = AT91_PWM;

PwmController* AT91_Pwm_GetControllers() {
    return (PwmController*)&g_AT91_PWM;
}

