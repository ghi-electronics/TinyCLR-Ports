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
static const AT91_Gpio_Pin g_at91_uart_tx_pins[] = AT91_UART_TX_PINS;
static const AT91_Gpio_Pin g_at91_uart_rx_pins[] = AT91_UART_RX_PINS;
static const AT91_Gpio_Pin g_at91_uart_rts_pins[] = AT91_UART_RTS_PINS;
static const AT91_Gpio_Pin g_at91_uart_cts_pins[] = AT91_UART_CTS_PINS;

int32_t AT91_Uart_GetTxPin(int32_t portNum) {
    return g_at91_uart_tx_pins[portNum].number;
}

int32_t AT91_Uart_GetRxPin(int32_t portNum) {
    return g_at91_uart_rx_pins[portNum].number;
}

int32_t AT91_Uart_GetRtsPin(int32_t portNum) {
    return g_at91_uart_rts_pins[portNum].number;
}

int32_t AT91_Uart_GetCtsPin(int32_t portNum) {
    return g_at91_uart_cts_pins[portNum].number;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetTxAlternateFunction(int32_t portNum) {
    return g_at91_uart_tx_pins[portNum].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRxAlternateFunction(int32_t portNum) {
    return g_at91_uart_rx_pins[portNum].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetRtsAlternateFunction(int32_t portNum) {
    return g_at91_uart_rts_pins[portNum].peripheralSelection;
}

AT91_Gpio_PeripheralSelection AT91_Uart_GetCtsAlternateFunction(int32_t portNum) {
    return g_at91_uart_cts_pins[portNum].peripheralSelection;
}

// ADC
static const AT91_Gpio_Pin g_AT91_Adc_Pins[] = AT91_ADC_PINS;

int32_t AT91_Adc_GetControllerCount() {
    return SIZEOF_ARRAY(g_AT91_Adc_Pins);
}

int32_t AT91_Adc_GetPin(int32_t channel) {
    return  g_AT91_Adc_Pins[channel].number;
}

AT91_Gpio_PeripheralSelection AT91_Adc_GetPeripheralSelection(int32_t channel) {
    return  g_AT91_Adc_Pins[channel].peripheralSelection;
}

//PWM
static const AT91_Gpio_Pin g_at91_pwm_pins[TOTAL_PWM_CONTROLLER][MAX_PWM_PER_CONTROLLER] = AT91_PWM_PINS;

AT91_Gpio_Pin AT91_Pwm_GetPins(int32_t controller, int32_t channel) {
   return g_at91_pwm_pins[controller][channel];
}

