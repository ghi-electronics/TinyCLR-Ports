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

#include "../../Drivers/AT49BV322DT_Flash/AT49BV322DT_Flash.h"

#include "../../Drivers/SPIDisplay/SPIDisplay.h"

void LPC24_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider) {
    apiProvider->Add(apiProvider, SPIDisplay_GetApi());
    
    extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices;

    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->Add(interopProvider, &Interop_GHIElectronics_TinyCLR_Devices);
}

static int32_t lpc24_deviceId = 0;

int32_t LPC24_Startup_GetDeviceId() {

    if (lpc24_deviceId == 0) {
        lpc24_deviceId = LPC24_Flash_GetPartId();
    }

    return lpc24_deviceId;
}

const TinyCLR_Startup_UsbDebuggerConfiguration LPC24_Startup_UsbDebuggerConfiguration = {
    USB_DEBUGGER_VENDOR_ID,
    USB_DEBUGGER_PRODUCT_ID,
    CONCAT(L,DEVICE_MANUFACTURER),
    CONCAT(L,DEVICE_NAME),
    0
};
void LPC24_Startup_GetDebuggerTransportProvider(const TinyCLR_Api_Info*& api, size_t& index, const void*& configuration) {
#if defined(DEBUGGER_SELECTOR_PIN)
    TinyCLR_Gpio_PinValue value, valueUsbActive;
    auto controller = static_cast<const TinyCLR_Gpio_Provider*>(LPC24_Gpio_GetApi()->Implementation);

    controller->AcquirePin(controller, DEBUGGER_SELECTOR_PIN);
    controller->SetDriveMode(controller, DEBUGGER_SELECTOR_PIN, DEBUGGER_SELECTOR_PULL);
    controller->Read(controller, DEBUGGER_SELECTOR_PIN, value);
    controller->ReleasePin(controller, DEBUGGER_SELECTOR_PIN);

    valueUsbActive = DEBUGGER_SELECTOR_USB_STATE;

    if (value == valueUsbActive) {
        api = LPC24_UsbClient_GetApi();
        index = USB_DEBUGGER_INDEX;
        configuration = (const void*)&LPC24_Startup_UsbDebuggerConfiguration;
    }
    else {
        api = LPC24_Uart_GetApi();
        index = UART_DEBUGGER_INDEX;
    }
#elif defined(DEBUGGER_FORCE_API) && defined(DEBUGGER_FORCE_INDEX)
    api = DEBUGGER_FORCE_API;
    index = DEBUGGER_FORCE_INDEX;
#else
#error You must specify a debugger mode pin or specify the API explicitly.
#endif
}

void LPC24_Startup_GetRunApp(bool& runApp) {
#if defined(RUN_APP_PIN)
    TinyCLR_Gpio_PinValue value;
    auto controller = static_cast<const TinyCLR_Gpio_Provider*>(LPC24_Gpio_GetApi()->Implementation);
    controller->AcquirePin(controller, RUN_APP_PIN);
    controller->SetDriveMode(controller, RUN_APP_PIN, RUN_APP_PULL);
    controller->Read(controller, RUN_APP_PIN, value);
    controller->ReleasePin(controller, RUN_APP_PIN);

    runApp = value == RUN_APP_STATE;
#elif defined(RUN_APP_FORCE_STATE)
    runApp = RUN_APP_FORCE_STATE;
#else
    runApp = true;
#endif	
}

// UsbClient
void LPC24_UsbClient_PinConfiguration() {
    OTGClkCtrl = 0x1F;
    while ((OTGClkSt & 0x1F) != 0x1F);

    LPC24_Gpio_ConfigurePin(PIN(0, 14), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction2, LPC24_Gpio_PinMode::Inactive); // connect pin
    LPC24_Gpio_ConfigurePin(PIN(0, 31), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction1, LPC24_Gpio_PinMode::Inactive); // D2+ pin. D2- has only USBD- function. no need to config

    OTGStCtrl |= 3;
}

// Uart
// TFT version
static const LPC24_Gpio_Pin g_lpc2478_uart_tx_pins[] = LPC2478_UART_TX_PINS;
static const LPC24_Gpio_Pin g_lpc2478_uart_rx_pins[] = LPC2478_UART_RX_PINS;
static const LPC24_Gpio_Pin g_lpc2478_uart_rts_pins[] = LPC2478_UART_RTS_PINS;
static const LPC24_Gpio_Pin g_lpc2478_uart_cts_pins[] = LPC2478_UART_CTS_PINS;


// Non-TFT version
static const LPC24_Gpio_Pin g_lpc2468_uart_tx_pins[] = LPC2468_UART_TX_PINS;
static const LPC24_Gpio_Pin g_lpc2468_uart_rx_pins[] = LPC2468_UART_RX_PINS;
static const LPC24_Gpio_Pin g_lpc2468_uart_rts_pins[] = LPC2468_UART_RTS_PINS;
static const LPC24_Gpio_Pin g_lpc2468_uart_cts_pins[] = LPC2468_UART_CTS_PINS;

int32_t LPC24_Uart_GetTxPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_tx_pins[portNum].number;
    else
        return g_lpc2478_uart_tx_pins[portNum].number;
}

int32_t LPC24_Uart_GetRxPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_rx_pins[portNum].number;
    else
        return g_lpc2478_uart_rx_pins[portNum].number;
}

int32_t LPC24_Uart_GetRtsPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_rts_pins[portNum].number;
    else
        return g_lpc2478_uart_rts_pins[portNum].number;
}

int32_t LPC24_Uart_GetCtsPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_cts_pins[portNum].number;
    else
        return g_lpc2478_uart_cts_pins[portNum].number;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetTxAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_tx_pins[portNum].pinFunction;
    else
        return g_lpc2478_uart_tx_pins[portNum].pinFunction;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetRxAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_rx_pins[portNum].pinFunction;
    else
        return g_lpc2478_uart_rx_pins[portNum].pinFunction;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetRtsAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_rts_pins[portNum].pinFunction;
    else
        return g_lpc2478_uart_rts_pins[portNum].pinFunction;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetCtsAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_uart_cts_pins[portNum].pinFunction;
    else
        return g_lpc2478_uart_cts_pins[portNum].pinFunction;
}

// ADC
static const LPC24_Gpio_Pin g_lpc24_adc_pins[] = LPC24_ADC_PINS;
int32_t LPC24_Adc_GetControllerCount() {
    return SIZEOF_ARRAY(g_lpc24_adc_pins);
}

int32_t LPC24_Adc_GetPin(int32_t channel) {
    return  g_lpc24_adc_pins[channel].number;
}

LPC24_Gpio_PinFunction LPC24_Adc_GetPinFunction(int32_t channel) {
    return  g_lpc24_adc_pins[channel].pinFunction;
}

//PWM
const LPC24_Gpio_Pin g_lpc2478_pwm_pins[TOTAL_PWM_CONTROLLER][MAX_PWM_PER_CONTROLLER] = LPC2478_PWM_PINS;
const LPC24_Gpio_Pin g_lpc2468_pwm_pins[TOTAL_PWM_CONTROLLER][MAX_PWM_PER_CONTROLLER] = LPC2468_PWM_PINS;

LPC24_Gpio_Pin LPC24_Pwm_GetPins(int32_t controller, int32_t channel) {
    if (LPC24_Startup_GetDeviceId() == LPC2468_PARTID)
        return g_lpc2468_pwm_pins[controller][channel];
    else
        return g_lpc2478_pwm_pins[controller][channel];
}


