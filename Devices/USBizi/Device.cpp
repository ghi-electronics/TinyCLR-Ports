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

#include "../../Drivers/SPIDisplay/SPIDisplay.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_Devices.h"

void LPC24_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    apiManager->Add(apiManager, SPIDisplay_GetApi());
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices);
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

const TinyCLR_Startup_UartDebuggerConfiguration AT91_Startup_UartDebuggerConfiguration = {

};

void LPC24_Startup_GetDebuggerTransportApi(const TinyCLR_Api_Info*& api, const void*& configuration) {
#if defined(DEBUGGER_SELECTOR_PIN)
    TinyCLR_Gpio_PinValue value, valueUsbActive;
    auto provider = static_cast<const TinyCLR_Gpio_Controller*>(LPC24_Gpio_GetApi()->Implementation);

    provider->OpenPin(provider, DEBUGGER_SELECTOR_PIN);
    provider->SetDriveMode(provider, DEBUGGER_SELECTOR_PIN, DEBUGGER_SELECTOR_PULL);
    provider->Read(provider, DEBUGGER_SELECTOR_PIN, value);
    provider->ClosePin(provider, DEBUGGER_SELECTOR_PIN);

    valueUsbActive = DEBUGGER_SELECTOR_USB_STATE;

    if (value == valueUsbActive) {
        api = LPC24_UsbDevice_GetApi();
        configuration = (const void*)&LPC24_Startup_UsbDebuggerConfiguration;
    }
    else {
        api = LPC24_Uart_GetApi();
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
    auto provider = static_cast<const TinyCLR_Gpio_Controller*>(LPC24_Gpio_GetApi()->Implementation);

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

// UsbClient
void LPC24_UsbDevice_PinConfiguration() {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2) {
        OTGClkCtrl = 0x1F;
        while ((OTGClkSt & 0x1F) != 0x1F);

        LPC24_Gpio_ConfigurePin(PIN(0, 14), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction2, LPC24_Gpio_PinMode::Inactive); // connect pin
        LPC24_Gpio_ConfigurePin(PIN(0, 31), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction1, LPC24_Gpio_PinMode::Inactive); // D2+ pin. D2- has only USBD- function. no need to config

        OTGStCtrl |= 3;
    }
    else {
        // LPC2387
        OTGClkCtrl = (1 << 1) + (1 << 4);

        while ((OTGClkSt & ((1 << 1) + (1 << 4))) != ((1 << 1) + (1 << 4)));
        // connect
        LPC24_Gpio_ConfigurePin(PIN(2, 9), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction1, LPC24_Gpio_PinMode::Inactive);
        // D+
        LPC24_Gpio_ConfigurePin(PIN(0, 29), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction1, LPC24_Gpio_PinMode::Inactive);
        // D-
        LPC24_Gpio_ConfigurePin(PIN(0, 30), LPC24_Gpio_Direction::Input, LPC24_Gpio_PinFunction::PinFunction1, LPC24_Gpio_PinMode::Inactive);
    }
}

// Uart
static const LPC24_Gpio_Pin g_lpc2387_uart_tx_pins[] = LPC2387_UART_TX_PINS;
static const LPC24_Gpio_Pin g_lpc2387_uart_rx_pins[] = LPC2387_UART_RX_PINS;
static const LPC24_Gpio_Pin g_lpc2387_uart_rts_pins[] = LPC2387_UART_RTS_PINS;
static const LPC24_Gpio_Pin g_lpc2387_uart_cts_pins[] = LPC2387_UART_CTS_PINS;

static const LPC24_Gpio_Pin g_lpc2388_uart_tx_pins[] = LPC2388_UART_TX_PINS;
static const LPC24_Gpio_Pin g_lpc2388_uart_rx_pins[] = LPC2388_UART_RX_PINS;
static const LPC24_Gpio_Pin g_lpc2388_uart_rts_pins[] = LPC2388_UART_RTS_PINS;
static const LPC24_Gpio_Pin g_lpc2388_uart_cts_pins[] = LPC2388_UART_CTS_PINS;


int32_t LPC24_Uart_GetTxPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_tx_pins[portNum].number;
    else
        return g_lpc2387_uart_tx_pins[portNum].number;
}

int32_t LPC24_Uart_GetRxPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_rx_pins[portNum].number;
    else
        return g_lpc2387_uart_rx_pins[portNum].number;
}

int32_t LPC24_Uart_GetRtsPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_rts_pins[portNum].number;
    else
        return g_lpc2387_uart_rts_pins[portNum].number;
}

int32_t LPC24_Uart_GetCtsPin(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_cts_pins[portNum].number;
    else
        return g_lpc2387_uart_cts_pins[portNum].number;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetTxAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_tx_pins[portNum].pinFunction;
    else
        return g_lpc2387_uart_tx_pins[portNum].pinFunction;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetRxAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_rx_pins[portNum].pinFunction;
    else
        return g_lpc2387_uart_rx_pins[portNum].pinFunction;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetRtsAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_rts_pins[portNum].pinFunction;
    else
        return g_lpc2387_uart_rts_pins[portNum].pinFunction;
}

LPC24_Gpio_PinFunction LPC24_Uart_GetCtsAlternateFunction(int32_t portNum) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return g_lpc2388_uart_cts_pins[portNum].pinFunction;
    else
        return g_lpc2387_uart_cts_pins[portNum].pinFunction;
}

// ADC
static const LPC24_Gpio_Pin g_lpc2388_adcPins[] = LPC2388_ADC_PINS;
static const LPC24_Gpio_Pin g_lpc2387_adcPins[] = LPC2387_ADC_PINS;

uint32_t LPC24_Adc_GetChannelCount() {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return  SIZEOF_ARRAY(g_lpc2388_adcPins);
    else
        return  SIZEOF_ARRAY(g_lpc2387_adcPins);
}

uint32_t LPC24_Adc_GetPin(uint32_t channel) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return  g_lpc2388_adcPins[channel].number;
    else
        return  g_lpc2387_adcPins[channel].number;
}

LPC24_Gpio_PinFunction LPC24_Adc_GetPinFunction(uint32_t channel) {
    if (LPC24_Startup_GetDeviceId() != LPC2387_PARTID_1 && LPC24_Startup_GetDeviceId() != LPC2387_PARTID_2)
        return  g_lpc2388_adcPins[channel].pinFunction;
    else
        return  g_lpc2387_adcPins[channel].pinFunction;
}

//PWM
const LPC24_Gpio_Pin g_lpc24_pwm_pins[MAX_PWM_PER_CONTROLLER] = LPC24_PWM_PINS;

LPC24_Gpio_Pin LPC24_Pwm_GetPins(int32_t controller, int32_t channel) {
    return g_lpc24_pwm_pins[channel];
}
