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

#define EXPAND(a) CONCAT(DEVICE_TARGET, a)

void OnSoftReset(const TinyCLR_Api_Provider* apiProvider) {
#ifdef INCLUDE_ADC
    apiProvider->Add(apiProvider, EXPAND(_Adc_GetApi)());
    apiProvider->SetDefaultSelector(apiProvider, TinyCLR_Api_Type::AdcProvider, EXPAND(_Adc_GetApi)()->Name);

    EXPAND(_Adc_Reset)();
#endif

#ifdef INCLUDE_DAC
    apiProvider->Add(apiProvider, EXPAND(_Dac_GetApi)());
    apiProvider->SetDefaultSelector(apiProvider, TinyCLR_Api_Type::DacProvider, EXPAND(_Dac_GetApi)()->Name);

    EXPAND(_Dac_Reset)();
#endif

#ifdef INCLUDE_DISPLAY
    apiProvider->Add(apiProvider, EXPAND(_Display_GetApi)());
    apiProvider->SetDefaultSelector(apiProvider, TinyCLR_Api_Type::DisplayProvider, EXPAND(_Display_GetApi)()->Name);

    EXPAND(_Display_Reset)();
#endif

#ifdef INCLUDE_GPIO
    apiProvider->Add(apiProvider, EXPAND(_Gpio_GetApi)());
    apiProvider->SetDefaultSelector(apiProvider, TinyCLR_Api_Type::GpioProvider, EXPAND(_Gpio_GetApi)()->Name);

    EXPAND(_Gpio_Reset)();
#endif

#ifdef INCLUDE_I2C
    apiProvider->Add(apiProvider, EXPAND(_I2c_GetApi)());

    EXPAND(_I2c_Reset)();
#endif

#ifdef INCLUDE_PWM
    apiProvider->Add(apiProvider, EXPAND(_Pwm_GetApi)());

    EXPAND(_Pwm_Reset)();
#endif

#ifdef INCLUDE_SPI
    apiProvider->Add(apiProvider, EXPAND(_Spi_GetApi)());

    EXPAND(_Spi_Reset)();
#endif

#ifdef INCLUDE_UART
    apiProvider->Add(apiProvider, EXPAND(_Uart_GetApi)());

    EXPAND(_Uart_Reset)();
#endif

#ifdef INCLUDE_USBCLIENT
    apiProvider->Add(apiProvider, EXPAND(_UsbClient_GetApi)());

    EXPAND(_UsbClient_Reset)();
#endif
}

int main() {
    EXPAND(_Startup_Initialize)();


    uint8_t* heapStart;
    size_t heapLength;

    EXPAND(_Startup_GetHeap)(heapStart, heapLength);
    TinyCLR_Startup_AddHeapRegion(heapStart, heapLength);


    const TinyCLR_Api_Info* debuggerApi;
    size_t debuggerIndex;

    EXPAND(_Startup_GetDebugger)(debuggerApi, debuggerIndex);
    TinyCLR_Startup_SetDebugger(debuggerApi, debuggerIndex);


    TinyCLR_Startup_SetDeviceInformation(DEVICE_NAME, DEVICE_MANUFACTURER, DEVICE_VERSION);

    TinyCLR_Startup_SetRequiredProviders(EXPAND(_Deployment_GetApi)(), EXPAND(_Interrupt_GetApi)(), EXPAND(_Power_GetApi)(), EXPAND(_Time_GetApi)());


    auto runApp = true;

    EXPAND(_Startup_GetRunApp)(runApp);
    TinyCLR_Startup_Start(&OnSoftReset, runApp);

    return 0;
}
