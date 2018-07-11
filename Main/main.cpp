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

#define TARGET(a) CONCAT(DEVICE_TARGET, a)

const TinyCLR_Api_Provider* apiProvider = nullptr;

void OnSoftReset(const TinyCLR_Api_Provider* apiProvider) {
    ::apiProvider = apiProvider;

#ifdef INCLUDE_ADC
    apiProvider->Add(apiProvider, TARGET(_Adc_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::AdcProvider, TARGET(_Adc_GetApi)()->Name);
#endif

#ifdef INCLUDE_CAN
    apiProvider->Add(apiProvider, TARGET(_Can_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::CanProvider, TARGET(_Can_GetApi)()->Name);
#endif

#ifdef INCLUDE_DAC
    apiProvider->Add(apiProvider, TARGET(_Dac_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::DacProvider, TARGET(_Dac_GetApi)()->Name);
#endif

#ifdef INCLUDE_DISPLAY
    apiProvider->Add(apiProvider, TARGET(_Display_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::DisplayProvider, TARGET(_Display_GetApi)()->Name);
#endif

#ifdef INCLUDE_GPIO
    apiProvider->Add(apiProvider, TARGET(_Gpio_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::GpioProvider, TARGET(_Gpio_GetApi)()->Name);
#endif

#ifdef INCLUDE_I2C
    apiProvider->Add(apiProvider, TARGET(_I2c_GetApi)());
#endif

#ifdef INCLUDE_PWM
    apiProvider->Add(apiProvider, TARGET(_Pwm_GetApi)());
#endif

#ifdef INCLUDE_RTC
    apiProvider->Add(apiProvider, TARGET(_Rtc_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::RtcProvider, TARGET(_Rtc_GetApi)()->Name);
#endif

#ifdef INCLUDE_SD
    apiProvider->Add(apiProvider, TARGET(_SdCard_GetApi)());
    apiProvider->SetDefaultName(apiProvider, TinyCLR_Api_Type::SdCardProvider, TARGET(_SdCard_GetApi)()->Name);
#endif

#ifdef INCLUDE_SPI
    apiProvider->Add(apiProvider, TARGET(_Spi_GetApi)());
#endif

#ifdef INCLUDE_UART
    apiProvider->Add(apiProvider, TARGET(_Uart_GetApi)());
#endif

#ifdef INCLUDE_USBCLIENT
    apiProvider->Add(apiProvider, TARGET(_UsbClient_GetApi)());
#endif

    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    TARGET(_Startup_OnSoftReset)(apiProvider, interopProvider);
    TARGET(_Startup_OnSoftResetDevice)(apiProvider, interopProvider);
}

int main() {
    TARGET(_Startup_Initialize)();

    uint8_t* heapStart;
    size_t heapLength;

    TARGET(_Startup_GetHeap)(heapStart, heapLength);
    TinyCLR_Startup_AddHeapRegion(heapStart, heapLength);

    const TinyCLR_Api_Info* debuggerApi;
    size_t debuggerIndex;

    const void* debuggerConfiguration;

    apiProvider = nullptr;

    TARGET(_Startup_GetDebuggerTransportProvider)(debuggerApi, debuggerIndex, debuggerConfiguration);
    TinyCLR_Startup_SetDebuggerTransportProvider(debuggerApi, debuggerIndex, debuggerConfiguration);


    TinyCLR_Startup_SetDeviceInformation(DEVICE_NAME, DEVICE_MANUFACTURER, DEVICE_VERSION);

    TinyCLR_Startup_SetRequiredProviders(TARGET(_Deployment_GetApi)(), TARGET(_Interrupt_GetApi)(), TARGET(_Power_GetApi)(), TARGET(_Time_GetApi)());


    auto runApp = true;

    TARGET(_Startup_GetRunApp)(runApp);
    TinyCLR_Startup_Start(&OnSoftReset, runApp);

    return 0;
}
