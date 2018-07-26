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

const TinyCLR_Api_Manager* apiManager = nullptr;

void OnSoftReset(const TinyCLR_Api_Manager* apiManager) {
    ::apiManager = apiManager;

#ifdef INCLUDE_ADC
    apiManager->Add(apiManager, TARGET(_Adc_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::AdcController, TARGET(_Adc_GetApi)()->Name);
#endif

#ifdef INCLUDE_CAN
    apiManager->Add(apiManager, TARGET(_Can_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::CanController, TARGET(_Can_GetApi)()->Name);
#endif

#ifdef INCLUDE_DAC
    apiManager->Add(apiManager, TARGET(_Dac_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::DacController, TARGET(_Dac_GetApi)()->Name);
#endif

#ifdef INCLUDE_DISPLAY
    apiManager->Add(apiManager, TARGET(_Display_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::DisplayController, TARGET(_Display_GetApi)()->Name);
#endif

#ifdef INCLUDE_GPIO
    apiManager->Add(apiManager, TARGET(_Gpio_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::GpioController, TARGET(_Gpio_GetApi)()->Name);
#endif

#ifdef INCLUDE_I2C
    apiManager->Add(apiManager, TARGET(_I2c_GetApi)());
#endif

#ifdef INCLUDE_PWM
    apiManager->Add(apiManager, TARGET(_Pwm_GetApi)());
#endif

#ifdef INCLUDE_RTC
    apiManager->Add(apiManager, TARGET(_Rtc_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::RtcController, TARGET(_Rtc_GetApi)()->Name);
#endif

#ifdef INCLUDE_SD
    apiManager->Add(apiManager, TARGET(_SdCard_GetApi)());
    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::SdCardController, TARGET(_SdCard_GetApi)()->Name);
#endif

#ifdef INCLUDE_SPI
    apiManager->Add(apiManager, TARGET(_Spi_GetApi)());
#endif

#ifdef INCLUDE_UART
    apiManager->Add(apiManager, TARGET(_Uart_GetApi)());
#endif

#ifdef INCLUDE_USBCLIENT
    apiManager->Add(apiManager, TARGET(_UsbDevice_GetApi)());
#endif

    auto interopManager = reinterpret_cast<const TinyCLR_Interop_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::InteropManager));

    TARGET(_Startup_OnSoftReset)(apiManager, interopManager);
    TARGET(_Startup_OnSoftResetDevice)(apiManager, interopManager);
}

int main() {
    apiManager = nullptr;

    TARGET(_Startup_Initialize)();

    uint8_t* heapStart;
    size_t heapLength;

    TARGET(_Startup_GetHeap)(heapStart, heapLength);
    TinyCLR_Startup_AddHeapRegion(heapStart, heapLength);


    const TinyCLR_Api_Info* debuggerApi;
    const void* debuggerConfiguration;

    TARGET(_Startup_GetDebuggerTransportApi)(debuggerApi, debuggerConfiguration);
    TinyCLR_Startup_SetDebuggerTransportApi(debuggerApi, debuggerConfiguration);


    TinyCLR_Startup_SetDeviceInformation(DEVICE_NAME, DEVICE_MANUFACTURER, DEVICE_VERSION);

    TinyCLR_Startup_SetRequiredApis(TARGET(_Interrupt_GetApi)(), TARGET(_Power_GetApi)(), TARGET(_Time_GetApi)());


    auto runApp = true;

    TARGET(_Startup_GetRunApp)(runApp);
    TinyCLR_Startup_Start(&OnSoftReset, runApp);


    return 0;
}
