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
    TARGET(_Adc_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::AdcController, TARGET(_Adc_GetApi)()->Name);
#endif

#ifdef INCLUDE_CAN
    TARGET(_Can_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::CanController, TARGET(_Can_GetApi)()->Name);
#endif

#ifdef INCLUDE_DAC
    TARGET(_Dac_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::DacController, TARGET(_Dac_GetApi)()->Name);
#endif

#ifdef INCLUDE_DISPLAY
    TARGET(_Display_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::DisplayController, TARGET(_Display_GetApi)()->Name);
#endif

#ifdef INCLUDE_GPIO
    TARGET(_Gpio_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::GpioController, TARGET(_Gpio_GetApi)()->Name);
#endif

#ifdef INCLUDE_I2C
    TARGET(_I2c_AddApi)(apiManager);
#endif

#ifdef INCLUDE_PWM
    TARGET(_Pwm_AddApi)(apiManager);
#endif

#ifdef INCLUDE_RTC
    TARGET(_Rtc_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::RtcController, TARGET(_Rtc_GetApi)()->Name);
#endif

#ifdef INCLUDE_SD
    TARGET(_SdCard_AddApi)(apiManager);
    //apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::StorageController, TARGET(_SdCard_GetApi)()->Name);
#endif

#ifdef INCLUDE_SPI
    TARGET(_Spi_AddApi)(apiManager);
#endif

#ifdef INCLUDE_UART
    TARGET(_Uart_AddApi)(apiManager);
#endif

#ifdef INCLUDE_USBCLIENT
    TARGET(_UsbDevice_AddApi)(apiManager);
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


    const TinyCLR_Api_Info *debuggerApi, *deploymentApi;
    const void* debuggerConfiguration;
    const TinyCLR_Startup_DeploymentConfiguration* deploymentConfiguration;

    TARGET(_Startup_GetDebuggerTransportApi)(debuggerApi, debuggerConfiguration);
    TinyCLR_Startup_SetDebuggerTransportApi(debuggerApi, debuggerConfiguration);

    TARGET(_Startup_GetDeploymentApi)(deploymentApi, deploymentConfiguration);
    TinyCLR_Startup_SetDeploymentApi(deploymentApi, deploymentConfiguration);

    TinyCLR_Startup_SetDeviceInformation(DEVICE_NAME, DEVICE_MANUFACTURER, DEVICE_VERSION);
    TinyCLR_Startup_SetRequiredApis(TARGET(_Interrupt_GetRequiredApi)(), TARGET(_Power_GetRequiredApi)(), TARGET(_Time_GetRequiredApi()));

    auto runApp = true;

    TARGET(_Startup_GetRunApp)(runApp);
    TinyCLR_Startup_Start(&OnSoftReset, runApp);


    return 0;
}
