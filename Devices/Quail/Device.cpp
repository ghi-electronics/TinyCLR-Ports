#include "Device.h"
#include "../../Drivers/SPIDisplay/SPIDisplay.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_Devices.h"

void STM32F4_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    apiManager->Add(apiManager, SPIDisplay_GetApi());
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices);
}