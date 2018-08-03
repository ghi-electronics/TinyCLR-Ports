#include "Device.h"

#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_Devices.h"

void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    SPIDisplay_AddApi(apiManager);
    interopManager->Add(interopManager, &Interop_GHIElectronics_TinyCLR_Devices);
}
