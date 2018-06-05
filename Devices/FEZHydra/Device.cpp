#include "Device.h"

#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
#include "../../Drivers/SPIDisplay/SPIDisplay.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_Devices.h"

void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider, const TinyCLR_Interop_Provider* interopProvider) {
    apiProvider->Add(apiProvider, SPIDisplay_GetApi());
    interopProvider->Add(interopProvider, &Interop_GHIElectronics_TinyCLR_Devices);
}
