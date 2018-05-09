#include "Device.h"


#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
#include "../../Drivers/SPIDisplay/SPIDisplay.h"

void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider) {
    apiProvider->Add(apiProvider, SPIDisplay_GetApi());
    
    extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices;

    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->Add(interopProvider, &Interop_GHIElectronics_TinyCLR_Devices);
}
