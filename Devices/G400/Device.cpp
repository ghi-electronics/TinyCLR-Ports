#include "Device.h"

#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_InteropUtil.h"

void AT91SAM9X35_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    DevicesInterop_Add(interopManager);
}
