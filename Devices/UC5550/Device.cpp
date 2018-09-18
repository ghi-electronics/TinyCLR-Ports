#include "Device.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_InteropUtil.h"

void STM32F7_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    DevicesInterop_Add(interopManager);
}
