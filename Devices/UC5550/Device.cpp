#include "Device.h"
#include "../../Drivers/DevicesInterop/GHIElectronics_TinyCLR_InteropUtil.h"

extern "C" {
    extern int HeapBegin;
    extern uint32_t Image$$ER_RAM_RW$$Base;
    extern uint32_t __Vectors;
}

void STM32F7_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopManager) {
    DevicesInterop_Add(interopManager);
}

void STM32F7_Startup_MpuConfiguration() {
    STM32F7_Mpu_Configuration(reinterpret_cast<uint32_t>(&__Vectors), STM32F7_Mpu_RegionSize::Size_64KBytes, STM32F7_Mpu_RegionNumber::Region0, true);
    STM32F7_Mpu_Configuration(reinterpret_cast<uint32_t>(&Image$$ER_RAM_RW$$Base), STM32F7_Mpu_RegionSize::Size_256KBytes, STM32F7_Mpu_RegionNumber::Region1, true);
    STM32F7_Mpu_Configuration(reinterpret_cast<uint32_t>(reinterpret_cast<uint32_t*>(&HeapBegin)), STM32F7_Mpu_RegionSize::Size_32MBytes, STM32F7_Mpu_RegionNumber::Region2, true);
}
