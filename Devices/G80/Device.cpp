#include "Device.h"
#include "../../Drivers/SPIDisplay/SPIDisplay.h"
#include "../../Drivers/UsbClient/UsbClient.h"

void STM32F4_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider) {
    apiProvider->Add(apiProvider, SPIDisplay_GetApi());
    
#ifdef INCLUDE_USBCLIENT
    apiProvider->Add(apiProvider, UsbClient_GetApi());
#endif
}
