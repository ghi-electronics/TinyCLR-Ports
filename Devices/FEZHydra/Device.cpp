#include "Device.h"

#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
#include "../../Drivers/SPIDisplay/SPIDisplay.h"

void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider) {
    apiProvider->Add(apiProvider, SPIDisplay_GetApi());

#ifdef INCLUDE_USBCLIENT
    apiProvider->Add(apiProvider, UsbClient_GetApi());
#endif
}
