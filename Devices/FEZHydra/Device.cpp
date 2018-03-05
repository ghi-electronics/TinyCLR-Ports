#include "Device.h"

#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
#include "../../Drivers/SPIDisplay/SPIDisplay.h"

void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider) {
    apiProvider->Add(apiProvider, SPIDisplay_GetApi());
}

const TinyCLR_Api_Info* AT91_Deployment_GetApi() {
    return AT45DB321D_Deployment_GetApi();
}