#include "Device.h"


#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"
void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider) {

}

const TinyCLR_Api_Info* AT91_Deployment_GetApi() {
    return AT45DB321D_Deployment_GetApi();
}