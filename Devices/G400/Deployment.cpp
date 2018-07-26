// Copyright Microsoft Corporation
// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <AT91.h>
#include "../../Drivers/AT45DB321D_Flash/AT45DB321D_Flash.h"

#define TOTAL_DEPLOYMENT_CONTROLLERS 1

static TinyCLR_Storage_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];

const TinyCLR_Api_Info* AT91_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &AT91_Deployment_Acquire;
        deploymentControllers[i].Release = &AT91_Deployment_Release;
        deploymentControllers[i].Read = &AT91_Deployment_Read;
        deploymentControllers[i].Write = &AT91_Deployment_Write;
        deploymentControllers[i].EraseSector = &AT91_Deployment_EraseSector;
        deploymentControllers[i].IsSectorErased = &AT91_Deployment_IsSectorErased;
        deploymentControllers[i].GetSectorMap = &AT91_Deployment_GetSectorMap;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.AT91.DeploymentController";
        deploymentApi[i].Type = TinyCLR_Api_Type::DeploymentController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = nullptr;
    }

    return (const TinyCLR_Api_Info*)&deploymentApi;
}


TinyCLR_Result AT91_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    const TinyCLR_Api_Info* spiApi = &CONCAT(DEVICE_TARGET, _Spi_GetApi)()[AT91_DEPLOYMENT_SPI_PORT];
    TinyCLR_Spi_Controller* spiController = (TinyCLR_Spi_Controller*)spiApi->Implementation;

    const TinyCLR_Api_Info* timeApi = &CONCAT(DEVICE_TARGET, _Time_GetApi)()[0];
    TinyCLR_NativeTime_Controller* timerController = (TinyCLR_NativeTime_Controller*)timeApi->Implementation;

    return AT45DB321D_Flash_Acquire(spiController, timerController, AT91_DEPLOYMENT_SPI_ENABLE_PIN, supportXIP);
}

TinyCLR_Result AT91_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    return AT45DB321D_Flash_Release();
}

TinyCLR_Result AT91_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t length, uint8_t* buffer) {
    return AT45DB321D_Flash_Read(address, length, buffer);
}

TinyCLR_Result AT91_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t length, const uint8_t* buffer) {
    return AT45DB321D_Flash_Write(address, length, buffer);;
}

TinyCLR_Result AT91_Deployment_EraseSector(const TinyCLR_Storage_Controller* self, uint64_t sector) {
    sector += AT91_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_EraseBlock(sector);
}

TinyCLR_Result AT91_Deployment_IsSectorErased(const TinyCLR_Storage_Controller* self, uint64_t sector, bool& erased) {
    sector += AT91_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result AT91_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
    return AT45DB321D_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result AT91_Deployment_GetSectorMap(const TinyCLR_Storage_Controller* self, const uint64_t*& addresses, const size_t*& sizes, size_t& count) {
    AT45DB321D_Flash_GetSectorMap(addresses, sizes, count);

    addresses += AT91_DEPLOYMENT_SECTOR_START;
    sizes += AT91_DEPLOYMENT_SECTOR_START;
    count = AT91_DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

