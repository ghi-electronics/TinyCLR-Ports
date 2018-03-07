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

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

const TinyCLR_Api_Info* AT91_Deployment_GetApi() {
    deploymentProvider.Parent = &deploymentApi;
    deploymentProvider.Index = 0;
    deploymentProvider.Acquire = &AT91_Deployment_Acquire;
    deploymentProvider.Release = &AT91_Deployment_Release;
    deploymentProvider.Read = &AT91_Deployment_Read;
    deploymentProvider.Write = &AT91_Deployment_Write;
    deploymentProvider.EraseSector = &AT91_Deployment_EraseBlock;
    deploymentProvider.IsSectorErased = &AT91_Deployment_IsBlockErased;
    deploymentProvider.GetSectorMap = &AT91_Deployment_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT45DB321D.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Count = 1;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
}

TinyCLR_Result AT91_Deployment_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportXIP) {
    const TinyCLR_Api_Info* spiApi = CONCAT(DEVICE_TARGET, _Spi_GetApi)();
    TinyCLR_Spi_Provider** spiProvider = (TinyCLR_Spi_Provider**)spiApi->Implementation;

    const TinyCLR_Api_Info* timeApi = CONCAT(DEVICE_TARGET, _Time_GetApi)();;
    TinyCLR_Time_Provider** timeProvider = (TinyCLR_Time_Provider**)timeApi->Implementation;

    return AT45DB321D_Flash_Acquire(reinterpret_cast<const TinyCLR_Spi_Provider*>(&spiProvider[0]), reinterpret_cast<const TinyCLR_Time_Provider*>(&timeProvider[0]), AT91_DEPLOYMENT_SPI_ENABLE_PIN, supportXIP);
}

TinyCLR_Result AT91_Deployment_Release(const TinyCLR_Deployment_Provider* self) {
    return AT45DB321D_Flash_Release();
}

TinyCLR_Result AT91_Deployment_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    return AT45DB321D_Flash_Read(address, length, buffer);
}

TinyCLR_Result AT91_Deployment_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    return AT45DB321D_Flash_Write(address, length, buffer);;
}

TinyCLR_Result AT91_Deployment_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    sector += AT91_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_EraseBlock(sector);
}

TinyCLR_Result AT91_Deployment_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool& erased) {
    sector += AT91_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result AT91_Deployment_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
    return AT45DB321D_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result AT91_Deployment_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    AT45DB321D_Flash_GetSectorMap(addresses, sizes, count);

    addresses += AT91_DEPLOYMENT_SECTOR_START;
    sizes += AT91_DEPLOYMENT_SECTOR_START;
    count = AT91_DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

