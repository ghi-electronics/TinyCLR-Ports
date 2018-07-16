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

#include <LPC17.h>
#include "../../Drivers/S25FL032_Flash/S25FL032_Flash.h"

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

const TinyCLR_Api_Info* LPC17_Deployment_GetApi() {
    deploymentProvider.ApiInfo = &deploymentApi;
    deploymentProvider.Initialize = &LPC17_Deployment_Initialize;
    deploymentProvider.Uninitialize = &LPC17_Deployment_Uninitialize;
    deploymentProvider.Read = &LPC17_Deployment_Read;
    deploymentProvider.Write = &LPC17_Deployment_Write;
    deploymentProvider.EraseSector = &LPC17_Deployment_EraseBlock;
    deploymentProvider.IsSectorErased = &LPC17_Deployment_IsBlockErased;
    deploymentProvider.GetSectorMap = &LPC17_Deployment_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.S25FL032.DeploymentController";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentController;
    deploymentApi.Version = 0;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
}

TinyCLR_Result LPC17_Deployment_Initialize(const TinyCLR_Deployment_Provider* self, bool& supportXIP) {
    const TinyCLR_Api_Info* spiApi = CONCAT(DEVICE_TARGET, _Spi_GetApi)();
    TinyCLR_Spi_Provider* spiProvider = (TinyCLR_Spi_Provider*)spiApi->Implementation;

    return S25FL032_Flash_Acquire(spiProvider, LPC17_DEPLOYMENT_SPI_PORT, LPC17_DEPLOYMENT_SPI_ENABLE_PIN, supportXIP);
}

TinyCLR_Result LPC17_Deployment_Uninitialize(const TinyCLR_Deployment_Provider* self) {
    return S25FL032_Flash_Release();
}

TinyCLR_Result LPC17_Deployment_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    return S25FL032_Flash_Read(address, length, buffer);
}

TinyCLR_Result LPC17_Deployment_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    return S25FL032_Flash_Write(address, length, buffer);;
}

TinyCLR_Result LPC17_Deployment_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    sector += LPC17_DEPLOYMENT_SECTOR_START;

    return S25FL032_Flash_EraseBlock(sector);
}

TinyCLR_Result LPC17_Deployment_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool& erased) {
    sector += LPC17_DEPLOYMENT_SECTOR_START;

    return S25FL032_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result LPC17_Deployment_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
    return S25FL032_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result LPC17_Deployment_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    S25FL032_Flash_GetSectorMap(addresses, sizes, count);

    addresses += LPC17_DEPLOYMENT_SECTOR_START;
    sizes += LPC17_DEPLOYMENT_SECTOR_START;
    count = LPC17_DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

