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

#include <LPC24.h>
#include "../../Drivers/AT49BV322DT_Flash/AT49BV322DT_Flash.h"

#define TOTAL_DEPLOYMENT_CONTROLLERS 1

static TinyCLR_Storage_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];

const TinyCLR_Api_Info* LPC24_Deployment_GetApi() {
    for (auto i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &LPC24_Deployment_Acquire;
        deploymentControllers[i].Release = &LPC24_Deployment_Release;
        deploymentControllers[i].Read = &LPC24_Deployment_Read;
        deploymentControllers[i].Write = &LPC24_Deployment_Write;
        deploymentControllers[i].EraseSector = &LPC24_Deployment_EraseBlock;
        deploymentControllers[i].IsSectorErased = &LPC24_Deployment_IsBlockErased;
        deploymentControllers[i].GetSectorMap = &LPC24_Deployment_GetSectorMap;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.DeploymentController";
        deploymentApi[i].Type = TinyCLR_Api_Type::DeploymentController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = nullptr;
    }

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result LPC24_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    return AT49BV322DT_Flash_Acquire(supportXIP);
}

TinyCLR_Result LPC24_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    return AT49BV322DT_Flash_Release();
}

TinyCLR_Result LPC24_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t length, uint8_t* buffer) {
    return AT49BV322DT_Flash_Read(address, length, buffer);
}

TinyCLR_Result LPC24_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t length, const uint8_t* buffer) {
    return AT49BV322DT_Flash_Write(address, length, buffer);;
}

TinyCLR_Result LPC24_Deployment_EraseBlock(const TinyCLR_Storage_Controller* self, uint64_t sector) {
    sector += LPC24_DEPLOYMENT_SECTOR_START;

    return AT49BV322DT_Flash_EraseBlock(sector);
}

TinyCLR_Result LPC24_Deployment_IsBlockErased(const TinyCLR_Storage_Controller* self, uint64_t sector, bool& erased) {
    sector += LPC24_DEPLOYMENT_SECTOR_START;

    return AT49BV322DT_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result LPC24_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
    return AT49BV322DT_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result LPC24_Deployment_GetSectorMap(const TinyCLR_Storage_Controller* self, const uint64_t*& addresses, const size_t*& sizes, size_t& count) {
    AT49BV322DT_Flash_GetSectorMap(addresses, sizes, count);

    addresses += LPC24_DEPLOYMENT_SECTOR_START;
    sizes += LPC24_DEPLOYMENT_SECTOR_START;
    count = LPC24_DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

