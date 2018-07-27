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

struct DeploymentState {
    uint32_t controllerIndex;

    size_t regionCount;
    const uint64_t* regionAddresses;
    const size_t* regionSizes;

    TinyCLR_Storage_Descriptor storageDescriptor;
    TinyCLR_Startup_DeploymentConfiguration deploymentConfiguration;

    bool isOpened = false;
};

static DeploymentState deploymentState[TOTAL_DEPLOYMENT_CONTROLLERS];

const TinyCLR_Api_Info* LPC24_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &LPC24_Deployment_Acquire;
        deploymentControllers[i].Release = &LPC24_Deployment_Release;
        deploymentControllers[i].Open = &LPC24_Deployment_Open;
        deploymentControllers[i].Close = &LPC24_Deployment_Close;
        deploymentControllers[i].Read = &LPC24_Deployment_Read;
        deploymentControllers[i].Write = &LPC24_Deployment_Write;
        deploymentControllers[i].Erase = &LPC24_Deployment_Erase;
        deploymentControllers[i].IsErased = &LPC24_Deployment_IsErased;
        deploymentControllers[i].GetDescriptor = &LPC24_Deployment_GetDescriptor;
        deploymentControllers[i].IsPresent = &LPC24_Deployment_IsPresent;
        deploymentControllers[i].SetPresenceChangedHandler = &LPC24_Deployment_SetPresenceChangedHandler;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.StorageController";
        deploymentApi[i].Type = TinyCLR_Api_Type::StorageController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = &deploymentState[i];

        deploymentState[i].controllerIndex = i;
        deploymentState[i].regionCount = LPC24_DEPLOYMENT_SECTOR_NUM;
    }

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result LPC24_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    return AT49BV322DT_Flash_Acquire();
}

TinyCLR_Result LPC24_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    return AT49BV322DT_Flash_Release();
}

TinyCLR_Result LPC24_Deployment_Open(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (state->isOpened)
        return TinyCLR_Result::SharingViolation;

    state->storageDescriptor.CanReadDirect = true;
    state->storageDescriptor.CanWriteDirect = true;
    state->storageDescriptor.CanExecuteDirect = true;
    state->storageDescriptor.EraseBeforeWrite = true;
    state->storageDescriptor.Removable = false;
    state->storageDescriptor.RegionsRepeat = true;

    size_t regionCount;

    AT49BV322DT_Flash_GetSectorMap(state->regionAddresses, state->regionSizes, regionCount);

    if (regionCount < state->regionCount)
        return TinyCLR_Result::ArgumentOutOfRange;

    state->regionAddresses += LPC24_DEPLOYMENT_SECTOR_START;
    state->regionSizes += LPC24_DEPLOYMENT_SECTOR_START;

    state->storageDescriptor.RegionCount = state->regionCount;
    state->storageDescriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
    state->storageDescriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

    state->deploymentConfiguration.RegionCount = state->storageDescriptor.RegionCount;
    state->deploymentConfiguration.RegionAddresses = state->storageDescriptor.RegionAddresses;
    state->deploymentConfiguration.RegionSizes = state->storageDescriptor.RegionSizes;

    state->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_Close(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (!state->isOpened)
        return TinyCLR_Result::NotFound;

    state->isOpened = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    return AT49BV322DT_Flash_Read(address, count, data);
}

TinyCLR_Result LPC24_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    return AT49BV322DT_Flash_Write(address, count, data);;
}

TinyCLR_Result LPC24_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    auto sector = address;

    sector += LPC24_DEPLOYMENT_SECTOR_START;

    return AT49BV322DT_Flash_EraseBlock(sector);
}

TinyCLR_Result LPC24_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, bool& erased) {
    auto sector = address;

    sector += LPC24_DEPLOYMENT_SECTOR_START;

    return AT49BV322DT_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result LPC24_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
    return AT49BV322DT_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result LPC24_Deployment_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    present = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    descriptor = &state->storageDescriptor;

    return descriptor->RegionCount > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotImplemented;
}

const TinyCLR_Startup_DeploymentConfiguration* LPC24_Deployment_GetDeploymentConfiguration() {
    auto state = &deploymentState[0];

    return reinterpret_cast<const TinyCLR_Startup_DeploymentConfiguration*>(&state->deploymentConfiguration);
}

