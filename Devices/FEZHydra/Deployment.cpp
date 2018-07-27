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

const TinyCLR_Api_Info* AT91_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &AT91_Deployment_Acquire;
        deploymentControllers[i].Release = &AT91_Deployment_Release;
        deploymentControllers[i].Open = &AT91_Deployment_Open;
        deploymentControllers[i].Close = &AT91_Deployment_Close;
        deploymentControllers[i].Read = &AT91_Deployment_Read;
        deploymentControllers[i].Write = &AT91_Deployment_Write;
        deploymentControllers[i].Erase = &AT91_Deployment_Erase;
        deploymentControllers[i].IsErased = &AT91_Deployment_IsErased;
        deploymentControllers[i].GetDescriptor = &AT91_Deployment_GetDescriptor;
        deploymentControllers[i].IsPresent = &AT91_Deployment_IsPresent;
        deploymentControllers[i].SetPresenceChangedHandler = &AT91_Deployment_SetPresenceChangedHandler;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.AT91.StorageController";
        deploymentApi[i].Type = TinyCLR_Api_Type::StorageController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = &deploymentState[i];

        deploymentState[i].controllerIndex = i;
        deploymentState[i].regionCount = AT91_DEPLOYMENT_SECTOR_NUM;
    }

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result AT91_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    const TinyCLR_Api_Info* spiApi = &CONCAT(DEVICE_TARGET, _Spi_GetApi)()[AT91_DEPLOYMENT_SPI_PORT];
    TinyCLR_Spi_Controller* spiController = (TinyCLR_Spi_Controller*)spiApi->Implementation;

    const TinyCLR_Api_Info* timeApi = &CONCAT(DEVICE_TARGET, _Time_GetApi)()[0];
    TinyCLR_NativeTime_Controller* timerController = (TinyCLR_NativeTime_Controller*)timeApi->Implementation;

    return AT45DB321D_Flash_Acquire(spiController, timerController, AT91_DEPLOYMENT_SPI_ENABLE_PIN);
}

TinyCLR_Result AT91_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    return AT45DB321D_Flash_Release();
}

TinyCLR_Result AT91_Deployment_Open(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (state->isOpened)
        return TinyCLR_Result::SharingViolation;

    state->storageDescriptor.CanReadDirect = true;
    state->storageDescriptor.CanWriteDirect = true;
    state->storageDescriptor.CanExecuteDirect = false;
    state->storageDescriptor.EraseBeforeWrite = true;
    state->storageDescriptor.Removable = false;
    state->storageDescriptor.RegionsRepeat = true;

    size_t regionCount;

    AT45DB321D_Flash_GetSectorMap(state->regionAddresses, state->regionSizes, regionCount);

    if (regionCount < state->regionCount)
        return TinyCLR_Result::ArgumentOutOfRange;

    state->regionAddresses += AT91_DEPLOYMENT_SECTOR_START;
    state->regionSizes += AT91_DEPLOYMENT_SECTOR_START;

    state->storageDescriptor.RegionCount = state->regionCount;
    state->storageDescriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
    state->storageDescriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

    state->deploymentConfiguration.RegionCount = state->storageDescriptor.RegionCount;
    state->deploymentConfiguration.RegionAddresses = state->storageDescriptor.RegionAddresses;
    state->deploymentConfiguration.RegionSizes = state->storageDescriptor.RegionSizes;

    state->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Deployment_Close(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (!state->isOpened)
        return TinyCLR_Result::NotFound;

    state->isOpened = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    return AT45DB321D_Flash_Read(address, count, data);
}

TinyCLR_Result AT91_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    return AT45DB321D_Flash_Write(address, count, data);;
}

TinyCLR_Result AT91_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    auto sector = address;

    sector += AT91_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_EraseBlock(sector);
}

TinyCLR_Result AT91_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, bool& erased) {
    auto sector = address;

    sector += AT91_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result AT91_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
    return AT45DB321D_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result AT91_Deployment_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Deployment_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    present = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    descriptor = &state->storageDescriptor;

    return descriptor->RegionCount > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotImplemented;
}

const TinyCLR_Startup_DeploymentConfiguration* AT91_Deployment_GetDeploymentConfiguration() {
    auto state = &deploymentState[0];

    return reinterpret_cast<const TinyCLR_Startup_DeploymentConfiguration*>(&state->deploymentConfiguration);
}

