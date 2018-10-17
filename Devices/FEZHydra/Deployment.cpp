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

#include <AT91SAM9Rx64.h>
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
    bool tableInitialized = false;
};

static DeploymentState deploymentStates[TOTAL_DEPLOYMENT_CONTROLLERS];

const char* deploymentApiNames[TOTAL_DEPLOYMENT_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.StorageController\\0"
};

void AT91SAM9Rx64_Deployment_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        if (deploymentStates[i].tableInitialized)
            continue;

        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &AT91SAM9Rx64_Deployment_Acquire;
        deploymentControllers[i].Release = &AT91SAM9Rx64_Deployment_Release;
        deploymentControllers[i].Open = &AT91SAM9Rx64_Deployment_Open;
        deploymentControllers[i].Close = &AT91SAM9Rx64_Deployment_Close;
        deploymentControllers[i].Read = &AT91SAM9Rx64_Deployment_Read;
        deploymentControllers[i].Write = &AT91SAM9Rx64_Deployment_Write;
        deploymentControllers[i].Erase = &AT91SAM9Rx64_Deployment_Erase;
        deploymentControllers[i].IsErased = &AT91SAM9Rx64_Deployment_IsErased;
        deploymentControllers[i].GetDescriptor = &AT91SAM9Rx64_Deployment_GetDescriptor;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = deploymentApiNames[i];
        deploymentApi[i].Type = TinyCLR_Api_Type::StorageController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = &deploymentStates[i];

        deploymentStates[i].controllerIndex = i;
        deploymentStates[i].regionCount = AT91SAM9Rx64_DEPLOYMENT_SECTOR_NUM;

        deploymentStates[i].tableInitialized = true;
    }
}

void AT91SAM9Rx64_Deployment_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration) {
    AT91SAM9Rx64_Deployment_EnsureTableInitialized();

    auto state = &deploymentStates[0];

    api = &deploymentApi[0];
    configuration = &state->deploymentConfiguration;
}

void AT91SAM9Rx64_Deployment_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91SAM9Rx64_Deployment_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &deploymentApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::StorageController, deploymentApi[0].Name);
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    auto spiApi = CONCAT(DEVICE_TARGET, _Spi_GetRequiredApi)();

    spiApi += AT91SAM9Rx64_DEPLOYMENT_SPI_PORT;

    TinyCLR_Spi_Controller* spiController = (TinyCLR_Spi_Controller*)spiApi->Implementation;

    auto timeApi = CONCAT(DEVICE_TARGET, _Time_GetRequiredApi)();

    TinyCLR_NativeTime_Controller* timerController = (TinyCLR_NativeTime_Controller*)timeApi->Implementation;

    return AT45DB321D_Flash_Acquire(spiController, timerController, AT91SAM9Rx64_DEPLOYMENT_SPI_ENABLE_PIN);
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    return AT45DB321D_Flash_Release();
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Open(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (state->isOpened)
        return TinyCLR_Result::SharingViolation;

    state->storageDescriptor.CanReadDirect = true;
    state->storageDescriptor.CanWriteDirect = true;
    state->storageDescriptor.CanExecuteDirect = false;
    state->storageDescriptor.EraseBeforeWrite = true;
    state->storageDescriptor.Removable = false;
    state->storageDescriptor.RegionsContiguous = false;
    state->storageDescriptor.RegionsEqualSized = false;

    size_t regionCount;

    AT45DB321D_Flash_GetSectorMap(state->regionAddresses, state->regionSizes, regionCount);

    if (regionCount < state->regionCount)
        return TinyCLR_Result::ArgumentOutOfRange;

    state->regionAddresses += AT91SAM9Rx64_DEPLOYMENT_SECTOR_START;
    state->regionSizes += AT91SAM9Rx64_DEPLOYMENT_SECTOR_START;

    state->storageDescriptor.RegionCount = state->regionCount;
    state->storageDescriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
    state->storageDescriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

    state->deploymentConfiguration.RegionCount = state->storageDescriptor.RegionCount;
    state->deploymentConfiguration.RegionAddresses = state->storageDescriptor.RegionAddresses;
    state->deploymentConfiguration.RegionSizes = state->storageDescriptor.RegionSizes;
    state->deploymentConfiguration.RegionsContiguous = false;
    state->deploymentConfiguration.RegionsEqualSized = false;

    state->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Close(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (!state->isOpened)
        return TinyCLR_Result::NotFound;

    state->isOpened = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    return AT45DB321D_Flash_Read(address, count, data);
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    return AT45DB321D_Flash_Write(address, count, data);;
}

TinyCLR_Result AT91SAM9Rx64_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    auto sector = address;

    sector += AT91SAM9Rx64_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_EraseBlock(sector);
}

TinyCLR_Result AT91SAM9Rx64_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased) {
    auto sector = address;

    sector += AT91SAM9Rx64_DEPLOYMENT_SECTOR_START;

    return AT45DB321D_Flash_IsBlockErased(sector, erased);
}

TinyCLR_Result AT91SAM9Rx64_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
    return AT45DB321D_Flash_GetBytesPerSector(address, size);
}

TinyCLR_Result AT91SAM9Rx64_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    descriptor = &state->storageDescriptor;

    return descriptor->RegionCount > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotAvailable;
}

const TinyCLR_Startup_DeploymentConfiguration* AT91SAM9Rx64_Deployment_GetDeploymentConfiguration() {
    auto state = &deploymentStates[0];

    return reinterpret_cast<const TinyCLR_Startup_DeploymentConfiguration*>(&state->deploymentConfiguration);
}


