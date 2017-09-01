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

#include "AT91.h"

#define DEPLOYMENT_SECTOR_START 17
#define DEPLOYMENT_SECTOR_END   27
#define DEPLOYMENT_SECTOR_NUM   (DEPLOYMENT_SECTOR_END - DEPLOYMENT_SECTOR_START + 1)

const uint32_t flashAddresses[] = { 0 };
const uint32_t flashSize[] = { 0 };

static uint32_t deploymentAddress[DEPLOYMENT_SECTOR_NUM];
static uint32_t deploymentSize[DEPLOYMENT_SECTOR_NUM];

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

const TinyCLR_Api_Info* AT91_Deployment_GetApi() {
    deploymentProvider.Parent = &deploymentApi;
    deploymentProvider.Index = 0;
    deploymentProvider.Acquire = &AT91_Flash_Acquire;
    deploymentProvider.Release = &AT91_Flash_Release;
    deploymentProvider.Read = &AT91_Flash_Read;
    deploymentProvider.Write = &AT91_Flash_Write;
    deploymentProvider.EraseSector = &AT91_Flash_EraseBlock;
    deploymentProvider.IsSectorErased = &AT91_Flash_IsBlockErased;
    deploymentProvider.GetSectorMap = &AT91_Flash_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Count = 1;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") AT91_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    GLOBAL_LOCK(irq);

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") AT91_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    GLOBAL_LOCK(irq);

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") AT91_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    GLOBAL_LOCK(irq);

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") AT91_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    GLOBAL_LOCK(irq);

    TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportXIP) {
    supportXIP = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Flash_Release(const TinyCLR_Deployment_Provider* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
    size = 0;

    int32_t startRegion;
    uint32_t regions = sizeof(flashAddresses) / sizeof(flashAddresses[0]);
    uint32_t startAddress = address;

    for (startRegion = 0; startRegion < regions - 1; startRegion++)
        if (startAddress < flashAddresses[startRegion + 1])
            break;

    size = flashSize[startRegion];

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < DEPLOYMENT_SECTOR_NUM; i++) {
        deploymentAddress[i] = flashAddresses[DEPLOYMENT_SECTOR_START + i];
        deploymentSize[i] = flashSize[DEPLOYMENT_SECTOR_START + i];
    }

    addresses = deploymentAddress;
    sizes = deploymentSize;
    count = DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Flash_Reset(const TinyCLR_Deployment_Provider* self) {
    return TinyCLR_Result::Success;
}
