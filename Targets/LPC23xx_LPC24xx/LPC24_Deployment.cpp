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

#include "LPC24.h"

typedef void(*IAP)(uint32_t[], uint32_t[]);

#define IAP_LOCATION 0x7ffffff1

#ifdef USE_INTERNAL_FLASH_DEPLOYMENT

//Deployment
#define DEPLOYMENT_SECTOR_START 17
#define DEPLOYMENT_SECTOR_END   27
#define DEPLOYMENT_SECTOR_NUM   (DEPLOYMENT_SECTOR_END - DEPLOYMENT_SECTOR_START + 1)

#define INTERNAL_FLASH_PROGRAM_SIZE_256 (256)
#define INTERNAL_FLASH_SECTOR_ADDRESS 0x00000000, 0x00001000, 0x00002000, 0x00003000, 0x00004000, 0x00005000, 0x00006000, 0x00007000, 0x00008000, 0x00010000, 0x00018000, 0x00020000, 0x00028000, 0x00030000, 0x00038000, 0x00040000, 0x00048000, 0x00050000, 0x00058000, 0x00060000, 0x00068000, 0x00070000, 0x00078000,  0x00079000, 0x0007A000, 0x0007B000,  0x0007C000,  0x0007D000
#define INTERNAL_FLASH_SECTOR_SIZE    0x00001000, 0x00001000, 0x00001000, 0x00001000, 0x00001000, 0x00001000, 0x00001000, 0x00001000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00008000, 0x00001000,  0x00001000, 0x00001000, 0x00001000,  0x00001000,  0x00001000

#define LPC24_AHB_CLOCK_HZ 72000000

#define TOTAL_DEPLOYMENT_CONTROLLERS 1

const uint32_t flashAddresses[] = { INTERNAL_FLASH_SECTOR_ADDRESS };
const uint32_t flashSize[] = { INTERNAL_FLASH_SECTOR_SIZE };

static TinyCLR_Storage_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];

static uint8_t data256[INTERNAL_FLASH_PROGRAM_SIZE_256];

struct DeploymentState {
    uint32_t controllerIndex;

    size_t regionCount;
    uint64_t regionAddresses[DEPLOYMENT_SECTOR_NUM];
    size_t regionSizes[DEPLOYMENT_SECTOR_NUM];

    TinyCLR_Storage_Descriptor storageDescriptor;
    TinyCLR_Startup_DeploymentConfiguration deploymentConfiguration;

    bool isOpened = false;
    bool tableInitialized = false;
};

static DeploymentState deploymentStates[TOTAL_DEPLOYMENT_CONTROLLERS];

const char* flashApiNames[TOTAL_DEPLOYMENT_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.InternalFlashStorageController\\0"
};

void LPC24_Deployment_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        if (deploymentStates[i].tableInitialized)
            continue;

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

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = flashApiNames[i];
        deploymentApi[i].Type = TinyCLR_Api_Type::StorageController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = &deploymentStates[i];

        deploymentStates[i].controllerIndex = i;
        deploymentStates[i].regionCount = DEPLOYMENT_SECTOR_NUM;

        deploymentStates[i].tableInitialized = true;

        for (auto ii = 0; ii < deploymentStates[i].regionCount; ii++) {
            deploymentStates[i].regionAddresses[ii] = flashAddresses[ii + DEPLOYMENT_SECTOR_START];
            deploymentStates[i].regionSizes[ii] = flashSize[ii + DEPLOYMENT_SECTOR_START];
        }
    }
}

void LPC24_Deployment_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration) {
    LPC24_Deployment_EnsureTableInitialized();

    auto state = &deploymentStates[0];

    api = &deploymentApi[0];
    configuration = &state->deploymentConfiguration;
}

void LPC24_Deployment_AddApi(const TinyCLR_Api_Manager* apiManager) {
    LPC24_Deployment_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &deploymentApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::StorageController, deploymentApi[0].Name);
}

int32_t __section("SectionForFlashOperations") LPC24_Deployment_PrepaireSector(int32_t startsec, int32_t endsec) {

    uint32_t command[3];
    uint32_t iap_result[2];

    command[0] = 50;
    command[1] = startsec;
    command[2] = endsec;

    IAP iap_entry = (IAP)IAP_LOCATION;

    DISABLE_INTERRUPTS_SCOPED(irq);

    iap_entry(command, iap_result);

    return iap_result[0];
}

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint16_t* ChipAddress = (uint16_t *)address;
    uint16_t* EndAddress = (uint16_t *)(address + count);
    uint16_t *pBuf = (uint16_t *)data;

    while (ChipAddress < EndAddress) {
        *pBuf++ = *ChipAddress++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t command[5];
    uint32_t iap_result[4];

    uint8_t *ptr = (uint8_t *)&data256[0];

    IAP iap_entry = (IAP)IAP_LOCATION;

    int32_t block = count / INTERNAL_FLASH_PROGRAM_SIZE_256;
    int32_t remainder = count % INTERNAL_FLASH_PROGRAM_SIZE_256;

    while (block > 0) {
        int32_t startRegion, endRegion;
        uint32_t regions = sizeof(flashAddresses) / sizeof(flashAddresses[0]);
        uint32_t startAddress = address;
        uint32_t endAddress = startAddress + (INTERNAL_FLASH_PROGRAM_SIZE_256)-1;

        for (startRegion = 0; startRegion < regions - 1; startRegion++)
            if (startAddress < flashAddresses[startRegion + 1])
                break;

        for (endRegion = regions - 1; endRegion > 0; endRegion--)
            if (endAddress > flashAddresses[endRegion])
                break;

        if (LPC24_Deployment_PrepaireSector(startRegion, endRegion))
            return TinyCLR_Result::InvalidOperation;

        memcpy(ptr, data, INTERNAL_FLASH_PROGRAM_SIZE_256);

        command[0] = 51;
        command[1] = (int)address;
        command[2] = (int)ptr;
        command[3] = INTERNAL_FLASH_PROGRAM_SIZE_256;
        command[4] = LPC24_AHB_CLOCK_HZ / 1000;

        iap_entry(command, iap_result);

        if (iap_result[0])
            return TinyCLR_Result::InvalidOperation;

        block--;
        address += INTERNAL_FLASH_PROGRAM_SIZE_256;
        data += INTERNAL_FLASH_PROGRAM_SIZE_256;

    }

    if (remainder > 0) {
        int32_t startRegion, endRegion;
        uint32_t regions = sizeof(flashAddresses) / sizeof(flashAddresses[0]);
        uint32_t startAddress = address;
        uint32_t endAddress = startAddress + (remainder)-1;

        for (startRegion = 0; startRegion < regions - 1; startRegion++)
            if (startAddress < flashAddresses[startRegion + 1])
                break;

        for (endRegion = regions - 1; endRegion > 0; endRegion--)
            if (endAddress > flashAddresses[endRegion])
                break;

        if (LPC24_Deployment_PrepaireSector(startRegion, endRegion))
            return TinyCLR_Result::InvalidOperation;

        memset(ptr, 0xFF, INTERNAL_FLASH_PROGRAM_SIZE_256);
        memcpy(ptr, data, remainder);

        command[0] = 51;
        command[1] = (int)address;
        command[2] = (int)ptr;
        command[3] = INTERNAL_FLASH_PROGRAM_SIZE_256;
        command[4] = LPC24_AHB_CLOCK_HZ / 1000;

        iap_entry(command, iap_result);

        if (iap_result[0])
            return TinyCLR_Result::InvalidOperation;

    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);
    auto sector = address;

    uint32_t startAddress = state->regionAddresses[sector];
    int32_t size = state->regionSizes[sector];
    uint32_t endAddress = startAddress + size;

    erased = true;

    while (startAddress < endAddress) {
        uint16_t data;
        size_t count = 2;

        LPC24_Deployment_Read(self, startAddress, count, reinterpret_cast<uint8_t*>(&data), -1);

        if (data != 0xFFFF) {
            erased = false;
            break;
        }

        startAddress += 2;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t command[5];
    uint32_t iap_result[4];

    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);
    auto sector = address;

    // Convert deployment sector to flash sector
    int32_t startRegion;
    uint32_t regions = sizeof(flashAddresses) / sizeof(flashAddresses[0]);
    uint32_t startAddress = state->regionAddresses[sector];

    for (startRegion = 0; startRegion < regions - 1; startRegion++)
        if (startAddress < flashAddresses[startRegion + 1])
            break;

    if (LPC24_Deployment_PrepaireSector(startRegion, startRegion))
        return TinyCLR_Result::InvalidOperation;

    //erase
    command[0] = 52;
    command[1] = startRegion;
    command[2] = startRegion;
    command[3] = LPC24_AHB_CLOCK_HZ / 1000;

    IAP iap_entry = (IAP)IAP_LOCATION;

    iap_entry(command, iap_result);

    if (iap_result[0])
        return TinyCLR_Result::InvalidOperation;


    TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_Acquire(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_Release(const TinyCLR_Storage_Controller* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
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

TinyCLR_Result LPC24_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    descriptor = &state->storageDescriptor;

    return descriptor->RegionCount > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotAvailable;
}

TinyCLR_Result LPC24_Deployment_Reset(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
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
    state->storageDescriptor.RegionsContiguous = true;
    state->storageDescriptor.RegionsEqualSized = false;

    for (auto i = 0; i < DEPLOYMENT_SECTOR_NUM; i++) {
        state->regionAddresses[i] = flashAddresses[DEPLOYMENT_SECTOR_START + i];
        state->regionSizes[i] = flashSize[DEPLOYMENT_SECTOR_START + i];
    }

    state->storageDescriptor.RegionCount = state->regionCount;
    state->storageDescriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
    state->storageDescriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

    state->deploymentConfiguration.RegionCount = state->storageDescriptor.RegionCount;
    state->deploymentConfiguration.RegionAddresses = state->storageDescriptor.RegionAddresses;
    state->deploymentConfiguration.RegionSizes = state->storageDescriptor.RegionSizes;
    state->deploymentConfiguration.RegionsContiguous = state->storageDescriptor.RegionsContiguous;
    state->deploymentConfiguration.RegionsEqualSized = state->storageDescriptor.RegionsEqualSized;

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

const TinyCLR_Startup_DeploymentConfiguration* LPC24_Deployment_GetDeploymentConfiguration() {
    auto state = &deploymentStates[0];

    return reinterpret_cast<const TinyCLR_Startup_DeploymentConfiguration*>(&state->deploymentConfiguration);
}


#endif //USE_INTERNAL_FLASH_DEPLOYMENT

uint32_t LPC24_Flash_GetPartId() {
    uint32_t command[1];
    uint32_t iap_result[2];

    IAP iap_entry = (IAP)IAP_LOCATION;

    command[0] = 54;

    DISABLE_INTERRUPTS_SCOPED(irq);

    iap_entry(command, iap_result);

    return iap_result[1];
}

