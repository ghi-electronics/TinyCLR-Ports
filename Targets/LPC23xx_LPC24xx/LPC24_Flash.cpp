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

const uint32_t flashAddresses[] = { INTERNAL_FLASH_SECTOR_ADDRESS };
const uint32_t flashSize[] = { INTERNAL_FLASH_SECTOR_SIZE };

static uint32_t deploymentAddress[DEPLOYMENT_SECTOR_NUM];
static uint32_t deploymentSize[DEPLOYMENT_SECTOR_NUM];

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

static uint8_t data256[INTERNAL_FLASH_PROGRAM_SIZE_256];

const TinyCLR_Api_Info* LPC24_Deployment_GetApi() {
    deploymentProvider.ApiInfo = &deploymentApi;
    deploymentProvider.Acquire = &LPC24_Deployment_Acquire;
    deploymentProvider.Release = &LPC24_Deployment_Release;
    deploymentProvider.Read = &LPC24_Deployment_Read;
    deploymentProvider.Write = &LPC24_Deployment_Write;
    deploymentProvider.EraseSector = &LPC24_Deployment_EraseBlock;
    deploymentProvider.IsSectorErased = &LPC24_Deployment_IsBlockErased;
    deploymentProvider.GetSectorMap = &LPC24_Deployment_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
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

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint16_t* ChipAddress = (uint16_t *)address;
    uint16_t* EndAddress = (uint16_t *)(address + length);
    uint16_t *pBuf = (uint16_t *)buffer;

    while (ChipAddress < EndAddress) {
        *pBuf++ = *ChipAddress++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t command[5];
    uint32_t iap_result[4];

    uint8_t *ptr = (uint8_t *)&data256[0];

    IAP iap_entry = (IAP)IAP_LOCATION;

    int32_t block = length / INTERNAL_FLASH_PROGRAM_SIZE_256;
    int32_t remainder = length % INTERNAL_FLASH_PROGRAM_SIZE_256;

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

        memcpy(ptr, buffer, INTERNAL_FLASH_PROGRAM_SIZE_256);

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
        buffer += INTERNAL_FLASH_PROGRAM_SIZE_256;

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
        memcpy(ptr, buffer, remainder);

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

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t address = deploymentAddress[sector];
    int32_t size = deploymentSize[sector];
    uint32_t endAddress = address + size;

    erased = true;

    while (address < endAddress) {
        uint16_t data;

        LPC24_Deployment_Read(self, address, 2, reinterpret_cast<uint8_t*>(&data));

        if (data != 0xFFFF) {
            erased = false;
            break;
        }

        address += 2;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC24_Deployment_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t command[5];
    uint32_t iap_result[4];

    // Convert deployment sector to flash sector
    int32_t startRegion;
    uint32_t regions = sizeof(flashAddresses) / sizeof(flashAddresses[0]);
    uint32_t startAddress = deploymentAddress[sector];

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

TinyCLR_Result LPC24_Deployment_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportXIP) {
    supportXIP = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_Release(const TinyCLR_Deployment_Provider* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
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

TinyCLR_Result LPC24_Deployment_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < DEPLOYMENT_SECTOR_NUM; i++) {
        deploymentAddress[i] = flashAddresses[DEPLOYMENT_SECTOR_START + i];
        deploymentSize[i] = flashSize[DEPLOYMENT_SECTOR_START + i];
    }

    addresses = deploymentAddress;
    sizes = deploymentSize;
    count = DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Deployment_Reset(const TinyCLR_Deployment_Provider* self) {
    return TinyCLR_Result::Success;
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

