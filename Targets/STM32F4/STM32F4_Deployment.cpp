// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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

#include "STM32F4.h"
#include <stdio.h>

#ifndef STM32F4_FLASH
#define STM32F4_FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#endif

typedef uint16_t CHIP_WORD;

#define FLASH_CR_PSIZE_BITS FLASH_CR_PSIZE_0 // 16 bit programming

#if STM32F4_SUPPLY_VOLTAGE_MV < 2100
#error 16 bit Flash programming not allowed for voltages below 2.1V
#endif
#if STM32F4_AHB_CLOCK_HZ < 1000000
#error Flash programming not allowed for HCLK below 1MHz
#endif

TinyCLR_Result STM32F4_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);

static const uint32_t STM32F4_FLASH_KEY1 = 0x45670123;
static const uint32_t STM32F4_FLASH_KEY2 = 0xcdef89ab;

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

const TinyCLR_Api_Info* STM32F4_Deployment_GetApi() {
    deploymentProvider.Parent = &deploymentApi;
    deploymentProvider.Index = 0;
    deploymentProvider.Acquire = &STM32F4_Flash_Acquire;
    deploymentProvider.Release = &STM32F4_Flash_Release;
    deploymentProvider.Read = &STM32F4_Flash_Read;
    deploymentProvider.Write = &STM32F4_Flash_Write;
    deploymentProvider.EraseSector = &STM32F4_Flash_EraseSector;
    deploymentProvider.IsSectorErased = &STM32F4_Flash_IsSectorErased;
    deploymentProvider.GetSectorMap = &STM32F4_Flash_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Count = 1;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    int32_t bytePerSector = 0;

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F4_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    CHIP_WORD* ChipAddress = (CHIP_WORD *)address;
    CHIP_WORD* EndAddress = (CHIP_WORD *)(address + length);
    CHIP_WORD *pBuf = (CHIP_WORD *)buffer;

    while (ChipAddress < EndAddress) {
        *pBuf++ = *ChipAddress++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    int32_t bytePerSector = 0;

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F4_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    if (STM32F4_FLASH->CR & FLASH_CR_LOCK) { // unlock
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY1;
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY2;
    }

    CHIP_WORD* ChipAddress = (CHIP_WORD *)address;
    CHIP_WORD* EndAddress = (CHIP_WORD *)(address + length);
    CHIP_WORD *pBuf = (CHIP_WORD *)buffer;

    // enable programming
    STM32F4_FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_BITS;

    while (ChipAddress < EndAddress) {
        if (*ChipAddress != *pBuf) {
            // write data
            *ChipAddress = *pBuf;
            // wait for completion
            while (STM32F4_FLASH->SR & FLASH_SR_BSY);
            // check
            if (*ChipAddress != *pBuf) {
                return TinyCLR_Result::InvalidOperation;
            }
        }
        ChipAddress++;
        pBuf++;
    }

    // reset & lock the controller
    STM32F4_FLASH->CR = FLASH_CR_LOCK;

    return TinyCLR_Result::Success;
}

const uint32_t deploymentSectorAddress[] = DEPLOYMENT_SECTOR_ADDRESS;
const uint32_t deploymentSectorSize[] = DEPLOYMENT_SECTOR_SIZES;
const uint32_t flashSectorAddress[] = FLASH_SECTOR_ADDRESS;

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_IsSectorErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    if (sector >= SIZEOF_ARRAY(deploymentSectorAddress)) return TinyCLR_Result::IndexOutOfRange;

    uint32_t address = deploymentSectorAddress[sector];
    size_t length = deploymentSectorSize[sector];

    CHIP_WORD* ChipAddress = (CHIP_WORD *)address;
    CHIP_WORD* EndAddress = (CHIP_WORD *)(address + length);

    erased = true;

    while (ChipAddress < EndAddress) {
        if (*ChipAddress != (CHIP_WORD)-1) {
            erased = false;

            break;
        }

        ChipAddress++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_EraseSector(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    if (sector >= SIZEOF_ARRAY(deploymentSectorAddress)) return TinyCLR_Result::IndexOutOfRange;

    uint32_t address = deploymentSectorAddress[sector];
    uint32_t num = 0;

    while (num < SIZEOF_ARRAY(flashSectorAddress)) {
        if (address <= flashSectorAddress[num])
            break;

        num++;
    }

    if (num == SIZEOF_ARRAY(flashSectorAddress))
        return TinyCLR_Result::ArgumentInvalid;

    if (STM32F4_FLASH->CR & FLASH_CR_LOCK) { // unlock
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY1;
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY2;
    }

    if (num >= 12) {
        num += 4;
    }

    // enable erasing
    uint32_t cr = num * FLASH_CR_SNB_0 | FLASH_CR_SER;
    STM32F4_FLASH->CR = cr;
    // start erase
    cr |= FLASH_CR_STRT;
    STM32F4_FLASH->CR = cr;
    // assure busy flag is set up (see STM32F4 errata)
    STM32F4_FLASH->CR = cr;
    // wait for completion
    while (STM32F4_FLASH->SR & FLASH_SR_BSY);

    // reset & lock the controller
    STM32F4_FLASH->CR = FLASH_CR_LOCK;

    TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportsXip) {
    supportsXip = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_Release(const TinyCLR_Deployment_Provider* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}


TinyCLR_Result STM32F4_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
    int32_t sectors = SIZEOF_ARRAY(deploymentSectorAddress);

    size = 0;

    for (int32_t i = 0; i < sectors; i++) {
        if (address >= deploymentSectorAddress[i] && address < deploymentSectorAddress[i] + deploymentSectorSize[i]) {
            size = deploymentSectorSize[i];

            break;
        }
    }

    return size > 0 ? TinyCLR_Result::Success : TinyCLR_Result::ArgumentInvalid;
}

TinyCLR_Result STM32F4_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    addresses = deploymentSectorAddress;
    sizes = deploymentSectorSize;
    count = SIZEOF_ARRAY(deploymentSectorAddress);

    return TinyCLR_Result::Success;
}