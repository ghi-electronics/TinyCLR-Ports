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

struct STM32F4_Flash_Deployment {
    uint32_t id;
    uint32_t address;
    uint32_t size;
};

#if STM32F4_SUPPLY_VOLTAGE_MV < 2100
#error 16 bit Flash programming not allowed for voltages below 2.1V
#endif
#if STM32F4_AHB_CLOCK_HZ < 1000000
#error Flash programming not allowed for HCLK below 1MHz
#endif

TinyCLR_Result STM32F4_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);

static const STM32F4_Flash_Deployment deploymentSectors[] = DEPLOYMENT_SECTORS;
uint32_t deploymentSectorAddress[SIZEOF_ARRAY(deploymentSectors)];
uint32_t deploymentSectorSize[SIZEOF_ARRAY(deploymentSectors)];

static const uint32_t STM32F4_FLASH_KEY1 = 0x45670123;
static const uint32_t STM32F4_FLASH_KEY2 = 0xcdef89ab;

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

const TinyCLR_Api_Info* STM32F4_Deployment_GetApi() {
    deploymentProvider.ApiInfo = &deploymentApi;
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
    deploymentApi.Implementation = &deploymentProvider;

    STM32F4_Deplpoyment_Reset();

    return &deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    int32_t bytePerSector = 0;

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F4_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + length);
    uint32_t* pBuf = (uint32_t*)buffer;

    while (addressStart < addressEnd) {
        *pBuf++ = *addressStart++;
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

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + length);
    uint32_t* pBuf = (uint32_t*)buffer;

    // enable programming
    STM32F4_FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_1;

    while (addressStart < addressEnd) {
        if (*addressStart != *pBuf) {
            // write data
            *addressStart = *pBuf;
            // wait for completion
            while (STM32F4_FLASH->SR & FLASH_SR_BSY);
            // check
            if (*addressStart != *pBuf) {
                return TinyCLR_Result::InvalidOperation;
            }
        }
        addressStart++;
        pBuf++;
    }

    // reset & lock the controller
    STM32F4_FLASH->CR = FLASH_CR_LOCK;

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_IsSectorErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    if (sector >= SIZEOF_ARRAY(deploymentSectors)) return TinyCLR_Result::IndexOutOfRange;

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(deploymentSectors[sector].address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(deploymentSectors[sector].address + deploymentSectors[sector].size);

    erased = true;

    while (addressStart < addressEnd) {
        if (*addressStart != 0xFFFFFFFF) {
            erased = false;

            break;
        }

        addressStart++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_EraseSector(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    if (sector >= SIZEOF_ARRAY(deploymentSectors)) return TinyCLR_Result::IndexOutOfRange;

    uint32_t num = deploymentSectors[sector].id;

    if (num > 11) num += 4;

    if (STM32F4_FLASH->CR & FLASH_CR_LOCK) { // unlock
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY1;
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY2;
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
    int32_t sectors = SIZEOF_ARRAY(deploymentSectors);

    size = 0;

    for (int32_t i = 0; i < sectors; i++) {
        if (address >= deploymentSectors[i].address && address < deploymentSectors[i].address + deploymentSectors[i].size) {
            size = deploymentSectors[i].size;

            break;
        }
    }

    return size > 0 ? TinyCLR_Result::Success : TinyCLR_Result::ArgumentInvalid;
}

TinyCLR_Result STM32F4_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    addresses = deploymentSectorAddress;
    sizes = deploymentSectorSize;
    count = SIZEOF_ARRAY(deploymentSectorAddress);

    return count > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotImplemented;
}

void STM32F4_Deplpoyment_Reset() {
    for (int32_t i = 0; i < SIZEOF_ARRAY(deploymentSectors); i++) {
        deploymentSectorAddress[i] = deploymentSectors[i].address;
        deploymentSectorSize[i] = deploymentSectors[i].size;
    }
}