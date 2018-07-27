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

#define TOTAL_DEPLOYMENT_CONTROLLERS 1

#ifndef STM32F4_FLASH
#define STM32F4_FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#endif

struct DeploymentSector {
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

TinyCLR_Result STM32F4_Flash_GetSectorSizeForAddress(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size);

static const DeploymentSector deploymentSectors[] = DEPLOYMENT_SECTORS;
static uint64_t deploymentSectorAddress[SIZEOF_ARRAY(deploymentSectors)];
static size_t deploymentSectorSize[SIZEOF_ARRAY(deploymentSectors)];

static const uint32_t STM32F4_FLASH_KEY1 = 0x45670123;
static const uint32_t STM32F4_FLASH_KEY2 = 0xcdef89ab;

static TinyCLR_Storage_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Storage_Descriptor deploymentDescriptor;
TinyCLR_Startup_DeploymentConfiguration deploymentConfiguration;

const TinyCLR_Api_Info* STM32F4_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &STM32F4_Flash_Acquire;
        deploymentControllers[i].Release = &STM32F4_Flash_Release;
        deploymentControllers[i].Open = &STM32F4_Flash_Open;
        deploymentControllers[i].Close = &STM32F4_Flash_Close;
        deploymentControllers[i].Read = &STM32F4_Flash_Read;
        deploymentControllers[i].Write = &STM32F4_Flash_Write;
        deploymentControllers[i].Erase = &STM32F4_Flash_Erase;
        deploymentControllers[i].IsErased = &STM32F4_Flash_IsErased;
        deploymentControllers[i].GetDescriptor = &STM32F4_Flash_GetDescriptor;
        deploymentControllers[i].IsPresent = &STM32F4_Flash_IsPresent;
        deploymentControllers[i].SetPresenceChangedHandler = &STM32F4_Flash_SetPresenceChangedHandler;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.StorageController";
        deploymentApi[i].Type = TinyCLR_Api_Type::StorageController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = nullptr;
    }

    STM32F4_Deplpoyment_Reset();

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    int32_t bytePerSector = 0;

    if (data == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F4_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + count);
    uint32_t* pBuf = (uint32_t*)data;

    while (addressStart < addressEnd) {
        *pBuf++ = *addressStart++;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    int32_t bytePerSector = 0;

    if (data == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F4_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    if (STM32F4_FLASH->CR & FLASH_CR_LOCK) { // unlock
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY1;
        STM32F4_FLASH->KEYR = STM32F4_FLASH_KEY2;
    }

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + count);
    uint32_t* pBuf = (uint32_t*)data;

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

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, bool& erased) {
    auto sector = address; //address is sector. Use sector for clear

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

TinyCLR_Result __section("SectionForFlashOperations") STM32F4_Flash_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    auto sector = address; //address is sector. Use sector for clear

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

TinyCLR_Result STM32F4_Flash_Acquire(const TinyCLR_Storage_Controller* self) {
    deploymentDescriptor.CanReadDirect = true;
    deploymentDescriptor.CanWriteDirect = true;
    deploymentDescriptor.CanExecuteDirect = true;
    deploymentDescriptor.EraseBeforeWrite = true;
    deploymentDescriptor.Removable = false;
    deploymentDescriptor.RegionsRepeat = false;

    deploymentDescriptor.RegionCount = SIZEOF_ARRAY(deploymentSectors);
    deploymentDescriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(deploymentSectorAddress);
    deploymentDescriptor.RegionSizes = reinterpret_cast<const size_t*>(deploymentSectorSize);

    deploymentConfiguration.RegionCount = deploymentDescriptor.RegionCount;
    deploymentConfiguration.RegionAddresses = deploymentDescriptor.RegionAddresses;
    deploymentConfiguration.RegionSizes = deploymentDescriptor.RegionSizes;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_Release(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_Open(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_Close(const TinyCLR_Storage_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_GetSectorSizeForAddress(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size) {
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

TinyCLR_Result STM32F4_Flash_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Flash_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor) {
    descriptor = &deploymentDescriptor;

    return descriptor->RegionCount > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotImplemented;
}

void STM32F4_Deplpoyment_Reset() {
    for (int32_t i = 0; i < SIZEOF_ARRAY(deploymentSectors); i++) {
        deploymentSectorAddress[i] = deploymentSectors[i].address;
        deploymentSectorSize[i] = deploymentSectors[i].size;
    }

}