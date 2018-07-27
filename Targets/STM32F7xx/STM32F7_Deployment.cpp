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

#if defined(__GNUC__)
// GCC ARM linker does not link to some variable below if optimize mode.
#pragma GCC optimize 0
#endif

#include "STM32F7.h"
#include <stdio.h>

#define TOTAL_DEPLOYMENT_CONTROLLERS 1

#ifndef STM32F7_FLASH
#define STM32F7_FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#endif

/** @defgroup FLASH_Program_Parallelism FLASH Program Parallelism
  * @{
  */
#define FLASH_PSIZE_BYTE           ((uint32_t)0x00000000)
#define FLASH_PSIZE_HALF_WORD      ((uint32_t)FLASH_CR_PSIZE_0)
#define FLASH_PSIZE_WORD           ((uint32_t)FLASH_CR_PSIZE_1)
#define FLASH_PSIZE_DOUBLE_WORD    ((uint32_t)FLASH_CR_PSIZE)
#define CR_PSIZE_MASK              ((uint32_t)0xFFFFFCFF)

  /*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos              (0U)
#define FLASH_SR_EOP_Msk              (0x1U << FLASH_SR_EOP_Pos)               /*!< 0x00000001 */
#define FLASH_SR_EOP                  FLASH_SR_EOP_Msk
#define FLASH_SR_OPERR_Pos            (1U)
#define FLASH_SR_OPERR_Msk            (0x1U << FLASH_SR_OPERR_Pos)             /*!< 0x00000002 */
#define FLASH_SR_OPERR                FLASH_SR_OPERR_Msk
#define FLASH_SR_WRPERR_Pos           (4U)
#define FLASH_SR_WRPERR_Msk           (0x1U << FLASH_SR_WRPERR_Pos)            /*!< 0x00000010 */
#define FLASH_SR_WRPERR               FLASH_SR_WRPERR_Msk
#define FLASH_SR_PGAERR_Pos           (5U)
#define FLASH_SR_PGAERR_Msk           (0x1U << FLASH_SR_PGAERR_Pos)            /*!< 0x00000020 */
#define FLASH_SR_PGAERR               FLASH_SR_PGAERR_Msk
#define FLASH_SR_PGPERR_Pos           (6U)
#define FLASH_SR_PGPERR_Msk           (0x1U << FLASH_SR_PGPERR_Pos)            /*!< 0x00000040 */
#define FLASH_SR_PGPERR               FLASH_SR_PGPERR_Msk
#define FLASH_SR_ERSERR_Pos           (7U)
#define FLASH_SR_ERSERR_Msk           (0x1U << FLASH_SR_ERSERR_Pos)            /*!< 0x00000080 */
#define FLASH_SR_ERSERR               FLASH_SR_ERSERR_Msk
#define FLASH_SR_BSY_Pos              (16U)
#define FLASH_SR_BSY_Msk              (0x1U << FLASH_SR_BSY_Pos)               /*!< 0x00010000 */
#define FLASH_SR_BSY                  FLASH_SR_BSY_Msk


#define SECTOR_MASK               ((uint32_t)0xFFFFFF07)


#if STM32F7_SUPPLY_VOLTAGE_MV < 2100
#error 16 bit Flash programming not allowed for voltages below 2.1V
#endif
#if STM32F7_AHB_CLOCK_HZ < 1000000
#error Flash programming not allowed for HCLK below 1MHz
#endif

static const uint32_t STM32F7_FLASH_KEY1 = 0x45670123;
static const uint32_t STM32F7_FLASH_KEY2 = 0xcdef89ab;

struct DeploymentSector {
    uint32_t id;
    uint32_t address;
    uint32_t size;
};

size_t STM32F7_Flash_GetSectorSizeFormAddress(const TinyCLR_Storage_Controller* self, uint32_t address);

static const DeploymentSector deploymentSectors[] = DEPLOYMENT_SECTORS;
static TinyCLR_Storage_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];

struct DeploymentState {
    uint32_t controllerIndex;

    size_t regionCount;
    uint64_t regionAddresses[SIZEOF_ARRAY(deploymentSectors)];
    size_t regionSizes[SIZEOF_ARRAY(deploymentSectors)];

    TinyCLR_Storage_Descriptor storageDescriptor;
    TinyCLR_Startup_DeploymentConfiguration deploymentConfiguration;

    bool isOpened = false;
};

static DeploymentState deploymentState[TOTAL_DEPLOYMENT_CONTROLLERS];

const TinyCLR_Api_Info* STM32F7_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Acquire = &STM32F7_Flash_Acquire;
        deploymentControllers[i].Release = &STM32F7_Flash_Release;
        deploymentControllers[i].Open = &STM32F7_Flash_Open;
        deploymentControllers[i].Close = &STM32F7_Flash_Close;
        deploymentControllers[i].Read = &STM32F7_Flash_Read;
        deploymentControllers[i].Write = &STM32F7_Flash_Write;
        deploymentControllers[i].Erase = &STM32F7_Flash_Erase;
        deploymentControllers[i].IsErased = &STM32F7_Flash_IsErased;
        deploymentControllers[i].GetDescriptor = &STM32F7_Flash_GetDescriptor;
        deploymentControllers[i].IsPresent = &STM32F7_Flash_IsPresent;
        deploymentControllers[i].SetPresenceChangedHandler = &STM32F7_Flash_SetPresenceChangedHandler;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.StorageController";
        deploymentApi[i].Type = TinyCLR_Api_Type::StorageController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = &deploymentState[i];

        deploymentState[i].controllerIndex = i;
        deploymentState[i].regionCount = SIZEOF_ARRAY(deploymentSectors);
    }

    STM32F7_Deplpoyment_Reset();

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout) {
    if (data == nullptr) return TinyCLR_Result::ArgumentNull;

    auto bytePerSector = STM32F7_Flash_GetSectorSizeFormAddress(self, address);

    if (bytePerSector <= 0) return TinyCLR_Result::IndexOutOfRange;

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + count);
    uint32_t* pBuf = (uint32_t*)data;

    while (addressStart < addressEnd) {
        *pBuf++ = *addressStart++;
    }

    return TinyCLR_Result::Success;
}


TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout) {
    if (data == nullptr) return TinyCLR_Result::ArgumentNull;

    auto bytePerSector = STM32F7_Flash_GetSectorSizeFormAddress(self, address);

    if (bytePerSector <= 0) return TinyCLR_Result::IndexOutOfRange;

    STM32F7_Startup_CacheDisable();

    if (STM32F7_FLASH->CR & FLASH_CR_LOCK) { // unlock
        STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY1;
        STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY2;
    }

    while (STM32F7_FLASH->SR & FLASH_SR_BSY);

    STM32F7_FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR);

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + count);
    uint32_t* pBuf = (uint32_t*)data;

    // enable programming
    auto to = timeout;

    while (addressStart < addressEnd) {
        if (*addressStart != *pBuf) {

            STM32F7_FLASH->CR &= CR_PSIZE_MASK;
            STM32F7_FLASH->CR |= FLASH_PSIZE_WORD;
            STM32F7_FLASH->CR |= FLASH_CR_EOPIE;
            STM32F7_FLASH->CR |= FLASH_CR_PG;

            // write data
            *addressStart = *pBuf;

            __DSB();

            // wait for completion
            to = timeout;

            while (((STM32F7_FLASH->SR & FLASH_SR_EOP) == 0) || (STM32F7_FLASH->SR & FLASH_SR_BSY)) {
                STM32F7_Time_Delay(nullptr, 1000);
                to--;

                if (to == 0)
                    goto end_programing;
            }

            STM32F7_FLASH->SR |= FLASH_SR_EOP;

            if (*addressStart != *pBuf) {
                to = 0;

                goto end_programing;
            }
        }

        addressStart++;
        pBuf++;
    }

end_programing:

    STM32F7_FLASH->CR &= (~FLASH_CR_PG);

    // reset & lock the controller
    STM32F7_FLASH->CR |= FLASH_CR_LOCK;

    STM32F7_Startup_CacheEnable();

    return timeout != 0 ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, bool& erased) {
    auto sector = address; //address is sector. Use sector for clear
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (sector >= state->regionCount) return TinyCLR_Result::IndexOutOfRange;

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

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout) {
    auto sector = address; //address is sector. Use sector for clear
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (sector >= state->regionCount) return TinyCLR_Result::IndexOutOfRange;

    uint32_t num = deploymentSectors[sector].id;

    if (num > 11) num += 4;

    STM32F7_Startup_CacheDisable();

    STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY1;
    STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY2;

    STM32F7_FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR);

    STM32F7_FLASH->CR = FLASH_PSIZE_WORD;
    STM32F7_FLASH->CR |= FLASH_CR_EOPIE;
    STM32F7_FLASH->CR |= (num << 3);
    STM32F7_FLASH->CR |= FLASH_CR_SER;

    STM32F7_FLASH->CR |= FLASH_CR_STRT;

    __DSB();
    // wait for completion
    while (((STM32F7_FLASH->SR & FLASH_SR_EOP) == 0) || (STM32F7_FLASH->SR & FLASH_SR_BSY));

    STM32F7_FLASH->CR &= (~FLASH_CR_SER);
    STM32F7_FLASH->CR &= SECTOR_MASK;

    // reset & lock the controller
    STM32F7_FLASH->CR |= FLASH_CR_LOCK;

    STM32F7_Startup_CacheEnable();

    TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_Acquire(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (state != nullptr)
        if (!state->isOpened)
            return TinyCLR_Result::Success;
        else
            return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::ArgumentNull;
}

TinyCLR_Result STM32F7_Flash_Release(const TinyCLR_Storage_Controller* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_Open(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (state->isOpened)
        return TinyCLR_Result::SharingViolation;

    state->storageDescriptor.CanReadDirect = true;
    state->storageDescriptor.CanWriteDirect = true;
    state->storageDescriptor.CanExecuteDirect = true;
    state->storageDescriptor.EraseBeforeWrite = true;
    state->storageDescriptor.Removable = false;
    state->storageDescriptor.RegionsRepeat = false;

    state->storageDescriptor.RegionCount = state->regionCount;
    state->storageDescriptor.RegionAddresses = reinterpret_cast<const uint64_t*>(state->regionAddresses);
    state->storageDescriptor.RegionSizes = reinterpret_cast<const size_t*>(state->regionSizes);

    state->deploymentConfiguration.RegionCount = state->storageDescriptor.RegionCount;
    state->deploymentConfiguration.RegionAddresses = state->storageDescriptor.RegionAddresses;
    state->deploymentConfiguration.RegionSizes = state->storageDescriptor.RegionSizes;

    state->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_Close(const TinyCLR_Storage_Controller* self) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    if (!state->isOpened)
        return TinyCLR_Result::NotFound;

    state->isOpened = false;

    return TinyCLR_Result::Success;
}

size_t STM32F7_Flash_GetSectorSizeFormAddress(const TinyCLR_Storage_Controller* self, uint32_t address) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    for (int32_t i = 0; i < state->regionCount; i++) {
        if (address >= state->regionAddresses[i] && address < state->regionAddresses[i] + state->regionSizes[i]) {
            return state->regionSizes[i];
        }
    }

    return -1;
}

TinyCLR_Result STM32F7_Flash_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_IsPresent(const TinyCLR_Storage_Controller* self, bool& present) {
    present = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& storageDescriptor) {
    auto state = reinterpret_cast<DeploymentState*>(self->ApiInfo->State);

    storageDescriptor = &state->storageDescriptor;

    return storageDescriptor->RegionCount > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotImplemented;
}

const TinyCLR_Startup_DeploymentConfiguration* STM32F7_Flash_GetDeploymentConfiguration() {
    auto state = &deploymentState[0];

    return reinterpret_cast<const TinyCLR_Startup_DeploymentConfiguration*>(&state->deploymentConfiguration);
}

void STM32F7_Deplpoyment_Reset() {
    for (auto c = 0; c < TOTAL_DEPLOYMENT_CONTROLLERS; c++) {
        auto state = &deploymentState[c];

        for (auto i = 0; i < state->regionCount; i++) {
            state->regionAddresses[i] = deploymentSectors[i].address;
            state->regionSizes[i] = deploymentSectors[i].size;
        }
    }
}