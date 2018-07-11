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


typedef uint32_t CHIP_WORD;

struct STM32F4_Flash_Deployment {
    uint32_t id;
    uint32_t address;
    uint32_t size;
};

#define FLASH_CR_PSIZE_BITS FLASH_CR_PSIZE_0 // 16 bit programming

#if STM32F7_SUPPLY_VOLTAGE_MV < 2100
#error 16 bit Flash programming not allowed for voltages below 2.1V
#endif
#if STM32F7_AHB_CLOCK_HZ < 1000000
#error Flash programming not allowed for HCLK below 1MHz
#endif

TinyCLR_Result STM32F7_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);

static const STM32F4_Flash_Deployment deploymentSectors[] = DEPLOYMENT_SECTORS;
uint32_t deploymentSectorAddress[SIZEOF_ARRAY(deploymentSectors)];
uint32_t deploymentSectorSize[SIZEOF_ARRAY(deploymentSectors)];

static const uint32_t STM32F7_FLASH_KEY1 = 0x45670123;
static const uint32_t STM32F7_FLASH_KEY2 = 0xcdef89ab;

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

const TinyCLR_Api_Info* STM32F7_Deployment_GetApi() {
    deploymentProvider.ApiInfo = &deploymentApi;
    deploymentProvider.Acquire = &STM32F7_Flash_Acquire;
    deploymentProvider.Release = &STM32F7_Flash_Release;
    deploymentProvider.Read = &STM32F7_Flash_Read;
    deploymentProvider.Write = &STM32F7_Flash_Write;
    deploymentProvider.EraseSector = &STM32F7_Flash_EraseSector;
    deploymentProvider.IsSectorErased = &STM32F7_Flash_IsSectorErased;
    deploymentProvider.GetSectorMap = &STM32F7_Flash_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Implementation = &deploymentProvider;

    for (int32_t i = 0; i < SIZEOF_ARRAY(deploymentSectors); i++) {
        deploymentSectorAddress[i] = deploymentSectors[i].address;
        deploymentSectorSize[i] = deploymentSectors[i].size;
    }

    return &deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    int32_t bytePerSector = 0;

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F7_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    CHIP_WORD* ChipAddress = (CHIP_WORD *)address;
    CHIP_WORD* EndAddress = (CHIP_WORD *)(address + length);
    CHIP_WORD *pBuf = (CHIP_WORD *)buffer;

    while (ChipAddress < EndAddress) {
        *pBuf++ = *ChipAddress++;
    }

    return TinyCLR_Result::Success;
}

volatile uint32_t debugflashstatus = 0;

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    volatile int32_t timeout = 2000000; // 2 seconds
    int32_t bytePerSector = 0;

    STM32F7_Startup_CacheDisable();

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F7_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY1;
    STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY2;

    while (STM32F7_FLASH->SR & FLASH_SR_BSY);

    STM32F7_FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR);

    CHIP_WORD* ChipAddress = (CHIP_WORD *)address;
    CHIP_WORD* EndAddress = (CHIP_WORD *)(address + length);
    CHIP_WORD *pBuf = (CHIP_WORD *)buffer;

    // enable programming
    while (ChipAddress < EndAddress) {
        if (*ChipAddress != *pBuf) {

            STM32F7_FLASH->CR &= CR_PSIZE_MASK;
            STM32F7_FLASH->CR |= FLASH_PSIZE_WORD;
            STM32F7_FLASH->CR |= FLASH_CR_EOPIE;
            STM32F7_FLASH->CR |= FLASH_CR_PG;

            // write data
            *ChipAddress = *pBuf;

            __DSB();

            // wait for completion
            while (((STM32F7_FLASH->SR & FLASH_SR_EOP) == 0) || (STM32F7_FLASH->SR & FLASH_SR_BSY)) {
                STM32F7_Time_Delay(nullptr, 1); // asure host recognizes reattach
                timeout--;

                if (timeout == 0)
                    return TinyCLR_Result::InvalidOperation;
            }

            STM32F7_FLASH->SR |= FLASH_SR_EOP;

            if (*ChipAddress != *pBuf) {
                return TinyCLR_Result::InvalidOperation;
            }
        }

        ChipAddress++;
        pBuf++;
    }

    STM32F7_FLASH->CR &= (~FLASH_CR_PG);

    // reset & lock the controller
    STM32F7_FLASH->CR |= FLASH_CR_LOCK;

    STM32F7_Startup_CacheEnable();

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_IsSectorErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    if (sector >= SIZEOF_ARRAY(deploymentSectors)) return TinyCLR_Result::IndexOutOfRange;

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



TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_EraseSector(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    uint32_t cr, num;

    if (sector >= SIZEOF_ARRAY(deploymentSectors)) return TinyCLR_Result::IndexOutOfRange;

    num = deploymentSectors[sector].id;

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

TinyCLR_Result STM32F7_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportsXip) {
    supportsXip = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_Release(const TinyCLR_Deployment_Provider* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}


TinyCLR_Result STM32F7_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
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

TinyCLR_Result STM32F7_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    addresses = deploymentSectorAddress;
    sizes = deploymentSectorSize;
    count = SIZEOF_ARRAY(deploymentSectorAddress);

    return TinyCLR_Result::Success;
}

