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


struct DeploymentSector {
    uint32_t id;
    uint32_t address;
    uint32_t size;
};

#if STM32F7_SUPPLY_VOLTAGE_MV < 2100
#error 16 bit Flash programming not allowed for voltages below 2.1V
#endif
#if STM32F7_AHB_CLOCK_HZ < 1000000
#error Flash programming not allowed for HCLK below 1MHz
#endif

TinyCLR_Result STM32F7_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Controller* self, uint32_t address, int32_t& size);

static const DeploymentSector deploymentSectors[] = DEPLOYMENT_SECTORS;
uint32_t deploymentSectorAddress[SIZEOF_ARRAY(deploymentSectors)];
uint32_t deploymentSectorSize[SIZEOF_ARRAY(deploymentSectors)];

static const uint32_t STM32F7_FLASH_KEY1 = 0x45670123;
static const uint32_t STM32F7_FLASH_KEY2 = 0xcdef89ab;

static TinyCLR_Deployment_Controller deploymentControllers[TOTAL_DEPLOYMENT_CONTROLLERS];
static TinyCLR_Api_Info deploymentApi[TOTAL_DEPLOYMENT_CONTROLLERS];

const TinyCLR_Api_Info* STM32F7_Deployment_GetApi() {
    for (int32_t i = 0; i < TOTAL_DEPLOYMENT_CONTROLLERS; i++) {
        deploymentControllers[i].ApiInfo = &deploymentApi[i];
        deploymentControllers[i].Initialize = &STM32F7_Flash_Initialize;
        deploymentControllers[i].Uninitialize = &STM32F7_Flash_Uninitialize;
        deploymentControllers[i].Read = &STM32F7_Flash_Read;
        deploymentControllers[i].Write = &STM32F7_Flash_Write;
        deploymentControllers[i].EraseSector = &STM32F7_Flash_EraseSector;
        deploymentControllers[i].IsSectorErased = &STM32F7_Flash_IsSectorErased;
        deploymentControllers[i].GetSectorMap = &STM32F7_Flash_GetSectorMap;

        deploymentApi[i].Author = "GHI Electronics, LLC";
        deploymentApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.DeploymentController";
        deploymentApi[i].Type = TinyCLR_Api_Type::DeploymentController;
        deploymentApi[i].Version = 0;
        deploymentApi[i].Implementation = &deploymentControllers[i];
        deploymentApi[i].State = nullptr;
    }

    STM32F7_Deplpoyment_Reset();

    return (const TinyCLR_Api_Info*)&deploymentApi;
}

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Read(const TinyCLR_Deployment_Controller* self, uint32_t address, size_t length, uint8_t* buffer) {
    int32_t bytePerSector = 0;

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F7_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + length);
    uint32_t* pBuf = (uint32_t*)buffer;

    while (addressStart < addressEnd) {
        *pBuf++ = *addressStart++;
    }

    return TinyCLR_Result::Success;
}


TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_Write(const TinyCLR_Deployment_Controller* self, uint32_t address, size_t length, const uint8_t* buffer) {
    int32_t bytePerSector = 0;

    STM32F7_Startup_CacheDisable();

    if (buffer == nullptr) return TinyCLR_Result::ArgumentNull;
    if (STM32F7_Flash_GetSectorSizeForAddress(self, address, bytePerSector) != TinyCLR_Result::Success)
        return TinyCLR_Result::IndexOutOfRange;

    if (STM32F7_FLASH->CR & FLASH_CR_LOCK) { // unlock
        STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY1;
        STM32F7_FLASH->KEYR = STM32F7_FLASH_KEY2;
    }

    while (STM32F7_FLASH->SR & FLASH_SR_BSY);

    STM32F7_FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR);

    uint32_t* addressStart = reinterpret_cast<uint32_t*>(address);
    uint32_t* addressEnd = reinterpret_cast<uint32_t*>(address + length);
    uint32_t* pBuf = (uint32_t*)buffer;

    // enable programming
    auto timeout = 10000000; // 10 seconds for a sector

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
            timeout = 10000000;

            while (((STM32F7_FLASH->SR & FLASH_SR_EOP) == 0) || (STM32F7_FLASH->SR & FLASH_SR_BSY)) {
                STM32F7_Time_Delay(nullptr, 1);
                timeout--;

                if (timeout == 0)
                    goto end_programing;
            }

            STM32F7_FLASH->SR |= FLASH_SR_EOP;

            if (*addressStart != *pBuf) {
                timeout = 0; // to return InvalidOperation

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

TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_IsSectorErased(const TinyCLR_Deployment_Controller* self, uint32_t sector, bool &erased) {
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



TinyCLR_Result __section("SectionForFlashOperations") STM32F7_Flash_EraseSector(const TinyCLR_Deployment_Controller* self, uint32_t sector) {
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

TinyCLR_Result STM32F7_Flash_Initialize(const TinyCLR_Deployment_Controller* self, bool& supportsXip) {
    supportsXip = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Flash_Uninitialize(const TinyCLR_Deployment_Controller* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}


TinyCLR_Result STM32F7_Flash_GetSectorSizeForAddress(const TinyCLR_Deployment_Controller* self, uint32_t address, int32_t& size) {
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

TinyCLR_Result STM32F7_Flash_GetSectorMap(const TinyCLR_Deployment_Controller* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    addresses = deploymentSectorAddress;
    sizes = deploymentSectorSize;
    count = SIZEOF_ARRAY(deploymentSectorAddress);

    return count > 0 ? TinyCLR_Result::Success : TinyCLR_Result::NotImplemented;
}

void STM32F7_Deplpoyment_Reset() {
    for (int32_t i = 0; i < SIZEOF_ARRAY(deploymentSectors); i++) {
        deploymentSectorAddress[i] = deploymentSectors[i].address;
        deploymentSectorSize[i] = deploymentSectors[i].size;
    }
}