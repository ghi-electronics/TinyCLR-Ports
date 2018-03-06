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

#include "AT49BV322DT_Flash.h"
#include <Device.h>

typedef void(*IAP)(uint32_t[], uint32_t[]);

#define AT49BV322DT_FLASH_BASE_ADDRESS                      0x80000000
#define AT49BV322DT_FLASH_SECTOR_SIZE                       (64*1024)

#define GET_ADDR(addr)	(volatile uint16_t *)(AT49BV322DT_FLASH_BASE_ADDRESS | (addr<<1))

#define AT49BV322DT_FLASH_SECTOR_NUM   (13)

uint32_t g_AT49BV322DT_Flash_SectorAddress[AT49BV322DT_FLASH_SECTOR_NUM];
uint32_t g_AT49BV322DT_Flash_SectorSize[AT49BV322DT_FLASH_SECTOR_NUM];

TinyCLR_Result AT49BV322DT_Flash_Read(uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint16_t* ChipAddress = (uint16_t *)address;
    uint16_t* EndAddress = (uint16_t *)(address + length);
    uint16_t *pBuf = (uint16_t *)buffer;

    while (ChipAddress < EndAddress) {
        *pBuf++ = *ChipAddress++;
    }

    return TinyCLR_Result::Success;
}


TinyCLR_Result AT49BV322DT_Flash_Write(uint32_t address, size_t length, const uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    volatile uint16_t *ip;
    uint32_t startAddress = address;
    int32_t length16 = length >> 1;

    while (length16 > 0) {

        ip = (uint16_t*)(startAddress);
        if (*ip != 0xffff)
            return TinyCLR_Result::InvalidOperation;//not eased

        ip = GET_ADDR(0x5555);
        *ip = 0x00AA;
        ip = GET_ADDR(0x2aaa);
        *ip = 0x0055;
        ip = GET_ADDR(0x5555);
        *ip = 0x00A0;

        ip = (uint16_t*)(startAddress);
        *ip = *((uint16_t*)(buffer));

        while (*ip != *ip);

        length16--;
        buffer += 2;
        startAddress += 2;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT49BV322DT_Flash_IsBlockErased(uint32_t sector, bool &erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t address = g_AT49BV322DT_Flash_SectorAddress[sector];
    int32_t size = g_AT49BV322DT_Flash_SectorSize[sector];
    uint32_t endAddress = address + size;

    erased = true;

    while (address < endAddress) {
        uint16_t data;

        AT49BV322DT_Flash_Read(address, 2, reinterpret_cast<uint8_t*>(&data));

        if (data != 0xFFFF) {
            erased = false;
            break;
        }

        address += 2;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT49BV322DT_Flash_EraseBlock(uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    volatile uint16_t *ip;
    uint32_t address = g_AT49BV322DT_Flash_SectorAddress[sector];

    ip = GET_ADDR(0x5555);
    *ip = 0x00AA;
    ip = GET_ADDR(0x2AAA);
    *ip = 0x0055;
    ip = GET_ADDR(0x5555);
    *ip = 0x0080;
    ip = GET_ADDR(0x5555);
    *ip = 0x00AA;
    ip = GET_ADDR(0x2AAA);
    *ip = 0x0055;

    ip = (uint16_t*)(address);
    *ip = 0x0030;

    //wait until finished
    while (*ip != *ip);

    TinyCLR_Result::Success;
}

TinyCLR_Result AT49BV322DT_Flash_Acquire(bool& supportXIP) {
    supportXIP = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT49BV322DT_Flash_Release() {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT49BV322DT_Flash_GetBytesPerSector(uint32_t address, int32_t& size) {
    size = AT49BV322DT_FLASH_SECTOR_SIZE;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT49BV322DT_Flash_GetSectorMap(const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < AT49BV322DT_FLASH_SECTOR_NUM; i++) {
        g_AT49BV322DT_Flash_SectorAddress[i] = (i * AT49BV322DT_FLASH_SECTOR_SIZE) | AT49BV322DT_FLASH_BASE_ADDRESS;
        g_AT49BV322DT_Flash_SectorSize[i] = AT49BV322DT_FLASH_SECTOR_SIZE;
    }

    addresses = g_AT49BV322DT_Flash_SectorAddress;
    sizes = g_AT49BV322DT_Flash_SectorSize;
    count = AT49BV322DT_FLASH_SECTOR_NUM;

    return TinyCLR_Result::Success;
}
