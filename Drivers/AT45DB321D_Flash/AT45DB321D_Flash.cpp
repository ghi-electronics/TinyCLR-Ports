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

#include "AT45DB321D_Flash.h"
#include <Device.h>

#define AT45DB321D_FLASH_SECTOR_START                 0
#define AT45DB321D_FLASH_SECTOR_END                   1023
#define AT45DB321D_FLASH_SECTOR_NUM                   (AT45DB321D_FLASH_SECTOR_END - AT45DB321D_FLASH_SECTOR_START + 1)

#define AT45DB321D_FLASH_COMMAND_READID                          0x9F
#define AT45DB321D_FLASH_COMMAND_READ_STATUS_REGISTER            0xD7
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_1                  0x84
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_2                  0x87
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_1_TO_MEMORY        0x83
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_2_TO_MEMORY        0x86
#define AT45DB321D_FLASH_COMMAND_READ_FROM_MAIN_MEMORY_LEGACY    0xE8
#define AT45DB321D_FLASH_COMMAND_READ_FROM_MAIN_MEMORY_DIRECT    0xD2
#define AT45DB321D_FLASH_COMMAND_PAGE_ERASE                      0x81

#define AT45DB321D_FLASH_COMMAND_READ_STATUS_REGISTER            0xD7
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_1                  0x84
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_2                  0x87
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_1_TO_MEMORY        0x83
#define AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_2_TO_MEMORY        0x86
#define AT45DB321D_FLASH_COMMAND_READ_FROM_MAIN_MEMORY_LEGACY    0xE8
#define AT45DB321D_FLASH_COMMAND_READ_FROM_MAIN_MEMORY_DIRECT    0xD2
#define AT45DB321D_FLASH_COMMAND_PAGE_ERASE                      0x81
#define AT45DB321D_FLASH_COMMAND_BLOCK_ERASE                     0x50

#define AT45DB321D_FLASH_COMMAND_SIZE                      4
#define AT45DB321D_FLASH_ACCESS_TIMEOUT                    1000

#define AT45DB321D_FLASH_PAGE_SIZE                         528
#define AT45DB321D_FLASH_BLOCK_SIZE                        (528 * 8)

#define AT45DB321D_FLASH_MANUFACTURER_CODE                 0x1F
#define AT45DB321D_FLASH_DEVICE_CODE                       0x27

#define AT45DB321D_SPI_CLOCK_HZ 20000000

const TinyCLR_Spi_Provider* g_AT45DB321D_Flash_SpiProvider;
const TinyCLR_NativeTime_Provider* g_AT45DB321D_Flash_TimeProvider;

uint32_t g_AT45DB321D_Flash_SpiChipSelectLine;

uint32_t g_AT45DB321D_Flash_SectorAddress[AT45DB321D_FLASH_SECTOR_NUM];
uint32_t g_AT45DB321D_Flash_SectorSize[AT45DB321D_FLASH_SECTOR_NUM];

uint8_t g_AT45DB321D_Flash_BufferRW[(AT45DB321D_FLASH_PAGE_SIZE * 1) + AT45DB321D_FLASH_COMMAND_SIZE];

uint8_t g_AT45DB321D_Flash_DataReadBuffer[AT45DB321D_FLASH_BLOCK_SIZE + 8];
uint8_t g_AT45DB321D_Flash_DataWriteBuffer[AT45DB321D_FLASH_BLOCK_SIZE + 8];

uint8_t AT45DB321D_Flash_GetStatus() {
    size_t writeLength;
    size_t readLength;

    g_AT45DB321D_Flash_DataWriteBuffer[0] = AT45DB321D_FLASH_COMMAND_READ_STATUS_REGISTER;
    g_AT45DB321D_Flash_DataWriteBuffer[1] = 0x00;

    writeLength = 2;
    readLength = 2;

    g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

    return g_AT45DB321D_Flash_DataReadBuffer[1];
}

TinyCLR_Result AT45DB321D_Flash_Read(uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t block = length / AT45DB321D_FLASH_PAGE_SIZE;
    int32_t rest = length % AT45DB321D_FLASH_PAGE_SIZE;
    int32_t index = 0;
    size_t writeLength;
    size_t readLength;

    g_AT45DB321D_Flash_SpiProvider->Acquire(g_AT45DB321D_Flash_SpiProvider);

    g_AT45DB321D_Flash_SpiProvider->SetActiveSettings(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_SpiChipSelectLine, AT45DB321D_SPI_CLOCK_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (block > 0) {
        uint32_t pageNumber = (address % AT45DB321D_FLASH_PAGE_SIZE) | ((address / AT45DB321D_FLASH_PAGE_SIZE) << 10);

        g_AT45DB321D_Flash_DataWriteBuffer[0] = AT45DB321D_FLASH_COMMAND_READ_FROM_MAIN_MEMORY_LEGACY;
        g_AT45DB321D_Flash_DataWriteBuffer[1] = pageNumber >> 16;
        g_AT45DB321D_Flash_DataWriteBuffer[2] = pageNumber >> 8;
        g_AT45DB321D_Flash_DataWriteBuffer[3] = pageNumber;
        g_AT45DB321D_Flash_DataWriteBuffer[4] = 0x00;
        g_AT45DB321D_Flash_DataWriteBuffer[5] = 0x00;
        g_AT45DB321D_Flash_DataWriteBuffer[6] = 0x00;
        g_AT45DB321D_Flash_DataWriteBuffer[7] = 0x00;

        writeLength = AT45DB321D_FLASH_PAGE_SIZE + 8;
        readLength = AT45DB321D_FLASH_PAGE_SIZE + 8;

        g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

        int32_t timeout;

        for (timeout = 0; timeout < AT45DB321D_FLASH_ACCESS_TIMEOUT; timeout++) {
            if (AT45DB321D_Flash_GetStatus() & 0x80)
                break;

            g_AT45DB321D_Flash_TimeProvider->WaitMicroseconds(g_AT45DB321D_Flash_TimeProvider, 1000);
        }

        memcpy(&buffer[index], &g_AT45DB321D_Flash_DataReadBuffer[8], AT45DB321D_FLASH_PAGE_SIZE);

        address += AT45DB321D_FLASH_PAGE_SIZE;
        index += AT45DB321D_FLASH_PAGE_SIZE;
        block--;
    }

    if (rest > 0) {
        uint32_t pageNumber = (address % AT45DB321D_FLASH_PAGE_SIZE) | ((address / AT45DB321D_FLASH_PAGE_SIZE) << 10);

        g_AT45DB321D_Flash_DataWriteBuffer[0] = AT45DB321D_FLASH_COMMAND_READ_FROM_MAIN_MEMORY_LEGACY;
        g_AT45DB321D_Flash_DataWriteBuffer[1] = pageNumber >> 16;
        g_AT45DB321D_Flash_DataWriteBuffer[2] = pageNumber >> 8;
        g_AT45DB321D_Flash_DataWriteBuffer[3] = pageNumber;
        g_AT45DB321D_Flash_DataWriteBuffer[4] = 0x00;
        g_AT45DB321D_Flash_DataWriteBuffer[5] = 0x00;
        g_AT45DB321D_Flash_DataWriteBuffer[6] = 0x00;
        g_AT45DB321D_Flash_DataWriteBuffer[7] = 0x00;

        writeLength = rest + 8;
        readLength = rest + 8;

        g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

        int32_t timeout;

        for (timeout = 0; timeout < AT45DB321D_FLASH_ACCESS_TIMEOUT; timeout++) {
            if (AT45DB321D_Flash_GetStatus() & 0x80)
                break;

            g_AT45DB321D_Flash_TimeProvider->WaitMicroseconds(g_AT45DB321D_Flash_TimeProvider, 1000);
        }

        memcpy(&buffer[index], &g_AT45DB321D_Flash_DataReadBuffer[8], rest);

        address += rest;
        index += rest;
        block--;
    }

    g_AT45DB321D_Flash_SpiProvider->Release(g_AT45DB321D_Flash_SpiProvider);

    return TinyCLR_Result::Success;
}

bool AT45DB321D_Flash_WriteSector(uint32_t pageNumber, uint8_t* dataBuffer) {
    size_t writeLength;
    size_t readLength;

    g_AT45DB321D_Flash_DataWriteBuffer[0] = AT45DB321D_FLASH_COMMAND_WRITE_BUFFER_1;
    g_AT45DB321D_Flash_DataWriteBuffer[1] = 0x00;
    g_AT45DB321D_Flash_DataWriteBuffer[2] = 0x00;
    g_AT45DB321D_Flash_DataWriteBuffer[3] = 0x00;

    memcpy(&g_AT45DB321D_Flash_DataWriteBuffer[4], dataBuffer, AT45DB321D_FLASH_PAGE_SIZE);

    writeLength = AT45DB321D_FLASH_COMMAND_SIZE + AT45DB321D_FLASH_PAGE_SIZE;
    readLength = AT45DB321D_FLASH_COMMAND_SIZE + AT45DB321D_FLASH_PAGE_SIZE;

    g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

    g_AT45DB321D_Flash_DataWriteBuffer[0] = 0x88;
    g_AT45DB321D_Flash_DataWriteBuffer[1] = (pageNumber << 2) >> 8;
    g_AT45DB321D_Flash_DataWriteBuffer[2] = pageNumber << 2;
    g_AT45DB321D_Flash_DataWriteBuffer[3] = 0x00;

    writeLength = AT45DB321D_FLASH_COMMAND_SIZE;
    readLength = AT45DB321D_FLASH_COMMAND_SIZE;

    g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

    int32_t timeout;

    for (timeout = 0; timeout < AT45DB321D_FLASH_ACCESS_TIMEOUT; timeout++) {
        if (AT45DB321D_Flash_GetStatus() & 0x80)
            return true;

        g_AT45DB321D_Flash_TimeProvider->WaitMicroseconds(g_AT45DB321D_Flash_TimeProvider, 1000);
    }

    return false;
}

TinyCLR_Result AT45DB321D_Flash_Write(uint32_t address, size_t length, const uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t pageNumber = address / AT45DB321D_FLASH_PAGE_SIZE;
    uint32_t pageOffset = address % AT45DB321D_FLASH_PAGE_SIZE;
    uint32_t currentIndex = 0;
    uint32_t remainingBytes = length;
    uint32_t beginningBytes = AT45DB321D_FLASH_PAGE_SIZE - pageOffset;
    uint32_t remainingBytesPageSegmentSize = AT45DB321D_FLASH_PAGE_SIZE - pageOffset;

    g_AT45DB321D_Flash_SpiProvider->Acquire(g_AT45DB321D_Flash_SpiProvider);

    g_AT45DB321D_Flash_SpiProvider->SetActiveSettings(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_SpiChipSelectLine, AT45DB321D_SPI_CLOCK_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    if (pageOffset) {
        memset(g_AT45DB321D_Flash_BufferRW, 0xFF, AT45DB321D_FLASH_PAGE_SIZE);

        if (length <= remainingBytesPageSegmentSize) {
            memcpy(&g_AT45DB321D_Flash_BufferRW[pageOffset], &buffer[currentIndex], length);

            remainingBytes -= length;
        }
        else {
            memcpy(&g_AT45DB321D_Flash_BufferRW[pageOffset], &buffer[currentIndex], beginningBytes);

            currentIndex += beginningBytes;
            remainingBytes -= beginningBytes;
        }

        AT45DB321D_Flash_WriteSector(pageNumber, g_AT45DB321D_Flash_BufferRW);

        pageNumber++;
    }

    if (!remainingBytes)
        TinyCLR_Result::Success;

    uint32_t sector = remainingBytes / AT45DB321D_FLASH_PAGE_SIZE;

    while (sector) {
        memset(g_AT45DB321D_Flash_BufferRW, 0xFF, AT45DB321D_FLASH_PAGE_SIZE);
        memcpy(g_AT45DB321D_Flash_BufferRW, &buffer[currentIndex], AT45DB321D_FLASH_PAGE_SIZE);

        AT45DB321D_Flash_WriteSector(pageNumber, g_AT45DB321D_Flash_BufferRW);

        currentIndex += AT45DB321D_FLASH_PAGE_SIZE;
        remainingBytes -= AT45DB321D_FLASH_PAGE_SIZE;
        sector--;
        pageNumber++;
    }

    if (!remainingBytes)
        TinyCLR_Result::Success;

    memset(g_AT45DB321D_Flash_BufferRW, 0xFF, AT45DB321D_FLASH_PAGE_SIZE);
    memcpy(g_AT45DB321D_Flash_BufferRW, &buffer[currentIndex], remainingBytes);

    AT45DB321D_Flash_WriteSector(pageNumber, g_AT45DB321D_Flash_BufferRW);

    g_AT45DB321D_Flash_SpiProvider->Release(g_AT45DB321D_Flash_SpiProvider);

    TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_IsBlockErased(uint32_t sector, bool &erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);
    uint32_t startAddress = g_AT45DB321D_Flash_SectorAddress[sector];

    int32_t block = AT45DB321D_FLASH_BLOCK_SIZE / AT45DB321D_FLASH_PAGE_SIZE;
    int32_t rest = AT45DB321D_FLASH_BLOCK_SIZE % AT45DB321D_FLASH_PAGE_SIZE;

    erased = true;

    while (block > 0) {
        AT45DB321D_Flash_Read(startAddress, AT45DB321D_FLASH_PAGE_SIZE, g_AT45DB321D_Flash_BufferRW);

        for (auto i = 0; i < AT45DB321D_FLASH_PAGE_SIZE; i++) {
            if (g_AT45DB321D_Flash_BufferRW[i] != 0xFF) {
                erased = false;

                return TinyCLR_Result::Success;
            }
        }

        block--;

        startAddress += AT45DB321D_FLASH_PAGE_SIZE;
    }

    if (rest > 0) {
        AT45DB321D_Flash_Read(startAddress, rest, g_AT45DB321D_Flash_BufferRW);

        for (auto i = 0; i < rest; i++) {
            if (g_AT45DB321D_Flash_BufferRW[i] != 0xFF) {
                erased = false;

                return TinyCLR_Result::Success;
            }
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_EraseBlock(uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    size_t writeLength;
    size_t readLength;

    uint32_t blockNumber = g_AT45DB321D_Flash_SectorAddress[sector] / (AT45DB321D_FLASH_BLOCK_SIZE);

    g_AT45DB321D_Flash_SpiProvider->Acquire(g_AT45DB321D_Flash_SpiProvider);

    g_AT45DB321D_Flash_SpiProvider->SetActiveSettings(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_SpiChipSelectLine, AT45DB321D_SPI_CLOCK_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    g_AT45DB321D_Flash_DataWriteBuffer[0] = AT45DB321D_FLASH_COMMAND_BLOCK_ERASE;
    g_AT45DB321D_Flash_DataWriteBuffer[1] = (blockNumber << 3u) >> 6u;
    g_AT45DB321D_Flash_DataWriteBuffer[2] = ((uint8_t)((blockNumber << 3u) & 0x3F) << 2u) + ((uint8_t)(0 >> 8u));
    g_AT45DB321D_Flash_DataWriteBuffer[3] = 0x00;

    writeLength = AT45DB321D_FLASH_COMMAND_SIZE;
    readLength = AT45DB321D_FLASH_COMMAND_SIZE;


    g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

    int32_t timeout;

    for (timeout = 0; timeout < AT45DB321D_FLASH_ACCESS_TIMEOUT; timeout++) {
        if (AT45DB321D_Flash_GetStatus() & 0x80)
            return TinyCLR_Result::Success;

        g_AT45DB321D_Flash_TimeProvider->WaitMicroseconds(g_AT45DB321D_Flash_TimeProvider, 1000);
    }

    g_AT45DB321D_Flash_SpiProvider->Release(g_AT45DB321D_Flash_SpiProvider);

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT45DB321D_Flash_Acquire(const TinyCLR_Spi_Provider* spiProvider, const TinyCLR_NativeTime_Provider* timeProvider, uint32_t chipSelectLine, bool& supportXIP) {

    size_t writeLength;
    size_t readLength;

    int32_t timeout;

    supportXIP = false;

    g_AT45DB321D_Flash_TimeProvider = timeProvider;
    g_AT45DB321D_Flash_SpiProvider = spiProvider;

    g_AT45DB321D_Flash_SpiChipSelectLine = chipSelectLine;

    g_AT45DB321D_Flash_SpiProvider->Acquire(g_AT45DB321D_Flash_SpiProvider);

    g_AT45DB321D_Flash_SpiProvider->SetActiveSettings(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_SpiChipSelectLine, AT45DB321D_SPI_CLOCK_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    g_AT45DB321D_Flash_DataWriteBuffer[0] = AT45DB321D_FLASH_COMMAND_READID;
    g_AT45DB321D_Flash_DataWriteBuffer[1] = 0x00;
    g_AT45DB321D_Flash_DataWriteBuffer[2] = 0x00;
    g_AT45DB321D_Flash_DataWriteBuffer[3] = 0x00;
    g_AT45DB321D_Flash_DataWriteBuffer[4] = 0x00;

    writeLength = 5;
    readLength = 5;

    g_AT45DB321D_Flash_SpiProvider->TransferFullDuplex(g_AT45DB321D_Flash_SpiProvider, g_AT45DB321D_Flash_DataWriteBuffer, writeLength, g_AT45DB321D_Flash_DataReadBuffer, readLength);

    if (AT45DB321D_FLASH_MANUFACTURER_CODE != g_AT45DB321D_Flash_DataReadBuffer[1])
        return TinyCLR_Result::InvalidOperation;

    if (AT45DB321D_FLASH_DEVICE_CODE != g_AT45DB321D_Flash_DataReadBuffer[2])
        return TinyCLR_Result::InvalidOperation;

    for (timeout = 0; timeout < AT45DB321D_FLASH_ACCESS_TIMEOUT; timeout++) {
        if (AT45DB321D_Flash_GetStatus() & 0x80)
            return TinyCLR_Result::Success;;

        g_AT45DB321D_Flash_TimeProvider->WaitMicroseconds(g_AT45DB321D_Flash_TimeProvider, 1000);
    }

    g_AT45DB321D_Flash_SpiProvider->Release(g_AT45DB321D_Flash_SpiProvider);

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT45DB321D_Flash_Release() {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_GetBytesPerSector(uint32_t address, int32_t& size) {
    size = 0;

    int32_t startRegion;
    uint32_t regions = sizeof(g_AT45DB321D_Flash_SectorAddress) / sizeof(g_AT45DB321D_Flash_SectorAddress[0]);
    uint32_t startAddress = address;

    for (startRegion = 0; startRegion < regions - 1; startRegion++)
        if (startAddress < g_AT45DB321D_Flash_SectorAddress[startRegion + 1])
            break;

    size = g_AT45DB321D_Flash_SectorSize[startRegion];

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_GetSectorMap(const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < AT45DB321D_FLASH_SECTOR_NUM; i++) {
        g_AT45DB321D_Flash_SectorAddress[i] = (AT45DB321D_FLASH_SECTOR_START + i) * AT45DB321D_FLASH_BLOCK_SIZE;
        g_AT45DB321D_Flash_SectorSize[i] = AT45DB321D_FLASH_BLOCK_SIZE;
    }

    addresses = g_AT45DB321D_Flash_SectorAddress;
    sizes = g_AT45DB321D_Flash_SectorSize;
    count = AT45DB321D_FLASH_SECTOR_NUM;

    return TinyCLR_Result::Success;
}


