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

#define DEPLOYMENT_SECTOR_START                 640
#define DEPLOYMENT_SECTOR_END                   1020
#define DEPLOYMENT_SECTOR_NUM                   (DEPLOYMENT_SECTOR_END - DEPLOYMENT_SECTOR_START + 1)

#define COMMAND_READID                          0x9F
#define COMMAND_READ_STATUS_REGISTER            0xD7
#define COMMAND_WRITE_BUFFER_1                  0x84
#define COMMAND_WRITE_BUFFER_2                  0x87
#define COMMAND_WRITE_BUFFER_1_TO_MEMORY        0x83
#define COMMAND_WRITE_BUFFER_2_TO_MEMORY        0x86
#define COMMAND_READ_FROM_MAIN_MEMORY_LEGACY    0xE8
#define COMMAND_READ_FROM_MAIN_MEMORY_DIRECT    0xD2
#define COMMAND_PAGE_ERASE                      0x81

#define COMMAND_READ_STATUS_REGISTER            0xD7
#define COMMAND_WRITE_BUFFER_1                  0x84
#define COMMAND_WRITE_BUFFER_2                  0x87
#define COMMAND_WRITE_BUFFER_1_TO_MEMORY        0x83
#define COMMAND_WRITE_BUFFER_2_TO_MEMORY        0x86
#define COMMAND_READ_FROM_MAIN_MEMORY_LEGACY    0xE8
#define COMMAND_READ_FROM_MAIN_MEMORY_DIRECT    0xD2
#define COMMAND_PAGE_ERASE                      0x81
#define COMMAND_BLOCK_ERASE                     0x50

#define WRITE_COMMAND_SIZE                      4
#define FLASH_ACCESS_TIMEOUT                    1000 // 1000 ms

#define FLASH_PAGE_SIZE                         528
#define FLASH_BLOCK_SIZE                        (528 * 8)

#define FLASH_MANUFACTURER_CODE                 0x1F
#define FLASH_DEVICE_CODE                       0x27 // For 4MB Chip


static uint32_t deploymentAddress[DEPLOYMENT_SECTOR_NUM];
static uint32_t deploymentSize[DEPLOYMENT_SECTOR_NUM];

uint8_t AT45DB321D_Flash_BufferRW[(FLASH_PAGE_SIZE * 1) + WRITE_COMMAND_SIZE];

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

struct AT45DB321D_Flash_Controller {
    uint8_t dataReadBuffer[FLASH_BLOCK_SIZE + 8];
    uint8_t dataWriteBuffer[FLASH_BLOCK_SIZE + 8];

    TinyCLR_Spi_Provider* spiProvider;
    TinyCLR_Time_Provider* timeProvider;
};

AT45DB321D_Flash_Controller g_AT45DB321D_Flash_Controller;

const TinyCLR_Api_Info* AT45DB321D_Deployment_GetApi() {
    deploymentProvider.Parent = &deploymentApi;
    deploymentProvider.Index = 0;
    deploymentProvider.Acquire = &AT45DB321D_Flash_Acquire;
    deploymentProvider.Release = &AT45DB321D_Flash_Release;
    deploymentProvider.Read = &AT45DB321D_Flash_Read;
    deploymentProvider.Write = &AT45DB321D_Flash_Write;
    deploymentProvider.EraseSector = &AT45DB321D_Flash_EraseBlock;
    deploymentProvider.IsSectorErased = &AT45DB321D_Flash_IsBlockErased;
    deploymentProvider.GetSectorMap = &AT45DB321D_Flash_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT45DB321D.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Count = 1;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
}

uint8_t __section("SectionForFlashOperations") AT45DB321D_Flash_GetStatus() {
    size_t writeLength;
    size_t readLength;

    g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_STATUS_REGISTER;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = 0x00;

    writeLength = 2;
    readLength = 2;

    g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

    return g_AT45DB321D_Flash_Controller.dataReadBuffer[1];
}

TinyCLR_Result __section("SectionForFlashOperations") AT45DB321D_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    int32_t block = length / FLASH_PAGE_SIZE;
    int32_t rest = length % FLASH_PAGE_SIZE;
    int32_t index = 0;
    size_t writeLength;
    size_t readLength;

    while (block > 0) {
        uint32_t pageNumber = (address % FLASH_PAGE_SIZE) | ((address / FLASH_PAGE_SIZE) << 10);

        g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_FROM_MAIN_MEMORY_LEGACY;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = pageNumber >> 16;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[2] = pageNumber >> 8;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[3] = pageNumber;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[4] = 0x00;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[5] = 0x00;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[6] = 0x00;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[7] = 0x00;

        writeLength = FLASH_PAGE_SIZE + 8;
        readLength = FLASH_PAGE_SIZE + 8;

        g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

        int32_t timeout;

        for (timeout = 0; timeout < FLASH_ACCESS_TIMEOUT; timeout++) {
            if (AT45DB321D_Flash_GetStatus() & 0x80)
                break;

            g_AT45DB321D_Flash_Controller.timeProvider->Delay(reinterpret_cast<const TinyCLR_Time_Provider*>(g_AT45DB321D_Flash_Controller.timeProvider), 1000);
        }

        memcpy(&buffer[index], &g_AT45DB321D_Flash_Controller.dataReadBuffer[8], FLASH_PAGE_SIZE);

        address += FLASH_PAGE_SIZE;
        index += FLASH_PAGE_SIZE;
        block--;
    }

    if (rest > 0) {
        uint32_t pageNumber = (address % FLASH_PAGE_SIZE) | ((address / FLASH_PAGE_SIZE) << 10);

        g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_FROM_MAIN_MEMORY_LEGACY;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = pageNumber >> 16;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[2] = pageNumber >> 8;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[3] = pageNumber;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[4] = 0x00;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[5] = 0x00;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[6] = 0x00;
        g_AT45DB321D_Flash_Controller.dataWriteBuffer[7] = 0x00;

        writeLength = rest + 8;
        readLength = rest + 8;

        g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

        int32_t timeout;

        for (timeout = 0; timeout < FLASH_ACCESS_TIMEOUT; timeout++) {
            if (AT45DB321D_Flash_GetStatus() & 0x80)
                break;

            g_AT45DB321D_Flash_Controller.timeProvider->Delay(reinterpret_cast<const TinyCLR_Time_Provider*>(g_AT45DB321D_Flash_Controller.timeProvider), 1000);
        }

        memcpy(&buffer[index], &g_AT45DB321D_Flash_Controller.dataReadBuffer[8], rest);

        address += rest;
        index += rest;
        block--;
    }

    return TinyCLR_Result::Success;
}

bool AT45DB321D_Flash_WriteSector(uint32_t pageNumber, uint8_t* dataBuffer) {
    size_t writeLength;
    size_t readLength;

    g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = COMMAND_WRITE_BUFFER_1;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = 0x00;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[2] = 0x00;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[3] = 0x00;

    memcpy(&g_AT45DB321D_Flash_Controller.dataWriteBuffer[4], dataBuffer, FLASH_PAGE_SIZE);

    writeLength = WRITE_COMMAND_SIZE + FLASH_PAGE_SIZE;
    readLength = WRITE_COMMAND_SIZE + FLASH_PAGE_SIZE;

    g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

    g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = 0x88;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = (pageNumber << 2) >> 8;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[2] = pageNumber << 2;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[3] = 0x00;

    writeLength = WRITE_COMMAND_SIZE;
    readLength = WRITE_COMMAND_SIZE;

    g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

    int32_t timeout;

    for (timeout = 0; timeout < FLASH_ACCESS_TIMEOUT; timeout++) {
        if (AT45DB321D_Flash_GetStatus() & 0x80)
            return true;

        g_AT45DB321D_Flash_Controller.timeProvider->Delay(reinterpret_cast<const TinyCLR_Time_Provider*>(g_AT45DB321D_Flash_Controller.timeProvider), 1000);
    }

    return false;
}

TinyCLR_Result __section("SectionForFlashOperations") AT45DB321D_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t pageNumber = address / FLASH_PAGE_SIZE;
    uint32_t pageOffset = address % FLASH_PAGE_SIZE;
    uint32_t currentIndex = 0;
    uint32_t remainingBytes = length;
    uint32_t beginningBytes = FLASH_PAGE_SIZE - pageOffset;
    uint32_t remainingBytesPageSegmentSize = FLASH_PAGE_SIZE - pageOffset;

    if (pageOffset) {
        memset(AT45DB321D_Flash_BufferRW, 0xFF, FLASH_PAGE_SIZE);

        if (length <= remainingBytesPageSegmentSize) {
            memcpy(&AT45DB321D_Flash_BufferRW[pageOffset], &buffer[currentIndex], length);

            remainingBytes -= length;
        }
        else {
            memcpy(&AT45DB321D_Flash_BufferRW[pageOffset], &buffer[currentIndex], beginningBytes);

            currentIndex += beginningBytes;
            remainingBytes -= beginningBytes;
        }

        AT45DB321D_Flash_WriteSector(pageNumber, AT45DB321D_Flash_BufferRW);

        pageNumber++;
    }

    if (!remainingBytes)
        TinyCLR_Result::Success;

    uint32_t sector = remainingBytes / FLASH_PAGE_SIZE;

    while (sector) {
        memset(AT45DB321D_Flash_BufferRW, 0xFF, FLASH_PAGE_SIZE);
        memcpy(AT45DB321D_Flash_BufferRW, &buffer[currentIndex], FLASH_PAGE_SIZE);

        AT45DB321D_Flash_WriteSector(pageNumber, AT45DB321D_Flash_BufferRW);

        currentIndex += FLASH_PAGE_SIZE;
        remainingBytes -= FLASH_PAGE_SIZE;
        sector--;
        pageNumber++;
    }

    if (!remainingBytes)
        TinyCLR_Result::Success;

    memset(AT45DB321D_Flash_BufferRW, 0xFF, FLASH_PAGE_SIZE);
    memcpy(AT45DB321D_Flash_BufferRW, &buffer[currentIndex], remainingBytes);

    AT45DB321D_Flash_WriteSector(pageNumber, AT45DB321D_Flash_BufferRW);

    TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") AT45DB321D_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);
    uint32_t startAddress = deploymentAddress[sector];

    int32_t block = FLASH_BLOCK_SIZE / FLASH_PAGE_SIZE;
    int32_t rest = FLASH_BLOCK_SIZE % FLASH_PAGE_SIZE;

    erased = true;

    while (block > 0) {
        AT45DB321D_Flash_Read(self, startAddress, FLASH_PAGE_SIZE, AT45DB321D_Flash_BufferRW);

        for (auto i = 0; i < FLASH_PAGE_SIZE; i++) {
            if (AT45DB321D_Flash_BufferRW[i] != 0xFF) {
                erased = false;

                return TinyCLR_Result::Success;
            }
        }

        block--;

        startAddress += FLASH_PAGE_SIZE;
    }

    if (rest > 0) {
        AT45DB321D_Flash_Read(self, startAddress, rest, AT45DB321D_Flash_BufferRW);

        for (auto i = 0; i < rest; i++) {
            if (AT45DB321D_Flash_BufferRW[i] != 0xFF) {
                erased = false;

                return TinyCLR_Result::Success;
            }
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") AT45DB321D_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    size_t writeLength;
    size_t readLength;

    uint32_t blockNumber = deploymentAddress[sector] / (FLASH_BLOCK_SIZE);

    g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = COMMAND_BLOCK_ERASE;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = (blockNumber << 3u) >> 6u;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[2] = ((uint8_t)((blockNumber << 3u) & 0x3F) << 2u) + ((uint8_t)(0 >> 8u));
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[3] = 0x00;

    writeLength = WRITE_COMMAND_SIZE;
    readLength = WRITE_COMMAND_SIZE;


    g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

    int32_t timeout;

    for (timeout = 0; timeout < FLASH_ACCESS_TIMEOUT; timeout++) {
        if (AT45DB321D_Flash_GetStatus() & 0x80)
            return TinyCLR_Result::Success;

        g_AT45DB321D_Flash_Controller.timeProvider->Delay(reinterpret_cast<const TinyCLR_Time_Provider*>(g_AT45DB321D_Flash_Controller.timeProvider), 1000);
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT45DB321D_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportXIP) {
    const TinyCLR_Api_Info* spiApi = CONCAT(DEVICE_TARGET, _Spi_GetApi)();;
    TinyCLR_Spi_Provider** spiProvider = (TinyCLR_Spi_Provider**)spiApi->Implementation;

    const TinyCLR_Api_Info* timeApi = CONCAT(DEVICE_TARGET, _Time_GetApi)();;
    TinyCLR_Time_Provider** timeProvider = (TinyCLR_Time_Provider**)timeApi->Implementation;

    size_t writeLength;
    size_t readLength;

    int32_t timeout;

    supportXIP = false;

    g_AT45DB321D_Flash_Controller.timeProvider = reinterpret_cast<TinyCLR_Time_Provider*>(&timeProvider[0]);

    g_AT45DB321D_Flash_Controller.spiProvider = (spiApi->Count > 1) ? spiProvider[AT45DB321D_SPI_MODULE] : reinterpret_cast<TinyCLR_Spi_Provider*>(&spiProvider[0]);

    g_AT45DB321D_Flash_Controller.spiProvider->Acquire(g_AT45DB321D_Flash_Controller.spiProvider);

    g_AT45DB321D_Flash_Controller.spiProvider->SetActiveSettings(g_AT45DB321D_Flash_Controller.spiProvider, AT45DB321D_SPI_CS, AT45DB321D_SPI_CLOCK_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    AT45DB321D_Flash_Reset(self);

    g_AT45DB321D_Flash_Controller.dataWriteBuffer[0] = COMMAND_READID;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[1] = 0x00;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[2] = 0x00;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[3] = 0x00;
    g_AT45DB321D_Flash_Controller.dataWriteBuffer[4] = 0x00;

    writeLength = 5;
    readLength = 5;

    g_AT45DB321D_Flash_Controller.spiProvider->TransferFullDuplex(g_AT45DB321D_Flash_Controller.spiProvider, g_AT45DB321D_Flash_Controller.dataWriteBuffer, writeLength, g_AT45DB321D_Flash_Controller.dataReadBuffer, readLength);

    if (FLASH_MANUFACTURER_CODE != g_AT45DB321D_Flash_Controller.dataReadBuffer[1])
        return TinyCLR_Result::InvalidOperation;

    if (FLASH_DEVICE_CODE != g_AT45DB321D_Flash_Controller.dataReadBuffer[2])
        return TinyCLR_Result::InvalidOperation;

    for (timeout = 0; timeout < FLASH_ACCESS_TIMEOUT; timeout++) {
        if (AT45DB321D_Flash_GetStatus() & 0x80)
            return TinyCLR_Result::Success;;

        g_AT45DB321D_Flash_Controller.timeProvider->Delay(reinterpret_cast<const TinyCLR_Time_Provider*>(g_AT45DB321D_Flash_Controller.timeProvider), 1000);
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT45DB321D_Flash_Release(const TinyCLR_Deployment_Provider* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
    size = 0;

    int32_t startRegion;
    uint32_t regions = sizeof(deploymentAddress) / sizeof(deploymentAddress[0]);
    uint32_t startAddress = address;

    for (startRegion = 0; startRegion < regions - 1; startRegion++)
        if (startAddress < deploymentAddress[startRegion + 1])
            break;

    size = deploymentSize[startRegion];

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {

    addresses = deploymentAddress;
    sizes = deploymentSize;
    count = DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT45DB321D_Flash_Reset(const TinyCLR_Deployment_Provider* self) {
    for (auto i = 0; i < DEPLOYMENT_SECTOR_NUM; i++) {
        deploymentAddress[i] = (DEPLOYMENT_SECTOR_START + i) * FLASH_BLOCK_SIZE;
        deploymentSize[i] = FLASH_BLOCK_SIZE;
    }

    return TinyCLR_Result::Success;
}

