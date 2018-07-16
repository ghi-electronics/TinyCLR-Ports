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

#include <string.h>

#include "S25FL032_Flash.h"
#include <Device.h>

#define SPI_CLOCK_RATE_HZ 20000000

const TinyCLR_Spi_Controller* g_S25FL032_Flash_SpiProvider;
int32_t g_S25FL032_Flash_SpiControllerId;
uint32_t g_S25FL032_Flash_SpiChipSelectLine;

uint8_t g_S25FL032_Flash_DataReadBuffer[S25FL032_FLASH_SECTOR_SIZE + 4];
uint8_t g_S25FL032_Flash_DataWriteBuffer[S25FL032_FLASH_SECTOR_SIZE + 4];

uint32_t g_S25FL032_Flash_SectorAddress[S25FL032_FLASH_SECTOR_NUM];
uint32_t g_S25FL032_Flash_SectorSize[S25FL032_FLASH_SECTOR_NUM];

bool S25FL032_Flash_WriteEnable() {
    g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_WRITE_ENABLE;
    g_S25FL032_Flash_DataWriteBuffer[1] = 0x00;

    size_t writeLength = 1;
    size_t readLength = 0;

    g_S25FL032_Flash_SpiProvider->TransferFullDuplex(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength, g_S25FL032_Flash_DataReadBuffer, readLength);

    g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_STATUS_REGISTER;
    g_S25FL032_Flash_DataWriteBuffer[1] = 0x00;

    writeLength = 2;
    readLength = 2;

    g_S25FL032_Flash_SpiProvider->TransferFullDuplex(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength, g_S25FL032_Flash_DataReadBuffer, readLength);

    if ((g_S25FL032_Flash_DataReadBuffer[1] & 0x2) != 0)
        return true;
    else
        return false;
}

bool S25FL032_Flash_WriteInProgress() {
    g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_STATUS_REGISTER;
    g_S25FL032_Flash_DataWriteBuffer[1] = 0x00;

    size_t writeLength = 2;
    size_t readLength = 2;

    g_S25FL032_Flash_SpiProvider->TransferFullDuplex(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength, g_S25FL032_Flash_DataReadBuffer, readLength);

    if ((g_S25FL032_Flash_DataReadBuffer[1] & 0x1) != 0)
        return true;
    else
        return false;
}

TinyCLR_Result S25FL032_Flash_Read(uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    g_S25FL032_Flash_SpiProvider->Acquire(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    g_S25FL032_Flash_SpiProvider->SetActiveSettings(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_SpiChipSelectLine, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (S25FL032_Flash_WriteInProgress() == true);

    int32_t block = length / S25FL032_FLASH_SECTOR_SIZE;
    int32_t rest = length % S25FL032_FLASH_SECTOR_SIZE;
    int32_t index = 0;
    size_t writeLength = S25FL032_FLASH_SECTOR_SIZE + 4;
    size_t readLength = S25FL032_FLASH_SECTOR_SIZE + 4;

    while (block > 0) {
        g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_DATA;
        g_S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)((address) >> 16);
        g_S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)((address) >> 8);
        g_S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = S25FL032_FLASH_SECTOR_SIZE + 4;
        readLength = S25FL032_FLASH_SECTOR_SIZE + 4;

        // start to read
        g_S25FL032_Flash_SpiProvider->TransferFullDuplex(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength, g_S25FL032_Flash_DataReadBuffer, readLength);

        // copy to buffer
        memcpy(&buffer[index], &g_S25FL032_Flash_DataReadBuffer[4], S25FL032_FLASH_SECTOR_SIZE);

        address += S25FL032_FLASH_SECTOR_SIZE;
        index += S25FL032_FLASH_SECTOR_SIZE;
        block--;

    }

    if (rest > 0) {
        g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_DATA;
        g_S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)((address) >> 16);
        g_S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)((address) >> 8);
        g_S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = rest + 4;
        readLength = rest + 4;

        // start to read
        g_S25FL032_Flash_SpiProvider->TransferFullDuplex(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength, g_S25FL032_Flash_DataReadBuffer, readLength);

        // copy to buffer
        memcpy(&buffer[index], &g_S25FL032_Flash_DataReadBuffer[4], rest);

        address += rest;
        index += rest;
        rest = 0;
    }

    g_S25FL032_Flash_SpiProvider->Release(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    return index == length ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result S25FL032_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer) {

    uint32_t addr = byteAddress;
    uint32_t addr_misalignment = addr % ALIGNMENT_WINDOW;
    uint32_t extended_size = NumberOfBytesToWrite + addr_misalignment;
    uint32_t block_cnt = extended_size / ALIGNMENT_WINDOW + (extended_size % ALIGNMENT_WINDOW > 0 ? 1 : 0);
    uint32_t block_size;
    uint32_t source_index = 0;

    if (extended_size > ALIGNMENT_WINDOW)
        block_size = ALIGNMENT_WINDOW - addr_misalignment;
    else
        block_size = extended_size - addr_misalignment;

    while (block_cnt > 0) {
        while (S25FL032_Flash_WriteEnable() == false);

        g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_PAGE_PROGRAMMING; //0x2
        g_S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)(addr >> 16);
        g_S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)(addr >> 8);
        g_S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)(addr >> 0);

        size_t actualWrite = block_size + 4;

        memcpy(&g_S25FL032_Flash_DataWriteBuffer[4], pointerToWriteBuffer + source_index, block_size);

        // Write cmd to Write
        g_S25FL032_Flash_SpiProvider->Write(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, actualWrite);

        while (S25FL032_Flash_WriteInProgress() == true);

        //-------------Next Round----------------//
        block_cnt--;
        source_index += block_size;
        addr += block_size;

        if ((NumberOfBytesToWrite - source_index) > ALIGNMENT_WINDOW)
            block_size = ALIGNMENT_WINDOW;
        else
            block_size = NumberOfBytesToWrite - source_index;
    }

    if ((NumberOfBytesToWrite - source_index) == 0)
        return TinyCLR_Result::Success;
    else
        return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result S25FL032_Flash_Write(uint32_t address, size_t length, const uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    g_S25FL032_Flash_SpiProvider->Acquire(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    g_S25FL032_Flash_SpiProvider->SetActiveSettings(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_SpiChipSelectLine, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    TinyCLR_Result result = S25FL032_Flash_PageProgram(address, length, buffer);

    g_S25FL032_Flash_SpiProvider->Release(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    return result;
}

bool __section("SectionForCodeReadOnlyRAM") CompareArrayValueToValue(uint32_t* ptr, uint32_t value, uint32_t len) {
    for (auto x = 0; x < len; x++) {
        if (*ptr != value) {
            return false;
        }

        ptr++;
    }

    return true;
}

TinyCLR_Result S25FL032_Flash_IsBlockErased(uint32_t sector, bool &erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t address = g_S25FL032_Flash_SectorAddress[sector];

    S25FL032_Flash_Read(address, S25FL032_FLASH_SECTOR_SIZE, g_S25FL032_Flash_DataReadBuffer);

    uint32_t *ptr = (uint32_t*)& g_S25FL032_Flash_DataReadBuffer;

    erased = CompareArrayValueToValue(ptr, 0xFFFFFFFF, S25FL032_FLASH_SECTOR_SIZE / 4);

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_EraseBlock(uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    g_S25FL032_Flash_SpiProvider->Acquire(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    g_S25FL032_Flash_SpiProvider->SetActiveSettings(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_SpiChipSelectLine, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (S25FL032_Flash_WriteEnable() == false);

    uint32_t address = g_S25FL032_Flash_SectorAddress[sector];

    g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_ERASE_SECTOR_64K;
    g_S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)((address) >> 16);
    g_S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)((address) >> 8);
    g_S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)((address) >> 0);

    size_t writeLength = 4;

    g_S25FL032_Flash_SpiProvider->Write(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength);

    while (S25FL032_Flash_WriteInProgress() == true);

    g_S25FL032_Flash_SpiProvider->Release(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_Acquire(const TinyCLR_Spi_Controller* spiProvider, int32_t controller, uint32_t chipSelectLine, bool& supportXIP) {
    supportXIP = false;

    g_S25FL032_Flash_SpiControllerId = controller;
    g_S25FL032_Flash_DataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READID;
    g_S25FL032_Flash_DataWriteBuffer[1] = 0x00;
    g_S25FL032_Flash_DataWriteBuffer[2] = 0x00;
    g_S25FL032_Flash_DataWriteBuffer[3] = 0x00;

    size_t writeLength = 4;
    size_t readLength = 4;

    g_S25FL032_Flash_SpiProvider = spiProvider;
    g_S25FL032_Flash_SpiChipSelectLine = chipSelectLine;

    g_S25FL032_Flash_SpiProvider->Acquire(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    g_S25FL032_Flash_SpiProvider->SetActiveSettings(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_SpiChipSelectLine, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    g_S25FL032_Flash_SpiProvider->TransferFullDuplex(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId, g_S25FL032_Flash_DataWriteBuffer, writeLength, g_S25FL032_Flash_DataReadBuffer, readLength);

    if (S25F_FLASH_MANUFACTURER_CODE != g_S25FL032_Flash_DataReadBuffer[1] && MX25L_FLASH_MANUFACTURER_CODE != g_S25FL032_Flash_DataReadBuffer[1]) {

        return TinyCLR_Result::WrongType;
    }

    if ((S25F_FLASH_DEVICE_CODE_0 != g_S25FL032_Flash_DataReadBuffer[2] || S25F_FLASH_DEVICE_CODE_1 != g_S25FL032_Flash_DataReadBuffer[3]) && (MX25L_FLASH_DEVICE_CODE_0 != g_S25FL032_Flash_DataReadBuffer[2] || MX25L_FLASH_DEVICE_CODE_1 != g_S25FL032_Flash_DataReadBuffer[3])) {

        return TinyCLR_Result::WrongType;
    }

    g_S25FL032_Flash_SpiProvider->Release(g_S25FL032_Flash_SpiProvider, g_S25FL032_Flash_SpiControllerId);

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_Release() {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_GetBytesPerSector(uint32_t address, int32_t& size) {
    size = S25FL032_FLASH_SECTOR_SIZE;

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_GetSectorMap(const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < S25FL032_FLASH_SECTOR_NUM; i++) {
        g_S25FL032_Flash_SectorAddress[i] = (i * S25FL032_FLASH_SECTOR_SIZE);
        g_S25FL032_Flash_SectorSize[i] = S25FL032_FLASH_SECTOR_SIZE;
    }

    addresses = g_S25FL032_Flash_SectorAddress;
    sizes = g_S25FL032_Flash_SectorSize;
    count = S25FL032_FLASH_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

