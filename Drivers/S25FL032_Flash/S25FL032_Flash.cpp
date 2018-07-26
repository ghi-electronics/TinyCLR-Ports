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

const TinyCLR_Spi_Controller* s25fl032FlashSpiProvider;
static uint32_t s25fl032FlashSpiChipSelectLine;

static uint8_t s25fl032FlashDataReadBuffer[S25FL032_FLASH_SECTOR_SIZE + 4];
static uint8_t s25fl032FlashDataWriteBuffer[S25FL032_FLASH_SECTOR_SIZE + 4];

static uint64_t s25fl032FlashSectorAddress[S25FL032_FLASH_SECTOR_NUM];
static size_t s25fl032FlashSectorSize[S25FL032_FLASH_SECTOR_NUM];

bool S25FL032_Flash_WriteEnable() {
    s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_WRITE_ENABLE;
    s25fl032FlashDataWriteBuffer[1] = 0x00;

    size_t writeLength = 1;
    size_t readLength = 0;

    s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, s25fl032FlashDataReadBuffer, readLength, false);

    s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_STATUS_REGISTER;
    s25fl032FlashDataWriteBuffer[1] = 0x00;

    writeLength = 2;
    readLength = 2;

    s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, s25fl032FlashDataReadBuffer, readLength, false);

    if ((s25fl032FlashDataReadBuffer[1] & 0x2) != 0)
        return true;
    else
        return false;
}

bool S25FL032_Flash_WriteInProgress() {
    s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_STATUS_REGISTER;
    s25fl032FlashDataWriteBuffer[1] = 0x00;

    size_t writeLength = 2;
    size_t readLength = 2;

    s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, s25fl032FlashDataReadBuffer, readLength, false);

    if ((s25fl032FlashDataReadBuffer[1] & 0x1) != 0)
        return true;
    else
        return false;
}

TinyCLR_Result S25FL032_Flash_Read(uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    s25fl032FlashSpiProvider->Acquire(s25fl032FlashSpiProvider);

    s25fl032FlashSpiProvider->SetActiveSettings(s25fl032FlashSpiProvider, s25fl032FlashSpiChipSelectLine, false,SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (S25FL032_Flash_WriteInProgress() == true);

    int32_t block = length / S25FL032_FLASH_SECTOR_SIZE;
    int32_t rest = length % S25FL032_FLASH_SECTOR_SIZE;
    int32_t index = 0;
    size_t writeLength = S25FL032_FLASH_SECTOR_SIZE + 4;
    size_t readLength = S25FL032_FLASH_SECTOR_SIZE + 4;

    while (block > 0) {
        s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_DATA;
        s25fl032FlashDataWriteBuffer[1] = (uint8_t)((address) >> 16);
        s25fl032FlashDataWriteBuffer[2] = (uint8_t)((address) >> 8);
        s25fl032FlashDataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = S25FL032_FLASH_SECTOR_SIZE + 4;
        readLength = S25FL032_FLASH_SECTOR_SIZE + 4;

        // start to read
        s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, s25fl032FlashDataReadBuffer, readLength, false);

        // copy to buffer
        memcpy(&buffer[index], &s25fl032FlashDataReadBuffer[4], S25FL032_FLASH_SECTOR_SIZE);

        address += S25FL032_FLASH_SECTOR_SIZE;
        index += S25FL032_FLASH_SECTOR_SIZE;
        block--;

    }

    if (rest > 0) {
        s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READ_DATA;
        s25fl032FlashDataWriteBuffer[1] = (uint8_t)((address) >> 16);
        s25fl032FlashDataWriteBuffer[2] = (uint8_t)((address) >> 8);
        s25fl032FlashDataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = rest + 4;
        readLength = rest + 4;

        // start to read
        s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, s25fl032FlashDataReadBuffer, readLength, false);

        // copy to buffer
        memcpy(&buffer[index], &s25fl032FlashDataReadBuffer[4], rest);

        address += rest;
        index += rest;
        rest = 0;
    }

    s25fl032FlashSpiProvider->Release(s25fl032FlashSpiProvider);

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

        s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_PAGE_PROGRAMMING; //0x2
        s25fl032FlashDataWriteBuffer[1] = (uint8_t)(addr >> 16);
        s25fl032FlashDataWriteBuffer[2] = (uint8_t)(addr >> 8);
        s25fl032FlashDataWriteBuffer[3] = (uint8_t)(addr >> 0);

        size_t actualWrite = block_size + 4;
        size_t actualRead = 0;

        memcpy(&s25fl032FlashDataWriteBuffer[4], pointerToWriteBuffer + source_index, block_size);

        // Write cmd to Write
        s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, actualWrite, nullptr, actualRead, false);

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

    s25fl032FlashSpiProvider->Acquire(s25fl032FlashSpiProvider);

    s25fl032FlashSpiProvider->SetActiveSettings(s25fl032FlashSpiProvider, s25fl032FlashSpiChipSelectLine, false,SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    TinyCLR_Result result = S25FL032_Flash_PageProgram(address, length, buffer);

    s25fl032FlashSpiProvider->Release(s25fl032FlashSpiProvider);

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

    uint32_t address = s25fl032FlashSectorAddress[sector];

    S25FL032_Flash_Read(address, S25FL032_FLASH_SECTOR_SIZE, s25fl032FlashDataReadBuffer);

    uint32_t *ptr = (uint32_t*)& s25fl032FlashDataReadBuffer;

    erased = CompareArrayValueToValue(ptr, 0xFFFFFFFF, S25FL032_FLASH_SECTOR_SIZE / 4);

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_EraseBlock(uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    s25fl032FlashSpiProvider->Acquire(s25fl032FlashSpiProvider);

    s25fl032FlashSpiProvider->SetActiveSettings(s25fl032FlashSpiProvider, s25fl032FlashSpiChipSelectLine, false,SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (S25FL032_Flash_WriteEnable() == false);

    uint32_t address = s25fl032FlashSectorAddress[sector];

    s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_ERASE_SECTOR_64K;
    s25fl032FlashDataWriteBuffer[1] = (uint8_t)((address) >> 16);
    s25fl032FlashDataWriteBuffer[2] = (uint8_t)((address) >> 8);
    s25fl032FlashDataWriteBuffer[3] = (uint8_t)((address) >> 0);

    size_t writeLength = 4;
    size_t readLength = 0;

    s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, nullptr, readLength, false);

    while (S25FL032_Flash_WriteInProgress() == true);

    s25fl032FlashSpiProvider->Release(s25fl032FlashSpiProvider);

    TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_Acquire(const TinyCLR_Spi_Controller* spiProvider, uint32_t chipSelectLine) {
    auto controller = *reinterpret_cast<int32_t*>(spiProvider->ApiInfo->State);

    s25fl032FlashDataWriteBuffer[0] = S25FL032_FLASH_COMMAND_READID;
    s25fl032FlashDataWriteBuffer[1] = 0x00;
    s25fl032FlashDataWriteBuffer[2] = 0x00;
    s25fl032FlashDataWriteBuffer[3] = 0x00;

    size_t writeLength = 4;
    size_t readLength = 4;

    s25fl032FlashSpiProvider = spiProvider;
    s25fl032FlashSpiChipSelectLine = chipSelectLine;

    s25fl032FlashSpiProvider->Acquire(s25fl032FlashSpiProvider);

    s25fl032FlashSpiProvider->SetActiveSettings(s25fl032FlashSpiProvider, s25fl032FlashSpiChipSelectLine, false,SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    s25fl032FlashSpiProvider->WriteRead(s25fl032FlashSpiProvider, s25fl032FlashDataWriteBuffer, writeLength, s25fl032FlashDataReadBuffer, readLength, false);

    if (S25F_FLASH_MANUFACTURER_CODE != s25fl032FlashDataReadBuffer[1] && MX25L_FLASH_MANUFACTURER_CODE != s25fl032FlashDataReadBuffer[1]) {

        return TinyCLR_Result::WrongType;
    }

    if ((S25F_FLASH_DEVICE_CODE_0 != s25fl032FlashDataReadBuffer[2] || S25F_FLASH_DEVICE_CODE_1 != s25fl032FlashDataReadBuffer[3]) && (MX25L_FLASH_DEVICE_CODE_0 != s25fl032FlashDataReadBuffer[2] || MX25L_FLASH_DEVICE_CODE_1 != s25fl032FlashDataReadBuffer[3])) {

        return TinyCLR_Result::WrongType;
    }

    s25fl032FlashSpiProvider->Release(s25fl032FlashSpiProvider);

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

TinyCLR_Result S25FL032_Flash_GetSectorMap(const uint64_t*& addresses, const size_t*& sizes, size_t& count) {
    for (auto i = 0; i < S25FL032_FLASH_SECTOR_NUM; i++) {
        s25fl032FlashSectorAddress[i] = (i * S25FL032_FLASH_SECTOR_SIZE);
        s25fl032FlashSectorSize[i] = S25FL032_FLASH_SECTOR_SIZE;
    }

    addresses = s25fl032FlashSectorAddress;
    sizes = s25fl032FlashSectorSize;
    count = S25FL032_FLASH_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

