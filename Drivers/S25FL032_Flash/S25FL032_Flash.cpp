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

TinyCLR_Spi_Provider* S25FL032_Flash_SpiProvider;

uint8_t S25FL032_Flash_DataReadBuffer[FLASH_SECTOR_SIZE + 4];
uint8_t S25FL032_Flash_DataWriteBuffer[FLASH_SECTOR_SIZE + 4];

uint32_t S25FL032_Flash_SectorAddress[DEPLOYMENT_SECTOR_NUM];
uint32_t S25FL032_Flash_SectorSize[DEPLOYMENT_SECTOR_NUM];

bool S25FL032_Flash_WriteEnable() {
    S25FL032_Flash_DataWriteBuffer[0] = COMMAND_WRITE_ENABLE;
    S25FL032_Flash_DataWriteBuffer[1] = 0x00;

    size_t writeLength = 1;
    size_t readLength = 0;

    S25FL032_Flash_SpiProvider->TransferFullDuplex(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength, S25FL032_Flash_DataReadBuffer, readLength);

    S25FL032_Flash_DataWriteBuffer[0] = COMMAND_READ_STATUS_REGISTER;
    S25FL032_Flash_DataWriteBuffer[1] = 0x00;

    writeLength = 2;
    readLength = 2;

    S25FL032_Flash_SpiProvider->TransferFullDuplex(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength, S25FL032_Flash_DataReadBuffer, readLength);

    if ((S25FL032_Flash_DataReadBuffer[1] & 0x2) != 0)
        return true;
    else
        return false;
}

bool S25FL032_Flash_WriteInProgress() {
    S25FL032_Flash_DataWriteBuffer[0] = COMMAND_READ_STATUS_REGISTER;
    S25FL032_Flash_DataWriteBuffer[1] = 0x00;

    size_t writeLength = 2;
    size_t readLength = 2;

    S25FL032_Flash_SpiProvider->TransferFullDuplex(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength, S25FL032_Flash_DataReadBuffer, readLength);

    if ((S25FL032_Flash_DataReadBuffer[1] & 0x1) != 0)
        return true;
    else
        return false;
}

TinyCLR_Result S25FL032_Flash_Read(uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    S25FL032_Flash_SpiProvider->Acquire(S25FL032_Flash_SpiProvider);

    S25FL032_Flash_SpiProvider->SetActiveSettings(S25FL032_Flash_SpiProvider, SPI_CS, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (S25FL032_Flash_WriteInProgress() == true);

    int32_t block = length / FLASH_SECTOR_SIZE;
    int32_t rest = length % FLASH_SECTOR_SIZE;
    int32_t index = 0;
    size_t writeLength = FLASH_SECTOR_SIZE + 4;
    size_t readLength = FLASH_SECTOR_SIZE + 4;

    address -= FLASH_BASE_ADDRESS;

    while (block > 0) {
        S25FL032_Flash_DataWriteBuffer[0] = COMMAND_READ_DATA;
        S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)((address) >> 16);
        S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)((address) >> 8);
        S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = FLASH_SECTOR_SIZE + 4;
        readLength = FLASH_SECTOR_SIZE + 4;

        // start to read
        S25FL032_Flash_SpiProvider->TransferFullDuplex(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength, S25FL032_Flash_DataReadBuffer, readLength);

        // copy to buffer
        memcpy(&buffer[index], &S25FL032_Flash_DataReadBuffer[4], FLASH_SECTOR_SIZE);

        address += FLASH_SECTOR_SIZE;
        index += FLASH_SECTOR_SIZE;
        block--;

    }

    if (rest > 0) {
        S25FL032_Flash_DataWriteBuffer[0] = COMMAND_READ_DATA;
        S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)((address) >> 16);
        S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)((address) >> 8);
        S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = rest + 4;
        readLength = rest + 4;

        // start to read
        S25FL032_Flash_SpiProvider->TransferFullDuplex(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength, S25FL032_Flash_DataReadBuffer, readLength);

        // copy to buffer
        memcpy(&buffer[index], &S25FL032_Flash_DataReadBuffer[4], rest);

        address += rest;
        index += rest;
        rest = 0;
    }

    S25FL032_Flash_SpiProvider->Release(S25FL032_Flash_SpiProvider);

    return index == length ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result S25FL032_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer) {

    uint32_t addr = byteAddress - FLASH_BASE_ADDRESS;
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

        S25FL032_Flash_DataWriteBuffer[0] = COMMAND_PAGE_PROGRAMMING; //0x2
        S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)(addr >> 16);
        S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)(addr >> 8);
        S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)(addr >> 0);

        size_t actualWrite = block_size + 4;

        memcpy(&S25FL032_Flash_DataWriteBuffer[4], pointerToWriteBuffer + source_index, block_size);

        // Write cmd to Write
        S25FL032_Flash_SpiProvider->Write(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, actualWrite);

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

    S25FL032_Flash_SpiProvider->Acquire(S25FL032_Flash_SpiProvider);

    S25FL032_Flash_SpiProvider->SetActiveSettings(S25FL032_Flash_SpiProvider, SPI_CS, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    TinyCLR_Result result = S25FL032_Flash_PageProgram(address, length, buffer);

    S25FL032_Flash_SpiProvider->Release(S25FL032_Flash_SpiProvider);

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

    uint32_t address = S25FL032_Flash_SectorAddress[sector];

    S25FL032_Flash_Read(address, FLASH_SECTOR_SIZE, S25FL032_Flash_DataReadBuffer);

    uint32_t *ptr = (uint32_t*)& S25FL032_Flash_DataReadBuffer;

    erased = CompareArrayValueToValue(ptr, 0xFFFFFFFF, FLASH_SECTOR_SIZE / 4);

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_EraseBlock(uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    S25FL032_Flash_SpiProvider->Acquire(S25FL032_Flash_SpiProvider);

    S25FL032_Flash_SpiProvider->SetActiveSettings(S25FL032_Flash_SpiProvider, SPI_CS, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    while (S25FL032_Flash_WriteEnable() == false);

    uint32_t address = S25FL032_Flash_SectorAddress[sector];

    S25FL032_Flash_DataWriteBuffer[0] = COMMAND_ERASE_SECTOR_64K;
    S25FL032_Flash_DataWriteBuffer[1] = (uint8_t)((address - FLASH_BASE_ADDRESS) >> 16);
    S25FL032_Flash_DataWriteBuffer[2] = (uint8_t)((address - FLASH_BASE_ADDRESS) >> 8);
    S25FL032_Flash_DataWriteBuffer[3] = (uint8_t)((address - FLASH_BASE_ADDRESS) >> 0);

    size_t writeLength = 4;

    S25FL032_Flash_SpiProvider->Write(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength);

    while (S25FL032_Flash_WriteInProgress() == true);

    S25FL032_Flash_SpiProvider->Release(S25FL032_Flash_SpiProvider);

    TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_Acquire(bool& supportXIP) {
    const TinyCLR_Api_Info* spiApi = CONCAT(DEVICE_TARGET, _Spi_GetApi)();
    TinyCLR_Spi_Provider** spiProvider = (TinyCLR_Spi_Provider**)spiApi->Implementation;

    supportXIP = false;

    S25FL032_Flash_DataWriteBuffer[0] = COMMAND_READID;
    S25FL032_Flash_DataWriteBuffer[1] = 0x00;
    S25FL032_Flash_DataWriteBuffer[2] = 0x00;
    S25FL032_Flash_DataWriteBuffer[3] = 0x00;

    size_t writeLength = 4;
    size_t readLength = 4;

    S25FL032_Flash_SpiProvider = spiProvider[SPI_MODULE];

    S25FL032_Flash_SpiProvider->Acquire(S25FL032_Flash_SpiProvider);

    S25FL032_Flash_SpiProvider->SetActiveSettings(S25FL032_Flash_SpiProvider, SPI_CS, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    S25FL032_Flash_SpiProvider->TransferFullDuplex(S25FL032_Flash_SpiProvider, S25FL032_Flash_DataWriteBuffer, writeLength, S25FL032_Flash_DataReadBuffer, readLength);

    if (S25F_FLASH_MANUFACTURER_CODE != S25FL032_Flash_DataReadBuffer[1] && MX25L_FLASH_MANUFACTURER_CODE != S25FL032_Flash_DataReadBuffer[1]) {

        return TinyCLR_Result::WrongType;
    }

    if ((S25F_FLASH_DEVICE_CODE_0 != S25FL032_Flash_DataReadBuffer[2] || S25F_FLASH_DEVICE_CODE_1 != S25FL032_Flash_DataReadBuffer[3]) && (MX25L_FLASH_DEVICE_CODE_0 != S25FL032_Flash_DataReadBuffer[2] || MX25L_FLASH_DEVICE_CODE_1 != S25FL032_Flash_DataReadBuffer[3])) {

        return TinyCLR_Result::WrongType;
    }

    S25FL032_Flash_SpiProvider->Release(S25FL032_Flash_SpiProvider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_Release() {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_GetBytesPerSector(uint32_t address, int32_t& size) {
    size = FLASH_SECTOR_SIZE;

    return TinyCLR_Result::Success;
}

TinyCLR_Result S25FL032_Flash_GetSectorMap(const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < DEPLOYMENT_SECTOR_NUM; i++) {
        S25FL032_Flash_SectorAddress[i] = ((DEPLOYMENT_SECTOR_START + i) * FLASH_SECTOR_SIZE) | FLASH_BASE_ADDRESS;
        S25FL032_Flash_SectorSize[i] = FLASH_SECTOR_SIZE;
    }

    addresses = S25FL032_Flash_SectorAddress;
    sizes = S25FL032_Flash_SectorSize;
    count = DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

