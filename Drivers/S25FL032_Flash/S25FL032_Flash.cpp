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

#include <stdio.h>
#include <string.h>

#include <Device.h>

#define FLASH_BASE_ADDRESS                      0xF0000000 // This MUST be zero or you need to substract the base address before doing any process in the the flash draiver from the address.
#define FLASH_SECTOR_SIZE                       (64*1024) //528 //(528*2) // 512 for FEZ Hydra || 1024 on original SAM

//Command
#define COMMAND_SIZE							4
#define COMMAND_READID							0x9F
#define COMMAND_READ_STATUS_REGISTER			0x05
#define COMMAND_READ_DATA						0x03
#define COMMAND_WRITE_ENABLE					0x06
#define COMMAND_PAGE_PROGRAMMING				0x02
#define COMMAND_ERASE_SECTOR_64K				0xD8
#define COMMAND_ERASE_SECTOR_4K_DO_NOT_USE		0x20

// Buffer Size
#define ALIGNMENT_WINDOW 256
#define DATA_BUFFER_SIZE_TRANSFER				256

//Manufacture ID code
#define S25F_FLASH_MANUFACTURER_CODE                 0x01
#define S25F_FLASH_DEVICE_CODE_0                     0x02
#define S25F_FLASH_DEVICE_CODE_1                     0x15

#define MX25L_FLASH_MANUFACTURER_CODE                0xC2
#define MX25L_FLASH_DEVICE_CODE_0                    0x20
#define MX25L_FLASH_DEVICE_CODE_1                    0x16

//Deployment
#define DEPLOYMENT_SECTOR_START 20
#define DEPLOYMENT_SECTOR_END   57
#define DEPLOYMENT_SECTOR_NUM   (DEPLOYMENT_SECTOR_END - DEPLOYMENT_SECTOR_START + 1)

//SPI config
#define SPI_CS                    PIN(4,27)
#define SPI_CLOCK_RATE_HZ         20000000
#define SPI_MODULE                1 // SPI1

static uint32_t sectorAddress[DEPLOYMENT_SECTOR_NUM];
static uint32_t sectorSize[DEPLOYMENT_SECTOR_NUM];

static TinyCLR_Deployment_Provider deploymentProvider;
static TinyCLR_Api_Info deploymentApi;

TinyCLR_Spi_Provider* flashSpiProviders;

struct ST25_Flash_Controller {
    uint8_t dataReadBuffer[FLASH_SECTOR_SIZE + 4];    // Read 64K bytes
    uint8_t dataWriteBuffer[FLASH_SECTOR_SIZE + 4];   // Write 256 bytes

    TinyCLR_Spi_Provider* provider;
};

ST25_Flash_Controller g_ST25_Flash_Controller;

const TinyCLR_Api_Info* LPC17_Deployment_GetApi() {
    deploymentProvider.Parent = &deploymentApi;
    deploymentProvider.Index = 0;
    deploymentProvider.Acquire = &LPC17_Flash_Acquire;
    deploymentProvider.Release = &LPC17_Flash_Release;
    deploymentProvider.Read = &LPC17_Flash_Read;
    deploymentProvider.Write = &LPC17_Flash_Write;
    deploymentProvider.EraseSector = &LPC17_Flash_EraseBlock;
    deploymentProvider.IsSectorErased = &LPC17_Flash_IsBlockErased;
    deploymentProvider.GetSectorMap = &LPC17_Flash_GetSectorMap;

    deploymentApi.Author = "GHI Electronics, LLC";
    deploymentApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.DeploymentProvider";
    deploymentApi.Type = TinyCLR_Api_Type::DeploymentProvider;
    deploymentApi.Version = 0;
    deploymentApi.Count = 1;
    deploymentApi.Implementation = &deploymentProvider;

    return &deploymentApi;
}

bool __section("SectionForFlashOperations") LPC17_Flash_WriteEnable() {
    g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_WRITE_ENABLE;
    g_ST25_Flash_Controller.dataWriteBuffer[1] = 0x00;

    size_t writeLength = 1;
    size_t readLength = 0;

    LPC17_Spi_TransferFullDuplex(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength, g_ST25_Flash_Controller.dataReadBuffer, readLength);

    g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_STATUS_REGISTER;
    g_ST25_Flash_Controller.dataWriteBuffer[1] = 0x00;

    writeLength = 2;
    readLength = 2;

    LPC17_Spi_TransferFullDuplex(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength, g_ST25_Flash_Controller.dataReadBuffer, readLength);

    if ((g_ST25_Flash_Controller.dataReadBuffer[1] & 0x2) != 0)
        return true;
    else
        return false;
}

bool __section("SectionForFlashOperations") LPC17_Flash_WriteInProgress() {
    g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_STATUS_REGISTER;
    g_ST25_Flash_Controller.dataWriteBuffer[1] = 0x00;

    size_t writeLength = 2;
    size_t readLength = 2;

    LPC17_Spi_TransferFullDuplex(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength, g_ST25_Flash_Controller.dataReadBuffer, readLength);

    if ((g_ST25_Flash_Controller.dataReadBuffer[1] & 0x1) != 0)
        return true;
    else
        return false;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC17_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    while (LPC17_Flash_WriteInProgress() == true);

    int32_t block = length / FLASH_SECTOR_SIZE;
    int32_t rest = length % FLASH_SECTOR_SIZE;
    int32_t index = 0;
    size_t writeLength = FLASH_SECTOR_SIZE + 4;
    size_t readLength = FLASH_SECTOR_SIZE + 4;

    address -= FLASH_BASE_ADDRESS;

    while (block > 0) {
        g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_DATA;
        g_ST25_Flash_Controller.dataWriteBuffer[1] = (uint8_t)((address) >> 16);
        g_ST25_Flash_Controller.dataWriteBuffer[2] = (uint8_t)((address) >> 8);
        g_ST25_Flash_Controller.dataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = FLASH_SECTOR_SIZE + 4;
        readLength = FLASH_SECTOR_SIZE + 4;

        // start to read
        LPC17_Spi_TransferFullDuplex(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength, g_ST25_Flash_Controller.dataReadBuffer, readLength);

        // copy to buffer
        memcpy(&buffer[index], &g_ST25_Flash_Controller.dataReadBuffer[4], FLASH_SECTOR_SIZE);

        address += FLASH_SECTOR_SIZE;
        index += FLASH_SECTOR_SIZE;
        block--;

    }

    if (rest > 0) {
        g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_READ_DATA;
        g_ST25_Flash_Controller.dataWriteBuffer[1] = (uint8_t)((address) >> 16);
        g_ST25_Flash_Controller.dataWriteBuffer[2] = (uint8_t)((address) >> 8);
        g_ST25_Flash_Controller.dataWriteBuffer[3] = (uint8_t)((address) >> 0);

        writeLength = rest + 4;
        readLength = rest + 4;

        // start to read
        LPC17_Spi_TransferFullDuplex(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength, g_ST25_Flash_Controller.dataReadBuffer, readLength);

        // copy to buffer
        memcpy(&buffer[index], &g_ST25_Flash_Controller.dataReadBuffer[4], rest);

        address += rest;
        index += rest;
        rest = 0;
    }

    return index == length ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

bool __section("SectionForFlashOperations") LPC17_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer) {

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
        while (LPC17_Flash_WriteEnable() == false);

        g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_PAGE_PROGRAMMING; //0x2
        g_ST25_Flash_Controller.dataWriteBuffer[1] = (uint8_t)(addr >> 16);
        g_ST25_Flash_Controller.dataWriteBuffer[2] = (uint8_t)(addr >> 8);
        g_ST25_Flash_Controller.dataWriteBuffer[3] = (uint8_t)(addr >> 0);

        size_t actualWrite = block_size + 4;

        memcpy(&g_ST25_Flash_Controller.dataWriteBuffer[4], pointerToWriteBuffer + source_index, block_size);

        // Write cmd to Write
        LPC17_Spi_Write(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, actualWrite);

        while (LPC17_Flash_WriteInProgress() == true);

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
        return true;
    else
        return false;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC17_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    bool result = LPC17_Flash_PageProgram(address, length, buffer);

    return result == true ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
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

TinyCLR_Result __section("SectionForFlashOperations") LPC17_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t address = sectorAddress[sector];

    LPC17_Flash_Read(&deploymentProvider, address, FLASH_SECTOR_SIZE, g_ST25_Flash_Controller.dataReadBuffer);

    uint32_t *ptr = (uint32_t*)&g_ST25_Flash_Controller.dataReadBuffer;

    erased = CompareArrayValueToValue(ptr, 0xFFFFFFFF, FLASH_SECTOR_SIZE / 4);

    return TinyCLR_Result::Success;
}

TinyCLR_Result __section("SectionForFlashOperations") LPC17_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    while (LPC17_Flash_WriteEnable() == false);

    uint32_t address = sectorAddress[sector];

    g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_ERASE_SECTOR_64K;
    g_ST25_Flash_Controller.dataWriteBuffer[1] = (uint8_t)((address - FLASH_BASE_ADDRESS) >> 16);
    g_ST25_Flash_Controller.dataWriteBuffer[2] = (uint8_t)((address - FLASH_BASE_ADDRESS) >> 8);
    g_ST25_Flash_Controller.dataWriteBuffer[3] = (uint8_t)((address - FLASH_BASE_ADDRESS) >> 0);

    size_t writeLength = 4;

    LPC17_Spi_Write(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength);

    while (LPC17_Flash_WriteInProgress() == true);

    TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportXIP) {
    const TinyCLR_Api_Info* spiApi = LPC17_Spi_GetApi();
    TinyCLR_Spi_Provider** spiProvider = (TinyCLR_Spi_Provider**)spiApi->Implementation;

    supportXIP = false;

    g_ST25_Flash_Controller.dataWriteBuffer[0] = COMMAND_READID;
    g_ST25_Flash_Controller.dataWriteBuffer[1] = 0x00;
    g_ST25_Flash_Controller.dataWriteBuffer[2] = 0x00;
    g_ST25_Flash_Controller.dataWriteBuffer[3] = 0x00;

    size_t writeLength = 4;
    size_t readLength = 4;

    g_ST25_Flash_Controller.provider = spiProvider[SPI_MODULE];

    LPC17_Spi_Acquire(g_ST25_Flash_Controller.provider);

    LPC17_Spi_SetActiveSettings(g_ST25_Flash_Controller.provider, SPI_CS, SPI_CLOCK_RATE_HZ, 8, TinyCLR_Spi_Mode::Mode0);

    LPC17_Spi_TransferFullDuplex(g_ST25_Flash_Controller.provider, g_ST25_Flash_Controller.dataWriteBuffer, writeLength, g_ST25_Flash_Controller.dataReadBuffer, readLength);

    if (S25F_FLASH_MANUFACTURER_CODE != g_ST25_Flash_Controller.dataReadBuffer[1] && MX25L_FLASH_MANUFACTURER_CODE != g_ST25_Flash_Controller.dataReadBuffer[1]) {

        return TinyCLR_Result::WrongType;
    }

    if ((S25F_FLASH_DEVICE_CODE_0 != g_ST25_Flash_Controller.dataReadBuffer[2] || S25F_FLASH_DEVICE_CODE_1 != g_ST25_Flash_Controller.dataReadBuffer[3]) && (MX25L_FLASH_DEVICE_CODE_0 != g_ST25_Flash_Controller.dataReadBuffer[2] || MX25L_FLASH_DEVICE_CODE_1 != g_ST25_Flash_Controller.dataReadBuffer[3])) {

        return TinyCLR_Result::WrongType;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Flash_Release(const TinyCLR_Deployment_Provider* self) {
    // UnInitialize Flash can be here
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size) {
    size = FLASH_SECTOR_SIZE;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count) {
    for (auto i = 0; i < DEPLOYMENT_SECTOR_NUM; i++) {
        sectorAddress[i] = ((DEPLOYMENT_SECTOR_START + i) * FLASH_SECTOR_SIZE) | FLASH_BASE_ADDRESS;
        sectorSize[i] = FLASH_SECTOR_SIZE;
    }

    addresses = sectorAddress;
    sizes = sectorSize;
    count = DEPLOYMENT_SECTOR_NUM;

    return TinyCLR_Result::Success;
}

