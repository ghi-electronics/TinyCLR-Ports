#pragma once

#include <TinyCLR.h>

#define S25FL032_FLASH_SECTOR_SIZE                  (64*1024) 
#define S25FL032_FLASH_SECTOR_NUM                   64

//Command
#define COMMAND_SIZE                                4
#define COMMAND_READID                              0x9F
#define COMMAND_READ_STATUS_REGISTER                0x05
#define COMMAND_READ_DATA                           0x03
#define COMMAND_WRITE_ENABLE                        0x06
#define COMMAND_PAGE_PROGRAMMING                    0x02
#define COMMAND_ERASE_SECTOR_64K                    0xD8

// Buffer Size
#define ALIGNMENT_WINDOW 256
#define DATA_BUFFER_SIZE_TRANSFER                   256

//Manufacture ID code
#define S25F_FLASH_MANUFACTURER_CODE                0x01
#define S25F_FLASH_DEVICE_CODE_0                    0x02
#define S25F_FLASH_DEVICE_CODE_1                    0x15

#define MX25L_FLASH_MANUFACTURER_CODE               0xC2
#define MX25L_FLASH_DEVICE_CODE_0                   0x20
#define MX25L_FLASH_DEVICE_CODE_1                   0x16

TinyCLR_Result S25FL032_Flash_Acquire(const TinyCLR_Spi_Provider* spiProvider, uint32_t chipSelectLine, bool& supportXIP);
TinyCLR_Result S25FL032_Flash_Release();
TinyCLR_Result S25FL032_Flash_Read(uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result S25FL032_Flash_Write(uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result S25FL032_Flash_EraseBlock(uint32_t sector);
TinyCLR_Result S25FL032_Flash_IsBlockErased(uint32_t sector, bool& erased);
TinyCLR_Result S25FL032_Flash_GetBytesPerSector(uint32_t address, int32_t& size);
TinyCLR_Result S25FL032_Flash_GetSectorMap(const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
TinyCLR_Result S25FL032_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer);
