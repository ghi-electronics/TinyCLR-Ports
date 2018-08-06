#pragma once

#include <TinyCLR.h>

TinyCLR_Result AT45DB321D_Flash_Acquire(const TinyCLR_Spi_Controller* spiProvider, const TinyCLR_NativeTime_Controller* timeProvider, uint32_t chipSelectLine);
TinyCLR_Result AT45DB321D_Flash_Release();
TinyCLR_Result AT45DB321D_Flash_Read(uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result AT45DB321D_Flash_Write(uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result AT45DB321D_Flash_EraseBlock(uint32_t sector);
TinyCLR_Result AT45DB321D_Flash_IsBlockErased(uint32_t sector, bool &erased);
TinyCLR_Result AT45DB321D_Flash_GetBytesPerSector(uint32_t address, int32_t& size);
TinyCLR_Result AT45DB321D_Flash_GetSectorMap(const uint64_t*& addresses, const size_t*& sizes, size_t& count);
