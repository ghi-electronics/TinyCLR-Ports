#pragma once

#include <TinyCLR.h>

//Deployment
const TinyCLR_Api_Info* AT45DB321D_Deployment_GetApi();
TinyCLR_Result AT45DB321D_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool &supportXIP);
TinyCLR_Result AT45DB321D_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result AT45DB321D_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result AT45DB321D_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result AT45DB321D_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result AT45DB321D_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased);
TinyCLR_Result AT45DB321D_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);
TinyCLR_Result AT45DB321D_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
TinyCLR_Result AT45DB321D_Flash_Reset(const TinyCLR_Deployment_Provider* self);