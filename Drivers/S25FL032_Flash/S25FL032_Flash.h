#pragma once

#include <TinyCLR.h>

//Deployment
const TinyCLR_Api_Info* S25FL032_Deployment_GetApi();
TinyCLR_Result S25FL032_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool &supportXIP);
TinyCLR_Result S25FL032_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result S25FL032_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result S25FL032_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result S25FL032_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result S25FL032_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased);
TinyCLR_Result S25FL032_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);
TinyCLR_Result S25FL032_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
bool S25FL032_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer);
bool S25FL032_Flash_IsSupportsXIP(const TinyCLR_Deployment_Provider* self);