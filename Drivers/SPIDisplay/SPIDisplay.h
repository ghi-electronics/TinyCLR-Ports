#pragma once

#include <TinyCLR.h>

const TinyCLR_Api_Info* SPIDisplay_GetApi();
TinyCLR_Result SPIDisplay_Acquire(const TinyCLR_Display_Controller* self);
TinyCLR_Result SPIDisplay_Release(const TinyCLR_Display_Controller* self);
TinyCLR_Result SPIDisplay_Enable(const TinyCLR_Display_Controller* self);
TinyCLR_Result SPIDisplay_Disable(const TinyCLR_Display_Controller* self);
TinyCLR_Result SPIDisplay_GetCapabilities(const TinyCLR_Display_Controller* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount);
TinyCLR_Result SPIDisplay_GetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration);
TinyCLR_Result SPIDisplay_SetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration);
TinyCLR_Result SPIDisplay_DrawBuffer(const TinyCLR_Display_Controller* self, uint32_t x, uint32_t y, uint32_t width, uint32_t height, const uint8_t* data);
TinyCLR_Result SPIDisplay_DrawString(const TinyCLR_Display_Controller* self, const char* data, size_t length);
TinyCLR_Result SPIDisplay_GetControllerCount(const TinyCLR_Display_Controller* self, int32_t& count);
