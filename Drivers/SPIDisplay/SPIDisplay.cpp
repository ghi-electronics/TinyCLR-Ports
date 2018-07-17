#include "SPIDisplay.h"

#define TOTAL_SPI_DISPLAY_CONTROLLER 1

extern const TinyCLR_Api_Manager* apiManager;

static TinyCLR_Display_Controller spiDisplayControllers[TOTAL_SPI_DISPLAY_CONTROLLER];
static TinyCLR_Api_Info spiDisplayApi[TOTAL_SPI_DISPLAY_CONTROLLER];
static TinyCLR_Display_DataFormat spiDisplayDataFormats[] = { TinyCLR_Display_DataFormat::Rgb565 };
static int32_t spiDisplayWidth = 0;
static int32_t spiDisplayHeight = 0;
static TinyCLR_Display_SpiConfiguration spiDisplayConfig;
static const TinyCLR_Spi_Controller* spiDisplayBus;

const TinyCLR_Api_Info* SPIDisplay_GetApi() {
    for (auto i = 0; i < TOTAL_SPI_DISPLAY_CONTROLLER; i++) {
        spiDisplayControllers[i].ApiInfo = &spiDisplayApi[i];
        spiDisplayControllers[i].Acquire = &SPIDisplay_Acquire;
        spiDisplayControllers[i].Release = &SPIDisplay_Release;
        spiDisplayControllers[i].Enable = &SPIDisplay_Enable;
        spiDisplayControllers[i].Disable = &SPIDisplay_Disable;
        spiDisplayControllers[i].SetConfiguration = &SPIDisplay_SetConfiguration;
        spiDisplayControllers[i].GetConfiguration = &SPIDisplay_GetConfiguration;
        spiDisplayControllers[i].GetCapabilities = &SPIDisplay_GetCapabilities;
        spiDisplayControllers[i].DrawBuffer = &SPIDisplay_DrawBuffer;
        spiDisplayControllers[i].WriteString = &SPIDisplay_WriteString;

        spiDisplayApi[i].Author = "GHI Electronics, LLC";
        spiDisplayApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.Drivers.SPIDisplay";
        spiDisplayApi[i].Type = TinyCLR_Api_Type::DisplayController;
        spiDisplayApi[i].Version = 0;
        spiDisplayApi[i].Implementation = &spiDisplayControllers[i];
        spiDisplayApi[i].State = nullptr;
    }

    return (const TinyCLR_Api_Info*)&spiDisplayApi;
}

TinyCLR_Result SPIDisplay_Acquire(const TinyCLR_Display_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_Release(const TinyCLR_Display_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_Enable(const TinyCLR_Display_Controller* self) {
    auto res = apiManager->Find(apiManager, spiDisplayConfig.SpiSelector, TinyCLR_Api_Type::SpiController);

    if (res == nullptr)
        return TinyCLR_Result::InvalidOperation;

    spiDisplayBus = reinterpret_cast<const TinyCLR_Spi_Controller*>(res->Implementation);

    return spiDisplayBus != nullptr ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result SPIDisplay_Disable(const TinyCLR_Display_Controller* self) {
    spiDisplayBus = nullptr;

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_WriteString(const TinyCLR_Display_Controller* self, const char* buffer, size_t length) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result SPIDisplay_GetCapabilities(const TinyCLR_Display_Controller* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount) {
    type = TinyCLR_Display_InterfaceType::Spi;
    supportedDataFormatCount = sizeof(spiDisplayDataFormats) / sizeof(spiDisplayDataFormats[0]);
    supportedDataFormats = spiDisplayDataFormats;

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_GetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration) {
    dataFormat = TinyCLR_Display_DataFormat::Rgb565;
    width = spiDisplayWidth;
    height = spiDisplayHeight;

    if (configuration != nullptr)
        reinterpret_cast<TinyCLR_Display_SpiConfiguration*>(configuration)->SpiSelector = spiDisplayConfig.SpiSelector;

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_SetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration) {
    if (dataFormat != TinyCLR_Display_DataFormat::Rgb565) return TinyCLR_Result::NotSupported;

    spiDisplayWidth = width;
    spiDisplayHeight = height;

    if (configuration != nullptr)
        spiDisplayConfig.SpiSelector = reinterpret_cast<const TinyCLR_Display_SpiConfiguration*>(configuration)->SpiSelector;

    return TinyCLR_Result::Success;
}

static void Swap(uint8_t* a, uint8_t* b) {
    uint8_t temp = *a;
    *a = *b;
    *b = temp;
}

TinyCLR_Result SPIDisplay_DrawBuffer(const TinyCLR_Display_Controller* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data) {
    auto d = const_cast<uint8_t*>(data);

    for (auto i = 0; i < spiDisplayWidth * spiDisplayHeight * 2; i += 2)
        Swap(d + i, d + i + 1);

    if (x == 0 && spiDisplayWidth == width) {
        auto len = static_cast<size_t>(width * height * 2);

        spiDisplayBus->Write(spiDisplayBus, data + (y * spiDisplayWidth * 2), len);
    }
    else {
        auto len = static_cast<size_t>(width * 2);

        for (auto yy = y; yy < y + height; yy++)
            spiDisplayBus->Write(spiDisplayBus, data + (yy * spiDisplayWidth * 2) + (x * 2), len);
    }

    for (auto i = 0; i < (spiDisplayWidth * spiDisplayHeight * 2); i += 2)
        Swap(d + i, d + i + 1);

    return TinyCLR_Result::Success;
}
