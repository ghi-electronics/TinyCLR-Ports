#include "SPIDisplay.h"

extern const TinyCLR_Api_Provider* apiProvider;

static TinyCLR_Display_Provider spiDisplayProvider;
static TinyCLR_Api_Info spiDisplayApi;
static TinyCLR_Display_DataFormat spiDisplayDataFormats[] = { TinyCLR_Display_DataFormat::Rgb565 };
static int32_t spiDisplayWidth = 0;
static int32_t spiDisplayHeight = 0;
static TinyCLR_Display_SpiConfiguration spiDisplayConfig;
static const TinyCLR_Spi_Provider* spiDisplayBus;

const TinyCLR_Api_Info* SPIDisplay_GetApi() {
    spiDisplayProvider.ApiInfo = &spiDisplayApi;
    spiDisplayProvider.Acquire = &SPIDisplay_Acquire;
    spiDisplayProvider.Release = &SPIDisplay_Release;
    spiDisplayProvider.Enable = &SPIDisplay_Enable;
    spiDisplayProvider.Disable = &SPIDisplay_Disable;
    spiDisplayProvider.SetConfiguration = &SPIDisplay_SetConfiguration;
    spiDisplayProvider.GetConfiguration = &SPIDisplay_GetConfiguration;
    spiDisplayProvider.GetCapabilities = &SPIDisplay_GetCapabilities;
    spiDisplayProvider.DrawBuffer = &SPIDisplay_DrawBuffer;
    spiDisplayProvider.WriteString = &SPIDisplay_WriteString;

    spiDisplayApi.Author = "GHI Electronics, LLC";
    spiDisplayApi.Name = "GHIElectronics.TinyCLR.NativeApis.Drivers.SPIDisplay";
    spiDisplayApi.Type = TinyCLR_Api_Type::DisplayProvider;
    spiDisplayApi.Version = 0;
    spiDisplayApi.Implementation = &spiDisplayProvider;

    return &spiDisplayApi;
}

TinyCLR_Result SPIDisplay_Acquire(const TinyCLR_Display_Provider* self, int32_t controller) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_Release(const TinyCLR_Display_Provider* self, int32_t controller) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_Enable(const TinyCLR_Display_Provider* self, int32_t controller) {
    spiDisplayBus = reinterpret_cast<const TinyCLR_Spi_Provider*>(apiProvider->FindBySelector(apiProvider, spiDisplayConfig.SpiSelector, TinyCLR_Api_Type::SpiProvider));

    return spiDisplayBus != nullptr ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result SPIDisplay_Disable(const TinyCLR_Display_Provider* self, int32_t controller) {
    spiDisplayBus = nullptr;

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_WriteString(const TinyCLR_Display_Provider* self, int32_t controller, const char* buffer, size_t length) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result SPIDisplay_GetCapabilities(const TinyCLR_Display_Provider* self, int32_t controller, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount) {
    type = TinyCLR_Display_InterfaceType::Spi;
    supportedDataFormatCount = sizeof(spiDisplayDataFormats) / sizeof(spiDisplayDataFormats[0]);
    supportedDataFormats = spiDisplayDataFormats;

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_GetConfiguration(const TinyCLR_Display_Provider* self, int32_t controller, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration) {
    dataFormat = TinyCLR_Display_DataFormat::Rgb565;
    width = spiDisplayWidth;
    height = spiDisplayHeight;

    if (configuration != nullptr)
        reinterpret_cast<TinyCLR_Display_SpiConfiguration*>(configuration)->SpiSelector = spiDisplayConfig.SpiSelector;

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_SetConfiguration(const TinyCLR_Display_Provider* self, int32_t controller, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration) {
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

TinyCLR_Result SPIDisplay_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t controller, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data) {
    auto d = const_cast<uint8_t*>(data);

    for (auto i = 0; i < spiDisplayWidth * spiDisplayHeight * 2; i += 2)
        Swap(d + i, d + i + 1);

    if (x == 0 && spiDisplayWidth == width) {
        auto len = static_cast<size_t>(width * height * 2);

        spiDisplayBus->Write(spiDisplayBus, controller, data + (y * spiDisplayWidth * 2), len);
    }
    else {
        auto len = static_cast<size_t>(width * 2);

        for (auto yy = y; yy < y + height; yy++)
            spiDisplayBus->Write(spiDisplayBus, controller, data + (yy * spiDisplayWidth * 2) + (x * 2), len);
    }

    for (auto i = 0; i < (spiDisplayWidth * spiDisplayHeight * 2); i += 2)
        Swap(d + i, d + i + 1);

    return TinyCLR_Result::Success;
}

TinyCLR_Result SPIDisplay_GetControllerCount(const TinyCLR_Display_Provider* self, int32_t controller, int32_t& count) {
    count = 1;

    return TinyCLR_Result::Success;
}