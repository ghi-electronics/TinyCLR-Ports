#include "AT91.h"

#ifdef INCLUDE_SD
// stm32f4

static TinyCLR_SdCard_Provider sdCardProvider;
static TinyCLR_Api_Info sdApi;

#define AT91_SD_SECTOR_SIZE 512
#define AT91_SD_TIMEOUT 5000000

struct SdController {
    int32_t controller;
    size_t  sectorCount;

    size_t  *sectorSizes;
    uint8_t *pBuffer;
};

static const AT91_Gpio_Pin g_AT91_SdCard_Data0_Pins[] = AT91_SD_DATA0_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Data1_Pins[] = AT91_SD_DATA1_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Data2_Pins[] = AT91_SD_DATA2_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Data3_Pins[] = AT91_SD_DATA3_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Clk_Pins[] = AT91_SD_CLK_PINS;
static const AT91_Gpio_Pin g_AT91_SdCard_Cmd_Pins[] = AT91_SD_CMD_PINS;

SdController sdController[1];

void AT91_SdCard_Interrupt(void* param) {
    SD_ProcessIRQSrc();
}

const TinyCLR_Api_Info* AT91_SdCard_GetApi() {
    sdCardProvider.Parent = &sdApi;

    sdCardProvider.Acquire = &AT91_SdCard_Acquire;
    sdCardProvider.Release = &AT91_SdCard_Release;
    sdCardProvider.GetControllerCount = &AT91_SdCard_GetControllerCount;

    sdCardProvider.WriteSectors = &AT91_SdCard_WriteSector;
    sdCardProvider.ReadSectors = &AT91_SdCard_ReadSector;
    sdCardProvider.EraseSectors = &AT91_SdCard_EraseSector;
    sdCardProvider.IsSectorErased = &AT91_SdCard_IsSectorErased;
    sdCardProvider.GetSectorMap = &AT91_SdCard_GetSectorMap;

    sdApi.Author = "GHI Electronics, LLC";
    sdApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.SdCardProvider";
    sdApi.Type = TinyCLR_Api_Type::SdCardProvider;
    sdApi.Version = 0;
    sdApi.Implementation = &sdCardProvider;

    return &sdApi;
}

TinyCLR_Result AT91_SdCard_Acquire(const TinyCLR_SdCard_Provider* self, int32_t controller) {
    sdController[controller].controller = controller;
    // Initialize hal layer here

    auto d0 = g_AT91_SdCard_Data0_Pins[controller];
    auto d1 = g_AT91_SdCard_Data1_Pins[controller];
    auto d2 = g_AT91_SdCard_Data2_Pins[controller];
    auto d3 = g_AT91_SdCard_Data3_Pins[controller];
    auto clk = g_AT91_SdCard_Clk_Pins[controller];
    auto cmd = g_AT91_SdCard_Cmd_Pins[controller];

    if (!AT91_GpioInternal_OpenPin(d0.number)
        || !AT91_GpioInternal_OpenPin(d1.number)
        || !AT91_GpioInternal_OpenPin(d2.number)
        || !AT91_GpioInternal_OpenPin(d3.number)
        || !AT91_GpioInternal_OpenPin(clk.number)
        || !AT91_GpioInternal_OpenPin(cmd.number)
        )
        return TinyCLR_Result::SharingViolation;

    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_Release(const TinyCLR_SdCard_Provider* self, int32_t controller) {
    auto d0 = g_AT91_SdCard_Data0_Pins[controller];
    auto d1 = g_AT91_SdCard_Data1_Pins[controller];
    auto d2 = g_AT91_SdCard_Data2_Pins[controller];
    auto d3 = g_AT91_SdCard_Data3_Pins[controller];
    auto clk = g_AT91_SdCard_Clk_Pins[controller];
    auto cmd = g_AT91_SdCard_Cmd_Pins[controller];

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    memoryProvider->Free(memoryProvider, sdController[controller].pBuffer);
    memoryProvider->Free(memoryProvider, sdController[controller].sectorSizes);

    AT91_GpioInternal_ClosePin(d0.number);
    AT91_GpioInternal_ClosePin(d1.number);
    AT91_GpioInternal_ClosePin(d2.number);
    AT91_GpioInternal_ClosePin(d3.number);
    AT91_GpioInternal_ClosePin(clk.number);
    AT91_GpioInternal_ClosePin(cmd.number);

    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_GetControllerCount(const TinyCLR_SdCard_Provider* self, int32_t& count) {
    count = SIZEOF_ARRAY(g_AT91_SdCard_Data0_Pins);

    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_WriteSector(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, size_t& count, const uint8_t* data, int32_t timeout) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_ReadSector(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, size_t& count, uint8_t* data, int32_t timeout) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_IsSectorErased(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, bool& erased) {
    erased = true;
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_EraseSector(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, size_t& count, int32_t timeout) {
    uint32_t addressStart = sector * AT91_SD_SECTOR_SIZE;

    uint32_t addressEnd = addressStart + (count * AT91_SD_SECTOR_SIZE);

    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_GetSectorMap(const TinyCLR_SdCard_Provider* self, int32_t controller, const size_t*& sizes, size_t& count, bool& isUniform) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result AT91_SdCard_Reset() {
    return TinyCLR_Result::NotImplemented;
}
#endif // INCLUDE_SD