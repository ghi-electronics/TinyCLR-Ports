#include "LPC17.h"

#ifdef INCLUDE_SD
// stm32f4

static TinyCLR_SdCard_Provider sdCardProvider;
static TinyCLR_Api_Info sdApi;

#define LPC17_SD_SECTOR_SIZE 512
#define LPC17_SD_TIMEOUT 5000000

struct SdController {
    int32_t controller;
    size_t  sectorCount;

    size_t  *sectorSizes;
    uint8_t *pBuffer;
};

static const LPC17_Gpio_Pin g_LPC17_SdCard_Data0_Pins[] = LPC17_SD_DATA0_PINS;
static const LPC17_Gpio_Pin g_LPC17_SdCard_Data1_Pins[] = LPC17_SD_DATA1_PINS;
static const LPC17_Gpio_Pin g_LPC17_SdCard_Data2_Pins[] = LPC17_SD_DATA2_PINS;
static const LPC17_Gpio_Pin g_LPC17_SdCard_Data3_Pins[] = LPC17_SD_DATA3_PINS;
static const LPC17_Gpio_Pin g_LPC17_SdCard_Clk_Pins[] = LPC17_SD_CLK_PINS;
static const LPC17_Gpio_Pin g_LPC17_SdCard_Cmd_Pins[] = LPC17_SD_CMD_PINS;

SdController sdController[1];

void LPC17_SdCard_Interrupt(void* param) {
    SD_ProcessIRQSrc();
}

const TinyCLR_Api_Info* LPC17_SdCard_GetApi() {
    sdCardProvider.Parent = &sdApi;

    sdCardProvider.Acquire = &LPC17_SdCard_Acquire;
    sdCardProvider.Release = &LPC17_SdCard_Release;
    sdCardProvider.GetControllerCount = &LPC17_SdCard_GetControllerCount;

    sdCardProvider.WriteSectors = &LPC17_SdCard_WriteSector;
    sdCardProvider.ReadSectors = &LPC17_SdCard_ReadSector;
    sdCardProvider.EraseSectors = &LPC17_SdCard_EraseSector;
    sdCardProvider.IsSectorErased = &LPC17_SdCard_IsSectorErased;
    sdCardProvider.GetSectorMap = &LPC17_SdCard_GetSectorMap;

    sdApi.Author = "GHI Electronics, LLC";
    sdApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.SdCardProvider";
    sdApi.Type = TinyCLR_Api_Type::SdCardProvider;
    sdApi.Version = 0;
    sdApi.Implementation = &sdCardProvider;

    return &sdApi;
}

TinyCLR_Result LPC17_SdCard_Acquire(const TinyCLR_SdCard_Provider* self, int32_t controller) {
    sdController[controller].controller = controller;
    // Initialize hal layer here

    auto d0 = g_LPC17_SdCard_Data0_Pins[controller];
    auto d1 = g_LPC17_SdCard_Data1_Pins[controller];
    auto d2 = g_LPC17_SdCard_Data2_Pins[controller];
    auto d3 = g_LPC17_SdCard_Data3_Pins[controller];
    auto clk = g_LPC17_SdCard_Clk_Pins[controller];
    auto cmd = g_LPC17_SdCard_Cmd_Pins[controller];

    if (!LPC17_GpioInternal_OpenPin(d0.number)
        || !LPC17_GpioInternal_OpenPin(d1.number)
        || !LPC17_GpioInternal_OpenPin(d2.number)
        || !LPC17_GpioInternal_OpenPin(d3.number)
        || !LPC17_GpioInternal_OpenPin(clk.number)
        || !LPC17_GpioInternal_OpenPin(cmd.number)
        )
        return TinyCLR_Result::SharingViolation;

    LPC17_GpioInternal_ConfigurePin(d0.number, LPC17_Gpio_PortMode::AlternateFunction, LPC17_Gpio_OutputType::PushPull, LPC17_Gpio_OutputSpeed::High, LPC17_Gpio_PullDirection::PullUp, d0.alternateFunction);
    LPC17_GpioInternal_ConfigurePin(d1.number, LPC17_Gpio_PortMode::AlternateFunction, LPC17_Gpio_OutputType::PushPull, LPC17_Gpio_OutputSpeed::High, LPC17_Gpio_PullDirection::PullUp, d1.alternateFunction);
    LPC17_GpioInternal_ConfigurePin(d2.number, LPC17_Gpio_PortMode::AlternateFunction, LPC17_Gpio_OutputType::PushPull, LPC17_Gpio_OutputSpeed::High, LPC17_Gpio_PullDirection::PullUp, d2.alternateFunction);
    LPC17_GpioInternal_ConfigurePin(d3.number, LPC17_Gpio_PortMode::AlternateFunction, LPC17_Gpio_OutputType::PushPull, LPC17_Gpio_OutputSpeed::High, LPC17_Gpio_PullDirection::PullUp, d3.alternateFunction);
    LPC17_GpioInternal_ConfigurePin(clk.number, LPC17_Gpio_PortMode::AlternateFunction, LPC17_Gpio_OutputType::PushPull, LPC17_Gpio_OutputSpeed::High, LPC17_Gpio_PullDirection::None, clk.alternateFunction);
    LPC17_GpioInternal_ConfigurePin(cmd.number, LPC17_Gpio_PortMode::AlternateFunction, LPC17_Gpio_OutputType::PushPull, LPC17_Gpio_OutputSpeed::High, LPC17_Gpio_PullDirection::PullUp, cmd.alternateFunction);

    RCC->APB2ENR |= (1 << 11);

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    sdController[controller].pBuffer = (uint8_t*)memoryProvider->Allocate(memoryProvider, LPC17_SD_SECTOR_SIZE);
    sdController[controller].sectorSizes = (size_t*)memoryProvider->Allocate(memoryProvider, sizeof(size_t));

    SD_DeInit();

    int32_t to = 2;

    SD_Error error = SD_Init();

    if (error != SD_OK) // try one more time
        return SD_Init() == SD_OK ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_SdCard_Release(const TinyCLR_SdCard_Provider* self, int32_t controller) {
    auto d0 = g_LPC17_SdCard_Data0_Pins[controller];
    auto d1 = g_LPC17_SdCard_Data1_Pins[controller];
    auto d2 = g_LPC17_SdCard_Data2_Pins[controller];
    auto d3 = g_LPC17_SdCard_Data3_Pins[controller];
    auto clk = g_LPC17_SdCard_Clk_Pins[controller];
    auto cmd = g_LPC17_SdCard_Cmd_Pins[controller];

    SD_DeInit();

    RCC->APB2ENR &= ~(1 << 11);

    auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

    memoryProvider->Free(memoryProvider, sdController[controller].pBuffer);
    memoryProvider->Free(memoryProvider, sdController[controller].sectorSizes);

    LPC17_GpioInternal_ClosePin(d0.number);
    LPC17_GpioInternal_ClosePin(d1.number);
    LPC17_GpioInternal_ClosePin(d2.number);
    LPC17_GpioInternal_ClosePin(d3.number);
    LPC17_GpioInternal_ClosePin(clk.number);
    LPC17_GpioInternal_ClosePin(cmd.number);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_SdCard_GetControllerCount(const TinyCLR_SdCard_Provider* self, int32_t& count) {
    count = SIZEOF_ARRAY(g_LPC17_SdCard_Data0_Pins);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_SdCard_WriteSector(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, size_t& count, const uint8_t* data, int32_t timeout) {
    int32_t index = 0;

    int32_t to;

    auto sectorCount = count;

    auto sectorNum = sector;

    uint8_t* pData = (uint8_t*)data;

    while (sectorCount) {
        to = timeout;

        while (SD_GetStatus() != SD_TRANSFER_OK && to--) {
            LPC17_Time_Delay(nullptr, 1);
        }

        if (to > 0 && SD_WriteBlock(&pData[index], sectorNum * LPC17_SD_SECTOR_SIZE, LPC17_SD_SECTOR_SIZE) == SD_OK) {
            index += LPC17_SD_SECTOR_SIZE;
            sectorNum++;
            sectorCount--;
        }
        else {
            SD_StopTransfer();
        }

        if (!to) {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    return TinyCLR_Result::Success;

}

TinyCLR_Result LPC17_SdCard_ReadSector(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, size_t& count, uint8_t* data, int32_t timeout) {
    int32_t index = 0;

    int32_t to;

    auto sectorCount = count;

    auto sectorNum = sector;

    while (sectorCount) {
        to = timeout;

        while (SD_GetStatus() != SD_TRANSFER_OK && to--) {
            LPC17_Time_Delay(nullptr, 1);
        }

        if (to > 0 && SD_ReadBlock(&data[index], sectorNum * LPC17_SD_SECTOR_SIZE, LPC17_SD_SECTOR_SIZE) == SD_OK) {

            index += LPC17_SD_SECTOR_SIZE;
            sectorNum++;
            sectorCount--;
        }
        else {
            SD_StopTransfer();
        }

        if (!to) {
            return TinyCLR_Result::InvalidOperation;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_SdCard_IsSectorErased(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, bool& erased) {
    erased = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_SdCard_EraseSector(const TinyCLR_SdCard_Provider* self, int32_t controller, uint64_t sector, size_t& count, int32_t timeout) {
    uint32_t addressStart = sector * LPC17_SD_SECTOR_SIZE;

    uint32_t addressEnd = addressStart + (count * LPC17_SD_SECTOR_SIZE);

    return SD_Erase(addressStart, addressEnd) == SD_OK ? TinyCLR_Result::Success : TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result LPC17_SdCard_GetSectorMap(const TinyCLR_SdCard_Provider* self, int32_t controller, const size_t*& sizes, size_t& count, bool& isUniform) {
    sdController[controller].sectorSizes[0] = LPC17_SD_SECTOR_SIZE;

    sizes = sdController[controller].sectorSizes;
    count = SDCardInfo.CardCapacity / LPC17_SD_SECTOR_SIZE;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_SdCard_Reset() {
    return TinyCLR_Result::Success;
}
#endif // INCLUDE_SD