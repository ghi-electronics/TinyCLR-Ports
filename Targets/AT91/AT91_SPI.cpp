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

#include "AT91.h"

#define DATA_BIT_LENGTH_16  16
#define DATA_BIT_LENGTH_8   8

static const uint32_t g_AT91_Spi_Miso_Pins[] = AT91_SPI_MISO_PINS;
static const uint32_t g_AT91_Spi_Mosi_Pins[] = AT91_SPI_MOSI_PINS;
static const uint32_t g_AT91_Spi_Sclk_Pins[] = AT91_SPI_CLK_PINS;

static const AT91_Gpio_PinFunction g_AT91_Spi_Miso_AltMode[] = AT91_SPI_MISO_ALT_MODE;
static const AT91_Gpio_PinFunction g_AT91_Spi_Mosi_AltMode[] = AT91_SPI_MOSI_ALT_MODE;
static const AT91_Gpio_PinFunction g_AT91_Spi_Sclk_AltMode[] = AT91_SPI_CLK_ALT_MODE;

struct SpiController {
    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t writeLength;
    size_t readLength;
    size_t readOffset;

    int32_t ChipSelectLine;
    int32_t ClockFrequency;
    int32_t DataBitLength;

    TinyCLR_Spi_Mode Mode;
};

static SpiController g_SpiController[TOTAL_SPI_CONTROLLERS];

static uint8_t spiProviderDefs[TOTAL_SPI_CONTROLLERS * sizeof(TinyCLR_Spi_Provider)];
static TinyCLR_Spi_Provider* spiProviders[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi;

const TinyCLR_Api_Info* AT91_Spi_GetApi() {
    for (int i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiProviders[i] = (TinyCLR_Spi_Provider*)(spiProviderDefs + (i * sizeof(TinyCLR_Spi_Provider)));
        spiProviders[i]->Parent = &spiApi;
        spiProviders[i]->Index = i;
        spiProviders[i]->Acquire = &AT91_Spi_Acquire;
        spiProviders[i]->Release = &AT91_Spi_Release;
        spiProviders[i]->SetActiveSettings = &AT91_Spi_SetActiveSettings;
        spiProviders[i]->Read = &AT91_Spi_Read;
        spiProviders[i]->Write = &AT91_Spi_Write;
        spiProviders[i]->TransferFullDuplex = &AT91_Spi_TransferFullDuplex;
        spiProviders[i]->TransferSequential = &AT91_Spi_TransferSequential;
        spiProviders[i]->GetChipSelectLineCount = &AT91_Spi_GetChipSelectLineCount;
        spiProviders[i]->GetMinClockFrequency = &AT91_Spi_GetMinClockFrequency;
        spiProviders[i]->GetMaxClockFrequency = &AT91_Spi_GetMaxClockFrequency;
        spiProviders[i]->GetSupportedDataBitLengths = &AT91_Spi_GetSupportedDataBitLengths;
    }

    spiApi.Author = "GHI Electronics, LLC";
    spiApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.SpiProvider";
    spiApi.Type = TinyCLR_Api_Type::SpiProvider;
    spiApi.Version = 0;
    spiApi.Count = TOTAL_SPI_CONTROLLERS;
    spiApi.Implementation = spiProviders;
    return &spiApi;
}

bool AT91_Spi_Transaction_Start(int32_t controller) {

    return true;
}

bool AT91_Spi_Transaction_Stop(int32_t controller) {
    return true;
}


bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controller) {

    return true;
}

bool AT91_Spi_Transaction_nWrite16_nRead16(int32_t controller) {

    return true;
}

TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (AT91_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return AT91_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result AT91_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = readBuffer;
    g_SpiController[controller].readLength = readLength;
    g_SpiController[controller].writeBuffer = (uint8_t*)writeBuffer;
    g_SpiController[controller].writeLength = writeLength;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = buffer;
    g_SpiController[controller].readLength = length;
    g_SpiController[controller].writeBuffer = nullptr;
    g_SpiController[controller].writeLength = 0;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = nullptr;
    g_SpiController[controller].readLength = 0;
    g_SpiController[controller].writeBuffer = (uint8_t*)buffer;
    g_SpiController[controller].writeLength = length;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    int32_t controller = (self->Index);

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].ChipSelectLine = chipSelectLine;
    g_SpiController[controller].ClockFrequency = clockFrequency;
    g_SpiController[controller].DataBitLength = dataBitLength;
    g_SpiController[controller].Mode = mode;
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Acquire(const TinyCLR_Spi_Provider* self) {
    int32_t controller = (self->Index);

    int32_t clkPin = g_AT91_Spi_Sclk_Pins[controller];
    int32_t misoPin = g_AT91_Spi_Miso_Pins[controller];
    int32_t mosiPin = g_AT91_Spi_Mosi_Pins[controller];

    // Check each pin single time make sure once fail not effect to other pins
    if (!AT91_Gpio_OpenPin(clkPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(misoPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(mosiPin))
        return TinyCLR_Result::SharingViolation;

    switch (controller) {
        case 0:
            AT91XX::SYSCON().PCONP |= PCONP_PCSSP0;
            break;

        case 1:
            AT91XX::SYSCON().PCONP |= PCONP_PCSSP1;
            break;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Provider* self) {
    int32_t controller = (self->Index);

    int32_t clkPin = g_AT91_Spi_Sclk_Pins[controller];
    int32_t misoPin = g_AT91_Spi_Miso_Pins[controller];
    int32_t mosiPin = g_AT91_Spi_Mosi_Pins[controller];

    // Check each pin single time make sure once fail not effect to other pins
    AT91_Gpio_ClosePin(clkPin);
    AT91_Gpio_ClosePin(misoPin);
    AT91_Gpio_ClosePin(mosiPin);

    switch (controller) {
        case 0:

            break;

        case 1:

            break;

    }

    return TinyCLR_Result::Success;
}

int32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self) {
    return 1;
}

int32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self) {
    return 48000000;
}

int32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self) {
    return TOTAL_GPIO_PINS;
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void AT91_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        AT91_Spi_Release(spiProviders[i]);
    }
}