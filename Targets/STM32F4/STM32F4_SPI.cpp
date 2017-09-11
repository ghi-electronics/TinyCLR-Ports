// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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

#include "STM32F4.h"

typedef  SPI_TypeDef* ptr_SPI_TypeDef;

#define DATA_BIT_LENGTH_16  16
#define DATA_BIT_LENGTH_8   8

static const STM32F4_Pin g_STM32F4_Spi_Sclk_Pins[] = STM32F4_SPI_SCLK_PINS;
static const STM32F4_Pin g_STM32F4_Spi_Miso_Pins[] = STM32F4_SPI_MISO_PINS;
static const STM32F4_Pin g_STM32F4_Spi_Mosi_Pins[] = STM32F4_SPI_MOSI_PINS;

static const int TOTAL_SPI_CONTROLLERS = SIZEOF_CONST_ARRAY(g_STM32F4_Spi_Sclk_Pins);

static ptr_SPI_TypeDef g_STM32_Spi_Port[TOTAL_SPI_CONTROLLERS];

// Pins

struct SpiController {
    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t readLength;
    size_t writeLength;

    int32_t ChipSelectLine;
    int32_t ClockFrequency;
    int32_t DataBitLength;
    TinyCLR_Spi_Mode Mode;
};

static SpiController g_SpiController[TOTAL_SPI_CONTROLLERS];

static uint8_t spiProviderDefs[TOTAL_SPI_CONTROLLERS * sizeof(TinyCLR_Spi_Provider)];
static TinyCLR_Spi_Provider* spiProviders[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi;

const TinyCLR_Api_Info* STM32F4_Spi_GetApi() {
    for (int i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiProviders[i] = (TinyCLR_Spi_Provider*)(spiProviderDefs + (i * sizeof(TinyCLR_Spi_Provider)));
        spiProviders[i]->Parent = &spiApi;
        spiProviders[i]->Index = i;
        spiProviders[i]->Acquire = &STM32F4_Spi_Acquire;
        spiProviders[i]->Release = &STM32F4_Spi_Release;
        spiProviders[i]->SetActiveSettings = &STM32F4_Spi_SetActiveSettings;
        spiProviders[i]->Read = &STM32F4_Spi_Read;
        spiProviders[i]->Write = &STM32F4_Spi_Write;
        spiProviders[i]->TransferFullDuplex = &STM32F4_Spi_TransferFullDuplex;
        spiProviders[i]->TransferSequential = &STM32F4_Spi_TransferSequential;
        spiProviders[i]->GetChipSelectLineCount = &STM32F4_Spi_GetChipSelectLineCount;
        spiProviders[i]->GetMinClockFrequency = &STM32F4_Spi_GetMinClockFrequency;
        spiProviders[i]->GetMaxClockFrequency = &STM32F4_Spi_GetMaxClockFrequency;
        spiProviders[i]->GetSupportedDataBitLengths = &STM32F4_Spi_GetSupportedDataBitLengths;

    }

    spiApi.Author = "GHI Electronics, LLC";
    spiApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.SpiProvider";
    spiApi.Type = TinyCLR_Api_Type::SpiProvider;
    spiApi.Version = 0;
    spiApi.Count = TOTAL_SPI_CONTROLLERS;
    spiApi.Implementation = (spiApi.Count > 1) ? spiProviders : (TinyCLR_Spi_Provider**)&spiProviderDefs;

    return &spiApi;
}

bool STM32F4_Spi_Transaction_Start(int32_t controller) {
    switch (controller) {
        case 0:
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
            break; // enable SPI1 clock

        case 1:
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
            break; // enable SPI2 clock
#if TOTAL_SPI_CONTROLLERS > 2
        case 2:
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
            break; // enable SPI3 clock

        case 3:
            RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
            break; // enable SPI4 clock

        case 4:
            RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
            break; // enable SPI5 clock

        case 5:
            RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;
            break; // enable SPI6 clock
#endif
    }

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    // set mode bits
    uint32_t cr1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        cr1 |= SPI_CR1_DFF;
    }

    switch (g_SpiController[controller].Mode) {

        case TinyCLR_Spi_Mode::Mode0: // CPOL = 0, CPHA = 0.

            break;

        case TinyCLR_Spi_Mode::Mode1: // CPOL = 0, CPHA = 1.
            cr1 |= SPI_CR1_CPHA;
            break;

        case TinyCLR_Spi_Mode::Mode2: //  CPOL = 1, CPHA = 0.
            cr1 |= SPI_CR1_CPOL;
            break;

        case TinyCLR_Spi_Mode::Mode3: // CPOL = 1, CPHA = 1
            cr1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
            break;
    }

    // set clock prescaler
    uint32_t clock = SYSTEM_APB2_CLOCK_HZ / 2000; // SPI1 on APB2

    uint32_t clockKhz = g_SpiController[controller].ClockFrequency / 1000;

    if (controller > 0 && controller < 3) {
        clock = SYSTEM_APB1_CLOCK_HZ / 2000; // SPI2/3 on APB1
    }

    if (clock > clockKhz << 3) {
        clock >>= 4;
        cr1 |= SPI_CR1_BR_2;
    }

    if (clock > clockKhz << 1) {
        clock >>= 2;
        cr1 |= SPI_CR1_BR_1;
    }

    if (clock > clockKhz) {
        cr1 |= SPI_CR1_BR_0;
    }
    spi->CR1 = cr1;

    auto& sclk = g_STM32F4_Spi_Sclk_Pins[controller];
    auto& miso = g_STM32F4_Spi_Miso_Pins[controller];
    auto& mosi = g_STM32F4_Spi_Mosi_Pins[controller];

    STM32F4_Gpio_ConfigurePin(sclk.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, sclk.alternateFunction);
    STM32F4_Gpio_ConfigurePin(miso.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, miso.alternateFunction);
    STM32F4_Gpio_ConfigurePin(mosi.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, mosi.alternateFunction);

    // CS setup
    STM32F4_Gpio_ConfigurePin(g_SpiController[controller].ChipSelectLine, STM32F4_Gpio_PortMode::GeneralPurposeOutput, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);
    STM32F4_Gpio_WritePin(g_SpiController[controller].ChipSelectLine, false);

    return true;
}

bool STM32F4_Spi_Transaction_Stop(int32_t controller) {
    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    while (spi->SR & SPI_SR_BSY); // wait for completion

    spi->CR1 = 0; // disable SPI

    STM32F4_Gpio_WritePin(g_SpiController[controller].ChipSelectLine, true);

    auto& sclk = g_STM32F4_Spi_Sclk_Pins[controller];
    auto& miso = g_STM32F4_Spi_Miso_Pins[controller];
    auto& mosi = g_STM32F4_Spi_Mosi_Pins[controller];

    TinyCLR_Gpio_PinDriveMode res = TinyCLR_Gpio_PinDriveMode::InputPullDown;

    if (g_SpiController[controller].Mode == TinyCLR_Spi_Mode::Mode3)
        STM32F4_Gpio_ConfigurePin(sclk.number, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::PullUp, STM32F4_Gpio_AlternateFunction::AF0);
    else
        STM32F4_Gpio_ConfigurePin(sclk.number, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::PullDown, STM32F4_Gpio_AlternateFunction::AF0);

    STM32F4_Gpio_ConfigurePin(miso.number, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::PullDown, STM32F4_Gpio_AlternateFunction::AF0);
    STM32F4_Gpio_ConfigurePin(mosi.number, STM32F4_Gpio_PortMode::Input, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::Fast, STM32F4_Gpio_PullDirection::PullDown, STM32F4_Gpio_AlternateFunction::AF0);

    switch (controller) {
        case 0:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
            break; // disable SPI1 clock

        case 1:
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
            break; // disable SPI2 clock
#if TOTAL_SPI_CONTROLLERS > 2
        case 2:
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
            break; // disable SPI3 clock

        case 3:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
            break; // disable SPI4 clock

        case 4:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
            break; // disable SPI5 clock

        case 5:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
            break; // disable SPI6 clock
#endif
    }

    return true;
}


bool STM32F4_Spi_Transaction_nWrite8_nRead8(int32_t controller) {
    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    uint8_t* outBuf = g_SpiController[controller].writeBuffer;
    uint8_t* inBuf = g_SpiController[controller].readBuffer;
    int32_t outLen = g_SpiController[controller].writeLength;

    int32_t num, ii, i = 0;

    if (g_SpiController[controller].readLength) { // write & read
        num = g_SpiController[controller].readLength;
        ii = 0;
    }
    else { // write only
        num = outLen;
        ii = 0x80000000; // disable write to inBuf
    }

    uint8_t out = outBuf[0];
    uint16_t in;
    spi->DR = out; // write first word
    while (++i < num) {
        if (i < outLen) {
            out = outBuf[i]; // get new output data
        }

        while (!(spi->SR & SPI_SR_RXNE)) { /* wait for Rx buffer full */
        }

        in = spi->DR; // read input
        spi->DR = out; // start output

        if (ii >= 0) {
            inBuf[ii] = (uint8_t)in; // save input data
        }

        ii++;
    }

    while (!(spi->SR & SPI_SR_RXNE)) { /* wait for Rx buffer full */
    }

    in = spi->DR; // read last input

    if (ii >= 0) {
        inBuf[ii] = (uint8_t)in; // save last input
    }

    return true;
}

bool STM32F4_Spi_Transaction_nWrite16_nRead16(int32_t controller) {
    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    uint16_t* outBuf = (uint16_t*)g_SpiController[controller].writeBuffer;
    uint16_t* inBuf = (uint16_t*)g_SpiController[controller].readBuffer;
    int32_t outLen = (g_SpiController[controller].writeLength % 2) == 0 ? g_SpiController[controller].writeLength >> 1 : (g_SpiController[controller].writeLength >> 1) + 1;
    int32_t inLen = (g_SpiController[controller].readLength % 2) == 0 ? g_SpiController[controller].readLength >> 1 : (g_SpiController[controller].readLength >> 1) + 1;

    int32_t num, ii, i = 0;

    if (inLen) { // write & read
        num = inLen;
        ii = 0;
    }
    else { // write only
        num = outLen;
        ii = 0x80000000; // disable write to inBuf
    }

    uint16_t out = outBuf[0];
    uint16_t in;

    spi->DR = out; // write first word

    while (++i < num) {
        if (i < outLen) {
            out = outBuf[i]; // get new output data
        }

        while (!(spi->SR & SPI_SR_RXNE)) {
            /* wait for Rx buffer full */
        }

        in = spi->DR; // read input
        spi->DR = out; // start output

        if (ii >= 0)
            inBuf[ii] = in; // save input data

        ii++;
    }
    while (!(spi->SR & SPI_SR_RXNE)) {
        /* wait for Rx buffer full */
    }

    in = spi->DR; // read last input

    if (ii >= 0)
        inBuf[ii] = in; // save last input

    return true;
}


TinyCLR_Result STM32F4_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (STM32F4_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return STM32F4_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result STM32F4_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F4_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = readBuffer;
    g_SpiController[controller].readLength = readLength;
    g_SpiController[controller].writeBuffer = (uint8_t*)writeBuffer;
    g_SpiController[controller].writeLength = writeLength;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!STM32F4_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!STM32F4_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!STM32F4_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length) {

    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F4_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = buffer;
    g_SpiController[controller].readLength = length;
    g_SpiController[controller].writeBuffer = nullptr;
    g_SpiController[controller].writeLength = 0;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!STM32F4_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!STM32F4_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!STM32F4_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length) {

    int32_t controller = self->Index;

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F4_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = nullptr;
    g_SpiController[controller].readLength = 0;
    g_SpiController[controller].writeBuffer = (uint8_t*)buffer;
    g_SpiController[controller].writeLength = length;

    if (g_SpiController[controller].DataBitLength == DATA_BIT_LENGTH_16) {
        if (!STM32F4_Spi_Transaction_nWrite16_nRead16(controller))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!STM32F4_Spi_Transaction_nWrite8_nRead8(controller))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!STM32F4_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    int32_t controller = (self->Index);

    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].ChipSelectLine = chipSelectLine;
    g_SpiController[controller].ClockFrequency = clockFrequency;
    g_SpiController[controller].DataBitLength = dataBitLength;
    g_SpiController[controller].Mode = mode;

    if (g_SpiController[controller].ChipSelectLine == PIN_NONE) // For case no need CS. CS always high
        return TinyCLR_Result::Success;

    if (STM32F4_Gpio_OpenPin(g_SpiController[controller].ChipSelectLine))
        return TinyCLR_Result::Success;

    return TinyCLR_Result::NotAvailable;
}

TinyCLR_Result STM32F4_Spi_Acquire(const TinyCLR_Spi_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t controller = (self->Index);

    auto& sclk = g_STM32F4_Spi_Sclk_Pins[controller];
    auto& miso = g_STM32F4_Spi_Miso_Pins[controller];
    auto& mosi = g_STM32F4_Spi_Mosi_Pins[controller];

    g_SpiController[controller].ChipSelectLine = PIN_NONE;

    // Check each pin single time make sure once fail not effect to other pins
    if (STM32F4_Gpio_OpenPin(sclk.number) && STM32F4_Gpio_OpenPin(miso.number) && STM32F4_Gpio_OpenPin(mosi.number))
        return TinyCLR_Result::Success;

    return TinyCLR_Result::SharingViolation;
}

TinyCLR_Result STM32F4_Spi_Release(const TinyCLR_Spi_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t controller = (self->Index);

    auto& sclk = g_STM32F4_Spi_Sclk_Pins[controller];
    auto& miso = g_STM32F4_Spi_Miso_Pins[controller];
    auto& mosi = g_STM32F4_Spi_Mosi_Pins[controller];

    switch (controller) {
        case 0:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
            break; // disable SPI1 clock

        case 1:
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
            break; // disable SPI2 clock
#if TOTAL_SPI_CONTROLLERS > 2
        case 2:
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
            break; // disable SPI3 clock

        case 3:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
            break; // disable SPI4 clock

        case 4:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
            break; // disable SPI5 clock

        case 5:
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
            break; // disable SPI6 clock
#endif
    }

    STM32F4_Gpio_ClosePin(sclk.number);
    STM32F4_Gpio_ClosePin(miso.number);
    STM32F4_Gpio_ClosePin(mosi.number);

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self) {
    // Theoretically this could read the Clock and PLL configurations
    // to determine an actual realistic minimum, however there doesn't
    // seem to be a lot of value in that since the CPU_SPI_Xaction_Start
    // has to determine the applicability of the selected speed at the
    // time a transaction is started anyway.
    return 1;
}

int32_t STM32F4_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self) {
    // Theoretically this could read the Clock and PLL configurations
    // to determine an actual realistic maximum, however there doesn't
    // seem to be a lot of value in that since the CPU_SPI_Xaction_Start
    // has to determine the applicability of the selected speed at the
    // time a transaction is started anyway.
    // Max supported (e.g. not overclocked) AHB speed / 2
    return 48000000;
}

int32_t STM32F4_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self) {
    // This could maintain a map of the actual pins
    // that are available for a particular port.
    // (Not all pins can be mapped to all ports.)
    // The value of doing that, however, is marginal
    // since the count of possible chip selects doesn't
    // really help in determining which chip select to
    // use so just report the total count of all GPIO
    // pins as possible so that the selected Chip select
    // line coresponds to a GPIO pin number directly
    // without needing any additional translation/mapping.
    return TOTAL_GPIO_PINS;
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result STM32F4_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void STM32F4_Spi_Reset() {
    if (TOTAL_SPI_CONTROLLERS > 0) g_STM32_Spi_Port[0] = SPI1;
    if (TOTAL_SPI_CONTROLLERS > 1) g_STM32_Spi_Port[1] = SPI2;
    if (TOTAL_SPI_CONTROLLERS > 2) g_STM32_Spi_Port[2] = SPI3;
    if (TOTAL_SPI_CONTROLLERS > 3) g_STM32_Spi_Port[3] = SPI4;
    if (TOTAL_SPI_CONTROLLERS > 4) g_STM32_Spi_Port[4] = SPI5;
    if (TOTAL_SPI_CONTROLLERS > 5) g_STM32_Spi_Port[5] = SPI6;

    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        int32_t controller = i;

        STM32F4_Spi_Release(spiProviders[controller]);

        if (g_SpiController[controller].ChipSelectLine != PIN_NONE)
            STM32F4_Gpio_ClosePin(g_SpiController[controller].ChipSelectLine);
    }
}