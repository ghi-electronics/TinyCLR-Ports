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
#include <string.h>

bool STM32F4_Spi_Transaction_Start(int32_t controller);
bool STM32F4_Spi_Transaction_Stop(int32_t controller);
bool STM32F4_Spi_Transaction_nWrite8_nRead8(int32_t controller);

typedef  SPI_TypeDef* ptr_SPI_TypeDef;

#define DATA_BIT_LENGTH_16  16
#define DATA_BIT_LENGTH_8   8

static const STM32F4_Gpio_Pin g_STM32F4_Spi_Sclk_Pins[] = STM32F4_SPI_SCLK_PINS;
static const STM32F4_Gpio_Pin g_STM32F4_Spi_Miso_Pins[] = STM32F4_SPI_MISO_PINS;
static const STM32F4_Gpio_Pin g_STM32F4_Spi_Mosi_Pins[] = STM32F4_SPI_MOSI_PINS;

static const int TOTAL_SPI_CONTROLLERS = SIZEOF_ARRAY(g_STM32F4_Spi_Sclk_Pins);

static ptr_SPI_TypeDef g_STM32_Spi_Port[TOTAL_SPI_CONTROLLERS];

struct SpiController {
    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t readLength;
    size_t writeLength;

    int32_t chipSelectLine;
    int32_t dataBitLength;
    int32_t clockFrequency;

    bool isOpened;

    TinyCLR_Spi_Mode spiMode;
};

static SpiController g_SpiController[TOTAL_SPI_CONTROLLERS];

static TinyCLR_Spi_Controller spiProviders;
static TinyCLR_Api_Info spiApi;

const TinyCLR_Api_Info* STM32F4_Spi_GetApi() {
    spiProviders.ApiInfo = &spiApi;
    spiProviders.Acquire = &STM32F4_Spi_Acquire;
    spiProviders.Release = &STM32F4_Spi_Release;
    spiProviders.SetActiveSettings = &STM32F4_Spi_SetActiveSettings;
    spiProviders.Read = &STM32F4_Spi_Read;
    spiProviders.Write = &STM32F4_Spi_Write;
    spiProviders.TransferFullDuplex = &STM32F4_Spi_TransferFullDuplex;
    spiProviders.TransferSequential = &STM32F4_Spi_TransferSequential;
    spiProviders.GetChipSelectLineCount = &STM32F4_Spi_GetChipSelectLineCount;
    spiProviders.GetMinClockFrequency = &STM32F4_Spi_GetMinClockFrequency;
    spiProviders.GetMaxClockFrequency = &STM32F4_Spi_GetMaxClockFrequency;
    spiProviders.GetSupportedDataBitLengths = &STM32F4_Spi_GetSupportedDataBitLengths;
    spiProviders.GetControllerCount = &STM32F4_Spi_GetControllerCount;

    spiApi.Author = "GHI Electronics, LLC";
    spiApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.SpiProvider";
    spiApi.Type = TinyCLR_Api_Type::SpiProvider;
    spiApi.Version = 0;
    spiApi.Implementation = &spiProviders;

#ifdef SPI1
    if (TOTAL_SPI_CONTROLLERS > 0) g_STM32_Spi_Port[0] = SPI1;
#ifdef SPI2
    if (TOTAL_SPI_CONTROLLERS > 1) g_STM32_Spi_Port[1] = SPI2;
#ifdef SPI3
    if (TOTAL_SPI_CONTROLLERS > 2) g_STM32_Spi_Port[2] = SPI3;
#ifdef SPI4
    if (TOTAL_SPI_CONTROLLERS > 3) g_STM32_Spi_Port[3] = SPI4;
#ifdef SPI5
    if (TOTAL_SPI_CONTROLLERS > 4) g_STM32_Spi_Port[4] = SPI5;
#ifdef SPI6
    if (TOTAL_SPI_CONTROLLERS > 5) g_STM32_Spi_Port[5] = SPI6;
#endif
#endif
#endif
#endif
#endif
#endif
    return &spiApi;
}

bool STM32F4_Spi_Transaction_Start(int32_t controller) {

    STM32F4_GpioInternal_WritePin(g_SpiController[controller].chipSelectLine, false);

    STM32F4_Time_Delay(nullptr, ((1000000 / (g_SpiController[controller].clockFrequency / 1000)) / 1000));

    return true;
}

bool STM32F4_Spi_Transaction_Stop(int32_t controller) {
    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    while (spi->SR & SPI_SR_BSY); // wait for completion

    STM32F4_Time_Delay(nullptr, ((1000000 / (g_SpiController[controller].clockFrequency / 1000)) / 1000));

    STM32F4_GpioInternal_WritePin(g_SpiController[controller].chipSelectLine, true);

    return true;
}


bool STM32F4_Spi_Transaction_nWrite8_nRead8(int32_t controller) {
    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    uint8_t* outBuf = g_SpiController[controller].writeBuffer;
    uint8_t* inBuf = g_SpiController[controller].readBuffer;
    int32_t outLen = g_SpiController[controller].writeLength;
    int32_t inLen = g_SpiController[controller].readLength;

    int32_t num = outLen > inLen ? outLen : inLen;
    int32_t i = 0;
    int32_t ii = 0;
    uint8_t out = outLen > 0 ? outBuf[0] : 0;
    uint8_t in;

    while (!(spi->SR & SPI_SR_TXE)); // wait for Tx empty

    spi->DR = out; // write first word

    while (++i < num) {
        if (i < outLen) {
            out = outBuf[i]; // get new output data
        }
        else {
            out = 0;
        }

        while (!(spi->SR & SPI_SR_RXNE));

        in = spi->DR; // read input

        while (!(spi->SR & SPI_SR_TXE)); // wait for Tx empty

        spi->DR = out; // start output

        if (ii < inLen) {
            inBuf[ii] = (uint8_t)in; // save input data
        }

        ii++;
    }

    while (!(spi->SR & SPI_SR_RXNE));

    in = spi->DR; // read last input

    if (ii < inLen) {
        inBuf[ii] = (uint8_t)in; // save last input
    }

    return true;
}

TinyCLR_Result STM32F4_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, int32_t controller, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (STM32F4_Spi_Write(self, controller, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return STM32F4_Spi_Read(self, controller, readBuffer, readLength);
}

TinyCLR_Result STM32F4_Spi_TransferFullDuplex(const TinyCLR_Spi_Controller* self, int32_t controller, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {


    if (!STM32F4_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = readBuffer;
    g_SpiController[controller].readLength = readLength;
    g_SpiController[controller].writeBuffer = (uint8_t*)writeBuffer;
    g_SpiController[controller].writeLength = writeLength;

    if (!STM32F4_Spi_Transaction_nWrite8_nRead8(controller))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F4_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_Read(const TinyCLR_Spi_Controller* self, int32_t controller, uint8_t* buffer, size_t& length) {


    if (!STM32F4_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = buffer;
    g_SpiController[controller].readLength = length;
    g_SpiController[controller].writeBuffer = nullptr;
    g_SpiController[controller].writeLength = 0;

    if (!STM32F4_Spi_Transaction_nWrite8_nRead8(controller))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F4_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_Write(const TinyCLR_Spi_Controller* self, int32_t controller, const uint8_t* buffer, size_t& length) {


    if (!STM32F4_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = nullptr;
    g_SpiController[controller].readLength = 0;
    g_SpiController[controller].writeBuffer = (uint8_t*)buffer;
    g_SpiController[controller].writeLength = length;

    if (!STM32F4_Spi_Transaction_nWrite8_nRead8(controller))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F4_Spi_Transaction_Stop(controller))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, int32_t controller, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {


    if (g_SpiController[controller].chipSelectLine == chipSelectLine
        && g_SpiController[controller].dataBitLength == dataBitLength
        && g_SpiController[controller].spiMode == mode
        && g_SpiController[controller].clockFrequency == clockFrequency) {
        return TinyCLR_Result::Success;
    }

    g_SpiController[controller].chipSelectLine = chipSelectLine;
    g_SpiController[controller].dataBitLength = dataBitLength;
    g_SpiController[controller].spiMode = mode;
    g_SpiController[controller].clockFrequency = clockFrequency;

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];


    uint32_t cr1 = SPI_CR1_DFF | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    // Clear configuration
    spi->CR1 &= ~cr1;

    cr1 = 0;

    switch (mode) {

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
    uint32_t clock = STM32F4_APB2_CLOCK_HZ / 2000; // SPI1 on APB2

    uint32_t clockKhz = clockFrequency / 1000;

    if (controller > 0 && controller < 3) {
        clock = STM32F4_APB1_CLOCK_HZ / 2000; // SPI2/3 on APB1
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

    spi->CR1 |= cr1;

    if (STM32F4_GpioInternal_OpenPin(g_SpiController[controller].chipSelectLine)) {
        // CS setup
        STM32F4_GpioInternal_ConfigurePin(g_SpiController[controller].chipSelectLine, STM32F4_Gpio_PortMode::GeneralPurposeOutput, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);

        STM32F4_GpioInternal_WritePin(g_SpiController[controller].chipSelectLine, true);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_Acquire(const TinyCLR_Spi_Controller* self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;



    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    auto& sclk = g_STM32F4_Spi_Sclk_Pins[controller];
    auto& miso = g_STM32F4_Spi_Miso_Pins[controller];
    auto& mosi = g_STM32F4_Spi_Mosi_Pins[controller];

    g_SpiController[controller].chipSelectLine = PIN_NONE;
    g_SpiController[controller].dataBitLength = 0;
    g_SpiController[controller].spiMode = TinyCLR_Spi_Mode::Mode0;
    g_SpiController[controller].clockFrequency = 0;

    // Check each pin single time make sure once fail not effect to other pins
    if (!STM32F4_GpioInternal_OpenPin(sclk.number) || !STM32F4_GpioInternal_OpenPin(miso.number) || !STM32F4_GpioInternal_OpenPin(mosi.number)) {
        return TinyCLR_Result::SharingViolation;
    }

    switch (controller) {
#ifdef SPI1
    case 0:
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        break; // enable SPI1 clock

#ifdef SPI2
    case 1:
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        break; // enable SPI2 clock

#ifdef SPI3
    case 2:
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        break; // enable SPI3 clock

#ifdef SPI4
    case 3:
        RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
        break; // enable SPI4 clock

#ifdef SPI5
    case 4:
        RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
        break; // enable SPI5 clock

#ifdef SPI6
    case 5:
        RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;
        break; // enable SPI6 clock
#endif
#endif
#endif
#endif
#endif
#endif
    }

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controller];

    spi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;;

    STM32F4_GpioInternal_ConfigurePin(sclk.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, sclk.alternateFunction);
    STM32F4_GpioInternal_ConfigurePin(miso.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, miso.alternateFunction);
    STM32F4_GpioInternal_ConfigurePin(mosi.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::None, mosi.alternateFunction);

    g_SpiController[controller].isOpened = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Spi_Release(const TinyCLR_Spi_Controller* self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;



    switch (controller) {
#ifdef SPI1
    case 0:
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
        break; // disable SPI1 clock

#ifdef SPI2
    case 1:
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
        break; // disable SPI2 clock

#ifdef SPI3
    case 2:
        RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
        break; // disable SPI3 clock

#ifdef SPI4
    case 3:
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
        break; // disable SPI4 clock

#ifdef SPI5
    case 4:
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
        break; // disable SPI5 clock

#ifdef SPI6
    case 5:
        RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
        break; // disable SPI6 clock
#endif
#endif
#endif
#endif
#endif
#endif
    }

    if (g_SpiController[controller].isOpened) {
        auto& sclk = g_STM32F4_Spi_Sclk_Pins[controller];
        auto& miso = g_STM32F4_Spi_Miso_Pins[controller];
        auto& mosi = g_STM32F4_Spi_Mosi_Pins[controller];

        STM32F4_GpioInternal_ClosePin(sclk.number);
        STM32F4_GpioInternal_ClosePin(miso.number);
        STM32F4_GpioInternal_ClosePin(mosi.number);

        if (g_SpiController[controller].chipSelectLine != PIN_NONE) {
            STM32F4_GpioInternal_ClosePin(g_SpiController[controller].chipSelectLine);

            g_SpiController[controller].chipSelectLine = PIN_NONE;
        }
    }

    g_SpiController[controller].isOpened = false;

    return TinyCLR_Result::Success;
}

int32_t STM32F4_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self, int32_t controller) {
    return ((controller > 0 && controller < 3) ? (STM32F4_APB1_CLOCK_HZ / 256) : (STM32F4_APB2_CLOCK_HZ / 256));
}

int32_t STM32F4_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self, int32_t controller) {
    return ((controller > 0 && controller < 3) ? (STM32F4_APB1_CLOCK_HZ >> 1) : (STM32F4_APB2_CLOCK_HZ >> 1));
}

int32_t STM32F4_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self, int32_t controller) {
    auto gpioController = 0; //TODO Temporary set to 0

    return STM32F4_Gpio_GetPinCount(nullptr, gpioController);
}

static const int32_t STM32F4_SPI_DATA_BITS_COUNT = 1;

TinyCLR_Result STM32F4_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, int32_t controller, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        dataBitLengths[0] = 8;

    dataBitLengthsCount = STM32F4_SPI_DATA_BITS_COUNT;

    return TinyCLR_Result::Success;
}

void STM32F4_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        STM32F4_Spi_Release(&spiProviders, i);

        g_SpiController[i].isOpened = false;
    }
}

TinyCLR_Result STM32F4_Spi_GetControllerCount(const TinyCLR_Spi_Controller* self, int32_t& count) {
    count = TOTAL_SPI_CONTROLLERS;

    return TinyCLR_Result::Success;
}