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

#include "STM32F7.h"
#include <string.h>

bool STM32F7_Spi_Transaction_Start(int32_t controllerIndex);
bool STM32F7_Spi_Transaction_Stop(int32_t controllerIndex);
bool STM32F7_Spi_Transaction_nWrite8_nRead8(int32_t controllerIndex);

typedef  SPI_TypeDef* ptr_SPI_TypeDef;

#define DATA_BIT_LENGTH_16  16
#define DATA_BIT_LENGTH_8   8

static const STM32F7_Gpio_Pin g_STM32F7_Spi_Sclk_Pins[] = STM32F7_SPI_SCLK_PINS;
static const STM32F7_Gpio_Pin g_STM32F7_Spi_Miso_Pins[] = STM32F7_SPI_MISO_PINS;
static const STM32F7_Gpio_Pin g_STM32F7_Spi_Mosi_Pins[] = STM32F7_SPI_MOSI_PINS;

static const int TOTAL_SPI_CONTROLLERS = SIZEOF_ARRAY(g_STM32F7_Spi_Sclk_Pins);

static ptr_SPI_TypeDef g_STM32_Spi_Port[TOTAL_SPI_CONTROLLERS];

struct SpiDriver {
    int32_t controllerIndex;

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

static SpiDriver spiDrivers[TOTAL_SPI_CONTROLLERS];

static TinyCLR_Spi_Controller spiControllers[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi[TOTAL_SPI_CONTROLLERS];

const TinyCLR_Api_Info* STM32F7_Spi_GetApi() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiControllers[i].ApiInfo = &spiApi[i];
        spiControllers[i].Acquire = &STM32F7_Spi_Acquire;
        spiControllers[i].Release = &STM32F7_Spi_Release;
        spiControllers[i].SetActiveSettings = &STM32F7_Spi_SetActiveSettings;
        spiControllers[i].Read = &STM32F7_Spi_Read;
        spiControllers[i].Write = &STM32F7_Spi_Write;
        spiControllers[i].TransferFullDuplex = &STM32F7_Spi_TransferFullDuplex;
        spiControllers[i].TransferSequential = &STM32F7_Spi_TransferSequential;
        spiControllers[i].GetChipSelectLineCount = &STM32F7_Spi_GetChipSelectLineCount;
        spiControllers[i].GetMinClockFrequency = &STM32F7_Spi_GetMinClockFrequency;
        spiControllers[i].GetMaxClockFrequency = &STM32F7_Spi_GetMaxClockFrequency;
        spiControllers[i].GetSupportedDataBitLengths = &STM32F7_Spi_GetSupportedDataBitLengths;

        spiApi[i].Author = "GHI Electronics, LLC";
        spiApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.SpiController";
        spiApi[i].Type = TinyCLR_Api_Type::SpiController;
        spiApi[i].Version = 0;
        spiApi[i].Implementation = &spiControllers[i];
        spiApi[i].State = &spiDrivers[i];

        spiDrivers[i].controllerIndex = i;
    }

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
    return (const TinyCLR_Api_Info*)&spiApi;
}

bool STM32F7_Spi_Transaction_Start(int32_t controllerIndex) {
    auto driver = &spiDrivers[controllerIndex];

    STM32F7_GpioInternal_WritePin(driver->chipSelectLine, false);

    STM32F7_Time_Delay(nullptr, ((1000000 / (driver->clockFrequency / 1000)) / 1000));

    return true;
}

bool STM32F7_Spi_Transaction_Stop(int32_t controllerIndex) {
    auto driver = &spiDrivers[controllerIndex];

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controllerIndex];

    while (spi->SR & SPI_SR_BSY); // wait for completion

    STM32F7_Time_Delay(nullptr, ((1000000 / (driver->clockFrequency / 1000)) / 1000));

    STM32F7_GpioInternal_WritePin(driver->chipSelectLine, true);

    return true;
}


bool STM32F7_Spi_Transaction_nWrite8_nRead8(int32_t controllerIndex) {
    auto driver = &spiDrivers[controllerIndex];

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controllerIndex];

    uint8_t* outBuf = driver->writeBuffer;
    uint8_t* inBuf = driver->readBuffer;
    int32_t outLen = driver->writeLength;
    int32_t inLen = driver->readLength;

    int32_t num = outLen > inLen ? outLen : inLen;
    int32_t i = 0;
    int32_t ii = 0;
    uint8_t out = outLen > 0 ? outBuf[0] : 0;
    uint8_t in;

    volatile uint8_t *dataReg = reinterpret_cast<uint8_t*>((uint32_t)&spi->DR);

    while (!(spi->SR & SPI_SR_TXE)); // wait for Tx empty

    *dataReg = out; // write first word

    while (++i < num) {
        if (i < outLen) {
            out = outBuf[i]; // get new output data
        }
        else {
            out = 0;
        }
        while (!(spi->SR & SPI_SR_RXNE));

        in = *dataReg; // read input

        while (!(spi->SR & SPI_SR_TXE)); // wait for Tx empty

        *dataReg = out; // start output

        if (ii < inLen) {
            inBuf[ii] = (uint8_t)in; // save input data
        }

        ii++;
    }

    while (!(spi->SR & SPI_SR_RXNE));

    in = *dataReg; // read last input

    if (ii < inLen) {
        inBuf[ii] = (uint8_t)in; // save last input
    }

    return true;
}

TinyCLR_Result STM32F7_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (STM32F7_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return STM32F7_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result STM32F7_Spi_TransferFullDuplex(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (!STM32F7_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    driver->readBuffer = readBuffer;
    driver->readLength = readLength;
    driver->writeBuffer = (uint8_t*)writeBuffer;
    driver->writeLength = writeLength;

    if (!STM32F7_Spi_Transaction_nWrite8_nRead8(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F7_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length) {
    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (!STM32F7_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    driver->readBuffer = buffer;
    driver->readLength = length;
    driver->writeBuffer = nullptr;
    driver->writeLength = 0;

    if (!STM32F7_Spi_Transaction_nWrite8_nRead8(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F7_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length) {
    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (!STM32F7_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    driver->readBuffer = nullptr;
    driver->readLength = 0;
    driver->writeBuffer = (uint8_t*)buffer;
    driver->writeLength = length;

    if (!STM32F7_Spi_Transaction_nWrite8_nRead8(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F7_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (driver->chipSelectLine == chipSelectLine
        && driver->dataBitLength == dataBitLength
        && driver->spiMode == mode
        && driver->clockFrequency == clockFrequency) {
        return TinyCLR_Result::Success;
    }

    driver->chipSelectLine = chipSelectLine;
    driver->dataBitLength = dataBitLength;
    driver->spiMode = mode;
    driver->clockFrequency = clockFrequency;

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controllerIndex];

    uint32_t cr1 = SPI_CR1_CRCL | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    // Clear configuration
    spi->CR1 &= ~cr1;
    spi->CR2 &= ~SPI_CR2_FRXTH;

    cr1 = 0;
    // set new configuration
    spi->CR2 |= SPI_CR2_FRXTH;
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
    uint32_t clock = STM32F7_APB2_CLOCK_HZ / 2000; // SPI1 on APB2

    uint32_t clockKhz = clockFrequency / 1000;

    if (controllerIndex > 0 && controllerIndex < 3) {
        clock = STM32F7_APB1_CLOCK_HZ / 2000; // SPI2/3 on APB1
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

    if (STM32F7_GpioInternal_OpenPin(driver->chipSelectLine)) {
        // CS setup
        STM32F7_GpioInternal_ConfigurePin(driver->chipSelectLine, STM32F7_Gpio_PortMode::GeneralPurposeOutput, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, STM32F7_Gpio_AlternateFunction::AF0);

        STM32F7_GpioInternal_WritePin(driver->chipSelectLine, true);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Acquire(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (controllerIndex >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    auto& sclk = g_STM32F7_Spi_Sclk_Pins[controllerIndex];
    auto& miso = g_STM32F7_Spi_Miso_Pins[controllerIndex];
    auto& mosi = g_STM32F7_Spi_Mosi_Pins[controllerIndex];

    driver->chipSelectLine = PIN_NONE;
    driver->dataBitLength = 0;
    driver->spiMode = TinyCLR_Spi_Mode::Mode0;
    driver->clockFrequency = 0;

    // Check each pin single time make sure once fail not effect to other pins
    if (!STM32F7_GpioInternal_OpenPin(sclk.number) || !STM32F7_GpioInternal_OpenPin(miso.number) || !STM32F7_GpioInternal_OpenPin(mosi.number)) {
        return TinyCLR_Result::SharingViolation;
    }

    switch (controllerIndex) {
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

    ptr_SPI_TypeDef spi = g_STM32_Spi_Port[controllerIndex];


    spi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;

    STM32F7_GpioInternal_ConfigurePin(sclk.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, sclk.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(miso.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, miso.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(mosi.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, mosi.alternateFunction);

    driver->isOpened = true;
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Release(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    switch (controllerIndex) {
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

    if (driver->isOpened) {
        auto& sclk = g_STM32F7_Spi_Sclk_Pins[controllerIndex];
        auto& miso = g_STM32F7_Spi_Miso_Pins[controllerIndex];
        auto& mosi = g_STM32F7_Spi_Mosi_Pins[controllerIndex];

        STM32F7_GpioInternal_ClosePin(sclk.number);
        STM32F7_GpioInternal_ClosePin(miso.number);
        STM32F7_GpioInternal_ClosePin(mosi.number);

        if (driver->chipSelectLine != PIN_NONE) {
            STM32F7_GpioInternal_ClosePin(driver->chipSelectLine);

            driver->chipSelectLine = PIN_NONE;
        }
    }

    driver->isOpened = false;

    return TinyCLR_Result::Success;
}

int32_t STM32F7_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self) {
    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    return ((controllerIndex > 0 && controllerIndex < 3) ? (STM32F7_APB1_CLOCK_HZ / 256) : (STM32F7_APB2_CLOCK_HZ / 256));
}

int32_t STM32F7_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self) {
    auto driver = reinterpret_cast<SpiDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    return ((controllerIndex > 0 && controllerIndex < 3) ? (STM32F7_APB1_CLOCK_HZ >> 1) : (STM32F7_APB2_CLOCK_HZ >> 1));
}

int32_t STM32F7_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self) {
    return STM32F7_Gpio_GetPinCount(nullptr);
}

static const int32_t STM32F7_SPI_DATA_BITS_COUNT = 1;

TinyCLR_Result STM32F7_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        dataBitLengths[0] = 8;

    dataBitLengthsCount = STM32F7_SPI_DATA_BITS_COUNT;

    return TinyCLR_Result::Success;
}

void STM32F7_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        STM32F7_Spi_Release(&spiControllers[i]);

        spiDrivers[i].isOpened = false;
    }
}
