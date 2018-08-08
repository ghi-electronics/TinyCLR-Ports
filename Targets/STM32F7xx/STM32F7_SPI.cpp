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

static const STM32F7_Gpio_Pin spiClkPins[] = STM32F7_SPI_SCLK_PINS;
static const STM32F7_Gpio_Pin spiMisoPins[] = STM32F7_SPI_MISO_PINS;
static const STM32F7_Gpio_Pin spiMosiPins[] = STM32F7_SPI_MOSI_PINS;

static ptr_SPI_TypeDef spiPortRegs[TOTAL_SPI_CONTROLLERS];

struct SpiState {
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

    uint32_t initializeCount;
};

static SpiState spiStates[TOTAL_SPI_CONTROLLERS];

static TinyCLR_Spi_Controller spiControllers[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi[TOTAL_SPI_CONTROLLERS];

const char* spiApiNames[TOTAL_SPI_CONTROLLERS] = {
#if TOTAL_SPI_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.STM32F7.SpiController\\0",
#if TOTAL_SPI_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.STM32F7.SpiController\\1",
#if TOTAL_SPI_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.STM32F7.SpiController\\2",
#if TOTAL_SPI_CONTROLLERS > 3
"GHIElectronics.TinyCLR.NativeApis.STM32F7.SpiController\\3",
#if TOTAL_SPI_CONTROLLERS > 4
"GHIElectronics.TinyCLR.NativeApis.STM32F7.SpiController\\4",
#endif
#endif
#endif
#endif
#endif
};

void STM32F7_Spi_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiControllers[i].ApiInfo = &spiApi[i];
        spiControllers[i].Acquire = &STM32F7_Spi_Acquire;
        spiControllers[i].Release = &STM32F7_Spi_Release;
        spiControllers[i].WriteRead = &STM32F7_Spi_WriteRead;
        spiControllers[i].SetActiveSettings = &STM32F7_Spi_SetActiveSettings;
        spiControllers[i].GetChipSelectLineCount = &STM32F7_Spi_GetChipSelectLineCount;
        spiControllers[i].GetMinClockFrequency = &STM32F7_Spi_GetMinClockFrequency;
        spiControllers[i].GetMaxClockFrequency = &STM32F7_Spi_GetMaxClockFrequency;
        spiControllers[i].GetSupportedDataBitLengths = &STM32F7_Spi_GetSupportedDataBitLengths;

        spiApi[i].Author = "GHI Electronics, LLC";
        spiApi[i].Name = spiApiNames[i];
        spiApi[i].Type = TinyCLR_Api_Type::SpiController;
        spiApi[i].Version = 0;
        spiApi[i].Implementation = &spiControllers[i];
        spiApi[i].State = &spiStates[i];

        spiStates[i].controllerIndex = i;

        apiManager->Add(apiManager, &spiApi[i]);
    }

#ifdef SPI1
    if (TOTAL_SPI_CONTROLLERS > 0) spiPortRegs[0] = SPI1;
#ifdef SPI2
    if (TOTAL_SPI_CONTROLLERS > 1) spiPortRegs[1] = SPI2;
#ifdef SPI3
    if (TOTAL_SPI_CONTROLLERS > 2) spiPortRegs[2] = SPI3;
#ifdef SPI4
    if (TOTAL_SPI_CONTROLLERS > 3) spiPortRegs[3] = SPI4;
#ifdef SPI5
    if (TOTAL_SPI_CONTROLLERS > 4) spiPortRegs[4] = SPI5;
#ifdef SPI6
    if (TOTAL_SPI_CONTROLLERS > 5) spiPortRegs[5] = SPI6;
#endif
#endif
#endif
#endif
#endif
#endif

}

bool STM32F7_Spi_Transaction_Start(int32_t controllerIndex) {
    auto state = &spiStates[controllerIndex];

    STM32F7_GpioInternal_WritePin(state->chipSelectLine, false);

    STM32F7_Time_Delay(nullptr, ((1000000 / (state->clockFrequency / 1000)) / 1000));

    return true;
}

bool STM32F7_Spi_Transaction_Stop(int32_t controllerIndex) {
    auto state = &spiStates[controllerIndex];

    ptr_SPI_TypeDef spi = spiPortRegs[controllerIndex];

    while (spi->SR & SPI_SR_BSY); // wait for completion

    STM32F7_Time_Delay(nullptr, ((1000000 / (state->clockFrequency / 1000)) / 1000));

    STM32F7_GpioInternal_WritePin(state->chipSelectLine, true);

    return true;
}


bool STM32F7_Spi_Transaction_nWrite8_nRead8(int32_t controllerIndex) {
    auto state = &spiStates[controllerIndex];

    ptr_SPI_TypeDef spi = spiPortRegs[controllerIndex];

    uint8_t* outBuf = state->writeBuffer;
    uint8_t* inBuf = state->readBuffer;
    int32_t outLen = state->writeLength;
    int32_t inLen = state->readLength;

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

TinyCLR_Result STM32F7_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter) {
    if (STM32F7_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return STM32F7_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result STM32F7_Spi_WriteRead(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (!STM32F7_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    state->readBuffer = readBuffer;
    state->readLength = readLength;
    state->writeBuffer = (uint8_t*)writeBuffer;
    state->writeLength = writeLength;

    if (!STM32F7_Spi_Transaction_nWrite8_nRead8(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F7_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (!STM32F7_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    state->readBuffer = buffer;
    state->readLength = length;
    state->writeBuffer = nullptr;
    state->writeLength = 0;

    if (!STM32F7_Spi_Transaction_nWrite8_nRead8(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F7_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (!STM32F7_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    state->readBuffer = nullptr;
    state->readLength = 0;
    state->writeBuffer = (uint8_t*)buffer;
    state->writeLength = length;

    if (!STM32F7_Spi_Transaction_nWrite8_nRead8(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    if (!STM32F7_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, uint32_t chipSelectLine, bool useControllerChipSelect, uint32_t clockFrequency, uint32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->chipSelectLine == chipSelectLine
        && state->dataBitLength == dataBitLength
        && state->spiMode == mode
        && state->clockFrequency == clockFrequency) {
        return TinyCLR_Result::Success;
    }

    state->chipSelectLine = chipSelectLine;
    state->dataBitLength = dataBitLength;
    state->spiMode = mode;
    state->clockFrequency = clockFrequency;

    ptr_SPI_TypeDef spi = spiPortRegs[controllerIndex];

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

    if (STM32F7_GpioInternal_OpenPin(state->chipSelectLine)) {
        // CS setup
        STM32F7_GpioInternal_ConfigurePin(state->chipSelectLine, STM32F7_Gpio_PortMode::GeneralPurposeOutput, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, STM32F7_Gpio_AlternateFunction::AF0);

        STM32F7_GpioInternal_WritePin(state->chipSelectLine, true);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Acquire(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {

        auto controllerIndex = state->controllerIndex;

        if (controllerIndex >= TOTAL_SPI_CONTROLLERS)
            return TinyCLR_Result::InvalidOperation;

        auto& sclk = spiClkPins[controllerIndex];
        auto& miso = spiMisoPins[controllerIndex];
        auto& mosi = spiMosiPins[controllerIndex];

        state->chipSelectLine = PIN_NONE;
        state->dataBitLength = 0;
        state->spiMode = TinyCLR_Spi_Mode::Mode0;
        state->clockFrequency = 0;

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

        ptr_SPI_TypeDef spi = spiPortRegs[controllerIndex];


        spi->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_SPE;

        STM32F7_GpioInternal_ConfigurePin(sclk.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, sclk.alternateFunction);
        STM32F7_GpioInternal_ConfigurePin(miso.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, miso.alternateFunction);
        STM32F7_GpioInternal_ConfigurePin(mosi.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::PushPull, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::None, mosi.alternateFunction);

        state->isOpened = true;
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Spi_Release(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

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

        if (state->isOpened) {
            auto& sclk = spiClkPins[controllerIndex];
            auto& miso = spiMisoPins[controllerIndex];
            auto& mosi = spiMosiPins[controllerIndex];

            STM32F7_GpioInternal_ClosePin(sclk.number);
            STM32F7_GpioInternal_ClosePin(miso.number);
            STM32F7_GpioInternal_ClosePin(mosi.number);

            if (state->chipSelectLine != PIN_NONE) {
                STM32F7_GpioInternal_ClosePin(state->chipSelectLine);

                state->chipSelectLine = PIN_NONE;
            }
        }

        state->isOpened = false;
    }

    return TinyCLR_Result::Success;
}

uint32_t STM32F7_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    return ((controllerIndex > 0 && controllerIndex < 3) ? (STM32F7_APB1_CLOCK_HZ / 256) : (STM32F7_APB2_CLOCK_HZ / 256));
}

uint32_t STM32F7_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    return ((controllerIndex > 0 && controllerIndex < 3) ? (STM32F7_APB1_CLOCK_HZ >> 1) : (STM32F7_APB2_CLOCK_HZ >> 1));
}

uint32_t STM32F7_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self) {
    return STM32F7_Gpio_GetPinCount(nullptr);
}

static const int32_t STM32F7_SPI_DATA_BITS_COUNT = 1;

TinyCLR_Result STM32F7_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, uint32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        dataBitLengths[0] = 8;

    dataBitLengthsCount = STM32F7_SPI_DATA_BITS_COUNT;

    return TinyCLR_Result::Success;
}

void STM32F7_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        STM32F7_Spi_Release(&spiControllers[i]);

        spiStates[i].isOpened = false;
        spiStates[i].initializeCount = 0;
    }
}
