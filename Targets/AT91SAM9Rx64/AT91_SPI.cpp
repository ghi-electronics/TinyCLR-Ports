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

static const AT91_Gpio_Pin spiMisoPins[] = AT91_SPI_MISO_PINS;
static const AT91_Gpio_Pin spiMosiPins[] = AT91_SPI_MOSI_PINS;
static const AT91_Gpio_Pin spiClkPins[] = AT91_SPI_SCLK_PINS;

struct SpiState {
    int32_t controllerIndex;

    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t readLength;
    size_t writeLength;
    size_t readOffset;

    int32_t chipSelectLine;
    int32_t dataBitLength;
    int32_t clockFrequency;

    bool isOpened;

    TinyCLR_Spi_Mode spiMode;
};

static SpiState spiStates[TOTAL_SPI_CONTROLLERS];

static TinyCLR_Spi_Controller spiControllers[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi[TOTAL_SPI_CONTROLLERS];

const TinyCLR_Api_Info* AT91_Spi_GetApi() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        spiControllers[i].ApiInfo = &spiApi[i];
        spiControllers[i].Acquire = &AT91_Spi_Acquire;
        spiControllers[i].Release = &AT91_Spi_Release;
        spiControllers[i].SetActiveSettings = &AT91_Spi_SetActiveSettings;
        spiControllers[i].Read = &AT91_Spi_Read;
        spiControllers[i].Write = &AT91_Spi_Write;
        spiControllers[i].TransferFullDuplex = &AT91_Spi_TransferFullDuplex;
        spiControllers[i].TransferSequential = &AT91_Spi_TransferSequential;
        spiControllers[i].GetChipSelectLineCount = &AT91_Spi_GetChipSelectLineCount;
        spiControllers[i].GetMinClockFrequency = &AT91_Spi_GetMinClockFrequency;
        spiControllers[i].GetMaxClockFrequency = &AT91_Spi_GetMaxClockFrequency;
        spiControllers[i].GetSupportedDataBitLengths = &AT91_Spi_GetSupportedDataBitLengths;

        spiApi[i].Author = "GHI Electronics, LLC";
        spiApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.AT91.SpiController";
        spiApi[i].Type = TinyCLR_Api_Type::SpiController;
        spiApi[i].Version = 0;
        spiApi[i].Implementation = &spiControllers[i];
        spiApi[i].State = &spiStates[i];

        spiStates[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&spiApi;
}

bool AT91_Spi_Transaction_Start(int32_t controllerIndex) {
    auto driver = &spiStates[controllerIndex];

    AT91_Gpio_Write(nullptr, driver->chipSelectLine, TinyCLR_Gpio_PinValue::Low);

    return true;
}

bool AT91_Spi_Transaction_Stop(int32_t controllerIndex) {
    auto driver = &spiStates[controllerIndex];

    AT91_Gpio_Write(nullptr, driver->chipSelectLine, TinyCLR_Gpio_PinValue::High);

    return true;
}

bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controllerIndex) {
    uint8_t Data8;
    auto driver = &spiStates[controllerIndex];

    uint8_t* Write8 = driver->writeBuffer;
    int32_t WriteCount = driver->writeLength;
    uint8_t* Read8 = driver->readBuffer;
    int32_t ReadCount = driver->readLength;
    int32_t ReadStartOffset = driver->readOffset;
    int32_t ReadTotal = 0;

    if (ReadCount) {
        ReadTotal = ReadCount + ReadStartOffset;    // we need to read as many bytes as the buffer is long, plus the offset at which we start
    }

    int32_t loopCnt = ReadTotal;

    AT91_SPI &spi = AT91::SPI(controllerIndex);

    // take the max of Read+offset or WrCnt
    if (loopCnt < WriteCount)
        loopCnt = WriteCount;

    // we will use WriteCount to move in the Write8 array
    // so we do no want to go past the end when we will check for
    // WriteCount to be bigger than zero
    WriteCount -= 1;

    // Start transmission
    while (loopCnt--) {
        spi.SPI_TDR = Write8[0];

        // wait while the transmit buffer is empty
        while (!spi.TransmitBufferEmpty(spi));

        // reading clears the RBF bit and allows another transfer from the shift register
        Data8 = spi.SPI_RDR;

        // repeat last write word for all subsequent reads
        if (WriteCount) {
            WriteCount--;
            Write8++;
        }

        // only save data once we have reached ReadCount-1 portion of words
        ReadTotal--;
        if ((ReadTotal >= 0) && (ReadTotal < ReadCount)) {
            Read8[0] = Data8;
            Read8++;
        }

    }

    return true;
}

bool AT91_Spi_Transaction_nWrite16_nRead16(int32_t controllerIndex) {
    return false;
}

TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    if (AT91_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return AT91_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result AT91_Spi_TransferFullDuplex(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (!AT91_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    driver->readBuffer = readBuffer;
    driver->readLength = readLength;
    driver->writeBuffer = (uint8_t*)writeBuffer;
    driver->writeLength = writeLength;

    if (driver->dataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controllerIndex))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controllerIndex))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length) {
    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (!AT91_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    driver->readBuffer = buffer;
    driver->readLength = length;
    driver->writeBuffer = nullptr;
    driver->writeLength = 0;

    if (driver->dataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controllerIndex))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controllerIndex))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length) {
    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (!AT91_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    driver->readBuffer = nullptr;
    driver->readLength = 0;
    driver->writeBuffer = (uint8_t*)buffer;
    driver->writeLength = length;

    if (driver->dataBitLength == DATA_BIT_LENGTH_16) {
        if (!AT91_Spi_Transaction_nWrite16_nRead16(controllerIndex))
            return TinyCLR_Result::InvalidOperation;
    }
    else {
        if (!AT91_Spi_Transaction_nWrite8_nRead8(controllerIndex))
            return TinyCLR_Result::InvalidOperation;
    }

    if (!AT91_Spi_Transaction_Stop(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    if (driver->chipSelectLine == chipSelectLine
        && driver->dataBitLength == dataBitLength
        && driver->spiMode == mode
        && driver->clockFrequency == clockFrequency) {
        return TinyCLR_Result::Success;
    }

    driver->chipSelectLine = chipSelectLine;
    driver->clockFrequency = clockFrequency;
    driver->dataBitLength = dataBitLength;
    driver->spiMode = mode;

    AT91_SPI &spi = AT91::SPI(controllerIndex);

    uint32_t clkPin, misoPin, mosiPin;
    AT91_Gpio_PeripheralSelection clkMode, misoMode, mosiMode;

    clkPin = spiClkPins[controllerIndex].number;
    misoPin = spiMisoPins[controllerIndex].number;
    mosiPin = spiMosiPins[controllerIndex].number;

    clkMode = spiClkPins[controllerIndex].peripheralSelection;
    misoMode = spiMisoPins[controllerIndex].peripheralSelection;
    mosiMode = spiMosiPins[controllerIndex].peripheralSelection;

    AT91_Gpio_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_ResistorMode::Inactive);

    uint32_t CSR = 0;

    if (driver->dataBitLength == DATA_BIT_LENGTH_16) {
        CSR |= AT91_SPI::SPI_CSR_16BITS;
    }
    else {
        CSR |= AT91_SPI::SPI_CSR_8BITS;
    }

    // first build the mode register
    spi.SPI_MR = AT91_SPI::SPI_MR_MSTR | AT91_SPI::SPI_MR_CS0 | AT91_SPI::SPI_MR_MODFDIS;

    switch (driver->spiMode) {

    case TinyCLR_Spi_Mode::Mode0: // CPOL = 0, CPHA = 0.
        CSR |= AT91_SPI::SPI_CSR_NCPHA;
        break;

    case TinyCLR_Spi_Mode::Mode1: // CPOL = 0, CPHA = 1.

        break;

    case TinyCLR_Spi_Mode::Mode2: //  CPOL = 1, CPHA = 0.
        CSR |= AT91_SPI::SPI_CSR_CPOL | AT91_SPI::SPI_CSR_NCPHA;
        break;

    case TinyCLR_Spi_Mode::Mode3: // CPOL = 1, CPHA = 1
        CSR |= AT91_SPI::SPI_CSR_CPOL;
        break;
    }

    int32_t clockRateKhz = driver->clockFrequency / 1000;

    CSR |= AT91_SPI::ConvertClockRateToDivisor(clockRateKhz) << AT91_SPI::SPI_CSR_SCBR_SHIFT;

    spi.SPI_CSR0 = CSR;

    spi.SPI_CR |= AT91_SPI::SPI_CR_ENABLE_SPI;

    if (driver->chipSelectLine != PIN_NONE) {
        if (AT91_Gpio_OpenPin(driver->chipSelectLine)) {
            AT91_Gpio_EnableOutputPin(driver->chipSelectLine, true);
        }
        else {
            return TinyCLR_Result::SharingViolation;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Acquire(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    uint32_t clkPin, misoPin, mosiPin;
    AT91_Gpio_PeripheralSelection clkMode, misoMode, mosiMode;

    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    clkPin = spiClkPins[controllerIndex].number;
    misoPin = spiMisoPins[controllerIndex].number;
    mosiPin = spiMosiPins[controllerIndex].number;

    clkMode = spiClkPins[controllerIndex].peripheralSelection;
    misoMode = spiMisoPins[controllerIndex].peripheralSelection;
    mosiMode = spiMosiPins[controllerIndex].peripheralSelection;

    AT91_PMC &pmc = AT91::PMC();

    // Check each pin single time make sure once fail not effect to other pins
    if (!AT91_Gpio_OpenPin(clkPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(misoPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(mosiPin))
        return TinyCLR_Result::SharingViolation;

    switch (controllerIndex) {
    case 0:
        pmc.EnablePeriphClock(AT91C_ID_SPI0);
        break;
    }

    AT91_Gpio_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_ResistorMode::Inactive);

    driver->isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_PMC &pmc = AT91::PMC();

    auto driver = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    AT91_SPI &spi = AT91::SPI(controllerIndex);
    // off SPI module
    spi.SPI_CR |= AT91_SPI::SPI_CR_DISABLE_SPI;

    switch (controllerIndex) {
    case 0:
        pmc.DisablePeriphClock(AT91C_ID_SPI0);

        break;
    }
    if (driver->isOpened) {
        uint32_t clkPin, misoPin, mosiPin;

        clkPin = spiClkPins[controllerIndex].number;
        misoPin = spiMisoPins[controllerIndex].number;
        mosiPin = spiMosiPins[controllerIndex].number;

        AT91_Gpio_ClosePin(clkPin);
        AT91_Gpio_ClosePin(misoPin);
        AT91_Gpio_ClosePin(mosiPin);

        if (driver->chipSelectLine != PIN_NONE) {
            // Release the pin, set pin un-reserved
            AT91_Gpio_ClosePin(driver->chipSelectLine);

            // Keep chip select is inactive by internal pull up
            AT91_Gpio_ConfigurePin(driver->chipSelectLine, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullUp);

            driver->chipSelectLine = PIN_NONE;
        }
    }
    driver->clockFrequency = 0;
    driver->dataBitLength = 0;

    driver->isOpened = false;

    return TinyCLR_Result::Success;
}

int32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 255;
}

int32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 1;
}

int32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self) {
    return AT91_Gpio_GetPinCount(nullptr);
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void AT91_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        AT91_Spi_Release(&spiControllers[i]);

        spiStates[i].isOpened = false;
    }
}
