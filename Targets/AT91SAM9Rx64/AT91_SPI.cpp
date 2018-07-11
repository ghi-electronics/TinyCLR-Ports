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

static const AT91_Gpio_Pin g_at91_spi_miso_pins[] = AT91_SPI_MISO_PINS;
static const AT91_Gpio_Pin g_at91_spi_mosi_pins[] = AT91_SPI_MOSI_PINS;
static const AT91_Gpio_Pin g_at91_spi_sclk_pins[] = AT91_SPI_SCLK_PINS;

struct SpiController {
    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t writeLength;
    size_t readLength;
    size_t readOffset;

    int32_t chipSelectLine;
    int32_t clockFrequency;
    int32_t dataBitLength;

    bool isOpened;

    TinyCLR_Spi_Mode spiMode;
};

static SpiController g_SpiController[TOTAL_SPI_CONTROLLERS];

static TinyCLR_Spi_Provider spiProvider;
static TinyCLR_Api_Info spiApi;

const TinyCLR_Api_Info* AT91_Spi_GetApi() {
    spiProvider.ApiInfo = &spiApi;
    spiProvider.Acquire = &AT91_Spi_Acquire;
    spiProvider.Release = &AT91_Spi_Release;
    spiProvider.SetActiveSettings = &AT91_Spi_SetActiveSettings;
    spiProvider.Read = &AT91_Spi_Read;
    spiProvider.Write = &AT91_Spi_Write;
    spiProvider.TransferFullDuplex = &AT91_Spi_TransferFullDuplex;
    spiProvider.TransferSequential = &AT91_Spi_TransferSequential;
    spiProvider.GetChipSelectLineCount = &AT91_Spi_GetChipSelectLineCount;
    spiProvider.GetMinClockFrequency = &AT91_Spi_GetMinClockFrequency;
    spiProvider.GetMaxClockFrequency = &AT91_Spi_GetMaxClockFrequency;
    spiProvider.GetSupportedDataBitLengths = &AT91_Spi_GetSupportedDataBitLengths;
    spiProvider.GetControllerCount = &AT91_Spi_GetControllerCount;

    spiApi.Author = "GHI Electronics, LLC";
    spiApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.SpiProvider";
    spiApi.Type = TinyCLR_Api_Type::SpiProvider;
    spiApi.Version = 0;
    spiApi.Implementation = &spiProvider;

    return &spiApi;
}

bool AT91_Spi_Transaction_Start(int32_t controller) {
    auto gpioController = 0; //TODO Temporary set to 0

    AT91_Gpio_Write(nullptr, gpioController, g_SpiController[controller].chipSelectLine, TinyCLR_Gpio_PinValue::Low);

    return true;
}

bool AT91_Spi_Transaction_Stop(int32_t controller) {
    auto gpioController = 0; //TODO Temporary set to 0

    AT91_Gpio_Write(nullptr, gpioController, g_SpiController[controller].chipSelectLine, TinyCLR_Gpio_PinValue::High);

    return true;
}

bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controller) {
    uint8_t Data8;
    uint8_t* Write8 = g_SpiController[controller].writeBuffer;
    int32_t WriteCount = g_SpiController[controller].writeLength;
    uint8_t* Read8 = g_SpiController[controller].readBuffer;
    int32_t ReadCount = g_SpiController[controller].readLength;
    int32_t ReadStartOffset = g_SpiController[controller].readOffset;
    int32_t ReadTotal = 0;

    if (ReadCount) {
        ReadTotal = ReadCount + ReadStartOffset;    // we need to read as many bytes as the buffer is long, plus the offset at which we start
    }

    int32_t loopCnt = ReadTotal;

    AT91_SPI &spi = AT91::SPI(controller);

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

bool AT91_Spi_Transaction_nWrite16_nRead16(int32_t controller) {

    return true;
}

TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, int32_t controller, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (AT91_Spi_Write(self, controller, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return AT91_Spi_Read(self, controller, readBuffer, readLength);
}

TinyCLR_Result AT91_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, int32_t controller, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength) {
    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = readBuffer;
    g_SpiController[controller].readLength = readLength;
    g_SpiController[controller].writeBuffer = (uint8_t*)writeBuffer;
    g_SpiController[controller].writeLength = writeLength;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
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

TinyCLR_Result AT91_Spi_Read(const TinyCLR_Spi_Provider* self, int32_t controller, uint8_t* buffer, size_t& length) {
    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = buffer;
    g_SpiController[controller].readLength = length;
    g_SpiController[controller].writeBuffer = nullptr;
    g_SpiController[controller].writeLength = 0;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
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

TinyCLR_Result AT91_Spi_Write(const TinyCLR_Spi_Provider* self, int32_t controller, const uint8_t* buffer, size_t& length) {
    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (!AT91_Spi_Transaction_Start(controller))
        return TinyCLR_Result::InvalidOperation;

    g_SpiController[controller].readBuffer = nullptr;
    g_SpiController[controller].readLength = 0;
    g_SpiController[controller].writeBuffer = (uint8_t*)buffer;
    g_SpiController[controller].writeLength = length;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
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

TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t controller, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode) {
    if (controller >= TOTAL_SPI_CONTROLLERS)
        return TinyCLR_Result::InvalidOperation;

    if (g_SpiController[controller].chipSelectLine == chipSelectLine
        && g_SpiController[controller].dataBitLength == dataBitLength
        && g_SpiController[controller].spiMode == mode
        && g_SpiController[controller].clockFrequency == clockFrequency) {
        return TinyCLR_Result::Success;
    }

    g_SpiController[controller].chipSelectLine = chipSelectLine;
    g_SpiController[controller].clockFrequency = clockFrequency;
    g_SpiController[controller].dataBitLength = dataBitLength;
    g_SpiController[controller].spiMode = mode;

    AT91_SPI &spi = AT91::SPI(controller);

    uint32_t clkPin, misoPin, mosiPin;
    AT91_Gpio_PeripheralSelection clkMode, misoMode, mosiMode;

    clkPin = g_at91_spi_sclk_pins[controller].number;
    misoPin = g_at91_spi_miso_pins[controller].number;
    mosiPin = g_at91_spi_mosi_pins[controller].number;

    clkMode = g_at91_spi_sclk_pins[controller].peripheralSelection;
    misoMode = g_at91_spi_miso_pins[controller].peripheralSelection;
    mosiMode = g_at91_spi_mosi_pins[controller].peripheralSelection;

    AT91_Gpio_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_ResistorMode::Inactive);

    uint32_t CSR = 0;

    if (g_SpiController[controller].dataBitLength == DATA_BIT_LENGTH_16) {
        CSR |= AT91_SPI::SPI_CSR_16BITS;
    }
    else {
        CSR |= AT91_SPI::SPI_CSR_8BITS;
    }

    // first build the mode register
    spi.SPI_MR = AT91_SPI::SPI_MR_MSTR | AT91_SPI::SPI_MR_CS0 | AT91_SPI::SPI_MR_MODFDIS;

    switch (g_SpiController[controller].spiMode) {

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

    int32_t clockRateKhz = g_SpiController[controller].clockFrequency / 1000;

    CSR |= AT91_SPI::ConvertClockRateToDivisor(clockRateKhz) << AT91_SPI::SPI_CSR_SCBR_SHIFT;

    spi.SPI_CSR0 = CSR;

    spi.SPI_CR |= AT91_SPI::SPI_CR_ENABLE_SPI;

    if (g_SpiController[controller].chipSelectLine != PIN_NONE) {
        if (AT91_Gpio_OpenPin(g_SpiController[controller].chipSelectLine)) {
            AT91_Gpio_EnableOutputPin(g_SpiController[controller].chipSelectLine, true);
        }
        else {
            return TinyCLR_Result::SharingViolation;
        }
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Acquire(const TinyCLR_Spi_Provider* self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    uint32_t clkPin, misoPin, mosiPin;
    AT91_Gpio_PeripheralSelection clkMode, misoMode, mosiMode;

    clkPin = g_at91_spi_sclk_pins[controller].number;
    misoPin = g_at91_spi_miso_pins[controller].number;
    mosiPin = g_at91_spi_mosi_pins[controller].number;

    clkMode = g_at91_spi_sclk_pins[controller].peripheralSelection;
    misoMode = g_at91_spi_miso_pins[controller].peripheralSelection;
    mosiMode = g_at91_spi_mosi_pins[controller].peripheralSelection;

    AT91_PMC &pmc = AT91::PMC();

    // Check each pin single time make sure once fail not effect to other pins
    if (!AT91_Gpio_OpenPin(clkPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(misoPin))
        return TinyCLR_Result::SharingViolation;
    if (!AT91_Gpio_OpenPin(mosiPin))
        return TinyCLR_Result::SharingViolation;

    switch (controller) {
    case 0:
        pmc.EnablePeriphClock(AT91C_ID_SPI0);
        break;
    }

    AT91_Gpio_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_Gpio_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_ResistorMode::Inactive);

    g_SpiController[controller].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Provider* self, int32_t controller) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_PMC &pmc = AT91::PMC();

    AT91_SPI &spi = AT91::SPI(controller);
    // off SPI module
    spi.SPI_CR |= AT91_SPI::SPI_CR_DISABLE_SPI;

    switch (controller) {
    case 0:
        pmc.DisablePeriphClock(AT91C_ID_SPI0);

        break;
    }
    if (g_SpiController[controller].isOpened) {
        uint32_t clkPin, misoPin, mosiPin;

        clkPin = g_at91_spi_sclk_pins[controller].number;
        misoPin = g_at91_spi_miso_pins[controller].number;
        mosiPin = g_at91_spi_mosi_pins[controller].number;

        AT91_Gpio_ClosePin(clkPin);
        AT91_Gpio_ClosePin(misoPin);
        AT91_Gpio_ClosePin(mosiPin);

        if (g_SpiController[controller].chipSelectLine != PIN_NONE) {
            // Release the pin, set pin un-reserved
            AT91_Gpio_ClosePin(g_SpiController[controller].chipSelectLine);

            // Keep chip select is inactive by internal pull up
            AT91_Gpio_ConfigurePin(g_SpiController[controller].chipSelectLine, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullUp);

            g_SpiController[controller].chipSelectLine = PIN_NONE;
        }
    }
    g_SpiController[controller].clockFrequency = 0;
    g_SpiController[controller].dataBitLength = 0;

    g_SpiController[controller].isOpened = false;

    return TinyCLR_Result::Success;
}

int32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self, int32_t controller) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 255;
}

int32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self, int32_t controller) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 1;
}

int32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self, int32_t controller) {
    auto gpioController = 0; //TODO Temporary set to 0

    return AT91_Gpio_GetPinCount(nullptr, gpioController);
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t controller, int32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void AT91_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        AT91_Spi_Release(&spiProvider, i);

        g_SpiController[i].isOpened = false;
    }
}

TinyCLR_Result AT91_Spi_GetControllerCount(const TinyCLR_Spi_Provider* self, int32_t& count) {
    count = TOTAL_SPI_CONTROLLERS;

    return TinyCLR_Result::Success;
}