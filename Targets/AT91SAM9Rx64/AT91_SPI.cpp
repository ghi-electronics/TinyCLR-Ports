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

#define SPI_MOSI_PIN 0
#define SPI_MISO_PIN 1
#define SPI_CLK_PIN  2

static const AT91_Gpio_Pin spiPins[][3] = AT91_SPI_PINS;

struct SpiState {
    int32_t controllerIndex;

    uint8_t *readBuffer;
    uint8_t *writeBuffer;

    size_t readLength;
    size_t writeLength;

    int32_t chipSelectLine;
    int32_t dataBitLength;
    int32_t clockFrequency;

    uint32_t chipSelectSetupTime;
    uint32_t chipSelectHoldTime;
    TinyCLR_Spi_ChipSelectType chipSelectType;

    bool chipSelectActiveState;

    TinyCLR_Spi_Mode spiMode;

    bool tableInitialized;

    uint16_t initializeCount;
};

static SpiState spiStates[TOTAL_SPI_CONTROLLERS];

static TinyCLR_Spi_Controller spiControllers[TOTAL_SPI_CONTROLLERS];
static TinyCLR_Api_Info spiApi[TOTAL_SPI_CONTROLLERS];

const char* spiApiNames[TOTAL_SPI_CONTROLLERS] = {
#if TOTAL_SPI_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.AT91.SpiController\\0",
#if TOTAL_SPI_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.AT91.SpiController\\1",
#endif
#endif
};

void AT91_Spi_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        if (spiStates[i].tableInitialized)
            continue;

        spiControllers[i].ApiInfo = &spiApi[i];
        spiControllers[i].Acquire = &AT91_Spi_Acquire;
        spiControllers[i].Release = &AT91_Spi_Release;
        spiControllers[i].WriteRead = &AT91_Spi_WriteRead;
        spiControllers[i].SetActiveSettings = &AT91_Spi_SetActiveSettings;
        spiControllers[i].GetChipSelectLineCount = &AT91_Spi_GetChipSelectLineCount;
        spiControllers[i].GetMinClockFrequency = &AT91_Spi_GetMinClockFrequency;
        spiControllers[i].GetMaxClockFrequency = &AT91_Spi_GetMaxClockFrequency;
        spiControllers[i].GetSupportedDataBitLengths = &AT91_Spi_GetSupportedDataBitLengths;

        spiApi[i].Author = "GHI Electronics, LLC";
        spiApi[i].Name = spiApiNames[i];
        spiApi[i].Type = TinyCLR_Api_Type::SpiController;
        spiApi[i].Version = 0;
        spiApi[i].Implementation = &spiControllers[i];
        spiApi[i].State = &spiStates[i];

        spiStates[i].controllerIndex = i;
        spiStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* AT91_Spi_GetRequiredApi() {
    AT91_Spi_EnsureTableInitialized();

    return &spiApi[0];
}

void AT91_Spi_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91_Spi_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &spiApi[i]);
    }
}

bool AT91_Spi_Transaction_Start(int32_t controllerIndex) {
    auto state = &spiStates[controllerIndex];

    if (state->chipSelectType == TinyCLR_Spi_ChipSelectType::Gpio && state->chipSelectLine != PIN_NONE) {
        AT91_Gpio_Write(nullptr, state->chipSelectLine, state->chipSelectActiveState == false ? TinyCLR_Gpio_PinValue::Low : TinyCLR_Gpio_PinValue::High);
    }

    if (state->chipSelectSetupTime > 0) {
        auto currentTicks = AT91_Time_GetCurrentProcessorTime();

        while (AT91_Time_GetCurrentProcessorTime() - currentTicks < state->chipSelectSetupTime);
    }
    else {
        AT91_Time_Delay(nullptr, ((1000000 / (state->clockFrequency / 1000)) / 1000));
    }

    return true;
}

bool AT91_Spi_Transaction_Stop(int32_t controllerIndex) {
    auto state = &spiStates[controllerIndex];

    if (state->chipSelectHoldTime > 0) {
        auto currentTicks = AT91_Time_GetCurrentProcessorTime();

        while (AT91_Time_GetCurrentProcessorTime() - currentTicks < state->chipSelectHoldTime);
    }
    else {
        AT91_Time_Delay(nullptr, ((1000000 / (state->clockFrequency / 1000)) / 1000));
    }

    if (state->chipSelectType == TinyCLR_Spi_ChipSelectType::Gpio && state->chipSelectLine != PIN_NONE) {
        AT91_Gpio_Write(nullptr, state->chipSelectLine, state->chipSelectActiveState == false ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);
    }

    return true;
}

bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controllerIndex) {
    uint8_t Data8;
    auto state = &spiStates[controllerIndex];

    uint8_t* Write8 = state->writeBuffer;
    int32_t WriteCount = state->writeLength;
    uint8_t* Read8 = state->readBuffer;
    int32_t ReadCount = state->readLength;
    int32_t ReadTotal = 0;

    if (ReadCount) {
        ReadTotal = ReadCount;    // we need to read as many bytes as the buffer is long, plus the offset at which we start
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

TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    if (AT91_Spi_Write(self, writeBuffer, writeLength) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return AT91_Spi_Read(self, readBuffer, readLength);
}

TinyCLR_Result AT91_Spi_WriteRead(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter) {
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (!AT91_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    state->readBuffer = readBuffer;
    state->readLength = readLength;
    state->writeBuffer = (uint8_t*)writeBuffer;
    state->writeLength = writeLength;

    if (state->dataBitLength == DATA_BIT_LENGTH_16) {
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
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (!AT91_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    state->readBuffer = buffer;
    state->readLength = length;
    state->writeBuffer = nullptr;
    state->writeLength = 0;

    if (state->dataBitLength == DATA_BIT_LENGTH_16) {
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
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (!AT91_Spi_Transaction_Start(controllerIndex))
        return TinyCLR_Result::InvalidOperation;

    state->readBuffer = nullptr;
    state->readLength = 0;
    state->writeBuffer = (uint8_t*)buffer;
    state->writeLength = length;

    if (state->dataBitLength == DATA_BIT_LENGTH_16) {
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

TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, const TinyCLR_Spi_Settings* settings) {
    uint32_t chipSelectLine = settings->ChipSelectLine;
    TinyCLR_Spi_ChipSelectType chipSelectType = settings->ChipSelectType;
    uint32_t chipSelectSetupTime = settings->ChipSelectSetupTime;
    uint32_t chipSelectHoldTime = settings->ChipSelectHoldTime;
    bool chipSelectActiveState = settings->ChipSelectActiveState;
    uint32_t clockFrequency = settings->ClockFrequency;
    uint32_t dataBitLength = settings->DataBitLength;
    TinyCLR_Spi_Mode mode = settings->Mode;
    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (state->chipSelectLine == chipSelectLine &&
        state->chipSelectType == chipSelectType &&
        state->chipSelectSetupTime == chipSelectSetupTime &&
        state->chipSelectHoldTime == chipSelectHoldTime &&
        state->chipSelectActiveState == chipSelectActiveState &&
        state->clockFrequency == clockFrequency &&
        state->dataBitLength == dataBitLength &&
        state->spiMode == mode) {
        return TinyCLR_Result::Success;
    }

    state->chipSelectLine = chipSelectLine;
    state->chipSelectType = chipSelectType;
    state->chipSelectSetupTime = chipSelectSetupTime;
    state->chipSelectHoldTime = chipSelectHoldTime;
    state->chipSelectActiveState = chipSelectActiveState;
    state->clockFrequency = clockFrequency;
    state->dataBitLength = dataBitLength;
    state->spiMode = mode;

    AT91_SPI &spi = AT91::SPI(controllerIndex);

    uint32_t clkPin, misoPin, mosiPin;
    AT91_Gpio_PeripheralSelection clkMode, misoMode, mosiMode;

    clkPin = spiPins[controllerIndex][SPI_CLK_PIN].number;
    misoPin = spiPins[controllerIndex][SPI_MISO_PIN].number;
    mosiPin = spiPins[controllerIndex][SPI_MOSI_PIN].number;

    clkMode = spiPins[controllerIndex][SPI_CLK_PIN].peripheralSelection;
    misoMode = spiPins[controllerIndex][SPI_MISO_PIN].peripheralSelection;
    mosiMode = spiPins[controllerIndex][SPI_MOSI_PIN].peripheralSelection;

    AT91_GpioInternal_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_GpioInternal_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_ResistorMode::Inactive);
    AT91_GpioInternal_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_ResistorMode::Inactive);

    uint32_t CSR = 0;

    if (state->dataBitLength == DATA_BIT_LENGTH_16) {
        CSR |= AT91_SPI::SPI_CSR_16BITS;
    }
    else {
        CSR |= AT91_SPI::SPI_CSR_8BITS;
    }

    // first build the mode register
    spi.SPI_MR = AT91_SPI::SPI_MR_MSTR | AT91_SPI::SPI_MR_CS0 | AT91_SPI::SPI_MR_MODFDIS;

    switch (state->spiMode) {

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

    int32_t clockRateKhz = state->clockFrequency / 1000;

    CSR |= AT91_SPI::ConvertClockRateToDivisor(clockRateKhz) << AT91_SPI::SPI_CSR_SCBR_SHIFT;

    spi.SPI_CSR0 = CSR;

    spi.SPI_CR |= AT91_SPI::SPI_CR_ENABLE_SPI;

    if (state->chipSelectType == TinyCLR_Spi_ChipSelectType::Gpio && state->chipSelectLine != PIN_NONE) {
        if (AT91_GpioInternal_OpenPin(state->chipSelectLine)) {
            AT91_GpioInternal_EnableOutputPin(state->chipSelectLine, !state->chipSelectActiveState);
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

    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {

        auto controllerIndex = state->controllerIndex;

        clkPin = spiPins[controllerIndex][SPI_CLK_PIN].number;
        misoPin = spiPins[controllerIndex][SPI_MISO_PIN].number;
        mosiPin = spiPins[controllerIndex][SPI_MOSI_PIN].number;

        clkMode = spiPins[controllerIndex][SPI_CLK_PIN].peripheralSelection;
        misoMode = spiPins[controllerIndex][SPI_MISO_PIN].peripheralSelection;
        mosiMode = spiPins[controllerIndex][SPI_MOSI_PIN].peripheralSelection;

        AT91_PMC &pmc = AT91::PMC();

        // Check each pin single time make sure once fail not effect to other pins
        if (!AT91_GpioInternal_OpenMultiPins(spiPins[controllerIndex], 3))
            return TinyCLR_Result::SharingViolation;

        switch (controllerIndex) {
        case 0:
            pmc.EnablePeriphClock(AT91C_ID_SPI0);
            break;
        }

        AT91_GpioInternal_ConfigurePin(clkPin, AT91_Gpio_Direction::Input, clkMode, AT91_Gpio_ResistorMode::Inactive);
        AT91_GpioInternal_ConfigurePin(misoPin, AT91_Gpio_Direction::Input, misoMode, AT91_Gpio_ResistorMode::Inactive);
        AT91_GpioInternal_ConfigurePin(mosiPin, AT91_Gpio_Direction::Input, mosiMode, AT91_Gpio_ResistorMode::Inactive);
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_PMC &pmc = AT91::PMC();

    auto state = reinterpret_cast<SpiState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        AT91_SPI &spi = AT91::SPI(controllerIndex);
        // off SPI module
        spi.SPI_CR |= AT91_SPI::SPI_CR_DISABLE_SPI;

        switch (controllerIndex) {
        case 0:
            pmc.DisablePeriphClock(AT91C_ID_SPI0);

            break;
        }

        uint32_t clkPin, misoPin, mosiPin;

        clkPin = spiPins[controllerIndex][SPI_CLK_PIN].number;
        misoPin = spiPins[controllerIndex][SPI_MISO_PIN].number;
        mosiPin = spiPins[controllerIndex][SPI_MOSI_PIN].number;

        AT91_GpioInternal_ClosePin(clkPin);
        AT91_GpioInternal_ClosePin(misoPin);
        AT91_GpioInternal_ClosePin(mosiPin);

        if (state->chipSelectType == TinyCLR_Spi_ChipSelectType::Gpio && state->chipSelectLine != PIN_NONE) {
            // Release the pin, set pin un-reserved
            AT91_GpioInternal_ClosePin(state->chipSelectLine);

            // Keep chip select is inactive by internal pull up
            AT91_GpioInternal_ConfigurePin(state->chipSelectLine, AT91_Gpio_Direction::Input, AT91_Gpio_PeripheralSelection::None, AT91_Gpio_ResistorMode::PullUp);

            state->chipSelectLine = PIN_NONE;
        }
    }

    state->clockFrequency = 0;
    state->dataBitLength = 0;

    return TinyCLR_Result::Success;
}

uint32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 255;
}

uint32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self) {
    return AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 1;
}

uint32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self) {
    return AT91_Gpio_GetPinCount(nullptr);
}

static const int32_t dataBitsCount = 2;
static int32_t dataBits[dataBitsCount] = { 8, 16 };

TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, uint32_t* dataBitLengths, size_t& dataBitLengthsCount) {
    if (dataBitLengths != nullptr)
        memcpy(dataBitLengths, dataBits, (dataBitsCount < dataBitLengthsCount ? dataBitsCount : dataBitLengthsCount) * sizeof(int32_t));

    dataBitLengthsCount = dataBitsCount;

    return TinyCLR_Result::Success;
}

void AT91_Spi_Reset() {
    for (auto i = 0; i < TOTAL_SPI_CONTROLLERS; i++) {
        AT91_Spi_Release(&spiControllers[i]);

        spiStates[i].tableInitialized = false;
        spiStates[i].initializeCount = 0;
    }
}
