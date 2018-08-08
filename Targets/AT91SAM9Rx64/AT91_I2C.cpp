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

struct I2cConfiguration {

    int32_t                  address;
    uint8_t                  clockRate;     // primary clock factor to generate the i2c clock
    uint8_t                  clockRate2;   // additional clock factors, if more than one is needed for the clock (optional)
};

#define I2C_TRANSACTION_TIMEOUT 2000000

static const AT91_Gpio_Pin i2cSclPins[] = AT91_I2C_SCL_PINS;
static const AT91_Gpio_Pin i2cSdaPins[] = AT91_I2C_SDA_PINS;

struct I2cState {
    int32_t controllerIndex;

    I2cConfiguration i2cConfiguration;

    bool isOpened;

    uint16_t initializeCount;
};

static I2cState i2cStates[TOTAL_I2C_CONTROLLERS];

static TinyCLR_I2c_Controller i2cControllers[TOTAL_I2C_CONTROLLERS];
static TinyCLR_Api_Info i2cApi[TOTAL_I2C_CONTROLLERS];

const char* i2cApiNames[] = {
#if TOTAL_I2C_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.AT91.I2cController\\0"
#endif
};

void AT91_I2c_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        i2cControllers[i].ApiInfo = &i2cApi[i];
        i2cControllers[i].Acquire = &AT91_I2c_Acquire;
        i2cControllers[i].Release = &AT91_I2c_Release;
        i2cControllers[i].SetActiveSettings = &AT91_I2c_SetActiveSettings;
        i2cControllers[i].WriteRead = &AT91_I2c_WriteRead;

        i2cApi[i].Author = "GHI Electronics, LLC";
        i2cApi[i].Name = i2cApiNames[i];
        i2cApi[i].Type = TinyCLR_Api_Type::I2cController;
        i2cApi[i].Version = 0;
        i2cApi[i].Implementation = &i2cControllers[i];
        i2cApi[i].State = &i2cStates[i];

        i2cStates[i].controllerIndex = i;

        apiManager->Add(apiManager, &i2cApi[i]);
    }
}

TinyCLR_Result AT91_I2c_Read(const TinyCLR_I2c_Controller* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& error) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    uint32_t address;
    uint32_t control = 0;

    size_t bytesToTransfer = length;
    size_t bytesTransferred = 0;

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    AT91_I2C& I2C = AT91::I2C(controllerIndex);

    address = (state->i2cConfiguration.address << AT91_I2C::TWI_MMR_DADR_SHIFT) | AT91_I2C::TWI_MMR_MREAD_R;


    I2C.TWI_CWGR = state->i2cConfiguration.clockRate | (state->i2cConfiguration.clockRate << AT91_I2C::TWI_CWGR_CHDIV_SHIFT) | (state->i2cConfiguration.clockRate2 << AT91_I2C::TWI_CWGR_CKDIV_SHIFT);

    control = AT91_I2C::TWI_CR_MSEN | AT91_I2C::TWI_CR_SVDIS;

    control |= AT91_I2C::TWI_CR_START;

    I2C.TWI_MMR = address;

    I2C.TWI_CR = control;

    while (bytesToTransfer > 0 && timeout > 0) {
        if (bytesToTransfer == 1) {
            I2C.TWI_CR = AT91_I2C::TWI_CR_STOP;
        }

        while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_RXRDY) && timeout > 0) {
            AT91_Time_Delay(nullptr, 1);

            timeout--;
        }

        buffer[bytesTransferred] = I2C.TWI_RHR;
        bytesToTransfer--;
        bytesTransferred++;
        // Reset Timeout
        if (timeout > 0) {
            timeout = I2C_TRANSACTION_TIMEOUT;
        }
    }

    // Reset Timeout
    if (timeout > 0) {
        timeout = I2C_TRANSACTION_TIMEOUT;
    }

    while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_TXCOMP) && timeout > 0) {
        AT91_Time_Delay(nullptr, 1);

        timeout--;
    }

    I2C.TWI_CR = AT91_I2C::TWI_CR_MSDIS;

    length = bytesTransferred;

    if (timeout > 0)
        error = TinyCLR_I2c_TransferStatus::FullTransfer;
    else
        error = TinyCLR_I2c_TransferStatus::ClockStretchTimeout;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result AT91_I2c_Write(const TinyCLR_I2c_Controller* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& error) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    uint32_t address;
    uint32_t control = 0;

    size_t bytesToTransfer = length;
    size_t bytesTransferred = 0;

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    AT91_I2C& I2C = AT91::I2C(controllerIndex);

    address = state->i2cConfiguration.address << AT91_I2C::TWI_MMR_DADR_SHIFT;

    I2C.TWI_CWGR = state->i2cConfiguration.clockRate | (state->i2cConfiguration.clockRate << AT91_I2C::TWI_CWGR_CHDIV_SHIFT) | (state->i2cConfiguration.clockRate2 << AT91_I2C::TWI_CWGR_CKDIV_SHIFT);

    control = AT91_I2C::TWI_CR_MSEN | AT91_I2C::TWI_CR_SVDIS;

    control |= AT91_I2C::TWI_CR_START;

    I2C.TWI_MMR = address;

    I2C.TWI_CR = control;

    while (bytesToTransfer > 0 && timeout > 0) {
        while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_TXRDY) && timeout > 0) {
            AT91_Time_Delay(nullptr, 1);

            timeout--;
        }

        // Reset Timeout
        if (timeout > 0) {
            timeout = I2C_TRANSACTION_TIMEOUT;
        }

        I2C.TWI_THR = buffer[bytesTransferred];

        bytesTransferred++;
        bytesToTransfer--;

    }

    // Reset Timeout
    if (timeout > 0) {
        timeout = I2C_TRANSACTION_TIMEOUT;
    }

    while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_TXRDY) && timeout > 0) {
        AT91_Time_Delay(nullptr, 1);

        timeout--;
    }

    I2C.TWI_CR = AT91_I2C::TWI_CR_STOP;

    // Reset Timeout
    if (timeout > 0) {
        timeout = I2C_TRANSACTION_TIMEOUT;
    }

    while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_TXCOMP) && timeout > 0) {
        AT91_Time_Delay(nullptr, 1);

        timeout--;
    }

    I2C.TWI_CR = AT91_I2C::TWI_CR_MSDIS;

    length = bytesTransferred;

    if (timeout > 0)
        error = TinyCLR_I2c_TransferStatus::FullTransfer;
    else
        error = TinyCLR_I2c_TransferStatus::ClockStretchTimeout;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result AT91_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error) {
    if ((!(sendStartCondition & sendStopCondition)) || (readLength == 0 && writeLength == 0))
        return TinyCLR_Result::NotSupported;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    if (writeLength > 0 && readLength == 0) { // Do write only
        return AT91_I2c_Write(self, writeBuffer, writeLength, error);
    }
    else if (writeLength == 0 && readLength > 0) { // Do Read only
        return AT91_I2c_Read(self, readBuffer, readLength, error);
    }

    // WriteRead
    if (writeLength > 3) {
        AT91_I2c_Write(self, writeBuffer, writeLength, error);

        if (error == TinyCLR_I2c_TransferStatus::FullTransfer)
            AT91_I2c_Read(self, readBuffer, readLength, error);

        return error == TinyCLR_I2c_TransferStatus::FullTransfer ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
    }
    else if (writeLength == 0) {
        AT91_I2c_Read(self, readBuffer, readLength, error);

        return error == TinyCLR_I2c_TransferStatus::FullTransfer ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
    }

    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    uint32_t address;
    uint32_t control = 0;

    size_t bytesToTransfer = readLength;
    size_t bytesTransferred = 0;

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    AT91_I2C& I2C = AT91::I2C(controllerIndex);

    address = (state->i2cConfiguration.address << AT91_I2C::TWI_MMR_DADR_SHIFT) | AT91_I2C::TWI_MMR_MREAD_R;

    I2C.TWI_CWGR = state->i2cConfiguration.clockRate | (state->i2cConfiguration.clockRate << AT91_I2C::TWI_CWGR_CHDIV_SHIFT) | (state->i2cConfiguration.clockRate2 << AT91_I2C::TWI_CWGR_CKDIV_SHIFT);

    control = AT91_I2C::TWI_CR_MSEN | AT91_I2C::TWI_CR_SVDIS;

    control |= AT91_I2C::TWI_CR_START;

    I2C.TWI_MMR = address | (writeLength << 8);

    switch (writeLength) {
    case 1:
        I2C.TWI_IADR = writeBuffer[0];
        break;

    case 2:
        I2C.TWI_IADR = (writeBuffer[1] << 8) | (writeBuffer[0]);
        break;

    case 3:
        I2C.TWI_IADR = (writeBuffer[2] << 16) | (writeBuffer[1] << 8) | (writeBuffer[0]);
        break;
    }

    I2C.TWI_CR = control;

    while (bytesToTransfer > 0 && timeout > 0) {
        if (bytesToTransfer == 1) {
            I2C.TWI_CR = AT91_I2C::TWI_CR_STOP;
        }

        while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_RXRDY) && timeout > 0) {
            AT91_Time_Delay(nullptr, 1);

            timeout--;
        }

        readBuffer[bytesTransferred] = I2C.TWI_RHR;
        bytesToTransfer--;
        bytesTransferred++;
        // Reset Timeout
        if (timeout > 0) {
            timeout = I2C_TRANSACTION_TIMEOUT;
        }
    }

    // Reset Timeout
    if (timeout > 0) {
        timeout = I2C_TRANSACTION_TIMEOUT;
    }

    while (!(I2C.TWI_SR & AT91_I2C::TWI_SR_TXCOMP) && timeout > 0) {
        AT91_Time_Delay(nullptr, 1);

        timeout--;
    }

    I2C.TWI_CR = AT91_I2C::TWI_CR_MSDIS;

    readLength = bytesTransferred;

    if (timeout > 0)
        error = TinyCLR_I2c_TransferStatus::FullTransfer;
    else
        error = TinyCLR_I2c_TransferStatus::ClockStretchTimeout;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

#define CLOCK_RATE_CONSTANT     4
#define MIN_CLK_RATE    (AT91_SYSTEM_PERIPHERAL_CLOCK_HZ/1000)/(128 *255+CLOCK_RATE_CONSTANT)
#define MAX_CLK_RATE    400   //kHz

TinyCLR_Result AT91_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, uint32_t slaveAddress, TinyCLR_I2c_AddressFormat addressFormat, TinyCLR_I2c_BusSpeed busSpeed) {
    uint32_t rateKhz;
    uint8_t clockRate, clockRate2;

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    if (addressFormat == TinyCLR_I2c_AddressFormat::TenBit)
        return TinyCLR_Result::NotSupported;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (busSpeed == TinyCLR_I2c_BusSpeed::FastMode)
        rateKhz = 400; // FastMode
    else if (busSpeed == TinyCLR_I2c_BusSpeed::StandardMode)
        rateKhz = 100; // StandardMode
    else
        return TinyCLR_Result::NotSupported;

    if (rateKhz < MIN_CLK_RATE) {
        clockRate = 255;
        clockRate2 = 7;
    }
    else if (rateKhz >= MAX_CLK_RATE) {

        clockRate = (AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / (2 * 1000)) / MAX_CLK_RATE - CLOCK_RATE_CONSTANT;
        clockRate2 = 0;
    }
    else {

        uint32_t power = 1;
        uint32_t clkDiv;
        uint32_t clkLHDiv;

        clkDiv = 0;
        clkLHDiv = (AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / (2 * 1000)) / rateKhz - CLOCK_RATE_CONSTANT;

        if (clkLHDiv > 255) {
            clkLHDiv += CLOCK_RATE_CONSTANT;
            for (clkDiv = 1; clkDiv <= 7; clkDiv++) {
                clkLHDiv /= 2;
                power *= 2; // save the calculation
                if (clkLHDiv <= 255)
                    break;
            }

            clkLHDiv = (((AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / (2 * 1000)) / rateKhz) - CLOCK_RATE_CONSTANT) / power;
        }
        clockRate = clkLHDiv;
        clockRate2 = clkDiv;
    }

    state->i2cConfiguration.clockRate = (uint8_t)clockRate; // low byte
    state->i2cConfiguration.clockRate2 = (uint8_t)(clockRate2); // high byte
    state->i2cConfiguration.address = slaveAddress;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_I2c_Acquire(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        AT91_I2C& I2C = AT91::I2C(controllerIndex);
        AT91_PMC &pmc = AT91::PMC();

        if (!state->isOpened) {
            if (!AT91_Gpio_OpenPin(i2cSdaPins[controllerIndex].number) || !AT91_Gpio_OpenPin(i2cSclPins[controllerIndex].number))
                return TinyCLR_Result::SharingViolation;

            AT91_Gpio_ConfigurePin(i2cSdaPins[controllerIndex].number, AT91_Gpio_Direction::Input, i2cSdaPins[controllerIndex].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
            AT91_Gpio_ConfigurePin(i2cSclPins[controllerIndex].number, AT91_Gpio_Direction::Input, i2cSclPins[controllerIndex].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);

            pmc.EnablePeriphClock(controllerIndex == 0 ? AT91C_ID_TWI0 : AT91C_ID_TWI1);
            I2C.TWI_MMR = 0x7e << AT91_I2C::TWI_MMR_DADR_SHIFT;

            state->isOpened = true;
        }
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_I2c_Release(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        AT91_I2C& I2C = AT91::I2C(controllerIndex);

        I2C.TWI_CR = AT91_I2C::TWI_CR_SWRST;

        AT91_Interrupt_Disable(controllerIndex == 0 ? AT91C_ID_TWI0 : AT91C_ID_TWI1);

        AT91_PMC &pmc = AT91::PMC();
        pmc.DisablePeriphClock(controllerIndex == 0 ? AT91C_ID_TWI0 : AT91C_ID_TWI1);

        // disable
        I2C.TWI_CR = AT91_I2C::TWI_CR_MSDIS;

        // disable all the interrupt
        I2C.TWI_IDR = AT91_I2C::TWI_IDR_NACK | AT91_I2C::TWI_IDR_RXRDY | AT91_I2C::TWI_IDR_TXCOMP | AT91_I2C::TWI_IDR_TXRDY;


        state->isOpened = false;

        if (state->isOpened) {
            AT91_Gpio_ClosePin(i2cSdaPins[controllerIndex].number);
            AT91_Gpio_ClosePin(i2cSclPins[controllerIndex].number);
        }
    }

    return TinyCLR_Result::Success;
}

void AT91_I2c_Reset() {
    for (auto i = 0; i < SIZEOF_ARRAY(i2cSclPins); i++) {
        AT91_I2c_Release(&i2cControllers[i]);

        auto state = &i2cStates[i];

        state->i2cConfiguration.address = 0;
        state->i2cConfiguration.clockRate = 0;
        state->i2cConfiguration.clockRate2 = 0;

        state->isOpened = false;
        state->initializeCount = 0;
    }
}
