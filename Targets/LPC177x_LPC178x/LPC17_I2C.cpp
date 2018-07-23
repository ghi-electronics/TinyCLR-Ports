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
// limitations under the License.s

#include <LPC17.h>

struct LPC17xx_I2C {
    static const uint32_t c_I2C0_Base = 0x4001C000;
    static const uint32_t c_I2C1_Base = 0x4005C000;
    static const uint32_t c_I2C2_Base = 0x400A0000;

    static const uint32_t c_I2C_Clk_KHz = LPC17_SYSTEM_CLOCK_HZ / 2 / 1000;

    /****/ volatile uint32_t I2CONSET;
    static const    uint32_t I2EN = 0x00000040;
    static const    uint32_t STA = 0x00000020;
    static const    uint32_t STO = 0x00000010;
    static const    uint32_t SI = 0x00000008;
    static const    uint32_t AA = 0x00000004;

    /****/ volatile uint32_t I2STAT;

    /****/ volatile uint32_t I2DAT;

    /****/ volatile uint32_t I2ADR;

    /****/ volatile uint32_t I2SCLH;

    /****/ volatile uint32_t I2SCLL;

    /****/ volatile uint32_t I2CONCLR;
};

#define TOTAL_I2C_CONTROLLERS SIZEOF_ARRAY(i2cSclPins)

struct I2cConfiguration {

    int32_t                  address;

    uint8_t                  clockRate;     // primary clock factor to generate the i2c clock
    uint8_t                  clockRate2;   // additional clock factors, if more than one is needed for the clock (optional)

    bool                     isOpened;
};

struct I2cTransaction {
    bool                        isReadTransaction;
    bool                        repeatedStart;
    bool                        isDone;

    uint8_t                     *buffer;

    size_t                      bytesToTransfer;
    size_t                      bytesTransferred;

    TinyCLR_I2c_TransferStatus error;
};

#define I2C_TRANSACTION_TIMEOUT 2000 // 2 seconds

static const LPC17_Gpio_Pin i2cSclPins[] = LPC17_I2C_SCL_PINS;
static const LPC17_Gpio_Pin i2cSdaPins[] = LPC17_I2C_SDA_PINS;

struct I2cState {
    int32_t controllerIndex;

    I2cConfiguration i2cConfiguration;
    I2cTransaction   *currentI2cTransactionAction;
    I2cTransaction   readI2cTransactionAction;
    I2cTransaction   writeI2cTransactionAction;
};

static I2cState i2cStates[TOTAL_I2C_CONTROLLERS];

static TinyCLR_I2c_Controller i2cControllers[TOTAL_I2C_CONTROLLERS];
static TinyCLR_Api_Info i2cApi[TOTAL_I2C_CONTROLLERS];

const TinyCLR_Api_Info* LPC17_I2c_GetApi() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        i2cControllers[i].ApiInfo = &i2cApi[i];
        i2cControllers[i].Acquire = &LPC17_I2c_Acquire;
        i2cControllers[i].Release = &LPC17_I2c_Release;
        i2cControllers[i].SetActiveSettings = &LPC17_I2c_SetActiveSettings;       
        i2cControllers[i].WriteRead = &LPC17_I2c_WriteRead;

        i2cApi[i].Author = "GHI Electronics, LLC";
        i2cApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.I2cController";
        i2cApi[i].Type = TinyCLR_Api_Type::I2cController;
        i2cApi[i].Version = 0;
        i2cApi[i].Implementation = &i2cControllers[i];
        i2cApi[i].State = &i2cStates[i];

        i2cStates[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&i2cApi;
}

void LPC17_I2c_InterruptHandler(int32_t controllerIndex) {
    uint8_t address;

    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(controllerIndex == 0 ? LPC17xx_I2C::c_I2C0_Base : ((controllerIndex == 1 ? LPC17xx_I2C::c_I2C1_Base : LPC17xx_I2C::c_I2C2_Base)));

    auto state = &i2cStates[controllerIndex];

    // read status
    uint8_t status = I2C.I2STAT;

    I2cTransaction *transaction = state->currentI2cTransactionAction;

    if (!transaction) {
        I2C.I2CONCLR = LPC17xx_I2C::SI;
        return;
    }

    switch (status) {
    case 0x08: // Start Condition transmitted
    case 0x10: // Repeated Start Condition transmitted
        // Write Slave address and Data direction
        address = 0xFE & (state->i2cConfiguration.address << 1);
        address |= transaction->isReadTransaction ? 1 : 0;
        I2C.I2DAT = address;
        // Clear STA bit
        I2C.I2CONCLR = LPC17xx_I2C::STA;
        break;
    case 0x18: // Slave Address + W transmitted, Ack received
    case 0x28: // Data transmitted, Ack received
        // Write data
        // transaction completed
        if (transaction->bytesToTransfer == 0) {
            if (transaction->repeatedStart == false) {
                LPC17_I2c_StopTransaction(controllerIndex);
            }
            else {
                state->currentI2cTransactionAction = &state->readI2cTransactionAction;
                LPC17_I2c_StartTransaction(controllerIndex);
            }
        }
        else {
            //WriteToSlave( unit );
            I2C.I2DAT = transaction->buffer[transaction->bytesTransferred];

            transaction->bytesTransferred++;
            transaction->bytesToTransfer--;

        }
        break;
    case 0x20: // Write Address not acknowledged by slave
    case 0x30: // Data not acknowledged by slave
    case 0x48: // Read Address not acknowledged by slave
        LPC17_I2c_StopTransaction(controllerIndex);
        break;
    case 0x38: // Arbitration lost
        LPC17_I2c_StopTransaction(controllerIndex);
        break;
    case 0x40: // Slave Address + R transmitted, Ack received
        // if the transaction is one byte only to read, then we must send NAK immediately
        if (transaction->bytesToTransfer == 1) {
            I2C.I2CONCLR = LPC17xx_I2C::AA;
        }
        else {
            I2C.I2CONSET = LPC17xx_I2C::AA;
        }
        break;
    case 0x50: // Data received, Ack Sent
    case 0x58: // Data received, NO Ack sent
        // read next byte
        //ReadFromSlave( unit );
        transaction->buffer[transaction->bytesTransferred] = I2C.I2DAT;

        transaction->bytesTransferred++;
        transaction->bytesToTransfer--;

        if (transaction->bytesToTransfer == 1) {
            I2C.I2CONCLR = LPC17xx_I2C::AA;
        }
        if (transaction->bytesToTransfer == 0) {
            if (transaction->repeatedStart == false) {
                // send transaction stop
                LPC17_I2c_StopTransaction(controllerIndex);
            }
            else {
                // start next
                state->currentI2cTransactionAction = &state->readI2cTransactionAction;
                LPC17_I2c_StartTransaction(controllerIndex);
            }
        }
        break;
    case 0x00: // Bus Error
        // Clear Bus error
        I2C.I2CONSET = LPC17xx_I2C::STO;
        LPC17_I2c_StopTransaction(controllerIndex);
        break;
    default:
        LPC17_I2c_StopTransaction(controllerIndex);
        break;
    } // switch(status)

    // clear the interrupt flag to start the next I2C transfer
    I2C.I2CONCLR = LPC17xx_I2C::SI;

}

void LPC17_I2c_InterruptHandler0(void *param) {
    LPC17_I2c_InterruptHandler(0);
}
void LPC17_I2c_InterruptHandler1(void *param) {
    LPC17_I2c_InterruptHandler(1);
}
void LPC17_I2c_InterruptHandler2(void *param) {
    LPC17_I2c_InterruptHandler(2);
}

void LPC17_I2c_StartTransaction(int32_t controllerIndex) {
    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(controllerIndex == 0 ? LPC17xx_I2C::c_I2C0_Base : ((controllerIndex == 1 ? LPC17xx_I2C::c_I2C1_Base : LPC17xx_I2C::c_I2C2_Base)));

    auto state = &i2cStates[controllerIndex];

    if (!state->writeI2cTransactionAction.repeatedStart || state->writeI2cTransactionAction.bytesTransferred == 0) {
        I2C.I2SCLH = state->i2cConfiguration.clockRate | (state->i2cConfiguration.clockRate2 << 8);
        I2C.I2SCLL = state->i2cConfiguration.clockRate | (state->i2cConfiguration.clockRate2 << 8);

        I2C.I2CONSET = LPC17xx_I2C::STA;
    }
    else {
        I2C.I2CONSET = LPC17xx_I2C::STA;
    }

}

void LPC17_I2c_StopTransaction(int32_t controllerIndex) {
    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(controllerIndex == 0 ? LPC17xx_I2C::c_I2C0_Base : ((controllerIndex == 1 ? LPC17xx_I2C::c_I2C1_Base : LPC17xx_I2C::c_I2C2_Base)));

    I2C.I2CONSET = LPC17xx_I2C::STO;
    I2C.I2CONCLR = LPC17xx_I2C::AA | LPC17xx_I2C::SI | LPC17xx_I2C::STA;

    auto state = &i2cStates[controllerIndex];

    state->currentI2cTransactionAction->isDone = true;
}

TinyCLR_Result LPC17_I2c_Read(const TinyCLR_I2c_Controller* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& error) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    state->readI2cTransactionAction.isReadTransaction = true;
    state->readI2cTransactionAction.buffer = buffer;
    state->readI2cTransactionAction.bytesToTransfer = length;
    state->readI2cTransactionAction.isDone = false;
    state->readI2cTransactionAction.repeatedStart = false;
    state->readI2cTransactionAction.bytesTransferred = 0;

    state->currentI2cTransactionAction = &state->readI2cTransactionAction;

    LPC17_I2c_StartTransaction(controllerIndex);

    while (state->currentI2cTransactionAction->isDone == false && timeout > 0) {
        LPC17_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (state->currentI2cTransactionAction->bytesTransferred == length)
        error = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (state->currentI2cTransactionAction->bytesTransferred < length && state->currentI2cTransactionAction->bytesTransferred > 0)
        error = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = state->currentI2cTransactionAction->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result LPC17_I2c_Write(const TinyCLR_I2c_Controller* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& error) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    state->writeI2cTransactionAction.isReadTransaction = false;
    state->writeI2cTransactionAction.buffer = (uint8_t*)buffer;
    state->writeI2cTransactionAction.bytesToTransfer = length;
    state->writeI2cTransactionAction.isDone = false;
    state->writeI2cTransactionAction.repeatedStart = false;
    state->writeI2cTransactionAction.bytesTransferred = 0;

    state->currentI2cTransactionAction = &state->writeI2cTransactionAction;

    LPC17_I2c_StartTransaction(controllerIndex);

    while (state->currentI2cTransactionAction->isDone == false && timeout > 0) {
        LPC17_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (state->currentI2cTransactionAction->bytesTransferred == length)
        error = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (state->currentI2cTransactionAction->bytesTransferred < length && state->currentI2cTransactionAction->bytesTransferred > 0)
        error = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = state->currentI2cTransactionAction->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result LPC17_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStopAfter, TinyCLR_I2c_TransferStatus& error) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    state->writeI2cTransactionAction.isReadTransaction = false;
    state->writeI2cTransactionAction.buffer = (uint8_t*)writeBuffer;
    state->writeI2cTransactionAction.bytesToTransfer = writeLength;
    state->writeI2cTransactionAction.isDone = false;
    state->writeI2cTransactionAction.repeatedStart = true;
    state->writeI2cTransactionAction.bytesTransferred = 0;

    state->readI2cTransactionAction.isReadTransaction = true;
    state->readI2cTransactionAction.buffer = readBuffer;
    state->readI2cTransactionAction.bytesToTransfer = readLength;
    state->readI2cTransactionAction.isDone = false;
    state->readI2cTransactionAction.repeatedStart = false;
    state->readI2cTransactionAction.bytesTransferred = 0;

    state->currentI2cTransactionAction = &state->writeI2cTransactionAction;

    LPC17_I2c_StartTransaction(controllerIndex);

    while (state->currentI2cTransactionAction->isDone == false && timeout > 0) {
        LPC17_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (state->writeI2cTransactionAction.bytesTransferred != writeLength) {
        writeLength = state->writeI2cTransactionAction.bytesTransferred;
        error = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }
    else {
        readLength = state->readI2cTransactionAction.bytesTransferred;

        if (state->currentI2cTransactionAction->bytesTransferred == readLength)
            error = TinyCLR_I2c_TransferStatus::FullTransfer;
        else if (state->currentI2cTransactionAction->bytesTransferred < readLength && state->currentI2cTransactionAction->bytesTransferred > 0)
            error = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result LPC17_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, uint32_t slaveAddress, TinyCLR_I2c_AddressFormat addressFormat, TinyCLR_I2c_BusSpeed busSpeed) {
    uint32_t rateKhz;

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    if (busSpeed == TinyCLR_I2c_BusSpeed::FastMode)
        rateKhz = 400; // FastMode
    else if (busSpeed == TinyCLR_I2c_BusSpeed::StandardMode)
        rateKhz = 100; // StandardMode
    else
        return TinyCLR_Result::NotSupported;

    uint32_t divider = LPC17xx_I2C::c_I2C_Clk_KHz / (2 * rateKhz);

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    state->i2cConfiguration.clockRate = (uint8_t)divider; // low byte
    state->i2cConfiguration.clockRate2 = (uint8_t)(divider >> 8); // high byte
    state->i2cConfiguration.address = slaveAddress;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_I2c_Acquire(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(controllerIndex == 0 ? LPC17xx_I2C::c_I2C0_Base : ((controllerIndex == 1 ? LPC17xx_I2C::c_I2C1_Base : LPC17xx_I2C::c_I2C2_Base)));

    switch (controllerIndex) {
    case 0:
        LPC17_Interrupt_Activate(I2C0_IRQn, (uint32_t*)&LPC17_I2c_InterruptHandler0, 0);
        break;

    case 1:
        LPC17_Interrupt_Activate(I2C1_IRQn, (uint32_t*)&LPC17_I2c_InterruptHandler1, 0);
        break;

    case 2:
        LPC17_Interrupt_Activate(I2C2_IRQn, (uint32_t*)&LPC17_I2c_InterruptHandler2, 0);
        break;
    }

    if (!LPC17_Gpio_OpenPin(i2cSdaPins[controllerIndex].number) || !LPC17_Gpio_OpenPin(i2cSclPins[controllerIndex].number))
        return TinyCLR_Result::SharingViolation;

    LPC17_Gpio_ConfigurePin(i2cSdaPins[controllerIndex].number, LPC17_Gpio_Direction::Input, i2cSdaPins[controllerIndex].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(i2cSclPins[controllerIndex].number, LPC17_Gpio_Direction::Input, i2cSclPins[controllerIndex].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    // enable the I2c module
    I2C.I2CONSET = LPC17xx_I2C::I2EN;

    // set the slave address
    I2C.I2ADR = 0x7E;

    state->i2cConfiguration.isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_I2c_Release(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);
    auto controllerIndex = state->controllerIndex;

    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(controllerIndex == 0 ? LPC17xx_I2C::c_I2C0_Base : ((controllerIndex == 1 ? LPC17xx_I2C::c_I2C1_Base : LPC17xx_I2C::c_I2C2_Base)));

    LPC17_Interrupt_Deactivate(controllerIndex == 0 ? I2C0_IRQn : (controllerIndex == 1 ? I2C1_IRQn : I2C2_IRQn));

    I2C.I2CONCLR = (LPC17xx_I2C::AA | LPC17xx_I2C::SI | LPC17xx_I2C::STO | LPC17xx_I2C::STA | LPC17xx_I2C::I2EN);

    if (state->i2cConfiguration.isOpened) {
        LPC17_Gpio_ClosePin(i2cSdaPins[controllerIndex].number);
        LPC17_Gpio_ClosePin(i2cSclPins[controllerIndex].number);
    }

    state->i2cConfiguration.isOpened = false;

    return TinyCLR_Result::Success;
}

void LPC17_I2c_Reset() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        LPC17_I2c_Release(&i2cControllers[i]);

        auto state = &i2cStates[i];

        state->i2cConfiguration.address = 0;
        state->i2cConfiguration.clockRate = 0;
        state->i2cConfiguration.clockRate2 = 0;

        state->readI2cTransactionAction.bytesToTransfer = 0;
        state->readI2cTransactionAction.bytesTransferred = 0;

        state->writeI2cTransactionAction.bytesToTransfer = 0;
        state->writeI2cTransactionAction.bytesTransferred = 0;

        state->i2cConfiguration.isOpened = false;
    }
}

