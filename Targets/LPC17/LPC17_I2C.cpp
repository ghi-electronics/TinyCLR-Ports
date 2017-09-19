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

#include <LPC17.h>

struct LPC17xx_I2C {
    static const uint32_t c_I2C_Base = 0x4001C000;
    static const uint32_t c_I2C_Clk_KHz = LPC17_SYSTEM_CLOCK_HZ/2/1000;

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

struct LPC17_I2c_Configuration {

    int32_t                  address;
    uint8_t                  clockRate;     // primary clock factor to generate the i2c clock
    uint8_t                  clockRate2;   // additional clock factors, if more than one is needed for the clock (optional)
};

struct LPC17_I2c_Transaction {
    bool                        isReadTransaction;
    bool                        repeatedStart;
    bool                        isDone;

    uint8_t                     *buffer;

    size_t                      bytesToTransfer;
    size_t                      bytesTransferred;

    TinyCLR_I2c_TransferStatus  result;
};

#define I2C_TRANSACTION_TIMEOUT 2000 // 2 seconds

static LPC17_I2c_Configuration g_I2cConfiguration;
static LPC17_I2c_Transaction   *g_currentI2cTransactionAction;
static LPC17_I2c_Transaction   g_ReadI2cTransactionAction;
static LPC17_I2c_Transaction   g_WriteI2cTransactionAction;

static const LPC17_Gpio_Pin g_i2c_scl_pins[] = LPC17_I2C_SCL_PINS;
static const LPC17_Gpio_Pin g_i2c_sda_pins[] = LPC17_I2C_SDA_PINS;

static TinyCLR_I2c_Provider i2cProvider;
static TinyCLR_Api_Info i2cApi;

const TinyCLR_Api_Info* LPC17_I2c_GetApi() {
    i2cProvider.Parent = &i2cApi;
    i2cProvider.Index = 0;
    i2cProvider.Acquire = &LPC17_I2c_Acquire;
    i2cProvider.Release = &LPC17_I2c_Release;
    i2cProvider.SetActiveSettings = &LPC17_I2c_SetActiveSettings;
    i2cProvider.Read = &LPC17_I2c_ReadTransaction;
    i2cProvider.Write = &LPC17_I2c_WriteTransaction;
    i2cProvider.WriteRead = &LPC17_I2c_WriteReadTransaction;

    i2cApi.Author = "GHI Electronics, LLC";
    i2cApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.I2cProvider";
    i2cApi.Type = TinyCLR_Api_Type::I2cProvider;
    i2cApi.Version = 0;
    i2cApi.Count = 1;
    i2cApi.Implementation = &i2cProvider;

    return &i2cApi;
}

void LPC17_I2c_InterruptHandler(void *param) {
    uint8_t address;
    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(LPC17xx_I2C::c_I2C_Base);

    // read status
    uint8_t status = I2C.I2STAT;

    LPC17_I2c_Transaction *transaction = g_currentI2cTransactionAction;

    if (!transaction) {
        I2C.I2CONCLR = LPC17xx_I2C::SI;
        return;
    }

    switch (status) {
        case 0x08: // Start Condition transmitted
        case 0x10: // Repeated Start Condition transmitted
            // Write Slave address and Data direction
            address = 0xFE & (g_I2cConfiguration.address << 1);
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
                    LPC17_I2c_StopTransaction();
                }
                else {
                    g_currentI2cTransactionAction = &g_ReadI2cTransactionAction;
                    LPC17_I2c_StartTransaction();
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
            LPC17_I2c_StopTransaction();
            break;
        case 0x38: // Arbitration lost
            LPC17_I2c_StopTransaction();
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
                    LPC17_I2c_StopTransaction();
                }
                else {
                    // start next
                    g_currentI2cTransactionAction = &g_ReadI2cTransactionAction;
                    LPC17_I2c_StartTransaction();
                }
            }
            break;
        case 0x00: // Bus Error
            // Clear Bus error
            I2C.I2CONSET = LPC17xx_I2C::STO;
            LPC17_I2c_StopTransaction();
            break;
        default:
            LPC17_I2c_StopTransaction();
            break;
    } // switch(status)

    // clear the interrupt flag to start the next I2C transfer
    I2C.I2CONCLR = LPC17xx_I2C::SI;

}
void LPC17_I2c_StartTransaction() {
    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(LPC17xx_I2C::c_I2C_Base);

    if (!g_WriteI2cTransactionAction.repeatedStart || g_WriteI2cTransactionAction.bytesTransferred == 0) {
        I2C.I2SCLH = g_I2cConfiguration.clockRate | (g_I2cConfiguration.clockRate2 << 8);
        I2C.I2SCLL = g_I2cConfiguration.clockRate | (g_I2cConfiguration.clockRate2 << 8);

        I2C.I2CONSET = LPC17xx_I2C::STA;
    }
    else {
        I2C.I2CONSET = LPC17xx_I2C::STA;
    }

}

void LPC17_I2c_StopTransaction() {
    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(LPC17xx_I2C::c_I2C_Base);

    I2C.I2CONSET = LPC17xx_I2C::STO;
    I2C.I2CONCLR = LPC17xx_I2C::AA | LPC17xx_I2C::SI | LPC17xx_I2C::STA;

    g_currentI2cTransactionAction->isDone = true;
}

TinyCLR_Result LPC17_I2c_ReadTransaction(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    g_ReadI2cTransactionAction.isReadTransaction = true;
    g_ReadI2cTransactionAction.buffer = buffer;
    g_ReadI2cTransactionAction.bytesToTransfer = length;
    g_ReadI2cTransactionAction.isDone = false;
    g_ReadI2cTransactionAction.repeatedStart = false;
    g_ReadI2cTransactionAction.bytesTransferred = 0;

    g_currentI2cTransactionAction = &g_ReadI2cTransactionAction;

    LPC17_I2c_StartTransaction();

    while (g_currentI2cTransactionAction->isDone == false && timeout > 0) {
        LPC17_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (g_currentI2cTransactionAction->bytesTransferred == length)
        result = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (g_currentI2cTransactionAction->bytesTransferred < length && g_currentI2cTransactionAction->bytesTransferred > 0)
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = g_currentI2cTransactionAction->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result LPC17_I2c_WriteTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    g_WriteI2cTransactionAction.isReadTransaction = false;
    g_WriteI2cTransactionAction.buffer = (uint8_t*)buffer;
    g_WriteI2cTransactionAction.bytesToTransfer = length;
    g_WriteI2cTransactionAction.isDone = false;
    g_WriteI2cTransactionAction.repeatedStart = false;
    g_WriteI2cTransactionAction.bytesTransferred = 0;

    g_currentI2cTransactionAction = &g_WriteI2cTransactionAction;

    LPC17_I2c_StartTransaction();

    while (g_currentI2cTransactionAction->isDone == false && timeout > 0) {
        LPC17_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (g_currentI2cTransactionAction->bytesTransferred == length)
        result = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (g_currentI2cTransactionAction->bytesTransferred < length && g_currentI2cTransactionAction->bytesTransferred > 0)
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = g_currentI2cTransactionAction->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result LPC17_I2c_WriteReadTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    g_WriteI2cTransactionAction.isReadTransaction = false;
    g_WriteI2cTransactionAction.buffer = (uint8_t*)writeBuffer;
    g_WriteI2cTransactionAction.bytesToTransfer = writeLength;
    g_WriteI2cTransactionAction.isDone = false;
    g_WriteI2cTransactionAction.repeatedStart = true;
    g_WriteI2cTransactionAction.bytesTransferred = 0;

    g_ReadI2cTransactionAction.isReadTransaction = true;
    g_ReadI2cTransactionAction.buffer = readBuffer;
    g_ReadI2cTransactionAction.bytesToTransfer = readLength;
    g_ReadI2cTransactionAction.isDone = false;
    g_ReadI2cTransactionAction.repeatedStart = false;
    g_ReadI2cTransactionAction.bytesTransferred = 0;

    g_currentI2cTransactionAction = &g_WriteI2cTransactionAction;

    LPC17_I2c_StartTransaction();

    while (g_currentI2cTransactionAction->isDone == false && timeout > 0) {
        LPC17_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (g_WriteI2cTransactionAction.bytesTransferred != writeLength) {
        writeLength = g_WriteI2cTransactionAction.bytesTransferred;
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }
    else {
        readLength = g_ReadI2cTransactionAction.bytesTransferred;

        if (g_currentI2cTransactionAction->bytesTransferred == readLength)
            result = TinyCLR_I2c_TransferStatus::FullTransfer;
        else if (g_currentI2cTransactionAction->bytesTransferred < readLength && g_currentI2cTransactionAction->bytesTransferred > 0)
            result = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result LPC17_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed) {
    uint32_t rateKhz;

    if (busSpeed == TinyCLR_I2c_BusSpeed::FastMode)
        rateKhz = 400; // FastMode
    else if (busSpeed == TinyCLR_I2c_BusSpeed::StandardMode)
        rateKhz = 100; // StandardMode
    else
        return TinyCLR_Result::NotSupported;

    uint32_t divider = LPC17xx_I2C::c_I2C_Clk_KHz / (2 * rateKhz);

    g_I2cConfiguration.clockRate = (uint8_t)divider; // low byte
    g_I2cConfiguration.clockRate2 = (uint8_t)(divider >> 8); // high byte
    g_I2cConfiguration.address = slaveAddress;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_I2c_Acquire(const TinyCLR_I2c_Provider* self) {
    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(LPC17xx_I2C::c_I2C_Base);

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    if (!LPC17_Gpio_OpenPin(g_i2c_sda_pins[self->Index].number) || !LPC17_Gpio_OpenPin(g_i2c_scl_pins[self->Index].number))
        return TinyCLR_Result::SharingViolation;

    LPC17_Gpio_ConfigurePin(g_i2c_sda_pins[self->Index].number, LPC17_Gpio_Direction::Input, g_i2c_sda_pins[self->Index].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
    LPC17_Gpio_ConfigurePin(g_i2c_scl_pins[self->Index].number, LPC17_Gpio_Direction::Input, g_i2c_scl_pins[self->Index].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

    LPC17_Interrupt_Activate(I2C0_IRQn, (uint32_t*)&LPC17_I2c_InterruptHandler, 0);

    // enable the I2c module
    I2C.I2CONSET = LPC17xx_I2C::I2EN;

    // set the slave address
    I2C.I2ADR = 0x7E;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_I2c_Release(const TinyCLR_I2c_Provider* self) {

    LPC17xx_I2C& I2C = *(LPC17xx_I2C*)(size_t)(LPC17xx_I2C::c_I2C_Base);

    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    LPC17_Interrupt_Deactivate(I2C0_IRQn);

    I2C.I2CONCLR = (LPC17xx_I2C::AA | LPC17xx_I2C::SI | LPC17xx_I2C::STO | LPC17xx_I2C::STA | LPC17xx_I2C::I2EN);

    LPC17_Gpio_ClosePin(g_i2c_sda_pins[self->Index].number);
    LPC17_Gpio_ClosePin(g_i2c_scl_pins[self->Index].number);

    return TinyCLR_Result::Success;
}

void LPC17_I2c_Reset() {
    LPC17_I2c_Release(&i2cProvider);
}

