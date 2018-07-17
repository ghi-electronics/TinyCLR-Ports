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

void STM32F4_I2c_StartTransaction(int32_t controllerIndex);
void STM32F4_I2c_StopTransaction(int32_t controllerIndex);

static const STM32F4_Gpio_Pin g_STM32F4_I2c_Scl_Pins[] = STM32F4_I2C_SCL_PINS;
static const STM32F4_Gpio_Pin g_STM32F4_I2c_Sda_Pins[] = STM32F4_I2C_SDA_PINS;

static const int TOTAL_I2C_CONTROLLERS = SIZEOF_ARRAY(g_STM32F4_I2c_Scl_Pins);

static I2C_TypeDef* g_STM32_I2c_Port[TOTAL_I2C_CONTROLLERS];

#define I2C_TRANSACTION_TIMEOUT 2000 // 2 seconds

struct STM32F4_I2c_Configuration {

    int32_t     address;
    uint8_t     clockRate;
    uint8_t     clockRate2;

    bool        isOpened;
};
struct STM32F4_I2c_Transaction {
    bool                        isReadTransaction;
    bool                        repeatedStart;
    bool                        isDone;

    uint8_t                     *buffer;

    size_t                      bytesToTransfer;
    size_t                      bytesTransferred;

    TinyCLR_I2c_TransferStatus  result;
};

struct I2cDriver {
    int32_t controllerIndex;

    STM32F4_I2c_Configuration i2cConfiguration;
    STM32F4_I2c_Transaction   *currentI2cTransactionAction;
    STM32F4_I2c_Transaction   readI2cTransactionAction;
    STM32F4_I2c_Transaction   writeI2cTransactionAction;
};

static I2cDriver i2cDrivers[TOTAL_I2C_CONTROLLERS];

static TinyCLR_I2c_Controller i2cControllers[TOTAL_I2C_CONTROLLERS];;
static TinyCLR_Api_Info i2cApi[TOTAL_I2C_CONTROLLERS];;

const TinyCLR_Api_Info* STM32F4_I2c_GetApi() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        i2cControllers[i].ApiInfo = &i2cApi[i];
        i2cControllers[i].Acquire = &STM32F4_I2c_Acquire;
        i2cControllers[i].Release = &STM32F4_I2c_Release;
        i2cControllers[i].SetActiveSettings = &STM32F4_I2c_SetActiveSettings;
        i2cControllers[i].Read = &STM32F4_I2c_Read;
        i2cControllers[i].Write = &STM32F4_I2c_Write;
        i2cControllers[i].WriteRead = &STM32F4_I2c_WriteRead;

        i2cApi[i].Author = "GHI Electronics, LLC";
        i2cApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.I2cController";
        i2cApi[i].Type = TinyCLR_Api_Type::I2cController;
        i2cApi[i].Version = 0;
        i2cApi[i].Implementation = &i2cControllers[i];
        i2cApi[i].State = &i2cDrivers[i];

        i2cDrivers[i].controllerIndex = i;
    }

    if (TOTAL_I2C_CONTROLLERS > 0)
        g_STM32_I2c_Port[0] = I2C1;

    if (TOTAL_I2C_CONTROLLERS > 1)
        g_STM32_I2c_Port[1] = I2C2;

    if (TOTAL_I2C_CONTROLLERS > 2)
        g_STM32_I2c_Port[2] = I2C3;

    return (const TinyCLR_Api_Info*)&i2cApi;
}

void STM32F4_I2C_ER_Interrupt(int32_t controllerIndex) {// Error Interrupt Handler
    INTERRUPT_STARTED_SCOPED(isr);

    auto driver = &i2cDrivers[controllerIndex];

    g_STM32_I2c_Port[controllerIndex]->SR1 = 0; // reset errors

    if (driver->currentI2cTransactionAction != nullptr)
        driver->currentI2cTransactionAction->result = TinyCLR_I2c_TransferStatus::SlaveAddressNotAcknowledged;

    STM32F4_I2c_StopTransaction(controllerIndex);
}

void STM32F4_I2C_EV_Interrupt(int32_t controllerIndex) {// Event Interrupt Handler
    INTERRUPT_STARTED_SCOPED(isr);

    auto& I2Cx = g_STM32_I2c_Port[controllerIndex];

    auto driver = &i2cDrivers[controllerIndex];

    STM32F4_I2c_Transaction *transaction = driver->currentI2cTransactionAction;

    int todo = transaction->bytesToTransfer;
    int sr1 = I2Cx->SR1;  // read status register
    int sr2 = I2Cx->SR2;  // clear ADDR bit
    int cr1 = I2Cx->CR1;  // initial control register

    if (transaction->isReadTransaction) { // read transaction
        if (sr1 & I2C_SR1_SB) { // start bit
            if (todo == 1) {
                I2Cx->CR1 = (cr1 &= ~I2C_CR1_ACK); // last byte nack
            }
            else if (todo == 2) {
                I2Cx->CR1 = (cr1 |= I2C_CR1_POS); // prepare 2nd byte nack
            }
            uint8_t addr = driver->i2cConfiguration.address << 1; // address bits
            I2Cx->DR = addr + 1; // send header byte with read bit;
        }
        else {
            if (sr1 & I2C_SR1_ADDR) { // address sent
                if (todo == 1) {
                    I2Cx->CR1 = (cr1 |= I2C_CR1_STOP); // send stop after single byte
                }
                else if (todo == 2) {
                    I2Cx->CR1 = (cr1 &= ~I2C_CR1_ACK); // last byte nack
                }
            }
            else {
                while (sr1 & I2C_SR1_RXNE) { // data available
                    if (todo == 2) { // 2 bytes remaining
                        I2Cx->CR1 = (cr1 |= I2C_CR1_STOP); // stop after last byte
                    }
                    else if (todo == 3) { // 3 bytes remaining
                        if (!(sr1 & I2C_SR1_BTF)) break; // assure 2 bytes are received
                        I2Cx->CR1 = (cr1 &= ~I2C_CR1_ACK); // last byte nack
                    }
                    uint8_t data = I2Cx->DR; // read data
                    transaction->buffer[transaction->bytesTransferred] = data; // save data
                    transaction->bytesTransferred++;
                    transaction->bytesToTransfer = --todo; // update todo
                    sr1 = I2Cx->SR1;  // update status register copy
                }
            }
            if (todo == 1) {
                I2Cx->CR2 |= I2C_CR2_ITBUFEN; // enable I2C_SR1_RXNE interrupt
            }
        }
    }
    else { // write transaction
        if (sr1 & I2C_SR1_SB) { // start bit
            uint8_t addr = driver->i2cConfiguration.address << 1; // address bits
            I2Cx->DR = addr; // send header byte with write bit;
        }
        else {
            while (todo && (sr1 & I2C_SR1_TXE)) {
                I2Cx->DR = transaction->buffer[transaction->bytesTransferred]; // next data byte;
                transaction->bytesTransferred++;
                transaction->bytesToTransfer = --todo; // update todo
                sr1 = I2Cx->SR1;  // update status register copy
            }
            if (!(sr1 & I2C_SR1_BTF)) todo++; // last byte not yet sent
        }
    }

    if (todo == 0) { // all received or all sent
        if (transaction->repeatedStart) { // start next unit
            I2Cx->CR2 &= ~I2C_CR2_ITBUFEN; // disable I2C_SR1_RXNE interrupt
            I2Cx->CR1 = I2C_CR1_PE | I2C_CR1_START | I2C_CR1_ACK; // send restart

            driver->currentI2cTransactionAction = &driver->readI2cTransactionAction;
        }
        else {
            STM32F4_I2c_StopTransaction(controllerIndex);
        }
    }
}

void STM32F4_I2C1_ER_Interrupt(void *param) {
    STM32F4_I2C_ER_Interrupt(0);
}

void STM32F4_I2C2_ER_Interrupt(void *param) {
    STM32F4_I2C_ER_Interrupt(1);
}

void STM32F4_I2C3_ER_Interrupt(void *param) {
    STM32F4_I2C_ER_Interrupt(2);
}

void STM32F4_I2C1_EV_Interrupt(void *param) {
    STM32F4_I2C_EV_Interrupt(0);
}

void STM32F4_I2C2_EV_Interrupt(void *param) {
    STM32F4_I2C_EV_Interrupt(1);
}

void STM32F4_I2C3_EV_Interrupt(void *param) {
    STM32F4_I2C_EV_Interrupt(2);
}

void STM32F4_I2c_StartTransaction(int32_t controllerIndex) {
    auto& I2Cx = g_STM32_I2c_Port[controllerIndex];

    auto driver = &i2cDrivers[controllerIndex];

    uint32_t ccr = driver->i2cConfiguration.clockRate + (driver->i2cConfiguration.clockRate2 << 8);
    if (I2Cx->CCR != ccr) { // set clock rate and rise time
        uint32_t trise;
        if (ccr & I2C_CCR_FS) { // fast => 0.3ns rise time
            trise = STM32F4_APB1_CLOCK_HZ / (1000 * 3333) + 1; // PCLK1 / 3333kHz
        }
        else { // slow => 1.0ns rise time
            trise = STM32F4_APB1_CLOCK_HZ / (1000 * 1000) + 1; // PCLK1 / 1000kHz
        }
        I2Cx->CR1 = 0; // disable peripheral
        I2Cx->CCR = ccr;
        I2Cx->TRISE = trise;
    }

    I2Cx->CR1 = I2C_CR1_PE; // enable and reset special flags
    I2Cx->SR1 = 0; // reset error flags
    I2Cx->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN; // enable interrupts
    I2Cx->CR1 = I2C_CR1_PE | I2C_CR1_START | I2C_CR1_ACK; // send start
}

void STM32F4_I2c_StopTransaction(int32_t controllerIndex) {
    auto& I2Cx = g_STM32_I2c_Port[controllerIndex];

    auto driver = &i2cDrivers[controllerIndex];

    if (I2Cx->SR2 & I2C_SR2_BUSY && !(I2Cx->CR1 & I2C_CR1_STOP)) {
        I2Cx->CR1 |= I2C_CR1_STOP; // send stop
    }

    I2Cx->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN); // disable interrupts

    driver->currentI2cTransactionAction->isDone = true;
}

TinyCLR_Result STM32F4_I2c_Read(const TinyCLR_I2c_Controller* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    auto driver = reinterpret_cast<I2cDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    driver->readI2cTransactionAction.isReadTransaction = true;
    driver->readI2cTransactionAction.buffer = buffer;
    driver->readI2cTransactionAction.bytesToTransfer = length;
    driver->readI2cTransactionAction.isDone = false;
    driver->readI2cTransactionAction.repeatedStart = false;
    driver->readI2cTransactionAction.bytesTransferred = 0;

    driver->currentI2cTransactionAction = &driver->readI2cTransactionAction;

    STM32F4_I2c_StartTransaction(controllerIndex);

    while (driver->currentI2cTransactionAction->isDone == false && timeout > 0) {
        STM32F4_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (driver->currentI2cTransactionAction->bytesTransferred == length)
        result = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (driver->currentI2cTransactionAction->bytesTransferred < length && driver->currentI2cTransactionAction->bytesTransferred > 0)
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = driver->currentI2cTransactionAction->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F4_I2c_Write(const TinyCLR_I2c_Controller* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    auto driver = reinterpret_cast<I2cDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    driver->writeI2cTransactionAction.isReadTransaction = false;
    driver->writeI2cTransactionAction.buffer = (uint8_t*)buffer;
    driver->writeI2cTransactionAction.bytesToTransfer = length;
    driver->writeI2cTransactionAction.isDone = false;
    driver->writeI2cTransactionAction.repeatedStart = false;
    driver->writeI2cTransactionAction.bytesTransferred = 0;

    driver->currentI2cTransactionAction = &driver->writeI2cTransactionAction;

    STM32F4_I2c_StartTransaction(controllerIndex);

    while (driver->currentI2cTransactionAction->isDone == false && timeout > 0) {
        STM32F4_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (driver->currentI2cTransactionAction->bytesTransferred == length)
        result = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (driver->currentI2cTransactionAction->bytesTransferred < length && driver->currentI2cTransactionAction->bytesTransferred > 0)
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = driver->currentI2cTransactionAction->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F4_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    auto driver = reinterpret_cast<I2cDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    driver->writeI2cTransactionAction.isReadTransaction = false;
    driver->writeI2cTransactionAction.buffer = (uint8_t*)writeBuffer;
    driver->writeI2cTransactionAction.bytesToTransfer = writeLength;
    driver->writeI2cTransactionAction.isDone = false;
    driver->writeI2cTransactionAction.repeatedStart = true;
    driver->writeI2cTransactionAction.bytesTransferred = 0;

    driver->readI2cTransactionAction.isReadTransaction = true;
    driver->readI2cTransactionAction.buffer = readBuffer;
    driver->readI2cTransactionAction.bytesToTransfer = readLength;
    driver->readI2cTransactionAction.isDone = false;
    driver->readI2cTransactionAction.repeatedStart = false;
    driver->readI2cTransactionAction.bytesTransferred = 0;

    driver->currentI2cTransactionAction = &driver->writeI2cTransactionAction;

    STM32F4_I2c_StartTransaction(controllerIndex);

    while (driver->currentI2cTransactionAction->isDone == false && timeout > 0) {
        STM32F4_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (driver->writeI2cTransactionAction.bytesTransferred != writeLength) {
        writeLength = driver->writeI2cTransactionAction.bytesTransferred;
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }
    else {
        readLength = driver->readI2cTransactionAction.bytesTransferred;

        if (driver->currentI2cTransactionAction->bytesTransferred == readLength)
            result = TinyCLR_I2c_TransferStatus::FullTransfer;
        else if (driver->currentI2cTransactionAction->bytesTransferred < readLength && driver->currentI2cTransactionAction->bytesTransferred > 0)
            result = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F4_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed) {
    uint32_t rateKhz;
    uint32_t ccr;

    auto driver = reinterpret_cast<I2cDriver*>(self->ApiInfo->State);

    if (busSpeed == TinyCLR_I2c_BusSpeed::FastMode)
        rateKhz = 400; // FastMode
    else if (busSpeed == TinyCLR_I2c_BusSpeed::StandardMode)
        rateKhz = 100; // StandardMode
    else
        return TinyCLR_Result::NotSupported;

    if (rateKhz <= 100) { // slow clock
        ccr = (STM32F4_APB1_CLOCK_HZ / 1000 / 2 - 1) / rateKhz + 1; // round up
        if (ccr > 0xFFF) ccr = 0xFFF; // max divider
    }
    else { // fast clock
        ccr = (STM32F4_APB1_CLOCK_HZ / 1000 / 3 - 1) / rateKhz + 1; // round up
        ccr |= 0x8000; // set fast mode (duty cycle 1:2)
    }

    driver->i2cConfiguration.clockRate = (uint8_t)ccr; // low byte
    driver->i2cConfiguration.clockRate2 = (uint8_t)(ccr >> 8); // high byte
    driver->i2cConfiguration.address = slaveAddress;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_I2c_Acquire(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto driver = reinterpret_cast<I2cDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    auto& I2Cx = g_STM32_I2c_Port[controllerIndex];

    auto& scl = g_STM32F4_I2c_Scl_Pins[controllerIndex];
    auto& sda = g_STM32F4_I2c_Sda_Pins[controllerIndex];

    if (!STM32F4_GpioInternal_OpenPin(sda.number) || !STM32F4_GpioInternal_OpenPin(scl.number))
        return TinyCLR_Result::SharingViolation;

    STM32F4_GpioInternal_ConfigurePin(sda.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::OpenDrain, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, sda.alternateFunction);
    STM32F4_GpioInternal_ConfigurePin(scl.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::OpenDrain, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, scl.alternateFunction);

    RCC->APB1ENR |= (controllerIndex == 0 ? RCC_APB1ENR_I2C1EN : controllerIndex == 1 ? RCC_APB1ENR_I2C2EN : RCC_APB1ENR_I2C3EN);

    RCC->APB1RSTR = (controllerIndex == 0 ? RCC_APB1RSTR_I2C1RST : controllerIndex == 1 ? RCC_APB1RSTR_I2C2RST : RCC_APB1RSTR_I2C3RST);

    switch (controllerIndex) {
    case 0:
        STM32F4_InterruptInternal_Activate(I2C1_EV_IRQn, (uint32_t*)&STM32F4_I2C1_EV_Interrupt, 0);
        STM32F4_InterruptInternal_Activate(I2C1_ER_IRQn, (uint32_t*)&STM32F4_I2C1_ER_Interrupt, 0);
        break;

    case 1:

        STM32F4_InterruptInternal_Activate(I2C2_EV_IRQn, (uint32_t*)&STM32F4_I2C2_EV_Interrupt, 0);
        STM32F4_InterruptInternal_Activate(I2C2_ER_IRQn, (uint32_t*)&STM32F4_I2C2_ER_Interrupt, 0);
        break;

    case 2:
        STM32F4_InterruptInternal_Activate(I2C3_EV_IRQn, (uint32_t*)&STM32F4_I2C3_EV_Interrupt, 0);
        STM32F4_InterruptInternal_Activate(I2C3_ER_IRQn, (uint32_t*)&STM32F4_I2C3_ER_Interrupt, 0);
        break;
    }

    RCC->APB1RSTR = 0;

    I2Cx->CR2 = STM32F4_APB1_CLOCK_HZ / 1000000; // APB1 clock in MHz
    I2Cx->CCR = (STM32F4_APB1_CLOCK_HZ / 1000 / 2 - 1) / 100 + 1; // 100KHz
    I2Cx->TRISE = STM32F4_APB1_CLOCK_HZ / (1000 * 1000) + 1; // 1ns;
    I2Cx->OAR1 = 0x4000; // init address register

    I2Cx->CR1 = I2C_CR1_PE; // enable peripheral

    driver->i2cConfiguration.isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_I2c_Release(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto driver = reinterpret_cast<I2cDriver*>(self->ApiInfo->State);

    auto controllerIndex = driver->controllerIndex;

    auto& I2Cx = g_STM32_I2c_Port[controllerIndex];

    STM32F4_InterruptInternal_Deactivate(controllerIndex == 0 ? I2C1_EV_IRQn : controllerIndex == 1 ? I2C2_EV_IRQn : I2C3_EV_IRQn);
    STM32F4_InterruptInternal_Deactivate(controllerIndex == 0 ? I2C1_ER_IRQn : controllerIndex == 1 ? I2C2_ER_IRQn : I2C3_ER_IRQn);

    I2Cx->CR1 = 0; // disable peripheral

    RCC->APB1ENR &= (controllerIndex == 0 ? ~RCC_APB1ENR_I2C1EN : controllerIndex == 1 ? ~RCC_APB1ENR_I2C2EN : ~RCC_APB1ENR_I2C3EN);

    if (driver->i2cConfiguration.isOpened) {
        auto& scl = g_STM32F4_I2c_Scl_Pins[controllerIndex];
        auto& sda = g_STM32F4_I2c_Sda_Pins[controllerIndex];

        STM32F4_GpioInternal_ClosePin(sda.number);
        STM32F4_GpioInternal_ClosePin(scl.number);
    }

    driver->i2cConfiguration.isOpened = false;

    return TinyCLR_Result::Success;
}

void STM32F4_I2c_Reset() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        STM32F4_I2c_Release(&i2cControllers[i]);

        auto driver = &i2cDrivers[i];

        driver->i2cConfiguration.address = 0;
        driver->i2cConfiguration.clockRate = 0;
        driver->i2cConfiguration.clockRate2 = 0;

        driver->readI2cTransactionAction.bytesToTransfer = 0;
        driver->readI2cTransactionAction.bytesTransferred = 0;

        driver->writeI2cTransactionAction.bytesToTransfer = 0;
        driver->writeI2cTransactionAction.bytesTransferred = 0;

        driver->i2cConfiguration.isOpened = false;
    }
}
