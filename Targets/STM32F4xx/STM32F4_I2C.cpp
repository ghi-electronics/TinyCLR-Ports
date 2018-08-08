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

static const STM32F4_Gpio_Pin i2cSclPins[] = STM32F4_I2C_SCL_PINS;
static const STM32F4_Gpio_Pin i2cSdaPins[] = STM32F4_I2C_SDA_PINS;

static I2C_TypeDef* i2cPorts[TOTAL_I2C_CONTROLLERS];

#define I2C_TRANSACTION_TIMEOUT 2000 // 2 seconds

struct I2cConfiguration {

    int32_t     address;
    uint8_t     clockRate;
    uint8_t     clockRate2;

    bool        isOpened;
};
struct I2cTransaction {
    bool                        isReadTransaction;
    bool                        repeatedStart;
    bool                        isDone;

    uint8_t                     *buffer;

    size_t                      bytesToTransfer;
    size_t                      bytesTransferred;

    TinyCLR_I2c_TransferStatus  error;
};

struct I2cState {
    int32_t controllerIndex;

    I2cConfiguration i2cConfiguration;
    I2cTransaction   *currentI2cTransactionAction;
    I2cTransaction   readI2cTransactionAction;
    I2cTransaction   writeI2cTransactionAction;

    uint32_t intializeCount;
};

static I2cState i2cStates[TOTAL_I2C_CONTROLLERS];

static TinyCLR_I2c_Controller i2cControllers[TOTAL_I2C_CONTROLLERS];
static TinyCLR_Api_Info i2cApi[TOTAL_I2C_CONTROLLERS];

const char* i2cApiNames[] = {
#if TOTAL_I2C_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.STM32F4.I2cController\\0",
#if TOTAL_I2C_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.STM32F4.I2cController\\1",
#if TOTAL_I2C_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.STM32F4.I2cController\\2"
#endif
#endif
#endif
};

void STM32F4_I2c_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        i2cControllers[i].ApiInfo = &i2cApi[i];
        i2cControllers[i].Acquire = &STM32F4_I2c_Acquire;
        i2cControllers[i].Release = &STM32F4_I2c_Release;
        i2cControllers[i].SetActiveSettings = &STM32F4_I2c_SetActiveSettings;
        i2cControllers[i].WriteRead = &STM32F4_I2c_WriteRead;

        i2cApi[i].Author = "GHI Electronics, LLC";
        i2cApi[i].Name = i2cApiNames[i];
        i2cApi[i].Type = TinyCLR_Api_Type::I2cController;
        i2cApi[i].Version = 0;
        i2cApi[i].Implementation = &i2cControllers[i];
        i2cApi[i].State = &i2cStates[i];

        i2cStates[i].controllerIndex = i;

        apiManager->Add(apiManager, &i2cApi[i]);
    }

    if (TOTAL_I2C_CONTROLLERS > 0)
        i2cPorts[0] = I2C1;

    if (TOTAL_I2C_CONTROLLERS > 1)
        i2cPorts[1] = I2C2;

    if (TOTAL_I2C_CONTROLLERS > 2)
        i2cPorts[2] = I2C3;


}

void STM32F4_I2C_ER_Interrupt(int32_t controllerIndex) {// Error Interrupt Handler
    INTERRUPT_STARTED_SCOPED(isr);

    auto state = &i2cStates[controllerIndex];

    i2cPorts[controllerIndex]->SR1 = 0; // reset errors

    if (state->currentI2cTransactionAction != nullptr)
        state->currentI2cTransactionAction->error = TinyCLR_I2c_TransferStatus::SlaveAddressNotAcknowledged;

    STM32F4_I2c_StopTransaction(controllerIndex);
}

void STM32F4_I2C_EV_Interrupt(int32_t controllerIndex) {// Event Interrupt Handler
    INTERRUPT_STARTED_SCOPED(isr);

    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    I2cTransaction *transaction = state->currentI2cTransactionAction;

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
            uint8_t addr = state->i2cConfiguration.address << 1; // address bits
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
            uint8_t addr = state->i2cConfiguration.address << 1; // address bits
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

            state->currentI2cTransactionAction = &state->readI2cTransactionAction;
        }
        else {
            STM32F4_I2c_StopTransaction(controllerIndex);
        }
    }
}
#if TOTAL_I2C_CONTROLLERS > 0
void STM32F4_I2C1_ER_Interrupt(void *param) {
    STM32F4_I2C_ER_Interrupt(0);
}

void STM32F4_I2C1_EV_Interrupt(void *param) {
    STM32F4_I2C_EV_Interrupt(0);
}

#if TOTAL_I2C_CONTROLLERS > 1
void STM32F4_I2C2_ER_Interrupt(void *param) {
    STM32F4_I2C_ER_Interrupt(1);
}

void STM32F4_I2C2_EV_Interrupt(void *param) {
    STM32F4_I2C_EV_Interrupt(1);
}

#if TOTAL_I2C_CONTROLLERS > 2
void STM32F4_I2C3_ER_Interrupt(void *param) {
    STM32F4_I2C_ER_Interrupt(2);
}

void STM32F4_I2C3_EV_Interrupt(void *param) {
    STM32F4_I2C_EV_Interrupt(2);
}
#endif
#endif
#endif

void STM32F4_I2c_StartTransaction(int32_t controllerIndex) {
    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    uint32_t ccr = state->i2cConfiguration.clockRate + (state->i2cConfiguration.clockRate2 << 8);
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
    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    if (I2Cx->SR2 & I2C_SR2_BUSY && !(I2Cx->CR1 & I2C_CR1_STOP)) {
        I2Cx->CR1 |= I2C_CR1_STOP; // send stop
    }

    I2Cx->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN); // disable interrupts

    state->currentI2cTransactionAction->isDone = true;
}

TinyCLR_Result STM32F4_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error) {
    if ((!(sendStartCondition & sendStopCondition)) || (readLength == 0 && writeLength == 0))
        return TinyCLR_Result::NotSupported;

    auto timeout = I2C_TRANSACTION_TIMEOUT;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    auto controllerIndex = state->controllerIndex;

    state->writeI2cTransactionAction.isReadTransaction = false;
    state->writeI2cTransactionAction.buffer = (uint8_t*)writeBuffer;
    state->writeI2cTransactionAction.bytesToTransfer = writeLength;
    state->writeI2cTransactionAction.isDone = false;
    state->writeI2cTransactionAction.repeatedStart = readLength > 0 ? true : false;
    state->writeI2cTransactionAction.bytesTransferred = 0;

    state->readI2cTransactionAction.isReadTransaction = true;
    state->readI2cTransactionAction.buffer = readBuffer;
    state->readI2cTransactionAction.bytesToTransfer = readLength;
    state->readI2cTransactionAction.isDone = false;
    state->readI2cTransactionAction.repeatedStart = false;
    state->readI2cTransactionAction.bytesTransferred = 0;

    state->currentI2cTransactionAction = writeLength > 0 ? &state->writeI2cTransactionAction : &state->readI2cTransactionAction;

    error = TinyCLR_I2c_TransferStatus::FullTransfer;

    STM32F4_I2c_StartTransaction(controllerIndex);

    while (state->currentI2cTransactionAction->isDone == false && timeout > 0) {
        STM32F4_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (state->writeI2cTransactionAction.bytesTransferred != writeLength) {
        if (state->writeI2cTransactionAction.bytesTransferred == 0) {
            error = TinyCLR_I2c_TransferStatus::SlaveAddressNotAcknowledged;
        }
        else
            error = TinyCLR_I2c_TransferStatus::PartialTransfer;

        writeLength = state->writeI2cTransactionAction.bytesTransferred;
    }

    if (state->readI2cTransactionAction.bytesTransferred != readLength) {
        if (state->readI2cTransactionAction.bytesTransferred == 0) {
            error = TinyCLR_I2c_TransferStatus::SlaveAddressNotAcknowledged;
        }
        else
            error = TinyCLR_I2c_TransferStatus::PartialTransfer;

        readLength = state->readI2cTransactionAction.bytesTransferred;
    }

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F4_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, uint32_t slaveAddress, TinyCLR_I2c_AddressFormat addressFormat, TinyCLR_I2c_BusSpeed busSpeed) {
    uint32_t rateKhz;
    uint32_t ccr;

    if (addressFormat == TinyCLR_I2c_AddressFormat::TenBit)
        return TinyCLR_Result::NotSupported;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

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

    state->i2cConfiguration.clockRate = (uint8_t)ccr; // low byte
    state->i2cConfiguration.clockRate2 = (uint8_t)(ccr >> 8); // high byte
    state->i2cConfiguration.address = slaveAddress;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_I2c_Acquire(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (state->intializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        auto& I2Cx = i2cPorts[controllerIndex];

        auto& scl = i2cSclPins[controllerIndex];
        auto& sda = i2cSdaPins[controllerIndex];

        if (!STM32F4_GpioInternal_OpenPin(sda.number) || !STM32F4_GpioInternal_OpenPin(scl.number))
            return TinyCLR_Result::SharingViolation;

        STM32F4_GpioInternal_ConfigurePin(sda.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::OpenDrain, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, sda.alternateFunction);
        STM32F4_GpioInternal_ConfigurePin(scl.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::OpenDrain, STM32F4_Gpio_OutputSpeed::VeryHigh, STM32F4_Gpio_PullDirection::PullUp, scl.alternateFunction);

        RCC->APB1ENR |= (controllerIndex == 0 ? RCC_APB1ENR_I2C1EN : controllerIndex == 1 ? RCC_APB1ENR_I2C2EN : RCC_APB1ENR_I2C3EN);

        RCC->APB1RSTR = (controllerIndex == 0 ? RCC_APB1RSTR_I2C1RST : controllerIndex == 1 ? RCC_APB1RSTR_I2C2RST : RCC_APB1RSTR_I2C3RST);

        switch (controllerIndex) {
#if TOTAL_I2C_CONTROLLERS > 0
        case 0:
            STM32F4_InterruptInternal_Activate(I2C1_EV_IRQn, (uint32_t*)&STM32F4_I2C1_EV_Interrupt, 0);
            STM32F4_InterruptInternal_Activate(I2C1_ER_IRQn, (uint32_t*)&STM32F4_I2C1_ER_Interrupt, 0);
            break;
#if TOTAL_I2C_CONTROLLERS > 1
        case 1:

            STM32F4_InterruptInternal_Activate(I2C2_EV_IRQn, (uint32_t*)&STM32F4_I2C2_EV_Interrupt, 0);
            STM32F4_InterruptInternal_Activate(I2C2_ER_IRQn, (uint32_t*)&STM32F4_I2C2_ER_Interrupt, 0);
            break;
#if TOTAL_I2C_CONTROLLERS > 2
        case 2:
            STM32F4_InterruptInternal_Activate(I2C3_EV_IRQn, (uint32_t*)&STM32F4_I2C3_EV_Interrupt, 0);
            STM32F4_InterruptInternal_Activate(I2C3_ER_IRQn, (uint32_t*)&STM32F4_I2C3_ER_Interrupt, 0);
            break;
#endif
#endif
#endif
        }

        RCC->APB1RSTR = 0;

        I2Cx->CR2 = STM32F4_APB1_CLOCK_HZ / 1000000; // APB1 clock in MHz
        I2Cx->CCR = (STM32F4_APB1_CLOCK_HZ / 1000 / 2 - 1) / 100 + 1; // 100KHz
        I2Cx->TRISE = STM32F4_APB1_CLOCK_HZ / (1000 * 1000) + 1; // 1ns;
        I2Cx->OAR1 = 0x4000; // init address register

        I2Cx->CR1 = I2C_CR1_PE; // enable peripheral

        state->i2cConfiguration.isOpened = true;
    }

    state->intializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_I2c_Release(const TinyCLR_I2c_Controller* self) {
    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR::InvalidOperation;

    state->intializeCount--;

    if (state->intializeCount == 0) {

        auto controllerIndex = state->controllerIndex;

        auto& I2Cx = i2cPorts[controllerIndex];

        STM32F4_InterruptInternal_Deactivate(controllerIndex == 0 ? I2C1_EV_IRQn : controllerIndex == 1 ? I2C2_EV_IRQn : I2C3_EV_IRQn);
        STM32F4_InterruptInternal_Deactivate(controllerIndex == 0 ? I2C1_ER_IRQn : controllerIndex == 1 ? I2C2_ER_IRQn : I2C3_ER_IRQn);

        I2Cx->CR1 = 0; // disable peripheral

        RCC->APB1ENR &= (controllerIndex == 0 ? ~RCC_APB1ENR_I2C1EN : controllerIndex == 1 ? ~RCC_APB1ENR_I2C2EN : ~RCC_APB1ENR_I2C3EN);

        if (state->i2cConfiguration.isOpened) {
            auto& scl = i2cSclPins[controllerIndex];
            auto& sda = i2cSdaPins[controllerIndex];

            STM32F4_GpioInternal_ClosePin(sda.number);
            STM32F4_GpioInternal_ClosePin(scl.number);
        }

        state->i2cConfiguration.isOpened = false;
    }

    return TinyCLR_Result::Success;
}

void STM32F4_I2c_Reset() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        STM32F4_I2c_Release(&i2cControllers[i]);

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
