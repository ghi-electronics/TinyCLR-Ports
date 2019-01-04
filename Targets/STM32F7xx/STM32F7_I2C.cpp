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

/** @defgroup I2C_StartStopMode_definition I2C StartStopMode definition
  * @{
  */
#define  I2C_NO_STARTSTOP               ((uint32_t)0x00000000)
#define  I2C_GENERATE_STOP              I2C_CR2_STOP
#define  I2C_GENERATE_START_READ        (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN)
#define  I2C_GENERATE_START_WRITE       I2C_CR2_START

  /** @defgroup I2C_ReloadEndMode_definition I2C ReloadEndMode definition
    * @{
    */
#define  I2C_RELOAD_MODE                I2C_CR2_RELOAD
#define  I2C_AUTOEND_MODE               I2C_CR2_AUTOEND
#define  I2C_SOFTEND_MODE               ((uint32_t)0x00000000)

#define I2C_TRANSACTION_TIMEOUT 2000 // 2 seconds

#define I2C_MAX_TRANSFER 255

void STM32F7_I2c_StartTransaction(int32_t controllerIndex);
void STM32F7_I2c_StopTransaction(int32_t controllerIndex);

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

static const STM32F7_Gpio_Pin i2cPins[][2] = STM32F7_I2C_PINS;

static I2C_TypeDef* i2cPorts[TOTAL_I2C_CONTROLLERS];

struct I2cConfiguration {

    int32_t     address;
    uint8_t     clockRate;
    uint8_t     clockRate2;
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

struct I2cState {
    int32_t controllerIndex;

    I2cConfiguration i2cConfiguration;
    I2cTransaction   *currentI2cTransactionAction;
    I2cTransaction   readI2cTransactionAction;
    I2cTransaction   writeI2cTransactionAction;

    uint16_t initializeCount;
};

static I2cState i2cStates[TOTAL_I2C_CONTROLLERS];

static TinyCLR_I2c_Controller i2cControllers[TOTAL_I2C_CONTROLLERS];
static TinyCLR_Api_Info i2cApi[TOTAL_I2C_CONTROLLERS];

const char* i2cApiNames[] = {
#if TOTAL_I2C_CONTROLLERS > 0
"GHIElectronics.TinyCLR.NativeApis.STM32F7.I2cController\\0",
#if TOTAL_I2C_CONTROLLERS > 1
"GHIElectronics.TinyCLR.NativeApis.STM32F7.I2cController\\1",
#if TOTAL_I2C_CONTROLLERS > 2
"GHIElectronics.TinyCLR.NativeApis.STM32F7.I2cController\\2"
#endif
#endif
#endif
};

void STM32F7_I2c_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        i2cControllers[i].ApiInfo = &i2cApi[i];
        i2cControllers[i].Acquire = &STM32F7_I2c_Acquire;
        i2cControllers[i].Release = &STM32F7_I2c_Release;
        i2cControllers[i].SetActiveSettings = &STM32F7_I2c_SetActiveSettings;
        i2cControllers[i].WriteRead = &STM32F7_I2c_WriteRead;

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
}

void STM32F7_I2c_Enable(I2C_TypeDef* i2c) {
    SET_BIT(i2c->CR1, I2C_CR1_PE);
}

void STM32F7_I2c_Disable(I2C_TypeDef* i2c) {
    CLEAR_BIT(i2c->CR1, I2C_CR1_PE);
}

uint32_t STM32F7_I2c_GetFlag(I2C_TypeDef* i2c, uint32_t flag) {
    const uint32_t I2C_FLAG_MASK = ((uint32_t)0x0001FFFF);
    return (((i2c->ISR) & ((flag)& I2C_FLAG_MASK)) == ((flag)& I2C_FLAG_MASK)) ? SET : RESET;
}

void STM32F7_I2c_ClearFlag(I2C_TypeDef* i2c, uint32_t flag) {
    const uint32_t I2C_FLAG_MASK = ((uint32_t)0x0001FFFF);
    (i2c->ICR = ((flag)& I2C_FLAG_MASK));
}

uint32_t STM32F7_I2c_GetInterruptSource(I2C_TypeDef* i2c, uint32_t interruptFlag) {
    return (((i2c->CR1 & (interruptFlag)) == (interruptFlag)) ? SET : RESET);
}

void STM32F7_I2c_InterruptDisable(I2C_TypeDef* i2c, uint32_t interruptFlag) {
    i2c->CR1 &= ~(interruptFlag);
}

void STM32F7_I2c_InterruptEnable(I2C_TypeDef* i2c, uint32_t interruptFlag) {
    i2c->CR1 |= interruptFlag;
}

void STM32F7_I2c_InternalTransferConfig(int32_t controllerIndex, uint16_t deviceAddress, uint8_t bytesToTransfer, uint32_t transferMode, uint32_t request) {
    uint32_t tmpreg = 0;

    deviceAddress = deviceAddress << 1;
    if (request == I2C_GENERATE_START_READ) {
        deviceAddress += 1;
    }

    auto& I2Cx = i2cPorts[controllerIndex];

    /* Get the CR2 register value */
    tmpreg = I2Cx->CR2;

    /* clear tmpreg specific bits */
    tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));

    /* update tmpreg */
    tmpreg |= (uint32_t)(((uint32_t)deviceAddress & I2C_CR2_SADD) | (((uint32_t)bytesToTransfer << 16) & I2C_CR2_NBYTES) | \
        (uint32_t)transferMode | (uint32_t)request);

    tmpreg |= I2C_CR2_NACK;

    /* update CR2 register */
    I2Cx->CR2 = tmpreg;

}

void STM32F7_I2C_ER_Interrupt(int32_t controllerIndex) {// Error Interrupt Handler
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    /* I2C Bus error interrupt occurred ------------------------------------*/
    if ((STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_BERR) == SET) && (STM32F7_I2c_GetInterruptSource(I2Cx, I2C_CR1_ERRIE) == SET)) {
        /* Clear BERR flag */
        STM32F7_I2c_ClearFlag(I2Cx, I2C_ISR_BERR);
    }

    /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
    if ((STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_OVR) == SET) && (STM32F7_I2c_GetInterruptSource(I2Cx, I2C_CR1_ERRIE) == SET)) {
        /* Clear OVR flag */
        STM32F7_I2c_ClearFlag(I2Cx, I2C_ISR_OVR);
    }

    /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
    if ((STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_ARLO) == SET) && (STM32F7_I2c_GetInterruptSource(I2Cx, I2C_CR1_ERRIE) == SET)) {

        /* Clear ARLO flag */
        STM32F7_I2c_ClearFlag(I2Cx, I2C_ISR_ARLO);
    }

    if (state->currentI2cTransactionAction != nullptr)
        state->currentI2cTransactionAction->error = TinyCLR_I2c_TransferStatus::SlaveAddressNotAcknowledged;

    STM32F7_I2c_StopTransaction(controllerIndex);
}

void STM32F7_I2C1_ER_Interrupt(void *param) {
    STM32F7_I2C_ER_Interrupt(0);
}

void STM32F7_I2C2_ER_Interrupt(void *param) {
    STM32F7_I2C_ER_Interrupt(1);
}

void STM32F7_I2C_EV_Interrupt(int32_t controllerIndex) {// Event Interrupt Handler
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    I2cTransaction *transaction = state->currentI2cTransactionAction;

    int todo = transaction->bytesToTransfer;

    if ((STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_RXNE) == SET) && (STM32F7_I2c_GetInterruptSource(I2Cx, (I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_RXIE)) == SET)) {
        if (transaction->isReadTransaction) {

            uint8_t data = I2Cx->RXDR; // read data
            transaction->buffer[transaction->bytesTransferred] = data;
            transaction->bytesTransferred++;
            transaction->bytesToTransfer = --todo; // update todo
        }
        else {
            // Handle Error
        }
    }
    else if ((STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_TXIS) == SET) && (STM32F7_I2c_GetInterruptSource(I2Cx, (I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE)) == SET)) {
        if (!transaction->isReadTransaction) {

            I2Cx->TXDR = transaction->buffer[transaction->bytesTransferred]; // next data byte;
            transaction->bytesTransferred++;
            transaction->bytesToTransfer = --todo; // update todo
        }
        else {
            // Handle Error
        }
    }
    if (STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_TCR) == SET) {
        if ((transaction->bytesTransferred%I2C_MAX_TRANSFER == 0) && (todo != 0)) {
            if (todo > I2C_MAX_TRANSFER) {
                STM32F7_I2c_InternalTransferConfig(controllerIndex, state->i2cConfiguration.address, I2C_MAX_TRANSFER, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            }
            else {
                STM32F7_I2c_InternalTransferConfig(controllerIndex, state->i2cConfiguration.address, todo, 0, I2C_NO_STARTSTOP);
            }
        }
    }

    if (STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_STOPF) == SET) {
        /* Clear STOP Flag */
        STM32F7_I2c_ClearFlag(I2Cx, I2C_ISR_STOPF);

    }

    if (STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_NACKF) == SET) {
        /* Clear NACK Flag */
        STM32F7_I2c_ClearFlag(I2Cx, I2C_ISR_NACKF);
    }

    if (STM32F7_I2c_GetFlag(I2Cx, I2C_ISR_TC) == SET)  // all received or all sent
    {
        if (transaction->repeatedStart) { // start next unit // start next unit
            state->currentI2cTransactionAction = &state->readI2cTransactionAction;

            STM32F7_I2c_StartTransaction(controllerIndex); // Send restart conditon
        }
        else {
            STM32F7_I2c_StopTransaction(controllerIndex);
        }
    }
}

void STM32F7_I2C1_EV_Interrupt(void* param) {
    STM32F7_I2C_EV_Interrupt(0);
}

void STM32F7_I2C2_EV_Interrupt(void* param) {
    STM32F7_I2C_EV_Interrupt(1);
}
void STM32F7_I2c_StartTransaction(int32_t controllerIndex) {
    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    I2cTransaction *transaction = state->currentI2cTransactionAction;

    uint32_t ccr = state->i2cConfiguration.clockRate + (state->i2cConfiguration.clockRate2 << 8);

    uint32_t transferMode = I2C_SOFTEND_MODE;
    uint16_t deviceAddress = state->i2cConfiguration.address;
    uint8_t bytesToTransfer = transaction->bytesToTransfer;
    if (bytesToTransfer > I2C_MAX_TRANSFER) {
        transferMode = I2C_CR2_RELOAD;
        bytesToTransfer = I2C_MAX_TRANSFER;

    }
    /*Disable before set timing*/
    STM32F7_I2c_Disable(I2Cx);

    I2Cx->TIMINGR = (0xA0000000) | (ccr);

    /* Enable the selected I2C peripheral */
    STM32F7_I2c_Enable(I2Cx);

    if (transaction->isReadTransaction) {
        STM32F7_I2c_InternalTransferConfig(controllerIndex, deviceAddress, bytesToTransfer, transferMode, I2C_GENERATE_START_READ);
        STM32F7_I2c_InterruptEnable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_RXIE);
    }
    else {
        STM32F7_I2c_InternalTransferConfig(controllerIndex, deviceAddress, bytesToTransfer, transferMode, I2C_GENERATE_START_WRITE);
        STM32F7_I2c_InterruptEnable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE);
    }
}

void STM32F7_I2c_StopTransaction(int32_t controllerIndex) {
    auto& I2Cx = i2cPorts[controllerIndex];

    auto state = &i2cStates[controllerIndex];

    I2Cx->CR2 |= I2C_CR2_STOP;  // send stop
    STM32F7_I2c_InterruptDisable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE | I2C_CR1_RXIE); // disable interrupts

    state->currentI2cTransactionAction->isDone = true;
}

TinyCLR_Result STM32F7_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error) {
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

    STM32F7_I2c_StartTransaction(controllerIndex);

    while (state->currentI2cTransactionAction->isDone == false && timeout > 0) {
        STM32F7_Time_Delay(nullptr, 1000);

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

TinyCLR_Result STM32F7_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, const TinyCLR_I2c_Settings* settings) {
    uint32_t slaveAddress = settings->SlaveAddress;
    TinyCLR_I2c_AddressFormat addressFormat = settings->AddressFormat;
    TinyCLR_I2c_BusSpeed busSpeed = settings->BusSpeed;
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

    uint32_t clk_num;
    uint32_t period_ns = (1000000 / rateKhz);
    uint32_t clk_i2c_ns = ((1000000 / 54000)) *(0xA + 1); // Did = 0xA

    while ((1000000 / period_ns) > rateKhz) {
        period_ns++;
    }

    clk_num = (period_ns / clk_i2c_ns);
    clk_num -= 2; // There are 2 lock delay as default;
    clk_num = clk_num / 2;

    if (clk_num > I2C_MAX_TRANSFER)
        clk_num = I2C_MAX_TRANSFER;

    state->i2cConfiguration.clockRate = (uint8_t)(clk_num); // low byte
    state->i2cConfiguration.clockRate2 = (uint8_t)(clk_num); // high byte
    state->i2cConfiguration.address = slaveAddress;


    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_I2c_Acquire(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        auto& I2Cx = i2cPorts[controllerIndex];

        if (!STM32F7_GpioInternal_OpenMultiPins(i2cPins[controllerIndex], 2))
            return TinyCLR_Result::SharingViolation;

        STM32F7_GpioInternal_ConfigurePin(i2cPins[controllerIndex][I2C_SCL_PIN].number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::OpenDrain, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, i2cPins[controllerIndex][I2C_SCL_PIN].alternateFunction);
        STM32F7_GpioInternal_ConfigurePin(i2cPins[controllerIndex][I2C_SDA_PIN].number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::OpenDrain, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, i2cPins[controllerIndex][I2C_SDA_PIN].alternateFunction);

        switch (controllerIndex) {
        case 0:
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // enable I2C clock
            RCC->APB1RSTR = RCC_APB1RSTR_I2C1RST; // reset I2C peripheral
            STM32F7_InterruptInternal_Activate(I2C1_EV_IRQn, (uint32_t*)&STM32F7_I2C1_EV_Interrupt, 0);
            STM32F7_InterruptInternal_Activate(I2C1_ER_IRQn, (uint32_t*)&STM32F7_I2C1_ER_Interrupt, 0);
            break;

        case 1:
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // enable I2C clock
            RCC->APB1RSTR = RCC_APB1RSTR_I2C2RST; // reset I2C peripheral
            STM32F7_InterruptInternal_Activate(I2C2_EV_IRQn, (uint32_t*)&STM32F7_I2C2_EV_Interrupt, 0);
            STM32F7_InterruptInternal_Activate(I2C2_ER_IRQn, (uint32_t*)&STM32F7_I2C2_ER_Interrupt, 0);
            break;
        }

        RCC->APB1RSTR = 0;
    }

    state->initializeCount++;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_I2c_Release(const TinyCLR_I2c_Controller* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto state = reinterpret_cast<I2cState*>(self->ApiInfo->State);

    if (state->initializeCount == 0) return TinyCLR_Result::InvalidOperation;

    state->initializeCount--;

    if (state->initializeCount == 0) {
        auto controllerIndex = state->controllerIndex;

        auto& I2Cx = i2cPorts[controllerIndex];

        STM32F7_I2c_InterruptDisable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE | I2C_CR1_RXIE); // disable interrupts
        switch (controllerIndex) {
        case 0:


            STM32F7_InterruptInternal_Deactivate(I2C1_EV_IRQn);
            STM32F7_InterruptInternal_Deactivate(I2C1_ER_IRQn);

            STM32F7_I2c_Disable(I2Cx);
            RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN; // disable I2C clock
            break;

        case 1:
            STM32F7_InterruptInternal_Deactivate(I2C2_EV_IRQn);
            STM32F7_InterruptInternal_Deactivate(I2C2_ER_IRQn);

            STM32F7_I2c_Disable(I2Cx);
            RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN; // disable I2C clock
            break;
        }



        STM32F7_GpioInternal_ClosePin(i2cPins[controllerIndex][I2C_SDA_PIN].number);
        STM32F7_GpioInternal_ClosePin(i2cPins[controllerIndex][I2C_SCL_PIN].number);
    }

    return TinyCLR_Result::Success;
}

void STM32F7_I2c_Reset() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {

        STM32F7_I2c_Release(&i2cControllers[i]);

        auto state = &i2cStates[i];

        state->i2cConfiguration.address = 0;
        state->i2cConfiguration.clockRate = 0;
        state->i2cConfiguration.clockRate2 = 0;

        state->readI2cTransactionAction.bytesToTransfer = 0;
        state->readI2cTransactionAction.bytesTransferred = 0;

        state->writeI2cTransactionAction.bytesToTransfer = 0;
        state->writeI2cTransactionAction.bytesTransferred = 0;

        state->initializeCount = 0;
    }
}
