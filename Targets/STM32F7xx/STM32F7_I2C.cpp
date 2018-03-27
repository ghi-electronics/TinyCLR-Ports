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

void STM32F7_I2c_StartTransaction(int32_t port_id);
void STM32F7_I2c_StopTransaction(int32_t port_id);

static const STM32F7_Gpio_Pin g_STM32F7_I2c_Scl_Pins[] = STM32F7_I2C_SCL_PINS;
static const STM32F7_Gpio_Pin g_STM32F7_I2c_Sda_Pins[] = STM32F7_I2C_SDA_PINS;

static const int TOTAL_I2C_CONTROLLERS = SIZEOF_ARRAY(g_STM32F7_I2c_Scl_Pins);

static I2C_TypeDef* g_STM32_I2c_Port[TOTAL_I2C_CONTROLLERS];

struct STM32F7_I2c_Configuration {

    int32_t     address;
    uint8_t     clockRate;
    uint8_t     clockRate2;

    bool        isOpened;
};
struct STM32F7_I2c_Transaction {
    bool                        isReadTransaction;
    bool                        repeatedStart;
    bool                        isDone;

    uint8_t                     *buffer;

    size_t                      bytesToTransfer;
    size_t                      bytesTransferred;

    TinyCLR_I2c_TransferStatus  result;
};

static STM32F7_I2c_Configuration g_I2cConfiguration[TOTAL_I2C_CONTROLLERS];
static STM32F7_I2c_Transaction   *g_currentI2cTransactionAction[TOTAL_I2C_CONTROLLERS];
static STM32F7_I2c_Transaction   g_ReadI2cTransactionAction[TOTAL_I2C_CONTROLLERS];
static STM32F7_I2c_Transaction   g_WriteI2cTransactionAction[TOTAL_I2C_CONTROLLERS];

static uint8_t i2cProviderDefs[TOTAL_I2C_CONTROLLERS * sizeof(TinyCLR_I2c_Provider)];
static TinyCLR_I2c_Provider* i2cProvider[TOTAL_I2C_CONTROLLERS];
static TinyCLR_Api_Info i2cApi;

const TinyCLR_Api_Info* STM32F7_I2c_GetApi() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        i2cProvider[i] = (TinyCLR_I2c_Provider*)(i2cProviderDefs + (i * sizeof(TinyCLR_I2c_Provider)));
        i2cProvider[i]->Parent = &i2cApi;
        i2cProvider[i]->Index = i;
        i2cProvider[i]->Acquire = &STM32F7_I2c_Acquire;
        i2cProvider[i]->Release = &STM32F7_I2c_Release;
        i2cProvider[i]->SetActiveSettings = &STM32F7_I2c_SetActiveSettings;
        i2cProvider[i]->Read = &STM32F7_I2c_Read;
        i2cProvider[i]->Write = &STM32F7_I2c_Write;
        i2cProvider[i]->WriteRead = &STM32F7_I2c_WriteRead;
    }

    i2cApi.Author = "GHI Electronics, LLC";
    i2cApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.I2cProvider";
    i2cApi.Type = TinyCLR_Api_Type::I2cProvider;
    i2cApi.Version = 0;
    i2cApi.Count = TOTAL_I2C_CONTROLLERS;
    i2cApi.Implementation = (i2cApi.Count > 1) ? i2cProvider : (TinyCLR_I2c_Provider**)&i2cProviderDefs;

    if (TOTAL_I2C_CONTROLLERS > 0)
        g_STM32_I2c_Port[0] = I2C1;

    if (TOTAL_I2C_CONTROLLERS > 1)
        g_STM32_I2c_Port[1] = I2C2;

    return &i2cApi;
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

void STM32F7_I2c_InternalTransferConfig(int32_t port_id, uint16_t deviceAddress, uint8_t bytesToTransfer, uint32_t transferMode, uint32_t request) {
    uint32_t tmpreg = 0;

    deviceAddress = deviceAddress << 1;
    if (request == I2C_GENERATE_START_READ) {
        deviceAddress += 1;
    }

    auto& I2Cx = g_STM32_I2c_Port[port_id];

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

void STM32F7_I2C_ER_Interrupt(int32_t port_id) {// Error Interrupt Handler
    INTERRUPT_STARTED_SCOPED(isr);

    auto& I2Cx = g_STM32_I2c_Port[port_id];
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

    if (g_currentI2cTransactionAction != nullptr)
        g_currentI2cTransactionAction[port_id]->result = TinyCLR_I2c_TransferStatus::SlaveAddressNotAcknowledged;

    STM32F7_I2c_StopTransaction(port_id);
}

void STM32F7_I2C1_ER_Interrupt(void *param) {
    STM32F7_I2C_ER_Interrupt(0);
}

void STM32F7_I2C2_ER_Interrupt(void *param) {
    STM32F7_I2C_ER_Interrupt(1);
}

void STM32F7_I2C_EV_Interrupt(int32_t port_id) {// Event Interrupt Handler
    INTERRUPT_STARTED_SCOPED(isr);

    auto& I2Cx = g_STM32_I2c_Port[port_id];

    STM32F7_I2c_Transaction *transaction = g_currentI2cTransactionAction[port_id];

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
                STM32F7_I2c_InternalTransferConfig(port_id, g_I2cConfiguration[port_id].address, I2C_MAX_TRANSFER, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            }
            else {
                STM32F7_I2c_InternalTransferConfig(port_id, g_I2cConfiguration[port_id].address, todo, 0, I2C_NO_STARTSTOP);
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
            g_currentI2cTransactionAction[port_id] = &g_ReadI2cTransactionAction[port_id];

            STM32F7_I2c_StartTransaction(port_id); // Send restart conditon
        }
        else {
            STM32F7_I2c_StopTransaction(port_id);
        }
    }
}

void STM32F7_I2C1_EV_Interrupt(void* param) {
    STM32F7_I2C_EV_Interrupt(0);
}

void STM32F7_I2C2_EV_Interrupt(void* param) {
    STM32F7_I2C_EV_Interrupt(1);
}
void STM32F7_I2c_StartTransaction(int32_t port_id) {
    auto& I2Cx = g_STM32_I2c_Port[port_id];

    STM32F7_I2c_Transaction *transaction = g_currentI2cTransactionAction[port_id];

    uint32_t ccr = g_I2cConfiguration[port_id].clockRate + (g_I2cConfiguration[port_id].clockRate2 << 8);

    uint32_t transferMode = I2C_SOFTEND_MODE;
    uint16_t deviceAddress = g_I2cConfiguration[port_id].address;
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
        STM32F7_I2c_InternalTransferConfig(port_id, deviceAddress, bytesToTransfer, transferMode, I2C_GENERATE_START_READ);
        STM32F7_I2c_InterruptEnable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_RXIE);
    }
    else {
        STM32F7_I2c_InternalTransferConfig(port_id, deviceAddress, bytesToTransfer, transferMode, I2C_GENERATE_START_WRITE);
        STM32F7_I2c_InterruptEnable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE);
    }
}

void STM32F7_I2c_StopTransaction(int32_t port_id) {
    auto& I2Cx = g_STM32_I2c_Port[port_id];

    I2Cx->CR2 |= I2C_CR2_STOP;  // send stop
    STM32F7_I2c_InterruptDisable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE | I2C_CR1_RXIE); // disable interrupts

    g_currentI2cTransactionAction[port_id]->isDone = true;
}

TinyCLR_Result STM32F7_I2c_Read(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    int32_t port_id = self->Index;

    g_ReadI2cTransactionAction[port_id].isReadTransaction = true;
    g_ReadI2cTransactionAction[port_id].buffer = buffer;
    g_ReadI2cTransactionAction[port_id].bytesToTransfer = length;
    g_ReadI2cTransactionAction[port_id].isDone = false;
    g_ReadI2cTransactionAction[port_id].repeatedStart = false;
    g_ReadI2cTransactionAction[port_id].bytesTransferred = 0;

    g_currentI2cTransactionAction[port_id] = &g_ReadI2cTransactionAction[port_id];

    STM32F7_I2c_StartTransaction(port_id);

    while (g_currentI2cTransactionAction[port_id]->isDone == false && timeout > 0) {
        STM32F7_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (g_currentI2cTransactionAction[port_id]->bytesTransferred == length)
        result = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (g_currentI2cTransactionAction[port_id]->bytesTransferred < length && g_currentI2cTransactionAction[port_id]->bytesTransferred > 0)
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = g_currentI2cTransactionAction[port_id]->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F7_I2c_Write(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    int32_t port_id = self->Index;

    g_WriteI2cTransactionAction[port_id].isReadTransaction = false;
    g_WriteI2cTransactionAction[port_id].buffer = (uint8_t*)buffer;
    g_WriteI2cTransactionAction[port_id].bytesToTransfer = length;
    g_WriteI2cTransactionAction[port_id].isDone = false;
    g_WriteI2cTransactionAction[port_id].repeatedStart = false;
    g_WriteI2cTransactionAction[port_id].bytesTransferred = 0;

    g_currentI2cTransactionAction[port_id] = &g_WriteI2cTransactionAction[port_id];

    STM32F7_I2c_StartTransaction(port_id);

    while (g_currentI2cTransactionAction[port_id]->isDone == false && timeout > 0) {
        STM32F7_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (g_currentI2cTransactionAction[port_id]->bytesTransferred == length)
        result = TinyCLR_I2c_TransferStatus::FullTransfer;
    else if (g_currentI2cTransactionAction[port_id]->bytesTransferred < length && g_currentI2cTransactionAction[port_id]->bytesTransferred > 0)
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;

    length = g_currentI2cTransactionAction[port_id]->bytesTransferred;

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F7_I2c_WriteRead(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result) {
    int32_t timeout = I2C_TRANSACTION_TIMEOUT;

    int32_t port_id = self->Index;

    g_WriteI2cTransactionAction[port_id].isReadTransaction = false;
    g_WriteI2cTransactionAction[port_id].buffer = (uint8_t*)writeBuffer;
    g_WriteI2cTransactionAction[port_id].bytesToTransfer = writeLength;
    g_WriteI2cTransactionAction[port_id].isDone = false;
    g_WriteI2cTransactionAction[port_id].repeatedStart = true;
    g_WriteI2cTransactionAction[port_id].bytesTransferred = 0;

    g_ReadI2cTransactionAction[port_id].isReadTransaction = true;
    g_ReadI2cTransactionAction[port_id].buffer = readBuffer;
    g_ReadI2cTransactionAction[port_id].bytesToTransfer = readLength;
    g_ReadI2cTransactionAction[port_id].isDone = false;
    g_ReadI2cTransactionAction[port_id].repeatedStart = false;
    g_ReadI2cTransactionAction[port_id].bytesTransferred = 0;

    g_currentI2cTransactionAction[port_id] = &g_WriteI2cTransactionAction[port_id];

    STM32F7_I2c_StartTransaction(port_id);

    while (g_currentI2cTransactionAction[port_id]->isDone == false && timeout > 0) {
        STM32F7_Time_Delay(nullptr, 1000);

        timeout--;
    }

    if (g_WriteI2cTransactionAction[port_id].bytesTransferred != writeLength) {
        writeLength = g_WriteI2cTransactionAction[port_id].bytesTransferred;
        result = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }
    else {
        readLength = g_ReadI2cTransactionAction[port_id].bytesTransferred;

        if (g_currentI2cTransactionAction[port_id]->bytesTransferred == readLength)
            result = TinyCLR_I2c_TransferStatus::FullTransfer;
        else if (g_currentI2cTransactionAction[port_id]->bytesTransferred < readLength && g_currentI2cTransactionAction[port_id]->bytesTransferred > 0)
            result = TinyCLR_I2c_TransferStatus::PartialTransfer;
    }

    return timeout > 0 ? TinyCLR_Result::Success : TinyCLR_Result::TimedOut;
}

TinyCLR_Result STM32F7_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed) {
    uint32_t rateKhz;
    uint32_t ccr;

    int32_t port_id = self->Index;

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

    g_I2cConfiguration[port_id].clockRate = (uint8_t)(clk_num); // low byte
    g_I2cConfiguration[port_id].clockRate2 = (uint8_t)(clk_num); // high byte
    g_I2cConfiguration[port_id].address = slaveAddress;


    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_I2c_Acquire(const TinyCLR_I2c_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    int32_t port_id = self->Index;

    auto& I2Cx = g_STM32_I2c_Port[port_id];
    auto& scl = g_STM32F7_I2c_Scl_Pins[port_id];
    auto& sda = g_STM32F7_I2c_Sda_Pins[port_id];

    if (!STM32F7_GpioInternal_OpenPin(sda.number) || !STM32F7_GpioInternal_OpenPin(scl.number))
        return TinyCLR_Result::SharingViolation;

    STM32F7_GpioInternal_ConfigurePin(sda.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::OpenDrain, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, sda.alternateFunction);
    STM32F7_GpioInternal_ConfigurePin(scl.number, STM32F7_Gpio_PortMode::AlternateFunction, STM32F7_Gpio_OutputType::OpenDrain, STM32F7_Gpio_OutputSpeed::VeryHigh, STM32F7_Gpio_PullDirection::PullUp, scl.alternateFunction);

    switch (port_id) {
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

    g_I2cConfiguration[port_id].isOpened = true;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_I2c_Release(const TinyCLR_I2c_Provider* self) {
    if (self == nullptr)
        return TinyCLR_Result::ArgumentNull;

    auto& I2Cx = g_STM32_I2c_Port[self->Index];
    auto& scl = g_STM32F7_I2c_Scl_Pins[self->Index];
    auto& sda = g_STM32F7_I2c_Sda_Pins[self->Index];

    STM32F7_I2c_InterruptDisable(I2Cx, I2C_CR1_ERRIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_TXIE | I2C_CR1_RXIE); // disable interrupts
    switch (self->Index) {
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

    // TODO other ports
        break;
    }

    STM32F7_GpioInternal_ClosePin(sda.number);
    STM32F7_GpioInternal_ClosePin(scl.number);

    g_I2cConfiguration[self->Index].isOpened = false;

    return TinyCLR_Result::Success;
}

void STM32F7_I2c_Reset() {
    for (auto i = 0; i < TOTAL_I2C_CONTROLLERS; i++) {
        if (g_I2cConfiguration[i].isOpened)
            STM32F7_I2c_Release(i2cProvider[i]);

        g_I2cConfiguration[i].address = 0;
        g_I2cConfiguration[i].clockRate = 0;
        g_I2cConfiguration[i].clockRate2 = 0;

        g_ReadI2cTransactionAction[i].bytesToTransfer = 0;
        g_ReadI2cTransactionAction[i].bytesTransferred = 0;

        g_WriteI2cTransactionAction[i].bytesToTransfer = 0;
        g_WriteI2cTransactionAction[i].bytesTransferred = 0;

        g_I2cConfiguration[i].isOpened = false;
    }
}
