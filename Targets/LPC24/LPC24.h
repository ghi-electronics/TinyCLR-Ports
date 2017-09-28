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

#pragma once

#include <stdio.h>
#include <string.h>

#include <TinyCLR.h>
#include <Device.h>

#define SIZEOF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
#define CHARIZE2(c) #c
#define CHARIZE(c) (CHARIZE2(c)[0])

#define PCONP_OFFSET 0xC4
#define PCONP_PCTIM0_MASK 0x2
#define PCONP_PCTIM0 0x2
#define PCONP_PCTIM0_BIT 1
#define PCONP_PCTIM1_MASK 0x4
#define PCONP_PCTIM1 0x4
#define PCONP_PCTIM1_BIT 2
#define PCONP_PCUART0_MASK 0x8
#define PCONP_PCUART0 0x8
#define PCONP_PCUART0_BIT 3
#define PCONP_PCUART1_MASK 0x10
#define PCONP_PCUART1 0x10
#define PCONP_PCUART1_BIT 4
#define PCONP_PCPWM0_MASK 0x20
#define PCONP_PCPWM0 0x20
#define PCONP_PCPWM0_BIT 5
#define PCONP_PCPWM1_MASK 0x40
#define PCONP_PCPWM1 0x40
#define PCONP_PCPWM1_BIT 6
#define PCONP_PCI2C0_MASK 0x80
#define PCONP_PCI2C0 0x80
#define PCONP_PCI2C0_BIT 7
#define PCONP_PCSPI_MASK 0x100
#define PCONP_PCSPI 0x100
#define PCONP_PCSPI_BIT 8
#define PCONP_PCRTC_MASK 0x200
#define PCONP_PCRTC 0x200
#define PCONP_PCRTC_BIT 9
#define PCONP_PCSSP1_MASK 0x400
#define PCONP_PCSSP1 0x400
#define PCONP_PCSSP1_BIT 10
#define PCONP_PCEMC_MASK 0x800
#define PCONP_PCEMC 0x800
#define PCONP_PCEMC_BIT 11
#define PCONP_PCAD_MASK 0x1000
#define PCONP_PCAD 0x1000
#define PCONP_PCAD_BIT 12
#define PCONP_PCAN1_MASK 0x2000
#define PCONP_PCAN1 0x2000
#define PCONP_PCAN1_BIT 13
#define PCONP_PCAN2_MASK 0x4000
#define PCONP_PCAN2 0x4000
#define PCONP_PCAN2_BIT 14
#define PCONP_PCI2C1_MASK 0x80000
#define PCONP_PCI2C1 0x80000
#define PCONP_PCI2C1_BIT 19
#define PCONP_PCSSP0_MASK 0x200000
#define PCONP_PCSSP0 0x200000
#define PCONP_PCSSP0_BIT 21
#define PCONP_PCTIM2_MASK 0x400000
#define PCONP_PCTIM2 0x400000
#define PCONP_PCTIM2_BIT 22
#define PCONP_PCTIM3_MASK 0x800000
#define PCONP_PCTIM3 0x800000
#define PCONP_PCTIM3_BIT 23
#define PCONP_PCUART2_MASK 0x1000000
#define PCONP_PCUART2 0x1000000
#define PCONP_PCUART2_BIT 24
#define PCONP_PCUART3_MASK 0x2000000
#define PCONP_PCUART3 0x2000000
#define PCONP_PCUART3_BIT 25
#define PCONP_PCI2C2_MASK 0x4000000
#define PCONP_PCI2C2 0x4000000
#define PCONP_PCI2C2_BIT 26
#define PCONP_PCI2CS_MASK 0x8000000
#define PCONP_PCI2CS 0x8000000
#define PCONP_PCI2CS_BIT 27
#define PCONP_PCSDC_MASK 0x10000000
#define PCONP_PCSDC 0x10000000
#define PCONP_PCSDC_BIT 28
#define PCONP_PCGPDMA_MASK 0x20000000
#define PCONP_PCGPDMA 0x20000000
#define PCONP_PCGPDMA_BIT 29
#define PCONP_PCENET_MASK 0x40000000
#define PCONP_PCENET 0x40000000
#define PCONP_PCENET_BIT 30
#define PCONP_PUSB_MASK 0x80000000
#define PCONP_PUSB 0x80000000
#define PCONP_PUSB_BIT 31

// GPIO
enum class LPC24_Gpio_Direction : uint8_t {
    Input = 0,
    Output = 1,
};

enum class LPC24_Gpio_PinFunction : uint8_t {
    PinFunction0 = 0,
    PinFunction1 = 1,
    PinFunction2 = 2,
    PinFunction3 = 3,
};

enum class LPC24_Gpio_PinMode : uint8_t {
    PullUp = 0,
    Reserved = 1,
    Inactive = 2,
    PullDown = 3
};

struct LPC24_Gpio_Pin {
    uint32_t number;
    LPC24_Gpio_PinFunction pinFunction;
};

struct LPC24_Gpio_PinConfiguration {
    LPC24_Gpio_Direction pinDirection;
    LPC24_Gpio_PinMode pinMode;
    LPC24_Gpio_PinFunction pinFunction;
    bool apply;
};

#define PIN(port, pin) (port * 32 + pin)
#define PIN_NONE 0xFFFFFFFF
#define PF(num) (CONCAT(LPC24_Gpio_PinFunction::PinFunction, num))
#define PF_NONE LPC24_Gpio_PinFunction::PinFunction0

#define INIT(direction, pinMode, apply) { LPC24_Gpio_Direction::direction, LPC24_Gpio_PinMode::pinMode, LPC24_Gpio_PinFunction::PinFunction0, apply }
#define ALTFUN(direction, pinMode, pinFunction) { LPC24_Gpio_Direction::direction, LPC24_Gpio_PinMode::pinMode, LPC24_Gpio_PinFunction::pinFunction, true }
#define INPUT(pinMode) { LPC24_Gpio_Direction::Input, LPC24_Gpio_PinMode::pinMode, LPC24_Gpio_PinFunction::PinFunction0, true }
#define DEFAULT() { LPC24_Gpio_Direction::Input, LPC24_Gpio_PinMode::Inactive, LPC24_Gpio_PinFunction::PinFunction0, false }

void LPC24_Gpio_Reset();
const TinyCLR_Api_Info* LPC24_Gpio_GetApi();
TinyCLR_Result LPC24_Gpio_Acquire(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result LPC24_Gpio_Release(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result LPC24_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t alternate);
TinyCLR_Result LPC24_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result LPC24_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result LPC24_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, int32_t debounceTime);
TinyCLR_Result LPC24_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result LPC24_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
bool LPC24_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode LPC24_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t LPC24_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t LPC24_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result LPC24_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR);
TinyCLR_Result LPC24_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
void LPC24_Gpio_EnableOutputPin(int32_t pin, bool initialState);
void LPC24_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor);
bool LPC24_Gpio_OpenPin(int32_t pin);
bool LPC24_Gpio_ClosePin(int32_t pin);
bool LPC24_Gpio_ReadPin(int32_t pin);
void LPC24_Gpio_WritePin(int32_t pin, bool value);
bool LPC24_Gpio_ConfigurePin(int32_t pin, LPC24_Gpio_Direction pinDir, LPC24_Gpio_PinFunction alternateFunction, LPC24_Gpio_PinMode pullResistor);


// ADC
const TinyCLR_Api_Info* LPC24_Adc_GetApi();
void LPC24_Adc_Reset();
int32_t LPC24_Adc_GetControllerCount();
int32_t LPC24_Adc_GetPin(int32_t channel);
LPC24_Gpio_PinFunction LPC24_Adc_GetPinFunction(int32_t channel);
TinyCLR_Result LPC24_Adc_Acquire(const TinyCLR_Adc_Provider* self);
TinyCLR_Result LPC24_Adc_Release(const TinyCLR_Adc_Provider* self);
TinyCLR_Result LPC24_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result LPC24_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result LPC24_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value);
int32_t LPC24_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self);
int32_t LPC24_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self);
int32_t LPC24_Adc_GetMinValue(const TinyCLR_Adc_Provider* self);
int32_t LPC24_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self);
TinyCLR_Adc_ChannelMode LPC24_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self);
TinyCLR_Result LPC24_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);
bool LPC24_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);

//DAC
const TinyCLR_Api_Info* LPC24_Dac_GetApi();
void LPC24_Dac_Reset();
TinyCLR_Result LPC24_Dac_Acquire(const TinyCLR_Dac_Provider* self);
TinyCLR_Result LPC24_Dac_Release(const TinyCLR_Dac_Provider* self);
TinyCLR_Result LPC24_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result LPC24_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result LPC24_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value);
int32_t LPC24_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self);
int32_t LPC24_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self);
int32_t LPC24_Dac_GetMinValue(const TinyCLR_Dac_Provider* self);
int32_t LPC24_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self);

// PWM
struct PwmController {
    int32_t                     channel[MAX_PWM_PER_CONTROLLER];
    int32_t                     match[MAX_PWM_PER_CONTROLLER];
    LPC24_Gpio_Pin              gpioPin[MAX_PWM_PER_CONTROLLER];
    uint32_t                    outputEnabled[MAX_PWM_PER_CONTROLLER];
    uint32_t                    *matchAddress[MAX_PWM_PER_CONTROLLER];
    bool                        invert[MAX_PWM_PER_CONTROLLER];
    double                      frequency;
    double                      dutyCycle[MAX_PWM_PER_CONTROLLER];
};
const TinyCLR_Api_Info* LPC24_Pwm_GetApi();
void LPC24_Pwm_Reset();
void LPC24_Pwm_ResetController(int32_t controller);
LPC24_Gpio_Pin LPC24_Pwm_GetPins(int32_t controller, int32_t channel);
TinyCLR_Result LPC24_Pwm_Acquire(const TinyCLR_Pwm_Provider* self);
TinyCLR_Result LPC24_Pwm_Release(const TinyCLR_Pwm_Provider* self);
int32_t LPC24_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC24_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency);
TinyCLR_Result LPC24_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC24_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC24_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC24_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC24_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity);
double LPC24_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self);
double LPC24_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self);
double LPC24_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self);
int32_t LPC24_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self);

//SPI
const TinyCLR_Api_Info* LPC24_Spi_GetApi();
void LPC24_Spi_Reset();
bool LPC24_Spi_Transaction_Start(int32_t controller);
bool LPC24_Spi_Transaction_Stop(int32_t controller);
bool LPC24_Spi_Transaction_nWrite8_nRead8(int32_t controller);
TinyCLR_Result LPC24_Spi_Acquire(const TinyCLR_Spi_Provider* self);
TinyCLR_Result LPC24_Spi_Release(const TinyCLR_Spi_Provider* self);
TinyCLR_Result LPC24_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode);
TinyCLR_Result LPC24_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
TinyCLR_Result LPC24_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
int32_t LPC24_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self);
int32_t LPC24_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self);
int32_t LPC24_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self);
TinyCLR_Result LPC24_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
const TinyCLR_Api_Info* LPC24_Uart_GetApi();
void LPC24_Uart_Reset();
int32_t LPC24_Uart_GetTxPin(int32_t portNum);
int32_t LPC24_Uart_GetRxPin(int32_t portNum);
int32_t LPC24_Uart_GetRtsPin(int32_t portNum);
int32_t LPC24_Uart_GetCtsPin(int32_t portNum);
LPC24_Gpio_PinFunction LPC24_Uart_GetTxAlternateFunction(int32_t portNum);
LPC24_Gpio_PinFunction LPC24_Uart_GetRxAlternateFunction(int32_t portNum);
LPC24_Gpio_PinFunction LPC24_Uart_GetRtsAlternateFunction(int32_t portNum);
LPC24_Gpio_PinFunction LPC24_Uart_GetCtsAlternateFunction(int32_t portNum);
bool LPC24_Uart_TxHandshakeEnabledState(int portNum);
void LPC24_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable);
void LPC24_Uart_RxBufferFullInterruptEnable(int portNum, bool enable);
TinyCLR_Result LPC24_Uart_Acquire(const TinyCLR_Uart_Provider* self);
TinyCLR_Result LPC24_Uart_Release(const TinyCLR_Uart_Provider* self);
TinyCLR_Result LPC24_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking);
TinyCLR_Result LPC24_Uart_Flush(const TinyCLR_Uart_Provider* self);
TinyCLR_Result LPC24_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler);
TinyCLR_Result LPC24_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result LPC24_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result LPC24_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC24_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result LPC24_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC24_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC24_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC24_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC24_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result LPC24_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC24_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state);

//Deployment
const TinyCLR_Api_Info* LPC24_Deployment_GetApi();
TinyCLR_Result LPC24_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool &supportXIP);
TinyCLR_Result LPC24_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result LPC24_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result LPC24_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result LPC24_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result LPC24_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased);
TinyCLR_Result LPC24_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);
TinyCLR_Result LPC24_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
bool LPC24_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer);
bool LPC24_Flash_IsSupportsXIP(const TinyCLR_Deployment_Provider* self);
uint32_t LPC24_Flash_GetPartId();

// Interrupt
class LPC24_SmartPtr_IRQ {

    uint32_t m_state;

    void Disable();
    void Restore();

public:
    LPC24_SmartPtr_IRQ();
    ~LPC24_SmartPtr_IRQ();

    bool WasDisabled();
    void Acquire();
    void Release();
    void Probe();

    static uint32_t GetState();
};

class LPC24_SmartPtr_Interrupt {
public:
    LPC24_SmartPtr_Interrupt();
    ~LPC24_SmartPtr_Interrupt();
};

#define DISABLE_INTERRUPTS_SCOPED(name) LPC24_SmartPtr_IRQ name
#define INTERRUPT_STARTED_SCOPED(name) LPC24_SmartPtr_Interrupt name

const TinyCLR_Api_Info* LPC24_Interrupt_GetApi();
TinyCLR_Result LPC24_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result LPC24_Interrupt_Release();
bool LPC24_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param);
bool LPC24_Interrupt_Deactivate(uint32_t Irq_Index);
bool LPC24_Interrupt_Enable(uint32_t Irq_Index);
bool LPC24_Interrupt_Disable(uint32_t Irq_Index);
bool LPC24_Interrupt_EnableState(uint32_t Irq_Index);
bool LPC24_Interrupt_InterruptState(uint32_t Irq_Index);


bool LPC24_Interrupt_GlobalIsDisabled();
bool LPC24_Interrupt_GlobalEnable(bool force);
bool LPC24_Interrupt_GlobalDisable(bool force);

void LPC24_Interrupt_GlobalRestore();
void LPC24_Interrupt_GlobalWaitForInterrupt();

extern TinyCLR_Interrupt_StartStopHandler LPC24_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler LPC24_Interrupt_Ended;

// I2C
const TinyCLR_Api_Info* LPC24_I2c_GetApi();
void LPC24_I2c_Reset();
TinyCLR_Result LPC24_I2c_Acquire(const TinyCLR_I2c_Provider* self);
TinyCLR_Result LPC24_I2c_Release(const TinyCLR_I2c_Provider* self);
TinyCLR_Result LPC24_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result LPC24_I2c_ReadTransaction(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result LPC24_I2c_WriteTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result LPC24_I2c_WriteReadTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result);
void LPC24_I2c_StartTransaction();
void LPC24_I2c_StopTransaction();

// Time
const TinyCLR_Api_Info* LPC24_Time_GetApi();
TinyCLR_Result LPC24_Time_Acquire(const TinyCLR_Time_Provider* self);
TinyCLR_Result LPC24_Time_Release(const TinyCLR_Time_Provider* self);
TinyCLR_Result LPC24_Time_GetInitialTime(const TinyCLR_Time_Provider* self, int64_t& utcTime, int32_t& timeZoneOffsetMinutes);
uint64_t LPC24_Time_GetTimeForProcessorTicks(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t LPC24_Time_TimeToTicks(const TinyCLR_Time_Provider* self, uint64_t time);
uint64_t LPC24_Time_MillisecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t LPC24_Time_MicrosecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t microseconds);
uint64_t LPC24_Time_GetCurrentTicks(const TinyCLR_Time_Provider* self);
TinyCLR_Result LPC24_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks);
TinyCLR_Result LPC24_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback);
void LPC24_Time_DelayNoInterrupt(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void LPC24_Time_Delay(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void LPC24_Time_GetDriftParameters(const TinyCLR_Time_Provider* self, int32_t* a, int32_t* b, int64_t* c);

// Power
const TinyCLR_Api_Info* LPC24_Power_GetApi();
void LPC24_Power_SetHandlers(void(*stop)(), void(*restart)());
void LPC24_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_Sleep_Level level);
void LPC24_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter);
TinyCLR_Result LPC24_Power_Acquire(const TinyCLR_Power_Provider* self);
TinyCLR_Result LPC24_Power_Release(const TinyCLR_Power_Provider* self);

//UsbClient
#define USB_BASE 0xFFE0C000
#define OTGIntSt (*(volatile unsigned *)0xFFE0C100)
#define OTGIntEn (*(volatile unsigned *)0xFFE0C104)
#define OTGIntSet (*(volatile unsigned *)0xFFE0C108)
#define OTGIntClr (*(volatile unsigned *)0xFFE0C10C)
#define OTGStCtrl (*(volatile unsigned *)0xFFE0C110)
#define OTGTmr (*(volatile unsigned *)0xFFE0C114)
#define USBPortSel (*(volatile unsigned *)0xFFE0C110)
#define USBDevIntSt (*(volatile unsigned *)0xFFE0C200)
#define USBDevIntEn (*(volatile unsigned *)0xFFE0C204)
#define USBDevIntClr (*(volatile unsigned *)0xFFE0C208)
#define USBDevIntSet (*(volatile unsigned *)0xFFE0C20C)
#define USBCmdCode (*(volatile unsigned *)0xFFE0C210)
#define USBCmdData (*(volatile unsigned *)0xFFE0C214)
#define USBRxData (*(volatile unsigned *)0xFFE0C218)
#define USBTxData (*(volatile unsigned *)0xFFE0C21C)
#define USBRxPLen (*(volatile unsigned *)0xFFE0C220)
#define USBTxPLen (*(volatile unsigned *)0xFFE0C224)
#define USBCtrl (*(volatile unsigned *)0xFFE0C228)
#define USBDevIntPri (*(volatile unsigned *)0xFFE0C22C)
#define USBEpIntSt (*(volatile unsigned *)0xFFE0C230)
#define USBEpIntEn (*(volatile unsigned *)0xFFE0C234)
#define USBEpIntClr (*(volatile unsigned *)0xFFE0C238)
#define USBEpIntSet (*(volatile unsigned *)0xFFE0C23C)
#define USBEpIntPri (*(volatile unsigned *)0xFFE0C240)
#define USBReEp (*(volatile unsigned *)0xFFE0C244)
#define USBEpInd (*(volatile unsigned *)0xFFE0C248)
#define USBEpMaxPSize (*(volatile unsigned *)0xFFE0C24C)
#define USBDMARSt (*(volatile unsigned *)0xFFE0C250)
#define USBDMARClr (*(volatile unsigned *)0xFFE0C254)
#define USBDMARSet (*(volatile unsigned *)0xFFE0C258)
#define USBUDCAH (*(volatile unsigned *)0xFFE0C280)
#define USBEpDMASt (*(volatile unsigned *)0xFFE0C284)
#define USBEpDMAEn (*(volatile unsigned *)0xFFE0C288)
#define USBEpDMADis (*(volatile unsigned *)0xFFE0C28C)
#define USBDMAIntSt (*(volatile unsigned *)0xFFE0C290)
#define USBDMAIntEn (*(volatile unsigned *)0xFFE0C294)
#define USBEoTIntSt (*(volatile unsigned *)0xFFE0C2A0)
#define USBEoTIntClr (*(volatile unsigned *)0xFFE0C2A4)
#define USBEoTIntSet (*(volatile unsigned *)0xFFE0C2A8)
#define USBNDDRIntSt (*(volatile unsigned *)0xFFE0C2AC)
#define USBNDDRIntClr (*(volatile unsigned *)0xFFE0C2B0)
#define USBNDDRIntSet (*(volatile unsigned *)0xFFE0C2B4)
#define USBSysErrIntSt (*(volatile unsigned *)0xFFE0C2B8)
#define USBSysErrIntClr (*(volatile unsigned *)0xFFE0C2BC)
#define USBSysErrIntSet (*(volatile unsigned *)0xFFE0C2C0)
#define USBClkCtrl (*(volatile unsigned *)0xFFE0CFF4)
#define USBClkSt (*(volatile unsigned *)0xFFE0CFF8)
#define OTGClkCtrl (*(volatile unsigned *)0xFFE0CFF4)
#define OTGClkSt (*(volatile unsigned *)0xFFE0CFF8)

const TinyCLR_Api_Info* LPC24_UsbClient_GetApi();
void LPC24_UsbClient_Reset();
void LPC24_UsbClient_PinConfiguration();
TinyCLR_Result LPC24_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result LPC24_UsbClient_Release(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result LPC24_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t & stream, TinyCLR_UsbClient_StreamMode mode);
TinyCLR_Result LPC24_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result LPC24_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t stream, const uint8_t* data, size_t& length);
TinyCLR_Result LPC24_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t stream, uint8_t* data, size_t& length);
TinyCLR_Result LPC24_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result LPC24_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler);

TinyCLR_Result LPC24_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result LPC24_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result LPC24_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value);
TinyCLR_Result LPC24_UsbClient_SetOsExtendedPropertyHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler);

// LCD
void LPC24_Display_Reset();
const TinyCLR_Api_Info* LPC24_Display_GetApi();
TinyCLR_Result LPC24_Display_Acquire(const TinyCLR_Display_Provider* self, uint32_t width, uint32_t height);
TinyCLR_Result LPC24_Display_Release(const TinyCLR_Display_Provider* self);
TinyCLR_Result LPC24_Display_SetLcdConfiguration(const TinyCLR_Display_Provider* self, bool outputEnableIsFixed, bool outputEnablePolarity, bool pixelPolarity, uint32_t pixelClockRate, bool horizontalSyncPolarity, uint32_t horizontalSyncPulseWidth, uint32_t horizontalFrontPorch, uint32_t horizontalBackPorch, bool verticalSyncPolarity, uint32_t verticalSyncPulseWidth, uint32_t verticalFrontPorch, uint32_t verticalBackPorch);
TinyCLR_Result LPC24_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data, TinyCLR_Display_Format dataFormat);
TinyCLR_Result LPC24_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer);
int32_t LPC24_Display_GetWidth(const TinyCLR_Display_Provider* self);
int32_t LPC24_Display_GetHeight(const TinyCLR_Display_Provider* self);
TinyCLR_Display_InterfaceType LPC24_Display_GetType(const TinyCLR_Display_Provider* self);

//Startup
void LPC24_Startup_Initialize();
void LPC24_Startup_GetHeap(uint8_t*& start, size_t& length);
int32_t LPC24_Startup_GetDeviceId();
void LPC24_Startup_GetDebugger(const TinyCLR_Api_Info*& api, size_t& index);
void LPC24_Startup_GetRunApp(bool& runApp);


//////////////////////////////////////////////////////////////////////////////
// System Control Block
//
extern "C" struct LPC24XX_SYSCON {
    static const uint32_t c_SYSCON_Base = 0xE01FC000;

    // MAM Registers
    /****/ volatile uint32_t MAMCR;         // MAM Control register
    /****/ volatile uint32_t MAMTIM;        // MAM Timing register

    /****/ volatile uint32_t dummy0[14];    // Filler to align next register address

    // Memory Mapping Control
    /****/ volatile uint32_t MEMMAP;        // Memory map control register

    /****/ volatile uint32_t dummy1[15];    // Filler to align next register address

    // Phased Lock Loop
    /****/ volatile uint32_t PLLCON;        // PLL Control Register
    static const uint32_t PLLE = 0x1;       // Enable PLL
    static const uint32_t PLLC = 0x2;       // Connect PLL

    /****/ volatile uint32_t PLLCFG;        // PLL Configuration Register

    /****/ volatile uint32_t PLLSTAT;       // PLL Status Register
    static const uint32_t ENBLD = (0x1 << 24);  // PLL is enabled
    static const uint32_t CNCTD = (0x1 << 25);  // PLL is connected
    static const uint32_t LOCKD = (0x1 << 26);  // PLL is locked

    /****/ volatile uint32_t PLLFEED;       // PLL Feed Register

    /****/ volatile uint32_t dummy2[12];    // Filler to align next register address

    // Power Control
    /****/ volatile uint32_t PCON;          // Power Control Register
    /****/ volatile uint32_t PCONP;         // Power Control for Peripherals
    static const uint32_t ENABLE_ENET = 0x40000000;
    static const uint32_t ENABLE_LCD = 0x00100000;
    /****/ volatile uint32_t dummy3[15];    // Filler to align next register address

    // Clock Control
    /****/ volatile uint32_t CCLKCFG;        // CPU Clock Configuration Register
    /****/ volatile uint32_t USBCLKCFG;      // USB Clock Configuration Register

    /****/ volatile uint32_t CLKSRCSEL;      // Clock Source Select Register
    static const uint32_t IRC = 0x0;         // Internal RC Oscillator
    static const uint32_t OSC = 0x1;         // Main Oscillator
    static const uint32_t RTC = 0x2;         // RTC Oscillator

    /****/ volatile uint32_t dummy4[12];    // Filler to align next register address

    // External Interrupts
    /****/ volatile uint32_t EXTINT;        // External Interrupt Flag Register
    /****/ volatile uint32_t EXTWAKE;       // External Interrupt Wakeup Register
    /****/ volatile uint32_t EXTMODE;       // External Interrupt Mode Register
    /****/ volatile uint32_t EXTPOLAR;      // External Interrupt Polarity Register

    /****/ volatile uint32_t dummy5[12];    // Filler to align next register address

    /****/ volatile uint32_t RSID;          // Reset Source Identification Register

    /****/ volatile uint32_t dummy6[7];     // Filler to align next register address

    // System control and status
    /****/ volatile uint32_t SCS;           // System Control and Status
    static const uint32_t HS_IO = (0x1 << 0); // Enable High speed gpio on port 0 and 1
    static const uint32_t OSCEN = (0x1 << 5); // Oscillator enable
    static const uint32_t READY = (0x1 << 6); // Oscillator is ready to use


    // IRC trim
    /****/ volatile uint32_t IRCTRIM;       // IRC Trim Register
    // Peripheral clock control
    /****/ volatile uint32_t PCLKSEL0;      // Peripheral Clock Selection register 0
    /****/ volatile uint32_t PCLKSEL1;      // Peripheral Clock Selection register 1

    /****/ volatile uint32_t dummy7[2];     // Filler to align next register address
    //LCD clock Divider
    /****/ volatile uint32_t LCD_CFG;       // Filler to align next register address
};
//
// System Control Block
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Interrupt Controller
//
struct LPC24XX_VIC {
    static const uint32_t c_VIC_Base = 0xFFFFF000;

    //--//
    // Priority Levels, 0 = Highest priority
    static const uint32_t c_IRQ_Priority_0 = 0;
    static const uint32_t c_IRQ_Priority_1 = 1;
    static const uint32_t c_IRQ_Priority_2 = 2;
    static const uint32_t c_IRQ_Priority_3 = 3;
    static const uint32_t c_IRQ_Priority_4 = 4;
    static const uint32_t c_IRQ_Priority_5 = 5;
    static const uint32_t c_IRQ_Priority_6 = 6;
    static const uint32_t c_IRQ_Priority_7 = 7;
    static const uint32_t c_IRQ_Priority_8 = 8;
    static const uint32_t c_IRQ_Priority_9 = 9;
    static const uint32_t c_IRQ_Priority_10 = 10;
    static const uint32_t c_IRQ_Priority_11 = 11;
    static const uint32_t c_IRQ_Priority_12 = 12;
    static const uint32_t c_IRQ_Priority_13 = 13;
    static const uint32_t c_IRQ_Priority_14 = 14;
    static const uint32_t c_IRQ_Priority_15 = 15;

    // Interrupt slot assignments
    static const uint32_t c_IRQ_INDEX_WDT = 0;
    static const uint32_t c_IRQ_INDEX_SW = 1;
    static const uint32_t c_IRQ_INDEX_DBG_COM_RX = 2;
    static const uint32_t c_IRQ_INDEX_DBG_COM_TX = 3;
    static const uint32_t c_IRQ_INDEX_TIMER0 = 4;
    static const uint32_t c_IRQ_INDEX_TIMER1 = 5;
    static const uint32_t c_IRQ_INDEX_UART0 = 6;
    static const uint32_t c_IRQ_INDEX_UART1 = 7;
    static const uint32_t c_IRQ_INDEX_PWM_0_1 = 8;
    static const uint32_t c_IRQ_INDEX_I2C0 = 9;
    static const uint32_t c_IRQ_INDEX_SPI_SSP0 = 10;
    static const uint32_t c_IRQ_INDEX_SSP1 = 11;
    static const uint32_t c_IRQ_INDEX_PLL = 12;
    static const uint32_t c_IRQ_INDEX_RTC = 13;
    static const uint32_t c_IRQ_INDEX_EINT0 = 14;
    static const uint32_t c_IRQ_INDEX_EINT1 = 15;
    static const uint32_t c_IRQ_INDEX_EINT2_LCD = 16;
    static const uint32_t c_IRQ_INDEX_EINT3 = 17;
    static const uint32_t c_IRQ_INDEX_ADC0 = 18;
    static const uint32_t c_IRQ_INDEX_I2C1 = 19;
    static const uint32_t c_IRQ_INDEX_BOD = 20;
    static const uint32_t c_IRQ_INDEX_EMAC = 21;
    static const uint32_t c_IRQ_INDEX_USB = 22;
    static const uint32_t c_IRQ_INDEX_CAN = 23;
    static const uint32_t c_IRQ_INDEX_SD = 24;
    static const uint32_t c_IRQ_INDEX_DMA = 25;
    static const uint32_t c_IRQ_INDEX_TIMER2 = 26;
    static const uint32_t c_IRQ_INDEX_TIMER3 = 27;
    static const uint32_t c_IRQ_INDEX_UART2 = 28;
    static const uint32_t c_IRQ_INDEX_UART3 = 29;
    static const uint32_t c_IRQ_INDEX_I2C2 = 30;
    static const uint32_t c_IRQ_INDEX_I2S = 31;
    //--//

    /****/ volatile uint32_t IRQSTATUS;   // Status of enabled IRQ requests
    /****/ volatile uint32_t FIQSTATUS;   // Status of enabled FIQ requests
    /****/ volatile uint32_t RAWINTR;     // Status of interrupt lines irrespective of the classificatino/enabling
    /****/ volatile uint32_t INTRSEL;     // FIQ or IRQ
    /****/ volatile uint32_t INTENABLE;   // Enable interrupt line
    /****/ volatile uint32_t INTENCLR;    // Disable interrupt line
    /****/ volatile uint32_t SOFTINT;     // Force SW interrupt
    /****/ volatile uint32_t SOFTINTCLR;  // Clear SW interrupt
    /****/ volatile uint32_t PROTECTEN;   // VIC registers can only be accessed in privileged mode
    /****/ volatile uint32_t PRIORITYMASK;// Mask interrupt priority level

    volatile uint32_t Padding0[54];

    volatile uint32_t VECTADDR[32];      // Address of vectored interrupt handler for slot 0 - 31

    volatile uint32_t Padding1[32];

    volatile uint32_t VECTPRIORITY[32];  // Priority control register for slot 0 - 31

    volatile uint32_t Padding2[800];

    volatile uint32_t ADDRESS;           // Address of the current Active interrupt

   //--//

    static const uint32_t c_MaxInterruptIndex = 32;

    //--//

    bool IsInterruptEnabled(uint32_t Irq_Index) {

        return ((INTENABLE >> Irq_Index) & 0x1) > 0 ? true : false;
    }

    bool GetInterruptState(uint32_t Irq_Index) {
        // Make changes to include the FIQSTATUS if the driver is modifed to support FIQ
        return ((IRQSTATUS >> Irq_Index) & 0x1) > 0 ? true : false;;
    }

    uint32_t NormalInterruptPending() {
        uint32_t index;

        for (index = 0; index < c_MaxInterruptIndex; index++) {
            if ((IRQSTATUS >> index) & 0x1) {
                return index;
            }
        }

        return c_MaxInterruptIndex;
    }

    void ForceInterrupt(uint32_t Irq_Index) {
        SOFTINT |= 1 << Irq_Index;
    }

    void RemoveForcedInterrupt(uint32_t Irq_Index) {
        SOFTINTCLR |= 1 << Irq_Index;
    }
};
//
// Interrupt Controller
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// TIMERS
//
struct LPC24XX_TIMER {

    static const uint32_t c_Timer_0 = 0;
    static const uint32_t c_Timer_1 = 1;
    static const uint32_t c_Timer_2 = 2;
    static const uint32_t c_Timer_3 = 3;
    static const uint32_t c_MaxTimer_no = 3;

    static const uint32_t c_TimerBase_0 = 0xE0004000;
    static const uint32_t c_TimerBase_1 = 0xE0008000;
    static const uint32_t c_TimerBase_2 = 0xE0070000;
    static const uint32_t c_TimerBase_3 = 0xE0074000;

    /****/ volatile uint32_t IR;     // Interrupt register
    static const    uint32_t MR0_COMP = 0x00000001;
    static const    uint32_t MR0_RESET = 0x00000001;

    /****/ volatile uint32_t TCR;    // Timer control register
    static const    uint32_t TCR_TEN = 0x00000001;

    /****/ volatile uint32_t TC;     // Timer counter
    /****/ volatile uint32_t PR;     // Pre scale register
    /****/ volatile uint32_t PC;     // Pre scale counter register
    /****/ volatile uint32_t MCR;    // Match control register
    /****/ volatile uint32_t MR0;    // Match 0 register
    static const    uint32_t MR0_IRQEN = 0x00000001;

    /****/ volatile uint32_t MR1;    // Match 1 register
    /****/ volatile uint32_t MR2;    // Match 2 register
    /****/ volatile uint32_t MR3;    // Match 3 register
    /****/ volatile uint32_t CCR;    // Capture control register
    /****/ volatile uint32_t CR0;    // Capture 0 register
    /****/ volatile uint32_t CR1;    // Capture 1 register
    /****/ volatile uint32_t CR2;    // Capture 2 register
    /****/ volatile uint32_t CR3;    // Capture 3 register
    /****/ volatile uint32_t EMR;    // External match register
    //functions.
    static uint32_t inline getIntNo(int Timer) {
        switch (Timer) {
            case  c_Timer_0:
                return LPC24XX_VIC::c_IRQ_INDEX_TIMER0;
                break;
            case  c_Timer_1:
                return LPC24XX_VIC::c_IRQ_INDEX_TIMER1;
                break;
            case  c_Timer_2:
                return LPC24XX_VIC::c_IRQ_INDEX_TIMER2;
                break;
            case  c_Timer_3:
                return LPC24XX_VIC::c_IRQ_INDEX_TIMER3;
                break;
            default:
                return(Timer);
                break;
        }
    }
};
//
// TIMER
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// UART
//

struct LPC24XX_USART {
    static const uint32_t c_Uart_0 = 0;
    static const uint32_t c_Uart_1 = 1;
    static const uint32_t c_Uart_2 = 2;
    static const uint32_t c_Uart_3 = 3;
    static const uint32_t c_MaxUart_no = 3;

    static const uint32_t c_UartBase_0 = 0xE000C000;
    static const uint32_t c_UartBase_1 = 0xE0010000;
    static const uint32_t c_UartBase_2 = 0xE0078000;
    static const uint32_t c_UartBase_3 = 0xE007C000;

    static const uint32_t c_ClockRate = LPC24_AHB_CLOCK_HZ;

    static const uint32_t c_MAX_BAUDRATE = c_ClockRate / 16;
    static const uint32_t c_MIN_BAUDRATE = 0;

    //IER
    static const uint32_t UART_IER_RLSIE = 0x00000004;   //Receive line status interrupt enable.
    static const uint32_t UART_IER_THREIE = 0x00000002;   //transmit hold register empty interrupt enable.
    static const uint32_t UART_IER_RDAIE = 0x00000001;   //Receive data available interrupt enable.

    static const uint32_t UART_IER_INTR_ALL_SET = UART_IER_RLSIE | UART_IER_THREIE | UART_IER_RDAIE;

    //IID
    static const uint32_t UART_IIR_FMES = 0x00000080;   //FIFO MODE Enable status.
    static const uint32_t UART_IIR_RFTLS_mask = 0x00000060;   //RX FIFO threshold level status.
    static const uint32_t UART_IIR_RFTLS_shift = 5;
    static const uint32_t UART_IIR_IID_mask = 0x0000000F;   //Interrupt identification.
    // values.
    static const uint32_t UART_IIR_IID_Irpt_RLS = 0x00000006;   // Receiver line status interrupt (e.g. overrun error) .
    static const uint32_t UART_IIR_IID_Irpt_RDA = 0x00000004;   // Receive data ready interrupt.
    static const uint32_t UART_IIR_IID_Irpt_TOUT = 0x0000000C;   // Receive FIFO timeout interrupt.
    static const uint32_t UART_IIR_IID_Irpt_THRE = 0x00000002;   // Transmitter holding register empty.

    static const uint32_t UART_IIR_NIP = 0x00000001;   //There is no pending interrupt.

    //FCR
    static const uint32_t UART_FCR_RFITL_mask = 0x000000C0;     // Rx FIFO trigger level
    static const uint32_t UART_FCR_RFITL_shift = 6;

    static const uint32_t UART_FCR_RFITL_01 = 0x00000000;
    static const uint32_t UART_FCR_RFITL_04 = 0x00000001;
    static const uint32_t UART_FCR_RFITL_08 = 0x00000002;
    static const uint32_t UART_FCR_RFITL_14 = 0x00000003;

    static const uint32_t UART_FCR_TFR = 0x00000004;     // Tx FIFO reset
    static const uint32_t UART_FCR_RFR = 0x00000002;     // Rx FIFO reset
    static const uint32_t UART_FCR_FME = 0x00000001;     // FIFO Mode enable


    union {
        struct {
            /****/ volatile uint32_t UART_RBR;                             //receive data register
        } RBR;
        struct {
            /****/ volatile uint32_t UART_THR;                            //transmit data register.

        } THR;
        struct {
            /****/ volatile uint32_t UART_DLL;                            //Divisor Latch register. (LSB)
        } DLL;

    } SEL1;

    union {
        struct {
            /****/ volatile uint32_t UART_IER;                                //Interrupt enable register
        } IER;
        struct {
            /****/ volatile uint32_t UART_DLM;                               //Divisor Latch register.  (MSB)
        } DLM;
    } SEL2;

    union {
        struct {
            /****/ volatile uint32_t  UART_IIR;                                        //Interrupt identification register.
        } IIR;
        struct {
            /****/ volatile uint32_t  UART_FCR;
        } FCR;
    } SEL3;

    /****/ volatile uint32_t UART_LCR;                                       // line control register.
    //--//
    static const uint32_t UART_LCR_DLAB = 0x00000080;     // Dividor Latch Access bit.
    static const uint32_t UART_LCR_BCB = 0x00000040;     // Break control bit.
    static const uint32_t UART_LCR_SPE = 0x00000020;     // Stick parity enable.
    static const uint32_t UART_LCR_EPE = 0x00000010;     // Even  parity enable.
    static const uint32_t UART_LCR_PBE = 0x00000008;     // Parity bit enable.
    static const uint32_t UART_LCR_NSB_1_STOPBITS = 0x00000000;     // Number of stop bits (0 - 1 stop bit; 1 - 1.5 stop bits).
    static const uint32_t UART_LCR_NSB_15_STOPBITS = 0x00000004;     // Number of stop bits (0 - 1 stop bit; 1 - 1.5 stop bits).
    static const uint32_t UART_LCR_WLS_mask = 0x00000003;     // Word length select.
    static const uint32_t UART_LCR_WLS_shift = 0;
    static const uint32_t UART_LCR_WLS_5_BITS = 0x00000000;
    static const uint32_t UART_LCR_WLS_6_BITS = 0x00000001;
    static const uint32_t UART_LCR_WLS_7_BITS = 0x00000002;
    static const uint32_t UART_LCR_WLS_8_BITS = 0x00000003;

    /****/ volatile uint32_t UART_MCR;                                       // modem control register.

    /****/ volatile uint32_t UART_LSR;                                       //line status register.
    static const uint32_t UART_LSR_ERR_RX = 0x00000080;     //Rx FIFO error
    static const uint32_t UART_LSR_TE = 0x00000040;     //Transmitter Empty.
    static const uint32_t UART_LSR_THRE = 0x00000020;     //Transmitter Holding register Empty.
    static const uint32_t UART_LSR_BII = 0x00000010;     //Break interrupt indicator.
    static const uint32_t UART_LSR_FEI = 0x00000008;     //Framing Error indicator.
    static const uint32_t UART_LSR_PEI = 0x00000004;     //Parity Error indicator.
    static const uint32_t UART_LSR_OEI = 0x00000002;     //Overrun Error indicator.
    static const uint32_t UART_LSR_RFDR = 0x00000001;     //RX FIFO data ready.

    /****/ volatile uint32_t UART_MSR;                                   //Modem status register.

    /****/ volatile uint32_t UART_SCR;
    /****/ volatile uint32_t UART_ACR;
    /****/ volatile uint32_t UART_RESERVED;
    /****/ volatile uint32_t UART_FDR;
    //functions.
    static uint32_t inline getIntNo(int ComPortNum) {
        switch (ComPortNum) {
            case c_Uart_0:
                return LPC24XX_VIC::c_IRQ_INDEX_UART0;
                break;
            case c_Uart_1:
                return LPC24XX_VIC::c_IRQ_INDEX_UART1;
                break;
            case c_Uart_2:
                return LPC24XX_VIC::c_IRQ_INDEX_UART2;
                break;
            case c_Uart_3:
                return LPC24XX_VIC::c_IRQ_INDEX_UART3;
                break;
            default:
                return(ComPortNum);
                break;
        }
    }
};
//
// UART
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Pin Connect Block
//
struct LPC24XX_PCB {
    static const uint32_t c_PCB_Base = 0xE002C000;

    struct {
        /****/ volatile uint32_t PINSEL;    // Pin Configuration Register
    } Regs[10];

    /****/ volatile uint32_t PINSEL10;      // ETM Interface Pin control

    /****/ volatile uint32_t PINSEL11;      // LCD function and mode control

    /****/ volatile uint32_t dummy[4];      // Filler to align address

    struct {
        /****/ volatile uint32_t PINMODE;   // Pin Mode Register
    } Regsm[10];

    static const uint32_t FUNC0 = 0x0;
    static const uint32_t FUNC1 = 0x1;
    static const uint32_t FUNC2 = 0x2;
    static const uint32_t FUNC3 = 0x3;
};
//
// Pin Connect Block
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// WATCHDOG
//
struct LPC24XX_WATCHDOG {
    static const uint32_t c_WATCHDOG_Base = 0xE0000000;

    //--//

    /****/ volatile uint32_t WDMOD;
    static const    uint32_t WDMOD__WDEN = 0x00000001;
    static const    uint32_t WDMOD__WDRESET = 0x00000002;
    static const    uint32_t WDMOD__WDTOF = 0x00000004;
    static const    uint32_t WDMOD__WDINT = 0x00000008;

    /****/ volatile uint32_t WDTC;
    static const    uint32_t WDTC_mask = 0x000000FF;

    /****/ volatile uint32_t WDFEED;
    static const    uint32_t WDFEED_reload_1 = 0x000000AA;
    static const    uint32_t WDFEED_reload_2 = 0x00000055;
    static const    uint32_t WDFEED_mask = 0x000000FF;

    /****/ volatile uint32_t WDTV;
    static const    uint32_t WDTV_mask = 0x000000FF;

};
//
// WATCHDOG
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// I2C
//
struct LPC24XX_I2C {
    static const uint32_t c_I2C_Base = 0xE001C000;
    static const uint32_t c_I2C_Clk_KHz = SYSTEM_CLOCK_HZ/1000;

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
//
// I2C
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// SPI
//
struct LPC24XX_SPI {
    static const uint32_t c_SPI0_Base = 0xE0068000;
    static const uint32_t c_SPI1_Base = 0xE0030000;

    static const uint32_t c_SPI_Clk_KHz = (LPC24_AHB_CLOCK_HZ/1000);
    static const uint32_t c_SPI0 = 0;
    static const uint32_t c_SPI1 = 1;


    static const uint32_t c_MAX_SPI = 2;

    //static const uint32_t c_SPI0_SCLK = 42;//LPC22XX_GPIO::c_P0_04;
    //static const uint32_t c_SPI0_MISO = 40;//LPC22XX_GPIO::c_P0_05;
    //static const uint32_t c_SPI0_MOSI = 41;//LPC22XX_GPIO::c_P0_06;
    //static const uint32_t c_SPI0_SSEL = 43;//LPC22XX_GPIO::c_P0_07;

    //static const uint32_t c_SPI1_SCLK = 10;//LPC22XX_GPIO::c_P0_17;
    //static const uint32_t c_SPI1_MISO = 8;//LPC22XX_GPIO::c_P0_18;
    //static const uint32_t c_SPI1_MOSI = 6;//LPC22XX_GPIO::c_P0_19;
    //static const uint32_t c_SPI1_SSEL = 12;//LPC22XX_GPIO::c_P0_20;

    volatile uint32_t SSPxCR0;
    volatile uint32_t SSPxCR1;
    volatile uint32_t SSPxDR;
    volatile uint32_t SSPxSR;
    volatile uint32_t SSPxCPSR;


    static const uint32_t CONTROLREG_BitEnable = 0x00000004;
    static const uint32_t CONTROLREG_MODE_Master = 0x00000020;
    static const uint32_t CONTROLREG_PHA_1 = 0x00000008;
    static const uint32_t CONTROLREG_POL_1 = 0x00000010;
};
//
// SPI
//////////////////////////////////////////////////////////////////////////////

struct LPC24XX {
    static LPC24XX_VIC    & VIC() { return *(LPC24XX_VIC    *)(size_t)(LPC24XX_VIC::c_VIC_Base); }
    //static LPC24XX_GPIO   & GPIO   (         ) { return *(LPC24XX_GPIO   *)(size_t)(      LPC24XX_GPIO  ::c_GPIO_Base                                 ); }
    //static LPC24XX_GPIOIRQ   & GPIOIRQ   (         ) { return *(LPC24XX_GPIOIRQ   *)(size_t)(      LPC24XX_GPIOIRQ  ::c_GPIOIRQ_Base                                 ); }
    static LPC24XX_PCB    & PCB() { return *(LPC24XX_PCB    *)(size_t)(LPC24XX_PCB::c_PCB_Base); }
    static LPC24XX_SYSCON & SYSCON() { return *(LPC24XX_SYSCON *)(size_t)(LPC24XX_SYSCON::c_SYSCON_Base); }
    //static LPC24XX_EMC    & EMC    (         ) { return *(LPC24XX_EMC    *)(size_t)(      LPC24XX_EMC   ::c_EMC_Base                                  ); }
    static LPC24XX_SPI    & SPI(int sel) { return *(LPC24XX_SPI    *)(size_t)((sel == 0) ? (LPC24XX_SPI::c_SPI0_Base) : (LPC24XX_SPI::c_SPI1_Base)); }
    static LPC24XX_I2C    & I2C() { return *(LPC24XX_I2C    *)(size_t)(LPC24XX_I2C::c_I2C_Base); }
    static LPC24XX_WATCHDOG & WTDG() { return *(LPC24XX_WATCHDOG *)(size_t)(LPC24XX_WATCHDOG::c_WATCHDOG_Base); }
    //static LPC24XX_DAC    & DAC    (         ) { return *(LPC24XX_DAC    *)(size_t)(      LPC24XX_DAC   ::c_DAC_Base                                  ); }


    static LPC24XX_TIMER  & TIMER(int sel) {

        if (sel == LPC24XX_TIMER::c_Timer_0)
            return *(LPC24XX_TIMER  *)(size_t)LPC24XX_TIMER::c_TimerBase_0;
        else if (sel == LPC24XX_TIMER::c_Timer_1)
            return *(LPC24XX_TIMER  *)(size_t)LPC24XX_TIMER::c_TimerBase_1;
        else if (sel == LPC24XX_TIMER::c_Timer_2)
            return *(LPC24XX_TIMER  *)(size_t)LPC24XX_TIMER::c_TimerBase_2;
        else
            return *(LPC24XX_TIMER  *)(size_t)LPC24XX_TIMER::c_TimerBase_3;
    }

    static LPC24XX_USART  & UART(int sel) {

        if (sel == LPC24XX_USART::c_Uart_0)
            return *(LPC24XX_USART  *)(size_t)LPC24XX_USART::c_UartBase_0;
        else if (sel == LPC24XX_USART::c_Uart_1)
            return *(LPC24XX_USART  *)(size_t)LPC24XX_USART::c_UartBase_1;
        else if (sel == LPC24XX_USART::c_Uart_2)
            return *(LPC24XX_USART  *)(size_t)LPC24XX_USART::c_UartBase_2;
        else
            return *(LPC24XX_USART  *)(size_t)LPC24XX_USART::c_UartBase_3;
    }
    //--//
};

