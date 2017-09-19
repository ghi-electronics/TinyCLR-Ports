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


#include <inc\LPC177x_8x.h>
#include <TinyCLR.h>
#include <Device.h>

#undef LPC17

#define SIZEOF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
#define CHARIZE2(c) #c
#define CHARIZE(c) (CHARIZE2(c)[0])

// ADC
const TinyCLR_Api_Info* LPC17_Adc_GetApi();
void LPC17_Adc_Reset();
TinyCLR_Result LPC17_Adc_Acquire(const TinyCLR_Adc_Provider* self);
TinyCLR_Result LPC17_Adc_Release(const TinyCLR_Adc_Provider* self);
TinyCLR_Result LPC17_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result LPC17_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result LPC17_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value);
int32_t LPC17_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self);
int32_t LPC17_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self);
int32_t LPC17_Adc_GetMinValue(const TinyCLR_Adc_Provider* self);
int32_t LPC17_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self);
TinyCLR_Adc_ChannelMode LPC17_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self);
TinyCLR_Result LPC17_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);
bool LPC17_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);

//DAC
const TinyCLR_Api_Info* LPC17_Dac_GetApi();
void LPC17_Dac_Reset();
TinyCLR_Result LPC17_Dac_Acquire(const TinyCLR_Dac_Provider* self);
TinyCLR_Result LPC17_Dac_Release(const TinyCLR_Dac_Provider* self);
TinyCLR_Result LPC17_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result LPC17_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result LPC17_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value);
int32_t LPC17_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self);
int32_t LPC17_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self);
int32_t LPC17_Dac_GetMinValue(const TinyCLR_Dac_Provider* self);
int32_t LPC17_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self);

// GPIO
enum class LPC17_Gpio_Direction : uint8_t {
    Input = 0,
    Output = 1,
};

enum class LPC17_Gpio_PinFunction : uint8_t {
    PinFunction0 = 0,
    PinFunction1 = 1,
    PinFunction2 = 2,
    PinFunction3 = 3,
    PinFunction4 = 4,
    PinFunction5 = 5,
    PinFunction6 = 6,
    PinFunction7 = 7
};

enum class LPC17_Gpio_ResistorMode : uint8_t {
    Inactive = 0,
    PullDown = 1,
    PullUp = 2,
    RepeaterMode = 3
};

enum class LPC17_Gpio_Hysteresis : uint8_t {
    Disable = 0,
    Enable = 1
};

enum class LPC17_Gpio_InputPolarity : uint8_t {
    NotInverted = 0,
    Inverted = 1
};

enum class LPC17_Gpio_SlewRate : uint8_t {
    StandardMode = 0,
    FastMode = 1
};

enum class LPC17_Gpio_OutputType : uint8_t {
    PushPull = 0,
    OpenDrain = 1,
};

struct LPC17_Gpio_Pin {
    uint32_t number;
    LPC17_Gpio_PinFunction pinFunction;
};

struct LPC17_Gpio_PinConfiguration {
    LPC17_Gpio_Direction direction;
    LPC17_Gpio_ResistorMode resistorMode;
    LPC17_Gpio_Hysteresis hysteresis;
    LPC17_Gpio_InputPolarity inputPolarity;
    LPC17_Gpio_SlewRate slewRate;
    LPC17_Gpio_OutputType outputType;
    LPC17_Gpio_PinFunction pinFunction;
};

#define PIN(port, pin) (port * 32 + pin)
#define PIN_NONE 0xFFFFFFFF
#define PF(num) (CONCAT(LPC17_Gpio_PinFunction::PinFunction, num))
#define PF_NONE LPC17_Gpio_PinFunction::PinFunction0

#define INIT(direction, resistorMode, hysteresis, inputPolarity, slewRate, outputType, pinFunction) { LPC17_Gpio_Direction::direction, LPC17_Gpio_ResistorMode::resistorMode, LPC17_Gpio_Hysteresis::hysteresis, LPC17_Gpio_InputPolarity::inputPolarity, LPC17_Gpio_SlewRate::slewRate, LPC17_Gpio_OutputType::outputType,LPC17_Gpio_PinFunction::pinFunction }
#define ALTFUN(direction, resistorMode, outputType, pinFunction) { LPC17_Gpio_Direction::direction, LPC17_Gpio_ResistorMode::resistorMode, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::outputType,LPC17_Gpio_PinFunction::pinFunction }
#define INPUT(outputType, resistorMode) { LPC17_Gpio_Direction::Input, LPC17_Gpio_ResistorMode::resistorMode, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::outputType,LPC17_Gpio_PinFunction::PinFunction0 }

void LPC17_Gpio_Reset();
const TinyCLR_Api_Info* LPC17_Gpio_GetApi();
TinyCLR_Result LPC17_Gpio_Acquire(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result LPC17_Gpio_Release(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result LPC17_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t alternate);
TinyCLR_Result LPC17_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result LPC17_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result LPC17_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, int32_t debounceTime);
TinyCLR_Result LPC17_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result LPC17_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
bool LPC17_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode LPC17_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t LPC17_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t LPC17_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result LPC17_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR);
TinyCLR_Result LPC17_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin);

bool LPC17_Gpio_OpenPin(int32_t pin);
bool LPC17_Gpio_ClosePin(int32_t pin);
bool LPC17_Gpio_ConfigurePin(int32_t pin, LPC17_Gpio_Direction pinDir, LPC17_Gpio_PinFunction pinFunction, LPC17_Gpio_ResistorMode pullResistor, LPC17_Gpio_Hysteresis hysteresis, LPC17_Gpio_InputPolarity inputPolarity, LPC17_Gpio_SlewRate slewRate, LPC17_Gpio_OutputType outputType);
void LPC17_Gpio_EnableOutputPin(int32_t pin, bool initialState);
void LPC17_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor);

// PWM
#define PWM0MR0 (*(volatile unsigned long *)0x40014018)
#define PWM0MR1 ((uint32_t *)0x4001401C)
#define PWM0MR2 ((uint32_t *)0x40014020)
#define PWM0MR3 ((uint32_t *)0x40014024)
#define PWM0MR4 ((uint32_t *)0x40014040)
#define PWM0MR5 ((uint32_t *)0x40014044)
#define PWM0MR6 ((uint32_t *)0x40014048)

#define PWM1MR0 (*(volatile unsigned long *)0x40018018)
#define PWM1MR1 ((uint32_t *)0x4001801C)
#define PWM1MR2 ((uint32_t *)0x40018020)
#define PWM1MR3 ((uint32_t *)0x40018024)
#define PWM1MR4 ((uint32_t *)0x40018040)
#define PWM1MR5 ((uint32_t *)0x40018044)
#define PWM1MR6 ((uint32_t *)0x40018048)
#define MAX_PWM_PER_CONTROLLER              6

struct PwmController {
    int32_t                         id;
    int32_t                         channel[MAX_PWM_PER_CONTROLLER];
    int32_t                         subChannel[MAX_PWM_PER_CONTROLLER];
    LPC17_Gpio_Pin                  gpioPin[MAX_PWM_PER_CONTROLLER];    
    uint32_t                        outputEnabled[MAX_PWM_PER_CONTROLLER];
    uint32_t                        *matchAddress[MAX_PWM_PER_CONTROLLER];
    bool                            invert[MAX_PWM_PER_CONTROLLER];
    double                          frequency;
    double                          dutyCycle[MAX_PWM_PER_CONTROLLER];
};

const TinyCLR_Api_Info* LPC17_Pwm_GetApi();
PwmController* LPC17_Pwm_GetControllers();
void LPC17_Pwm_Reset();
void LPC17_Pwm_ResetController(int32_t controller);
TinyCLR_Result LPC17_Pwm_Acquire(const TinyCLR_Pwm_Provider* self);
TinyCLR_Result LPC17_Pwm_Release(const TinyCLR_Pwm_Provider* self);
int32_t LPC17_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC17_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency);
TinyCLR_Result LPC17_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC17_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC17_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC17_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result LPC17_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity);
double LPC17_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self);
double LPC17_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self);
double LPC17_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self);
int32_t LPC17_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self);

//SPI
const TinyCLR_Api_Info* LPC17_Spi_GetApi();
void LPC17_Spi_Reset();
bool LPC17_Spi_Transaction_Start(int32_t controller);
bool LPC17_Spi_Transaction_Stop(int32_t controller);
bool LPC17_Spi_Transaction_nWrite8_nRead8(int32_t controller);
TinyCLR_Result LPC17_Spi_Acquire(const TinyCLR_Spi_Provider* self);
TinyCLR_Result LPC17_Spi_Release(const TinyCLR_Spi_Provider* self);
TinyCLR_Result LPC17_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode);
TinyCLR_Result LPC17_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
TinyCLR_Result LPC17_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
int32_t LPC17_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self);
int32_t LPC17_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self);
int32_t LPC17_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self);
TinyCLR_Result LPC17_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
const TinyCLR_Api_Info* LPC17_Uart_GetApi();
void LPC17_Uart_Reset();
int32_t LPC17_Uart_GetTxPin(int32_t portNum);
int32_t LPC17_Uart_GetRxPin(int32_t portNum);
int32_t LPC17_Uart_GetRtsPin(int32_t portNum);
int32_t LPC17_Uart_GetCtsPin(int32_t portNum);
LPC17_Gpio_PinFunction LPC17_Uart_GetTxAlternateFunction(int32_t portNum);
LPC17_Gpio_PinFunction LPC17_Uart_GetRxAlternateFunction(int32_t portNum);
LPC17_Gpio_PinFunction LPC17_Uart_GetRtsAlternateFunction(int32_t portNum);
LPC17_Gpio_PinFunction LPC17_Uart_GetCtsAlternateFunction(int32_t portNum);
bool LPC17_Uart_TxHandshakeEnabledState(int portNum);
void LPC17_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable);
void LPC17_Uart_RxBufferFullInterruptEnable(int portNum, bool enable);
TinyCLR_Result LPC17_Uart_Acquire(const TinyCLR_Uart_Provider* self);
TinyCLR_Result LPC17_Uart_Release(const TinyCLR_Uart_Provider* self);
TinyCLR_Result LPC17_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking);
TinyCLR_Result LPC17_Uart_Flush(const TinyCLR_Uart_Provider* self);
TinyCLR_Result LPC17_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler);
TinyCLR_Result LPC17_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result LPC17_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result LPC17_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC17_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result LPC17_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC17_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC17_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC17_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC17_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result LPC17_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result LPC17_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state);

//Deployment
const TinyCLR_Api_Info* LPC17_Deployment_GetApi();
TinyCLR_Result LPC17_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool &supportXIP);
TinyCLR_Result LPC17_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result LPC17_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result LPC17_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result LPC17_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result LPC17_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased);
TinyCLR_Result LPC17_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);
TinyCLR_Result LPC17_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
bool LPC17_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer);
bool LPC17_Flash_IsSupportsXIP(const TinyCLR_Deployment_Provider* self);

// Interrupt
class LPC17_SmartPtr_IRQ {
    uint32_t m_state;

    void Disable();
    void Restore();

public:
    LPC17_SmartPtr_IRQ();
    ~LPC17_SmartPtr_IRQ();

    bool WasDisabled();
    void Acquire();
    void Release();
    void Probe();

    static bool GetState();
};

class LPC17_SmartPtr_Interrupt {
public:
    LPC17_SmartPtr_Interrupt();
    ~LPC17_SmartPtr_Interrupt();
};

#define DISABLE_INTERRUPTS_SCOPED(name) LPC17_SmartPtr_IRQ name
#define INTERRUPT_STARTED_SCOPED(name) LPC17_SmartPtr_Interrupt name

const TinyCLR_Api_Info* LPC17_Interrupt_GetApi();
TinyCLR_Result LPC17_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result LPC17_Interrupt_Release();
bool LPC17_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param);
bool LPC17_Interrupt_Deactivate(uint32_t Irq_Index);
bool LPC17_Interrupt_Enable(uint32_t Irq_Index);
bool LPC17_Interrupt_Disable(uint32_t Irq_Index);
bool LPC17_Interrupt_EnableState(uint32_t Irq_Index);
bool LPC17_Interrupt_InterruptState(uint32_t Irq_Index);

bool LPC17_Interrupt_GlobalEnabled(bool force);
bool LPC17_Interrupt_GlobalDisabled(bool force);
void LPC17_Interrupt_GlobalRestore();
bool LPC17_Interrupt_GlobalIsDisabled();
void LPC17_Interrupt_GlobalWaitForInterrupt();

extern TinyCLR_Interrupt_StartStopHandler LPC17_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler LPC17_Interrupt_Ended;

// I2C
const TinyCLR_Api_Info* LPC17_I2c_GetApi();
void LPC17_I2c_Reset();
TinyCLR_Result LPC17_I2c_Acquire(const TinyCLR_I2c_Provider* self);
TinyCLR_Result LPC17_I2c_Release(const TinyCLR_I2c_Provider* self);
TinyCLR_Result LPC17_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result LPC17_I2c_ReadTransaction(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result LPC17_I2c_WriteTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result LPC17_I2c_WriteReadTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result);
void LPC17_I2c_StartTransaction();
void LPC17_I2c_StopTransaction();

// Time
const TinyCLR_Api_Info* LPC17_Time_GetApi();
TinyCLR_Result LPC17_Time_Acquire(const TinyCLR_Time_Provider* self);
TinyCLR_Result LPC17_Time_Release(const TinyCLR_Time_Provider* self);
TinyCLR_Result LPC17_Time_GetInitialTime(const TinyCLR_Time_Provider* self, int64_t& utcTime, int32_t& timeZoneOffsetMinutes);
uint64_t LPC17_Time_GetTimeForProcessorTicks(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t LPC17_Time_TimeToTicks(const TinyCLR_Time_Provider* self, uint64_t time);
uint64_t LPC17_Time_MillisecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t LPC17_Time_MicrosecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t microseconds);
uint64_t LPC17_Time_GetCurrentTicks(const TinyCLR_Time_Provider* self);
TinyCLR_Result LPC17_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks);
TinyCLR_Result LPC17_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback);
void LPC17_Time_DelayNoInterrupt(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void LPC17_Time_Delay(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void LPC17_Time_GetDriftParameters(const TinyCLR_Time_Provider* self, int32_t* a, int32_t* b, int64_t* c);

// Power
const TinyCLR_Api_Info* LPC17_Power_GetApi();
void LPC17_Power_SetHandlers(void(*stop)(), void(*restart)());
void LPC17_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_Sleep_Level level);
void LPC17_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter);
TinyCLR_Result LPC17_Power_Acquire(const TinyCLR_Power_Provider* self);
TinyCLR_Result LPC17_Power_Release(const TinyCLR_Power_Provider* self);

//UsbClient
const TinyCLR_Api_Info* LPC17_UsbClient_GetApi();
void LPC17_UsbClient_Reset();
TinyCLR_Result LPC17_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result LPC17_UsbClient_Release(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result LPC17_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t & stream, TinyCLR_UsbClient_StreamMode mode);
TinyCLR_Result LPC17_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result LPC17_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t stream, const uint8_t* data, size_t& length);
TinyCLR_Result LPC17_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t stream, uint8_t* data, size_t& length);
TinyCLR_Result LPC17_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result LPC17_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler);

TinyCLR_Result LPC17_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result LPC17_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result LPC17_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value);
TinyCLR_Result LPC17_UsbClient_SetOsExtendedPropertyHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler);

// LCD
void LPC17_Display_Reset();
const TinyCLR_Api_Info* LPC17_Display_GetApi();
TinyCLR_Result LPC17_Display_Acquire(const TinyCLR_Display_Provider* self, uint32_t width, uint32_t height);
TinyCLR_Result LPC17_Display_Release(const TinyCLR_Display_Provider* self);
TinyCLR_Result LPC17_Display_SetLcdConfiguration(const TinyCLR_Display_Provider* self, bool outputEnableIsFixed, bool outputEnablePolarity, bool pixelPolarity, uint32_t pixelClockRate, bool horizontalSyncPolarity, uint32_t horizontalSyncPulseWidth, uint32_t horizontalFrontPorch, uint32_t horizontalBackPorch, bool verticalSyncPolarity, uint32_t verticalSyncPulseWidth, uint32_t verticalFrontPorch, uint32_t verticalBackPorch);
TinyCLR_Result LPC17_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data, TinyCLR_Display_Format dataFormat);
TinyCLR_Result LPC17_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer);
int32_t LPC17_Display_GetWidth(const TinyCLR_Display_Provider* self);
int32_t LPC17_Display_GetHeight(const TinyCLR_Display_Provider* self);
TinyCLR_Display_InterfaceType LPC17_Display_GetType(const TinyCLR_Display_Provider* self);

//Startup
void LPC17_Startup_Initialize();
void LPC17_Startup_GetHeap(uint8_t*& start, size_t& length);
void LPC17_Startup_GetDebugger(const TinyCLR_Api_Info*& api, size_t& index);
void LPC17_Startup_GetRunApp(bool& runApp);

//System Control Block
#define PCON_OFFSET 0xC0
#define PCON_IDL_MASK 0x1
#define PCON_IDL 0x1
#define PCON_IDL_BIT 0
#define PCON_PD_MASK 0x2
#define PCON_PD 0x2
#define PCON_PD_BIT 1
#define PCON_BODPDM_MASK 0x4
#define PCON_BODPDM 0x4
#define PCON_BODPDM_BIT 2
#define PCON_BOGD_MASK 0x8
#define PCON_BOGD 0x8
#define PCON_BOGD_BIT 3
#define PCON_BORD_MASK 0x10
#define PCON_BORD 0x10
#define PCON_BORD_BIT 4
#define PCON_PM2_MASK 0x80
#define PCON_PM2 0x80
#define PCON_PM2_BIT 7

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
#define PCONP_PCUART4_MASK 0x100
#define PCONP_PCUART4 0x100
#define PCONP_PCUART4_BIT 8
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
#define PCONP_PCSSP2_MASK 0x100000
#define PCONP_PCSSP2 0x100000
#define PCONP_PCSSP2_BIT 20
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

//////////////////////////////////////////////////////////////////////////////
// System Control Block
//
struct LPC17xx_SYSCON {
    static const uint32_t c_SYSCON_Base = 0x400FC000;

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
    static const uint32_t ENABLE_LCD = 0x00000001;
    /****/ volatile uint32_t dummy3[15];    // Filler to align next register address

    // Clock Control
    /****/ volatile uint32_t CCLKCFG;        // CPU Clock Configuration Register
    /****/ volatile uint32_t USBCLKCFG;      // USB Clock Configuration Register

    /****/ volatile uint32_t CLKSRCSEL;      // Clock Source Select Register
    static const uint32_t IRC = 0x0;         // Internal RC Oscillator
    static const uint32_t OSC = 0x1;         // Main Oscillator
    //static const uint32_t RTC = 0x2;         // RTC Oscillator

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
