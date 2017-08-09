// Copyright 2017 GHI Electronics, LLC
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

#include <defines.h>
#include <TinyCLR.h>
#include <DeviceSelector.h>

// ADC
const TinyCLR_Api_Info* STM32F4_Adc_GetApi();
void STM32F4_Adc_Reset();
TinyCLR_Result STM32F4_Adc_Acquire(const TinyCLR_Adc_Provider* self);
TinyCLR_Result STM32F4_Adc_Release(const TinyCLR_Adc_Provider* self);
TinyCLR_Result STM32F4_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value);
int32_t STM32F4_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self);
int32_t STM32F4_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self);
int32_t STM32F4_Adc_GetMinValue(const TinyCLR_Adc_Provider* self);
int32_t STM32F4_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self);
TinyCLR_Adc_ChannelMode STM32F4_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self);
TinyCLR_Result STM32F4_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);
bool STM32F4_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);

//DAC
const TinyCLR_Api_Info* STM32F4_Dac_GetApi();
void STM32F4_Dac_Reset();
TinyCLR_Result STM32F4_Dac_Acquire(const TinyCLR_Dac_Provider* self);
TinyCLR_Result STM32F4_Dac_Release(const TinyCLR_Dac_Provider* self);
TinyCLR_Result STM32F4_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value);
int32_t STM32F4_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self);
int32_t STM32F4_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self);
int32_t STM32F4_Dac_GetMinValue(const TinyCLR_Dac_Provider* self);
int32_t STM32F4_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self);

// GPIO
enum class STM32F4_Gpio_PullDirection : uint8_t {
    None = 0,
    PullUp = 1,
    PullDown = 2,
    Reserved = 3
};
enum class STM32F4_Gpio_PortMode : uint8_t {
    Input = 0,
    GeneralPurposeOutput = 1,
    AlternateFunction = 2,
    Analog = 3
};

enum class STM32F4_Gpio_OutputSpeed : uint8_t {
    Low = 0,
    Medium = 1,
    Fast = 2,
    High = 3
};

enum class STM32F4_Gpio_AlternateFunction : uint8_t {
    AF0 = 0,
    AF1 = 1,
    AF2 = 2,
    AF3 = 3,
    AF4 = 4,
    AF5 = 5,
    AF6 = 6,
    AF7 = 7,
    AF8 = 8,
    AF9 = 9,
    AF10 = 10,
    AF11 = 11,
    AF12 = 12,
    AF13 = 12,
    AF14 = 14,
    AF15 = 15
};

enum class STM32F4_Gpio_OutputType : uint8_t {
    PushPull = 0,
    OpenDrain = 1,
};

void STM32F4_Gpio_Reset();
const TinyCLR_Api_Info* STM32F4_Gpio_GetApi();
TinyCLR_Result STM32F4_Gpio_Acquire(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result STM32F4_Gpio_Release(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result STM32F4_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t output, uint32_t alternate);
TinyCLR_Result STM32F4_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result STM32F4_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result STM32F4_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, int32_t debounceTime);
TinyCLR_Result STM32F4_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result STM32F4_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
bool STM32F4_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode STM32F4_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t STM32F4_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t STM32F4_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result STM32F4_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR);
TinyCLR_Result STM32F4_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
bool STM32F4_Gpio_OpenPin(int32_t pin);
void STM32F4_Gpio_ClosePin(int32_t pin);
bool STM32F4_Gpio_ReadPin(int32_t pin);
void STM32F4_Gpio_WritePin(int32_t pin, bool value);
void STM32F4_Gpio_ConfigurePin(int32_t pin, STM32F4_Gpio_PortMode portMode, STM32F4_Gpio_OutputType outputType, STM32F4_Gpio_OutputSpeed outputSpeed, STM32F4_Gpio_PullDirection pullDirection, STM32F4_Gpio_AlternateFunction alternateFunction);

// PWM
const TinyCLR_Api_Info* STM32F4_Pwm_GetApi();
void STM32F4_Pwm_Reset();
void STM32F4_Pwm_ResetController(int32_t controller);
TinyCLR_Result STM32F4_Pwm_Acquire(const TinyCLR_Pwm_Provider* self);
TinyCLR_Result STM32F4_Pwm_Release(const TinyCLR_Pwm_Provider* self);
int32_t STM32F4_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency);
TinyCLR_Result STM32F4_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity);
double STM32F4_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self);
double STM32F4_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self);
double STM32F4_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self);
int32_t STM32F4_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self);

//SPI
const TinyCLR_Api_Info* STM32F4_Spi_GetApi();
void STM32F4_Spi_Reset();
bool STM32F4_Spi_Transaction_Start(int32_t controller);
bool STM32F4_Spi_Transaction_Stop(int32_t controller);
bool STM32F4_Spi_Transaction_nWrite8_nRead8(int32_t controller);
TinyCLR_Result STM32F4_Spi_Acquire(const TinyCLR_Spi_Provider* self);
TinyCLR_Result STM32F4_Spi_Release(const TinyCLR_Spi_Provider* self);
TinyCLR_Result STM32F4_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode);
TinyCLR_Result STM32F4_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F4_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F4_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
TinyCLR_Result STM32F4_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
int32_t STM32F4_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self);
int32_t STM32F4_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self);
int32_t STM32F4_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self);
TinyCLR_Result STM32F4_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
const TinyCLR_Api_Info* STM32F4_Uart_GetApi();
void STM32F4_Uart_Reset();
bool STM32F4_Uart_TxHandshakeEnabledState(int portNum);
void STM32F4_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable);
void STM32F4_Uart_RxBufferFullInterruptEnable(int portNum, bool enable);
TinyCLR_Result STM32F4_Uart_Acquire(const TinyCLR_Uart_Provider* self);
TinyCLR_Result STM32F4_Uart_Release(const TinyCLR_Uart_Provider* self);
TinyCLR_Result STM32F4_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking);
TinyCLR_Result STM32F4_Uart_Flush(const TinyCLR_Uart_Provider* self);
TinyCLR_Result STM32F4_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F4_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F4_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler);
TinyCLR_Result STM32F4_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result STM32F4_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result STM32F4_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result STM32F4_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result STM32F4_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result STM32F4_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result STM32F4_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result STM32F4_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result STM32F4_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result STM32F4_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result STM32F4_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state);

//Deployment
const TinyCLR_Api_Info* STM32F4_Deployment_GetApi();
TinyCLR_Result STM32F4_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool &supportXIP);
TinyCLR_Result STM32F4_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result STM32F4_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result STM32F4_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result STM32F4_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result STM32F4_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased);
TinyCLR_Result STM32F4_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);
TinyCLR_Result STM32F4_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
bool STM32F4_Flash_IsSupportsXIP(const TinyCLR_Deployment_Provider* self);

// Interrupt
class STM32F4_SmartPtr_IRQ {
    uint32_t m_state;

public:
    STM32F4_SmartPtr_IRQ() { Disable(); };
    ~STM32F4_SmartPtr_IRQ() { Restore(); };

    bool WasDisabled();
    void Acquire();
    void Release();
    void Probe();

    static bool GetState();

private:
    void Disable();
    void Restore();
};

const TinyCLR_Api_Info* STM32F4_Interrupt_GetApi();
TinyCLR_Result STM32F4_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result STM32F4_Interrupt_Release();
bool STM32F4_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param);
bool STM32F4_Interrupt_Deactivate(uint32_t Irq_Index);
bool STM32F4_Interrupt_Enable(uint32_t Irq_Index);
bool STM32F4_Interrupt_Disable(uint32_t Irq_Index);
bool STM32F4_Interrupt_EnableState(uint32_t Irq_Index);
bool STM32F4_Interrupt_InterruptState(uint32_t Irq_Index);

bool STM32F4_Interrupt_GlobalEnabled(bool force);
bool STM32F4_Interrupt_GlobalDisabled(bool force);
bool STM32F4_Interrupt_GlobalIsDisabled();
void STM32F4_Interrupt_GlobalWaitForInterrupt();
void STM32F4_Interrupt_GlobalRestore();

extern TinyCLR_Interrupt_StartStopHandler STM32F4_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler STM32F4_Interrupt_Ended;

// I2C
const TinyCLR_Api_Info* STM32F4_I2c_GetApi();
void STM32F4_I2c_Reset();
TinyCLR_Result STM32F4_I2c_Acquire(const TinyCLR_I2c_Provider* self);
TinyCLR_Result STM32F4_I2c_Release(const TinyCLR_I2c_Provider* self);
TinyCLR_Result STM32F4_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result STM32F4_I2c_ReadTransaction(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result STM32F4_I2c_WriteTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result STM32F4_I2c_WriteReadTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result);
void STM32F4_I2c_StartTransaction();
void STM32F4_I2c_StopTransaction();

// Time
const TinyCLR_Api_Info* STM32F4_Time_GetApi();
TinyCLR_Result STM32F4_Time_Acquire(const TinyCLR_Time_Provider* self);
TinyCLR_Result STM32F4_Time_Release(const TinyCLR_Time_Provider* self);
TinyCLR_Result STM32F4_Time_GetInitialTime(const TinyCLR_Time_Provider* self, int64_t& utcTime, int32_t& timeZoneOffsetMinutes);
uint64_t STM32F4_Time_TicksToTime(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t STM32F4_Time_TimeToTicks(const TinyCLR_Time_Provider* self, uint64_t time);
uint64_t STM32F4_Time_MillisecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t STM32F4_Time_MicrosecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t microseconds);
uint64_t STM32F4_Time_GetCurrentTicks(const TinyCLR_Time_Provider* self);
TinyCLR_Result STM32F4_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks);
TinyCLR_Result STM32F4_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback);
void STM32F4_Time_DelayNoInterrupt(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void STM32F4_Time_Delay(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void STM32F4_Time_GetDriftParameters(const TinyCLR_Time_Provider* self, int32_t* a, int32_t* b, int64_t* c);

// Power
const TinyCLR_Api_Info* STM32F4_Power_GetApi();
void STM32F4_Power_SetHandlers(void(*stop)(), void(*restart)());
void STM32F4_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_Sleep_Level level);
void STM32F4_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter);
TinyCLR_Result STM32F4_Power_Acquire(const TinyCLR_Power_Provider* self);
TinyCLR_Result STM32F4_Power_Release(const TinyCLR_Power_Provider* self);

//UsbClient
const TinyCLR_Api_Info* STM32F4_UsbClient_GetApi();
void STM32F4_UsbClient_Reset();
TinyCLR_Result STM32F4_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result STM32F4_UsbClient_Release(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result STM32F4_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t & stream, TinyCLR_UsbClient_StreamMode mode);
TinyCLR_Result STM32F4_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result STM32F4_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t stream, const uint8_t* data, size_t& length);
TinyCLR_Result STM32F4_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t stream, uint8_t* data, size_t& length);
TinyCLR_Result STM32F4_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result STM32F4_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler);

TinyCLR_Result STM32F4_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result STM32F4_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result STM32F4_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value);
TinyCLR_Result STM32F4_UsbClient_SetOsExtendedPropertyHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler);


//Startup
void STM32F4_Startup_InitializeRegions();
void STM32F4_Startup_GetHeap(uint8_t*& start, size_t& length);
int32_t STM32F4_Startup_GetLModePin();
TinyCLR_Gpio_PinValue STM32F4_Startup_GetLModeUsbState();

#include "core_cm4.h"
#include "stm32f4xx.h"