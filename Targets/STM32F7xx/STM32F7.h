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

#include <TinyCLR.h>
#include <Device.h>

#include "inc/stm32f7xx.h"

#undef STM32F7

#define SIZEOF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
#define CHARIZE2(c) #c
#define CHARIZE(c) (CHARIZE2(c)[0])

////////////////////////////////////////////////////////////////////////////////
//Deployment
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Deployment_AddApi(const TinyCLR_Api_Manager* apiManager);
void STM32F7_Flash_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
TinyCLR_Result STM32F7_Flash_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_Flash_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_Flash_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result STM32F7_Flash_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result STM32F7_Flash_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result STM32F7_Flash_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result STM32F7_Flash_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result STM32F7_Flash_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result STM32F7_Flash_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result STM32F7_Flash_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_Flash_Close(const TinyCLR_Storage_Controller* self);
const TinyCLR_Startup_DeploymentConfiguration* STM32F7_Flash_GetDeploymentConfiguration();
void STM32F7_Flash_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
void STM32F7_Deplpoyment_Reset();

////////////////////////////////////////////////////////////////////////////////
//Interrupt
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Interrupt_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* STM32F7_Interrupt_GetRequiredApi();
const TinyCLR_Api_Info* STM32F7_Interrupt_GetRequiredApi();
TinyCLR_Result STM32F7_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result STM32F7_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self);
bool STM32F7_Interrupt_Enable(bool force);
bool STM32F7_Interrupt_Disable(bool force);
void STM32F7_Interrupt_WaitForInterrupt();
bool STM32F7_Interrupt_IsDisabled();
void STM32F7_Interrupt_Restore();

////////////////////////////////////////////////////////////////////////////////
//Power
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Power_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* STM32F7_Power_GetRequiredApi();
TinyCLR_Result STM32F7_Power_Initialize(const TinyCLR_Power_Controller* self);
TinyCLR_Result STM32F7_Power_Uninitialize(const TinyCLR_Power_Controller* self);
void STM32F7_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter);
void STM32F7_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level);

////////////////////////////////////////////////////////////////////////////////
//Time
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Time_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* STM32F7_Time_GetRequiredApi();
TinyCLR_Result STM32F7_Time_Initialize(const TinyCLR_NativeTime_Controller* self);
TinyCLR_Result STM32F7_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self);
uint64_t STM32F7_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self);
uint64_t STM32F7_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks);
uint64_t STM32F7_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time);
TinyCLR_Result STM32F7_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback);
TinyCLR_Result STM32F7_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks);
void STM32F7_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
void STM32F7_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime);


////////////////////////////////////////////////////////////////////////////////
//Startup
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Startup_Initialize();
void STM32F7_Startup_GetHeap(uint8_t*& start, size_t& length);
void STM32F7_Startup_GetDebuggerTransportApi(const TinyCLR_Api_Info*& api, const void*& configuration);
void STM32F7_Startup_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
void STM32F7_Startup_GetRunApp(bool& runApp);
void STM32F7_Startup_CacheEnable(void);
void STM32F7_Startup_CacheDisable(void);

////////////////////////////////////////////////////////////////////////////////
//ADC
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Adc_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Adc_Acquire(const TinyCLR_Adc_Controller* self);
TinyCLR_Result STM32F7_Adc_Release(const TinyCLR_Adc_Controller* self);
TinyCLR_Result STM32F7_Adc_OpenChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Adc_CloseChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Adc_ReadChannel(const TinyCLR_Adc_Controller* self, uint32_t channel, int32_t& value);
TinyCLR_Result STM32F7_Adc_SetChannelMode(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);
TinyCLR_Adc_ChannelMode STM32F7_Adc_GetChannelMode(const TinyCLR_Adc_Controller* self);
bool STM32F7_Adc_IsChannelModeSupported(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);
int32_t STM32F7_Adc_GetMinValue(const TinyCLR_Adc_Controller* self);
int32_t STM32F7_Adc_GetMaxValue(const TinyCLR_Adc_Controller* self);
uint32_t STM32F7_Adc_GetResolutionInBits(const TinyCLR_Adc_Controller* self);
uint32_t STM32F7_Adc_GetChannelCount(const TinyCLR_Adc_Controller* self);
void STM32F7_Adc_Reset();

////////////////////////////////////////////////////////////////////////////////
//CAN
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Can_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Can_Acquire(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_Release(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_SoftReset(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_WriteMessage(const TinyCLR_Can_Controller* self, uint32_t arbitrationId, bool isExtendedId, bool isRemoteTransmissionRequest, const uint8_t* data, size_t length);
TinyCLR_Result STM32F7_Can_ReadMessage(const TinyCLR_Can_Controller* self, uint32_t& arbitrationId, bool& isExtendedId, bool& isRemoteTransmissionRequest, uint8_t* data, size_t& length, uint64_t& timestamp);
TinyCLR_Result STM32F7_Can_SetBitTiming(const TinyCLR_Can_Controller* self, uint32_t propagation, uint32_t phase1, uint32_t phase2, uint32_t baudratePrescaler, uint32_t synchronizationJumpWidth, bool useMultiBitSampling);
size_t STM32F7_Can_GetMessagesToRead(const TinyCLR_Can_Controller* self);
size_t STM32F7_Can_GetMessagesToWrite(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_SetMessageReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_MessageReceivedHandler handler);
TinyCLR_Result STM32F7_Can_SetErrorReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_ErrorReceivedHandler handler);
TinyCLR_Result STM32F7_Can_SetExplicitFilters(const TinyCLR_Can_Controller* self, const uint32_t* filters, size_t count);
TinyCLR_Result STM32F7_Can_SetGroupFilters(const TinyCLR_Can_Controller* self, const uint32_t* lowerBounds, const uint32_t* upperBounds, size_t count);
TinyCLR_Result STM32F7_Can_ClearReadBuffer(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_IsWritingAllowed(const TinyCLR_Can_Controller* self, bool& allowed);
size_t STM32F7_Can_GetWriteErrorCount(const TinyCLR_Can_Controller* self);
size_t STM32F7_Can_GetReadErrorCount(const TinyCLR_Can_Controller* self);
uint32_t STM32F7_Can_GetSourceClock(const TinyCLR_Can_Controller* self);
size_t STM32F7_Can_GetReadBufferSize(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_SetReadBufferSize(const TinyCLR_Can_Controller* self, size_t size);
size_t STM32F7_Can_GetWriteBufferSize(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_SetWriteBufferSize(const TinyCLR_Can_Controller* self, size_t size);
TinyCLR_Result STM32F7_Can_Enable(const TinyCLR_Can_Controller* self);
TinyCLR_Result STM32F7_Can_Disable(const TinyCLR_Can_Controller* self);
bool STM32F7_Can_CanWriteMessage(const TinyCLR_Can_Controller* self);
bool STM32F7_Can_CanReadMessage(const TinyCLR_Can_Controller* self);
void STM32F7_Can_Reset();
////////////////////////////////////////////////////////////////////////////////
//DAC
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Dac_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Dac_Acquire(const TinyCLR_Dac_Controller* self);
TinyCLR_Result STM32F7_Dac_Release(const TinyCLR_Dac_Controller* self);
TinyCLR_Result STM32F7_Dac_OpenChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Dac_CloseChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Dac_WriteValue(const TinyCLR_Dac_Controller* self, uint32_t channel, int32_t value);
int32_t STM32F7_Dac_GetMinValue(const TinyCLR_Dac_Controller* self);
int32_t STM32F7_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self);
uint32_t STM32F7_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self);
uint32_t STM32F7_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self);
void STM32F7_Dac_Reset();

////////////////////////////////////////////////////////////////////////////////
//GPIO
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* STM32F7_Gpio_GetRequiredApi();
TinyCLR_Result STM32F7_Gpio_Acquire(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result STM32F7_Gpio_Release(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result STM32F7_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
TinyCLR_Result STM32F7_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
TinyCLR_Result STM32F7_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result STM32F7_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value);
bool STM32F7_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode STM32F7_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin);
TinyCLR_Result STM32F7_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
uint64_t STM32F7_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin);
TinyCLR_Result STM32F7_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks);
TinyCLR_Result STM32F7_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler);
uint32_t STM32F7_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self);

////////////////////////////////////////////////////////////////////////////////
//I2C
////////////////////////////////////////////////////////////////////////////////
void STM32F7_I2c_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_I2c_Acquire(const TinyCLR_I2c_Controller* self);
TinyCLR_Result STM32F7_I2c_Release(const TinyCLR_I2c_Controller* self);
TinyCLR_Result STM32F7_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, uint32_t slaveAddress, TinyCLR_I2c_AddressFormat addressFormat, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result STM32F7_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error);
void STM32F7_I2c_Reset();

////////////////////////////////////////////////////////////////////////////////
//PWM
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Pwm_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Pwm_Acquire(const TinyCLR_Pwm_Controller* self);
TinyCLR_Result STM32F7_Pwm_Release(const TinyCLR_Pwm_Controller* self);
TinyCLR_Result STM32F7_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result STM32F7_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity);
TinyCLR_Result STM32F7_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency);
double STM32F7_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self);
double STM32F7_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self);
double STM32F7_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self);
uint32_t STM32F7_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self);
void STM32F7_Pwm_Reset();

////////////////////////////////////////////////////////////////////////////////
//RTC
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Rtc_Acquire(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result STM32F7_Rtc_Release(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result STM32F7_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value);
TinyCLR_Result STM32F7_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value);

////////////////////////////////////////////////////////////////////////////////
//SD
////////////////////////////////////////////////////////////////////////////////
void STM32F7_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager);

TinyCLR_Result STM32F7_SdCard_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_SdCard_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result STM32F7_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result STM32F7_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result STM32F7_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result STM32F7_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result STM32F7_SdCard_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_SdCard_Close(const TinyCLR_Storage_Controller* self);
TinyCLR_Result STM32F7_SdCard_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result STM32F7_SdCard_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);

TinyCLR_Result STM32F7_SdCard_Reset();

////////////////////////////////////////////////////////////////////////////////
//SPI
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Spi_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Spi_Acquire(const TinyCLR_Spi_Controller* self);
TinyCLR_Result STM32F7_Spi_Release(const TinyCLR_Spi_Controller* self);
TinyCLR_Result STM32F7_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, uint32_t chipSelectLine, bool useControllerChipSelect, uint32_t clockFrequency, uint32_t dataBitLength, TinyCLR_Spi_Mode mode);
TinyCLR_Result STM32F7_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F7_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F7_Spi_WriteRead(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
TinyCLR_Result STM32F7_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
uint32_t STM32F7_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self);
uint32_t STM32F7_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self);
uint32_t STM32F7_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self);
TinyCLR_Result STM32F7_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, uint32_t* dataBitLengths, size_t& dataBitLengthsCount);
void STM32F7_Spi_Reset();

////////////////////////////////////////////////////////////////////////////////
//UART
////////////////////////////////////////////////////////////////////////////////
void STM32F7_Uart_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* STM32F7_Uart_GetRequiredApi();
TinyCLR_Result STM32F7_Uart_Acquire(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_Release(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_Enable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_Disable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking);
TinyCLR_Result STM32F7_Uart_Flush(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F7_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result STM32F7_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result STM32F7_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result STM32F7_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result STM32F7_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler);
TinyCLR_Result STM32F7_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result STM32F7_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool state);
size_t STM32F7_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t STM32F7_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t STM32F7_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self);
size_t STM32F7_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self);
TinyCLR_Result STM32F7_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self);
void STM32F7_Uart_Reset();

////////////////////////////////////////////////////////////////////////////////
//USB Client
////////////////////////////////////////////////////////////////////////////////
void STM32F7_UsbDevice_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* STM32F7_UsbDevice_GetRequiredApi();
void STM32F7_UsbDevice_Reset();

struct USB_PACKET64;
struct UsClientState;
typedef void(*USB_NEXT_CALLBACK)(UsClientState*);

void TinyCLR_UsbClient_ClearEvent(UsClientState *usClientState, uint32_t event);
void TinyCLR_UsbClient_ClearEndpoints(UsClientState *usClientState, int32_t endpoint);
USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(UsClientState* usClientState, int32_t endpoint, bool& disableRx);
USB_PACKET64* TinyCLR_UsbClient_TxDequeue(UsClientState* usClientState, int32_t endpoint);
void TinyCLR_UsbClient_StateCallback(UsClientState* usClientState);
uint8_t TinyCLR_UsbClient_ControlCallback(UsClientState* usClientState);
bool TinyCLR_UsbClient_CanReceivePackage(UsClientState* usClientState, int32_t endpoint);

////////////////////////////////////////////////////////////////////////////////
//Interrupt Internal
////////////////////////////////////////////////////////////////////////////////
class STM32F7_DisableInterrupts_RaiiHelper {
    uint32_t state;

public:
    STM32F7_DisableInterrupts_RaiiHelper();
    ~STM32F7_DisableInterrupts_RaiiHelper();

    bool IsDisabled();
    void Acquire();
    void Release();
};

class STM32F7_InterruptStarted_RaiiHelper {
public:
    STM32F7_InterruptStarted_RaiiHelper();
    ~STM32F7_InterruptStarted_RaiiHelper();
};

#define DISABLE_INTERRUPTS_SCOPED(name) STM32F7_DisableInterrupts_RaiiHelper name
#define INTERRUPT_STARTED_SCOPED(name) STM32F7_InterruptStarted_RaiiHelper name

bool STM32F7_InterruptInternal_Activate(uint32_t index, uint32_t* isr, void* isrParam);
bool STM32F7_InterruptInternal_Deactivate(uint32_t index);

////////////////////////////////////////////////////////////////////////////////
//GPIO Internal
////////////////////////////////////////////////////////////////////////////////
enum class STM32F7_Gpio_PullDirection : uint8_t {
    None = 0,
    PullUp = 1,
    PullDown = 2,
    Reserved = 3
};

enum class STM32F7_Gpio_PortMode : uint8_t {
    Input = 0,
    GeneralPurposeOutput = 1,
    AlternateFunction = 2,
    Analog = 3
};

enum class STM32F7_Gpio_OutputSpeed : uint8_t {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3
};

enum class STM32F7_Gpio_AlternateFunction : uint8_t {
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

enum class STM32F7_Gpio_OutputType : uint8_t {
    PushPull = 0,
    OpenDrain = 1,
};

struct STM32F7_Gpio_Pin {
    uint32_t number;
    STM32F7_Gpio_AlternateFunction alternateFunction;
};

struct STM32F7_Gpio_PinConfiguration {
    STM32F7_Gpio_PortMode portMode;
    STM32F7_Gpio_OutputType outputType;
    STM32F7_Gpio_OutputSpeed outputSpeed;
    STM32F7_Gpio_PullDirection pullDirection;
    STM32F7_Gpio_AlternateFunction alternateFunction;
    bool outputDirection;
    bool apply;
};

#define PIN(port, pin) ((CHARIZE(port) - 'A') * 16 + pin)
#define PIN_NONE 0xFFFFFFFF
#define AF(num) (CONCAT(STM32F7_Gpio_AlternateFunction::AF, num))
#define AF_NONE STM32F7_Gpio_AlternateFunction::AF0

#define INIT(portMode, outputType, outputSpeed, outputDirection, pullDirection, alternateFunction, apply) { STM32F7_Gpio_PortMode::portMode, STM32F7_Gpio_OutputType::outputType, STM32F7_Gpio_OutputSpeed::outputSpeed, STM32F7_Gpio_PullDirection::pullDirection, STM32F7_Gpio_AlternateFunction::alternateFunction, outputDirection, apply }
#define ALTFUN(outputType, outputSpeed, pullDirection, alternateFunction) INIT(AlternateFunction, outputType, outputSpeed, false, pullDirection, alternateFunction, true)
#define ANALOG() INIT(Analog, PushPull, VeryHigh, false, None, AF0, true)
#define OUTPUT(outputType, outputSpeed, outputDirection, pullDirection) INIT(GeneralPurposeOutput, outputType, outputSpeed, outputDirection, pullDirection, AF0, true)
#define INPUT(pullDirection) INIT(Input, PushPull, VeryHigh, false, pullDirection, AF0, true)
#define DEFAULT() INIT(Input, PushPull, VeryHigh, false, PullDown, AF0, true)
#define NO_INIT() INIT(Input, PushPull, VeryHigh, false, PullDown, AF0, false)

bool STM32F7_GpioInternal_OpenPin(int32_t pin);
bool STM32F7_GpioInternal_ClosePin(int32_t pin);
bool STM32F7_GpioInternal_ReadPin(int32_t pin);
void STM32F7_GpioInternal_WritePin(int32_t pin, bool value);
bool STM32F7_GpioInternal_ConfigurePin(int32_t pin, STM32F7_Gpio_PortMode portMode, STM32F7_Gpio_OutputType outputType, STM32F7_Gpio_OutputSpeed outputSpeed, STM32F7_Gpio_PullDirection pullDirection, STM32F7_Gpio_AlternateFunction alternateFunction);
void STM32F7_Gpio_Reset();

void STM32F7_Display_Reset();
void STM32F7_Display_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result STM32F7_Display_Acquire(const TinyCLR_Display_Controller* self);
TinyCLR_Result STM32F7_Display_Release(const TinyCLR_Display_Controller* self);
TinyCLR_Result STM32F7_Display_Enable(const TinyCLR_Display_Controller* self);
TinyCLR_Result STM32F7_Display_Disable(const TinyCLR_Display_Controller* self);
TinyCLR_Result STM32F7_Display_GetCapabilities(const TinyCLR_Display_Controller* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount);
TinyCLR_Result STM32F7_Display_GetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration);
TinyCLR_Result STM32F7_Display_SetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration);
TinyCLR_Result STM32F7_Display_DrawBuffer(const TinyCLR_Display_Controller* self, int32_t x, uint32_t y, int32_t width, int32_t height, const uint8_t* data);
TinyCLR_Result STM32F7_Display_DrawPixel(const TinyCLR_Display_Controller* self, uint32_t x, uint32_t y, uint64_t color);
TinyCLR_Result STM32F7_Display_WriteString(const TinyCLR_Display_Controller* self, const char* buffer, size_t length);

void STM32F7_Startup_OnSoftReset(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);
void STM32F7_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);

extern const TinyCLR_Api_Manager* apiManager;