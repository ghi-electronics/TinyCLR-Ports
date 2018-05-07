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

#include "inc/stm32f4xx.h"

#undef STM32F4

#define SIZEOF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
#define CHARIZE2(c) #c
#define CHARIZE(c) (CHARIZE2(c)[0])

////////////////////////////////////////////////////////////////////////////////
//Deployment
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Deployment_GetApi();
TinyCLR_Result STM32F4_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool& supportsXip);
TinyCLR_Result STM32F4_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result STM32F4_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result STM32F4_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result STM32F4_Flash_EraseSector(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result STM32F4_Flash_IsSectorErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool& erased);
TinyCLR_Result STM32F4_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
void STM32F4_Deplpoyment_Reset();

////////////////////////////////////////////////////////////////////////////////
//Interrupt
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Interrupt_GetApi();
TinyCLR_Result STM32F4_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result STM32F4_Interrupt_Release();
bool STM32F4_Interrupt_Enable(bool force);
bool STM32F4_Interrupt_Disable(bool force);
void STM32F4_Interrupt_WaitForInterrupt();
bool STM32F4_Interrupt_IsDisabled();
void STM32F4_Interrupt_Restore();

////////////////////////////////////////////////////////////////////////////////
//Power
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Power_GetApi();
TinyCLR_Result STM32F4_Power_Acquire(const TinyCLR_Power_Provider* self);
TinyCLR_Result STM32F4_Power_Release(const TinyCLR_Power_Provider* self);
void STM32F4_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter);
void STM32F4_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_SleepLevel level);

////////////////////////////////////////////////////////////////////////////////
//Time
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Time_GetApi();
TinyCLR_Result STM32F4_Time_Acquire(const TinyCLR_NativeTime_Provider* self);
TinyCLR_Result STM32F4_Time_Release(const TinyCLR_NativeTime_Provider* self);
uint64_t STM32F4_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Provider* self);
uint64_t STM32F4_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Provider* self, uint64_t ticks);
uint64_t STM32F4_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Provider* self, uint64_t time);
TinyCLR_Result STM32F4_Time_SetTickCallback(const TinyCLR_NativeTime_Provider* self, TinyCLR_NativeTime_Callback callback);
TinyCLR_Result STM32F4_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Provider* self, uint64_t processorTicks);
void STM32F4_Time_Delay(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds);

////////////////////////////////////////////////////////////////////////////////
//Startup
////////////////////////////////////////////////////////////////////////////////
void STM32F4_Startup_Initialize();
void STM32F4_Startup_GetHeap(uint8_t*& start, size_t& length);
void STM32F4_Startup_GetDebuggerTransportProvider(const TinyCLR_Api_Info*& api, size_t& index, const void*& configuration);
void STM32F4_Startup_GetRunApp(bool& runApp);

////////////////////////////////////////////////////////////////////////////////
//ADC
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Adc_GetApi();
TinyCLR_Result STM32F4_Adc_Acquire(const TinyCLR_Adc_Provider* self);
TinyCLR_Result STM32F4_Adc_Release(const TinyCLR_Adc_Provider* self);
TinyCLR_Result STM32F4_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value);
TinyCLR_Result STM32F4_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);
TinyCLR_Adc_ChannelMode STM32F4_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self);
bool STM32F4_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);
int32_t STM32F4_Adc_GetMinValue(const TinyCLR_Adc_Provider* self);
int32_t STM32F4_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self);
int32_t STM32F4_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self);
int32_t STM32F4_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self);
void STM32F4_Adc_Reset();

////////////////////////////////////////////////////////////////////////////////
//CAN
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Can_GetApi();
TinyCLR_Result STM32F4_Can_Acquire(const TinyCLR_Can_Provider* self);
TinyCLR_Result STM32F4_Can_Release(const TinyCLR_Can_Provider* self);
TinyCLR_Result STM32F4_Can_SoftReset(const TinyCLR_Can_Provider* self);
TinyCLR_Result STM32F4_Can_WriteMessage(const TinyCLR_Can_Provider* self, uint32_t arbitrationId, bool isExtendedId, bool isRemoteTransmissionRequest, uint8_t* data, size_t length);
TinyCLR_Result STM32F4_Can_ReadMessage(const TinyCLR_Can_Provider* self, uint32_t& arbitrationId, bool& isExtendedId, bool& isRemoteTransmissionRequest, uint64_t& timestamp, uint8_t* data, size_t& length);
TinyCLR_Result STM32F4_Can_SetBitTiming(const TinyCLR_Can_Provider* self, int32_t propagation, int32_t phase1, int32_t phase2, int32_t baudratePrescaler, int32_t synchronizationJumpWidth, int8_t useMultiBitSampling);
TinyCLR_Result STM32F4_Can_GetUnreadMessageCount(const TinyCLR_Can_Provider* self, size_t& count);
TinyCLR_Result STM32F4_Can_SetMessageReceivedHandler(const TinyCLR_Can_Provider* self, TinyCLR_Can_MessageReceivedHandler handler, const TinyCLR_Interop_Provider* interopProvider);
TinyCLR_Result STM32F4_Can_SetErrorReceivedHandler(const TinyCLR_Can_Provider* self, TinyCLR_Can_ErrorReceivedHandler handler, const TinyCLR_Interop_Provider* interopProvider);
TinyCLR_Result STM32F4_Can_SetExplicitFilters(const TinyCLR_Can_Provider* self, uint8_t* filters, size_t length);
TinyCLR_Result STM32F4_Can_SetGroupFilters(const TinyCLR_Can_Provider* self, uint8_t* lowerBounds, uint8_t* upperBounds, size_t length);
TinyCLR_Result STM32F4_Can_ClearReadBuffer(const TinyCLR_Can_Provider* self);
TinyCLR_Result STM32F4_Can_IsWritingAllowed(const TinyCLR_Can_Provider* self, bool& allowed);
TinyCLR_Result STM32F4_Can_GetWriteErrorCount(const TinyCLR_Can_Provider* self, size_t& count);
TinyCLR_Result STM32F4_Can_GetReadErrorCount(const TinyCLR_Can_Provider* self, size_t& count);
TinyCLR_Result STM32F4_Can_GetSourceClock(const TinyCLR_Can_Provider* self, uint32_t& sourceClock);
TinyCLR_Result STM32F4_Can_GetReadBufferSize(const TinyCLR_Can_Provider* self, size_t& size);
TinyCLR_Result STM32F4_Can_SetReadBufferSize(const TinyCLR_Can_Provider* self, size_t size);
TinyCLR_Result STM32F4_Can_GetWriteBufferSize(const TinyCLR_Can_Provider* self, size_t& size);
TinyCLR_Result STM32F4_Can_SetWriteBufferSize(const TinyCLR_Can_Provider* self, size_t size);
void STM32F4_Can_Reset();

////////////////////////////////////////////////////////////////////////////////
//DAC
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Dac_GetApi();
TinyCLR_Result STM32F4_Dac_Acquire(const TinyCLR_Dac_Provider* self);
TinyCLR_Result STM32F4_Dac_Release(const TinyCLR_Dac_Provider* self);
TinyCLR_Result STM32F4_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result STM32F4_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value);
int32_t STM32F4_Dac_GetMinValue(const TinyCLR_Dac_Provider* self);
int32_t STM32F4_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self);
int32_t STM32F4_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self);
int32_t STM32F4_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self);
void STM32F4_Dac_Reset();

////////////////////////////////////////////////////////////////////////////////
//GPIO
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Gpio_GetApi();
TinyCLR_Result STM32F4_Gpio_Acquire(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result STM32F4_Gpio_Release(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result STM32F4_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result STM32F4_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue value);
bool STM32F4_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode STM32F4_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
int32_t STM32F4_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, int32_t debounceTime);
TinyCLR_Result STM32F4_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler handler);
int32_t STM32F4_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self);
void STM32F4_Gpio_Reset();

////////////////////////////////////////////////////////////////////////////////
//I2C
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_I2c_GetApi();
TinyCLR_Result STM32F4_I2c_Acquire(const TinyCLR_I2c_Provider* self);
TinyCLR_Result STM32F4_I2c_Release(const TinyCLR_I2c_Provider* self);
TinyCLR_Result STM32F4_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result STM32F4_I2c_Read(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result STM32F4_I2c_Write(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result STM32F4_I2c_WriteRead(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result);
void STM32F4_I2c_Reset();

////////////////////////////////////////////////////////////////////////////////
//PWM
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Pwm_GetApi();
TinyCLR_Result STM32F4_Pwm_Acquire(const TinyCLR_Pwm_Provider* self);
TinyCLR_Result STM32F4_Pwm_Release(const TinyCLR_Pwm_Provider* self);
TinyCLR_Result STM32F4_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result STM32F4_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity);
TinyCLR_Result STM32F4_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency);
double STM32F4_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self);
double STM32F4_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self);
double STM32F4_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self);
int32_t STM32F4_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self);
void STM32F4_Pwm_Reset();

////////////////////////////////////////////////////////////////////////////////
//RTC
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Rtc_GetApi();
TinyCLR_Result STM32F4_Rtc_Acquire(const TinyCLR_Rtc_Provider* self);
TinyCLR_Result STM32F4_Rtc_Release(const TinyCLR_Rtc_Provider* self);
TinyCLR_Result STM32F4_Rtc_GetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime& value);
TinyCLR_Result STM32F4_Rtc_SetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime value);

////////////////////////////////////////////////////////////////////////////////
//SPI
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Spi_GetApi();
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
void STM32F4_Spi_Reset();

////////////////////////////////////////////////////////////////////////////////
//UART
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_Uart_GetApi();
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
TinyCLR_Result STM32F4_Uart_GetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t& size);
TinyCLR_Result STM32F4_Uart_SetReadBufferSize(const TinyCLR_Uart_Provider* self, size_t size);
TinyCLR_Result STM32F4_Uart_GetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t& size);
TinyCLR_Result STM32F4_Uart_SetWriteBufferSize(const TinyCLR_Uart_Provider* self, size_t size);
TinyCLR_Result STM32F4_Uart_GetUnreadCount(const TinyCLR_Uart_Provider* self, size_t& count);
TinyCLR_Result STM32F4_Uart_GetUnwrittenCount(const TinyCLR_Uart_Provider* self, size_t& count);
TinyCLR_Result STM32F4_Uart_ClearReadBuffer(const TinyCLR_Uart_Provider* self);
TinyCLR_Result STM32F4_Uart_ClearWriteBuffer(const TinyCLR_Uart_Provider* self);
void STM32F4_Uart_Reset();

////////////////////////////////////////////////////////////////////////////////
//USB Client
////////////////////////////////////////////////////////////////////////////////
const TinyCLR_Api_Info* STM32F4_UsbClient_GetApi();
void STM32F4_UsbClient_Reset();

struct USB_PACKET64;
struct USB_CONTROLLER_STATE;
typedef void(*USB_NEXT_CALLBACK)(USB_CONTROLLER_STATE*);

void TinyCLR_UsbClient_ClearEvent(USB_CONTROLLER_STATE *usbState, uint32_t event);
void TinyCLR_UsbClient_ClearEndpoints(USB_CONTROLLER_STATE *usbState, int32_t endpoint);
USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(USB_CONTROLLER_STATE* usbState, int32_t endpoint, bool& disableRx);
USB_PACKET64* TinyCLR_UsbClient_TxDequeue(USB_CONTROLLER_STATE* usbState, int32_t endpoint);
void TinyCLR_UsbClient_StateCallback(USB_CONTROLLER_STATE* usbState);
uint8_t TinyCLR_UsbClient_ControlCallback(USB_CONTROLLER_STATE* usbState);
bool TinyCLR_UsbClient_CanReceivePackage(USB_CONTROLLER_STATE* usbState, int32_t endpoint);

////////////////////////////////////////////////////////////////////////////////
//Interrupt Internal
////////////////////////////////////////////////////////////////////////////////
class STM32F4_DisableInterrupts_RaiiHelper {
    uint32_t state;

public:
    STM32F4_DisableInterrupts_RaiiHelper();
    ~STM32F4_DisableInterrupts_RaiiHelper();

    bool IsDisabled();
    void Acquire();
    void Release();
};

class STM32F4_InterruptStarted_RaiiHelper {
public:
    STM32F4_InterruptStarted_RaiiHelper();
    ~STM32F4_InterruptStarted_RaiiHelper();
};

#define DISABLE_INTERRUPTS_SCOPED(name) STM32F4_DisableInterrupts_RaiiHelper name
#define INTERRUPT_STARTED_SCOPED(name) STM32F4_InterruptStarted_RaiiHelper name

bool STM32F4_InterruptInternal_Activate(uint32_t index, uint32_t* isr, void* isrParam);
bool STM32F4_InterruptInternal_Deactivate(uint32_t index);

////////////////////////////////////////////////////////////////////////////////
//GPIO Internal
////////////////////////////////////////////////////////////////////////////////
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
    High = 2,
    VeryHigh = 3
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

struct STM32F4_Gpio_Pin {
    uint32_t number;
    STM32F4_Gpio_AlternateFunction alternateFunction;
};

struct STM32F4_Gpio_PinConfiguration {
    STM32F4_Gpio_PortMode portMode;
    STM32F4_Gpio_OutputType outputType;
    STM32F4_Gpio_OutputSpeed outputSpeed;
    STM32F4_Gpio_PullDirection pullDirection;
    STM32F4_Gpio_AlternateFunction alternateFunction;
    bool outputDirection;
    bool apply;
};

#define PIN(port, pin) ((CHARIZE(port) - 'A') * 16 + pin)
#define PIN_NONE 0xFFFFFFFF
#define AF(num) (CONCAT(STM32F4_Gpio_AlternateFunction::AF, num))
#define AF_NONE STM32F4_Gpio_AlternateFunction::AF0

#define INIT(portMode, outputType, outputSpeed, outputDirection, pullDirection, alternateFunction, apply) { STM32F4_Gpio_PortMode::portMode, STM32F4_Gpio_OutputType::outputType, STM32F4_Gpio_OutputSpeed::outputSpeed, STM32F4_Gpio_PullDirection::pullDirection, STM32F4_Gpio_AlternateFunction::alternateFunction, outputDirection, apply }
#define ALTFUN(outputType, outputSpeed, pullDirection, alternateFunction) INIT(AlternateFunction, outputType, outputSpeed, false, pullDirection, alternateFunction, true)
#define ANALOG() INIT(Analog, PushPull, VeryHigh, false, None, AF0, true)
#define OUTPUT(outputType, outputSpeed, outputDirection, pullDirection) INIT(GeneralPurposeOutput, outputType, outputSpeed, outputDirection, pullDirection, AF0, true)
#define INPUT(pullDirection) INIT(Input, PushPull, VeryHigh, false, pullDirection, AF0, true)
#define DEFAULT() INIT(Input, PushPull, VeryHigh, false, PullDown, AF0, true)
#define NO_INIT() INIT(Input, PushPull, VeryHigh, false, PullDown, AF0, false)

bool STM32F4_GpioInternal_OpenPin(int32_t pin);
bool STM32F4_GpioInternal_ClosePin(int32_t pin);
bool STM32F4_GpioInternal_ReadPin(int32_t pin);
void STM32F4_GpioInternal_WritePin(int32_t pin, bool value);
bool STM32F4_GpioInternal_ConfigurePin(int32_t pin, STM32F4_Gpio_PortMode portMode, STM32F4_Gpio_OutputType outputType, STM32F4_Gpio_OutputSpeed outputSpeed, STM32F4_Gpio_PullDirection pullDirection, STM32F4_Gpio_AlternateFunction alternateFunction);

void STM32F4_Display_Reset();
const TinyCLR_Api_Info* STM32F4_Display_GetApi();
TinyCLR_Result STM32F4_Display_Acquire(const TinyCLR_Display_Provider* self);
TinyCLR_Result STM32F4_Display_Release(const TinyCLR_Display_Provider* self);
TinyCLR_Result STM32F4_Display_Enable(const TinyCLR_Display_Provider* self);
TinyCLR_Result STM32F4_Display_Disable(const TinyCLR_Display_Provider* self);
TinyCLR_Result STM32F4_Display_GetCapabilities(const TinyCLR_Display_Provider* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount);
TinyCLR_Result STM32F4_Display_GetConfiguration(const TinyCLR_Display_Provider* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration);
TinyCLR_Result STM32F4_Display_SetConfiguration(const TinyCLR_Display_Provider* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration);
TinyCLR_Result STM32F4_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data);
TinyCLR_Result STM32F4_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer, size_t length);

void STM32F4_Startup_OnSoftReset(const TinyCLR_Api_Provider* apiProvider);
void STM32F4_Startup_OnSoftResetDevice(const TinyCLR_Api_Provider* apiProvider);

extern const TinyCLR_Api_Provider* apiProvider;