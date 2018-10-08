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

#if defined(LPC177x_8x)
#include <inc\LPC177x_8x.h>
#endif

#define SIZEOF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
#define CHARIZE2(c) #c
#define CHARIZE(c) (CHARIZE2(c)[0])

// ADC
void LPC17_Adc_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC17_Adc_Reset();
TinyCLR_Result LPC17_Adc_Acquire(const TinyCLR_Adc_Controller* self);
TinyCLR_Result LPC17_Adc_Release(const TinyCLR_Adc_Controller* self);
TinyCLR_Result LPC17_Adc_OpenChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Adc_CloseChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Adc_ReadChannel(const TinyCLR_Adc_Controller* self, uint32_t channel, int32_t& value);
uint32_t LPC17_Adc_GetChannelCount(const TinyCLR_Adc_Controller* self);
uint32_t LPC17_Adc_GetResolutionInBits(const TinyCLR_Adc_Controller* self);
int32_t LPC17_Adc_GetMinValue(const TinyCLR_Adc_Controller* self);
int32_t LPC17_Adc_GetMaxValue(const TinyCLR_Adc_Controller* self);
TinyCLR_Adc_ChannelMode LPC17_Adc_GetChannelMode(const TinyCLR_Adc_Controller* self);
TinyCLR_Result LPC17_Adc_SetChannelMode(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);
bool LPC17_Adc_IsChannelModeSupported(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);

// CAN
void LPC17_Can_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result LPC17_Can_Acquire(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_Release(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_SoftReset(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_WriteMessage(const TinyCLR_Can_Controller* self, const TinyCLR_Can_Message* messages, size_t& length);
TinyCLR_Result LPC17_Can_ReadMessage(const TinyCLR_Can_Controller* self, TinyCLR_Can_Message* messages, size_t& length);
TinyCLR_Result LPC17_Can_SetBitTiming(const TinyCLR_Can_Controller* self, const TinyCLR_Can_BitTiming* timing);
size_t LPC17_Can_GetMessagesToRead(const TinyCLR_Can_Controller* self);
size_t LP17_Can_GetMessagesToWrite(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_SetMessageReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_MessageReceivedHandler handler);
TinyCLR_Result LPC17_Can_SetErrorReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_ErrorReceivedHandler handler);
TinyCLR_Result LPC17_Can_SetExplicitFilters(const TinyCLR_Can_Controller* self, const uint32_t* filters, size_t count);
TinyCLR_Result LPC17_Can_SetGroupFilters(const TinyCLR_Can_Controller* self, const uint32_t* lowerBounds, const uint32_t* upperBounds, size_t count);
TinyCLR_Result LPC17_Can_ClearReadBuffer(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_IsWritingAllowed(const TinyCLR_Can_Controller* self, bool& allowed);
size_t LPC17_Can_GetWriteErrorCount(const TinyCLR_Can_Controller* self);
size_t LPC17_Can_GetReadErrorCount(const TinyCLR_Can_Controller* self);
uint32_t LPC17_Can_GetSourceClock(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_SetReadBufferSize(const TinyCLR_Can_Controller* self, size_t size);
size_t LPC17_Can_GetReadBufferSize(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_SetReadBufferSize(const TinyCLR_Can_Controller* self, size_t size);
size_t LPC17_Can_GetWriteBufferSize(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_SetWriteBufferSize(const TinyCLR_Can_Controller* self, size_t size);
TinyCLR_Result LPC17_Can_Enable(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC17_Can_Disable(const TinyCLR_Can_Controller* self);
bool LPC17_Can_CanWriteMessage(const TinyCLR_Can_Controller* self);
bool LPC17_Can_CanReadMessage(const TinyCLR_Can_Controller* self);
void LPC17_Can_Reset();

//DAC
void LPC17_Dac_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC17_Dac_Reset();
TinyCLR_Result LPC17_Dac_Acquire(const TinyCLR_Dac_Controller* self);
TinyCLR_Result LPC17_Dac_Release(const TinyCLR_Dac_Controller* self);
TinyCLR_Result LPC17_Dac_OpenChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Dac_CloseChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Dac_WriteValue(const TinyCLR_Dac_Controller* self, uint32_t channel, int32_t value);
uint32_t LPC17_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self);
uint32_t LPC17_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self);
int32_t LPC17_Dac_GetMinValue(const TinyCLR_Dac_Controller* self);
int32_t LPC17_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self);

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
    bool outputDirection;
    bool apply;
};

#define PIN(port, pin) (port * 32 + pin)
#define PIN_NONE 0xFFFFFFFF
#define PF(num) (CONCAT(LPC17_Gpio_PinFunction::PinFunction, num))
#define PF_NONE LPC17_Gpio_PinFunction::PinFunction0

#define INIT(direction, resistorMode, hysteresis, inputPolarity, slewRate, outputType, pinFunction, outputDirection, apply) { LPC17_Gpio_Direction::direction, LPC17_Gpio_ResistorMode::resistorMode, LPC17_Gpio_Hysteresis::hysteresis, LPC17_Gpio_InputPolarity::inputPolarity, LPC17_Gpio_SlewRate::slewRate, LPC17_Gpio_OutputType::outputType,LPC17_Gpio_PinFunction::pinFunction, outputDirection, apply }
#define ALTFUN(direction, resistorMode, outputType, pinFunction) { LPC17_Gpio_Direction::direction, LPC17_Gpio_ResistorMode::resistorMode, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::outputType,LPC17_Gpio_PinFunction::pinFunction, true }
#define INPUT(outputType, resistorMode) { LPC17_Gpio_Direction::Input, LPC17_Gpio_ResistorMode::resistorMode, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::outputType,LPC17_Gpio_PinFunction::PinFunction0, true }
#define DEFAULT() INIT(Input, Inactive, Disable, NotInverted, StandardMode, PushPull, PinFunction0, false, true)
#define NO_INIT() INIT(Input, Inactive, Disable, NotInverted, StandardMode, PushPull, PinFunction0, false, false)

void LPC17_Gpio_Reset();
void LPC17_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC17_Gpio_GetRequiredApi();
TinyCLR_Result LPC17_Gpio_Acquire(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result LPC17_Gpio_Release(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result LPC17_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t alternate);
TinyCLR_Result LPC17_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result LPC17_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result LPC17_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks);
TinyCLR_Result LPC17_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result LPC17_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
bool LPC17_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode LPC17_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin);
uint64_t LPC17_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin);
uint32_t LPC17_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result LPC17_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler);
TinyCLR_Result LPC17_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin);

bool LPC17_GpioInternal_OpenPin(int32_t pin);
bool LPC17_GpioInternal_ClosePin(int32_t pin);
bool LPC17_GpioInternal_ConfigurePin(int32_t pin, LPC17_Gpio_Direction pinDir, LPC17_Gpio_PinFunction pinFunction, LPC17_Gpio_ResistorMode pullResistor, LPC17_Gpio_Hysteresis hysteresis, LPC17_Gpio_InputPolarity inputPolarity, LPC17_Gpio_SlewRate slewRate, LPC17_Gpio_OutputType outputType);
void LPC17_GpioInternal_EnableOutputPin(int32_t pin, bool initialState);
void LPC17_GpioInternal_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor);

struct PwmState {
    int32_t controllerIndex;

    int32_t                         channel[MAX_PWM_PER_CONTROLLER];
    int32_t                         match[MAX_PWM_PER_CONTROLLER];

    LPC17_Gpio_Pin                  gpioPin[MAX_PWM_PER_CONTROLLER];

    uint32_t                        outputEnabled[MAX_PWM_PER_CONTROLLER];
    uint32_t                        *matchAddress[MAX_PWM_PER_CONTROLLER];

    TinyCLR_Pwm_PulsePolarity invert[MAX_PWM_PER_CONTROLLER];
    bool                            isOpened[MAX_PWM_PER_CONTROLLER];

    double                          frequency;
    double                          dutyCycle[MAX_PWM_PER_CONTROLLER];

    uint16_t initializeCount;
};

void LPC17_Pwm_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC17_Pwm_Reset();
void LPC17_Pwm_ResetController(int32_t controllerIndex);
TinyCLR_Result LPC17_Pwm_Acquire(const TinyCLR_Pwm_Controller* self);
TinyCLR_Result LPC17_Pwm_Release(const TinyCLR_Pwm_Controller* self);
uint32_t LPC17_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency);
TinyCLR_Result LPC17_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC17_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity);
double LPC17_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self);
double LPC17_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self);
double LPC17_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self);
uint32_t LPC17_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self);
LPC17_Gpio_Pin LPC17_Pwm_GetPins(int32_t controllerIndex, int32_t channel);

////////////////////////////////////////////////////////////////////////////////
//RTC
////////////////////////////////////////////////////////////////////////////////
void LPC17_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result LPC17_Rtc_Acquire(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result LPC17_Rtc_Release(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result LPC17_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value);
TinyCLR_Result LPC17_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value);
TinyCLR_Result LPC17_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value);

////////////////////////////////////////////////////////////////////////////////
//SD
////////////////////////////////////////////////////////////////////////////////
void LPC17_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager);

TinyCLR_Result LPC17_SdCard_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC17_SdCard_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC17_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC17_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC17_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result LPC17_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result LPC17_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result LPC17_SdCard_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result LPC17_SdCard_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result LPC17_SdCard_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC17_SdCard_Close(const TinyCLR_Storage_Controller* self);

TinyCLR_Result LPC17_SdCard_Reset();

////////////////////////////////////////////////////////////////////////////////
//SPI
////////////////////////////////////////////////////////////////////////////////
void LPC17_Spi_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC17_Spi_GetRequiredApi();
void LPC17_Spi_Reset();
bool LPC17_Spi_Transaction_Start(int32_t controllerIndex);
bool LPC17_Spi_Transaction_Stop(int32_t controllerIndex);
bool LPC17_Spi_Transaction_nWrite8_nRead8(int32_t controllerIndex);
TinyCLR_Result LPC17_Spi_Acquire(const TinyCLR_Spi_Controller* self);
TinyCLR_Result LPC17_Spi_Release(const TinyCLR_Spi_Controller* self);
TinyCLR_Result LPC17_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, const TinyCLR_Spi_Settings* settings);
TinyCLR_Result LPC17_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Spi_WriteRead(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
TinyCLR_Result LPC17_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
uint32_t LPC17_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self);
uint32_t LPC17_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self);
uint32_t LPC17_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self);
TinyCLR_Result LPC17_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, uint32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
void LPC17_Uart_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC17_Uart_GetRequiredApi();
void LPC17_Uart_Reset();
int32_t LPC17_Uart_GetTxPin(int32_t controllerIndex);
int32_t LPC17_Uart_GetRxPin(int32_t controllerIndex);
int32_t LPC17_Uart_GetRtsPin(int32_t controllerIndex);
int32_t LPC17_Uart_GetCtsPin(int32_t controllerIndex);
LPC17_Gpio_PinFunction LPC17_Uart_GetTxAlternateFunction(int32_t controllerIndex);
LPC17_Gpio_PinFunction LPC17_Uart_GetRxAlternateFunction(int32_t controllerIndex);
LPC17_Gpio_PinFunction LPC17_Uart_GetRtsAlternateFunction(int32_t controllerIndex);
LPC17_Gpio_PinFunction LPC17_Uart_GetCtsAlternateFunction(int32_t controllerIndex);
bool LPC17_Uart_CanSend(int controllerIndex);
void LPC17_Uart_TxBufferEmptyInterruptEnable(int controllerIndex, bool enable);
void LPC17_Uart_RxBufferFullInterruptEnable(int controllerIndex, bool enable);

TinyCLR_Result LPC17_Uart_Acquire(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_Release(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_Enable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_Disable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, const TinyCLR_Uart_Settings* settings);
TinyCLR_Result LPC17_Uart_Flush(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC17_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result LPC17_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result LPC17_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result LPC17_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler);
TinyCLR_Result LPC17_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result LPC17_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool state);
size_t LPC17_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t LPC17_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t LPC17_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self);
size_t LPC17_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC17_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self);

//Deployment
void LPC17_Deployment_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC17_Deployment_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
TinyCLR_Result LPC17_Deployment_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC17_Deployment_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC17_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC17_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC17_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result LPC17_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result LPC17_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size);
TinyCLR_Result LPC17_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result LPC17_Deployment_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result LPC17_Deployment_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result LPC17_Deployment_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC17_Deployment_Close(const TinyCLR_Storage_Controller* self);

// Interrupt
////////////////////////////////////////////////////////////////////////////////
//Interrupt Internal
////////////////////////////////////////////////////////////////////////////////
class LPC17_DisableInterrupts_RaiiHelper {
    uint32_t state;

public:
    LPC17_DisableInterrupts_RaiiHelper();
    ~LPC17_DisableInterrupts_RaiiHelper();

    bool IsDisabled();
    void Acquire();
    void Release();
};

class LPC17_InterruptStarted_RaiiHelper {
public:
    LPC17_InterruptStarted_RaiiHelper();
    ~LPC17_InterruptStarted_RaiiHelper();
};

#define DISABLE_INTERRUPTS_SCOPED(name) LPC17_DisableInterrupts_RaiiHelper name
#define INTERRUPT_STARTED_SCOPED(name) LPC17_InterruptStarted_RaiiHelper name

bool LPC17_InterruptInternal_Activate(uint32_t index, uint32_t* isr, void* isrParam);
bool LPC17_InterruptInternal_Deactivate(uint32_t index);

void LPC17_Interrupt_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC17_Interrupt_GetRequiredApi();
TinyCLR_Result LPC17_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result LPC17_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self);
void LPC17_Interrupt_Enable();
void LPC17_Interrupt_Disable();
void LPC17_Interrupt_WaitForInterrupt();
bool LPC17_Interrupt_IsDisabled();

extern TinyCLR_Interrupt_StartStopHandler LPC17_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler LPC17_Interrupt_Ended;

// I2C
void LPC17_I2c_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC17_I2c_Reset();
TinyCLR_Result LPC17_I2c_Acquire(const TinyCLR_I2c_Controller* self);
TinyCLR_Result LPC17_I2c_Release(const TinyCLR_I2c_Controller* self);
TinyCLR_Result LPC17_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, const TinyCLR_I2c_Settings* settings);
TinyCLR_Result LPC17_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error);
void LPC17_I2c_StartTransaction(int32_t channel);
void LPC17_I2c_StopTransaction(int32_t channel);

// Time
void LPC17_Time_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC17_Time_GetRequiredApi();
TinyCLR_Result LPC17_Time_Initialize(const TinyCLR_NativeTime_Controller* self);
TinyCLR_Result LPC17_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self);
uint64_t LPC17_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self);
uint64_t LPC17_Time_GetCurrentProcessorTime();
uint64_t LPC17_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks);
uint64_t LPC17_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time);
TinyCLR_Result LPC17_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback);
TinyCLR_Result LPC17_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks);
void LPC17_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
void LPC17_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime);

// Power
void LPC17_Power_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC17_Power_GetRequiredApi();
void LPC17_Power_SetHandlers(void(*stop)(), void(*restart)());
TinyCLR_Result LPC17_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level, TinyCLR_Power_SleepWakeSource wakeSource);
TinyCLR_Result LPC17_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter);
TinyCLR_Result LPC17_Power_Initialize(const TinyCLR_Power_Controller* self);
TinyCLR_Result LPC17_Power_Uninitialize(const TinyCLR_Power_Controller* self);

//UsbClient
const TinyCLR_Api_Info* LPC17_UsbDevice_GetRequiredApi();
void LPC17_UsbDevice_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC17_UsbDevice_Reset();

struct USB_PACKET64;
struct UsClientState;
typedef void(*USB_NEXT_CALLBACK)(UsClientState*);

void TinyCLR_UsbClient_ClearEvent(UsClientState *usClientState, uint32_t event);
void TinyCLR_UsbClient_ClearEndpoints(UsClientState *usClientState, int32_t endpoint);
USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(UsClientState* usClientState, int32_t endpoint, bool& disableRx);
USB_PACKET64* TinyCLR_UsbClient_TxDequeue(UsClientState* usClientState, int32_t endpoint);
void TinyCLR_UsbClient_StateCallback(UsClientState* usClientState);
uint8_t TinyCLR_UsbClient_ControlCallback(UsClientState* usClientState);

// LCD
void LPC17_Display_Reset();
void LPC17_Display_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result LPC17_Display_Acquire(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC17_Display_Release(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC17_Display_Enable(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC17_Display_Disable(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC17_Display_GetCapabilities(const TinyCLR_Display_Controller* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount);
TinyCLR_Result LPC17_Display_GetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration);
TinyCLR_Result LPC17_Display_SetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration);
TinyCLR_Result LPC17_Display_DrawBuffer(const TinyCLR_Display_Controller* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data);
TinyCLR_Result LPC17_Display_DrawPixel(const TinyCLR_Display_Controller* self, uint32_t x, uint32_t y, uint64_t color);
TinyCLR_Result LPC17_Display_WriteString(const TinyCLR_Display_Controller* self, const char* buffer, size_t length);

//Startup
void LPC17_Startup_Initialize();
void LPC17_Startup_GetHeap(uint8_t*& start, size_t& length);
void LPC17_Startup_GetDebuggerTransportApi(const TinyCLR_Api_Info*& api, const void*& configuration);
void LPC17_Startup_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
void LPC17_Startup_GetRunApp(bool& runApp);
void LPC17_Startup_OnSoftReset(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);
void LPC17_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);
const TinyCLR_Startup_DeploymentConfiguration* LPC17_Deployment_GetDeploymentConfiguration();

extern const TinyCLR_Api_Manager* apiManager;

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
