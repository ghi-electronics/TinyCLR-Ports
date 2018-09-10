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
    bool outputDirection;
    bool apply;
};

#define PIN(port, pin) (port * 32 + pin)
#define PIN_NONE 0xFFFFFFFF
#define PF(num) (CONCAT(LPC24_Gpio_PinFunction::PinFunction, num))
#define PF_NONE LPC24_Gpio_PinFunction::PinFunction0

#define INIT(direction, pinMode, pinFunction, outputDirection, apply) { LPC24_Gpio_Direction::direction, LPC24_Gpio_PinMode::pinMode, LPC24_Gpio_PinFunction::pinFunction, outputDirection, apply }
#define ALTFUN(direction, pinMode, pinFunction) { LPC24_Gpio_Direction::direction, LPC24_Gpio_PinMode::pinMode, LPC24_Gpio_PinFunction::pinFunction, true }
#define INPUT(pinMode) { LPC24_Gpio_Direction::Input, LPC24_Gpio_PinMode::pinMode, LPC24_Gpio_PinFunction::PinFunction0, true }
#define DEFAULT() INIT(Input, Inactive, PinFunction0, false, true)
#define NO_INIT() INIT(Input, Inactive, PinFunction0, false, false)

extern const TinyCLR_Api_Manager* apiManager;

void LPC24_Gpio_Reset();
void LPC24_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC24_Gpio_GetRequiredApi();
TinyCLR_Result LPC24_Gpio_Acquire(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result LPC24_Gpio_Release(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result LPC24_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t alternate);
TinyCLR_Result LPC24_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result LPC24_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result LPC24_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks);
TinyCLR_Result LPC24_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result LPC24_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
bool LPC24_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode LPC24_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin);
uint64_t LPC24_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin);
uint32_t LPC24_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result LPC24_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler);
TinyCLR_Result LPC24_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
void LPC24_Gpio_EnableOutputPin(int32_t pin, bool initialState);
void LPC24_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor);
bool LPC24_Gpio_OpenPin(int32_t pin);
bool LPC24_Gpio_ClosePin(int32_t pin);
bool LPC24_Gpio_ReadPin(int32_t pin);
void LPC24_Gpio_WritePin(int32_t pin, bool value);
bool LPC24_Gpio_ConfigurePin(int32_t pin, LPC24_Gpio_Direction pinDir, LPC24_Gpio_PinFunction alternateFunction, LPC24_Gpio_PinMode pullResistor);

// ADC
void LPC24_Adc_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_Adc_Reset();
uint32_t LPC24_Adc_GetPin(uint32_t channel);
LPC24_Gpio_PinFunction LPC24_Adc_GetPinFunction(uint32_t channel);
TinyCLR_Result LPC24_Adc_Acquire(const TinyCLR_Adc_Controller* self);
TinyCLR_Result LPC24_Adc_Release(const TinyCLR_Adc_Controller* self);
TinyCLR_Result LPC24_Adc_OpenChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Adc_CloseChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Adc_ReadChannel(const TinyCLR_Adc_Controller* self, uint32_t channel, int32_t& value);
uint32_t LPC24_Adc_GetChannelCount(const TinyCLR_Adc_Controller* self);
uint32_t LPC24_Adc_GetResolutionInBits(const TinyCLR_Adc_Controller* self);
int32_t LPC24_Adc_GetMinValue(const TinyCLR_Adc_Controller* self);
int32_t LPC24_Adc_GetMaxValue(const TinyCLR_Adc_Controller* self);
TinyCLR_Adc_ChannelMode LPC24_Adc_GetChannelMode(const TinyCLR_Adc_Controller* self);
TinyCLR_Result LPC24_Adc_SetChannelMode(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);
bool LPC24_Adc_IsChannelModeSupported(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);

// CAN
void LPC24_Can_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result LPC24_Can_Acquire(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_Release(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_SoftReset(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_WriteMessage(const TinyCLR_Can_Controller* self, uint32_t arbitrationId, bool isExtendedId, bool isRemoteTransmissionRequest, const uint8_t* data, size_t length);
TinyCLR_Result LPC24_Can_ReadMessage(const TinyCLR_Can_Controller* self, uint32_t& arbitrationId, bool& isExtendedId, bool& isRemoteTransmissionRequest, uint8_t* data, size_t& length, uint64_t& timestamp);
TinyCLR_Result LPC24_Can_SetBitTiming(const TinyCLR_Can_Controller* self, uint32_t propagation, uint32_t phase1, uint32_t phase2, uint32_t baudratePrescaler, uint32_t synchronizationJumpWidth, bool useMultiBitSampling);
size_t LPC24_Can_GetMessagesToRead(const TinyCLR_Can_Controller* self);
size_t LP24_Can_GetMessagesToWrite(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_SetMessageReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_MessageReceivedHandler handler);
TinyCLR_Result LPC24_Can_SetErrorReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_ErrorReceivedHandler handler);
TinyCLR_Result LPC24_Can_SetExplicitFilters(const TinyCLR_Can_Controller* self, const uint32_t* filters, size_t count);
TinyCLR_Result LPC24_Can_SetGroupFilters(const TinyCLR_Can_Controller* self, const uint32_t* lowerBounds, const uint32_t* upperBounds, size_t count);
TinyCLR_Result LPC24_Can_ClearReadBuffer(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_IsWritingAllowed(const TinyCLR_Can_Controller* self, bool& allowed);
size_t LPC24_Can_GetWriteErrorCount(const TinyCLR_Can_Controller* self);
size_t LPC24_Can_GetReadErrorCount(const TinyCLR_Can_Controller* self);
uint32_t LPC24_Can_GetSourceClock(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_SetReadBufferSize(const TinyCLR_Can_Controller* self, size_t size);
size_t LPC24_Can_GetReadBufferSize(const TinyCLR_Can_Controller* self);
size_t LPC24_Can_GetWriteBufferSize(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_SetWriteBufferSize(const TinyCLR_Can_Controller* self, size_t size);
TinyCLR_Result LPC24_Can_Enable(const TinyCLR_Can_Controller* self);
TinyCLR_Result LPC24_Can_Disable(const TinyCLR_Can_Controller* self);
bool LPC24_Can_CanWriteMessage(const TinyCLR_Can_Controller* self);
bool LPC24_Can_CanReadMessage(const TinyCLR_Can_Controller* self);
void LPC24_Can_Reset();

//DAC
void LPC24_Dac_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_Dac_Reset();
TinyCLR_Result LPC24_Dac_Acquire(const TinyCLR_Dac_Controller* self);
TinyCLR_Result LPC24_Dac_Release(const TinyCLR_Dac_Controller* self);
TinyCLR_Result LPC24_Dac_OpenChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Dac_CloseChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Dac_WriteValue(const TinyCLR_Dac_Controller* self, uint32_t channel, int32_t value);
uint32_t LPC24_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self);
uint32_t LPC24_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self);
int32_t LPC24_Dac_GetMinValue(const TinyCLR_Dac_Controller* self);
int32_t LPC24_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self);

// PWM
struct PwmState {
    int32_t                     controllerIndex;
    int32_t                     channel[MAX_PWM_PER_CONTROLLER];
    int32_t                     match[MAX_PWM_PER_CONTROLLER];

    LPC24_Gpio_Pin              gpioPin[MAX_PWM_PER_CONTROLLER];

    uint32_t                    outputEnabled[MAX_PWM_PER_CONTROLLER];
    uint32_t                    *matchAddress[MAX_PWM_PER_CONTROLLER];

    TinyCLR_Pwm_PulsePolarity invert[MAX_PWM_PER_CONTROLLER];
    bool                        isOpened[MAX_PWM_PER_CONTROLLER];

    double                      frequency;
    double                      dutyCycle[MAX_PWM_PER_CONTROLLER];

    uint16_t initializeCount;
};
void LPC24_Pwm_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_Pwm_Reset();
void LPC24_Pwm_ResetController(int32_t controller);
LPC24_Gpio_Pin LPC24_Pwm_GetPins(int32_t controller, int32_t channel);
TinyCLR_Result LPC24_Pwm_Acquire(const TinyCLR_Pwm_Controller* self);
TinyCLR_Result LPC24_Pwm_Release(const TinyCLR_Pwm_Controller* self);
uint32_t LPC24_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency);
TinyCLR_Result LPC24_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result LPC24_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity);
double LPC24_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self);
double LPC24_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self);
double LPC24_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self);
uint32_t LPC24_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self);

//RTC
void LPC24_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result LPC24_Rtc_Acquire(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result LPC24_Rtc_Release(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result LPC24_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value);
TinyCLR_Result LPC24_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value);
TinyCLR_Result LPC24_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value);

////////////////////////////////////////////////////////////////////////////////
//SD
////////////////////////////////////////////////////////////////////////////////
void LPC24_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager);

TinyCLR_Result LPC24_SdCard_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC24_SdCard_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC24_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC24_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC24_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result LPC24_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result LPC24_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result LPC24_SdCard_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result LPC24_SdCard_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result LPC24_SdCard_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC24_SdCard_Close(const TinyCLR_Storage_Controller* self);

TinyCLR_Result LPC24_SdCard_Reset();

//SPI
void LPC24_Spi_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_Spi_Reset();
bool LPC24_Spi_Transaction_Start(int32_t controller);
bool LPC24_Spi_Transaction_Stop(int32_t controller);
bool LPC24_Spi_Transaction_nWrite8_nRead8(int32_t controller);
TinyCLR_Result LPC24_Spi_Acquire(const TinyCLR_Spi_Controller* self);
TinyCLR_Result LPC24_Spi_Release(const TinyCLR_Spi_Controller* self);
TinyCLR_Result LPC24_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, uint32_t chipSelectLine, TinyCLR_Spi_ChipSelectType chipSelectType, uint32_t clockFrequency, uint32_t dataBitLength, TinyCLR_Spi_Mode mode);
TinyCLR_Result LPC24_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Spi_WriteRead(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
TinyCLR_Result LPC24_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
uint32_t LPC24_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self);
uint32_t LPC24_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self);
uint32_t LPC24_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self);
TinyCLR_Result LPC24_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, uint32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
void LPC24_Uart_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC24_Uart_GetRequiredApi();
void LPC24_Uart_Reset();
int32_t LPC24_Uart_GetTxPin(int32_t controller);
int32_t LPC24_Uart_GetRxPin(int32_t controller);
int32_t LPC24_Uart_GetRtsPin(int32_t controller);
int32_t LPC24_Uart_GetCtsPin(int32_t controller);
LPC24_Gpio_PinFunction LPC24_Uart_GetTxAlternateFunction(int32_t controller);
LPC24_Gpio_PinFunction LPC24_Uart_GetRxAlternateFunction(int32_t controller);
LPC24_Gpio_PinFunction LPC24_Uart_GetRtsAlternateFunction(int32_t controller);
LPC24_Gpio_PinFunction LPC24_Uart_GetCtsAlternateFunction(int32_t controller);
bool LPC24_Uart_TxHandshakeEnabledState(int controller);
void LPC24_Uart_TxBufferEmptyInterruptEnable(int controller, bool enable);
void LPC24_Uart_RxBufferFullInterruptEnable(int controller, bool enable);

TinyCLR_Result LPC24_Uart_Acquire(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_Release(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_Enable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_Disable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking);
TinyCLR_Result LPC24_Uart_Flush(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result LPC24_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result LPC24_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result LPC24_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result LPC24_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler);
TinyCLR_Result LPC24_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result LPC24_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool state);
size_t LPC24_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t LPC24_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t LPC24_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self);
size_t LPC24_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self);
TinyCLR_Result LPC24_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self);

//Deployment
void LPC24_Deployment_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_Deployment_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
TinyCLR_Result LPC24_Deployment_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC24_Deployment_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC24_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC24_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result LPC24_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result LPC24_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result LPC24_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size);
TinyCLR_Result LPC24_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result LPC24_Deployment_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result LPC24_Deployment_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result LPC24_Deployment_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result LPC24_Deployment_Close(const TinyCLR_Storage_Controller* self);
bool LPC24_Deployment_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer);
bool LPC24_Deployment_IsSupportsXIP(const TinyCLR_Storage_Controller* self);
uint32_t LPC24_Flash_GetPartId();

// Interrupt
class LPC24_SmartPtr_IRQ {

    uint32_t m_state;

    void Disable();
    void Restore();

public:
    LPC24_SmartPtr_IRQ();
    ~LPC24_SmartPtr_IRQ();

    bool IsDisabled();
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

void LPC24_Interrupt_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC24_Interrupt_GetRequiredApi();
TinyCLR_Result LPC24_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result LPC24_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self);
bool LPC24_Interrupt_Activate(uint32_t Irq_Index, uint32_t *handler, void* ISR_Param);
bool LPC24_Interrupt_Deactivate(uint32_t Irq_Index);
bool LPC24_Interrupt_Enable(uint32_t Irq_Index);
bool LPC24_Interrupt_Disable(uint32_t Irq_Index);
bool LPC24_Interrupt_EnableState(uint32_t Irq_Index);
bool LPC24_Interrupt_InterruptState(uint32_t Irq_Index);


bool LPC24_Interrupt_GlobalIsDisabled();
bool LPC24_Interrupt_GlobalEnabled(bool force);
bool LPC24_Interrupt_GlobalDisabled(bool force);

void LPC24_Interrupt_GlobalRestore();
void LPC24_Interrupt_GlobalWaitForInterrupt();

extern TinyCLR_Interrupt_StartStopHandler LPC24_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler LPC24_Interrupt_Ended;

// I2C
void LPC24_I2c_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_I2c_Reset();
TinyCLR_Result LPC24_I2c_Acquire(const TinyCLR_I2c_Controller* self);
TinyCLR_Result LPC24_I2c_Release(const TinyCLR_I2c_Controller* self);
TinyCLR_Result LPC24_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, uint32_t slaveAddress, TinyCLR_I2c_AddressFormat addressFormat, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result LPC24_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error);
void LPC24_I2c_StartTransaction(int32_t channel);
void LPC24_I2c_StopTransaction(int32_t channel);

// Time
void LPC24_Time_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC24_Time_GetRequiredApi();
TinyCLR_Result LPC24_Time_Initialize(const TinyCLR_NativeTime_Controller* self);
TinyCLR_Result LPC24_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self);
uint64_t LPC24_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks);
uint64_t LPC24_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time);
uint64_t LPC24_Time_MillisecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks);
uint64_t LPC24_Time_MicrosecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
uint64_t LPC24_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self);
TinyCLR_Result LPC24_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks);
TinyCLR_Result LPC24_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback);
void LPC24_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
void LPC24_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
void LPC24_Time_GetDriftParameters(const TinyCLR_NativeTime_Controller* self, int32_t* a, int32_t* b, int64_t* c);
void LPC24_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime);

// Power
void LPC24_Power_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* LPC24_Power_GetRequiredApi();
void LPC24_Power_SetHandlers(void(*stop)(), void(*restart)());
TinyCLR_Result LPC24_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level, TinyCLR_Power_SleepWakeSource wakeSource);
TinyCLR_Result LPC24_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter);
TinyCLR_Result LPC24_Power_Initialize(const TinyCLR_Power_Controller* self);
TinyCLR_Result LPC24_Power_Uninitialize(const TinyCLR_Power_Controller* self);

//UsbClient

#define OTGStCtrl (*(volatile unsigned *)0xFFE0C110)
#define USBDevIntSt (*(volatile unsigned *)0xFFE0C200)
#define USBDevIntEn (*(volatile unsigned *)0xFFE0C204)
#define USBDevIntClr (*(volatile unsigned *)0xFFE0C208)

#define USBCmdCode (*(volatile unsigned *)0xFFE0C210)
#define USBCmdData (*(volatile unsigned *)0xFFE0C214)
#define USBRxData (*(volatile unsigned *)0xFFE0C218)
#define USBTxData (*(volatile unsigned *)0xFFE0C21C)
#define USBRxPLen (*(volatile unsigned *)0xFFE0C220)
#define USBTxPLen (*(volatile unsigned *)0xFFE0C224)
#define USBCtrl (*(volatile unsigned *)0xFFE0C228)

#define USBEpIntSt (*(volatile unsigned *)0xFFE0C230)
#define USBEpIntEn (*(volatile unsigned *)0xFFE0C234)
#define USBEpIntClr (*(volatile unsigned *)0xFFE0C238)
#define USBEpIntSet (*(volatile unsigned *)0xFFE0C23C)

#define USBReEp (*(volatile unsigned *)0xFFE0C244)
#define USBEpInd (*(volatile unsigned *)0xFFE0C248)
#define USBEpMaxPSize (*(volatile unsigned *)0xFFE0C24C)

#define USBClkCtrl (*(volatile unsigned *)0xFFE0CFF4)
#define USBClkSt (*(volatile unsigned *)0xFFE0CFF8)
#define OTGClkCtrl (*(volatile unsigned *)0xFFE0CFF4)
#define OTGClkSt (*(volatile unsigned *)0xFFE0CFF8)

const TinyCLR_Api_Info* LPC24_UsbDevice_GetRequiredApi();
void LPC24_UsbDevice_AddApi(const TinyCLR_Api_Manager* apiManager);
void LPC24_UsbDevice_Reset();
void LPC24_UsbDevice_PinConfiguration();

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
void LPC24_Display_Reset();
void LPC24_Display_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result LPC24_Display_Acquire(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC24_Display_Release(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC24_Display_Enable(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC24_Display_Disable(const TinyCLR_Display_Controller* self);
TinyCLR_Result LPC24_Display_GetCapabilities(const TinyCLR_Display_Controller* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount);
TinyCLR_Result LPC24_Display_GetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration);
TinyCLR_Result LPC24_Display_SetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration);
TinyCLR_Result LPC24_Display_DrawBuffer(const TinyCLR_Display_Controller* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data);
TinyCLR_Result LPC24_Display_DrawPixel(const TinyCLR_Display_Controller* self, uint32_t x, uint32_t y, uint64_t color);
TinyCLR_Result LPC24_Display_WriteString(const TinyCLR_Display_Controller* self, const char* buffer, size_t length);

//Startup
void LPC24_Startup_Initialize();
void LPC24_Startup_GetHeap(uint8_t*& start, size_t& length);
int32_t LPC24_Startup_GetDeviceId();
void LPC24_Startup_GetDebuggerTransportApi(const TinyCLR_Api_Info*& api, const void*& configuration);
void LPC24_Startup_GetRunApp(bool& runApp);
void LPC24_Startup_OnSoftReset(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);
void LPC24_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);
const TinyCLR_Startup_DeploymentConfiguration* LPC24_Deployment_GetDeploymentConfiguration();
void LPC24_Startup_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);


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
        // Make changes to include the FIQSTATUS if the state is modifed to support FIQ
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
    static const uint32_t c_I2C0_Base = 0xE001C000;
    static const uint32_t c_I2C1_Base = 0xE005C000;
    static const uint32_t c_I2C2_Base = 0xE0080000;

    static const uint32_t c_I2C_Clk_KHz = SYSTEM_CLOCK_HZ / 1000;

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

    static const uint32_t c_SPI_Clk_KHz = (LPC24_AHB_CLOCK_HZ / 1000);
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
    static LPC24XX_PCB    & PCB() { return *(LPC24XX_PCB    *)(size_t)(LPC24XX_PCB::c_PCB_Base); }
    static LPC24XX_SYSCON & SYSCON() { return *(LPC24XX_SYSCON *)(size_t)(LPC24XX_SYSCON::c_SYSCON_Base); }
    static LPC24XX_SPI    & SPI(int sel) { return *(LPC24XX_SPI    *)(size_t)((sel == 0) ? (LPC24XX_SPI::c_SPI0_Base) : (LPC24XX_SPI::c_SPI1_Base)); }
    static LPC24XX_I2C    & I2C(int sel) { return *(LPC24XX_I2C    *)(size_t)((sel == 0) ? (LPC24XX_I2C::c_I2C0_Base) : (sel == 1 ? (LPC24XX_I2C::c_I2C1_Base) : (LPC24XX_I2C::c_I2C2_Base))); }
    static LPC24XX_WATCHDOG & WTDG() { return *(LPC24XX_WATCHDOG *)(size_t)(LPC24XX_WATCHDOG::c_WATCHDOG_Base); }

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

