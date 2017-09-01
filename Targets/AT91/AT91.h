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

#include <defines.h>
#include <TinyCLR.h>
#include <DeviceSelector.h>

// GPIO
enum class AT91_Gpio_Direction : uint8_t {
    Input = 0,
    Output = 1,
};

enum class AT91_Gpio_PinFunction : uint8_t {
    PinFunction0 = 0,
    PinFunction1 = 1,
    PinFunction2 = 2,
    PinFunction3 = 3,
};

enum class AT91_Gpio_PinMode : uint8_t {
    PullUp = 0,
    Reserved = 1,
    Inactive = 2,
    PullDown = 3
};

void AT91_Gpio_Reset();
const TinyCLR_Api_Info* AT91_Gpio_GetApi();
TinyCLR_Result AT91_Gpio_Acquire(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result AT91_Gpio_Release(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result AT91_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t alternate);
TinyCLR_Result AT91_Gpio_Read(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result AT91_Gpio_Write(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result AT91_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin, int32_t debounceTime);
TinyCLR_Result AT91_Gpio_SetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result AT91_Gpio_AcquirePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
bool AT91_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode AT91_Gpio_GetDriveMode(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t AT91_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Provider* self, int32_t pin);
int32_t AT91_Gpio_GetPinCount(const TinyCLR_Gpio_Provider* self);
TinyCLR_Result AT91_Gpio_SetValueChangedHandler(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_ValueChangedHandler ISR);
TinyCLR_Result AT91_Gpio_ReleasePin(const TinyCLR_Gpio_Provider* self, int32_t pin);
void AT91_Gpio_EnableOutputPin(int32_t pin, bool initialState);
void AT91_Gpio_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor);
bool AT91_Gpio_OpenPin(int32_t pin);
bool AT91_Gpio_ClosePin(int32_t pin);
bool AT91_Gpio_ReadPin(int32_t pin);
void AT91_Gpio_WritePin(int32_t pin, bool value);
bool AT91_Gpio_ConfigurePin(int32_t pin, AT91_Gpio_Direction pinDir, AT91_Gpio_PinFunction alternateFunction, AT91_Gpio_PinMode pullResistor);

// ADC
const TinyCLR_Api_Info* AT91_Adc_GetApi();
void AT91_Adc_Reset();
int32_t AT91_Adc_GetControllerCount();
int32_t AT91_Adc_GetPin(int32_t channel);
AT91_Gpio_PinFunction AT91_Adc_GetPinFunction(int32_t channel);
TinyCLR_Result AT91_Adc_Acquire(const TinyCLR_Adc_Provider* self);
TinyCLR_Result AT91_Adc_Release(const TinyCLR_Adc_Provider* self);
TinyCLR_Result AT91_Adc_AcquireChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result AT91_Adc_ReleaseChannel(const TinyCLR_Adc_Provider* self, int32_t channel);
TinyCLR_Result AT91_Adc_ReadValue(const TinyCLR_Adc_Provider* self, int32_t channel, int32_t& value);
int32_t AT91_Adc_GetChannelCount(const TinyCLR_Adc_Provider* self);
int32_t AT91_Adc_GetResolutionInBits(const TinyCLR_Adc_Provider* self);
int32_t AT91_Adc_GetMinValue(const TinyCLR_Adc_Provider* self);
int32_t AT91_Adc_GetMaxValue(const TinyCLR_Adc_Provider* self);
TinyCLR_Adc_ChannelMode AT91_Adc_GetChannelMode(const TinyCLR_Adc_Provider* self);
TinyCLR_Result AT91_Adc_SetChannelMode(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);
bool AT91_Adc_IsChannelModeSupported(const TinyCLR_Adc_Provider* self, TinyCLR_Adc_ChannelMode mode);

//DAC
const TinyCLR_Api_Info* AT91_Dac_GetApi();
void AT91_Dac_Reset();
TinyCLR_Result AT91_Dac_Acquire(const TinyCLR_Dac_Provider* self);
TinyCLR_Result AT91_Dac_Release(const TinyCLR_Dac_Provider* self);
TinyCLR_Result AT91_Dac_AcquireChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result AT91_Dac_ReleaseChannel(const TinyCLR_Dac_Provider* self, int32_t channel);
TinyCLR_Result AT91_Dac_WriteValue(const TinyCLR_Dac_Provider* self, int32_t channel, int32_t value);
int32_t AT91_Dac_GetChannelCount(const TinyCLR_Dac_Provider* self);
int32_t AT91_Dac_GetResolutionInBits(const TinyCLR_Dac_Provider* self);
int32_t AT91_Dac_GetMinValue(const TinyCLR_Dac_Provider* self);
int32_t AT91_Dac_GetMaxValue(const TinyCLR_Dac_Provider* self);

// PWM
struct PwmController {
    int32_t                     id;
    int32_t                     channel[MAX_PWM_PER_CONTROLLER];
    int32_t                     subChannel[MAX_PWM_PER_CONTROLLER];
    uint32_t                    gpioPin[MAX_PWM_PER_CONTROLLER];
    AT91_Gpio_PinFunction      gpioAlternateFunction[MAX_PWM_PER_CONTROLLER];
    uint32_t                    outputEnabled[MAX_PWM_PER_CONTROLLER];
    uint32_t                    *matchAddress[MAX_PWM_PER_CONTROLLER];
    bool                        invert[MAX_PWM_PER_CONTROLLER];
    double                      frequency;
    double                      dutyCycle[MAX_PWM_PER_CONTROLLER];
};
const TinyCLR_Api_Info* AT91_Pwm_GetApi();
void AT91_Pwm_Reset();
void AT91_Pwm_ResetController(int32_t controller);
PwmController* AT91_Pwm_GetControllers();
TinyCLR_Result AT91_Pwm_Acquire(const TinyCLR_Pwm_Provider* self);
TinyCLR_Result AT91_Pwm_Release(const TinyCLR_Pwm_Provider* self);
int32_t AT91_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result AT91_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Provider* self, double& frequency);
TinyCLR_Result AT91_Pwm_AcquirePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result AT91_Pwm_ReleasePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result AT91_Pwm_EnablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result AT91_Pwm_DisablePin(const TinyCLR_Pwm_Provider* self, int32_t pin);
TinyCLR_Result AT91_Pwm_SetPulseParameters(const TinyCLR_Pwm_Provider* self, int32_t pin, double dutyCycle, bool invertPolarity);
double AT91_Pwm_GetMinFrequency(const TinyCLR_Pwm_Provider* self);
double AT91_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Provider* self);
double AT91_Pwm_GetActualFrequency(const TinyCLR_Pwm_Provider* self);
int32_t AT91_Pwm_GetPinCount(const TinyCLR_Pwm_Provider* self);

//SPI
const TinyCLR_Api_Info* AT91_Spi_GetApi();
void AT91_Spi_Reset();
bool AT91_Spi_Transaction_Start(int32_t controller);
bool AT91_Spi_Transaction_Stop(int32_t controller);
bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controller);
TinyCLR_Result AT91_Spi_Acquire(const TinyCLR_Spi_Provider* self);
TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Provider* self);
TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Provider* self, int32_t chipSelectLine, int32_t clockFrequency, int32_t dataBitLength, TinyCLR_Spi_Mode mode);
TinyCLR_Result AT91_Spi_Read(const TinyCLR_Spi_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Spi_Write(const TinyCLR_Spi_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Spi_TransferFullDuplex(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength);
int32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Provider* self);
int32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Provider* self);
int32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Provider* self);
TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Provider* self, int32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
const TinyCLR_Api_Info* AT91_Uart_GetApi();
void AT91_Uart_Reset();
int32_t AT91_Uart_GetTxPin(int32_t portNum);
int32_t AT91_Uart_GetRxPin(int32_t portNum);
int32_t AT91_Uart_GetRtsPin(int32_t portNum);
int32_t AT91_Uart_GetCtsPin(int32_t portNum);
AT91_Gpio_PinFunction AT91_Uart_GetTxAlternateFunction(int32_t portNum);
AT91_Gpio_PinFunction AT91_Uart_GetRxAlternateFunction(int32_t portNum);
AT91_Gpio_PinFunction AT91_Uart_GetRtsAlternateFunction(int32_t portNum);
AT91_Gpio_PinFunction AT91_Uart_GetCtsAlternateFunction(int32_t portNum);
bool AT91_Uart_TxHandshakeEnabledState(int portNum);
void AT91_Uart_TxBufferEmptyInterruptEnable(int portNum, bool enable);
void AT91_Uart_RxBufferFullInterruptEnable(int portNum, bool enable);
TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Provider* self);
TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Provider* self);
TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Provider* self, uint32_t baudRate, uint32_t dataBits, TinyCLR_Uart_Parity parity, TinyCLR_Uart_StopBitCount stopBits, TinyCLR_Uart_Handshake handshaking);
TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Provider* self);
TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Provider* self, uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Provider* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Uart_SetPinChangedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_PinChangedHandler handler);
TinyCLR_Result AT91_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result AT91_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Provider* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result AT91_Uart_GetBreakSignalState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result AT91_Uart_SetBreakSignalState(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result AT91_Uart_GetCarrierDetectState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result AT91_Uart_GetDataReadyState(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result AT91_Uart_GetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result AT91_Uart_SetIsDataTerminalReadyEnabled(const TinyCLR_Uart_Provider* self, bool state);
TinyCLR_Result AT91_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool& state);
TinyCLR_Result AT91_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Provider* self, bool state);

//Deployment
const TinyCLR_Api_Info* AT91_Deployment_GetApi();
TinyCLR_Result AT91_Flash_Acquire(const TinyCLR_Deployment_Provider* self, bool &supportXIP);
TinyCLR_Result AT91_Flash_Release(const TinyCLR_Deployment_Provider* self);
TinyCLR_Result AT91_Flash_Read(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, uint8_t* buffer);
TinyCLR_Result AT91_Flash_Write(const TinyCLR_Deployment_Provider* self, uint32_t address, size_t length, const uint8_t* buffer);
TinyCLR_Result AT91_Flash_EraseBlock(const TinyCLR_Deployment_Provider* self, uint32_t sector);
TinyCLR_Result AT91_Flash_IsBlockErased(const TinyCLR_Deployment_Provider* self, uint32_t sector, bool &erased);
TinyCLR_Result AT91_Flash_GetBytesPerSector(const TinyCLR_Deployment_Provider* self, uint32_t address, int32_t& size);
TinyCLR_Result AT91_Flash_GetSectorMap(const TinyCLR_Deployment_Provider* self, const uint32_t*& addresses, const uint32_t*& sizes, size_t& count);
bool AT91_Flash_PageProgram(uint32_t byteAddress, uint32_t NumberOfBytesToWrite, const uint8_t * pointerToWriteBuffer);
bool AT91_Flash_IsSupportsXIP(const TinyCLR_Deployment_Provider* self);
uint32_t AT91_Flash_GetPartId();

// Interrupt
class AT91_SmartPtr_IRQ {

    uint32_t m_state;

public:
    AT91_SmartPtr_IRQ() { Disable(); };
    ~AT91_SmartPtr_IRQ() { Restore(); };

    bool WasDisabled();
    void Acquire();
    void Release();
    void Probe();

    static uint32_t GetState();

private:
    void Disable();
    void Restore();
};

const TinyCLR_Api_Info* AT91_Interrupt_GetApi();
TinyCLR_Result AT91_Interrupt_Acquire(TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result AT91_Interrupt_Release();
bool AT91_Interrupt_Activate(uint32_t Irq_Index, uint32_t *ISR, void* ISR_Param);
bool AT91_Interrupt_Deactivate(uint32_t Irq_Index);
bool AT91_Interrupt_Enable(uint32_t Irq_Index);
bool AT91_Interrupt_Disable(uint32_t Irq_Index);
bool AT91_Interrupt_EnableState(uint32_t Irq_Index);
bool AT91_Interrupt_InterruptState(uint32_t Irq_Index);


bool AT91_Interrupt_GlobalIsDisabled();
bool AT91_Interrupt_GlobalEnable(bool force);
bool AT91_Interrupt_GlobalDisable(bool force);

void AT91_Interrupt_GlobalRestore();
void AT91_Interrupt_GlobalWaitForInterrupt();

extern TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Ended;

// I2C
const TinyCLR_Api_Info* AT91_I2c_GetApi();
void AT91_I2c_Reset();
TinyCLR_Result AT91_I2c_Acquire(const TinyCLR_I2c_Provider* self);
TinyCLR_Result AT91_I2c_Release(const TinyCLR_I2c_Provider* self);
TinyCLR_Result AT91_I2c_SetActiveSettings(const TinyCLR_I2c_Provider* self, int32_t slaveAddress, TinyCLR_I2c_BusSpeed busSpeed);
TinyCLR_Result AT91_I2c_ReadTransaction(const TinyCLR_I2c_Provider* self, uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result AT91_I2c_WriteTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* buffer, size_t& length, TinyCLR_I2c_TransferStatus& result);
TinyCLR_Result AT91_I2c_WriteReadTransaction(const TinyCLR_I2c_Provider* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, TinyCLR_I2c_TransferStatus& result);
void AT91_I2c_StartTransaction();
void AT91_I2c_StopTransaction();

// Time
const TinyCLR_Api_Info* AT91_Time_GetApi();
TinyCLR_Result AT91_Time_Acquire(const TinyCLR_Time_Provider* self);
TinyCLR_Result AT91_Time_Release(const TinyCLR_Time_Provider* self);
TinyCLR_Result AT91_Time_GetInitialTime(const TinyCLR_Time_Provider* self, int64_t& utcTime, int32_t& timeZoneOffsetMinutes);
uint64_t AT91_Time_TicksToTime(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t AT91_Time_TimeToTicks(const TinyCLR_Time_Provider* self, uint64_t time);
uint64_t AT91_Time_MillisecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t ticks);
uint64_t AT91_Time_MicrosecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t microseconds);
uint64_t AT91_Time_GetCurrentTicks(const TinyCLR_Time_Provider* self);
TinyCLR_Result AT91_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks);
TinyCLR_Result AT91_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback);
void AT91_Time_DelayNoInterrupt(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void AT91_Time_Delay(const TinyCLR_Time_Provider* self, uint64_t microseconds);
void AT91_Time_GetDriftParameters(const TinyCLR_Time_Provider* self, int32_t* a, int32_t* b, int64_t* c);

// Power
const TinyCLR_Api_Info* AT91_Power_GetApi();
void AT91_Power_SetHandlers(void(*stop)(), void(*restart)());
void AT91_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_Sleep_Level level);
void AT91_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter);
TinyCLR_Result AT91_Power_Acquire(const TinyCLR_Power_Provider* self);
TinyCLR_Result AT91_Power_Release(const TinyCLR_Power_Provider* self);

//UsbClient
const TinyCLR_Api_Info* AT91_UsbClient_GetApi();
void AT91_UsbClient_Reset();
void AT91_UsbClient_PinConfiguration();
TinyCLR_Result AT91_UsbClient_Acquire(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result AT91_UsbClient_Release(const TinyCLR_UsbClient_Provider* self);
TinyCLR_Result AT91_UsbClient_Open(const TinyCLR_UsbClient_Provider* self, int32_t & stream, TinyCLR_UsbClient_StreamMode mode);
TinyCLR_Result AT91_UsbClient_Close(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result AT91_UsbClient_Write(const TinyCLR_UsbClient_Provider* self, int32_t stream, const uint8_t* data, size_t& length);
TinyCLR_Result AT91_UsbClient_Read(const TinyCLR_UsbClient_Provider* self, int32_t stream, uint8_t* data, size_t& length);
TinyCLR_Result AT91_UsbClient_Flush(const TinyCLR_UsbClient_Provider* self, int32_t stream);
TinyCLR_Result AT91_UsbClient_SetDataReceivedHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_DataReceivedHandler handler);

TinyCLR_Result AT91_UsbClient_SetDeviceDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result AT91_UsbClient_SetConfigDescriptor(const TinyCLR_UsbClient_Provider* self, const void* descriptor, int32_t length);
TinyCLR_Result AT91_UsbClient_SetStringDescriptor(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_StringDescriptorType type, const wchar_t* value);
TinyCLR_Result AT91_UsbClient_SetOsExtendedPropertyHandler(const TinyCLR_UsbClient_Provider* self, TinyCLR_UsbClient_OsExtendedPropertyHandler handler);

// LCD
void AT91_Display_Reset();
const TinyCLR_Api_Info* AT91_Display_GetApi();
TinyCLR_Result AT91_Display_Acquire(const TinyCLR_Display_Provider* self, uint32_t width, uint32_t height);
TinyCLR_Result AT91_Display_Release(const TinyCLR_Display_Provider* self);
TinyCLR_Result AT91_Display_SetLcdConfiguration(const TinyCLR_Display_Provider* self, bool outputEnableIsFixed, bool outputEnablePolarity, bool pixelPolarity, uint32_t pixelClockRate, bool horizontalSyncPolarity, uint32_t horizontalSyncPulseWidth, uint32_t horizontalFrontPorch, uint32_t horizontalBackPorch, bool verticalSyncPolarity, uint32_t verticalSyncPulseWidth, uint32_t verticalFrontPorch, uint32_t verticalBackPorch);
TinyCLR_Result AT91_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data, TinyCLR_Display_Format dataFormat);
TinyCLR_Result AT91_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer);
int32_t AT91_Display_GetWidth(const TinyCLR_Display_Provider* self);
int32_t AT91_Display_GetHeight(const TinyCLR_Display_Provider* self);
TinyCLR_Display_InterfaceType AT91_Display_GetType(const TinyCLR_Display_Provider* self);

//Startup
void AT91_Startup_InitializeRegions();
void AT91_Startup_GetHeap(uint8_t*& start, size_t& length);
int32_t AT91_Startup_GetLModePin();
int32_t AT91_Startup_GetDeviceId();
TinyCLR_Gpio_PinValue AT91_Startup_GetLModeUsbState();
