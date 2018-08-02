#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_AdcChannel {
    static const size_t FIELD___ChannelNumber__BackingField___I4 = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesAdcAdcController = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_AdcController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesAdcProviderIAdcControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsChannelModeSupported___BOOLEAN__GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetChannelMode___GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetChannelMode___VOID__GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanBitTiming {
    static const size_t FIELD___Propagation__BackingField___I4 = 1;
    static const size_t FIELD___Phase1__BackingField___I4 = 2;
    static const size_t FIELD___Phase2__BackingField___I4 = 3;
    static const size_t FIELD___BaudratePrescaler__BackingField___I4 = 4;
    static const size_t FIELD___SynchronizationJumpWidth__BackingField___I4 = 5;
    static const size_t FIELD___UseMultiBitSampling__BackingField___BOOLEAN = 6;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesCanProviderICanControllerProvider = 1;
    static const size_t FIELD___MessageReceived___GHIElectronicsTinyCLRDevicesCanMessageReceivedEventHandler = 2;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesCanErrorReceivedEventHandler = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage {
    static const size_t FIELD___data___SZARRAY_U1 = 1;
    static const size_t FIELD___ArbitrationId__BackingField___I4 = 2;
    static const size_t FIELD___IsExtendedId__BackingField___BOOLEAN = 3;
    static const size_t FIELD___IsRemoteTransmissionRequest__BackingField___BOOLEAN = 4;
    static const size_t FIELD___Length__BackingField___I4 = 5;
    static const size_t FIELD___TimeStamp__BackingField___mscorlibSystemDateTime = 6;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_ErrorReceivedEventArgs {
    static const size_t FIELD___Error__BackingField___GHIElectronicsTinyCLRDevicesCanCanError = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_MessageReceivedEventArgs {
    static const size_t FIELD___Count__BackingField___I4 = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___messageReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 2;
    static const size_t FIELD___errorReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 3;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 4;
    static const size_t FIELD___MessageReceived___GHIElectronicsTinyCLRDevicesCanMessageReceivedEventHandler = 5;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesCanErrorReceivedEventHandler = 6;

    static TinyCLR_Result get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MessagesToWrite___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MessagesToRead___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_CanWriteMessage___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_CanReadMessage___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SourceClock___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Enable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Disable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetExplicitFilters___VOID__SZARRAY_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetGroupFilters___VOID__SZARRAY_I4__SZARRAY_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_DacChannel {
    static const size_t FIELD___ChannelNumber__BackingField___I4 = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesDacDacController = 2;
    static const size_t FIELD___LastWrittenValue__BackingField___I4 = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_DacController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesDacProviderIDacControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___VOID__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_DisplayController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesDisplayProviderIDisplayControllerProvider = 1;
    static const size_t FIELD___ActiveConfiguration__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayControllerSettings = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings {
    static const size_t FIELD___Width__BackingField___I4 = 1;
    static const size_t FIELD___Height__BackingField___I4 = 2;
    static const size_t FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings {
    static const size_t FIELD___DataEnableIsFixed__BackingField___BOOLEAN = 4;
    static const size_t FIELD___DataEnablePolarity__BackingField___BOOLEAN = 5;
    static const size_t FIELD___PixelPolarity__BackingField___BOOLEAN = 6;
    static const size_t FIELD___PixelClockRate__BackingField___I4 = 7;
    static const size_t FIELD___HorizontalSyncPolarity__BackingField___BOOLEAN = 8;
    static const size_t FIELD___HorizontalSyncPulseWidth__BackingField___I4 = 9;
    static const size_t FIELD___HorizontalFrontPorch__BackingField___I4 = 10;
    static const size_t FIELD___HorizontalBackPorch__BackingField___I4 = 11;
    static const size_t FIELD___VerticalSyncPolarity__BackingField___BOOLEAN = 12;
    static const size_t FIELD___VerticalSyncPulseWidth__BackingField___I4 = 13;
    static const size_t FIELD___VerticalFrontPorch__BackingField___I4 = 14;
    static const size_t FIELD___VerticalBackPorch__BackingField___I4 = 15;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result Enable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Disable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DrawBuffer___VOID__I4__I4__I4__I4__SZARRAY_U1__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DrawString___VOID__STRING(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_Interface___GHIElectronicsTinyCLRDevicesDisplayDisplayInterface(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SupportedDataFormats___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetParallelConfiguration___VOID__I4__I4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__BOOLEAN__BOOLEAN__BOOLEAN__I4__BOOLEAN__I4__I4__I4__BOOLEAN__I4__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetSpiConfiguration___VOID__I4__I4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__STRING(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_SpiDisplayControllerSettings {
    static const size_t FIELD___SpiApiName__BackingField___STRING = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesGpioProviderIGpioControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPin {
    static const size_t FIELD___callbacks___GHIElectronicsTinyCLRDevicesGpioGpioPinValueChangedEventHandler = 1;
    static const size_t FIELD___valueChangedEdge___GHIElectronicsTinyCLRDevicesGpioGpioPinEdge = 2;
    static const size_t FIELD___PinNumber__BackingField___I4 = 3;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioController = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPinValueChangedEventArgs {
    static const size_t FIELD___Edge__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioPinEdge = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___pinMap___mscorlibSystemCollectionsIDictionary = 2;
    static const size_t FIELD___dispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 3;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 4;

    static TinyCLR_Result get_PinCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenPin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClosePin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetDebounceTimeout___mscorlibSystemTimeSpan__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDebounceTimeout___VOID__I4__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetDriveMode___GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDriveMode___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___GHIElectronicsTinyCLRDevicesGpioGpioPinValue__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinValue(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsDriveModeSupported___BOOLEAN__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetPinChangedEdge___VOID__I4__GHIElectronicsTinyCLRDevicesGpioGpioPinEdge(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearPinChangedEdge___VOID__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings {
    static const size_t FIELD___SlaveAddress__BackingField___I4 = 1;
    static const size_t FIELD___AddressFormat__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cAddressFormat = 2;
    static const size_t FIELD___BusSpeed__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cController {
    static const size_t FIELD___active___GHIElectronicsTinyCLRDevicesI2cI2cDevice = 1;
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesI2cProviderII2cControllerProvider = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cDevice {
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cController = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cTransferResult {
    static const size_t FIELD___Status__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus = 1;
    static const size_t FIELD___BytesWritten__BackingField___I4 = 2;
    static const size_t FIELD___BytesRead__BackingField___I4 = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result WriteRead___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN__BOOLEAN__BYREF_I4__BYREF_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetActiveSettings___VOID__I4__GHIElectronicsTinyCLRDevicesI2cI2cAddressFormat__GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cControllerSoftwareProvider {
    static const size_t FIELD___usePullups___BOOLEAN = 1;
    static const size_t FIELD___sda___GHIElectronicsTinyCLRDevicesGpioGpioPin = 2;
    static const size_t FIELD___scl___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___writeAddress___U1 = 4;
    static const size_t FIELD___readAddress___U1 = 5;
    static const size_t FIELD___start___BOOLEAN = 6;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinFrequency___R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxFrequency___R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result EnableChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DisableChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetPulseParameters___VOID__I4__R8__GHIElectronicsTinyCLRDevicesPwmPwmPulsePolarity(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDesiredFrequency___R8__R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_PwmChannel {
    static const size_t FIELD___polarity___GHIElectronicsTinyCLRDevicesPwmPwmPulsePolarity = 1;
    static const size_t FIELD___dutyCycle___R8 = 2;
    static const size_t FIELD___ChannelNumber__BackingField___I4 = 3;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesPwmPwmController = 4;
    static const size_t FIELD___IsStarted__BackingField___BOOLEAN = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_PwmController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesPwmProviderIPwmControllerProvider = 1;
    static const size_t FIELD___ActualFrequency__BackingField___R8 = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result GetTime___GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetTime___VOID__GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesRtcProviderIRtcControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime {
    static const size_t FIELD___Year___I4 = 1;
    static const size_t FIELD___Month___I4 = 2;
    static const size_t FIELD___Week___I4 = 3;
    static const size_t FIELD___DayOfYear___I4 = 4;
    static const size_t FIELD___DayOfMonth___I4 = 5;
    static const size_t FIELD___DayOfWeek___I4 = 6;
    static const size_t FIELD___Hour___I4 = 7;
    static const size_t FIELD___Minute___I4 = 8;
    static const size_t FIELD___Second___I4 = 9;
    static const size_t FIELD___Millisecond___I4 = 10;
    static const size_t FIELD___Microsecond___I4 = 11;
    static const size_t FIELD___Nanosecond___I4 = 12;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Signals_PulseFeedback {
    static const size_t FIELD___mode___GHIElectronicsTinyCLRDevicesSignalsPulseFeedbackMode = 1;
    static const size_t FIELD___gpioApi___I = 2;
    static const size_t FIELD___pulsePinNumber___I4 = 3;
    static const size_t FIELD___echoPinNumber___I4 = 4;
    static const size_t FIELD___pulsePin___GHIElectronicsTinyCLRDevicesGpioGpioPin = 5;
    static const size_t FIELD___echoPin___GHIElectronicsTinyCLRDevicesGpioGpioPin = 6;
    static const size_t FIELD___DisableInterrupts__BackingField___BOOLEAN = 7;
    static const size_t FIELD___Timeout__BackingField___mscorlibSystemTimeSpan = 8;
    static const size_t FIELD___PulseLength__BackingField___mscorlibSystemTimeSpan = 9;
    static const size_t FIELD___PulsePinValue__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioPinValue = 10;
    static const size_t FIELD___EchoPinValue__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioPinValue = 11;
    static const size_t FIELD___EchoPinDriveMode__BackingField___GHIElectronicsTinyCLRDevicesGpioGpioPinDriveMode = 12;

    static TinyCLR_Result GeneratePulse___I8(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Signals_SignalCapture {
    static const size_t FIELD___gpioApi___I = 1;
    static const size_t FIELD___pinNumber___I4 = 2;
    static const size_t FIELD___pin___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___DisableInterrupts__BackingField___BOOLEAN = 4;
    static const size_t FIELD___Timeout__BackingField___mscorlibSystemTimeSpan = 5;

    static TinyCLR_Result Read___I4__BYREF_BOOLEAN__SZARRAY_I4__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__BOOLEAN__SZARRAY_I4__I4__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Signals_SignalGenerator {
    static const size_t FIELD___gpioApi___I = 1;
    static const size_t FIELD___pinNumber___I4 = 2;
    static const size_t FIELD___pin___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___DisableInterrupts__BackingField___BOOLEAN = 4;
    static const size_t FIELD___GeneratecarrierFrequency__BackingField___BOOLEAN = 5;
    static const size_t FIELD___CarrierFrequency__BackingField___I4 = 6;

    static TinyCLR_Result Write___VOID__SZARRAY_I4__I4__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChipSelectLineCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinClockFrequency___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxClockFrequency___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SupportedDataBitLengths___SZARRAY_I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteRead___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetActiveSettings___VOID__I4__BOOLEAN__I4__I4__GHIElectronicsTinyCLRDevicesSpiSpiMode(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerSoftwareProvider {
    static const size_t FIELD___chipSelects___mscorlibSystemCollectionsIDictionary = 1;
    static const size_t FIELD___gpioController___GHIElectronicsTinyCLRDevicesGpioGpioController = 2;
    static const size_t FIELD___mosi___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___miso___GHIElectronicsTinyCLRDevicesGpioGpioPin = 4;
    static const size_t FIELD___sck___GHIElectronicsTinyCLRDevicesGpioGpioPin = 5;
    static const size_t FIELD___cs___GHIElectronicsTinyCLRDevicesGpioGpioPin = 6;
    static const size_t FIELD___captureOnRisingEdge___BOOLEAN = 7;
    static const size_t FIELD___clockIdleState___GHIElectronicsTinyCLRDevicesGpioGpioPinValue = 8;
    static const size_t FIELD___clockActiveState___GHIElectronicsTinyCLRDevicesGpioGpioPinValue = 9;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings {
    static const size_t FIELD___UseControllerChipSelect__BackingField___BOOLEAN = 1;
    static const size_t FIELD___ChipSelectLine__BackingField___I4 = 2;
    static const size_t FIELD___ClockFrequency__BackingField___I4 = 3;
    static const size_t FIELD___DataBitLength__BackingField___I4 = 4;
    static const size_t FIELD___Mode__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiMode = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiController {
    static const size_t FIELD___active___GHIElectronicsTinyCLRDevicesSpiSpiDevice = 1;
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesSpiProviderISpiControllerProvider = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiDevice {
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiController = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_PresenceChangedEventArgs {
    static const size_t FIELD___Present__BackingField___BOOLEAN = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___presenceChangedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 2;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 3;
    static const size_t FIELD___PresenceChanged___GHIElectronicsTinyCLRDevicesStoragePresenceChangedEventHandler = 4;

    static TinyCLR_Result get_IsPresent___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_Descriptor___GHIElectronicsTinyCLRDevicesStorageStorageDescriptor(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Open___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Close___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__I8__I4__SZARRAY_U1__I4__I8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___I4__I8__I4__SZARRAY_U1__I4__I8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Erase___I4__I8__I4__I8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsErased___BOOLEAN__I8__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_StorageController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesStorageProviderIStorageControllerProvider = 1;
    static const size_t FIELD___PresenceChanged___GHIElectronicsTinyCLRDevicesStoragePresenceChangedEventHandler = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_StorageDescriptor {
    static const size_t FIELD___CanReadDirect__BackingField___BOOLEAN = 1;
    static const size_t FIELD___CanWriteDirect__BackingField___BOOLEAN = 2;
    static const size_t FIELD___CanExecuteDirect__BackingField___BOOLEAN = 3;
    static const size_t FIELD___EraseBeforeWrite__BackingField___BOOLEAN = 4;
    static const size_t FIELD___Removable__BackingField___BOOLEAN = 5;
    static const size_t FIELD___RegionsRepeat__BackingField___BOOLEAN = 6;
    static const size_t FIELD___RegionCount__BackingField___I4 = 7;
    static const size_t FIELD___RegionAddresses__BackingField___SZARRAY_I8 = 8;
    static const size_t FIELD___RegionSizes__BackingField___SZARRAY_I4 = 9;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_DataReceivedEventArgs {
    static const size_t FIELD___Count__BackingField___I4 = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_ErrorReceivedEventArgs {
    static const size_t FIELD___Error__BackingField___GHIElectronicsTinyCLRDevicesUartUartError = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_Provider_UartControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___clearToSendChangedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 2;
    static const size_t FIELD___dataReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 3;
    static const size_t FIELD___errorReceivedDispatcher___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeNativeEventDispatcher = 4;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 5;
    static const size_t FIELD___ClearToSendChanged___GHIElectronicsTinyCLRDevicesUartClearToSendChangedEventHandler = 6;
    static const size_t FIELD___DataReceived___GHIElectronicsTinyCLRDevicesUartDataReceivedEventHandler = 7;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesUartErrorReceivedEventHandler = 8;

    static TinyCLR_Result get_WriteBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_WriteBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadBufferSize___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ReadBufferSize___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_BytesToWrite___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_BytesToRead___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_IsRequestToSendEnabled___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_IsRequestToSendEnabled___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ClearToSendState___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Enable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Disable___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetActiveSettings___VOID__I4__I4__GHIElectronicsTinyCLRDevicesUartUartParity__GHIElectronicsTinyCLRDevicesUartUartStopBitCount__GHIElectronicsTinyCLRDevicesUartUartHandshake(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Flush___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Uart_UartController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesUartProviderIUartControllerProvider = 1;
    static const size_t FIELD___ClearToSendChanged___GHIElectronicsTinyCLRDevicesUartClearToSendChangedEventHandler = 2;
    static const size_t FIELD___DataReceived___GHIElectronicsTinyCLRDevicesUartDataReceivedEventHandler = 3;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesUartErrorReceivedEventHandler = 4;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices;
