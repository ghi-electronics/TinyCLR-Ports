#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_AdcChannel {
    static const size_t FIELD___m_channelNumber___I4 = 1;
    static const size_t FIELD___m_controller___GHIElectronicsTinyCLRDevicesAdcAdcController = 2;
    static const size_t FIELD___m_provider___GHIElectronicsTinyCLRDevicesAdcProviderIAdcControllerProvider = 3;
    static const size_t FIELD___m_disposed___BOOLEAN = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_AdcController {
    static const size_t FIELD___m_provider___GHIElectronicsTinyCLRDevicesAdcProviderIAdcControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 0;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesAdcProviderIAdcControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___idx___I4 = 2;

    static TinyCLR_Result get_ChannelMode___GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ChannelMode___VOID__GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsChannelModeSupported___BOOLEAN__GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReadValue___I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
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
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesCanProviderICanControllerProvider = 1;
    static const size_t FIELD___idx___U4 = 2;
    static const size_t FIELD___nativeMessageAvailableEvent___mscorlibSystemRuntimeInteropServicesNativeEventDispatcher = 3;
    static const size_t FIELD___nativeErrorEvent___mscorlibSystemRuntimeInteropServicesNativeEventDispatcher = 4;
    static const size_t FIELD___MessageReceived___GHIElectronicsTinyCLRDevicesCanMessageReceivedEventHandler = 5;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesCanErrorReceivedEventHandler = 6;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_CanMessage {
    static const size_t FIELD___data___SZARRAY_U1 = 1;
    static const size_t FIELD___ArbitrationId__BackingField___U4 = 2;
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

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_CanProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 1;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesCanProviderICanControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Can_Provider_DefaultCanControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___disposed___BOOLEAN = 2;
    static const size_t FIELD___idx___I4 = 3;

    static TinyCLR_Result Reset___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReadMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteMessages___I4__SZARRAY_GHIElectronicsTinyCLRDevicesCanCanMessage__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetBitTiming___VOID__GHIElectronicsTinyCLRDevicesCanCanBitTiming(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetExplicitFilters___VOID__SZARRAY_U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetGroupFilters___VOID__SZARRAY_U4__SZARRAY_U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_UnreadMessageCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_UnwrittenMessageCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_IsWritingAllowed___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadErrorCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_WriteErrorCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SourceClock___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeAcquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeRelease___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ReadBufferSize___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ReadBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_WriteBufferSize___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_WriteBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_DacChannel {
    static const size_t FIELD___channel___I4 = 1;
    static const size_t FIELD___disposed___BOOLEAN = 2;
    static const size_t FIELD___controller___GHIElectronicsTinyCLRDevicesDacDacController = 3;
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesDacProviderIDacControllerProvider = 4;
    static const size_t FIELD___LastWrittenValue__BackingField___I4 = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_DacController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesDacProviderIDacControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 2;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesDacProviderIDacControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___idx___I4 = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteValue___VOID__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_DisplayController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesDisplayProviderIDisplayControllerProvider = 1;
    static const size_t FIELD___ActiveSettings__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayControllerSettings = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings {
    static const size_t FIELD___Width__BackingField___U4 = 1;
    static const size_t FIELD___Height__BackingField___U4 = 2;
    static const size_t FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings {
    static const size_t FIELD___OutputEnableIsFixed__BackingField___BOOLEAN = 4;
    static const size_t FIELD___OutputEnablePolarity__BackingField___BOOLEAN = 5;
    static const size_t FIELD___PixelPolarity__BackingField___BOOLEAN = 6;
    static const size_t FIELD___PixelClockRate__BackingField___U4 = 7;
    static const size_t FIELD___HorizontalSyncPolarity__BackingField___BOOLEAN = 8;
    static const size_t FIELD___HorizontalSyncPulseWidth__BackingField___U4 = 9;
    static const size_t FIELD___HorizontalFrontPorch__BackingField___U4 = 10;
    static const size_t FIELD___HorizontalBackPorch__BackingField___U4 = 11;
    static const size_t FIELD___VerticalSyncPolarity__BackingField___BOOLEAN = 12;
    static const size_t FIELD___VerticalSyncPulseWidth__BackingField___U4 = 13;
    static const size_t FIELD___VerticalFrontPorch__BackingField___U4 = 14;
    static const size_t FIELD___VerticalBackPorch__BackingField___U4 = 15;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___idx___I4 = 2;

    static TinyCLR_Result get_Interface___GHIElectronicsTinyCLRDevicesDisplayDisplayInterface(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_SupportedDataFormats___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteString___VOID__STRING(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetParallelConfiguration___BOOLEAN__U4__U4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__BOOLEAN__BOOLEAN__BOOLEAN__U4__BOOLEAN__U4__U4__U4__BOOLEAN__U4__U4__U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetSpiConfiguration___BOOLEAN__U4__U4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__STRING(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 3;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayProviderIDisplayControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_SpiDisplayControllerSettings {
    static const size_t FIELD___SpiSelector__BackingField___STRING = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Enumeration_DeviceInformation {
    static const size_t FIELD___m_id___STRING = 1;
    static const size_t FIELD___m_isDefault___BOOLEAN = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeReader {
    static const size_t FIELD___timeout___I4 = 1;
    static const size_t FIELD___disposed___BOOLEAN = 2;
    static const size_t FIELD___port___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___pin___I4 = 4;

    static TinyCLR_Result NativeRead___STATIC___I4__U4__BYREF_BOOLEAN__SZARRAY_U4__I4__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeRead___STATIC___I4__U4__BOOLEAN__SZARRAY_U4__I4__I4__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioChangeWriter {
    static const size_t FIELD___pin___U4 = 1;
    static const size_t FIELD___disposed___BOOLEAN = 2;
    static const size_t FIELD___nativePointer___U4 = 3;

    static TinyCLR_Result NativeConstructor___BOOLEAN__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeDispose___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeIsActive___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeSet___VOID__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeSet___BOOLEAN__BOOLEAN__SZARRAY_U4__I4__I4__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeSet___VOID__BOOLEAN__SZARRAY_U4__I4__I4__U4__BOOLEAN__U4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesGpioProviderIGpioControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPin {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesGpioProviderIGpioPinProvider = 1;
    static const size_t FIELD___callbacks___GHIElectronicsTinyCLRDevicesGpioGpioPinValueChangedEventHandler = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPinValueChangedEventArgs {
    static const size_t FIELD___m_edge___GHIElectronicsTinyCLRDevicesGpioGpioPinEdge = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_GpioPulseReaderWriter {
    static const size_t FIELD___disposed___BOOLEAN = 1;
    static const size_t FIELD___timeout___I4 = 2;
    static const size_t FIELD___pulseLength___I4 = 3;
    static const size_t FIELD___pulseState___BOOLEAN = 4;
    static const size_t FIELD___echoState___BOOLEAN = 5;
    static const size_t FIELD___pulsePin___I4 = 6;
    static const size_t FIELD___echoPin___I4 = 7;
    static const size_t FIELD___mode___GHIElectronicsTinyCLRDevicesGpioGpioPulseReaderWriterMode = 8;
    static const size_t FIELD___driveMode___I4 = 9;

    static TinyCLR_Result NativeReadDrainTime___I8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeReadEcho___I8__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeFinalize___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___exclusivePins___mscorlibSystemCollectionsArrayList = 2;
    static const size_t FIELD___acquiredPins___mscorlibSystemCollectionsHashtable = 3;
    static const size_t FIELD___Name___STRING = 4;
    static const size_t FIELD___Index___U4 = 5;

    static TinyCLR_Result get_PinCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireNative___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID__I4(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioPinProvider {
    static const size_t FIELD_STATIC___s_eventListener___GHIElectronicsTinyCLRDevicesGpioProviderGpioPinEventListener = 4;

    static const size_t FIELD___m_syncLock___OBJECT = 1;
    static const size_t FIELD___m_disposed___BOOLEAN = 2;
    static const size_t FIELD___m_pinNumber___I4 = 3;
    static const size_t FIELD___m_driveMode___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinDriveMode = 4;
    static const size_t FIELD___m_lastOutputValue___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinValue = 5;
    static const size_t FIELD___m_callbacks___GHIElectronicsTinyCLRDevicesGpioProviderGpioPinProviderValueChangedEventHandler = 6;
    static const size_t FIELD___nativeProvider___I = 7;
    static const size_t FIELD___parent___GHIElectronicsTinyCLRDevicesGpioProviderDefaultGpioControllerProvider = 8;
    static const size_t FIELD___SharingMode__BackingField___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioSharingMode = 9;

    static TinyCLR_Result get_DebounceTimeout___mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_DebounceTimeout___VOID__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinValue(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___VOID__GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinValue(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDriveModeInternal___VOID__GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinDriveMode(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioPinEventListener {
    static const size_t FIELD___pinMap___mscorlibSystemCollectionsIDictionary = 1;
    static const size_t FIELD___dispatcher___mscorlibSystemRuntimeInteropServicesNativeEventDispatcher = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioPinProviderValueChangedEventArgs {
    static const size_t FIELD___m_edge___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinEdge = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_GpioProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 5;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesGpioProviderIGpioControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings {
    static const size_t FIELD___m_slaveAddress___I4 = 1;
    static const size_t FIELD___m_busSpeed___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed = 2;
    static const size_t FIELD___m_sharingMode___GHIElectronicsTinyCLRDevicesI2cI2cSharingMode = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesI2cProviderII2cControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cDevice {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesI2cProviderII2cDeviceProvider = 1;
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cManagedSoftwareDeviceProvider {
    static const size_t FIELD___writeAddress___U1 = 1;
    static const size_t FIELD___readAddress___U1 = 2;
    static const size_t FIELD___sda___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___scl___GHIElectronicsTinyCLRDevicesGpioGpioPin = 4;
    static const size_t FIELD___useSoftwarePullups___BOOLEAN = 5;
    static const size_t FIELD___start___BOOLEAN = 6;
    static const size_t FIELD___disposed___BOOLEAN = 7;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cNativeSoftwareDeviceProvider {
    static const size_t FIELD___address___U1 = 1;
    static const size_t FIELD___sda___I4 = 2;
    static const size_t FIELD___scl___I4 = 3;
    static const size_t FIELD___useSoftwarePullups___BOOLEAN = 4;
    static const size_t FIELD___disposed___BOOLEAN = 5;

    static TinyCLR_Result NativeWriteRead___STATIC___BOOLEAN__I4__I4__U1__BOOLEAN__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_U4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cSoftwareControllerProvider {
    static const size_t FIELD___controller___GHIElectronicsTinyCLRDevicesGpioGpioController = 1;
    static const size_t FIELD___sda___I4 = 2;
    static const size_t FIELD___scl___I4 = 3;
    static const size_t FIELD___useSoftwarePullups___BOOLEAN = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cSoftwareProvider {
    static const size_t FIELD___gpioController___GHIElectronicsTinyCLRDevicesGpioGpioController = 1;
    static const size_t FIELD___sda___I4 = 2;
    static const size_t FIELD___scl___I4 = 3;
    static const size_t FIELD___useSoftwarePullups___BOOLEAN = 4;
    static const size_t FIELD___i2cController___SZARRAY_GHIElectronicsTinyCLRDevicesI2cProviderII2cControllerProvider = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cTransferResult {
    static const size_t FIELD___Status___GHIElectronicsTinyCLRDevicesI2cI2cTransferStatus = 1;
    static const size_t FIELD___BytesTransferred___U4 = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___created___I4 = 2;
    static const size_t FIELD___isExclusive___BOOLEAN = 3;
    static const size_t FIELD___idx___I4 = 4;

    static TinyCLR_Result AcquireNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cDeviceProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___parent___GHIElectronicsTinyCLRDevicesI2cProviderDefaultI2cControllerProvider = 2;
    static const size_t FIELD___m_disposed___BOOLEAN = 3;
    static const size_t FIELD___m_settings___GHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings = 4;

    static TinyCLR_Result ReadInternal___VOID__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteInternal___VOID__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteReadInternal___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_I2cProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 6;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesI2cProviderII2cControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_ProviderI2cConnectionSettings {
    static const size_t FIELD___BusSpeed__BackingField___GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cBusSpeed = 1;
    static const size_t FIELD___SharingMode__BackingField___GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cSharingMode = 2;
    static const size_t FIELD___SlaveAddress__BackingField___I4 = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_ProviderI2cTransferResult {
    static const size_t FIELD___Status___GHIElectronicsTinyCLRDevicesI2cProviderProviderI2cTransferStatus = 1;
    static const size_t FIELD___BytesTransferred___U4 = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_LowLevelDevicesAggregateProvider {
    static const size_t FIELD___AdcControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesAdcProviderIAdcControllerProvider = 1;
    static const size_t FIELD___CanControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesCanProviderICanControllerProvider = 2;
    static const size_t FIELD___DacControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesDacProviderIDacControllerProvider = 3;
    static const size_t FIELD___DisplayControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesDisplayProviderIDisplayControllerProvider = 4;
    static const size_t FIELD___GpioControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesGpioProviderIGpioControllerProvider = 5;
    static const size_t FIELD___I2cControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesI2cProviderII2cControllerProvider = 6;
    static const size_t FIELD___PwmControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesPwmProviderIPwmControllerProvider = 7;
    static const size_t FIELD___SpiControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesSpiProviderISpiControllerProvider = 8;
    static const size_t FIELD___SdCardControllerProvider__BackingField___GHIElectronicsTinyCLRDevicesSdCardProviderISdCardControllerProvider = 9;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_LowLevelDevicesController {
    static const size_t FIELD_STATIC___DefaultProvider__BackingField___GHIElectronicsTinyCLRDevicesILowLevelDevicesAggregateProvider = 7;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___ActualFrequency__BackingField___R8 = 2;
    static const size_t FIELD___idx___I4 = 3;

    static TinyCLR_Result get_MaxFrequency___R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinFrequency___R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_PinCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DisablePin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result EnablePin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquirePin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleasePin___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDesiredFrequency___R8__R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetPulseParameters___VOID__I4__R8__BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result AcquireNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 8;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesPwmProviderIPwmControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_PwmController {
    static const size_t FIELD___m_provider___GHIElectronicsTinyCLRDevicesPwmProviderIPwmControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_PwmPin {
    static const size_t FIELD___m_pinNumber___I4 = 1;
    static const size_t FIELD___m_controller___GHIElectronicsTinyCLRDevicesPwmPwmController = 2;
    static const size_t FIELD___m_provider___GHIElectronicsTinyCLRDevicesPwmProviderIPwmControllerProvider = 3;
    static const size_t FIELD___m_started___BOOLEAN = 4;
    static const size_t FIELD___m_disposed___BOOLEAN = 5;
    static const size_t FIELD___m_dutyCycle___R8 = 6;
    static const size_t FIELD___m_polarity___GHIElectronicsTinyCLRDevicesPwmPwmPulsePolarity = 7;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 9;

    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;

    static TinyCLR_Result get_Now___mscorlibSystemDateTime(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_Now___VOID__mscorlibSystemDateTime(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesRtcProviderIRtcControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_DefaultSdCardControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___idx___I4 = 2;

    static TinyCLR_Result ReadSectors___I4__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteSectors___I4__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result EraseSectors___I4__I4__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetSectorMap___VOID__BYREF_SZARRAY_I4__BYREF_I4__BYREF_BOOLEAN(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 10;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesSdCardProviderISdCardControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_SdCardController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesSdCardProviderISdCardControllerProvider = 1;
    static const size_t FIELD___driveProvider___OBJECT = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_ErrorReceivedEventArgs {
    static const size_t FIELD___Error__BackingField___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialError = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_PinChangedEventArgs {
    static const size_t FIELD___PinChange__BackingField___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialPinChange = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice {
    static const size_t FIELD___stream___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialDeviceStream = 1;
    static const size_t FIELD___disposed___BOOLEAN = 2;
    static const size_t FIELD___BytesReceived__BackingField___U4 = 3;
    static const size_t FIELD___PortName__BackingField___STRING = 4;
    static const size_t FIELD___BaudRate__BackingField___U4 = 5;
    static const size_t FIELD___DataBits__BackingField___U2 = 6;
    static const size_t FIELD___Parity__BackingField___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialParity = 7;
    static const size_t FIELD___Handshake__BackingField___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialHandshake = 8;
    static const size_t FIELD___StopBits__BackingField___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialStopBitCount = 9;
    static const size_t FIELD___ReadTimeout__BackingField___mscorlibSystemTimeSpan = 10;
    static const size_t FIELD___WriteTimeout__BackingField___mscorlibSystemTimeSpan = 11;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___created___I4 = 2;
    static const size_t FIELD___isExclusive___BOOLEAN = 3;
    static const size_t FIELD___idx___I4 = 4;

    static TinyCLR_Result AcquireNative___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ReleaseNative___VOID(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiDeviceProvider {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___parent___GHIElectronicsTinyCLRDevicesSpiProviderDefaultSpiControllerProvider = 2;
    static const size_t FIELD___m_settings___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings = 3;
    static const size_t FIELD___m_disposed___BOOLEAN = 4;
    static const size_t FIELD___m_mskPin___I4 = 5;
    static const size_t FIELD___m_misoPin___I4 = 6;
    static const size_t FIELD___m_mosiPin___I4 = 7;
    static const size_t FIELD___m_spiBus___I4 = 8;

    static TinyCLR_Result ReadInternal___VOID__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result WriteInternal___VOID__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result TransferFullDuplexInternal___VOID__SZARRAY_U1__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result TransferSequentialInternal___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_ProviderSpiConnectionSettings {
    static const size_t FIELD___ChipSelectionLine__BackingField___I4 = 1;
    static const size_t FIELD___Mode__BackingField___GHIElectronicsTinyCLRDevicesSpiProviderProviderSpiMode = 2;
    static const size_t FIELD___DataBitLength__BackingField___I4 = 3;
    static const size_t FIELD___ClockFrequency__BackingField___I4 = 4;
    static const size_t FIELD___SharingMode__BackingField___GHIElectronicsTinyCLRDevicesSpiProviderProviderSpiSharingMode = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiProvider {
    static const size_t FIELD_STATIC___providers___mscorlibSystemCollectionsHashtable = 11;

    static const size_t FIELD___controllers___SZARRAY_GHIElectronicsTinyCLRDevicesSpiProviderISpiControllerProvider = 1;
    static const size_t FIELD___Name__BackingField___STRING = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiBusInfo {
    static const size_t FIELD___ChipSelectLineCount__BackingField___I4 = 1;
    static const size_t FIELD___MinClockFrequency__BackingField___I4 = 2;
    static const size_t FIELD___MaxClockFrequency__BackingField___I4 = 3;
    static const size_t FIELD___SupportedDataBitLengths__BackingField___SZARRAY_I4 = 4;

    static TinyCLR_Result ctor___VOID__I__I4(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings {
    static const size_t FIELD___m_chipSelectionLine___I4 = 1;
    static const size_t FIELD___m_dataBitLength___I4 = 2;
    static const size_t FIELD___m_clockFrequency___I4 = 3;
    static const size_t FIELD___m_mode___GHIElectronicsTinyCLRDevicesSpiSpiMode = 4;
    static const size_t FIELD___m_sharingMode___GHIElectronicsTinyCLRDevicesSpiSpiSharingMode = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiController {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesSpiProviderISpiControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiDevice {
    static const size_t FIELD___provider___GHIElectronicsTinyCLRDevicesSpiProviderISpiDeviceProvider = 1;
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiManagedSoftwareDeviceProvider {
    static const size_t FIELD___miso___GHIElectronicsTinyCLRDevicesGpioGpioPin = 1;
    static const size_t FIELD___mosi___GHIElectronicsTinyCLRDevicesGpioGpioPin = 2;
    static const size_t FIELD___sck___GHIElectronicsTinyCLRDevicesGpioGpioPin = 3;
    static const size_t FIELD___cs___GHIElectronicsTinyCLRDevicesGpioGpioPin = 4;
    static const size_t FIELD___captureOnRisingEdge___BOOLEAN = 5;
    static const size_t FIELD___clockIdleState___GHIElectronicsTinyCLRDevicesGpioGpioPinValue = 6;
    static const size_t FIELD___clockActiveState___GHIElectronicsTinyCLRDevicesGpioGpioPinValue = 7;
    static const size_t FIELD___disposed___BOOLEAN = 8;
    static const size_t FIELD___ConnectionSettings__BackingField___GHIElectronicsTinyCLRDevicesSpiProviderProviderSpiConnectionSettings = 9;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiSoftwareControllerProvider {
    static const size_t FIELD___controller___GHIElectronicsTinyCLRDevicesGpioGpioController = 1;
    static const size_t FIELD___miso___I4 = 2;
    static const size_t FIELD___mosi___I4 = 3;
    static const size_t FIELD___sck___I4 = 4;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_SpiSoftwareProvider {
    static const size_t FIELD___gpioController___GHIElectronicsTinyCLRDevicesGpioGpioController = 1;
    static const size_t FIELD___miso___I4 = 2;
    static const size_t FIELD___mosi___I4 = 3;
    static const size_t FIELD___sck___I4 = 4;
    static const size_t FIELD___spiController___SZARRAY_GHIElectronicsTinyCLRDevicesSpiProviderISpiControllerProvider = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SerialCommunication_SerialDevice__Stream {
    static const size_t FIELD___nativeProvider___I = 1;
    static const size_t FIELD___parent___GHIElectronicsTinyCLRDevicesSerialCommunicationSerialDevice = 2;
    static const size_t FIELD___providerId___STRING = 3;
    static const size_t FIELD___idx___U4 = 4;
    static const size_t FIELD___opened___BOOLEAN = 5;
    static const size_t FIELD___errorReceivedEvent___mscorlibSystemRuntimeInteropServicesNativeEventDispatcher = 6;
    static const size_t FIELD___pinChangedEvent___mscorlibSystemRuntimeInteropServicesNativeEventDispatcher = 7;
    static const size_t FIELD___ErrorReceived___GHIElectronicsTinyCLRDevicesSerialCommunicationErrorReceivedDelegate = 8;
    static const size_t FIELD___PinChanged___GHIElectronicsTinyCLRDevicesSerialCommunicationPinChangedDelegate = 9;

    static TinyCLR_Result get_ReadBufferSize___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_ReadBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_WriteBufferSize___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result set_WriteBufferSize___VOID__U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_UnreadCount___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_UnwrittenCount___U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearReadBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result ClearWriteBuffer___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeOpen___VOID__U4__U4__U4__U4__U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeClose___VOID__U4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeFlush___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeRead___I4__SZARRAY_U1__I4__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result NativeWrite___I4__SZARRAY_U1__I4__I4__I4(const TinyCLR_Interop_MethodData md);

    static TinyCLR_Result GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md);
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices;
