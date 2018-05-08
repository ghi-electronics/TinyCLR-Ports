#include "GHIElectronics_TinyCLR_Devices.h"

#define TIME_CONVERSION__TO_MILLISECONDS    10000

void IsrProcedure(const TinyCLR_Gpio_Provider* self, int32_t pin, TinyCLR_Gpio_PinValue pinState) {
    auto interopProvider = reinterpret_cast<const TinyCLR_Interop_Provider*>(apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::InteropProvider));

    if (interopProvider != nullptr)
        interopProvider->RaiseEvent(interopProvider, "GHIElectronics.TinyCLR.NativeEventNames.Gpio.ValueChanged", self->Parent->Name, self->Index, (uint64_t)pin, (uint64_t)(pinState == TinyCLR_Gpio_PinValue::High), 0);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioPinProvider::get_DebounceTimeout___mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;

    uint32_t portId = 0;
    int64_t value;
    uint32_t pin;

    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    TinyCLR_Interop_ClrValue fld;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    fld = TinyCLR_Interop_GetField(md, FIELD___m_pinNumber___I4);

    pin = fld.Data.Numeric->U4;

    value = (int64_t)provider->GetDebounceTimeout(provider, pin) * TIME_CONVERSION__TO_MILLISECONDS;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I8 = (int64_t)value;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioPinProvider::set_DebounceTimeout___VOID__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;

    uint32_t portId = 0;
    int64_t value;
    uint32_t pin;


    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);
    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    TinyCLR_Interop_ClrValue fld;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    value = (int64_t)arg.Data.Numeric->I8 / TIME_CONVERSION__TO_MILLISECONDS;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_pinNumber___I4);

    pin = fld.Data.Numeric->U4;

    if (provider->SetDebounceTimeout(provider, pin, value) != TinyCLR_Result::Success) {
        return TinyCLR_Result::ArgumentInvalid;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioPinProvider::Read___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinValue(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;

    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);
    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    TinyCLR_Interop_ClrValue fld;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    TinyCLR_Gpio_PinValue value;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_pinNumber___I4);

    provider->Read(provider, fld.Data.Numeric->U4, value);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)value;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioPinProvider::Write___VOID__GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinValue(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Result result = TinyCLR_Result::Success;

    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);
    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    TinyCLR_Interop_ClrValue fld;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_disposed___BOOLEAN);

    if (fld.Data.Numeric->I1 != 0) {
        return TinyCLR_Result::Disposed;
    }

    fld = TinyCLR_Interop_GetField(md, FIELD___m_lastOutputValue___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinValue);

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    fld.Data.Numeric->I4 = arg.Data.Numeric->I4;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_driveMode___GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinDriveMode);

    auto pinDriveMode = (TinyCLR_Gpio_PinDriveMode)fld.Data.Numeric->I4;

    // If the current drive mode is set to output, write the value to the pin.
    if (pinDriveMode == TinyCLR_Gpio_PinDriveMode::Output) {
        fld = TinyCLR_Interop_GetField(md, FIELD___m_pinNumber___I4);

        provider->Write(provider, fld.Data.Numeric->I4, arg.Data.Numeric->I4 ? TinyCLR_Gpio_PinValue::High : TinyCLR_Gpio_PinValue::Low);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Gpio_Provider_DefaultGpioPinProvider::SetDriveModeInternal___VOID__GHIElectronicsTinyCLRDevicesGpioProviderProviderGpioPinDriveMode(const TinyCLR_Interop_MethodData md) {
    int32_t portId = 0;

    TinyCLR_Gpio_PinDriveMode driveMode;
    TinyCLR_Gpio_PinValue pinValue;

    bool keepPinState = false;

    auto provider = (const TinyCLR_Gpio_Provider*)TinyCLR_Interop_GetProvider(md, FIELD___nativeProvider___I);

    auto result = TinyCLR_Result::NotSupported;

    if (provider == nullptr) return TinyCLR_Result::ArgumentNull;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    driveMode = (TinyCLR_Gpio_PinDriveMode)arg.Data.Numeric->I4;

    TinyCLR_Interop_ClrValue fld;

    fld = TinyCLR_Interop_GetField(md, FIELD___m_pinNumber___I4);

    portId = fld.Data.Numeric->U4;

    switch (driveMode) {
    case TinyCLR_Gpio_PinDriveMode::Output:
        if (provider->GetDriveMode(provider, portId) == TinyCLR_Gpio_PinDriveMode::Input || provider->GetDriveMode(provider, portId) == TinyCLR_Gpio_PinDriveMode::InputPullUp || provider->GetDriveMode(provider, portId) == TinyCLR_Gpio_PinDriveMode::InputPullDown) {
            keepPinState = true;

            provider->Read(provider, portId, pinValue);
        }

        result = provider->SetDriveMode(provider, portId, TinyCLR_Gpio_PinDriveMode::Output);

        if (result == TinyCLR_Result::Success && keepPinState == true)
            result = provider->Write(provider, portId, pinValue);
        break;

    case TinyCLR_Gpio_PinDriveMode::Input:
        result = provider->SetDriveMode(provider, portId, TinyCLR_Gpio_PinDriveMode::Input);
        goto SetIsr;

    case TinyCLR_Gpio_PinDriveMode::InputPullUp:
        result = provider->SetDriveMode(provider, portId, TinyCLR_Gpio_PinDriveMode::InputPullUp);
        goto SetIsr;

    case TinyCLR_Gpio_PinDriveMode::InputPullDown:
        result = provider->SetDriveMode(provider, portId, TinyCLR_Gpio_PinDriveMode::InputPullDown);
        goto SetIsr;

    SetIsr:
        fld = TinyCLR_Interop_GetField(md, FIELD___m_callbacks___GHIElectronicsTinyCLRDevicesGpioProviderGpioPinProviderValueChangedEventHandler);
        provider->SetValueChangedHandler(provider, portId, fld.Object != NULL ? IsrProcedure : nullptr);

        break;

    default:
        return TinyCLR_Result::ArgumentInvalid;
    }

    if (result != TinyCLR_Result::Success) {
        auto ret = TinyCLR_Interop_GetReturn(md);

        ret.Data.Numeric->Boolean = false;

        return TinyCLR_Result::InvalidOperation;
    }

    return result;
}
