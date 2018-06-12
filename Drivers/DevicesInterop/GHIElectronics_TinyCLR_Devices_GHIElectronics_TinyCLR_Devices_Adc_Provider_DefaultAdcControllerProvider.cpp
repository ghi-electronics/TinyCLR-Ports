#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

const TinyCLR_Adc_Provider* TinyCLR_Adc_GetProvider(const TinyCLR_Interop_MethodData md, TinyCLR_Interop_ClrValue& ret, TinyCLR_Interop_ClrValue& arg, int argIndex) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetReturn(interop, md.Stack, ret);

    if (argIndex >= 0)
        interop->GetArgument(interop, md.Stack, argIndex, arg);

    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I, fld);

    return (const TinyCLR_Adc_Provider*)fld.Data.Numeric->I;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->Acquire(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->Release(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_ChannelMode___GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    ret.Data.Numeric->I4 = (int32_t)provider->GetChannelMode(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::set_ChannelMode___VOID__GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, 1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->SetChannelMode(provider, controller, (TinyCLR_Adc_ChannelMode)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    ret.Data.Numeric->I4 = provider->GetChannelCount(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    ret.Data.Numeric->I4 = provider->GetMaxValue(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    ret.Data.Numeric->I4 = provider->GetMinValue(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, -1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    ret.Data.Numeric->I4 = provider->GetResolutionInBits(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::IsChannelModeSupported___BOOLEAN__GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, 1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    ret.Data.Numeric->I4 = provider->IsChannelModeSupported(provider, controller, (TinyCLR_Adc_ChannelMode)(int32_t)arg.Data.Numeric->U4) ? 1 : 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, 1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->AcquireChannel(provider, controller, (int32_t)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, 1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->ReleaseChannel(provider, controller, (int32_t)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::ReadValue___I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Adc_GetProvider(md, ret, arg, 1);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    int32_t value;

    if (provider->ReadValue(provider, controller, (int32_t)arg.Data.Numeric->U4, value) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = value;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto provider = (const TinyCLR_Adc_Provider*)(arg.Data.Numeric->I4);

    int32_t count;

    if (provider->GetControllerCount(provider, count) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = count;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}
