#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->Acquire(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->Release(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_ChannelMode___GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)provider->GetChannelMode(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::set_ChannelMode___VOID__GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto arg = TinyCLR_Interop_GetArguments(md, 1);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->SetChannelMode(provider, controller, (TinyCLR_Adc_ChannelMode)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetChannelCount(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetMaxValue(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetMinValue(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetResolutionInBits(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::IsChannelModeSupported___BOOLEAN__GHIElectronicsTinyCLRDevicesAdcProviderProviderAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->IsChannelModeSupported(provider, controller, (TinyCLR_Adc_ChannelMode)(int32_t)arg.Data.Numeric->U4) ? 1 : 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->AcquireChannel(provider, controller, (int32_t)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->ReleaseChannel(provider, controller, (int32_t)arg.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::ReadValue___I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Adc_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_DefaultAdcControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    auto ret = TinyCLR_Interop_GetReturn(md);

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
