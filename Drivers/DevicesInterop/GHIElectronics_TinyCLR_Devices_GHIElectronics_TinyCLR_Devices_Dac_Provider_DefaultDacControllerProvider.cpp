#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

#ifdef INCLUDE_DAC

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->Acquire(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->Release(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)provider->GetChannelCount(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)provider->GetMaxValue(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)provider->GetMinValue(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)provider->GetResolutionInBits(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->AcquireChannel(provider, controller, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto arg = TinyCLR_Interop_GetArguments(md, 1);

    return provider->ReleaseChannel(provider, controller, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::WriteValue___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Dac_Provider*)TinyCLR_Interop_GetProvider(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I);
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    int32_t arg1 = TinyCLR_Interop_GetArguments(md, 1).Data.Numeric->I4;
    int32_t arg2 = TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->I4;

    return provider->WriteValue(provider, controller, arg1, arg2);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto provider = (const TinyCLR_Dac_Provider*)(arg.Data.Numeric->I4);

    int32_t count;

    if (provider->GetControllerCount(provider, count) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = count;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}

#else //INCLUDE_DAC

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::WriteValue___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}
#endif //INCLUDE_DAC