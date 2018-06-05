#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

const TinyCLR_Dac_Provider* TinyCLR_Dac_GetProvider(const TinyCLR_Interop_MethodData md, TinyCLR_Interop_ClrValue& ret, TinyCLR_Interop_ClrValue& arg, int argIndex) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetReturn(interop, md.Stack, ret);

    if (argIndex >= 0)
        interop->GetArgument(interop, md.Stack, argIndex, arg);

    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::FIELD___nativeProvider___I, fld);

    return (const TinyCLR_Dac_Provider*)fld.Data.Numeric->I;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, -1);

    return provider->Acquire(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, -1);

    return provider->Release(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, -1);

    ret.Data.Numeric->I4 = (int32_t)provider->GetChannelCount(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, -1);

    ret.Data.Numeric->I4 = (int32_t)provider->GetMaxValue(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, -1);

    ret.Data.Numeric->I4 = (int32_t)provider->GetMinValue(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, -1);

    ret.Data.Numeric->I4 = (int32_t)provider->GetResolutionInBits(provider);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::AcquireChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, 1);

    if (provider->AcquireChannel(provider, arg.Data.Numeric->I4) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::ReleaseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, 1);

    if (provider->ReleaseChannel(provider, arg.Data.Numeric->I4) != TinyCLR_Result::Success)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DefaultDacControllerProvider::WriteValue___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Interop_ClrValue ret, arg;

    auto provider = TinyCLR_Dac_GetProvider(md, ret, arg, 1);

    int32_t arg1 = arg.Data.Numeric->I4;

    TinyCLR_Dac_GetProvider(md, ret, arg, 2);

    int32_t arg2 = arg.Data.Numeric->I4;

    if (provider->WriteValue(provider, arg1, arg2) != TinyCLR_Result::Success) {
        return TinyCLR_Result::InvalidOperation;
    }

    return TinyCLR_Result::Success;
}
