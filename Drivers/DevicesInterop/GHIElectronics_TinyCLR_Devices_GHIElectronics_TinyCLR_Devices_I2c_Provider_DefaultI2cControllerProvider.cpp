#include "GHIElectronics_TinyCLR_Devices.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::AcquireNative___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    int32_t channel = channelArg.Data.Numeric->I4;

    auto provider = (const TinyCLR_I2c_Provider*)fld.Data.Numeric->I;

    return provider->Acquire(provider, channel);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::ReleaseNative___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, channelArg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, channelArg);

    auto provider = (const TinyCLR_I2c_Provider*)fld.Data.Numeric->I;
    int32_t channel = channelArg.Data.Numeric->I4;

    return provider->Release(provider, channel);
}
