#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue field_provider, field_controller;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___nativeProvider___I, field_provider);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___idx___I4, field_controller);

    auto provider = (const TinyCLR_Spi_Provider*)field_provider.Data.Numeric->I;
    auto controller = field_controller.Data.Numeric->I4;

    return provider->Acquire(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Provider*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue field_provider, field_controller;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___nativeProvider___I, field_provider);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::FIELD___idx___I4, field_controller);

    auto provider = (const TinyCLR_Spi_Provider*)field_provider.Data.Numeric->I;
    auto controller = field_controller.Data.Numeric->I4;

    return provider->Release(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_DefaultSpiControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto provider = (const TinyCLR_Spi_Provider*)(arg.Data.Numeric->I4);

    int32_t count;

    if (provider->GetControllerCount(provider, count) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = count;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}