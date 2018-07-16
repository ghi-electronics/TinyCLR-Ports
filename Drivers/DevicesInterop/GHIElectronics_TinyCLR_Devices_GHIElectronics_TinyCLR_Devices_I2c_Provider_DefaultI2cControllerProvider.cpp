#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_I2c_Controller*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    return provider->Acquire(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;

    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_I2c_Controller*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    return provider->Release(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_Provider_DefaultI2cControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {

    return TinyCLR_Result::InvalidOperation;
}
