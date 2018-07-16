#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::AcquireNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto controller = field_id.Data.Numeric->I4;
    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;

    return provider->Acquire(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::ReleaseNative___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    return provider->Release(provider, controller);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::get_MaxFrequency___R8(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->R8 = provider->GetMaxFrequency(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::get_MinFrequency___R8(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->R8 = provider->GetMinFrequency(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::get_PinCount___I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = provider->GetPinCount(provider, controller);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::DisablePin___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto pinNumber = arg1.Data.Numeric->I4;

    provider->DisablePin(provider, controller, pinNumber);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::EnablePin___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto pinNumber = arg1.Data.Numeric->I4;

    provider->EnablePin(provider, controller, pinNumber);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::AcquirePin___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;
    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto pinNumber = arg1.Data.Numeric->I4;

    provider->AcquirePin(provider, controller, pinNumber);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::ReleasePin___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;
    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    auto pinNumber = arg1.Data.Numeric->I4;

    provider->ReleasePin(provider, controller, pinNumber);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::SetDesiredFrequency___R8__R8(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;

    fld = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___ActualFrequency__BackingField___R8);

    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);

    double desiredFrequency = arg1.Data.Numeric->R8;

    provider->SetDesiredFrequency(provider, controller, desiredFrequency);

    fld.Data.Numeric->R8 = desiredFrequency;

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->R8 = desiredFrequency;

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::SetPulseParameters___VOID__I4__R8__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, field_id;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::FIELD___idx___I4, field_id);

    auto provider = (const TinyCLR_Pwm_Provider*)fld.Data.Numeric->I;
    auto controller = field_id.Data.Numeric->I4;
    auto arg1 = TinyCLR_Interop_GetArguments(md, 1);
    auto arg2 = TinyCLR_Interop_GetArguments(md, 2);
    auto arg3 = TinyCLR_Interop_GetArguments(md, 3);

    int pinNumber = arg1.Data.Numeric->I4;
    double dutyCycle = arg2.Data.Numeric->R8;
    bool invert = arg3.Data.Numeric->I1 != 0;

    provider->SetPulseParameters(provider, controller, pinNumber, dutyCycle, invert);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_DefaultPwmControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto arg = TinyCLR_Interop_GetArguments(md, 0);

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto provider = (const TinyCLR_Pwm_Provider*)(arg.Data.Numeric->I4);

    int32_t count;

    if (provider->GetControllerCount(provider, count) == TinyCLR_Result::Success) {
        ret.Data.Numeric->I4 = count;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}
