#include "GHIElectronics_TinyCLR_Devices.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetChannelCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::get_MinFrequency___R8(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMinFrequency(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::get_MaxFrequency___R8(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->R8 = api->GetMaxFrequency(api);

    return TinyCLR_Result::Success;

}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto channel = arg0.Data.Numeric->I4;

    return  api->OpenChannel(api, channel);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto channel = arg0.Data.Numeric->I4;

    return  api->CloseChannel(api, channel);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::EnableChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto channel = arg0.Data.Numeric->I4;

    return  api->EnableChannel(api, channel);

}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::DisableChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto channel = arg0.Data.Numeric->I4;

    return  api->DisableChannel(api, channel);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::SetPulseParameters___VOID__I4__R8__GHIElectronicsTinyCLRDevicesPwmPwmPulsePolarity(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto channel = arg0.Data.Numeric->I4;
    auto dutyCycle = arg1.Data.Numeric->R8;
    auto polarity = static_cast<TinyCLR_Pwm_PulsePolarity>(arg2.Data.Numeric->I4);

    return  api->SetPulseParameters(api, channel, dutyCycle, polarity);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::SetDesiredFrequency___R8__R8(const TinyCLR_Interop_MethodData md) {

    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    double frequencyInOut = arg0.Data.Numeric->R8;

    auto result = api->SetDesiredFrequency(api, frequencyInOut);

    ret.Data.Numeric->R8 = frequencyInOut;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Pwm_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
