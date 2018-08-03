#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

#ifdef INCLUDE_DAC

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetChannelCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetResolutionInBits(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMinValue(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMaxValue(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    return api->OpenChannel(api, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg);

    return api->CloseChannel(api, arg.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::Write___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);

    return api->WriteValue(api, arg0.Data.Numeric->I4, arg1.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Dac_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}

#else //NO INCLUDE_DAC

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::Write___VOID__I4__I4(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    return TinyCLR_Result::NotImplemented;
}

#endif //INCLUDE_DAC
