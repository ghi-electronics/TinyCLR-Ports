#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::get_ChannelCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));        
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = api->GetChannelCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = api->GetResolutionInBits(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::get_MinValue___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = api->GetMinValue(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::get_MaxValue___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = api->GetMaxValue(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::IsChannelModeSupported___BOOLEAN__GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArgument(md, 0);
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->Boolean = api->IsChannelModeSupported(api, reintrepret_cast<TinyCLR_Adc_ChannelMode>(arg1.Data.Numeric->U4));

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::GetChannelMode___GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = static_cast<int32_t>(api->GetChannelMode(api));

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::SetChannelMode___VOID__GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArgument(md, 0);

    return api->SetChannelMode(api, static_cast<TinyCLR_Adc_ChannelMode>(arg1.Data.Numeric->U4));
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArgument(md, 0);

    return api->OpenChannel(api, arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArgument(md, 0);

    return api->CloseChannel(api, arg1.Data.Numeric->U4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::Read___I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    auto arg1 = TinyCLR_Interop_GetArgument(md, 0);
    auto ret = TinyCLR_Interop_GetReturn(md);

    return api->ReadChannel(api, arg1.Data.Numeric->U4, ret.Data.Numeric->I4);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Adc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
