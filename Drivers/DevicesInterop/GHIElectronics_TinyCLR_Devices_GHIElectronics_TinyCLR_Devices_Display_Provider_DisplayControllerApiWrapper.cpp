#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Enable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Enable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Disable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Disable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::DrawBuffer___VOID__I4__I4__I4__I4__SZARRAY_U1__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4, arg5;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 5, arg5);

    auto x = arg0.Data.Numeric->I4;
    auto y = arg1.Data.Numeric->I4;
    auto w = arg2.Data.Numeric->I4;
    auto h = arg3.Data.Numeric->I4;
    auto offset = arg5.Data.Numeric->I4;

    auto data = reinterpret_cast<uint8_t*>(arg4.Data.SzArray.Data);

    data += offset;

    return api->DrawBuffer(api, x, y, w, h, data);

}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::DrawString___VOID__STRING(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto stringType = arg0.Data.String;

    return api->DrawString(api, stringType.Data, stringType.Length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::get_Interface___GHIElectronicsTinyCLRDevicesDisplayDisplayInterface(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Display_InterfaceType type;

    const TinyCLR_Display_DataFormat* supportedDataFormats;

    size_t supportedDataFormatCount;

    auto res = api->GetCapabilities(api, type, supportedDataFormats, supportedDataFormatCount);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = static_cast<int32_t>(type);

    return res;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::get_SupportedDataFormats___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Display_InterfaceType type;

    const TinyCLR_Display_DataFormat* supportedDataFormats;

    size_t supportedDataFormatCount;

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    auto res = api->GetCapabilities(api, type, supportedDataFormats, supportedDataFormatCount);

    auto interop = md.InteropManager;

    const TinyCLR_Interop_ClrObject* self;

    TinyCLR_Interop_ClrValue arr;
    TinyCLR_Interop_ClrTypeId idx;

    if (res == TinyCLR_Result::Success) {
        interop->GetThisObject(interop, md.Stack, self);

        interop->FindType(interop, "GHIElectronics.TinyCLR.Devices", "GHIElectronics.TinyCLR.Devices.Display", "DisplayDataFormat", idx);

        interop->GetField(interop, ret.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat, arr);

        interop->CreateArray(interop, supportedDataFormatCount, idx, arr);

        auto p = reinterpret_cast<TinyCLR_Display_DataFormat*>(arr.Data.SzArray.Data);

        for (auto i = 0; i < supportedDataFormatCount; i++)
            p[i] = supportedDataFormats[i];
    }

    return res;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::SetParallelConfiguration___VOID__I4__I4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__BOOLEAN__BOOLEAN__BOOLEAN__I4__BOOLEAN__I4__I4__I4__BOOLEAN__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Display_ParallelConfiguration config;

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 5, arg5);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 6, arg6);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 7, arg7);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 8, arg8);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 9, arg9);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 10, arg10);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 11, arg11);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 12, arg12);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 13, arg13);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 14, arg14);

    uint32_t width = arg0.Data.Numeric->U4;
    uint32_t height = arg1.Data.Numeric->U4;

    auto type = static_cast<TinyCLR_Display_DataFormat>(arg2.Data.Numeric->U4);

    config.DataEnableIsFixed = arg3.Data.Numeric->U1 != 0;
    config.DataEnablePolarity = arg4.Data.Numeric->U1 != 0;
    config.PixelPolarity = arg5.Data.Numeric->U1 != 0;
    config.PixelClockRate = arg6.Data.Numeric->U4;
    config.HorizontalSyncPolarity = arg7.Data.Numeric->U1 != 0;
    config.HorizontalSyncPulseWidth = arg8.Data.Numeric->U4;
    config.HorizontalFrontPorch = arg9.Data.Numeric->U4;
    config.HorizontalBackPorch = arg10.Data.Numeric->U4;
    config.VerticalSyncPolarity = arg11.Data.Numeric->U1 != 0;
    config.VerticalSyncPulseWidth = arg12.Data.Numeric->U4;
    config.VerticalFrontPorch = arg13.Data.Numeric->U4;
    config.VerticalBackPorch = arg14.Data.Numeric->U4;

    return api->SetConfiguration(api, type, width, height, &config);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::SetSpiConfiguration___VOID__I4__I4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__STRING(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3;

    TinyCLR_Display_SpiConfiguration config;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);

    uint32_t width = arg0.Data.Numeric->U4;
    uint32_t height = arg1.Data.Numeric->U4;
    auto type = static_cast<TinyCLR_Display_DataFormat>(arg2.Data.Numeric->U4);

    config.ApiName = arg3.Data.String.Data;

    return api->SetConfiguration(api, type, width, height, &config);

}
