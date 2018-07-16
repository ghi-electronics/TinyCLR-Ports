#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::SetParallelConfiguration___BOOLEAN__U4__U4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__BOOLEAN__BOOLEAN__BOOLEAN__U4__BOOLEAN__U4__U4__U4__BOOLEAN__U4__U4__U4(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Display_ParallelConfiguration config;

    uint32_t width = TinyCLR_Interop_GetArguments(md, 1).Data.Numeric->U4;
    uint32_t height = TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->U4;
    auto type = (TinyCLR_Display_DataFormat)TinyCLR_Interop_GetArguments(md, 3).Data.Numeric->U4;

    config.DataEnableIsFixed = TinyCLR_Interop_GetArguments(md, 4).Data.Numeric->U1 != 0;
    config.DataEnablePolarity = TinyCLR_Interop_GetArguments(md, 5).Data.Numeric->U1 != 0;
    config.PixelPolarity = TinyCLR_Interop_GetArguments(md, 6).Data.Numeric->U1 != 0;
    config.PixelClockRate = TinyCLR_Interop_GetArguments(md, 7).Data.Numeric->U4;
    config.HorizontalSyncPolarity = TinyCLR_Interop_GetArguments(md, 8).Data.Numeric->U1 != 0;
    config.HorizontalSyncPulseWidth = TinyCLR_Interop_GetArguments(md, 9).Data.Numeric->U4;
    config.HorizontalFrontPorch = TinyCLR_Interop_GetArguments(md, 10).Data.Numeric->U4;
    config.HorizontalBackPorch = TinyCLR_Interop_GetArguments(md, 11).Data.Numeric->U4;
    config.VerticalSyncPolarity = TinyCLR_Interop_GetArguments(md, 12).Data.Numeric->U1 != 0;
    config.VerticalSyncPulseWidth = TinyCLR_Interop_GetArguments(md, 13).Data.Numeric->U4;
    config.VerticalFrontPorch = TinyCLR_Interop_GetArguments(md, 14).Data.Numeric->U4;
    config.VerticalBackPorch = TinyCLR_Interop_GetArguments(md, 15).Data.Numeric->U4;

    auto provider = (const TinyCLR_Display_Controller*)TinyCLR_Interop_GetManager(md, FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto res = provider->Acquire(provider);

    TinyCLR_Interop_ClrValue ret = TinyCLR_Interop_GetReturn(md);

    if (res == TinyCLR_Result::Success) {
        res = provider->SetConfiguration(provider, type, width, height, &config);

        if (res == TinyCLR_Result::Success) {
            res = provider->Enable(provider);

            ret.Data.Numeric->Boolean = (res == TinyCLR_Result::Success) ? true : false;
        }
        else {
            ret.Data.Numeric->Boolean = false;
        }
    }
    else {
        ret.Data.Numeric->Boolean = false;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::SetSpiConfiguration___BOOLEAN__U4__U4__GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat__STRING(const TinyCLR_Interop_MethodData md) {
    TinyCLR_Display_SpiConfiguration config;

    uint32_t width = TinyCLR_Interop_GetArguments(md, 1).Data.Numeric->U4;
    uint32_t height = TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->U4;
    auto type = (TinyCLR_Display_DataFormat)TinyCLR_Interop_GetArguments(md, 3).Data.Numeric->U4;

    config.SpiSelector = TinyCLR_Interop_GetArguments(md, 4).Data.String.Data;

    auto provider = (const TinyCLR_Display_Controller*)TinyCLR_Interop_GetManager(md, FIELD___nativeProvider___I);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto res = provider->Acquire(provider);

    TinyCLR_Interop_ClrValue ret = TinyCLR_Interop_GetReturn(md);

    if (res == TinyCLR_Result::Success) {
        res = provider->SetConfiguration(provider, type, width, height, &config);

        if (res == TinyCLR_Result::Success) {
            res = provider->Enable(provider);

            ret.Data.Numeric->Boolean = (res == TinyCLR_Result::Success) ? true : false;
        }
        else {
            ret.Data.Numeric->Boolean = false;
        }
    }
    else {
        ret.Data.Numeric->Boolean = false;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::get_Interface___GHIElectronicsTinyCLRDevicesDisplayDisplayInterface(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Display_Controller*)TinyCLR_Interop_GetManager(md, FIELD___nativeProvider___I);

    TinyCLR_Display_InterfaceType type;

    const TinyCLR_Display_DataFormat* supportedDataFormats;

    size_t supportedDataFormatCount;

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto res = provider->GetCapabilities(provider, type, supportedDataFormats, supportedDataFormatCount);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = (int32_t)type;

    return res;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::get_SupportedDataFormats___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Display_Controller*)TinyCLR_Interop_GetManager(md, FIELD___nativeProvider___I);

    TinyCLR_Display_InterfaceType type;

    const TinyCLR_Display_DataFormat* supportedDataFormats;

    size_t supportedDataFormatCount;

    auto ret = TinyCLR_Interop_GetReturn(md);

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    auto res = provider->GetCapabilities(provider, type, supportedDataFormats, supportedDataFormatCount);

    auto interop = (const TinyCLR_Interop_Manager*)md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager);

    const TinyCLR_Interop_ClrObject* self;

    TinyCLR_Interop_ClrValue arr;
    TinyCLR_Interop_ClrTypeId idx;

    if (res == TinyCLR_Result::Success) {
        interop->GetThisObject(interop, md.Stack, self);

        interop->FindType(interop, "GHIElectronics.TinyCLR.Devices", "GHIElectronics.TinyCLR.Devices.Display", "DisplayDataFormat", idx);

        auto top = TinyCLR_Interop_GetReturn(md);

        interop->GetField(interop, top.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat, arr);

        interop->CreateArray(interop, supportedDataFormatCount, idx, arr);

        auto p = (TinyCLR_Display_DataFormat*)(arr.Data.SzArray.Data);

        for (auto i = 0; i < supportedDataFormatCount; i++)
            p[i] = supportedDataFormats[i];
    }

    return res;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::WriteString___VOID__STRING(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_Display_Controller*)TinyCLR_Interop_GetManager(md, FIELD___nativeProvider___I);

    auto data = TinyCLR_Interop_GetArguments(md, 1).Data.String;

    auto stringType = TinyCLR_Interop_GetArguments(md, 1).Data.String;

    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::FIELD___idx___I4).Data.Numeric->I;

    return provider->WriteString(provider, stringType.Data, stringType.Length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Display_Provider_DefaultDisplayControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {

    return TinyCLR_Result::InvalidOperation;
}