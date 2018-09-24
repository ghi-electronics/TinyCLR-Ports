#include "GHIElectronics_TinyCLR_Devices_Display.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"
#include "../Spi/GHIElectronics_TinyCLR_Devices_Spi.h"
#include "../I2c/GHIElectronics_TinyCLR_Devices_I2c.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Enable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Enable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Disable___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Disable(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::DrawBuffer___VOID__I4__I4__I4__I4__SZARRAY_U1__I4(const TinyCLR_Interop_MethodData md) {
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::DrawPixel___VOID__I4__I4__I8(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto x = arg0.Data.Numeric->U4;
    auto y = arg1.Data.Numeric->U4;
    auto color = arg2.Data.Numeric->U8;

    return api->DrawPixel(api, x, y, color);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::DrawString___VOID__STRING(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    auto stringType = arg0.Data.String;

    return api->DrawString(api, stringType.Data, stringType.Length);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::get_Interface___GHIElectronicsTinyCLRDevicesDisplayDisplayInterface(const TinyCLR_Interop_MethodData md) {
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::get_SupportedDataFormats___SZARRAY_GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat(const TinyCLR_Interop_MethodData md) {
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

        interop->GetField(interop, ret.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat, arr);

        interop->CreateArray(interop, supportedDataFormatCount, idx, arr);

        auto p = reinterpret_cast<TinyCLR_Display_DataFormat*>(arr.Data.SzArray.Data);

        for (auto i = 0; i < supportedDataFormatCount; i++)
            p[i] = supportedDataFormats[i];
    }

    return res;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::SetConfiguration___VOID__GHIElectronicsTinyCLRDevicesDisplayParallelDisplayControllerSettings(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Display_ParallelConfiguration config;

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14;
    TinyCLR_Interop_ClrValue cfg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, cfg);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___Width__BackingField___I4, arg0);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___Height__BackingField___I4, arg1);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat, arg2);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___DataEnableIsFixed__BackingField___BOOLEAN, arg3);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___DataEnablePolarity__BackingField___BOOLEAN, arg4);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___PixelPolarity__BackingField___BOOLEAN, arg5);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___PixelClockRate__BackingField___I4, arg6);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___HorizontalSyncPolarity__BackingField___BOOLEAN, arg7);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___HorizontalSyncPulseWidth__BackingField___I4, arg8);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___HorizontalFrontPorch__BackingField___I4, arg9);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___HorizontalBackPorch__BackingField___I4, arg10);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___VerticalSyncPolarity__BackingField___BOOLEAN, arg11);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___VerticalSyncPulseWidth__BackingField___I4, arg12);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___VerticalFrontPorch__BackingField___I4, arg13);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_ParallelDisplayControllerSettings::FIELD___VerticalBackPorch__BackingField___I4, arg14);

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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::SetConfiguration___VOID__GHIElectronicsTinyCLRDevicesDisplaySpiDisplayControllerSettings(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Display_SpiConfiguration config;
    TinyCLR_Spi_Settings settings;

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4;
    TinyCLR_Interop_ClrValue cfg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, cfg);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___Width__BackingField___I4, arg0);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___Height__BackingField___I4, arg1);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat, arg2);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_SpiDisplayControllerSettings::FIELD___ApiName__BackingField___STRING, arg3);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_SpiDisplayControllerSettings::FIELD___Settings__BackingField___GHIElectronicsTinyCLRDevicesSpiGHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings, arg4);

    TinyCLR_Interop_ClrValue modeFld, clockFrequencyFld, dataBitLengthFld, chipSelectTypeFld, chipSelectLineFld, chipSelectSetupTimeFld, chipSelectHoldTimeFld, chipSelectActiveStateFld;
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___Mode__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiMode, modeFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ClockFrequency__BackingField___I4, clockFrequencyFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___DataBitLength__BackingField___I4, dataBitLengthFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectType__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiChipSelectType, chipSelectTypeFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectLine__BackingField___I4, chipSelectLineFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectSetupTime__BackingField___mscorlibSystemTimeSpan, chipSelectSetupTimeFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectHoldTime__BackingField___mscorlibSystemTimeSpan, chipSelectHoldTimeFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectActiveState__BackingField___BOOLEAN, chipSelectActiveStateFld);

    uint32_t width = arg0.Data.Numeric->U4;
    uint32_t height = arg1.Data.Numeric->U4;
    auto type = static_cast<TinyCLR_Display_DataFormat>(arg2.Data.Numeric->U4);

    settings.Mode = static_cast<TinyCLR_Spi_Mode>(modeFld.Data.Numeric->I4);
    settings.ClockFrequency = clockFrequencyFld.Data.Numeric->I4;
    settings.DataBitLength = dataBitLengthFld.Data.Numeric->I4;
    settings.ChipSelectType = static_cast<TinyCLR_Spi_ChipSelectType>(chipSelectTypeFld.Data.Numeric->I4);
    settings.ChipSelectLine = chipSelectLineFld.Data.Numeric->I4;
    settings.ChipSelectSetupTime = chipSelectSetupTimeFld.Data.Numeric->I8;
    settings.ChipSelectHoldTime = chipSelectHoldTimeFld.Data.Numeric->I8;
    settings.ChipSelectActiveState = chipSelectActiveStateFld.Data.Numeric->Boolean;

    config.ApiName = arg3.Data.String.Data;
    config.Settings = &settings;

    return api->SetConfiguration(api, type, width, height, &config);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_Provider_DisplayControllerApiWrapper::SetConfiguration___VOID__GHIElectronicsTinyCLRDevicesDisplayI2cDisplayControllerSettings(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Display_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Display_I2cConfiguration config;
    TinyCLR_I2c_Settings settings;

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4;
    TinyCLR_Interop_ClrValue cfg;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, cfg);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___Width__BackingField___I4, arg0);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___Height__BackingField___I4, arg1);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_DisplayControllerSettings::FIELD___DataFormat__BackingField___GHIElectronicsTinyCLRDevicesDisplayDisplayDataFormat, arg2);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_I2cDisplayControllerSettings::FIELD___ApiName__BackingField___STRING, arg3);
    md.InteropManager->GetField(md.InteropManager, cfg.Object, Interop_GHIElectronics_TinyCLR_Devices_Display_GHIElectronics_TinyCLR_Devices_Display_I2cDisplayControllerSettings::FIELD___Settings__BackingField___GHIElectronicsTinyCLRDevicesI2cGHIElectronicsTinyCLRDevicesI2cI2cConnectionSettings, arg4);

    TinyCLR_Interop_ClrValue slaveAddressFld, addressFormatFld, busSpeedFld;
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___SlaveAddress__BackingField___I4, slaveAddressFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___AddressFormat__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cAddressFormat, addressFormatFld);
    md.InteropManager->GetField(md.InteropManager, arg4.Object, Interop_GHIElectronics_TinyCLR_Devices_I2c_GHIElectronics_TinyCLR_Devices_I2c_I2cConnectionSettings::FIELD___BusSpeed__BackingField___GHIElectronicsTinyCLRDevicesI2cI2cBusSpeed, busSpeedFld);

    uint32_t width = arg0.Data.Numeric->U4;
    uint32_t height = arg1.Data.Numeric->U4;
    auto type = static_cast<TinyCLR_Display_DataFormat>(arg2.Data.Numeric->U4);

    settings.SlaveAddress = slaveAddressFld.Data.Numeric->I4;
    settings.AddressFormat = static_cast<TinyCLR_I2c_AddressFormat>(addressFormatFld.Data.Numeric->I4);
    settings.BusSpeed = static_cast<TinyCLR_I2c_BusSpeed>(busSpeedFld.Data.Numeric->I4);

    config.ApiName = arg3.Data.String.Data;
    config.Settings = &settings;

    return api->SetConfiguration(api, type, width, height, &config);
}
