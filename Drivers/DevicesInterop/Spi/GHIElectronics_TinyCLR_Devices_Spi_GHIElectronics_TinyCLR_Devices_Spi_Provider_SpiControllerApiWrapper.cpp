#include "GHIElectronics_TinyCLR_Devices_Spi.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_ChipSelectLineCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetChipSelectLineCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_MinClockFrequency___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMinClockFrequency(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_MaxClockFrequency___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMaxClockFrequency(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_SupportedDataBitLengths___SZARRAY_I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    const size_t MAX_SUPPORT_ARRAY_SIZE = 10;

    size_t ptrLen = MAX_SUPPORT_ARRAY_SIZE;

    uint32_t ptr[MAX_SUPPORT_ARRAY_SIZE];

    auto result = api->GetSupportedDataBitLengths(api, ptr, ptrLen);

    auto interop = md.InteropManager;

    ptrLen = ptrLen > MAX_SUPPORT_ARRAY_SIZE ? MAX_SUPPORT_ARRAY_SIZE : ptrLen;

    TinyCLR_Interop_ClrTypeId idx;

    TinyCLR_Interop_ClrValue arr, ret;

    interop->FindType(interop, "mscorlib", "System", "Int32", idx);
    interop->CreateArray(interop, ptrLen, idx, arr);

    auto p = reinterpret_cast<int32_t*>(arr.Data.SzArray.Data);

    for (auto i = 0; i < ptrLen; i++)
        p[i] = ptr[i];    

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    md.InteropManager->AssignObjectReference(md.InteropManager, ret, arr.Object);

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::WriteRead___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue args[7];

    for (auto i = 0; i < sizeof(args) / sizeof(TinyCLR_Interop_ClrValue); i++) {
        md.InteropManager->GetArgument(md.InteropManager, md.Stack, i, args[i]);
    }

    auto writeData = reinterpret_cast<uint8_t*>(args[0].Data.SzArray.Data);
    auto writeOffset = args[1].Data.Numeric->I4;
    auto writeLength = static_cast<size_t>(args[2].Data.Numeric->I4);

    auto readData = reinterpret_cast<uint8_t*>(args[3].Data.SzArray.Data);
    auto readOffset = args[4].Data.Numeric->I4;
    auto readLength = static_cast<size_t>(args[5].Data.Numeric->I4);

    auto deselectAfter = args[6].Data.Numeric->Boolean;

    return  api->WriteRead(api, reinterpret_cast<const uint8_t*>(writeData + writeOffset), writeLength, readData + readOffset, readLength, deselectAfter);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return  api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return  api->Release(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::SetActiveSettings___VOID__GHIElectronicsTinyCLRDevicesSpiSpiConnectionSettings(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue obj;
    TinyCLR_Interop_ClrValue args[8];
    TinyCLR_Spi_Settings settings;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, obj);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectLine__BackingField___I4, args[0]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectType__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiChipSelectType, args[1]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ClockFrequency__BackingField___I4, args[2]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___DataBitLength__BackingField___I4, args[3]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___Mode__BackingField___GHIElectronicsTinyCLRDevicesSpiSpiMode, args[4]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectSetupTime__BackingField___mscorlibSystemTimeSpan, args[5]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectHoldTime__BackingField___mscorlibSystemTimeSpan, args[6]);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Spi_GHIElectronics_TinyCLR_Devices_Spi_SpiConnectionSettings::FIELD___ChipSelectActiveState__BackingField___BOOLEAN, args[7]);

    settings.ChipSelectLine = args[0].Data.Numeric->I4;
    settings.ChipSelectType = static_cast<TinyCLR_Spi_ChipSelectType>(args[1].Data.Numeric->I4);
    settings.ClockFrequency = args[2].Data.Numeric->I4;
    settings.DataBitLength = args[3].Data.Numeric->I4;
    settings.Mode = static_cast<TinyCLR_Spi_Mode>(args[4].Data.Numeric->I4);
    settings.ChipSelectSetupTime = args[5].Data.Numeric->U8;
    settings.ChipSelectHoldTime = args[6].Data.Numeric->U8;
    settings.ChipSelectActiveState = args[7].Data.Numeric->Boolean;

    return api->SetActiveSettings(api, &settings);
}
