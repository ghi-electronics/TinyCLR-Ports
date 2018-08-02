#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_ChipSelectLineCount___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetChipSelectLineCount(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_MinClockFrequency___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMinClockFrequency(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_MaxClockFrequency___I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = api->GetMaxClockFrequency(api);

    return TinyCLR_Result::Success;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::get_SupportedDataBitLengths___SZARRAY_I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    size_t ptrLen = 10;
    uint32_t ptr[ptrLen];

    auto result = api->GetSupportedDataBitLengths(api, ptr, ptrLen);

    auto interop = md.InteropManager;

    TinyCLR_Interop_ClrTypeId idx;

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    interop->FindType(interop, "mscorlib", "System", "Int32", idx);
    interop->CreateArray(interop, ptrLen, idx, ret);

    auto p = reinterpret_cast<int32_t*>(ret.Data.SzArray.Data);

    for (auto i = 0; i < ptrLen; i++)
        p[i] = ptr[i];

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::WriteRead___VOID__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4, arg5, arg6;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 5, arg5);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 6, arg6);

    auto writeData = reinterpret_cast<uint8_t*>(arg0.Data.SzArray.Data);
    auto writeOffset = arg1.Data.Numeric->I4;
    auto writeLength = static_cast<size_t>(arg2.Data.Numeric->I4);

    auto readData = reinterpret_cast<uint8_t*>(arg3.Data.SzArray.Data);
    auto readOffset = arg4.Data.Numeric->I4;
    auto readLength = static_cast<size_t>(arg5.Data.Numeric->I4);

    auto deselectAfter = arg6.Data.Numeric->Boolean;

    return  api->WriteRead(api, reinterpret_cast<const uint8_t*>(writeData + writeOffset), writeLength, readData + readOffset, readLength, deselectAfter);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return  api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return  api->Release(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Spi_Provider_SpiControllerApiWrapper::SetActiveSettings___VOID__I4__BOOLEAN__I4__I4__GHIElectronicsTinyCLRDevicesSpiSpiMode(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Spi_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);

    auto chipSelectLine = arg0.Data.Numeric->I4;
    auto useControllerChipSelect = arg1.Data.Numeric->Boolean;
    auto clockFrequency = arg2.Data.Numeric->I4;
    auto dataBitLength = arg3.Data.Numeric->I4;
    auto mode = static_cast<TinyCLR_Spi_Mode>(arg4.Data.Numeric->I4);

    return  api->SetActiveSettings(api, chipSelectLine, useControllerChipSelect, clockFrequency, dataBitLength, mode);
}
