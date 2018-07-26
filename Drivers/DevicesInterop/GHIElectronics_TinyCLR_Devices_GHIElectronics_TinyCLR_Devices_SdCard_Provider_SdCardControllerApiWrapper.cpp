#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::ReadSectors___I4__I8__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);

    auto sector = static_cast<uint64_t>(arg0.Data.Numeric->I8);
    auto count = static_cast<size_t>(arg1.Data.Numeric->I4);
    auto buffer = reinterpret_cast<uint8_t*>(arg2.Data.SzArray.Data);
    auto offset = arg3.Data.Numeric->I4;
    auto timeout = arg4.Data.Numeric->I4;

    buffer += offset;

    auto result = api->ReadSectors(api, sector, count, buffer, timeout);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::WriteSectors___I4__I8__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2, arg3, arg4;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 3, arg3);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 4, arg4);

    auto sector = static_cast<uint64_t>(arg0.Data.Numeric->I8);
    auto count = static_cast<size_t>(arg1.Data.Numeric->I4);
    auto buffer = reinterpret_cast<uint8_t*>(arg2.Data.SzArray.Data);
    auto offset = arg3.Data.Numeric->I4;
    auto timeout = arg4.Data.Numeric->I4;

    buffer += offset;

    auto result = api->WriteSectors(api, sector, count, buffer, timeout);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::EraseSectors___I4__I8__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto sector = static_cast<uint64_t>(arg0.Data.Numeric->I8);
    auto count = static_cast<size_t>(arg1.Data.Numeric->I4);

    auto timeout = arg2.Data.Numeric->I4;

    auto result = api->EraseSectors(api, sector, count, timeout);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::IsSectorErased___BOOLEAN__I8(const TinyCLR_Interop_MethodData md) {

    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::GetSectorMap___VOID__BYREF_SZARRAY_I4__BYREF_I4__BYREF_BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto sizes = reinterpret_cast<const size_t*>(arg0.Data.SzArray.Data);
    auto count = static_cast<size_t>(arg1.Data.Numeric->I4);
    auto uniform = arg2.Data.Numeric->Boolean;

    auto result = api->GetSectorMap(api, sizes, count, uniform);

    arg0.Data.SzArray.Data = reinterpret_cast<void*>(const_cast<size_t*>(sizes));
    arg1.Data.Numeric->I4 = count;
    arg2.Data.Numeric->Boolean = uniform;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_SdCardControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
