#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::get_IsPresent___BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    return api->IsPresent(api, ret.Data.Numeric->Boolean);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::get_Descriptor___GHIElectronicsTinyCLRDevicesStorageStorageDescriptor(const TinyCLR_Interop_MethodData md) {


    return TinyCLR_Result::NotImplemented;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Open___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Open(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Close___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Close(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Read___I4__I8__I4__SZARRAY_U1__I4__I8(const TinyCLR_Interop_MethodData md) {
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
    auto offset = arg3.Data.Numeric->I8;
    auto timeout = arg4.Data.Numeric->I4;

    buffer += offset;

    auto result = api->Read(api, sector, count, buffer, timeout);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Write___I4__I8__I4__SZARRAY_U1__I4__I8(const TinyCLR_Interop_MethodData md) {
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
    auto timeout = arg4.Data.Numeric->I8;

    buffer += offset;

    auto result = api->Write(api, sector, count, buffer, timeout);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Erase___I4__I8__I4__I8(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0, arg1, arg2;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 2, arg2);

    auto sector = static_cast<uint64_t>(arg0.Data.Numeric->I8);
    auto count = static_cast<size_t>(arg1.Data.Numeric->I4);

    auto timeout = arg2.Data.Numeric->I8;

    auto result = api->Erase(api, sector, count, timeout);

    TinyCLR_Interop_ClrValue ret;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    ret.Data.Numeric->I4 = count;

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::IsErased___BOOLEAN__I8__I4(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));
    TinyCLR_Interop_ClrValue arg0, arg1, ret;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);
    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 1, arg1);
    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    size_t count = arg1.Data.Numeric->I4;

    return api->IsErased(api, arg0.Data.Numeric->I8, count, ret.Data.Numeric->Boolean);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Storage_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
