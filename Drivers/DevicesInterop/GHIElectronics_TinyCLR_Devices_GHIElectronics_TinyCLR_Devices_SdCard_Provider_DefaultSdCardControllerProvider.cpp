#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_DefaultSdCardControllerProvider::ReadSectors___I4__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_SdCard_Provider*)TinyCLR_Interop_GetFieldInMethodData(md, FIELD___nativeProvider___I).Data.Numeric->I;
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___I4).Data.Numeric->I4;

    auto sector = static_cast<uint64_t>(TinyCLR_Interop_GetArguments(md, 1).Data.Numeric->I4);
    auto count = static_cast<size_t>(TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->I4);
    auto buffer = reinterpret_cast<uint8_t*>(TinyCLR_Interop_GetArguments(md, 3).Data.SzArray.Data);
    auto offset = TinyCLR_Interop_GetArguments(md, 4).Data.Numeric->I4;
    auto timeout = TinyCLR_Interop_GetArguments(md, 5).Data.Numeric->I4;

    buffer += offset;

    return provider->ReadSectors(provider, controller, sector, count, buffer, timeout);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_DefaultSdCardControllerProvider::WriteSectors___I4__I4__I4__SZARRAY_U1__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_SdCard_Provider*)TinyCLR_Interop_GetFieldInMethodData(md, FIELD___nativeProvider___I).Data.Numeric->I;
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___I4).Data.Numeric->I4;

    auto sector = static_cast<uint64_t>(TinyCLR_Interop_GetArguments(md, 1).Data.Numeric->I4);
    auto count = static_cast<size_t>(TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->I4);
    auto buffer = (uint8_t*)TinyCLR_Interop_GetArguments(md, 3).Data.SzArray.Data;
    auto offset = TinyCLR_Interop_GetArguments(md, 4).Data.Numeric->I4;
    auto timeout = TinyCLR_Interop_GetArguments(md, 5).Data.Numeric->I4;

    buffer += offset;

    return provider->WriteSectors(provider, controller, sector, count, buffer, timeout);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_DefaultSdCardControllerProvider::EraseSectors___I4__I4__I4__I4(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_SdCard_Provider*)TinyCLR_Interop_GetFieldInMethodData(md, FIELD___nativeProvider___I).Data.Numeric->I;
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___I4).Data.Numeric->I4;

    auto sector = static_cast<uint64_t>(TinyCLR_Interop_GetArguments(md, 1).Data.Numeric->I4);
    auto count = static_cast<size_t>(TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->I4);
    auto timeout = TinyCLR_Interop_GetArguments(md, 3).Data.Numeric->I4;

    return provider->EraseSectors(provider, controller, sector, count, timeout);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_DefaultSdCardControllerProvider::GetSectorMap___VOID__BYREF_SZARRAY_I4__BYREF_I4__BYREF_BOOLEAN(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_SdCard_Provider*)TinyCLR_Interop_GetFieldInMethodData(md, FIELD___nativeProvider___I).Data.Numeric->I;
    auto controller = TinyCLR_Interop_GetFieldInMethodData(md, FIELD___idx___I4).Data.Numeric->I4;

    auto sizes = reinterpret_cast<const size_t*>(TinyCLR_Interop_GetArguments(md, 1).Data.SzArray.Data);
    auto count = static_cast<size_t>(TinyCLR_Interop_GetArguments(md, 2).Data.Numeric->I4);
    auto uniform = TinyCLR_Interop_GetArguments(md, 3).Data.Numeric->Boolean;

    return provider->GetSectorMap(provider, controller, sizes, count, uniform);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_SdCard_Provider_DefaultSdCardControllerProvider::GetControllerCount___STATIC___I4__I(const TinyCLR_Interop_MethodData md) {
    auto provider = (const TinyCLR_SdCard_Provider*)TinyCLR_Interop_GetArguments(md, 0).Data.Numeric->I;

    int32_t count;

    provider->GetControllerCount(provider, count);

    auto ret = TinyCLR_Interop_GetReturn(md);

    ret.Data.Numeric->I4 = count;

    return TinyCLR_Result::Success;
}
