#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Storage_GHIElectronics_TinyCLR_Devices_Storage_Provider_StorageControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_Descriptor___GHIElectronicsTinyCLRDevicesStorageStorageDescriptor(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Open___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Close___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__I8__I4__SZARRAY_U1__I4__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___I4__I8__I4__SZARRAY_U1__I4__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Erase___I4__I8__I4__mscorlibSystemTimeSpan(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsErased___BOOLEAN__I8__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Storage_GHIElectronics_TinyCLR_Devices_Storage_StorageController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesStorageProviderIStorageControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Storage_GHIElectronics_TinyCLR_Devices_Storage_StorageDescriptor {
    static const size_t FIELD___CanReadDirect__BackingField___BOOLEAN = 1;
    static const size_t FIELD___CanWriteDirect__BackingField___BOOLEAN = 2;
    static const size_t FIELD___CanExecuteDirect__BackingField___BOOLEAN = 3;
    static const size_t FIELD___EraseBeforeWrite__BackingField___BOOLEAN = 4;
    static const size_t FIELD___Removable__BackingField___BOOLEAN = 5;
    static const size_t FIELD___RegionsContiguous__BackingField___BOOLEAN = 6;
    static const size_t FIELD___RegionsEqualSized__BackingField___BOOLEAN = 7;
    static const size_t FIELD___RegionCount__BackingField___I4 = 8;
    static const size_t FIELD___RegionAddresses__BackingField___SZARRAY_I8 = 9;
    static const size_t FIELD___RegionSizes__BackingField___SZARRAY_I4 = 10;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Storage;
