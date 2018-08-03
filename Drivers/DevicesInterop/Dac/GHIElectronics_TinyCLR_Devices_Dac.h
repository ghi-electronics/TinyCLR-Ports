#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Dac_GHIElectronics_TinyCLR_Devices_Dac_DacChannel {
    static const size_t FIELD___ChannelNumber__BackingField___I4 = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesDacDacController = 2;
    static const size_t FIELD___LastWrittenValue__BackingField___I4 = 3;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Dac_GHIElectronics_TinyCLR_Devices_Dac_DacController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesDacProviderIDacControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Dac_GHIElectronics_TinyCLR_Devices_Dac_Provider_DacControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Write___VOID__I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Dac;
