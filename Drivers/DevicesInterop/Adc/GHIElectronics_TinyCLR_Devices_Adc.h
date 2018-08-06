#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Adc_GHIElectronics_TinyCLR_Devices_Adc_AdcChannel {
    static const size_t FIELD___ChannelNumber__BackingField___I4 = 1;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesAdcAdcController = 2;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Adc_GHIElectronics_TinyCLR_Devices_Adc_AdcController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesAdcProviderIAdcControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Adc_GHIElectronics_TinyCLR_Devices_Adc_Provider_AdcControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_ResolutionInBits___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxValue___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result IsChannelModeSupported___BOOLEAN__GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result GetChannelMode___GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetChannelMode___VOID__GHIElectronicsTinyCLRDevicesAdcAdcChannelMode(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Read___I4__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Adc;
