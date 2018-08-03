#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Pwm_GHIElectronics_TinyCLR_Devices_Pwm_Provider_PwmControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result get_ChannelCount___I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MinFrequency___R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_MaxFrequency___R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result OpenChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result CloseChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result EnableChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result DisableChannel___VOID__I4(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetPulseParameters___VOID__I4__R8__GHIElectronicsTinyCLRDevicesPwmPwmPulsePolarity(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetDesiredFrequency___R8__R8(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Pwm_GHIElectronics_TinyCLR_Devices_Pwm_PwmChannel {
    static const size_t FIELD___polarity___GHIElectronicsTinyCLRDevicesPwmPwmPulsePolarity = 1;
    static const size_t FIELD___dutyCycle___R8 = 2;
    static const size_t FIELD___ChannelNumber__BackingField___I4 = 3;
    static const size_t FIELD___Controller__BackingField___GHIElectronicsTinyCLRDevicesPwmPwmController = 4;
    static const size_t FIELD___IsStarted__BackingField___BOOLEAN = 5;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Pwm_GHIElectronics_TinyCLR_Devices_Pwm_PwmController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesPwmProviderIPwmControllerProvider = 1;
    static const size_t FIELD___ActualFrequency__BackingField___R8 = 2;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Pwm;
