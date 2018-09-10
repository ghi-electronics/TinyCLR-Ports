#pragma once

#include <TinyCLR.h>

struct Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper {
    static const size_t FIELD___impl___I = 1;
    static const size_t FIELD___Api__BackingField___GHIElectronicsTinyCLRNativeGHIElectronicsTinyCLRNativeApi = 2;

    static TinyCLR_Result GetTime___GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result SetTime___VOID__GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result get_IsValid___BOOLEAN(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Acquire___VOID(const TinyCLR_Interop_MethodData md);
    static TinyCLR_Result Release___VOID(const TinyCLR_Interop_MethodData md);
};

struct Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcController {
    static const size_t FIELD___Provider__BackingField___GHIElectronicsTinyCLRDevicesRtcProviderIRtcControllerProvider = 1;
};

struct Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime {
    static const size_t FIELD___Year___I4 = 1;
    static const size_t FIELD___Month___I4 = 2;
    static const size_t FIELD___Week___I4 = 3;
    static const size_t FIELD___DayOfYear___I4 = 4;
    static const size_t FIELD___DayOfMonth___I4 = 5;
    static const size_t FIELD___DayOfWeek___I4 = 6;
    static const size_t FIELD___Hour___I4 = 7;
    static const size_t FIELD___Minute___I4 = 8;
    static const size_t FIELD___Second___I4 = 9;
    static const size_t FIELD___Millisecond___I4 = 10;
    static const size_t FIELD___Microsecond___I4 = 11;
    static const size_t FIELD___Nanosecond___I4 = 12;
};

extern const TinyCLR_Interop_Assembly Interop_GHIElectronics_TinyCLR_Devices_Rtc;
