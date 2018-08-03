#include "GHIElectronics_TinyCLR_Devices_Rtc.h"
#include "../GHIElectronics_TinyCLR_InteropUtil.h"

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::GetTime___GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Rtc_DateTime now;

    auto result = api->GetTime(api, now);

    TinyCLR_Interop_ClrValue obj, ret;
    TinyCLR_Interop_ClrTypeId type;

    md.InteropManager->FindType(md.InteropManager, "GHIElectronics.TinyCLR.Devices", "GHIElectronics.TinyCLR.Devices.Rtc", "RtcDateTime", type);

    md.InteropManager->CreateObject(md.InteropManager, md.Stack, type, obj);

    TinyCLR_Interop_ClrValue year, month, week, dayofweek, dayofmonth, dayofyear, hour, minute, second, milisecond, microsecond, nanosecond;

    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Year___I4, year);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Month___I4, month);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Week___I4, week);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfYear___I4, dayofweek);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfMonth___I4, dayofmonth);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfWeek___I4, dayofyear);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Hour___I4, hour);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Minute___I4, minute);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Second___I4, second);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Millisecond___I4, milisecond);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Microsecond___I4, microsecond);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Nanosecond___I4, nanosecond);

    year.Data.Numeric->I4 = now.Year;
    month.Data.Numeric->I4 = now.Month;
    week.Data.Numeric->I4 = now.Week;
    dayofweek.Data.Numeric->I4 = now.DayOfYear;
    dayofmonth.Data.Numeric->I4 = now.DayOfMonth;
    dayofyear.Data.Numeric->I4 = now.DayOfWeek;
    hour.Data.Numeric->I4 = now.Hour;
    minute.Data.Numeric->I4 = now.Minute;
    second.Data.Numeric->I4 = now.Second;
    milisecond.Data.Numeric->I4 = now.Millisecond;
    microsecond.Data.Numeric->I4 = now.Microsecond;
    nanosecond.Data.Numeric->I4 = now.Nanosecond;

    md.InteropManager->GetReturn(md.InteropManager, md.Stack, ret);

    md.InteropManager->AssignObjectReference(md.InteropManager, ret, obj.Object);

    return result;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::SetTime___VOID__GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    TinyCLR_Rtc_DateTime now;

    TinyCLR_Interop_ClrValue year, month, week, dayofweek, dayofmonth, dayofyear, hour, minute, second, milisecond, microsecond, nanosecond;

    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Year___I4, year);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Month___I4, month);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Week___I4, week);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfYear___I4, dayofweek);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfMonth___I4, dayofmonth);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfWeek___I4, dayofyear);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Hour___I4, hour);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Minute___I4, minute);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Second___I4, second);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Millisecond___I4, milisecond);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Microsecond___I4, microsecond);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Nanosecond___I4, nanosecond);

    now.Year = year.Data.Numeric->I4;
    now.Month = month.Data.Numeric->I4;
    now.Week = week.Data.Numeric->I4;
    now.DayOfYear = dayofweek.Data.Numeric->I4;
    now.DayOfMonth = dayofmonth.Data.Numeric->I4;
    now.DayOfWeek = dayofyear.Data.Numeric->I4;
    now.Hour = hour.Data.Numeric->I4;
    now.Minute = minute.Data.Numeric->I4;
    now.Second = second.Data.Numeric->I4;
    now.Millisecond = milisecond.Data.Numeric->I4;
    now.Microsecond = microsecond.Data.Numeric->I4;
    now.Nanosecond = nanosecond.Data.Numeric->I4;

    return api->SetTime(api, now);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_Rtc_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
