#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_InteropUtil.h"

const int CummulativeDaysForMonth[13] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

#define BASE_YEAR                   1601
#define BASE_YEAR_LEAPYEAR_ADJUST   388
#define DAYS_IN_NORMAL_YEAR         365
#define BASE_YEAR_DAYOFWEEK_SHIFT   1

#define IS_LEAP_YEAR(y)             (((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0))
#define NUMBER_OF_LEAP_YEARS(y)     ((((y - 1) / 4) - ((y - 1) / 100) + ((y - 1) / 400)) - BASE_YEAR_LEAPYEAR_ADJUST) /// Number of leap years until base year, not including the target year itself.
#define NUMBER_OF_YEARS(y)          (y - BASE_YEAR)

#define YEARS_TO_DAYS(y)            ((NUMBER_OF_YEARS(y) * DAYS_IN_NORMAL_YEAR) + NUMBER_OF_LEAP_YEARS(y))
#define MONTH_TO_DAYS(y, m)         (CummulativeDaysForMonth[m - 1] + ((IS_LEAP_YEAR(y) && (m > 2)) ? 1 : 0))

#define TIMEUNIT_TO_MINUTES         600000000
#define TIMEUNIT_TO_MILLISECONDS    10000
#define MILLISECONDS_TO_SECONDS     1000
#define SECONDS_TO_MINUTES          60
#define MINUTES_TO_HOUR             60
#define HOURS_TO_DAY                24

static inline uint64_t Interop_Rtc_FromSystemTime(const TinyCLR_Rtc_DateTime* systemTime) {
    uint64_t r = YEARS_TO_DAYS(systemTime->Year) + MONTH_TO_DAYS(systemTime->Year, systemTime->Month) + systemTime->DayOfMonth - 1;
    r = (((((r * HOURS_TO_DAY) + systemTime->Hour) * MINUTES_TO_HOUR + systemTime->Minute) * SECONDS_TO_MINUTES + systemTime->Second) * MILLISECONDS_TO_SECONDS + systemTime->Millisecond) * TIMEUNIT_TO_MILLISECONDS;

    return r;
}

static inline void Interop_Rtc_ToSystemTime(uint64_t time, TinyCLR_Rtc_DateTime* systemTime) {
    int ytd = 0;
    int mtd = 0;

    time /= TIMEUNIT_TO_MILLISECONDS;
    systemTime->Millisecond = time % MILLISECONDS_TO_SECONDS;
    time /= MILLISECONDS_TO_SECONDS;
    systemTime->Second = time % SECONDS_TO_MINUTES;
    time /= SECONDS_TO_MINUTES;
    systemTime->Minute = time % MINUTES_TO_HOUR;
    time /= MINUTES_TO_HOUR;
    systemTime->Hour = time % HOURS_TO_DAY;
    time /= HOURS_TO_DAY;

    systemTime->DayOfWeek = (time + BASE_YEAR_DAYOFWEEK_SHIFT) % 7;
    systemTime->Year = (uint32_t)(time / DAYS_IN_NORMAL_YEAR + BASE_YEAR);
    ytd = YEARS_TO_DAYS(systemTime->Year);
    if (ytd > time) {
        systemTime->Year--;
        ytd = YEARS_TO_DAYS(systemTime->Year);
    }

    time -= ytd;

    systemTime->Month = (uint32_t)(time / 31 + 1);
    mtd = MONTH_TO_DAYS(systemTime->Year, systemTime->Month + 1);

    if (time >= mtd) {
        systemTime->Month++;
    }

    mtd = MONTH_TO_DAYS(systemTime->Year, systemTime->Month);

    systemTime->DayOfMonth = (uint32_t)(time - mtd + 1);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::GetTime___GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Rtc_DateTime now;

    auto result = api->GetTime(api, now);

    TinyCLR_Interop_ClrValue obj, ret;
    TinyCLR_Interop_ClrTypeId type;

    md.InteropManager->FindType(md.InteropManager, "GHIElectronics.TinyCLR.Devices", "GHIElectronics.TinyCLR.Devices.Rtc", "RtcDateTime", type);

    md.InteropManager->CreateObject(md.InteropManager, md.Stack, type, obj);

    TinyCLR_Interop_ClrValue year, month, week, dayofweek, dayofmonth, dayofyear, hour, minute, second, milisecond, microsecond, nanosecond;

    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Year___I4, year);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Month___I4, month);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Week___I4, week);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfYear___I4, dayofweek);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfMonth___I4, dayofmonth);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfWeek___I4, dayofyear);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Hour___I4, hour);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Minute___I4, minute);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Second___I4, second);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Millisecond___I4, milisecond);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Microsecond___I4, microsecond);
    md.InteropManager->GetField(md.InteropManager, obj.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Nanosecond___I4, nanosecond);

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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::SetTime___VOID__GHIElectronicsTinyCLRDevicesRtcRtcDateTime(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    TinyCLR_Interop_ClrValue arg0;

    md.InteropManager->GetArgument(md.InteropManager, md.Stack, 0, arg0);

    TinyCLR_Rtc_DateTime now;

    TinyCLR_Interop_ClrValue year, month, week, dayofweek, dayofmonth, dayofyear, hour, minute, second, milisecond, microsecond, nanosecond;

    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Year___I4, year);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Month___I4, month);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Week___I4, week);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfYear___I4, dayofweek);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfMonth___I4, dayofmonth);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___DayOfWeek___I4, dayofyear);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Hour___I4, hour);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Minute___I4, minute);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Second___I4, second);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Millisecond___I4, milisecond);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Microsecond___I4, microsecond);
    md.InteropManager->GetField(md.InteropManager, arg0.Object, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_RtcDateTime::FIELD___Nanosecond___I4, nanosecond);

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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Acquire(api);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerApiWrapper::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto api = reinterpret_cast<const TinyCLR_Rtc_Controller*>(TinyCLR_Interop_GetApi(md, FIELD___impl___I));

    return api->Release(api);
}
