#include "GHIElectronics_TinyCLR_Devices.h"
#include "GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Interop.h"

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

uint64_t Interop_Rtc_FromSystemTime(const TinyCLR_Rtc_DateTime* systemTime) {
    uint64_t r = YEARS_TO_DAYS(systemTime->Year) + MONTH_TO_DAYS(systemTime->Year, systemTime->Month) + systemTime->DayOfMonth - 1;
    r = (((((r * HOURS_TO_DAY) + systemTime->Hour) * MINUTES_TO_HOUR + systemTime->Minute) * SECONDS_TO_MINUTES + systemTime->Second) * MILLISECONDS_TO_SECONDS + systemTime->Millisecond) * TIMEUNIT_TO_MILLISECONDS;

    return r;
}

void Interop_Rtc_ToSystemTime(uint64_t time, TinyCLR_Rtc_DateTime* systemTime) {
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

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::Acquire___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = reinterpret_cast<const TinyCLR_Interop_Provider*>(md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager));

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = reinterpret_cast<const TinyCLR_Rtc_Provider*>(fld.Data.Numeric->I);

    return provider->Acquire(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::Release___VOID(const TinyCLR_Interop_MethodData md) {
    auto interop = reinterpret_cast<const TinyCLR_Interop_Provider*>(md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager));

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::FIELD___nativeProvider___I, fld);

    auto provider = reinterpret_cast<const TinyCLR_Rtc_Provider*>(fld.Data.Numeric->I);

    return provider->Release(provider);
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::get_Now___mscorlibSystemDateTime(const TinyCLR_Interop_MethodData md) {
    auto interop = reinterpret_cast<const TinyCLR_Interop_Provider*>(md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager));

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, ret;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetReturn(interop, md.Stack, ret);

    auto provider = reinterpret_cast<const TinyCLR_Rtc_Provider*>(fld.Data.Numeric->I);

    TinyCLR_Rtc_DateTime now;

    auto res = provider->GetNow(provider, now);

    ret.Data.Numeric->I8 = Interop_Rtc_FromSystemTime(&now);

    return res;
}

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::set_Now___VOID__mscorlibSystemDateTime(const TinyCLR_Interop_MethodData md) {
    auto interop = reinterpret_cast<const TinyCLR_Interop_Provider*>(md.ApiManager.FindDefault(&md.ApiManager, TinyCLR_Api_Type::InteropManager));

    const TinyCLR_Interop_ClrObject* self;
    TinyCLR_Interop_ClrValue fld, arg;
    interop->GetThisObject(interop, md.Stack, self);
    interop->GetField(interop, self, Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_Rtc_Provider_RtcControllerProvider::FIELD___nativeProvider___I, fld);
    interop->GetArgument(interop, md.Stack, 1, arg);

    auto provider = reinterpret_cast<const TinyCLR_Rtc_Provider*>(fld.Data.Numeric->I);
    TinyCLR_Rtc_DateTime now;

    Interop_Rtc_ToSystemTime(arg.Data.Numeric->I8, &now);

    return provider->SetNow(provider, now);
}
