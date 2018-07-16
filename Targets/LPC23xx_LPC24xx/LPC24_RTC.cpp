// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "LPC24.h"


#define RTC_BASE 0xE0024000

#define RTC_CCR 0xE0024008
#define RTC_CIIR 0xE002400C
#define RTC_CISS  0xE0024040

#define RTC_SEC 0xE0024020
#define RTC_MIN 0xE0024024
#define RTC_HOUR 0xE0024028
#define RTC_DOM 0xE002402C
#define RTC_DOW 0xE0024030
#define RTC_DOY 0xE0024034
#define RTC_MONTH 0xE0024038
#define RTC_YEAR 0xE002403C


static TinyCLR_Rtc_Controller rtcProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* LPC24_Rtc_GetApi() {
    rtcProvider.ApiInfo = &timeApi;
    rtcProvider.Acquire = &LPC24_Rtc_Acquire;
    rtcProvider.Release = &LPC24_Rtc_Release;
    rtcProvider.GetNow = &LPC24_Rtc_GetNow;
    rtcProvider.SetNow = &LPC24_Rtc_SetNow;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.RtcProvider";
    timeApi.Type = TinyCLR_Api_Type::RtcProvider;
    timeApi.Version = 0;
    timeApi.Implementation = &rtcProvider;

    return &timeApi;
}

TinyCLR_Result LPC24_Rtc_Acquire(const TinyCLR_Rtc_Controller* self) {
    LPC24XX::SYSCON().PCONP |= PCONP_PCRTC;

    uint32_t* rtc_ccr_reg = reinterpret_cast<uint32_t*>(RTC_CCR);

    if (*rtc_ccr_reg != (1 | (1 << 4))) {
        *rtc_ccr_reg = 0;
        *rtc_ccr_reg = (1 | (1 << 4));
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Rtc_Release(const TinyCLR_Rtc_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Rtc_GetNow(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value) {
    uint32_t* rtc_ccr_reg = reinterpret_cast<uint32_t*>(RTC_CCR);

    if (*rtc_ccr_reg != (1 | (1 << 4))) {
        TinyCLR_Result::InvalidOperation;
    }

    value.Hour = *(reinterpret_cast<uint32_t*>(RTC_HOUR));
    value.Minute = *(reinterpret_cast<uint32_t*>(RTC_MIN));
    value.Second = *(reinterpret_cast<uint32_t*>(RTC_SEC));
    value.Millisecond = 0;

    value.Year = *(reinterpret_cast<uint32_t*>(RTC_YEAR));
    value.Month = *(reinterpret_cast<uint32_t*>(RTC_MONTH));
    value.DayOfMonth = *(reinterpret_cast<uint32_t*>(RTC_DOM));
    value.DayOfWeek = *(reinterpret_cast<uint32_t*>(RTC_DOW));

    if ((value.Year < 1601) || (value.Year > 3000) ||
        (value.Month < 1) || (value.Month > 12) ||
        (value.DayOfMonth < 1) || (value.DayOfMonth > 31)) {
        value.Hour = 0;
        value.Minute = 0;
        value.Second = 0;
        value.Millisecond = 0;

        value.Year = 1980;
        value.Month = 1;
        value.DayOfMonth = 1;
        value.DayOfWeek = 3;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Rtc_SetNow(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value) {
    uint32_t* rtc_ccr_reg = reinterpret_cast<uint32_t*>(RTC_CCR);
    if (*rtc_ccr_reg != (1 | (1 << 4))) {
        TinyCLR_Result::InvalidOperation;
    }

    if ((value.Year < 1601) || (value.Year > 3000) ||
        (value.Month < 1) || (value.Month > 12) ||
        (value.DayOfMonth < 1) || (value.DayOfMonth > 31)) {
        TinyCLR_Result::ArgumentInvalid;
    }

    *(reinterpret_cast<uint32_t*>(RTC_YEAR)) = value.Year;
    *(reinterpret_cast<uint32_t*>(RTC_MONTH)) = value.Month;
    *(reinterpret_cast<uint32_t*>(RTC_DOM)) = value.DayOfMonth;
    *(reinterpret_cast<uint32_t*>(RTC_DOW)) = value.DayOfWeek;
    *(reinterpret_cast<uint32_t*>(RTC_HOUR)) = value.Hour;
    *(reinterpret_cast<uint32_t*>(RTC_MIN)) = value.Minute;
    *(reinterpret_cast<uint32_t*>(RTC_SEC)) = value.Second;

    *(reinterpret_cast<uint32_t*>(RTC_CIIR)) = 0;
    *(reinterpret_cast<uint32_t*>(RTC_CISS)) = 0;

    return TinyCLR_Result::Success;
}
