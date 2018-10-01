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

#define TOTAL_RTC_CONTROLLERS 1

static TinyCLR_Rtc_Controller rtcControllers[TOTAL_RTC_CONTROLLERS];
static TinyCLR_Api_Info rtcApi[TOTAL_RTC_CONTROLLERS];

const char* rtcApiNames[TOTAL_RTC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.RtcController\\0"
};

void LPC24_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_RTC_CONTROLLERS; i++) {
        rtcControllers[i].ApiInfo = &rtcApi[i];
        rtcControllers[i].Acquire = &LPC24_Rtc_Acquire;
        rtcControllers[i].Release = &LPC24_Rtc_Release;
        rtcControllers[i].IsValid = &LPC24_Rtc_IsValid;
        rtcControllers[i].GetTime = &LPC24_Rtc_GetTime;
        rtcControllers[i].SetTime = &LPC24_Rtc_SetTime;

        rtcApi[i].Author = "GHI Electronics, LLC";
        rtcApi[i].Name = rtcApiNames[i];
        rtcApi[i].Type = TinyCLR_Api_Type::RtcController;
        rtcApi[i].Version = 0;
        rtcApi[i].Implementation = &rtcControllers[i];
        rtcApi[i].State = nullptr;

        apiManager->Add(apiManager, &rtcApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::RtcController, rtcApi[0].Name);
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

TinyCLR_Result LPC24_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value) {
    TinyCLR_Rtc_DateTime rtcNow;

    value = (LPC24_Rtc_GetTime(self, rtcNow) == TinyCLR_Result::Success);

    if (rtcNow.Second >= 60 || rtcNow.Minute >= 60 || rtcNow.Hour >= 24 || rtcNow.DayOfMonth >= 32 || rtcNow.Month >= 13 || rtcNow.DayOfWeek >= 8)
        value = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value) {
    uint32_t* rtc_ccr_reg = reinterpret_cast<uint32_t*>(RTC_CCR);

    if (*rtc_ccr_reg != (1 | (1 << 4))) {
        return TinyCLR_Result::InvalidOperation;
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

TinyCLR_Result LPC24_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value) {
    uint32_t* rtc_ccr_reg = reinterpret_cast<uint32_t*>(RTC_CCR);
    if (*rtc_ccr_reg != (1 | (1 << 4))) {
        return TinyCLR_Result::InvalidOperation;
    }

    if ((value.Year < 1601) || (value.Year > 3000) ||
        (value.Month < 1) || (value.Month > 12) ||
        (value.DayOfMonth < 1) || (value.DayOfMonth > 31)) {
        return TinyCLR_Result::ArgumentInvalid;
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
