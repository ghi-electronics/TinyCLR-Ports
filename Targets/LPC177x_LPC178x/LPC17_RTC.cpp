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

#include "LPC17.h"

static TinyCLR_Rtc_Provider rtcProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* LPC17_Rtc_GetApi() {
    rtcProvider.Parent = &timeApi;
    rtcProvider.Index = 0;
    rtcProvider.Acquire = &LPC17_Rtc_Acquire;
    rtcProvider.Release = &LPC17_Rtc_Release;
    rtcProvider.GetNow = &LPC17_Rtc_GetNow;
    rtcProvider.SetNow = &LPC17_Rtc_SetNow;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.RtcProvider";
    timeApi.Type = TinyCLR_Api_Type::RtcProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &rtcProvider;

    return &timeApi;
}

TinyCLR_Result LPC17_Rtc_Acquire(const TinyCLR_Rtc_Provider* self) {
    LPC_SC->PCONP |= PCONP_PCRTC;

    if (LPC_RTC->CCR != 1) {
        LPC_RTC->CCR = 0;
        LPC_RTC->CCR = 1;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_Release(const TinyCLR_Rtc_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_GetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime& value) {
    if (LPC_RTC->CCR != 1) {
        TinyCLR_Result::InvalidOperation;
    }

    value.Hour = LPC_RTC->HOUR;
    value.Minute = LPC_RTC->MIN;
    value.Second = LPC_RTC->SEC;
    value.Millisecond = 0;


    value.Year = LPC_RTC->YEAR;
    value.Month = LPC_RTC->MONTH;
    value.DayOfMonth = LPC_RTC->DOM;
    value.DayOfWeek = LPC_RTC->DOW;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_SetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime value) {
    if (LPC_RTC->CCR != 1) {
        TinyCLR_Result::InvalidOperation;
    }

    LPC_RTC->YEAR = value.Year;
    LPC_RTC->MONTH = value.Month;
    LPC_RTC->DOM = value.DayOfMonth;
    LPC_RTC->DOW = value.DayOfWeek;
    LPC_RTC->HOUR = value.Hour;
    LPC_RTC->MIN = value.Minute;
    LPC_RTC->SEC = value.Second;

    LPC_RTC->CIIR = 0;

    return TinyCLR_Result::Success;
}
