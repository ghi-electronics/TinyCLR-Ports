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
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_Release(const TinyCLR_Rtc_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_GetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime& value) {
    uint32_t  time;
    uint32_t  date;

    /* Get the RTC_TR register */
    time = 0;

    uint8_t hour = static_cast<uint8_t>(0);
    uint8_t minute = static_cast<uint8_t>(0);
    uint8_t second = static_cast<uint8_t>(0);

    value.Hour = 0;
    value.Minute = 0;
    value.Second = 0;
    value.Millisecond = 0;

    /* Get the RTC_TR register */
    date = 0;

    uint8_t year = static_cast<uint8_t>(0);
    uint8_t month = static_cast<uint8_t>(0);
    uint8_t day_of_month = static_cast<uint8_t>(0);
    uint8_t day_of_week = static_cast<uint8_t>(0);

    value.Year = 1980;
    value.Month = 0;
    value.DayOfMonth = 0;
    value.DayOfWeek = 0;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_SetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime value) {
    uint32_t  time;
    uint32_t  date;

    return TinyCLR_Result::Success;
}
