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

#define TOTAL_RTC_CONTROLLERS 1

static TinyCLR_Rtc_Controller rtcControllers[TOTAL_RTC_CONTROLLERS];
static TinyCLR_Api_Info rtcApi[TOTAL_RTC_CONTROLLERS];

const char* rtcApiNames[TOTAL_RTC_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC17.RtcController\\0"
};

void LPC17_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (auto i = 0; i < TOTAL_RTC_CONTROLLERS; i++) {
        rtcControllers[i].ApiInfo = &rtcApi[i];
        rtcControllers[i].Acquire = &LPC17_Rtc_Acquire;
        rtcControllers[i].Release = &LPC17_Rtc_Release;
        rtcControllers[i].IsValid = &LPC17_Rtc_IsValid;
        rtcControllers[i].GetTime = &LPC17_Rtc_GetTime;
        rtcControllers[i].SetTime = &LPC17_Rtc_SetTime;

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

TinyCLR_Result LPC17_Rtc_Acquire(const TinyCLR_Rtc_Controller* self) {
    LPC_SC->PCONP |= PCONP_PCRTC;

    if (LPC_RTC->CCR != 1) {
        LPC_RTC->CCR = 0;
        LPC_RTC->CCR = 1;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_Release(const TinyCLR_Rtc_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value) {
    TinyCLR_Rtc_DateTime rtcNow;

    value = (LPC17_Rtc_GetTime(self, rtcNow) == TinyCLR_Result::Success);

    if (rtcNow.Second >= 60 || rtcNow.Minute >= 60 || rtcNow.Hour >= 24 || rtcNow.DayOfMonth >= 32 || rtcNow.Month >= 13 || rtcNow.DayOfWeek >= 8)
        value = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value) {
    if (LPC_RTC->CCR != 1) {
        return TinyCLR_Result::InvalidOperation;
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

TinyCLR_Result LPC17_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value) {
    if (LPC_RTC->CCR != 1) {
        return TinyCLR_Result::InvalidOperation;
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
