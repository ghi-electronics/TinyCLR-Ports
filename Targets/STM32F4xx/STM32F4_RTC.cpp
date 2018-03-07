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

#include "STM32F4.h"

static TinyCLR_Rtc_Provider rtcProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* STM32F4_Rtc_GetApi() {
    rtcProvider.Parent = &timeApi;
    rtcProvider.Index = 0;
    rtcProvider.Acquire = &STM32F4_Rtc_Acquire;
    rtcProvider.Release = &STM32F4_Rtc_Release;
    rtcProvider.GetNow = &STM32F4_Rtc_GetNow;
    rtcProvider.SetNow = &STM32F4_Rtc_SetNow;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.RtcProvider";
    timeApi.Type = TinyCLR_Api_Type::RtcProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &rtcProvider;

    return &timeApi;
}

TinyCLR_Result STM32F4_Rtc_Acquire(const TinyCLR_Rtc_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Rtc_Release(const TinyCLR_Rtc_Provider* self) {
    return TinyCLR_Result::Success;
}

uint32_t val;

TinyCLR_Result STM32F4_Rtc_GetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime& value) {
    value.Year = val;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Rtc_SetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime value) {
    val = value.Year;

    return TinyCLR_Result::Success;
}
