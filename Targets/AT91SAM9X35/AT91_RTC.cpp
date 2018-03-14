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

#include "AT91.h"

static TinyCLR_Rtc_Provider rtcProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* AT91_Rtc_GetApi() {
    rtcProvider.Parent = &timeApi;
    rtcProvider.Index = 0;
    rtcProvider.Acquire = &AT91_Rtc_Acquire;
    rtcProvider.Release = &AT91_Rtc_Release;
    rtcProvider.GetNow = &AT91_Rtc_GetNow;
    rtcProvider.SetNow = &AT91_Rtc_SetNow;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.RtcProvider";
    timeApi.Type = TinyCLR_Api_Type::RtcProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &rtcProvider;

    return &timeApi;
}

TinyCLR_Result AT91_Rtc_Acquire(const TinyCLR_Rtc_Provider* self) {
   
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_Release(const TinyCLR_Rtc_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_GetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime& value) {  
    if (false) {
        TinyCLR_Result::InvalidOperation;
    }   

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Rtc_SetNow(const TinyCLR_Rtc_Provider* self, TinyCLR_Rtc_DateTime value) {    
    if (false) {
        TinyCLR_Result::InvalidOperation;
    }  
    
    return TinyCLR_Result::Success;
}
