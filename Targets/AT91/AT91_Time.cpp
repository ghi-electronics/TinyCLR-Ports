// Copyright Microsoft Corporation
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

#define CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 3
#define  AT91_TIME_OVERFLOW_FLAG 0x80000000

static TinyCLR_Time_Provider timeProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* AT91_Time_GetApi() {
    timeProvider.Parent = &timeApi;
    timeProvider.Index = 0;
    timeProvider.GetInitialTime = &AT91_Time_GetInitialTime;
    timeProvider.GetTimeForProcessorTicks = &AT91_Time_TicksToTime;
    timeProvider.GetProcessorTicksForTime = &AT91_Time_TimeToTicks;
    timeProvider.GetCurrentProcessorTicks = &AT91_Time_GetCurrentTicks;
    timeProvider.SetTickCallback = &AT91_Time_SetCompareCallback;
    timeProvider.SetNextTickCallbackTime = &AT91_Time_SetCompare;
    timeProvider.Acquire = &AT91_Time_Acquire;
    timeProvider.Release = &AT91_Time_Release;
    timeProvider.DelayNoInterrupt = &AT91_Time_DelayNoInterrupt;
    timeProvider.Delay = &AT91_Time_Delay;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.TimeProvider";
    timeApi.Type = TinyCLR_Api_Type::TimeProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &timeProvider;

    return &timeApi;
}

TinyCLR_Result AT91_Time_GetInitialTime(const TinyCLR_Time_Provider* self, int64_t& utcTime, int32_t& timeZoneOffsetMinutes) {
    return TinyCLR_Result::NotSupported;
}

uint32_t AT91_Time_GetSystemClock(const TinyCLR_Time_Provider* self) {
    return SYSTEM_CLOCK_HZ;
}

uint32_t AT91_Time_GetTicksPerSecond(const TinyCLR_Time_Provider* self) {
    return SLOW_CLOCKS_PER_SECOND;
}

uint32_t AT91_Time_GetSystemCycleClock(const TinyCLR_Time_Provider* self) {
    return SYSTEM_CYCLE_CLOCK_HZ;
}

uint64_t AT91_Time_TicksToTime(const TinyCLR_Time_Provider* self, uint64_t ticks) {
    ticks *= (TEN_MHZ / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t AT91_Time_TimeToTicks(const TinyCLR_Time_Provider* self, uint64_t time) {
    return AT91_Time_MicrosecondsToTicks(self, time / 10);
}

uint64_t AT91_Time_MillisecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t ticks) {
    ticks *= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_MILLISECOND_GCD);
    ticks /= (1000 / SLOW_CLOCKS_MILLISECOND_GCD);

    return ticks;
}

uint64_t AT91_Time_MicrosecondsToTicks(const TinyCLR_Time_Provider* self, uint64_t microseconds) {
#if ONE_MHZ <= SLOW_CLOCKS_PER_SECOND
    return microseconds * (SLOW_CLOCKS_PER_SECOND / ONE_MHZ);
#else
    return microseconds / (ONE_MHZ / SLOW_CLOCKS_PER_SECOND);
#endif
}

uint64_t AT91_Time_GetCurrentTicks(const TinyCLR_Time_Provider* self) {
    int32_t timer = 0;

    if (self != nullptr)
        timer = self->Index;
    
    return (uint64_t)0;
}

TinyCLR_Result AT91_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks) {
    int32_t timer = 0;

    if (self != nullptr)
        timer = self->Index;

    GLOBAL_LOCK(irq);

    
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Acquire(const TinyCLR_Time_Provider* self) {
    int32_t timer = self->Index;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Release(const TinyCLR_Time_Provider* self) {
    int32_t timer = self->Index;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback) {
    return TinyCLR_Result::Success;
}

void AT91_Time_DelayNoInterrupt(const TinyCLR_Time_Provider* self, uint64_t microseconds) {
    GLOBAL_LOCK(irq);

    uint64_t current = AT91_Time_GetCurrentTicks(self);
    uint64_t maxDiff = AT91_Time_MicrosecondsToTicks(self, microseconds);

    if (maxDiff <= CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS) maxDiff = 0;
    else maxDiff -= CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS;

    while (((int32_t)(AT91_Time_GetCurrentTicks(self) - current)) <= maxDiff);

}

extern "C" void IDelayLoop(int32_t iterations);

void AT91_Time_Delay(const TinyCLR_Time_Provider* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= (SYSTEM_CYCLE_CLOCK_HZ / CLOCK_COMMON_FACTOR);
    microseconds /= (ONE_MHZ / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int32_t iterations = (int32_t)microseconds - 5;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

//******************** Profiler ********************

