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

//////////////////////////////////////////////////////////////////////////////
// AT91 TIMER driver
//

struct AT91_Timer_Controller {
    bool m_configured;

    uint64_t m_lastRead;
    uint64_t m_nextCompare;

    TinyCLR_Time_TickCallback m_DequeuAndExecute;

    static bool   Initialize(uint32_t timer, uint32_t* ISR, void* ISR_Param);
    static bool   Uninitialize(uint32_t timer);

    static uint32_t ReadCounter(uint32_t timer);

    static void EnableCompareInterrupt(uint32_t timer) {

        AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

        TIMER.MCR |= AT91XX_TIMER::MR0_IRQEN;
    }

    static void DisableCompareInterrupt(uint32_t timer) {

        AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

        TIMER.MCR &= ~AT91XX_TIMER::MR0_IRQEN;
    }

    static void ForceInterrupt(uint32_t timer) {

        AT91XX::VIC().ForceInterrupt(AT91XX_TIMER::getIntNo(timer));
    }

    static void SetCompare(uint32_t timer, uint32_t Compare) {

        AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

        TIMER.MR0 = Compare;
    }

    static bool DidCompareHit(uint32_t timer) {

        AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

        return (TIMER.IR & AT91XX_TIMER::MR0_COMP) != 0;
    }

    static void ResetCompareHit(uint32_t timer) {

        AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

        TIMER.IR = AT91XX_TIMER::MR0_RESET;
    }
};
//
// AT91 TIMER driver
//////////////////////////////////////////////////////////////////////////////

AT91_Timer_Controller g_AT91_Timer_Controller;

bool AT91_Timer_Controller::Initialize(uint32_t timer, uint32_t* ISR, void* ISR_Param) {
    GLOBAL_LOCK(irq);

    if (g_AT91_Timer_Controller.m_configured == true) return false;

    //--//

    if (ISR) {
        if (!AT91_Interrupt_Activate(AT91XX_TIMER::getIntNo(timer), ISR, ISR_Param)) return false;
    }

    AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

    TIMER.TCR = AT91XX_TIMER::TCR_TEN;

    //--//

    g_AT91_Timer_Controller.m_configured = true;

    return true;
}

bool AT91_Timer_Controller::Uninitialize(uint32_t timer) {

    GLOBAL_LOCK(irq);

    if (g_AT91_Timer_Controller.m_configured == false) return false;

    //--//

    if (!AT91_Interrupt_Deactivate(AT91XX_TIMER::getIntNo(timer))) return false;

    AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

    //Reset timer
    TIMER.TCR = 0x2;
    // disable timer
    TIMER.TCR = 0;

    //--//

    g_AT91_Timer_Controller.m_configured = false;

    return true;
}

#pragma arm section code = "SectionForFlashOperations"

uint32_t __section("SectionForFlashOperations") AT91_Timer_Controller::ReadCounter(uint32_t timer) {

    AT91XX_TIMER& TIMER = AT91XX::TIMER(timer);

    return TIMER.TC;
}

#pragma arm section code
//--//

void AT91_Time_InterruptHandler(void* Param) {
    TinyCLR_Time_Provider *provider = (TinyCLR_Time_Provider*)Param;

    int32_t timer = 0;

    if (provider != nullptr)
        timer = provider->Index;

    if (AT91_Timer_Controller::DidCompareHit(timer)) {
        AT91_Timer_Controller::ResetCompareHit(timer);
    }

    g_AT91_Timer_Controller.m_lastRead = AT91_Time_GetCurrentTicks(provider);

    if (g_AT91_Timer_Controller.m_lastRead >= g_AT91_Timer_Controller.m_nextCompare) {
        // this also schedules the next one, if there is one
        g_AT91_Timer_Controller.m_DequeuAndExecute();
    }
    else {
        //
        // Because we are limited in the resolution of timer,
        // resetting the compare will properly configure the next interrupt.
        //
        AT91_Time_SetCompare(provider, g_AT91_Timer_Controller.m_nextCompare);
    }
}

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

    uint64_t lastValue = g_AT91_Timer_Controller.m_lastRead;

    uint32_t value = AT91_Timer_Controller::ReadCounter(timer);

    uint32_t resHigh = (uint32_t)(lastValue >> 32);
    uint32_t resLow = (uint32_t)lastValue;

    if ((resLow & AT91_TIME_OVERFLOW_FLAG) != (value & AT91_TIME_OVERFLOW_FLAG)) {
        if ((value & AT91_TIME_OVERFLOW_FLAG) == 0) {
            resHigh += 1;
        }
    }

    return (uint64_t)resHigh << 32 | value;
}

TinyCLR_Result AT91_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks) {
    int32_t timer = 0;

    if (self != nullptr)
        timer = self->Index;

    GLOBAL_LOCK(irq);

    g_AT91_Timer_Controller.m_nextCompare = processorTicks;

    uint32_t highComp = (uint32_t)(processorTicks >> 32);
    uint32_t lowComp = (uint32_t)processorTicks;

    uint32_t highRead = (uint32_t)(g_AT91_Timer_Controller.m_lastRead >> 32);
    uint32_t lowRead = (uint32_t)g_AT91_Timer_Controller.m_lastRead;

    bool fForceInterrupt = false;

    uint32_t lowReadNew = AT91_Timer_Controller::ReadCounter(timer);

    if ((lowRead & AT91_TIME_OVERFLOW_FLAG) != (lowReadNew & AT91_TIME_OVERFLOW_FLAG)) {
        fForceInterrupt = true;
    }
    else {
        lowRead = lowReadNew;

        if (highComp < highRead) {
            fForceInterrupt = true;
        }
        else if (highComp == highRead) {
            if (lowComp <= lowRead) {
                fForceInterrupt = true;
            }
        }
        else {
            lowComp = 0xFFFFFFFF;
        }

        if (fForceInterrupt == false) {
            uint32_t nextComp = (lowRead & AT91_TIME_OVERFLOW_FLAG) ? 0 : AT91_TIME_OVERFLOW_FLAG;

            int32_t diff1 = (int32_t)(lowComp - lowRead);
            int32_t diff2 = (int32_t)(nextComp - lowRead);

            if (diff1 > 0 && diff1 < diff2) {
                nextComp = lowComp;
            }

            AT91_Timer_Controller::SetCompare(timer, nextComp);

            lowReadNew = AT91_Timer_Controller::ReadCounter(timer);

            int32_t diff = nextComp - lowReadNew;

            if (diff < 0) {
                fForceInterrupt = true;
            }
        }
    }

    if (fForceInterrupt) {
        // Force interrupt to process this.
        AT91_Timer_Controller::ForceInterrupt(timer);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Acquire(const TinyCLR_Time_Provider* self) {
    int32_t timer = self->Index;

    g_AT91_Timer_Controller.m_lastRead = 0;
    g_AT91_Timer_Controller.m_nextCompare = 0x0000FFFFFFFFFFFFull;

    if (!AT91_Timer_Controller::Initialize(timer, (uint32_t*)&AT91_Time_InterruptHandler, (void*)self))
        return TinyCLR_Result::InvalidOperation;

    AT91_Timer_Controller::SetCompare(timer, AT91_TIME_OVERFLOW_FLAG);

    AT91_Timer_Controller::EnableCompareInterrupt(timer);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Release(const TinyCLR_Time_Provider* self) {
    int32_t timer = self->Index;

    if (AT91_Timer_Controller::Uninitialize(timer) == false)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback) {
    if (g_AT91_Timer_Controller.m_DequeuAndExecute != nullptr)
        return TinyCLR_Result::InvalidOperation;

    g_AT91_Timer_Controller.m_DequeuAndExecute = callback;

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

