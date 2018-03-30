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

#include "LPC24.h"

#define TIMER_IDLE_VALUE  0x0000FFFFFFFFFFFFFull

#define CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 3
#define  LPC24_TIME_OVERFLOW_FLAG 0x80000000

#define CLOCK_COMMON_FACTOR                  1000000 // GCD(SYSTEM_CLOCK_HZ, 1M)
#define SLOW_CLOCKS_PER_SECOND               18000000 // SYSTEM_CLOCK_HZ MHz
#define SLOW_CLOCKS_TEN_MHZ_GCD              1000000 // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000 // GCD(SLOW_CLOCKS_PER_SECOND, 1k)

//////////////////////////////////////////////////////////////////////////////
// LPC24 TIMER driver
//

struct LPC24_Timer_Controller {
    bool m_configured;

    uint64_t m_lastRead;
    uint64_t m_nextCompare;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;

    static bool   Initialize(uint32_t timer, uint32_t* ISR, void* ISR_Param);
    static bool   Uninitialize(uint32_t timer);

    static uint32_t ReadCounter(uint32_t timer);

    static void EnableCompareInterrupt(uint32_t timer) {

        LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

        TIMER.MCR |= LPC24XX_TIMER::MR0_IRQEN;
    }

    static void DisableCompareInterrupt(uint32_t timer) {

        LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

        TIMER.MCR &= ~LPC24XX_TIMER::MR0_IRQEN;
    }

    static void ForceInterrupt(uint32_t timer) {

        LPC24XX::VIC().ForceInterrupt(LPC24XX_TIMER::getIntNo(timer));
    }

    static void SetCompare(uint32_t timer, uint32_t Compare) {

        LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

        TIMER.MR0 = Compare;
    }

    static bool DidCompareHit(uint32_t timer) {

        LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

        return (TIMER.IR & LPC24XX_TIMER::MR0_COMP) != 0;
    }

    static void ResetCompareHit(uint32_t timer) {

        LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

        TIMER.IR = LPC24XX_TIMER::MR0_RESET;
    }
};
//
// LPC24 TIMER driver
//////////////////////////////////////////////////////////////////////////////

LPC24_Timer_Controller g_LPC24_Timer_Controller;

bool LPC24_Timer_Controller::Initialize(uint32_t timer, uint32_t* ISR, void* ISR_Param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_LPC24_Timer_Controller.m_configured == true) return false;

    //--//

    if (ISR) {
        if (!LPC24_Interrupt_Activate(LPC24XX_TIMER::getIntNo(timer), ISR, ISR_Param)) return false;
    }

    LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

    TIMER.TCR = LPC24XX_TIMER::TCR_TEN;

    //--//

    g_LPC24_Timer_Controller.m_configured = true;

    return true;
}

bool LPC24_Timer_Controller::Uninitialize(uint32_t timer) {

    DISABLE_INTERRUPTS_SCOPED(irq);

    if (g_LPC24_Timer_Controller.m_configured == false) return false;

    //--//

    if (!LPC24_Interrupt_Deactivate(LPC24XX_TIMER::getIntNo(timer))) return false;

    LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

    //Reset timer
    TIMER.TCR = 0x2;
    // disable timer
    TIMER.TCR = 0;

    //--//

    g_LPC24_Timer_Controller.m_configured = false;

    return true;
}

#pragma arm section code = "SectionForFlashOperations"

uint32_t __section("SectionForFlashOperations") LPC24_Timer_Controller::ReadCounter(uint32_t timer) {

    LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

    return TIMER.TC;
}

#pragma arm section code
//--//

void LPC24_Time_InterruptHandler(void* Param) {
    TinyCLR_NativeTime_Provider *provider = (TinyCLR_NativeTime_Provider*)Param;

    int32_t timer = 0;

    if (provider != nullptr)
        timer = provider->Index;

    if (LPC24_Timer_Controller::DidCompareHit(timer)) {
        LPC24_Timer_Controller::ResetCompareHit(timer);
    }

    g_LPC24_Timer_Controller.m_lastRead = LPC24_Time_GetCurrentTicks(provider);

    if (g_LPC24_Timer_Controller.m_lastRead >= g_LPC24_Timer_Controller.m_nextCompare) {
        // this also schedules the next one, if there is one
        g_LPC24_Timer_Controller.m_DequeuAndExecute();
    }
    else {
        //
        // Because we are limited in the resolution of timer,
        // resetting the compare will properly configure the next interrupt.
        //
        LPC24_Time_SetCompare(provider, g_LPC24_Timer_Controller.m_nextCompare);
    }
}

static TinyCLR_NativeTime_Provider timeProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* LPC24_Time_GetApi() {
    timeProvider.Parent = &timeApi;
    timeProvider.Index = 0;
    timeProvider.ConvertNativeTimeToSystemTime = &LPC24_Time_GetTimeForProcessorTicks;
    timeProvider.ConvertSystemTimeToNativeTime = &LPC24_Time_TimeToTicks;
    timeProvider.GetNativeTime = &LPC24_Time_GetCurrentTicks;
    timeProvider.SetCallback = &LPC24_Time_SetCompareCallback;
    timeProvider.ScheduleCallback = &LPC24_Time_SetCompare;
    timeProvider.Acquire = &LPC24_Time_Acquire;
    timeProvider.Release = &LPC24_Time_Release;
    timeProvider.WaitMicroseconds = &LPC24_Time_Delay;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.NativeTimeProvider";
    timeApi.Type = TinyCLR_Api_Type::NativeTimeProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &timeProvider;

    return &timeApi;
}

uint32_t LPC24_Time_GetSystemClock(const TinyCLR_NativeTime_Provider* self) {
    return SYSTEM_CLOCK_HZ;
}

uint32_t LPC24_Time_GetTicksPerSecond(const TinyCLR_NativeTime_Provider* self) {
    return SLOW_CLOCKS_PER_SECOND;
}

uint32_t LPC24_Time_GetSystemCycleClock(const TinyCLR_NativeTime_Provider* self) {
    return LPC24_AHB_CLOCK_HZ;
}

uint64_t LPC24_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Provider* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t LPC24_Time_TimeToTicks(const TinyCLR_NativeTime_Provider* self, uint64_t time) {
    return LPC24_Time_MicrosecondsToTicks(self, time / 10);
}

uint64_t LPC24_Time_MillisecondsToTicks(const TinyCLR_NativeTime_Provider* self, uint64_t ticks) {
    ticks *= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_MILLISECOND_GCD);
    ticks /= (1000 / SLOW_CLOCKS_MILLISECOND_GCD);

    return ticks;
}

uint64_t LPC24_Time_MicrosecondsToTicks(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds) {
#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return microseconds * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return microseconds / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif
}

uint64_t LPC24_Time_GetCurrentTicks(const TinyCLR_NativeTime_Provider* self) {
    int32_t timer = 0;

    if (self != nullptr)
        timer = self->Index;

    uint64_t lastValue = g_LPC24_Timer_Controller.m_lastRead;

    uint32_t value = LPC24_Timer_Controller::ReadCounter(timer);

    uint32_t resHigh = (uint32_t)(lastValue >> 32);
    uint32_t resLow = (uint32_t)lastValue;

    if ((resLow & LPC24_TIME_OVERFLOW_FLAG) != (value & LPC24_TIME_OVERFLOW_FLAG)) {
        if ((value & LPC24_TIME_OVERFLOW_FLAG) == 0) {
            resHigh += 1;
        }
    }

    return (uint64_t)resHigh << 32 | value;
}

TinyCLR_Result LPC24_Time_SetCompare(const TinyCLR_NativeTime_Provider* self, uint64_t processorTicks) {
    int32_t timer = 0;

    if (self != nullptr)
        timer = self->Index;

    DISABLE_INTERRUPTS_SCOPED(irq);

    g_LPC24_Timer_Controller.m_nextCompare = processorTicks;

    if (g_LPC24_Timer_Controller.m_nextCompare >= TIMER_IDLE_VALUE && g_LPC24_Timer_Controller.m_lastRead >= TIMER_IDLE_VALUE) {
        g_LPC24_Timer_Controller.m_nextCompare = g_LPC24_Timer_Controller.m_nextCompare > g_LPC24_Timer_Controller.m_lastRead ? (g_LPC24_Timer_Controller.m_nextCompare - g_LPC24_Timer_Controller.m_lastRead) : 0;

        g_LPC24_Timer_Controller.m_lastRead = 0;
    }

    uint32_t highComp = (uint32_t)(g_LPC24_Timer_Controller.m_nextCompare >> 32);
    uint32_t lowComp = (uint32_t)g_LPC24_Timer_Controller.m_nextCompare;

    uint32_t highRead = (uint32_t)(g_LPC24_Timer_Controller.m_lastRead >> 32);
    uint32_t lowRead = (uint32_t)g_LPC24_Timer_Controller.m_lastRead;

    bool fForceInterrupt = false;

    uint32_t lowReadNew = LPC24_Timer_Controller::ReadCounter(timer);

    if ((lowRead & LPC24_TIME_OVERFLOW_FLAG) != (lowReadNew & LPC24_TIME_OVERFLOW_FLAG)) {
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
            uint32_t nextComp = (lowRead & LPC24_TIME_OVERFLOW_FLAG) ? 0 : LPC24_TIME_OVERFLOW_FLAG;

            int32_t diff1 = (int32_t)(lowComp - lowRead);
            int32_t diff2 = (int32_t)(nextComp - lowRead);

            if (diff1 > 0 && diff1 < diff2) {
                nextComp = lowComp;
            }

            LPC24_Timer_Controller::SetCompare(timer, nextComp);

            lowReadNew = LPC24_Timer_Controller::ReadCounter(timer);

            int32_t diff = nextComp - lowReadNew;

            if (diff < 0) {
                fForceInterrupt = true;
            }
        }
    }

    if (fForceInterrupt) {
        // Force interrupt to process this.
        LPC24_Timer_Controller::ForceInterrupt(timer);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Time_Acquire(const TinyCLR_NativeTime_Provider* self) {
    int32_t timer = self->Index;

    g_LPC24_Timer_Controller.m_lastRead = 0;
    g_LPC24_Timer_Controller.m_nextCompare = TIMER_IDLE_VALUE;

    if (!LPC24_Timer_Controller::Initialize(timer, (uint32_t*)&LPC24_Time_InterruptHandler, (void*)self))
        return TinyCLR_Result::InvalidOperation;

    LPC24_Timer_Controller::SetCompare(timer, LPC24_TIME_OVERFLOW_FLAG);

    LPC24_Timer_Controller::EnableCompareInterrupt(timer);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Time_Release(const TinyCLR_NativeTime_Provider* self) {
    int32_t timer = self->Index;

    if (LPC24_Timer_Controller::Uninitialize(timer) == false)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Time_SetCompareCallback(const TinyCLR_NativeTime_Provider* self, TinyCLR_NativeTime_Callback callback) {
    if (g_LPC24_Timer_Controller.m_DequeuAndExecute != nullptr)
        return TinyCLR_Result::InvalidOperation;

    g_LPC24_Timer_Controller.m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

void LPC24_Time_DelayNoInterrupt(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint64_t current = LPC24_Time_GetCurrentTicks(self);
    uint64_t maxDiff = LPC24_Time_MicrosecondsToTicks(self, microseconds);

    if (maxDiff <= CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS) maxDiff = 0;
    else maxDiff -= CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS;

    while (((int32_t)(LPC24_Time_GetCurrentTicks(self) - current)) <= maxDiff);

}

extern "C" void IDelayLoop(int32_t iterations);

void LPC24_Time_Delay(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= (LPC24_AHB_CLOCK_HZ / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int32_t iterations = (int32_t)microseconds - 5;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

//******************** Profiler ********************

