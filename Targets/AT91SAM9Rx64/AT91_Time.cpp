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

#define TIMER_IDLE_VALUE  0x0000FFFFFFFFFFFFFull

#define AT91_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 4

#define SLOW_CLOCKS_PER_SECOND              (AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 128)
#define CLOCK_COMMON_FACTOR                 250
#define SLOW_CLOCKS_TEN_MHZ_GCD             250
#define SLOW_CLOCKS_MILLISECOND_GCD         250

//////////////////////////////////////////////////////////////////////////////
// TIMER driver
//
struct AT91_TIMER_Driver {
    static const uint32_t c_SystemTimer = 0;
    static const uint32_t c_MaxTimerValue = 0xFFFFFFFF;  // Change to 32 bit TC

    static const uint32_t c_MaxTimer = 3;
    //--//

    static bool Initialize(uint32_t Timer, bool FreeRunning, uint32_t ClkSource, uint32_t* ISR, void* ISR_Param);
    static bool Uninitialize(uint32_t Timer);
    static void ISR_TIMER(void* param);

    static void EnableCompareInterrupt(uint32_t Timer) {

        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;

        AT91_TC &TC = AT91::TIMER(Timer);

        (void)TC.TC_SR;
        TC.TC_IER = AT91_TC::TC_CPCS;
        TC.TC_CCR = (AT91_TC::TC_SWTRG | AT91_TC::TC_CLKEN);
    }

    static void DisableCompareInterrupt(uint32_t Timer) {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;


        AT91_TC &TC = AT91::TIMER(Timer);

        TC.TC_IDR = AT91_TC::TC_CPCS;
    }

    static void ForceInterrupt(uint32_t Timer) {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;

        AT91_Interrupt_ForceInterrupt(AT91C_ID_TC0);
    }

    static void SetCompare(uint32_t Timer, uint32_t Compare)  // Change to 32 bit TC
    {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;

        AT91_TC &TC = AT91::TIMER(Timer);

        TC.TC_RC = Compare;
    }

    static uint32_t ReadCounter(uint32_t Timer) {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return 0;

        AT91_TC &TC = AT91::TIMER(Timer);
        return    TC.TC_CV;
    }

    static bool DidTimerOverFlow(uint32_t Timer) {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return false;

        AT91_TC &TC = AT91::TIMER(Timer);

        return (TC.TC_SR & AT91_TC::TC_COVFS) != 0;
    }

    //--//

private:
    struct Descriptors {
        AT91_Interrupt_Callback isr;
        void* arg;
        bool configured;
    };
    Descriptors m_descriptors[c_MaxTimer];




};

AT91_TIMER_Driver g_AT91_TIMER_Driver;

bool AT91_TIMER_Driver::Initialize(uint32_t timer, bool freeRunning, uint32_t clkSource, uint32_t* ISR, void* ISR_Param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (!(timer < AT91_TIMER_Driver::c_MaxTimer))
        return false;

    if (g_AT91_TIMER_Driver.m_descriptors[timer].configured == true) return false;

    g_AT91_TIMER_Driver.m_descriptors[timer].isr.Initialize(ISR, (void*)(size_t)ISR_Param);

    //--//

    if (ISR) {
        if (!AT91_Interrupt_Activate(AT91C_ID_TC0, (uint32_t*)&ISR_TIMER, (void*)timer))
            return false;
    }

    {
        AT91_TC &tc = AT91::TIMER(timer);


        // First, enable the clock of the TIMER
        AT91_PMC &pmc = AT91::PMC();
        pmc.PMC_PCER = (1 << (AT91C_ID_TC0 + timer));//hyddra
        //pmc.PMC_PCER = (1 << (17 + timer));//G400

        // Disable the clock and the interrupts
        tc.TC_CCR = AT91_TC::TC_CLKDIS;
        tc.TC_IDR = 0xFFFFFFFF;

        // Clear status bit
        ////(void) tc.TC_SR;
        volatile int x = tc.TC_SR;

        // Set the Mode of the timer Counter
        if (freeRunning == true)
            tc.TC_CMR = (clkSource); // 2;
        else
            tc.TC_CMR = (AT91_TC::TC_CPCTRG | clkSource);

        // Enable Interrupt (Compare RC)
        tc.TC_IER = AT91_TC::TC_CPCS;

        // Enable the clock
        tc.TC_CCR = (AT91_TC::TC_CLKEN | AT91_TC::TC_SWTRG);
    }

    g_AT91_TIMER_Driver.m_descriptors[timer].configured = true;

    return true;
}

bool AT91_TIMER_Driver::Uninitialize(uint32_t timer) {
    // Get Timer pointer
    AT91_TC &tc = AT91::TIMER(timer);

    // Disable the clock
    tc.TC_CCR = AT91_TC::TC_CLKDIS;

    // Clear Irq Status
    (void)tc.TC_SR;

    // Disable All Irq
    tc.TC_IDR = 0xFFFFFFFF;

    // First, enable the clock of the TIMER
    // Todo clock API - PMC
    *((volatile int*)(AT91C_BASE_PMC + 0x014)) = (1 << AT91C_ID_TC0);

    g_AT91_TIMER_Driver.m_descriptors[timer].configured = false;

    return true;
}

void AT91_TIMER_Driver::ISR_TIMER(void* param) {
    uint32_t timer = (uint32_t)param;
    if (!(timer < AT91_TIMER_Driver::c_MaxTimer))
        return;

    // Execute the ISR for the Timer
    g_AT91_TIMER_Driver.m_descriptors[timer].isr.Execute();
}

//////////////////////////////////////////////////////////////////////////////
// AT91_TIME_Driver
//
struct AT91_TIME_Driver {
    uint64_t m_lastRead;
    uint64_t m_nextCompare;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;
};

AT91_TIME_Driver g_AT91_TIME_Driver;
//
// AT91_TIME_Driver
//////////////////////////////////////////////////////////////////////////////


static TinyCLR_NativeTime_Provider timeProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* AT91_Time_GetApi() {
    timeProvider.Parent = &timeApi;
    timeProvider.ConvertNativeTimeToSystemTime = &AT91_Time_GetTimeForProcessorTicks;
    timeProvider.ConvertSystemTimeToNativeTime = &AT91_Time_TimeToTicks;
    timeProvider.GetNativeTime = &AT91_Time_GetCurrentProcessorTicks;
    timeProvider.SetCallback = &AT91_Time_SetTickCallback;
    timeProvider.ScheduleCallback = &AT91_Time_SetNextTickCallbackTime;
    timeProvider.Acquire = &AT91_Time_Acquire;
    timeProvider.Release = &AT91_Time_Release;
    timeProvider.WaitMicroseconds = &AT91_Time_Delay;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.NativeTimeProvider";
    timeApi.Type = TinyCLR_Api_Type::NativeTimeProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &timeProvider;

    return &timeApi;
}

void AT91_Time_InterruptHandler(void* Param) {
    TinyCLR_NativeTime_Provider *provider = (TinyCLR_NativeTime_Provider*)Param;

    if (AT91_Time_GetCurrentProcessorTicks(provider) >= g_AT91_TIME_Driver.m_nextCompare) {
        // this also schedules the next one, if there is one
        g_AT91_TIME_Driver.m_DequeuAndExecute();
    }
    else {
        //
        // Because we are limited in the resolution of timer,
        // resetting the compare will properly configure the next interrupt.
        //
        AT91_Time_SetNextTickCallbackTime(provider, g_AT91_TIME_Driver.m_nextCompare);
    }
}

uint32_t AT91_Time_GetTicksPerSecond(const TinyCLR_NativeTime_Provider* self) {
    return SLOW_CLOCKS_PER_SECOND;
}

uint64_t AT91_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Provider* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t AT91_Time_TimeToTicks(const TinyCLR_NativeTime_Provider* self, uint64_t time) {
    return AT91_Time_MicrosecondsToTicks(self, time / 10);
}

uint64_t AT91_Time_MillisecondsToTicks(const TinyCLR_NativeTime_Provider* self, uint64_t ticks) {
    ticks *= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_MILLISECOND_GCD);
    ticks /= (1000 / SLOW_CLOCKS_MILLISECOND_GCD);

    return ticks;
}

uint64_t AT91_Time_MicrosecondsToTicks(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds) {
#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return microseconds * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return microseconds / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif

}

uint64_t AT91_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Provider* self) {
    int32_t timer = AT91_TIME_DEFAULT_CONTROLLER_ID;

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint16_t value = AT91_TIMER_Driver::ReadCounter(AT91_TIMER_Driver::c_SystemTimer);

    g_AT91_TIME_Driver.m_lastRead &= (0xFFFFFFFFFFFF0000ull);

    if (AT91_TIMER_Driver::DidTimerOverFlow(AT91_TIMER_Driver::c_SystemTimer)) {
        g_AT91_TIME_Driver.m_lastRead += (0x1ull << 16);
    }

    g_AT91_TIME_Driver.m_lastRead |= value;

    return (uint64_t)g_AT91_TIME_Driver.m_lastRead;
}

TinyCLR_Result AT91_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Provider* self, uint64_t processorTicks) {
    int32_t timer = AT91_TIME_DEFAULT_CONTROLLER_ID;

    DISABLE_INTERRUPTS_SCOPED(irq);

    g_AT91_TIME_Driver.m_nextCompare = processorTicks;

    bool fForceInterrupt = false;

    uint64_t ticks = AT91_Time_GetCurrentProcessorTicks(self);

    if (g_AT91_TIME_Driver.m_nextCompare >= TIMER_IDLE_VALUE && ticks >= TIMER_IDLE_VALUE) {
        // Calculate next compare after overflow
        if (g_AT91_TIME_Driver.m_nextCompare > processorTicks)
            g_AT91_TIME_Driver.m_nextCompare = g_AT91_TIME_Driver.m_nextCompare - processorTicks;
        else
            g_AT91_TIME_Driver.m_nextCompare = 0;

        // Reset current tick
        g_AT91_TIME_Driver.m_lastRead = 0;
        ticks = AT91_Time_GetCurrentProcessorTicks(self);
    }

    if (ticks >= g_AT91_TIME_Driver.m_nextCompare) {
        fForceInterrupt = true;
    }
    else {
        uint32_t diff;

        if ((g_AT91_TIME_Driver.m_nextCompare - ticks) > AT91_TIMER_Driver::c_MaxTimerValue) {
            diff = AT91_TIMER_Driver::c_MaxTimerValue;
        }
        else {
            diff = (uint32_t)(g_AT91_TIME_Driver.m_nextCompare - ticks);
        }

        AT91_TIMER_Driver::SetCompare(AT91_TIMER_Driver::c_SystemTimer,
            (uint16_t)(AT91_TIMER_Driver::ReadCounter(AT91_TIMER_Driver::c_SystemTimer) + diff));

        if (AT91_Time_GetCurrentProcessorTicks(self) > g_AT91_TIME_Driver.m_nextCompare) {
            fForceInterrupt = true;
        }
    }

    if (fForceInterrupt) {
        // Force interrupt to process this.
        AT91_TIMER_Driver::ForceInterrupt(AT91_TIMER_Driver::c_SystemTimer);
    }


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Acquire(const TinyCLR_NativeTime_Provider* self) {
    int32_t timer = AT91_TIME_DEFAULT_CONTROLLER_ID;

    g_AT91_TIME_Driver.m_lastRead = 0;
    g_AT91_TIME_Driver.m_nextCompare = (uint64_t)AT91_TIMER_Driver::c_MaxTimerValue;

    if (!AT91_TIMER_Driver::Initialize(AT91_TIMER_Driver::c_SystemTimer, true, AT91_TC::TC_CLKS_TIMER_DIV4_CLOCK, (uint32_t*)&AT91_Time_InterruptHandler, (void*)self))
        return TinyCLR_Result::InvalidOperation;;

    AT91_TIMER_Driver::SetCompare(AT91_TIMER_Driver::c_SystemTimer, AT91_TIMER_Driver::c_MaxTimerValue);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Release(const TinyCLR_NativeTime_Provider* self) {
    int32_t timer = AT91_TIME_DEFAULT_CONTROLLER_ID;

    if (!AT91_TIMER_Driver::Uninitialize(AT91_TIMER_Driver::c_SystemTimer))
        return TinyCLR_Result::InvalidOperation;;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_SetTickCallback(const TinyCLR_NativeTime_Provider* self, TinyCLR_NativeTime_Callback callback) {
    if (g_AT91_TIME_Driver.m_DequeuAndExecute != nullptr)
        return TinyCLR_Result::InvalidOperation;

    g_AT91_TIME_Driver.m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

extern "C" void IDelayLoop(int32_t iterations);

void AT91_Time_Delay(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= ((AT91_AHB_CLOCK_HZ / 2) / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int iterations = (int)microseconds - 14;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}


