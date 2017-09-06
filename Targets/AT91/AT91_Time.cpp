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

#define AT91_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 4

//////////////////////////////////////////////////////////////////////////////
// TIMER driver
//
struct AT91_TIMER_Driver
{
    static const uint32_t c_SystemTimer = 0;
    static const uint32_t c_MaxTimerValue = 0xFFFFFFFF;  // Change to 32 bit TC

    static const uint32_t c_MaxTimer = 3;
    //--//

    static bool Initialize(uint32_t Timer, bool FreeRunning, uint32_t ClkSource, uint32_t* ISR, void* ISR_Param);
    static bool Uninitialize(uint32_t Timer);
    static void ISR_TIMER(void* param);

    static void EnableCompareInterrupt(uint32_t Timer)
    {

        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;

        AT91_TC &TC = AT91::TIMER(Timer);


        (void)TC.TC_SR;
        TC.TC_IER = AT91_TC::TC_CPCS;
        TC.TC_CCR = (AT91_TC::TC_SWTRG | AT91_TC::TC_CLKEN);
    }

    static void DisableCompareInterrupt(uint32_t Timer)
    {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;


        AT91_TC &TC = AT91::TIMER(Timer);

        TC.TC_IDR = AT91_TC::TC_CPCS;
    }

    static void ForceInterrupt(uint32_t Timer)
    {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;


        ASSERT_IRQ_MUST_BE_OFF();

        AT91_Interrupt_ForceInterrupt(AT91C_ID_TC0_TC1);
    }

    static void SetCompare(uint32_t Timer, uint32_t Compare)  // Change to 32 bit TC
    {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return;


        AT91_TC &TC = AT91::TIMER(Timer);

        TC.TC_RC = Compare;
    }

    static uint32_t ReadCounter(uint32_t Timer)
    {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return 0;


        AT91_TC &TC = AT91::TIMER(Timer);
        return    TC.TC_CV;
    }

    //    static UINT16 ReadCounter( uint32_t Timer )
    //    {
    //        ASSERT(Timer < AT91_TIMER_Driver::c_MaxTimer);
    //        AT91_TC &TC = AT91::TIMER(Timer);
    //        return    TC.TC_CV;
    //    }

    static bool DidTimerOverFlow(uint32_t Timer)
    {
        if (!(Timer < AT91_TIMER_Driver::c_MaxTimer))
            return false;


        AT91_TC &TC = AT91::TIMER(Timer);

        return (TC.TC_SR & AT91_TC::TC_COVFS) != 0;
    }

    //--//

private:
    struct Descriptors
    {
        AT91_Interrupt_Callback isr;
        void* arg;
        bool configured;
    };
    Descriptors m_descriptors[c_MaxTimer];




};

AT91_TIMER_Driver g_AT91_TIMER_Driver;

bool AT91_TIMER_Driver::Initialize(uint32_t timer, bool freeRunning, uint32_t clkSource, uint32_t* ISR, void* ISR_Param)
{
    GLOBAL_LOCK(irq);

    if (!(timer < AT91_TIMER_Driver::c_MaxTimer))
        return false;

    if (g_AT91_TIMER_Driver.m_descriptors[timer].configured == true) return false;

    //g_AT91_TIMER_Driver.m_descriptors[timer].isr = ISR;
    //g_AT91_TIMER_Driver.m_descriptors[timer].arg = ISR_Param;
    g_AT91_TIMER_Driver.m_descriptors[timer].isr.Initialize(ISR, (void*)(size_t)ISR_Param);

    //--//

    if (ISR)
    {
        if (!AT91_Interrupt_Activate(AT91C_ID_TC0_TC1, (uint32_t*)&ISR_TIMER, (void*)timer))
            return false;
    }

    {
        AT91_TC &tc = AT91::TIMER(timer);


        // First, enable the clock of the TIMER
        AT91_PMC &pmc = AT91::PMC();
        //pmc.PMC_PCER = (1 << (AT91C_ID_TC0+timer));//hyddra
        pmc.PMC_PCER = (1 << (17 + timer));//G400

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

bool AT91_TIMER_Driver::Uninitialize(uint32_t timer)
{
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
    *((volatile int*)(AT91C_BASE_PMC + 0x014)) = (1 << AT91C_ID_TC0_TC1);

    g_AT91_TIMER_Driver.m_descriptors[timer].configured = false;

    return true;
}

void AT91_TIMER_Driver::ISR_TIMER(void* param)
{
    uint32_t timer = (uint32_t)param;
    if (!(timer < AT91_TIMER_Driver::c_MaxTimer))
        return;

    // Execute the ISR for the Timer
    //(g_AT91_TIMER_Driver.m_descriptors[timer].isr)(g_AT91_TIMER_Driver.m_descriptors[timer].arg);
    g_AT91_TIMER_Driver.m_descriptors[timer].isr.Execute();
}

//////////////////////////////////////////////////////////////////////////////
// AT91_TIME_Driver
//
struct AT91_TIME_Driver
{
    uint64_t m_lastRead;
    uint64_t m_nextCompare;

    TinyCLR_Time_TickCallback m_DequeuAndExecute;

    // static bool Initialize  ();
    // static bool Uninitialize();
    // static uint64_t CounterValue();
    // static void SetCompareValue( uint64_t processorTicks );
    // static int64_t TicksToTime( uint64_t Ticks );
    // static int64_t CurrentTime();
    // static void Sleep_uSec( uint32_t uSec );
    // static void Sleep_uSec_Loop( uint32_t uSec );
    // static void ISR( void* Param );
};

AT91_TIME_Driver g_AT91_TIME_Driver;
//
// AT91_TIME_Driver
//////////////////////////////////////////////////////////////////////////////


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

void AT91_Time_InterruptHandler(void* Param) {
    TinyCLR_Time_Provider *provider = (TinyCLR_Time_Provider*)Param;

    if (AT91_Time_GetCurrentTicks(provider) >= g_AT91_TIME_Driver.m_nextCompare)
    {
        // this also schedules the next one, if there is one
        g_AT91_TIME_Driver.m_DequeuAndExecute();
    }
    else
    {
        //
        // Because we are limited in the resolution of timer,
        // resetting the compare will properly configure the next interrupt.
        //
        AT91_Time_SetCompare(provider, g_AT91_TIME_Driver.m_nextCompare);
    }
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

    GLOBAL_LOCK(irq);

    uint32_t value = AT91_TIMER_Driver::ReadCounter(AT91_TIMER_Driver::c_SystemTimer);
    //    UINT16 value = AT91_TIMER_Driver::ReadCounter( AT91_TIMER_Driver::c_SystemTimer );

    g_AT91_TIME_Driver.m_lastRead &= (0xFFFFFFFF00000000ull);
    //    g_AT91_TIME_Driver.m_lastRead &= (0xFFFFFFFFFFFF0000ull);

    if (AT91_TIMER_Driver::DidTimerOverFlow(AT91_TIMER_Driver::c_SystemTimer))
    {
        g_AT91_TIME_Driver.m_lastRead += (0x1ull << 32);
        //        g_AT91_TIME_Driver.m_lastRead += (0x1ull << 16);
    }

    g_AT91_TIME_Driver.m_lastRead |= value;

    return (uint64_t)g_AT91_TIME_Driver.m_lastRead;
}

TinyCLR_Result AT91_Time_SetCompare(const TinyCLR_Time_Provider* self, uint64_t processorTicks) {
    int32_t timer = 0;

    if (self != nullptr)
        timer = self->Index;

    GLOBAL_LOCK(irq);

    g_AT91_TIME_Driver.m_nextCompare = processorTicks;

    bool fForceInterrupt = false;

    uint64_t CntrValue = AT91_Time_GetCurrentTicks(self);

    if (processorTicks <= CntrValue)
    {
        fForceInterrupt = true;
    }
    else
    {
        uint32_t diff;

        if ((processorTicks - CntrValue) > AT91_TIMER_Driver::c_MaxTimerValue)
        {
            diff = AT91_TIMER_Driver::c_MaxTimerValue;
        }
        else
        {
            diff = (uint32_t)(processorTicks - CntrValue);
        }

        AT91_TIMER_Driver::SetCompare(AT91_TIMER_Driver::c_SystemTimer,
            (uint32_t)(AT91_TIMER_Driver::ReadCounter(AT91_TIMER_Driver::c_SystemTimer) + diff));

        if (AT91_Time_GetCurrentTicks(self) > processorTicks)
        {
            fForceInterrupt = true;
        }
    }

    if (fForceInterrupt)
    {
        // Force interrupt to process this.
        AT91_TIMER_Driver::ForceInterrupt(AT91_TIMER_Driver::c_SystemTimer);
    }


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Acquire(const TinyCLR_Time_Provider* self) {
    int32_t timer = self->Index;

    g_AT91_TIME_Driver.m_lastRead = 0;
    g_AT91_TIME_Driver.m_nextCompare = (uint64_t)AT91_TIMER_Driver::c_MaxTimerValue;

    if (!AT91_TIMER_Driver::Initialize(AT91_TIMER_Driver::c_SystemTimer, true, AT91_TC::TC_CLKS_TIMER_DIV3_CLOCK, (uint32_t*)&AT91_Time_InterruptHandler, (void*)self))
        return TinyCLR_Result::InvalidOperation;;

    AT91_TIMER_Driver::SetCompare(AT91_TIMER_Driver::c_SystemTimer, AT91_TIMER_Driver::c_MaxTimerValue);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_Release(const TinyCLR_Time_Provider* self) {
    int32_t timer = self->Index;

    if (!AT91_TIMER_Driver::Uninitialize(AT91_TIMER_Driver::c_SystemTimer))
        return TinyCLR_Result::InvalidOperation;;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Time_SetCompareCallback(const TinyCLR_Time_Provider* self, TinyCLR_Time_TickCallback callback) {
    if (g_AT91_TIME_Driver.m_DequeuAndExecute != nullptr)
        return TinyCLR_Result::InvalidOperation;

    g_AT91_TIME_Driver.m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

void AT91_Time_DelayNoInterrupt(const TinyCLR_Time_Provider* self, uint64_t microseconds) {
    GLOBAL_LOCK(irq);

    uint64_t value = AT91_TIMER_Driver::ReadCounter(AT91_TIMER_Driver::c_SystemTimer);
    uint64_t maxDiff = AT91_Time_MicrosecondsToTicks(self, microseconds);      // The free-running timer clocks at a constant 3.25 MHz

    if (maxDiff <= AT91_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS) maxDiff = AT91_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS;
    else                                                 maxDiff -= AT91_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS;

    while ((AT91_TIMER_Driver::ReadCounter(AT91_TIMER_Driver::c_SystemTimer) - value) <= maxDiff);


}

extern "C" void IDelayLoop(int32_t iterations);

void AT91_Time_Delay(const TinyCLR_Time_Provider* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= ((SYSTEM_CYCLE_CLOCK_HZ / 2) / CLOCK_COMMON_FACTOR);
    microseconds /= (ONE_MHZ / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int iterations = (int)microseconds - 14;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

//******************** Profiler ********************

