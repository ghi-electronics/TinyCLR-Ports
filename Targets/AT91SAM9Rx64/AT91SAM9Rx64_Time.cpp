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

#include "AT91SAM9Rx64.h"

#define TIMER_IDLE_VALUE  0x0000FFFFFFFFFFFFFull

#define AT91SAM9Rx64_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 4

#define SLOW_CLOCKS_PER_SECOND              (AT91SAM9Rx64_SYSTEM_PERIPHERAL_CLOCK_HZ / 128)
#define CLOCK_COMMON_FACTOR                 250
#define SLOW_CLOCKS_TEN_MHZ_GCD             250
#define SLOW_CLOCKS_MILLISECOND_GCD         250

//////////////////////////////////////////////////////////////////////////////
// TIMER state
//
struct At91TimerDriver {
    static const uint32_t c_SystemTimer = 0;
    static const uint32_t c_MaxTimerValue = 0xFFFFFFFF;  // Change to 32 bit TC

    static const uint32_t c_MaxTimer = 3;
    //--//

    static bool Initialize(uint32_t Timer, bool FreeRunning, uint32_t ClkSource, uint32_t* ISR, void* ISR_Param);
    static bool Uninitialize(uint32_t Timer);
    static void ISR_TIMER(void* param);

    static void EnableCompareInterrupt(uint32_t Timer) {

        if (!(Timer < At91TimerDriver::c_MaxTimer))
            return;

        AT91SAM9Rx64_TC &TC = AT91::TIMER(Timer);

        (void)TC.TC_SR;
        TC.TC_IER = AT91SAM9Rx64_TC::TC_CPCS;
        TC.TC_CCR = (AT91SAM9Rx64_TC::TC_SWTRG | AT91SAM9Rx64_TC::TC_CLKEN);
    }

    static void DisableCompareInterrupt(uint32_t Timer) {
        if (!(Timer < At91TimerDriver::c_MaxTimer))
            return;


        AT91SAM9Rx64_TC &TC = AT91::TIMER(Timer);

        TC.TC_IDR = AT91SAM9Rx64_TC::TC_CPCS;
    }

    static void ForceInterrupt(uint32_t Timer) {
        if (!(Timer < At91TimerDriver::c_MaxTimer))
            return;

        AT91SAM9Rx64_Interrupt_ForceInterrupt(AT91C_ID_TC0);
    }

    static void SetCompare(uint32_t Timer, uint32_t Compare)  // Change to 32 bit TC
    {
        if (!(Timer < At91TimerDriver::c_MaxTimer))
            return;

        AT91SAM9Rx64_TC &TC = AT91::TIMER(Timer);

        TC.TC_RC = Compare;
    }

    static uint32_t ReadCounter(uint32_t Timer) {
        if (!(Timer < At91TimerDriver::c_MaxTimer))
            return 0;

        AT91SAM9Rx64_TC &TC = AT91::TIMER(Timer);
        return    TC.TC_CV;
    }

    static bool DidTimerOverFlow(uint32_t Timer) {
        if (!(Timer < At91TimerDriver::c_MaxTimer))
            return false;

        AT91SAM9Rx64_TC &TC = AT91::TIMER(Timer);

        return (TC.TC_SR & AT91SAM9Rx64_TC::TC_COVFS) != 0;
    }

    //--//

private:
    struct Descriptors {
        AT91SAM9Rx64_Interrupt_Callback isr;
        void* arg;
        bool configured;
    };
    Descriptors m_descriptors[c_MaxTimer];




};

At91TimerDriver at91TimerDriver;

bool At91TimerDriver::Initialize(uint32_t timer, bool freeRunning, uint32_t clkSource, uint32_t* ISR, void* ISR_Param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    if (!(timer < At91TimerDriver::c_MaxTimer))
        return false;

    if (at91TimerDriver.m_descriptors[timer].configured == true) return false;

    at91TimerDriver.m_descriptors[timer].isr.Initialize(ISR, (void*)(size_t)ISR_Param);

    //--//

    if (ISR) {
        if (!AT91SAM9Rx64_InterruptInternal_Activate(AT91C_ID_TC0, (uint32_t*)&ISR_TIMER, (void*)timer))
            return false;
    }

    {
        AT91SAM9Rx64_TC &tc = AT91::TIMER(timer);


        // First, enable the clock of the TIMER
        AT91SAM9Rx64_PMC &pmc = AT91::PMC();
        pmc.PMC_PCER = (1 << (AT91C_ID_TC0 + timer));//hyddra
        //pmc.PMC_PCER = (1 << (17 + timer));//G400

        // Disable the clock and the interrupts
        tc.TC_CCR = AT91SAM9Rx64_TC::TC_CLKDIS;
        tc.TC_IDR = 0xFFFFFFFF;

        // Clear status bit
        ////(void) tc.TC_SR;
        volatile int x = tc.TC_SR;

        // Set the Mode of the timer Counter
        if (freeRunning == true)
            tc.TC_CMR = (clkSource); // 2;
        else
            tc.TC_CMR = (AT91SAM9Rx64_TC::TC_CPCTRG | clkSource);

        // Enable Interrupt (Compare RC)
        tc.TC_IER = AT91SAM9Rx64_TC::TC_CPCS;

        // Enable the clock
        tc.TC_CCR = (AT91SAM9Rx64_TC::TC_CLKEN | AT91SAM9Rx64_TC::TC_SWTRG);
    }

    at91TimerDriver.m_descriptors[timer].configured = true;

    return true;
}

bool At91TimerDriver::Uninitialize(uint32_t timer) {
    // Get Timer pointer
    AT91SAM9Rx64_TC &tc = AT91::TIMER(timer);

    // Disable the clock
    tc.TC_CCR = AT91SAM9Rx64_TC::TC_CLKDIS;

    // Clear Irq Status
    (void)tc.TC_SR;

    // Disable All Irq
    tc.TC_IDR = 0xFFFFFFFF;

    // First, enable the clock of the TIMER
    // Todo clock API - PMC
    *((volatile int*)(AT91C_BASE_PMC + 0x014)) = (1 << AT91C_ID_TC0);

    at91TimerDriver.m_descriptors[timer].configured = false;

    return true;
}

void At91TimerDriver::ISR_TIMER(void* param) {
    uint32_t timer = (uint32_t)param;
    if (!(timer < At91TimerDriver::c_MaxTimer))
        return;

    // Execute the ISR for the Timer
    at91TimerDriver.m_descriptors[timer].isr.Execute();
}

//////////////////////////////////////////////////////////////////////////////
// TimeState
//
struct TimeState {
    int32_t controllerIndex;
    uint64_t m_lastRead;
    uint64_t m_nextCompare;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;
    bool tableInitialized;
};

#define TOTAL_TIME_CONTROLLERS 1

static TimeState timeStates[TOTAL_TIME_CONTROLLERS];
//
// TimeState
//////////////////////////////////////////////////////////////////////////////


static TinyCLR_NativeTime_Controller timeControllers[TOTAL_TIME_CONTROLLERS];
static TinyCLR_Api_Info timeApi[TOTAL_TIME_CONTROLLERS];

const char* timeApiNames[TOTAL_TIME_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91SAM9Rx64.NativeTimeController\\0",

};

void AT91SAM9Rx64_Time_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        if (timeStates[i].tableInitialized)
            continue;

        timeControllers[i].ApiInfo = &timeApi[i];
        timeControllers[i].Initialize = &AT91SAM9Rx64_Time_Initialize;
        timeControllers[i].Uninitialize = &AT91SAM9Rx64_Time_Uninitialize;
        timeControllers[i].GetNativeTime = &AT91SAM9Rx64_Time_GetCurrentProcessorTicks;
        timeControllers[i].ConvertNativeTimeToSystemTime = &AT91SAM9Rx64_Time_GetTimeForProcessorTicks;
        timeControllers[i].ConvertSystemTimeToNativeTime = &AT91SAM9Rx64_Time_GetProcessorTicksForTime;
        timeControllers[i].SetCallback = &AT91SAM9Rx64_Time_SetTickCallback;
        timeControllers[i].ScheduleCallback = &AT91SAM9Rx64_Time_SetNextTickCallbackTime;
        timeControllers[i].Wait = &AT91SAM9Rx64_Time_DelayNative;

        timeApi[i].Author = "GHI Electronics, LLC";
        timeApi[i].Name = timeApiNames[i];
        timeApi[i].Type = TinyCLR_Api_Type::NativeTimeController;
        timeApi[i].Version = 0;
        timeApi[i].Implementation = &timeControllers[i];
        timeApi[i].State = &timeStates[i];

        timeStates[i].controllerIndex = i;
        timeStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* AT91SAM9Rx64_Time_GetRequiredApi() {
    AT91SAM9Rx64_Time_EnsureTableInitialized();

    return &timeApi[0];
}

void AT91SAM9Rx64_Time_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91SAM9Rx64_Time_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &timeApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::NativeTimeController, timeApi[0].Name);
}

void AT91SAM9Rx64_Time_InterruptHandler(void* Param) {
    TinyCLR_NativeTime_Controller *self = (TinyCLR_NativeTime_Controller*)Param;

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    if (AT91SAM9Rx64_Time_GetCurrentProcessorTicks(self) >= state->m_nextCompare) {
        // this also schedules the next one, if there is one
        state->m_DequeuAndExecute();
    }
    else {
        //
        // Because we are limited in the resolution of timer,
        // resetting the compare will properly configure the next interrupt.
        //
        AT91SAM9Rx64_Time_SetNextTickCallbackTime(self, state->m_nextCompare);
    }
}

uint32_t AT91SAM9Rx64_Time_GetTicksPerSecond(const TinyCLR_NativeTime_Controller* self) {
    return SLOW_CLOCKS_PER_SECOND;
}

uint64_t AT91SAM9Rx64_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t AT91SAM9Rx64_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time) {
    return AT91SAM9Rx64_Time_MicrosecondsToTicks(self, time / 10);
}

uint64_t AT91SAM9Rx64_Time_GetCurrentProcessorTime() {
    return AT91SAM9Rx64_Time_GetTimeForProcessorTicks(nullptr, AT91SAM9Rx64_Time_GetCurrentProcessorTicks(nullptr));
}

uint64_t AT91SAM9Rx64_Time_MillisecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks) {
    ticks *= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_MILLISECOND_GCD);
    ticks /= (1000 / SLOW_CLOCKS_MILLISECOND_GCD);

    return ticks;
}

uint64_t AT91SAM9Rx64_Time_MicrosecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds) {
#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return microseconds * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return microseconds / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif

}

uint64_t AT91SAM9Rx64_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self) {
    int32_t timer = AT91SAM9Rx64_TIME_DEFAULT_CONTROLLER_ID;

    DISABLE_INTERRUPTS_SCOPED(irq);

    uint16_t value = At91TimerDriver::ReadCounter(At91TimerDriver::c_SystemTimer);

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    state->m_lastRead &= (0xFFFFFFFFFFFF0000ull);

    if (At91TimerDriver::DidTimerOverFlow(At91TimerDriver::c_SystemTimer)) {
        state->m_lastRead += (0x1ull << 16);
    }

    state->m_lastRead |= value;

    return (uint64_t)state->m_lastRead;
}

TinyCLR_Result AT91SAM9Rx64_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks) {
    int32_t timer = AT91SAM9Rx64_TIME_DEFAULT_CONTROLLER_ID;

    DISABLE_INTERRUPTS_SCOPED(irq);

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    state->m_nextCompare = processorTicks;

    bool fForceInterrupt = false;

    uint64_t ticks = AT91SAM9Rx64_Time_GetCurrentProcessorTicks(self);

    if (state->m_nextCompare >= TIMER_IDLE_VALUE && ticks >= TIMER_IDLE_VALUE) {
        // Calculate next compare after overflow
        if (state->m_nextCompare > processorTicks)
            state->m_nextCompare = state->m_nextCompare - processorTicks;
        else
            state->m_nextCompare = 0;

        // Reset current tick
        state->m_lastRead = 0;
        ticks = AT91SAM9Rx64_Time_GetCurrentProcessorTicks(self);
    }

    if (ticks >= state->m_nextCompare) {
        fForceInterrupt = true;
    }
    else {
        uint32_t diff;

        if ((state->m_nextCompare - ticks) > At91TimerDriver::c_MaxTimerValue) {
            diff = At91TimerDriver::c_MaxTimerValue;
        }
        else {
            diff = (uint32_t)(state->m_nextCompare - ticks);
        }

        At91TimerDriver::SetCompare(At91TimerDriver::c_SystemTimer,
            (uint16_t)(At91TimerDriver::ReadCounter(At91TimerDriver::c_SystemTimer) + diff));

        if (AT91SAM9Rx64_Time_GetCurrentProcessorTicks(self) > state->m_nextCompare) {
            fForceInterrupt = true;
        }
    }

    if (fForceInterrupt) {
        // Force interrupt to process this.
        At91TimerDriver::ForceInterrupt(At91TimerDriver::c_SystemTimer);
    }


    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Time_Initialize(const TinyCLR_NativeTime_Controller* self) {
    int32_t timer = AT91SAM9Rx64_TIME_DEFAULT_CONTROLLER_ID;

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));
    state->m_lastRead = 0;
    state->m_nextCompare = (uint64_t)At91TimerDriver::c_MaxTimerValue;

    if (!At91TimerDriver::Initialize(At91TimerDriver::c_SystemTimer, true, AT91SAM9Rx64_TC::TC_CLKS_TIMER_DIV4_CLOCK, (uint32_t*)&AT91SAM9Rx64_Time_InterruptHandler, (void*)self))
        return TinyCLR_Result::InvalidOperation;;

    At91TimerDriver::SetCompare(At91TimerDriver::c_SystemTimer, At91TimerDriver::c_MaxTimerValue);

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self) {
    int32_t timer = AT91SAM9Rx64_TIME_DEFAULT_CONTROLLER_ID;

    if (!At91TimerDriver::Uninitialize(At91TimerDriver::c_SystemTimer))
        return TinyCLR_Result::InvalidOperation;;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9Rx64_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback) {
    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    if (state->m_DequeuAndExecute != nullptr)
        return TinyCLR_Result::InvalidOperation;

    state->m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

extern "C" void IDelayLoop(int32_t iterations);

void AT91SAM9Rx64_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= ((AT91SAM9Rx64_AHB_CLOCK_HZ / 2) / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int iterations = (int)microseconds - 14;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

void AT91SAM9Rx64_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime) {
    //TODO do inline later, don't call out to Delay

    auto microseconds = AT91SAM9Rx64_Time_GetTimeForProcessorTicks(self, nativeTime) / 10;

    AT91SAM9Rx64_Time_Delay(self, microseconds);
}
