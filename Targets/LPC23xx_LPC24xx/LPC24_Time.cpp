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

#define TOTAL_TIME_CONTROLLERS 1

#define CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 3
#define  LPC24_TIME_OVERFLOW_FLAG 0x80000000

#define CLOCK_COMMON_FACTOR                  1000000 // GCD(SYSTEM_CLOCK_HZ, 1M)
#define SLOW_CLOCKS_PER_SECOND               18000000 // SYSTEM_CLOCK_HZ MHz
#define SLOW_CLOCKS_TEN_MHZ_GCD              1000000 // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000 // GCD(SLOW_CLOCKS_PER_SECOND, 1k)

//////////////////////////////////////////////////////////////////////////////
// LPC24 TIMER state
//

struct TimeState {
    int32_t controllerIndex;

    bool m_configured;

    uint64_t m_lastRead;
    uint64_t m_nextCompare;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;
    bool tableInitialized;

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
// LPC24 TIMER state
//////////////////////////////////////////////////////////////////////////////

static TimeState timeStates[TOTAL_TIME_CONTROLLERS];

bool TimeState::Initialize(uint32_t timer, uint32_t* ISR, void* ISR_Param) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = &timeStates[LPC24_TIME_DEFAULT_CONTROLLER_ID];

    if (state->m_configured == true) return false;

    //--//

    if (ISR) {
        if (!LPC24_Interrupt_Activate(LPC24XX_TIMER::getIntNo(timer), ISR, ISR_Param)) return false;
    }

    LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

    TIMER.TCR = LPC24XX_TIMER::TCR_TEN;

    //--//

    state->m_configured = true;

    return true;
}

bool TimeState::Uninitialize(uint32_t timer) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto state = &timeStates[LPC24_TIME_DEFAULT_CONTROLLER_ID];

    if (state->m_configured == false) return false;

    //--//

    if (!LPC24_Interrupt_Deactivate(LPC24XX_TIMER::getIntNo(timer))) return false;

    LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

    //Reset timer
    TIMER.TCR = 0x2;
    // disable timer
    TIMER.TCR = 0;

    //--//

    state->m_configured = false;

    return true;
}

#pragma arm section code = "SectionForFlashOperations"

uint32_t __section("SectionForFlashOperations") TimeState::ReadCounter(uint32_t timer) {

    LPC24XX_TIMER& TIMER = LPC24XX::TIMER(timer);

    return TIMER.TC;
}

#pragma arm section code
//--//

void LPC24_Time_InterruptHandler(void* Param) {
    TinyCLR_NativeTime_Controller *provider = (TinyCLR_NativeTime_Controller*)Param;

    auto state = reinterpret_cast<TimeState*>(provider->ApiInfo->State);

    int32_t timer = LPC24_TIME_DEFAULT_CONTROLLER_ID;

    if (TimeState::DidCompareHit(timer)) {
        TimeState::ResetCompareHit(timer);
    }

    state->m_lastRead = LPC24_Time_GetCurrentProcessorTicks(provider);

    if (state->m_lastRead >= state->m_nextCompare) {
        // this also schedules the next one, if there is one
        state->m_DequeuAndExecute();
    }
    else {
        //
        // Because we are limited in the resolution of timer,
        // resetting the compare will properly configure the next interrupt.
        //
        LPC24_Time_SetNextTickCallbackTime(provider, state->m_nextCompare);
    }
}


static TinyCLR_NativeTime_Controller timeControllers[TOTAL_TIME_CONTROLLERS];
static TinyCLR_Api_Info timeApi[TOTAL_TIME_CONTROLLERS];

const char* timeApiNames[TOTAL_TIME_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.NativeTimeController\\0",

};

void LPC24_Time_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        if (timeStates[i].tableInitialized)
            continue;

        timeControllers[i].ApiInfo = &timeApi[i];
        timeControllers[i].Initialize = &LPC24_Time_Initialize;
        timeControllers[i].Uninitialize = &LPC24_Time_Uninitialize;
        timeControllers[i].GetNativeTime = &LPC24_Time_GetCurrentProcessorTicks;
        timeControllers[i].ConvertNativeTimeToSystemTime = &LPC24_Time_GetTimeForProcessorTicks;
        timeControllers[i].ConvertSystemTimeToNativeTime = &LPC24_Time_GetProcessorTicksForTime;
        timeControllers[i].SetCallback = &LPC24_Time_SetTickCallback;
        timeControllers[i].ScheduleCallback = &LPC24_Time_SetNextTickCallbackTime;
        timeControllers[i].Wait = &LPC24_Time_DelayNative;

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

const TinyCLR_Api_Info* LPC24_Time_GetRequiredApi() {
    LPC24_Time_EnsureTableInitialized();

    return &timeApi[0];
}

void LPC24_Time_AddApi(const TinyCLR_Api_Manager* apiManager) {
    LPC24_Time_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &timeApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::NativeTimeController, timeApi[0].Name);
}

uint32_t LPC24_Time_GetSystemClock(const TinyCLR_NativeTime_Controller* self) {
    return SYSTEM_CLOCK_HZ;
}

uint32_t LPC24_Time_GetTicksPerSecond(const TinyCLR_NativeTime_Controller* self) {
    return SLOW_CLOCKS_PER_SECOND;
}

uint32_t LPC24_Time_GetSystemCycleClock(const TinyCLR_NativeTime_Controller* self) {
    return LPC24_AHB_CLOCK_HZ;
}

uint64_t LPC24_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t LPC24_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time) {
    return LPC24_Time_MicrosecondsToTicks(self, time / 10);
}

uint64_t LPC24_Time_MillisecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks) {
    ticks *= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_MILLISECOND_GCD);
    ticks /= (1000 / SLOW_CLOCKS_MILLISECOND_GCD);

    return ticks;
}

uint64_t LPC24_Time_MicrosecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds) {
#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return microseconds * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return microseconds / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif
}

uint64_t LPC24_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self) {
    int32_t timer = LPC24_TIME_DEFAULT_CONTROLLER_ID;

    auto state = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    if (self == nullptr) { // some cases in hal layer call directly LPC24_Time_GetCurrentProcessorTicks(nullptr), use first controller as default
        state = &timeStates[0];
    }

    uint64_t lastValue = state->m_lastRead;

    uint32_t value = TimeState::ReadCounter(timer);

    uint32_t resHigh = (uint32_t)(lastValue >> 32);
    uint32_t resLow = (uint32_t)lastValue;

    if ((resLow & LPC24_TIME_OVERFLOW_FLAG) != (value & LPC24_TIME_OVERFLOW_FLAG)) {
        if ((value & LPC24_TIME_OVERFLOW_FLAG) == 0) {
            resHigh += 1;
        }
    }

    return (uint64_t)resHigh << 32 | value;
}

TinyCLR_Result LPC24_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks) {
    int32_t timer = LPC24_TIME_DEFAULT_CONTROLLER_ID;

    auto state = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    DISABLE_INTERRUPTS_SCOPED(irq);

    state->m_nextCompare = processorTicks;

    if (state->m_nextCompare >= TIMER_IDLE_VALUE && state->m_lastRead >= TIMER_IDLE_VALUE) {
        state->m_nextCompare = state->m_nextCompare > state->m_lastRead ? (state->m_nextCompare - state->m_lastRead) : 0;

        state->m_lastRead = 0;
    }

    uint32_t highComp = (uint32_t)(state->m_nextCompare >> 32);
    uint32_t lowComp = (uint32_t)state->m_nextCompare;

    uint32_t highRead = (uint32_t)(state->m_lastRead >> 32);
    uint32_t lowRead = (uint32_t)state->m_lastRead;

    bool fForceInterrupt = false;

    uint32_t lowReadNew = TimeState::ReadCounter(timer);

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

            TimeState::SetCompare(timer, nextComp);

            lowReadNew = TimeState::ReadCounter(timer);

            int32_t diff = nextComp - lowReadNew;

            if (diff < 0) {
                fForceInterrupt = true;
            }
        }
    }

    if (fForceInterrupt) {
        // Force interrupt to process this.
        TimeState::ForceInterrupt(timer);
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Time_Initialize(const TinyCLR_NativeTime_Controller* self) {
    int32_t timer = LPC24_TIME_DEFAULT_CONTROLLER_ID;

    auto state = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    state->m_lastRead = 0;
    state->m_nextCompare = TIMER_IDLE_VALUE;

    if (!TimeState::Initialize(timer, (uint32_t*)&LPC24_Time_InterruptHandler, (void*)self))
        return TinyCLR_Result::InvalidOperation;

    TimeState::SetCompare(timer, LPC24_TIME_OVERFLOW_FLAG);

    TimeState::EnableCompareInterrupt(timer);

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self) {
    int32_t timer = LPC24_TIME_DEFAULT_CONTROLLER_ID;

    if (TimeState::Uninitialize(timer) == false)
        return TinyCLR_Result::InvalidOperation;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback) {
    auto state = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    if (state->m_DequeuAndExecute != nullptr)
        return TinyCLR_Result::InvalidOperation;

    state->m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

extern "C" void IDelayLoop(int32_t iterations);

void LPC24_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= (LPC24_AHB_CLOCK_HZ / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int32_t iterations = (int32_t)microseconds - 5;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

void LPC24_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime) {
    //TODO do inline later, don't call out to Delay

    auto microseconds = LPC24_Time_GetTimeForProcessorTicks(self, nativeTime) / 10;

    LPC24_Time_Delay(self, microseconds);
}

//******************** Profiler ********************

