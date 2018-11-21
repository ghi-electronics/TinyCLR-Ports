// Copyright Microsoft Corporation
// Copyright Oberon microsystems, Inc
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

#define TIMER_IDLE_VALUE  0x0000FFFFFFFFFFFFFull

#define TOTAL_TIME_CONTROLLERS 1
#define SLOW_CLOCKS_PER_SECOND STM32F4_AHB_CLOCK_HZ
#define SLOW_CLOCKS_TEN_MHZ_GCD           1000000   // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000   // GCD(SLOW_CLOCKS_PER_SECOND, 1k)
#define CLOCK_COMMON_FACTOR               1000000   // GCD(STM32F4_SYSTEM_CLOCK_HZ, 1M)
#define CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 3

struct TimeState {
    int32_t controllerIndex;
    uint64_t m_lastRead;
    uint32_t m_currentTick;
    uint32_t m_periodTicks;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;
    const TinyCLR_SystemTime_Manager* systemTime;

    static void Reload(uint32_t value);

    bool tableInitialized;
};

static TimeState timeStates[TOTAL_TIME_CONTROLLERS];

static TinyCLR_NativeTime_Controller timeControllers[TOTAL_TIME_CONTROLLERS];
static TinyCLR_Api_Info timeApi[TOTAL_TIME_CONTROLLERS];

const char* timeApiNames[TOTAL_TIME_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.STM32F4.NativeTimeController\\0",

};

void STM32F4_Time_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        if (timeStates[i].tableInitialized)
            continue;

        timeControllers[i].ApiInfo = &timeApi[i];
        timeControllers[i].Initialize = &STM32F4_Time_Initialize;
        timeControllers[i].Uninitialize = &STM32F4_Time_Uninitialize;
        timeControllers[i].GetNativeTime = &STM32F4_Time_GetCurrentProcessorTicks;
        timeControllers[i].ConvertNativeTimeToSystemTime = &STM32F4_Time_GetTimeForProcessorTicks;
        timeControllers[i].ConvertSystemTimeToNativeTime = &STM32F4_Time_GetProcessorTicksForTime;
        timeControllers[i].SetCallback = &STM32F4_Time_SetTickCallback;
        timeControllers[i].ScheduleCallback = &STM32F4_Time_SetNextTickCallbackTime;
        timeControllers[i].Wait = &STM32F4_Time_DelayNative;

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

const TinyCLR_Api_Info* STM32F4_Time_GetRequiredApi() {
    STM32F4_Time_EnsureTableInitialized();

    return &timeApi[0];
}

void STM32F4_Time_AddApi(const TinyCLR_Api_Manager* apiManager) {
    STM32F4_Time_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &timeApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::NativeTimeController, timeApi[0].Name);
}

static uint64_t timerNextEvent;   // tick time of next event to be scheduled

uint64_t STM32F4_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t STM32F4_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time) {
    time /= 10;

#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return time * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return time / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif
}

uint64_t STM32F4_Time_GetCurrentProcessorTime() {
    return STM32F4_Time_GetTimeForProcessorTicks(nullptr, STM32F4_Time_GetCurrentProcessorTicks(nullptr));
}

uint64_t STM32F4_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    uint32_t tick_spent;
    uint32_t reg = SysTick->CTRL;
    uint32_t ticks = (SysTick->VAL & SysTick_LOAD_RELOAD_Msk);

    if ((reg & SysTick_CTRL_COUNTFLAG_Msk) == SysTick_CTRL_COUNTFLAG_Msk   // Interrupt was trigger on time as expected
        || ticks >= state->m_currentTick) {                // Interrupt was trigger slower than expected
        if (ticks > 0) {
            tick_spent = state->m_currentTick + (SysTick->LOAD - ticks);
        }
        else {
            tick_spent = state->m_currentTick;
        }
    }
    else {
        tick_spent = state->m_currentTick - ticks;
    }

    state->m_currentTick = ticks;
    state->m_lastRead += tick_spent;

    return (uint64_t)(state->m_lastRead);
}

TinyCLR_Result STM32F4_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks) {
    uint64_t ticks;

    DISABLE_INTERRUPTS_SCOPED(irq);

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    ticks = STM32F4_Time_GetCurrentProcessorTicks(self);

    timerNextEvent = processorTicks;

    if (timerNextEvent >= TIMER_IDLE_VALUE) {
        if (ticks >= TIMER_IDLE_VALUE) {
            timerNextEvent = timerNextEvent > ticks ? (timerNextEvent - ticks) : 0;

            state->m_lastRead = 0;

            state->m_currentTick = timerNextEvent;
            state->m_periodTicks = timerNextEvent;

            SysTick_Config(state->m_periodTicks);

            state->Reload(state->m_periodTicks);

        }
        else {
            state->m_periodTicks = SysTick_LOAD_RELOAD_Msk;
            state->Reload(SysTick_LOAD_RELOAD_Msk);
        }
    }
    else {
        if (ticks >= timerNextEvent) { // missed event
            state->m_DequeuAndExecute();
        }
        else {
            state->m_periodTicks = (timerNextEvent - ticks);

            if (state->m_periodTicks >= SysTick_LOAD_RELOAD_Msk) {
                state->Reload(SysTick_LOAD_RELOAD_Msk);
            }
            else {
                state->Reload(state->m_periodTicks);
            }
        }
    }

    return TinyCLR_Result::Success;
}

extern "C" {

    void SysTick_Handler(void *param) {
        INTERRUPT_STARTED_SCOPED(isr);

        auto controllerIndex = 0; // default index if no specific
        auto state = &timeStates[controllerIndex];
        auto self = &timeControllers[controllerIndex];

        if (STM32F4_Time_GetCurrentProcessorTicks(self) >= timerNextEvent) { // handle event
            state->m_DequeuAndExecute();
        }
        else {
            STM32F4_Time_SetNextTickCallbackTime(self, timerNextEvent);
        }
    }

}

TinyCLR_Result STM32F4_Time_Initialize(const TinyCLR_NativeTime_Controller* self) {
    timerNextEvent = TIMER_IDLE_VALUE;

    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    state->m_lastRead = 0;

    state->m_currentTick = SysTick_LOAD_RELOAD_Msk;
    state->m_periodTicks = SysTick_LOAD_RELOAD_Msk;

    SysTick_Config(state->m_periodTicks);

    state->Reload(state->m_periodTicks);

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback) {
    TimeState* state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    if (state->m_DequeuAndExecute != nullptr) return TinyCLR_Result::InvalidOperation;

    state->m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

uint64_t STM32F4_Time_GetSystemTime(const TinyCLR_NativeTime_Controller* self) {
    uint64_t utc;
    int32_t tz;

    auto state = ((self == nullptr) ? &timeStates[0] : reinterpret_cast<TimeState*>(self->ApiInfo->State));

    if (state->systemTime == nullptr)
        state->systemTime = reinterpret_cast<const TinyCLR_SystemTime_Manager*>(apiManager->FindDefault(apiManager, TinyCLR_Api_Type::SystemTimeManager));

    state->systemTime->GetTime(state->systemTime, utc, tz);

    return utc;
}

extern "C" void IDelayLoop(int32_t iterations);

void STM32F4_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= (STM32F4_AHB_CLOCK_HZ / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int32_t iterations = (int32_t)microseconds - 5;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

void STM32F4_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime) {
    //TODO do inline later, don't call out to Delay

    auto microseconds = STM32F4_Time_GetTimeForProcessorTicks(self, nativeTime) / 10;

    STM32F4_Time_Delay(self, microseconds);
}

//******************** Profiler ********************

void TimeState::Reload(uint32_t value) {
    auto state = &timeStates[0];

    state->m_currentTick = value;

    SysTick->LOAD = (uint32_t)(state->m_currentTick - 1UL);
    SysTick->VAL = 0UL;
}

#ifdef __GNUC__
asm volatile (
    ".syntax unified\n\t"
    ".cpu cortex-m4\n\t"
    ".thumb\n\t"
    ".global  IDelayLoop\n\t"
    "@AREA ||i.IDelayLoop||, CODE, READONLY @ void IDelayLoop(UINT32 count)\n\t"
    ".section SectionForCodeIDelayLoop, \"ax\", %progbits\n\t"
    ".thumb_func\n\t"
    "IDelayLoop:\n\t"
#if defined(STM32F413xx)
    "subs    r0, r0, #6          @@ 1 cycle\n\t"
#else
    "subs    r0, r0, #3          @@ 1 cycle\n\t"
#endif
    "bgt     IDelayLoop          @@ 3 cycles taken, 1 cycle not taken.\n\t"
    "bx lr                       @@ 3 cycles\n\t"
    );
#else
asm(
    "EXPORT  IDelayLoop\n\t"
    "AREA ||i.IDelayLoop||, CODE, READONLY ; void IDelayLoop(UINT32 count)\n\t"
    "IDelayLoop\n\t"
    "subs    r0, r0, #3          ;; 1 cycle\n\t"
    "bgt     IDelayLoop          ;; 3 cycles taken, 1 cycle not taken.\n\t"
    "bx      lr                  ;; 3 cycles\n\t"
);
#endif
