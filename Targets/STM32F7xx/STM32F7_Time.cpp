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

#include "STM32F7.h"

#define TIMER_IDLE_VALUE  0x0000FFFFFFFFFFFFFull

#define SLOW_CLOCKS_PER_SECOND STM32F7_AHB_CLOCK_HZ
#define SLOW_CLOCKS_TEN_MHZ_GCD           1000000   // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000   // GCD(SLOW_CLOCKS_PER_SECOND, 1k)
#define CLOCK_COMMON_FACTOR               1000000   // GCD(STM32F7_SYSTEM_CLOCK_HZ, 1M)
#define CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 3

struct STM32F7_Timer_Driver {

    uint64_t m_lastRead;
    uint32_t m_currentTick;
    uint32_t m_periodTicks;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;

    static void Reload(uint32_t value);

};

static TinyCLR_NativeTime_Provider timeProvider;
static TinyCLR_Api_Info timeApi;

const TinyCLR_Api_Info* STM32F7_Time_GetApi() {
    timeProvider.Parent = &timeApi;
    timeProvider.Acquire = &STM32F7_Time_Acquire;
    timeProvider.Release = &STM32F7_Time_Release;
    timeProvider.GetNativeTime = &STM32F7_Time_GetCurrentProcessorTicks;
    timeProvider.ConvertNativeTimeToSystemTime = &STM32F7_Time_GetTimeForProcessorTicks;
    timeProvider.ConvertSystemTimeToNativeTime = &STM32F7_Time_GetProcessorTicksForTime;
    timeProvider.SetCallback = &STM32F7_Time_SetTickCallback;
    timeProvider.ScheduleCallback = &STM32F7_Time_SetNextTickCallbackTime;
    timeProvider.WaitMicroseconds = &STM32F7_Time_Delay;

    timeApi.Author = "GHI Electronics, LLC";
    timeApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.NativeTimeProvider";
    timeApi.Type = TinyCLR_Api_Type::NativeTimeProvider;
    timeApi.Version = 0;
    timeApi.Count = 1;
    timeApi.Implementation = &timeProvider;

    return &timeApi;
}

static uint64_t g_nextEvent;   // tick time of next event to be scheduled

STM32F7_Timer_Driver g_STM32F7_Timer_Driver;

uint64_t STM32F7_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Provider* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t STM32F7_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Provider* self, uint64_t time) {
    time /= 10;

#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return time * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return time / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif
}

uint64_t STM32F7_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Provider* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    uint32_t tick_spent;
    uint32_t reg = SysTick->CTRL;
    uint32_t ticks = (SysTick->VAL & SysTick_LOAD_RELOAD_Msk);

    if ((reg & SysTick_CTRL_COUNTFLAG_Msk) == SysTick_CTRL_COUNTFLAG_Msk   // Interrupt was trigger on time as expected
        || ticks >= g_STM32F7_Timer_Driver.m_currentTick) {                // Interrupt was trigger slower than expected
        if (ticks > 0) {
            tick_spent = g_STM32F7_Timer_Driver.m_currentTick + (SysTick->LOAD - ticks);
        }
        else {
            tick_spent = g_STM32F7_Timer_Driver.m_currentTick;
        }
    }
    else {
        tick_spent = g_STM32F7_Timer_Driver.m_currentTick - ticks;
    }

    g_STM32F7_Timer_Driver.m_currentTick = ticks;
    g_STM32F7_Timer_Driver.m_lastRead += tick_spent;

    return (uint64_t)(g_STM32F7_Timer_Driver.m_lastRead & TIMER_IDLE_VALUE);
}

TinyCLR_Result STM32F7_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Provider* self, uint64_t processorTicks) {
    uint64_t ticks;

    DISABLE_INTERRUPTS_SCOPED(irq);

    ticks = STM32F7_Time_GetCurrentProcessorTicks(self);

    g_nextEvent = processorTicks;

    if (g_nextEvent >= TIMER_IDLE_VALUE) {
        if (ticks >= TIMER_IDLE_VALUE) {
            g_nextEvent = g_nextEvent > ticks ? (g_nextEvent - ticks) : 0;

            g_STM32F7_Timer_Driver.m_lastRead = 0;

            g_STM32F7_Timer_Driver.m_currentTick = g_nextEvent;
            g_STM32F7_Timer_Driver.m_periodTicks = g_nextEvent;

            SysTick_Config(g_STM32F7_Timer_Driver.m_periodTicks);

            g_STM32F7_Timer_Driver.Reload(g_STM32F7_Timer_Driver.m_periodTicks);

        }
        else {
            g_STM32F7_Timer_Driver.m_periodTicks = SysTick_LOAD_RELOAD_Msk;
            g_STM32F7_Timer_Driver.Reload(SysTick_LOAD_RELOAD_Msk);
        }
    }
    else {
        if (ticks >= g_nextEvent) { // missed event
            g_STM32F7_Timer_Driver.m_DequeuAndExecute();
        }
        else {
            g_STM32F7_Timer_Driver.m_periodTicks = (g_nextEvent - ticks);

            if (g_STM32F7_Timer_Driver.m_periodTicks >= SysTick_LOAD_RELOAD_Msk) {
                g_STM32F7_Timer_Driver.Reload(SysTick_LOAD_RELOAD_Msk);
            }
            else {
                g_STM32F7_Timer_Driver.Reload(g_STM32F7_Timer_Driver.m_periodTicks);
            }
        }
    }

    return TinyCLR_Result::Success;
}

extern "C" {

    void SysTick_Handler(void *param) {
        INTERRUPT_STARTED_SCOPED(isr);

        if (STM32F7_Time_GetCurrentProcessorTicks(nullptr) >= g_nextEvent) { // handle event
            g_STM32F7_Timer_Driver.m_DequeuAndExecute();
        }
        else {
            STM32F7_Time_SetNextTickCallbackTime(nullptr, g_nextEvent);
        }
    }

}

TinyCLR_Result STM32F7_Time_Acquire(const TinyCLR_NativeTime_Provider* self) {
    g_nextEvent = TIMER_IDLE_VALUE;

    g_STM32F7_Timer_Driver.m_lastRead = 0;

    g_STM32F7_Timer_Driver.m_currentTick = SysTick_LOAD_RELOAD_Msk;
    g_STM32F7_Timer_Driver.m_periodTicks = SysTick_LOAD_RELOAD_Msk;

    SysTick_Config(g_STM32F7_Timer_Driver.m_periodTicks);

    g_STM32F7_Timer_Driver.Reload(g_STM32F7_Timer_Driver.m_periodTicks);
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Time_Release(const TinyCLR_NativeTime_Provider* self) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Time_SetTickCallback(const TinyCLR_NativeTime_Provider* self, TinyCLR_NativeTime_Callback callback) {
    if (g_STM32F7_Timer_Driver.m_DequeuAndExecute != nullptr) return TinyCLR_Result::InvalidOperation;

    g_STM32F7_Timer_Driver.m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

extern "C" void IDelayLoop(int32_t iterations);

void STM32F7_Time_Delay(const TinyCLR_NativeTime_Provider* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= (STM32F7_AHB_CLOCK_HZ / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int32_t iterations = (int32_t)microseconds - 5;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

//******************** Profiler ********************

void STM32F7_Timer_Driver::Reload(uint32_t value) {
    g_STM32F7_Timer_Driver.m_currentTick = value;

    SysTick->LOAD = (uint32_t)(g_STM32F7_Timer_Driver.m_currentTick - 1UL);
    SysTick->VAL = 0UL;
}

#ifdef __GNUC__
asm volatile (
    ".syntax unified\n\t"
    ".cpu cortex-m7\n\t"
    ".thumb\n\t"
    ".global  IDelayLoop\n\t"
    "@AREA ||i.IDelayLoop||, CODE, READONLY @ void IDelayLoop(UINT32 count)\n\t"
    ".section i.IDelayLoop, \"ax\", %progbits\n\t"
    ".thumb_func\n\t"
    "IDelayLoop:\n\t"
    "subs    r0, r0, #2          @@ 1 cycle\n\t"
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
