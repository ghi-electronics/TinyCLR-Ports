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

#include "LPC17.h"

#define TIMER_IDLE_VALUE  0x0000FFFFFFFFFFFFFull

#define TOTAL_TIME_CONTROLLERS 1

#define SLOW_CLOCKS_PER_SECOND LPC17_AHB_CLOCK_HZ
#define SLOW_CLOCKS_TEN_MHZ_GCD           1000000   // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000   // GCD(SLOW_CLOCKS_PER_SECOND, 1k)
#define CLOCK_COMMON_FACTOR               1000000   // GCD(SYSTEM_CLOCK_HZ, 1M)
#define CORTEXM_SLEEP_USEC_FIXED_OVERHEAD_CLOCKS 3

struct TimeState {
    int32_t controllerIndex;
    uint64_t m_lastRead;
    uint32_t m_currentTick;
    uint32_t m_periodTicks;

    TinyCLR_NativeTime_Callback m_DequeuAndExecute;

    static void Reload(uint32_t value);
};

static TimeState timeStates[TOTAL_TIME_CONTROLLERS];

static TinyCLR_NativeTime_Controller timeControllers[TOTAL_TIME_CONTROLLERS];
static TinyCLR_Api_Info timeApi[TOTAL_TIME_CONTROLLERS];

const TinyCLR_Api_Info* LPC17_Time_GetApi() {
    for (int32_t i = 0; i < TOTAL_TIME_CONTROLLERS; i++) {
        timeControllers[i].ApiInfo = &timeApi[i];
        timeControllers[i].Initialize = &LPC17_Time_Initialize;
        timeControllers[i].Uninitialize = &LPC17_Time_Uninitialize;
        timeControllers[i].GetNativeTime = &LPC17_Time_GetCurrentProcessorTicks;
        timeControllers[i].ConvertNativeTimeToSystemTime = &LPC17_Time_GetTimeForProcessorTicks;
        timeControllers[i].ConvertSystemTimeToNativeTime = &LPC17_Time_GetProcessorTicksForTime;
        timeControllers[i].SetCallback = &LPC17_Time_SetTickCallback;
        timeControllers[i].ScheduleCallback = &LPC17_Time_SetNextTickCallbackTime;
        timeControllers[i].Wait = &LPC17_Time_DelayNative;

        timeApi[i].Author = "GHI Electronics, LLC";
        timeApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.NativeTimeController";
        timeApi[i].Type = TinyCLR_Api_Type::NativeTimeController;
        timeApi[i].Version = 0;
        timeApi[i].Implementation = &timeControllers[i];
        timeApi[i].State = &timeStates[i];

        timeStates[i].controllerIndex = i;
    }

    return (const TinyCLR_Api_Info*)&timeApi;
}

static uint64_t timerNextEvent;   // tick time of next event to be scheduled

uint64_t LPC17_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks) {
    ticks *= (10000000 / SLOW_CLOCKS_TEN_MHZ_GCD);
    ticks /= (SLOW_CLOCKS_PER_SECOND / SLOW_CLOCKS_TEN_MHZ_GCD);

    return ticks;
}

uint64_t LPC17_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time) {
    time /= 10;

#if 1000000 <= SLOW_CLOCKS_PER_SECOND
    return time * (SLOW_CLOCKS_PER_SECOND / 1000000);
#else
    return time / (1000000 / SLOW_CLOCKS_PER_SECOND);
#endif
}

uint64_t LPC17_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self) {
    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    uint32_t tick_spent;
    uint32_t reg = SysTick->CTRL;
    uint32_t ticks = (SysTick->VAL & SysTick_LOAD_RELOAD_Msk);

    if ((reg & SysTick_CTRL_COUNTFLAG_Msk) == SysTick_CTRL_COUNTFLAG_Msk   // Interrupt was trigger on time as expected
        || ticks >= driver->m_currentTick) {                // Interrupt was trigger slower than expected
        if (ticks > 0) {
            tick_spent = driver->m_currentTick + (SysTick->LOAD - ticks);
        }
        else {
            tick_spent = driver->m_currentTick;
        }
    }
    else {
        tick_spent = driver->m_currentTick - ticks;
    }

    driver->m_currentTick = ticks;
    driver->m_lastRead += tick_spent;

    return (uint64_t)(driver->m_lastRead);
}

TinyCLR_Result LPC17_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks) {
    uint64_t ticks;

    DISABLE_INTERRUPTS_SCOPED(irq);

    auto driver = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    ticks = LPC17_Time_GetCurrentProcessorTicks(self);

    timerNextEvent = processorTicks;

    if (timerNextEvent >= TIMER_IDLE_VALUE) {
        if (ticks >= TIMER_IDLE_VALUE) {
            timerNextEvent = timerNextEvent > ticks ? (timerNextEvent - ticks) : 0;

            driver->m_lastRead = 0;

            driver->m_currentTick = timerNextEvent;
            driver->m_periodTicks = timerNextEvent;

            SysTick_Config(driver->m_periodTicks);

            driver->Reload(driver->m_periodTicks);

        }
        else {
            driver->m_periodTicks = SysTick_LOAD_RELOAD_Msk;
            driver->Reload(SysTick_LOAD_RELOAD_Msk);
        }
    }
    else {
        if (ticks >= timerNextEvent) { // missed event
            driver->m_DequeuAndExecute();
        }
        else {
            driver->m_periodTicks = (timerNextEvent - ticks);

            if (driver->m_periodTicks >= SysTick_LOAD_RELOAD_Msk) {
                driver->Reload(SysTick_LOAD_RELOAD_Msk);
            }
            else {
                driver->Reload(driver->m_periodTicks);
            }
        }
    }

    return TinyCLR_Result::Success;
}

extern "C" {

    void SysTick_Handler(void *param) {
        INTERRUPT_STARTED_SCOPED(isr);

        auto controllerIndex = 0; // default index if no specific

        auto driver = &timeStates[controllerIndex];

        auto self = &timeControllers[controllerIndex];

        if (LPC17_Time_GetCurrentProcessorTicks(self) >= timerNextEvent) { // handle event
            driver->m_DequeuAndExecute();
        }
        else {
            LPC17_Time_SetNextTickCallbackTime(self, timerNextEvent);
        }
    }

}

TinyCLR_Result LPC17_Time_Initialize(const TinyCLR_NativeTime_Controller* self) {
    timerNextEvent = TIMER_IDLE_VALUE;

    auto driver = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    driver->m_lastRead = 0;

    driver->m_currentTick = SysTick_LOAD_RELOAD_Msk;
    driver->m_periodTicks = SysTick_LOAD_RELOAD_Msk;

    SysTick_Config(driver->m_periodTicks);

    driver->Reload(driver->m_periodTicks);
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback) {
    auto driver = reinterpret_cast<TimeState*>(self->ApiInfo->State);

    if (driver->m_DequeuAndExecute != nullptr) return TinyCLR_Result::InvalidOperation;

    driver->m_DequeuAndExecute = callback;

    return TinyCLR_Result::Success;
}

extern "C" void IDelayLoop(int32_t iterations);

void LPC17_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds) {

    // iterations must be signed so that negative iterations will result in the minimum delay

    microseconds *= (LPC17_AHB_CLOCK_HZ / CLOCK_COMMON_FACTOR);
    microseconds /= (1000000 / CLOCK_COMMON_FACTOR);

    // iterations is equal to the number of CPU instruction cycles in the required time minus
    // overhead cycles required to call this subroutine.
    int32_t iterations = (int32_t)microseconds - 5;      // Subtract off call & calculation overhead
    IDelayLoop(iterations);
}

void LPC17_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime) {
    //TODO do inline later, don't call out to Delay

    auto microseconds = LPC17_Time_GetTimeForProcessorTicks(self, nativeTime) / 10;

    LPC17_Time_Delay(self, microseconds);
}

//******************** Profiler ********************

void TimeState::Reload(uint32_t value) {
    auto driver = &timeStates[0];

    driver->m_currentTick = value;

    SysTick->LOAD = (uint32_t)(driver->m_currentTick - 1UL);
    SysTick->VAL = 0UL;
}

#ifdef __GNUC__
asm volatile (
    ".syntax unified\n\t"
    ".arch armv7-m\n\t"
    ".thumb\n\t"
    ".global  IDelayLoop\n\t"
    "@AREA ||i.IDelayLoop||, CODE, READONLY @ void IDelayLoop(UINT32 count)\n\t"
    ".section i.IDelayLoop, \"ax\", %progbits\n\t"
    ".thumb_func\n\t"
    "IDelayLoop:\n\t"
    "subs    r0, r0, #3          @@ 1 cycle\n\t"
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
