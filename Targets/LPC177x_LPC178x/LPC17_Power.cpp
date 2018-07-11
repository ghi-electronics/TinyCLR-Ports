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

#define LPC_SC_PCON_PM0_Pos            0                                             /*!< Power mode control bit 0. */
#define LPC_SC_PCON_PM0_Msk            (1UL << LPC_SC_PCON_PM0_Pos)                  /*!< LPC_SC_PCON_PM0_Msk. */

#define LPC_SC_PCON_PM1_Pos            1                                             /*!< Power mode control bit 0. */
#define LPC_SC_PCON_PM1_Msk            (1UL << LPC_SC_PCON_PM0_Pos)                  /*!< LPC_SC_PCON_PM0_Msk. */

static void(*g_LPC17_stopHandler)();
static void(*g_LPC17_restartHandler)();

static TinyCLR_Power_Provider powerProvider;
static TinyCLR_Api_Info powerApi;

const TinyCLR_Api_Info* LPC17_Power_GetApi() {
    powerProvider.ApiInfo = &powerApi;
    powerProvider.Acquire = &LPC17_Power_Acquire;
    powerProvider.Release = &LPC17_Power_Release;
    powerProvider.Reset = &LPC17_Power_Reset;
    powerProvider.Sleep = &LPC17_Power_Sleep;

    powerApi.Author = "GHI Electronics, LLC";
    powerApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.PowerProvider";
    powerApi.Type = TinyCLR_Api_Type::PowerProvider;
    powerApi.Version = 0;
    powerApi.Implementation = &powerProvider;

    return &powerApi;
}

void LPC17_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    g_LPC17_stopHandler = stop;
    g_LPC17_restartHandler = restart;
}

void LPC17_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_SleepLevel level) {
    switch (level) {

    case TinyCLR_Power_SleepLevel::Hibernate: // stop
        if (g_LPC17_stopHandler != 0)
            g_LPC17_stopHandler();

        return;

    case TinyCLR_Power_SleepLevel::Off: // standby
        // stop peripherals if needed
        if (g_LPC17_stopHandler != 0)
            g_LPC17_stopHandler();

        __WFI(); // soft power off, never returns
        return;

    default: // sleep
        LPC_SC->PCON &= ~(LPC_SC_PCON_PM0_Msk | LPC_SC_PCON_PM1_Msk); // clear PM0 and PM1 to 0 => sleep
        __WFI(); // sleep and wait for interrupt

        return;
    }
}

void LPC17_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter) {
#if defined RAM_BOOTLOADER_HOLD_VALUE && defined RAM_BOOTLOADER_HOLD_ADDRESS && RAM_BOOTLOADER_HOLD_ADDRESS > 0
    if (!runCoreAfter)
        *((uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
#endif
    volatile int delay;

    for (delay = 0; delay < 0xFF; delay++);

    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos)  // unlock key
        | (1 << SCB_AIRCR_SYSRESETREQ_Pos); // reset request

    while (1); // wait for reset
}

TinyCLR_Result LPC17_Power_Acquire(const TinyCLR_Power_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Power_Release(const TinyCLR_Power_Provider* self) {
    return TinyCLR_Result::Success;
}
