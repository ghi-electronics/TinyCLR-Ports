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

#define PCON (*(volatile unsigned char *)0xE01FC0C0)

static void(*g_LPC24_stopHandler)();
static void(*g_LPC24_restartHandler)();

static TinyCLR_Power_Provider powerProvider;
static TinyCLR_Api_Info powerApi;

const TinyCLR_Api_Info* LPC24_Power_GetApi() {
    powerProvider.ApiInfo = &powerApi;
    powerProvider.Initialize = &LPC24_Power_Initialize;
    powerProvider.Uninitialize = &LPC24_Power_Uninitialize;
    powerProvider.Reset = &LPC24_Power_Reset;
    powerProvider.Sleep = &LPC24_Power_Sleep;

    powerApi.Author = "GHI Electronics, LLC";
    powerApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC24.PowerProvider";
    powerApi.Type = TinyCLR_Api_Type::PowerProvider;
    powerApi.Version = 0;
    powerApi.Implementation = &powerProvider;

    return &powerApi;
}

void LPC24_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    g_LPC24_stopHandler = stop;
    g_LPC24_restartHandler = restart;
}

void LPC24_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_SleepLevel level) {
    switch (level) {

    case TinyCLR_Power_SleepLevel::Hibernate: // stop
        if (g_LPC24_stopHandler != 0)
            g_LPC24_stopHandler();

        return;

    case TinyCLR_Power_SleepLevel::Off: // standby
        // stop peripherals if needed
        if (g_LPC24_stopHandler != 0)
            g_LPC24_stopHandler();

        return;

    default: // sleep
        PCON |= 1;

        return;
    }
}

void LPC24_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter) {
#if defined RAM_BOOTLOADER_HOLD_VALUE && defined RAM_BOOTLOADER_HOLD_ADDRESS && RAM_BOOTLOADER_HOLD_ADDRESS > 0
    if (!runCoreAfter) {
        //See section 1.9 of UM10211.pdf. A write-back buffer holds the last written value. Two writes guarantee it'll appear after a reset.
        *((volatile uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
        *((volatile uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
    }
#endif

    LPC24XX_WATCHDOG& WTDG = LPC24XX::WTDG();

    // disable interrupts
    DISABLE_INTERRUPTS_SCOPED(irq);
    // set the smallest value
    WTDG.WDTC = 0xFF;

    // assure its enabled (and counter is zero)
    WTDG.WDMOD = LPC24XX_WATCHDOG::WDMOD__WDEN | LPC24XX_WATCHDOG::WDMOD__WDRESET;

    WTDG.WDFEED = LPC24XX_WATCHDOG::WDFEED_reload_1;
    WTDG.WDFEED = LPC24XX_WATCHDOG::WDFEED_reload_2;

    while (1); // wait for reset
}

TinyCLR_Result LPC24_Power_Initialize(const TinyCLR_Power_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Power_Uninitialize(const TinyCLR_Power_Provider* self) {
    return TinyCLR_Result::Success;
}
