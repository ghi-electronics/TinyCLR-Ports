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

static void(*g_AT91_stopHandler)();
static void(*g_AT91_restartHandler)();

static TinyCLR_Power_Provider powerProvider;
static TinyCLR_Api_Info powerApi;

const TinyCLR_Api_Info* AT91_Power_GetApi() {
    powerProvider.ApiInfo = &powerApi;
    powerProvider.Initialize = &AT91_Power_Initialize;
    powerProvider.Uninitialize = &AT91_Power_Uninitialize;
    powerProvider.Reset = &AT91_Power_Reset;
    powerProvider.Sleep = &AT91_Power_Sleep;

    powerApi.Author = "GHI Electronics, LLC";
    powerApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.PowerController";
    powerApi.Type = TinyCLR_Api_Type::PowerController;
    powerApi.Version = 0;
    powerApi.Implementation = &powerProvider;

    return &powerApi;
}

void AT91_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    g_AT91_stopHandler = stop;
    g_AT91_restartHandler = restart;
}

void AT91_Power_Sleep(const TinyCLR_Power_Provider* self, TinyCLR_Power_SleepLevel level) {
    switch (level) {

    case TinyCLR_Power_SleepLevel::Hibernate: // stop
        if (g_AT91_stopHandler != 0)
            g_AT91_stopHandler();

        return;

    case TinyCLR_Power_SleepLevel::Off: // standby
        // stop peripherals if needed
        if (g_AT91_stopHandler != 0)
            g_AT91_stopHandler();

        return;

    default: // sleep

        uint32_t reg = 0;

        // ARM926EJ-S Wait For Interrupt
#ifdef __GNUC__
        asm("MCR p15, 0, %0, c7, c0, 4" :: "r" (reg));
#else
        __asm
        {
            mcr     p15, 0, reg, c7, c0, 4
        }
#endif

        return;
    }
}

void AT91_Power_Reset(const TinyCLR_Power_Provider* self, bool runCoreAfter) {
#if defined RAM_BOOTLOADER_HOLD_VALUE && defined RAM_BOOTLOADER_HOLD_ADDRESS && RAM_BOOTLOADER_HOLD_ADDRESS > 0
    if (!runCoreAfter) {
        //See section 1.9 of UM10211.pdf. A write-back buffer holds the last written value. Two writes guarantee it'll appear after a reset.
        *((volatile uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
        *((volatile uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
    }
#endif

    volatile uint32_t *pReset = (volatile uint32_t*)AT91C_BASE_RSTC;
    volatile uint32_t *pResetMode = (volatile uint32_t*)(AT91C_BASE_RSTC + AT91C_RTSC__EXTRST);

    *pResetMode = (AT91C_RSTC__RESET_KEY | 4ul << 8);
    *pReset = (AT91C_RSTC__RESET_KEY | AT91C_RTSC__PROCRST | AT91C_RTSC__PERRST | AT91C_RTSC__EXTRST);

    while (1); // wait for reset
}

TinyCLR_Result AT91_Power_Initialize(const TinyCLR_Power_Provider* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Power_Uninitialize(const TinyCLR_Power_Provider* self) {
    return TinyCLR_Result::Success;
}
