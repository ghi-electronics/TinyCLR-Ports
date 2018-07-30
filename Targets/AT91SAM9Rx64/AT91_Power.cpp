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

#define TOTAL_POWER_CONTROLLERS 1

static void(*PowerStopHandler)();
static void(*PowerRestartHandler)();

static TinyCLR_Power_Controller powerControllers[TOTAL_POWER_CONTROLLERS];
static TinyCLR_Api_Info powerApi[TOTAL_POWER_CONTROLLERS];

void AT91_Power_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        powerControllers[i].ApiInfo = &powerApi[i];
        powerControllers[i].Initialize = &AT91_Power_Initialize;
        powerControllers[i].Uninitialize = &AT91_Power_Uninitialize;
        powerControllers[i].Reset = &AT91_Power_Reset;
        powerControllers[i].Sleep = &AT91_Power_Sleep;

        powerApi[i].Author = "GHI Electronics, LLC";
        powerApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.AT91.PowerController";
        powerApi[i].Type = TinyCLR_Api_Type::PowerController;
        powerApi[i].Version = 0;
        powerApi[i].Implementation = &powerControllers[i];
        powerApi[i].State = nullptr;
    }

    
}

void AT91_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    PowerStopHandler = stop;
    PowerRestartHandler = restart;
}

void AT91_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level) {
    switch (level) {

    case TinyCLR_Power_SleepLevel::Hibernate: // stop
        if (PowerStopHandler != 0)
            PowerStopHandler();

        return;

    case TinyCLR_Power_SleepLevel::Off: // standby
        // stop peripherals if needed
        if (PowerStopHandler != 0)
            PowerStopHandler();

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

void AT91_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter) {
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

TinyCLR_Result AT91_Power_Initialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Power_Uninitialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}
