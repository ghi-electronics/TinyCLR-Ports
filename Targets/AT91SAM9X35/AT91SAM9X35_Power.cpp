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

#include "AT91SAM9X35.h"

#define TOTAL_POWER_CONTROLLERS 1

static void(*PowerStopHandler)();
static void(*PowerRestartHandler)();

struct PowerState {
    uint32_t controllerIndex;
    bool tableInitialized;
};

const char* powerApiNames[TOTAL_POWER_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.AT91SAM9X35.PowerController\\0"
};

static TinyCLR_Power_Controller powerControllers[TOTAL_POWER_CONTROLLERS];
static TinyCLR_Api_Info powerApi[TOTAL_POWER_CONTROLLERS];
static PowerState powerStates[TOTAL_POWER_CONTROLLERS];

void AT91SAM9X35_Power_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        if (powerStates[i].tableInitialized)
            continue;

        powerControllers[i].ApiInfo = &powerApi[i];
        powerControllers[i].Initialize = &AT91SAM9X35_Power_Initialize;
        powerControllers[i].Uninitialize = &AT91SAM9X35_Power_Uninitialize;
        powerControllers[i].Reset = &AT91SAM9X35_Power_Reset;
        powerControllers[i].SetLevel = &AT91SAM9X35_Power_SetLevel;

        powerApi[i].Author = "GHI Electronics, LLC";
        powerApi[i].Name = powerApiNames[i];
        powerApi[i].Type = TinyCLR_Api_Type::PowerController;
        powerApi[i].Version = 0;
        powerApi[i].Implementation = &powerControllers[i];
        powerApi[i].State = &powerStates[i];

        powerStates[i].controllerIndex = i;
        powerStates[i].tableInitialized = true;
    }
}

const TinyCLR_Api_Info* AT91SAM9X35_Power_GetRequiredApi() {
    AT91SAM9X35_Power_EnsureTableInitialized();

    return &powerApi[0];
}

void AT91SAM9X35_Power_AddApi(const TinyCLR_Api_Manager* apiManager) {
    AT91SAM9X35_Power_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &powerApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::PowerController, powerApi[0].Name);
}

void AT91SAM9X35_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    PowerStopHandler = stop;
    PowerRestartHandler = restart;
}

TinyCLR_Result AT91SAM9X35_Power_SetLevel(const TinyCLR_Power_Controller* self, TinyCLR_Power_Level level, TinyCLR_Power_WakeSource wakeSource, uint64_t data) {
    volatile uint32_t reg = 0;

    switch (level) {
    case TinyCLR_Power_Level::Sleep1: // Sleep
    case TinyCLR_Power_Level::Sleep2: // Sleep
    case TinyCLR_Power_Level::Sleep3: // Sleep
    case TinyCLR_Power_Level::Off:    // Off
    case TinyCLR_Power_Level::Custom: // Custom
        //TODO
        return TinyCLR_Result::NotSupported;

    case TinyCLR_Power_Level::Idle:   // Idle
        // ARM926EJ-S Wait For Interrupt
#ifdef __GNUC__
        asm("MCR p15, 0, %0, c7, c0, 4" :: "r" (reg));
#else
        __asm
        {
            mcr     p15, 0, reg, c7, c0, 4
        }
#endif

        return TinyCLR_Result::Success;

    case TinyCLR_Power_Level::Active: // Active
    default:
        // Highest performance
        return TinyCLR_Result::Success;
    }
}

TinyCLR_Result AT91SAM9X35_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter) {
#if defined RAM_BOOTLOADER_HOLD_VALUE && defined RAM_BOOTLOADER_HOLD_ADDRESS && RAM_BOOTLOADER_HOLD_ADDRESS > 0
    if (!runCoreAfter) {
        //See section 1.9 of UM10211.pdf. A write-back buffer holds the last written value. Two writes guarantee it'll appear after a reset.
        *((volatile uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
        *((volatile uint32_t*)RAM_BOOTLOADER_HOLD_ADDRESS) = RAM_BOOTLOADER_HOLD_VALUE;
    }
#endif

    volatile uint32_t *pReset = (volatile uint32_t*)AT91C_BASE_RSTC;
    volatile uint32_t *pResetMode = (volatile uint32_t*)AT91C_BASE_RSTC_MR;

    *pResetMode = (AT91C_RSTC__RESET_KEY | 4ul << 8);
    *pReset = (AT91C_RSTC__RESET_KEY | AT91C_RTSC__PROCRST | AT91C_RTSC__PERRST | AT91C_RTSC__EXTRST);

    while (1); // wait for reset

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91SAM9X35_Power_Initialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91SAM9X35_Power_Uninitialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}
