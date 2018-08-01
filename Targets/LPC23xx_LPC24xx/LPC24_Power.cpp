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

static void(*PowerStopHandler)();
static void(*PowerRestartHandler)();

#define TOTAL_POWER_CONTROLLERS 1

struct PowerState {
    uint32_t controllerIndex;
    bool tableInitialized;
};

const char* powerApiNames[TOTAL_POWER_CONTROLLERS] = {
    "GHIElectronics.TinyCLR.NativeApis.LPC24.PowerController\\0"
};

static TinyCLR_Power_Controller powerControllers[TOTAL_POWER_CONTROLLERS];
static TinyCLR_Api_Info powerApi[TOTAL_POWER_CONTROLLERS];
static PowerState powerStates[TOTAL_POWER_CONTROLLERS];

void LPC24_Power_EnsureTableInitialized() {
    for (auto i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        if (powerStates[i].tableInitialized)
            continue;

        powerControllers[i].ApiInfo = &powerApi[i];
        powerControllers[i].Initialize = &LPC24_Power_Initialize;
        powerControllers[i].Uninitialize = &LPC24_Power_Uninitialize;
        powerControllers[i].Reset = &LPC24_Power_Reset;
        powerControllers[i].Sleep = &LPC24_Power_Sleep;

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

const TinyCLR_Api_Info* LPC24_Power_GetRequiredApi() {
    LPC24_Power_EnsureTableInitialized();

    return &powerApi[0];
}

void LPC24_Power_AddApi(const TinyCLR_Api_Manager* apiManager) {
    LPC24_Power_EnsureTableInitialized();

    for (auto i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        apiManager->Add(apiManager, &powerApi[i]);
    }

    apiManager->SetDefaultName(apiManager, TinyCLR_Api_Type::PowerController, powerApi[0].Name);
}

void LPC24_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    PowerStopHandler = stop;
    PowerRestartHandler = restart;
}

void LPC24_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level) {
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
        PCON |= 1;

        return;
    }
}

void LPC24_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter) {
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

TinyCLR_Result LPC24_Power_Initialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC24_Power_Uninitialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}
