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

static void(*PowerStopHandler)();
static void(*PowerRestartHandler)();

#define TOTAL_POWER_CONTROLLERS 1

static TinyCLR_Power_Controller powerControllers[TOTAL_POWER_CONTROLLERS];
static TinyCLR_Api_Info powerApi[TOTAL_POWER_CONTROLLERS];

const TinyCLR_Api_Info* LPC17_Power_GetApi() {
    for (int32_t i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        powerControllers[i].ApiInfo = &powerApi[i];
        powerControllers[i].Initialize = &LPC17_Power_Initialize;
        powerControllers[i].Uninitialize = &LPC17_Power_Uninitialize;
        powerControllers[i].Reset = &LPC17_Power_Reset;
        powerControllers[i].Sleep = &LPC17_Power_Sleep;

        powerApi[i].Author = "GHI Electronics, LLC";
        powerApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.PowerController";
        powerApi[i].Type = TinyCLR_Api_Type::PowerController;
        powerApi[i].Version = 0;
        powerApi[i].Implementation = &powerControllers[i];
        powerApi[i].State = nullptr;
    }

    return (const TinyCLR_Api_Info*)&powerApi;
}

void LPC17_Power_SetHandlers(void(*stop)(), void(*restart)()) {
    PowerStopHandler = stop;
    PowerRestartHandler = restart;
}

void LPC17_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level) {
    switch (level) {

    case TinyCLR_Power_SleepLevel::Hibernate: // stop
        if (PowerStopHandler != 0)
            PowerStopHandler();

        return;

    case TinyCLR_Power_SleepLevel::Off: // standby
        // stop peripherals if needed
        if (PowerStopHandler != 0)
            PowerStopHandler();

        __WFI(); // soft power off, never returns
        return;

    default: // sleep
        LPC_SC->PCON &= ~(LPC_SC_PCON_PM0_Msk | LPC_SC_PCON_PM1_Msk); // clear PM0 and PM1 to 0 => sleep
        __WFI(); // sleep and wait for interrupt

        return;
    }
}

void LPC17_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter) {
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

TinyCLR_Result LPC17_Power_Initialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Power_Uninitialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}
