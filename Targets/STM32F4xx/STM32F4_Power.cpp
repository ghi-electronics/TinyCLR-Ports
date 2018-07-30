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

#define TOTAL_POWER_CONTROLLERS 1

static TinyCLR_Power_Controller powerControllers[TOTAL_POWER_CONTROLLERS];
static TinyCLR_Api_Info powerApi[TOTAL_POWER_CONTROLLERS];

void STM32F4_Power_AddApi(const TinyCLR_Api_Manager* apiManager) {
    for (int32_t i = 0; i < TOTAL_POWER_CONTROLLERS; i++) {
        powerControllers[i].ApiInfo = &powerApi[i];
        powerControllers[i].Initialize = &STM32F4_Power_Initialize;
        powerControllers[i].Uninitialize = &STM32F4_Power_Uninitialize;
        powerControllers[i].Reset = &STM32F4_Power_Reset;
        powerControllers[i].Sleep = &STM32F4_Power_Sleep;

        powerApi[i].Author = "GHI Electronics, LLC";
        powerApi[i].Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.PowerController";
        powerApi[i].Type = TinyCLR_Api_Type::PowerController;
        powerApi[i].Version = 0;
        powerApi[i].Implementation = &powerControllers[i];
        powerApi[i].State = nullptr;
    }

    
}

void STM32F4_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level) {
    switch (level) {
    case TinyCLR_Power_SleepLevel::Hibernate:
        //TODO
        return;

    case TinyCLR_Power_SleepLevel::Off:
        // TODO
        return;

    default:
        PWR->CR |= PWR_CR_CWUF;

        __WFI(); // sleep and wait for interrupt
        return;
    }
}

void STM32F4_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter) {
#if defined BOOTLOADER_HOLD_VALUE && defined BOOTLOADER_HOLD_ADDRESS && BOOTLOADER_HOLD_ADDRESS > 0
    if (!runCoreAfter)
        *((uint32_t*)BOOTLOADER_HOLD_ADDRESS) = BOOTLOADER_HOLD_VALUE;
#endif

    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos)  // unlock key
        | (1 << SCB_AIRCR_SYSRESETREQ_Pos); // reset request

    while (1); // wait for reset
}

TinyCLR_Result STM32F4_Power_Initialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Power_Uninitialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}
