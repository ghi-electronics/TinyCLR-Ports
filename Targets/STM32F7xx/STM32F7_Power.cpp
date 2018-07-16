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

#define PWR_MAINREGULATOR_ON                        ((uint32_t)0x00000000U)
#define PWR_LOWPOWERREGULATOR_ON                    PWR_CR1_LPDS

static TinyCLR_Power_Controller powerProvider;
static TinyCLR_Api_Info powerApi;

const TinyCLR_Api_Info* STM32F7_Power_GetApi() {
    powerProvider.ApiInfo = &powerApi;
    powerProvider.Initialize = &STM32F7_Power_Initialize;
    powerProvider.Uninitialize = &STM32F7_Power_Uninitialize;
    powerProvider.Reset = &STM32F7_Power_Reset;
    powerProvider.Sleep = &STM32F7_Power_Sleep;

    powerApi.Author = "GHI Electronics, LLC";
    powerApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F7.PowerController";
    powerApi.Type = TinyCLR_Api_Type::PowerController;
    powerApi.Version = 0;
    powerApi.Implementation = &powerProvider;

    return &powerApi;
}

void STM32F7_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level) {
    uint32_t tmpreg = 0;
    switch (level) {

    case TinyCLR_Power_SleepLevel::Hibernate: // stop
        /* Select the regulator state in Stop mode ---------------------------------*/
        tmpreg = PWR->CR1;
        /* Clear PDDS and LPDS bits */
        tmpreg &= (uint32_t)~(PWR_CR1_PDDS | PWR_CR1_LPDS);

        /* Set LPDS, MRLVDS and LPLVDS bits according to PWR_LOWPOWERREGULATOR_ON value */
        tmpreg |= PWR_LOWPOWERREGULATOR_ON;

        /* Store the new value */
        PWR->CR1 = tmpreg;

        /* Set SLEEPDEEP bit of Cortex System Control Register */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* Request Wait For Interrupt */
        __WFI();

        return;

    case TinyCLR_Power_SleepLevel::Off: // standby
        /* Select Standby mode */
        PWR->CR1 |= PWR_CR1_PDDS;

        /* Set SLEEPDEEP bit of Cortex System Control Register */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* Request Wait For Interrupt */
        __WFI();

        return;

    default: // sleep
        CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

        /* Request Wait For Interrupt */
        __WFI();

        return;
    }
}

void STM32F7_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter) {
#if defined BOOTLOADER_HOLD_VALUE && defined BOOTLOADER_HOLD_ADDRESS && BOOTLOADER_HOLD_ADDRESS > 0
    if (!runCoreAfter)
        *((uint32_t*)BOOTLOADER_HOLD_ADDRESS) = BOOTLOADER_HOLD_VALUE;
#endif

    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos)  // unlock key
        | (1 << SCB_AIRCR_SYSRESETREQ_Pos); // reset request

    while (1); // wait for reset
}

TinyCLR_Result STM32F7_Power_Initialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F7_Power_Uninitialize(const TinyCLR_Power_Controller* self) {
    return TinyCLR_Result::Success;
}
