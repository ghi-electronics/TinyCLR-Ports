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

#define CKGR_MOR_MOSCXTEN (0x1u << 0) /**< \brief (CKGR_MOR) Main Crystal Oscillator Enable */
#define CKGR_MOR_MOSCXTBY (0x1u << 1) /**< \brief (CKGR_MOR) Main Crystal Oscillator Bypass */
#define CKGR_MOR_MOSCRCEN (0x1u << 3) /**< \brief (CKGR_MOR) Main On-Chip RC Oscillator Enable */
#define CKGR_MOR_MOSCXTST_Pos 8
#define CKGR_MOR_MOSCXTST_Msk (0xffu << CKGR_MOR_MOSCXTST_Pos) /**< \brief (CKGR_MOR) Main Crystal Oscillator Start-up Time */
#define CKGR_MOR_MOSCXTST(value) ((CKGR_MOR_MOSCXTST_Msk & ((value) << CKGR_MOR_MOSCXTST_Pos)))
#define CKGR_MOR_KEY_Pos 16
#define CKGR_MOR_KEY_Msk (0xffu << CKGR_MOR_KEY_Pos) /**< \brief (CKGR_MOR) Password */
#define CKGR_MOR_KEY(value) ((CKGR_MOR_KEY_Msk & ((value) << CKGR_MOR_KEY_Pos)))
#define CKGR_MOR_MOSCSEL (0x1u << 24) /**< \brief (CKGR_MOR) Main Oscillator Selection */
#define CKGR_PLLAR_MULA_Pos 16
#define CKGR_PLLAR_MULA_Msk (0x7ffu << CKGR_PLLAR_MULA_Pos) /**< \brief (CKGR_PLLAR) PLLA Multiplier */

#define PMC_MCKR_CSS_Pos 0
#define PMC_MCKR_CSS_Msk (0x3u << PMC_MCKR_CSS_Pos) /**< \brief (PMC_MCKR) Master/Processor Clock Source Selection */
#define PMC_MCKR_CSS_SLOW_CLK (0x0u << 0) /**< \brief (PMC_MCKR) Slow Clock is selected */
#define PMC_MCKR_PRES_Pos 4
#define PMC_MCKR_PRES_Msk (0x7u << PMC_MCKR_PRES_Pos) /**< \brief (PMC_MCKR) Master/Processor Clock Prescaler */
#define PMC_MCKR_PRES_CLOCK (0x0u << 4) /**< \brief (PMC_MCKR) Selected clock */
#define PMC_MCKR_PRES_CLOCK_DIV64 (0x6u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 64 */
#define PMC_MCKR_MDIV_Pos 8
#define PMC_MCKR_MDIV_Msk (0x3u << PMC_MCKR_MDIV_Pos) /**< \brief (PMC_MCKR) Master Clock Division */
#define PMC_MCKR_PLLADIV2 (0x1u << 12) /**< \brief (PMC_MCKR) PLLA divisor by 2 */

#define PMC_SR_MOSCXTS (0x1u << 0) /**< \brief (PMC_SR) Main XTAL Oscillator Status */
#define PMC_SR_LOCKA (0x1u << 1) /**< \brief (PMC_SR) PLLA Lock Status */
#define PMC_SR_MCKRDY (0x1u << 3) /**< \brief (PMC_SR) Master Clock Status */
#define PMC_SR_MOSCSELS (0x1u << 16) /**< \brief (PMC_SR) Main Oscillator Selection Status */

#define SCK_CR_REG      (*(volatile uint32_t*)0xFFFFFE50)
#define SHDW_CR_REG     (*(volatile uint32_t*)0xFFFFFE10)
#define SHDW_MR_REG     (*(volatile uint32_t*)0xFFFFFE14)
#define SHDW_SR_REG     (*(volatile uint32_t*)0xFFFFFE18)

uint32_t __section(".SectionForInternalRam.pcmMoRegisterBackup") pcmMoRegisterBackup;
uint32_t __section(".SectionForInternalRam.pcmPllaRegisterBackup") pcmPllaRegisterBackup;
uint32_t __section(".SectionForInternalRam.pcmMckRegisterBackup") pcmMckRegisterBackup;

void __section(".SectionForInternalRam.RestoreClock") AT91SAM9X35_Power_RestoreClock() {
    uint32_t tmp;
    AT91SAM9X35_PMC &pmc = AT91::PMC();

    /* switch to slow clock first */
    if ((pmc.PMC_MCKR & PMC_MCKR_CSS_Msk) != PMC_MCKR_CSS_SLOW_CLK) {
        pmc.PMC_MCKR = (pmc.PMC_MCKR & (uint32_t)~PMC_MCKR_CSS_Msk)
            | (PMC_MCKR_CSS_SLOW_CLK);
        while (!(pmc.PMC_SR & PMC_SR_MCKRDY));
    }

    pmc.PMC_MCKR = (pmc.PMC_MCKR & (uint32_t)~PMC_MCKR_PRES_Msk)
        | (PMC_MCKR_PRES_CLOCK);

    /* Restart Main Oscillator */
    pmc.PMC_CKGR_MOR = CKGR_MOR_KEY(0x37)
        | (pcmMoRegisterBackup & CKGR_MOR_MOSCXTST_Msk)
        | (CKGR_MOR_MOSCRCEN)
        | (CKGR_MOR_MOSCXTEN);
    while (!(pmc.PMC_SR & PMC_SR_MOSCXTS));

    /* Switch to moscsel */
    if (pcmMoRegisterBackup & CKGR_MOR_MOSCSEL) {
        pmc.PMC_CKGR_MOR = CKGR_MOR_KEY(0x37)
            | CKGR_MOR_MOSCXTST(0x3F)
            | CKGR_MOR_MOSCRCEN
            | CKGR_MOR_MOSCXTEN
            | CKGR_MOR_MOSCSEL;
        while (!(pmc.PMC_SR & PMC_SR_MOSCSELS));
    }
    /* Restart PLL A */
    if ((pcmPllaRegisterBackup & CKGR_PLLAR_MULA_Msk) != 0) {
        pmc.PMC_CKGR_PLLAR = pcmPllaRegisterBackup;
        while (!(pmc.PMC_SR & PMC_SR_LOCKA));
    }
    /* Wait for master clock */
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));

    /* Switch to main oscillator */
    tmp = pmc.PMC_MCKR;
    tmp &= ~(PMC_MCKR_MDIV_Msk | PMC_MCKR_PLLADIV2);
    tmp |= (PMC_MCKR_MDIV_Msk | PMC_MCKR_PLLADIV2) & pcmMckRegisterBackup;
    pmc.PMC_MCKR = tmp;
    /* Wait for master clock */
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));

    /* Switch to main oscillator + prescaler */
    tmp = pmc.PMC_MCKR;
    tmp &= ~PMC_MCKR_PRES_Msk;
    tmp |= PMC_MCKR_PRES_Msk & pcmMckRegisterBackup;
    pmc.PMC_MCKR = tmp;
    /* Wait for master clock */
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));

    /* Switch to fast clock */
    tmp = pmc.PMC_MCKR;
    tmp &= ~PMC_MCKR_CSS_Msk;
    tmp |= PMC_MCKR_CSS_Msk & pcmMckRegisterBackup;
    pmc.PMC_MCKR = tmp;
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));

    /* Done */
    pmc.PMC_MCKR = pcmMckRegisterBackup;
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));

}

void __section(".SectionForInternalRam.EnterRcSlowClockMode") AT91SAM9X35_Power_EnterRcSlowClockMode(uint32_t dwPres) {
    uint32_t tmp;
    AT91SAM9X35_PMC &pmc = AT91::PMC();
    /* Switch to slow clock */
    pmc.PMC_MCKR = (pmc.PMC_MCKR & (uint32_t)~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_SLOW_CLK;
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));
    /* Switch to slow clock + prescaler */
    pmc.PMC_MCKR = (pmc.PMC_MCKR & (uint32_t)~PMC_MCKR_PRES_Msk) | dwPres;
    while (!(pmc.PMC_SR & PMC_SR_MCKRDY));

    /* Stop PLL A */
    pmc.PMC_CKGR_PLLAR = 0;
    /* Stop Main Oscillator */
    tmp = pmc.PMC_CKGR_MOR & ~(CKGR_MOR_KEY_Msk | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN);
    pmc.PMC_CKGR_MOR = tmp | CKGR_MOR_KEY(0x37);
}

void __section(".SectionForInternalRam.EnableRcSlowClockMode") AT91SAM9X35_Power_EnableRcSlowClockMode() {

    volatile int i;

    SCK_CR_REG |= 1;//Rc enable

    i = 32768; while (i-- > 0);

    SCK_CR_REG &= ~(1 << 3); // clear xtal to selelct rc

    i = 32768; while (i-- > 0);

    SCK_CR_REG &= ~(1 << 1); // disable xtal

    i = 32768; while (i-- > 0);
}

void __section(".SectionForInternalRam.EnterLowPowerMode") AT91SAM9X35_Power_EnterLowPowerMode() {
    volatile uint32_t reg = 0;

    int i;
    AT91_DDRS *ddrs = (AT91_DDRS *)(AT91C_BASE_DDRS);
    AT91SAM9X35_PMC &pmc = AT91::PMC();

    pcmMoRegisterBackup = pmc.PMC_CKGR_MOR;
    pcmPllaRegisterBackup = pmc.PMC_CKGR_PLLAR;
    pcmMckRegisterBackup = pmc.PMC_MCKR;

    ddrs->DDRSDRC_LPR |= (2 << 12) | 1;

    AT91SAM9X35_Power_EnableRcSlowClockMode();

    AT91SAM9X35_Power_EnterRcSlowClockMode(PMC_MCKR_PRES_CLOCK_DIV64);

    asm("MCR p15, 0, %0, c7, c0, 4" :: "r" (reg));

    ddrs->DDRSDRC_LPR &= ~1;
}

TinyCLR_Result AT91SAM9X35_Power_SetLevel(const TinyCLR_Power_Controller* self, TinyCLR_Power_Level level, TinyCLR_Power_WakeSource wakeSource, uint64_t data) {
    volatile uint32_t reg = 0;
    uint32_t irqStatus;

    AT91SAM9X35_AIC &aic = AT91::AIC();

    switch (level) {
    case TinyCLR_Power_Level::Sleep1: // Sleep
    case TinyCLR_Power_Level::Sleep2: // Sleep
    case TinyCLR_Power_Level::Sleep3: // Sleep

        irqStatus = aic.AIC_IMR;

        for (auto i = 4; i <= 30; i++) {
            aic.AIC_ICCR = (1 << i);
            aic.AIC_IDCR = (1 << i);
        }

        AT91SAM9X35_Display_Disable(nullptr);

        TinyCLR_UsbClient_Uninitialize(nullptr);

        AT91SAM9X35_Power_EnterLowPowerMode();

        for (auto i = 4; i <= 30; i++) {
            if (irqStatus & (1 << i)) {
                aic.AIC_IDCR = (0x01 << i);
                aic.AIC_ICCR = (0x01 << i);
                aic.AIC_IECR = 0x01 << i;
            }
        }

        TinyCLR_UsbClient_Initialize(nullptr);

        AT91SAM9X35_Display_Enable(nullptr);

        break;

    case TinyCLR_Power_Level::Off:    // Off

        *((volatile uint32_t*)SHDW_MR_REG) = 3;
        *((volatile uint32_t*)SHDW_CR_REG) = (0xA5000001);

        break;
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

    return TinyCLR_Result::Success;
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
