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
#include <stdio.h>

void STM32F4_Startup_OnSoftReset(const TinyCLR_Api_Provider* apiProvider) {
#ifdef INCLUDE_ADC
    STM32F4_Adc_Reset();
#endif
#ifdef INCLUDE_CAN
    STM32F4_Can_Reset();
#endif
#ifdef INCLUDE_DAC
    STM32F4_Dac_Reset();
#endif
#ifdef INCLUDE_DISPLAY
    STM32F4_Display_Reset();
#endif
#ifdef INCLUDE_GPIO
    STM32F4_Gpio_Reset();
#endif
#ifdef INCLUDE_I2C
    STM32F4_I2c_Reset();
#endif    
#ifdef INCLUDE_PWM
    STM32F4_Pwm_Reset();
#endif
#ifdef INCLUDE_SPI
    STM32F4_Spi_Reset();
#endif
#ifdef INCLUDE_UART
    STM32F4_Uart_Reset();
#endif
#ifdef INCLUDE_USBCLIENT 
    UsbClient_Reset();
#endif
}

#ifndef FLASH
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#endif

///////////////////////////////////////////////////////////////////////////////

#define ONE_MHZ                             1000000

/* STM32F4 clock configuration */
#if !defined(STM32F4_EXT_CRYSTAL_CLOCK_HZ)
#define STM32F4_INTERNAL_OSCILATOR_CLOCK_HZ 16000000
#define STM32F4_EXT_CRYSTAL_CLOCK_HZ STM32F4_INTERNAL_OSCILATOR_CLOCK_HZ
#define RCC_PLLCFGR_PLLS_BITS (RCC_PLLCFGR_PLLSRC_HSI)
#else
#define RCC_PLLCFGR_PLLS_BITS (RCC_PLLCFGR_PLLSRC_HSE)
#endif

#if STM32F4_EXT_CRYSTAL_CLOCK_HZ % ONE_MHZ != 0
#error STM32F4_EXT_CRYSTAL_CLOCK_HZ must be a multiple of 1MHz
#endif

#if (STM32F4_SYSTEM_CLOCK_HZ * 2 >= 192000000)\
 && (STM32F4_SYSTEM_CLOCK_HZ * 2 <= 432000000)\
 && (STM32F4_SYSTEM_CLOCK_HZ * 2 % 48000000 == 0)
#define RCC_PLLCFGR_PLLM_BITS (STM32F4_EXT_CRYSTAL_CLOCK_HZ / ONE_MHZ * RCC_PLLCFGR_PLLM_0)
#define RCC_PLLCFGR_PLLN_BITS (STM32F4_SYSTEM_CLOCK_HZ * 2 / ONE_MHZ * RCC_PLLCFGR_PLLN_0)
#define RCC_PLLCFGR_PLLP_BITS (0)  // P = 2
#define RCC_PLLCFGR_PLLQ_BITS (STM32F4_SYSTEM_CLOCK_HZ * 2 / 48000000 * RCC_PLLCFGR_PLLQ_0)
#elif (STM32F4_SYSTEM_CLOCK_HZ * 4 >= 192000000)\
   && (STM32F4_SYSTEM_CLOCK_HZ * 4 <= 432000000)\
   && (STM32F4_SYSTEM_CLOCK_HZ * 4 % 48000000 == 0)
#define RCC_PLLCFGR_PLLM_BITS (STM32F4_EXT_CRYSTAL_CLOCK_HZ / ONE_MHZ * RCC_PLLCFGR_PLLM_0)
#define RCC_PLLCFGR_PLLN_BITS (STM32F4_SYSTEM_CLOCK_HZ * 4 / ONE_MHZ * RCC_PLLCFGR_PLLN_0)
#define RCC_PLLCFGR_PLLP_BITS (RCC_PLLCFGR_PLLP_0)  // P = 4
#define RCC_PLLCFGR_PLLQ_BITS (STM32F4_SYSTEM_CLOCK_HZ * 4 / 48000000 * RCC_PLLCFGR_PLLQ_0)
#elif (STM32F4_SYSTEM_CLOCK_HZ * 6 >= 192000000)\
   && (STM32F4_SYSTEM_CLOCK_HZ * 6 <= 432000000)\
   && (STM32F4_SYSTEM_CLOCK_HZ * 6 % 48000000 == 0)
#define RCC_PLLCFGR_PLLM_BITS (STM32F4_EXT_CRYSTAL_CLOCK_HZ / ONE_MHZ * RCC_PLLCFGR_PLLM_0)
#define RCC_PLLCFGR_PLLN_BITS (STM32F4_SYSTEM_CLOCK_HZ * 6 / ONE_MHZ * RCC_PLLCFGR_PLLN_0)
#define RCC_PLLCFGR_PLLP_BITS (RCC_PLLCFGR_PLLP_1)  // P = 6
#define RCC_PLLCFGR_PLLQ_BITS (STM32F4_SYSTEM_CLOCK_HZ * 6 / 48000000 * RCC_PLLCFGR_PLLQ_0)
#elif (STM32F4_SYSTEM_CLOCK_HZ * 8 >= 192000000)\
   && (STM32F4_SYSTEM_CLOCK_HZ * 8 <= 432000000)\
   && (STM32F4_SYSTEM_CLOCK_HZ * 8 % 48000000 == 0)
#define RCC_PLLCFGR_PLLM_BITS (STM32F4_EXT_CRYSTAL_CLOCK_HZ / ONE_MHZ * RCC_PLLCFGR_PLLM_0)
#define RCC_PLLCFGR_PLLN_BITS (STM32F4_SYSTEM_CLOCK_HZ * 8 / ONE_MHZ * RCC_PLLCFGR_PLLN_0)
#define RCC_PLLCFGR_PLLP_BITS (RCC_PLLCFGR_PLLP_0 | RCC_PLLCFGR_PLLP_1)  // P = 8
#define RCC_PLLCFGR_PLLQ_BITS (STM32F4_SYSTEM_CLOCK_HZ * 8 / 48000000 * RCC_PLLCFGR_PLLQ_0)
#elif (STM32F4_SYSTEM_CLOCK_HZ * 2 >= 192000000)\
    && (STM32F4_SYSTEM_CLOCK_HZ * 2 <= 432000000)\
    && (STM32F4_SYSTEM_CLOCK_HZ * 4 % 48000000 == 0)
#define RCC_PLLCFGR_PLLM_BITS (STM32F4_EXT_CRYSTAL_CLOCK_HZ / ONE_MHZ / RCC_PLLCFGR_PLLM_1 * RCC_PLLCFGR_PLLM_0)
#define RCC_PLLCFGR_PLLN_BITS (STM32F4_SYSTEM_CLOCK_HZ *2 / ONE_MHZ * RCC_PLLCFGR_PLLN_0)
#define RCC_PLLCFGR_PLLP_BITS (RCC_PLLCFGR_PLLP_0 )
#define RCC_PLLCFGR_PLLQ_BITS (STM32F4_SYSTEM_CLOCK_HZ *4 / 48000000 * RCC_PLLCFGR_PLLQ_0)
#else
#error illegal STM32F4_SYSTEM_CLOCK_HZ frequency
#endif

#define RCC_PLLCFGR_PLL_BITS (RCC_PLLCFGR_PLLM_BITS \
                            | RCC_PLLCFGR_PLLN_BITS \
                            | RCC_PLLCFGR_PLLP_BITS \
                            | RCC_PLLCFGR_PLLQ_BITS \
                            | RCC_PLLCFGR_PLLS_BITS)

#if STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 1
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV1
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 2
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV2
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 4
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV4
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 8
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV8
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 16
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV16
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 64
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV64
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 128
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV128
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 256
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV256
#elif STM32F4_SYSTEM_CLOCK_HZ == STM32F4_AHB_CLOCK_HZ * 512
#define RCC_CFGR_HPRE_DIV_BITS RCC_CFGR_HPRE_DIV512
#else
#error STM32F4_SYSTEM_CLOCK_HZ must be STM32F4_AHB_CLOCK_HZ * 1, 2, 4, 8, .. 256, or 512
#endif

#if STM32F4_AHB_CLOCK_HZ == STM32F4_APB1_CLOCK_HZ * 1
#define RCC_CFGR_PPRE1_DIV_BITS RCC_CFGR_PPRE1_DIV1
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB1_CLOCK_HZ * 2
#define RCC_CFGR_PPRE1_DIV_BITS RCC_CFGR_PPRE1_DIV2
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB1_CLOCK_HZ * 4
#define RCC_CFGR_PPRE1_DIV_BITS RCC_CFGR_PPRE1_DIV4
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB1_CLOCK_HZ * 8
#define RCC_CFGR_PPRE1_DIV_BITS RCC_CFGR_PPRE1_DIV8
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB1_CLOCK_HZ * 16
#define RCC_CFGR_PPRE1_DIV_BITS RCC_CFGR_PPRE1_DIV16
#else
#error STM32F4_AHB_CLOCK_HZ must be STM32F4_APB1_CLOCK_HZ * 1, 2, 4, 8, or 16
#endif

#if STM32F4_AHB_CLOCK_HZ == STM32F4_APB2_CLOCK_HZ * 1
#define RCC_CFGR_PPRE2_DIV_BITS RCC_CFGR_PPRE2_DIV1
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB2_CLOCK_HZ * 2
#define RCC_CFGR_PPRE2_DIV_BITS RCC_CFGR_PPRE2_DIV2
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB2_CLOCK_HZ * 4
#define RCC_CFGR_PPRE2_DIV_BITS RCC_CFGR_PPRE2_DIV4
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB2_CLOCK_HZ * 8
#define RCC_CFGR_PPRE2_DIV_BITS RCC_CFGR_PPRE2_DIV8
#elif STM32F4_AHB_CLOCK_HZ == STM32F4_APB2_CLOCK_HZ * 16
#define RCC_CFGR_PPRE2_DIV_BITS RCC_CFGR_PPRE2_DIV16
#else
#error STM32F4_AHB_CLOCK_HZ must be STM32F4_APB2_CLOCK_HZ * 1, 2, 4, 8, or 16
#endif

#if STM32F4_SUPPLY_VOLTAGE_MV < 2100
#if STM32F4_AHB_CLOCK_HZ <= 20000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_0WS // no wait states
#elif STM32F4_AHB_CLOCK_HZ <= 40000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_1WS // 1 wait state
#elif STM32F4_AHB_CLOCK_HZ <= 60000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_2WS // 2 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 80000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_3WS // 3 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 100000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_4WS // 4 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 120000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_5WS // 5 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 140000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_6WS // 6 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 160000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_7WS // 7 wait states
#else
#error STM32F4_AHB_CLOCK_HZ must be <= 160MHz at < 2.1V
#endif
#elif STM32F4_SUPPLY_VOLTAGE_MV < 2400
#if STM32F4_AHB_CLOCK_HZ <= 22000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_0WS // no wait states
#elif STM32F4_AHB_CLOCK_HZ <= 44000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_1WS // 1 wait state
#elif STM32F4_AHB_CLOCK_HZ <= 66000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_2WS // 2 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 88000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_3WS // 3 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 110000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_4WS // 4 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 1328000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_5WS // 5 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 154000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_6WS // 6 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 176000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_7WS // 7 wait states
#else
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_8WS // 8 wait states
#endif
#elif STM32F4_SUPPLY_VOLTAGE_MV < 2700
#if STM32F4_AHB_CLOCK_HZ <= 24000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_0WS // no wait states
#elif STM32F4_AHB_CLOCK_HZ <= 48000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_1WS // 1 wait state
#elif STM32F4_AHB_CLOCK_HZ <= 72000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_2WS // 2 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 96000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_3WS // 3 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 120000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_4WS // 4 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 144000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_5WS // 5 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 168000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_6WS // 6 wait states
#else
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_7WS // 7 wait states
#endif
#else
#if STM32F4_AHB_CLOCK_HZ <= 30000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_0WS // no wait states
#elif STM32F4_AHB_CLOCK_HZ <= 60000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_1WS // 1 wait state
#elif STM32F4_AHB_CLOCK_HZ <= 90000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_2WS // 2 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 120000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_3WS // 3 wait states
#elif STM32F4_AHB_CLOCK_HZ <= 150000000
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_4WS // 4 wait states
#else
#define FLASH_ACR_LATENCY_BITS FLASH_ACR_LATENCY_5WS // 5 wait states
#endif
#endif

extern "C" {
    void SystemInit() {
        // enable FPU coprocessors (CP10, CP11)
        SCB->CPACR |= 0x3 << 2 * 10 | 0x3 << 2 * 11; // full access

#if DEBUG || _DEBUG
// configure jtag debug support
        DBGMCU->CR = DBGMCU_CR_DBG_SLEEP;
#endif

        // allow unaligned memory access and do not enforce 8 byte stack alignment
        SCB->CCR &= ~(SCB_CCR_UNALIGN_TRP_Msk | SCB_CCR_STKALIGN_Msk);

        // for clock configuration the cpu has to run on the internal 16MHz oscillator
        RCC->CR |= RCC_CR_HSION;
        while (!(RCC->CR & RCC_CR_HSIRDY));

        RCC->CFGR = RCC_CFGR_SW_HSI;         // sysclk = AHB = APB1 = APB2 = HSI (16MHz)
        RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON); // pll off

#if RCC_PLLCFGR_PLLS_BITS == RCC_PLLCFGR_PLLSRC_HSE
// turn HSE on
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY));
#endif

        // Set flash access time and enable caches & prefetch buffer
        // The prefetch buffer must not be enabled on rev A devices.
        // Rev A cannot be read from revision field (another rev A error!).
        // The wrong device field (411=F2) must be used instead!
        if ((DBGMCU->IDCODE & 0xFF) == 0x11) {
            FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_BITS;
        }
        else {
            FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_BITS;
        }

        // setup PLL
        RCC->PLLCFGR = RCC_PLLCFGR_PLL_BITS; // pll multipliers
        RCC->CR |= RCC_CR_PLLON;             // pll on
        while (!(RCC->CR & RCC_CR_PLLRDY));

        // final clock setup
        RCC->CFGR = RCC_CFGR_SW_PLL          // sysclk = pll out (STM32F4_SYSTEM_CLOCK_HZ)
            | RCC_CFGR_HPRE_DIV_BITS   // AHB clock
            | RCC_CFGR_PPRE1_DIV_BITS  // APB1 clock
            | RCC_CFGR_PPRE2_DIV_BITS; // APB2 clock

    // minimal peripheral clocks
#ifdef RCC_AHB1ENR_CCMDATARAMEN
        RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN; // 64k RAM (CCM)
#endif
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;    // PWR clock used for sleep;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // SYSCFG clock used for IO;

    // stop HSI clock
#if RCC_PLLCFGR_PLLS_BITS == RCC_PLLCFGR_PLLSRC_HSE
        RCC->CR &= ~RCC_CR_HSION;
#endif

        // remove Flash remap to Boot area to avoid problems with Monitor_Execute
        SYSCFG->MEMRMP = 1; // map System memory to Boot area

        // GPIO port A to D is always present
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

#ifdef RCC_AHB1ENR_GPIOEEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
#endif

#ifdef RCC_AHB1ENR_GPIOFEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
#endif

#ifdef RCC_AHB1ENR_GPIOGEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
#endif

#ifdef RCC_AHB1ENR_GPIOHEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
#endif

#ifdef RCC_AHB1ENR_GPIOIEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
#endif

#ifdef RCC_AHB1ENR_GPIOJEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
#endif

#ifdef RCC_AHB1ENR_GPIOKEN
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;
#endif
    }
}

extern "C" {
    extern int HeapBegin;
    extern int HeapEnd;

    extern uint32_t Load$$ER_RAM_RW$$Base;
    extern uint32_t Image$$ER_RAM_RW$$Base;
    extern uint32_t Image$$ER_RAM_RW$$Length;

    extern uint32_t Image$$ER_RAM_RW$$ZI$$Base;
    extern uint32_t Image$$ER_RAM_RW$$ZI$$Length;

    extern uint32_t Load$$ER_RAM_RO$$Base;
    extern uint32_t Image$$ER_RAM_RO$$Base;
    extern uint32_t Image$$ER_RAM_RO$$Length;
}

#pragma arm section code = "SectionForBootstrapOperations"

static void __section("SectionForBootstrapOperations") Prepare_Copy(uint32_t* src, uint32_t* dst, uint32_t len) {
    if (dst != src) {
        int32_t extraLen = len & 0x00000003;
        len = len & 0xFFFFFFFC;

        while (len != 0) {
            *dst++ = *src++;

            len -= 4;
        }

        // thumb2 code can be multiples of 2...

        uint8_t *dst8 = (uint8_t*)dst, *src8 = (uint8_t*)src;

        while (extraLen > 0) {
            *dst8++ = *src8++;

            extraLen--;
        }
    }
}

static void __section("SectionForBootstrapOperations") Prepare_Zero(uint32_t* dst, uint32_t len) {
    int32_t extraLen = len & 0x00000003;
    len = len & 0xFFFFFFFC;

    while (len != 0) {
        *dst++ = 0;

        len -= 4;
    }

    // thumb2 code can be multiples of 2...

    uint8_t *dst8 = (uint8_t*)dst;

    while (extraLen > 0) {
        *dst8++ = 0;

        extraLen--;
    }
}

void STM32F4_Startup_GetHeap(uint8_t*& start, size_t& length) {
    start = (uint8_t*)&HeapBegin;
    length = (size_t)(((int)&HeapEnd) - ((int)&HeapBegin));
}

void STM32F4_Startup_Initialize() {
    //
    // Copy RAM RO regions into proper location.
    //
    {
        uint32_t* src = (uint32_t*)((uint32_t)&Load$$ER_RAM_RO$$Base);
        uint32_t* dst = (uint32_t*)((uint32_t)&Image$$ER_RAM_RO$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_RAM_RO$$Length);

        if (len != 0) // only copy if len is not 0
            Prepare_Copy(src, dst, len);
    }

    //
    // Copy RAM RW regions into proper location.
    //
    {
        uint32_t* src = (uint32_t*)((uint32_t)&Load$$ER_RAM_RW$$Base);
        uint32_t* dst = (uint32_t*)((uint32_t)&Image$$ER_RAM_RW$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_RAM_RW$$Length);

        if (len != 0) // only copy if len is not zero or not compress RW data
            Prepare_Copy(src, dst, len);
    }

    //
    // Initialize RAM ZI regions.
    //
    {
        uint32_t* dst = (uint32_t*)((uint32_t)&Image$$ER_RAM_RW$$ZI$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_RAM_RW$$ZI$$Length);

        Prepare_Zero(dst, len);
    }

}

void STM32F4_Startup_GetDebuggerTransportProvider(const TinyCLR_Api_Info*& api, size_t& index) {
#if defined(DEBUGGER_SELECTOR_PIN) && defined(DEBUGGER_SELECTOR_PULL) && defined(DEBUGGER_SELECTOR_USB_STATE)
    TinyCLR_Gpio_PinValue value;
    auto controller = static_cast<const TinyCLR_Gpio_Provider*>(STM32F4_Gpio_GetApi()->Implementation);

    controller->AcquirePin(controller, DEBUGGER_SELECTOR_PIN);
    controller->SetDriveMode(controller, DEBUGGER_SELECTOR_PIN, DEBUGGER_SELECTOR_PULL);
    controller->Read(controller, DEBUGGER_SELECTOR_PIN, value);
    controller->ReleasePin(controller, DEBUGGER_SELECTOR_PIN);

    if (value == DEBUGGER_SELECTOR_USB_STATE) {
        api = UsbClient_GetApi();
        index = USB_DEBUGGER_INDEX;
    }
    else {
        api = STM32F4_Uart_GetApi();
        index = UART_DEBUGGER_INDEX;
    }
#elif defined(DEBUGGER_FORCE_API) && defined(DEBUGGER_FORCE_INDEX)
    api = DEBUGGER_FORCE_API;
    index = DEBUGGER_FORCE_INDEX;
#else
#error You must specify a debugger mode pin or specify the API explicitly.
#endif
}

void STM32F4_Startup_GetRunApp(bool& runApp) {
#if defined(RUN_APP_PIN) && defined(RUN_APP_PULL) && defined(RUN_APP_STATE)
    TinyCLR_Gpio_PinValue value;
    auto controller = static_cast<const TinyCLR_Gpio_Provider*>(STM32F4_Gpio_GetApi()->Implementation);
    controller->AcquirePin(controller, RUN_APP_PIN);
    controller->SetDriveMode(controller, RUN_APP_PIN, RUN_APP_PULL);
    controller->Read(controller, RUN_APP_PIN, value);
    controller->ReleasePin(controller, RUN_APP_PIN);

    runApp = value == RUN_APP_STATE;
#elif defined(RUN_APP_FORCE_STATE)
    runApp = RUN_APP_FORCE_STATE;
#else
    runApp = true;
#endif
}

