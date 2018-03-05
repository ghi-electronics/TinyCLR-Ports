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

void LPC24_Startup_OnSoftReset(const TinyCLR_Api_Provider* apiProvider) {
    LPC24_Gpio_Reset();
}

#define MEM_MAP_REG 0xE01FC040 // memory maping register

// manually filled in
#define PLL_MVAL                        12 // 14
#define PLL_NVAL                        1  // 1
#define CCLK_DIVIDER                    4  // 5
#define USB_DIVIDER                     6  // 7

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

    extern uint32_t Load$$ER_FLASH$$Base;

    extern uint32_t ARM_Vectors;
    extern uint32_t Image$$ER_VECTOR$$Base;
    extern uint32_t Image$$ER_VECTOR$$Length;
}

#pragma arm section code = "SectionForBootstrapOperations"

extern "C" {
    void __section("SectionForBootstrapOperations") SystemInit() {
        // Disconnect the PLL if already connected
        volatile uint32_t i = 65355;
        if ((LPC24XX::SYSCON().PLLSTAT & LPC24XX_SYSCON::CNCTD)) {
            LPC24XX::SYSCON().PLLCON = LPC24XX_SYSCON::PLLE;
            LPC24XX::SYSCON().PLLFEED = 0xAA;
            LPC24XX::SYSCON().PLLFEED = 0x55;
        }

        // Disable the PLL
        LPC24XX::SYSCON().PLLCON = 0x0;
        LPC24XX::SYSCON().PLLFEED = 0xAA;
        LPC24XX::SYSCON().PLLFEED = 0x55;

        // Enable the Oscillator and wait for it to be stable
        LPC24XX::SYSCON().SCS = (LPC24XX::SYSCON().SCS | LPC24XX_SYSCON::OSCEN);
        while (((LPC24XX::SYSCON().SCS & LPC24XX_SYSCON::READY) == 0)) {
        }

        // Select Main Oscillator as the PLL clock source
        LPC24XX::SYSCON().CLKSRCSEL = LPC24XX_SYSCON::OSC;
        LPC24XX::SYSCON().PLLCFG = (((PLL_NVAL - 1) << 16) | (PLL_MVAL - 1));
        LPC24XX::SYSCON().PLLCON = LPC24XX_SYSCON::PLLE;
        LPC24XX::SYSCON().PLLFEED = 0xAA;
        LPC24XX::SYSCON().PLLFEED = 0x55;

        // Wait while PLL locks
        while ((LPC24XX::SYSCON().PLLSTAT & LPC24XX_SYSCON::LOCKD) == 0x0) {
        }

        // Enable MAM
        LPC24XX::SYSCON().MAMCR = 0; // turn off
        while (LPC24XX::SYSCON().MAMCR != 0 && i > 0) {
            i--;
        }
        LPC24XX::SYSCON().MAMTIM = 4; // turn on
        LPC24XX::SYSCON().MAMCR = 2; // set max performance

        // Set CCLK and USB clock divider
        LPC24XX::SYSCON().CCLKCFG = (CCLK_DIVIDER - 1);
        LPC24XX::SYSCON().USBCLKCFG = (USB_DIVIDER - 1);

        // PLL locked now connect it
        LPC24XX::SYSCON().PLLCON = (LPC24XX_SYSCON::PLLC | LPC24XX_SYSCON::PLLE);
        LPC24XX::SYSCON().PLLFEED = 0xAA;
        LPC24XX::SYSCON().PLLFEED = 0x55;

        LPC24XX::SYSCON().PCONP &= (~(1 << 2 | 1 << 19 | 1 << 20 | 1 << 22 | 1 << 23 | 1 << 26 | 1 << 27));

        LPC24XX::SYSCON().PCONP |= 1 << 30;
        LPC24XX::SYSCON().PCONP |= 1 << 31;

        // all perihperals run at 72 / 4, but spi make it 72/1. for uart see code.
        LPC24XX::SYSCON().PCLKSEL0 = 1 << 20;
        LPC24XX::SYSCON().PCLKSEL1 = 1 << 10;

        return;

    }
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

void LPC24_Startup_GetHeap(uint8_t*& start, size_t& length) {
    start = (uint8_t*)&HeapBegin;
    length = (size_t)(((int)&HeapEnd) - ((int)&HeapBegin));
}

void LPC24_Startup_Initialize() {
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

    //
    // Copy Interrupt to RAM.
    //
    {
        uint32_t *src = (uint32_t*)((uint32_t)&ARM_Vectors);
        uint32_t *dst = (uint32_t*)((uint32_t)&Image$$ER_VECTOR$$Base);
        uint32_t  len = (uint32_t)((uint32_t)&Image$$ER_VECTOR$$Length);
        memcpy(dst, src, len);
    }

    //
    // Map vector to RAM.
    //
    {
        uint32_t* memoryMapReg = (uint32_t*)MEM_MAP_REG;
        *memoryMapReg = 0x2; // map ping memory to RAM
    }

}


