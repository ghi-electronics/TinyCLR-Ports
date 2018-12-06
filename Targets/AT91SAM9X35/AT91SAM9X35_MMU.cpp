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

//--//


uint32_t* ARM9_MMU::GetL1Entry(uint32_t* base, uint32_t address) {
    return &base[address >> 20];
}

void ARM9_MMU::InitializeL1(uint32_t* baseOfTTBs) {
    uint32_t* dst = baseOfTTBs;
    uint32_t* end = (uint32_t*)((uint32_t)baseOfTTBs + ARM9_MMU::c_TTB_size);

    do {
        *dst++ = 0;
    } while (dst < end);
}

uint32_t ARM9_MMU::GenerateL1_Section(uint32_t address, uint32_t AP, uint32_t domain, bool Cachable, bool Buffered, bool Xtended) {
    uint32_t ret;

    ret = (address & 0xFFF00000);
    ret |= (Xtended ? 1 : 0) << 12;
    ret |= (AP & 0x00000003) << 10;
    ret |= (domain & 0x0000000F) << 5;
    ret |= (Cachable ? 1 : 0) << 3;
    ret |= (Buffered ? 1 : 0) << 2;
    ret |= c_MMU_L1_Section;

    return ret;
}

void ARM9_MMU::GenerateL1_Sections(uint32_t* baseOfTTBs, uint32_t mappedAddress, uint32_t physAddress, int32_t size, uint32_t AP, uint32_t domain, bool Cachable, bool Buffered, bool Xtended) {
    uint32_t* dst = ARM9_MMU::GetL1Entry(baseOfTTBs, mappedAddress);

    do {
        *dst++ = ARM9_MMU::GenerateL1_Section(physAddress, AP, domain, Cachable, Buffered, Xtended);

        physAddress += ARM9_MMU::c_MMU_L1_size;
        size -= ARM9_MMU::c_MMU_L1_size;
    } while (size > 0);
}

#if defined(__GNUC__)

extern "C"
{
    void	AT91SAM9X35_CPU_InvalidateTLBs_asm();
    void	AT91SAM9X35_CPU_EnableMMU_asm(void* TTB);
    void	AT91SAM9X35_CPU_DisableMMU_asm();
    bool	AT91SAM9X35_CPU_IsMMUEnabled_asm();
}

void AT91SAM9X35_CPU_InvalidateTLBs() {
    AT91SAM9X35_CPU_InvalidateTLBs_asm();
}

void AT91SAM9X35_CPU_EnableMMU(void* TTB) {
    AT91SAM9X35_CPU_EnableMMU_asm(TTB);
    AT91SAM9X35_CPU_InvalidateTLBs_asm();
}

void AT91SAM9X35_CPU_DisableMMU() {
    AT91SAM9X35_CPU_DisableMMU_asm();
}

bool AT91SAM9X35_CPU_IsMMUEnabled() {
    return AT91SAM9X35_CPU_IsMMUEnabled_asm();
}

#elif (defined(COMPILE_ARM) || defined(COMPILE_THUMB))

#pragma ARM

#define ARM9_MMU_ASM_START() uint32_t regTmp
#define ARM9_MMU_ASM_WAIT() \
        mrc     p15, 0, regTmp, c2, c0, 0; \
        nop \

void CPU_InvalidateTLBs() {
    ARM9_MMU_ASM_START();
    uint32_t reg = 0;

    __asm
    {
        mov     reg, #0;
        mcr     p15, 0, reg, c8, c7, 0;             // Invalidate MMU TLBs.

        ARM9_MMU_ASM_WAIT();
    }
}

void CPU_EnableMMU(void* TTB) {
    ARM9_MMU_ASM_START();
    uint32_t reg;

    __asm
    {
        mcr     p15, 0, TTB, c2, c0, 0;        // Set the TTB address location to CP15

        ARM9_MMU_ASM_WAIT();

        mrc     p15, 0, reg, c1, c0, 0;
        orr     reg, reg, #0x0001;             // Enable MMU
        mcr     p15, 0, reg, c1, c0, 0;

        ARM9_MMU_ASM_WAIT();

        // Note that the 2 preceeding instruction would still be prefetched
        // under the physical address space instead of in virtual address space.
    }

    CPU_InvalidateTLBs();
}

void CPU_DisableMMU() {
    ARM9_MMU_ASM_START();
    uint32_t reg;

    __asm
    {
        mrc     p15, 0, reg, c1, c0, 0;
        bic     reg, reg, #0x0001;             // Disable MMU
        mcr     p15, 0, reg, c1, c0, 0;

        ARM9_MMU_ASM_WAIT();

        // Note that the 2 preceeding instruction would still be prefetched
        // under the physical address space instead of in virtual address space.
    }
}

bool CPU_IsMMUEnabled() {
    ARM9_MMU_ASM_START();
    uint32_t reg;

    __asm
    {
        mrc     p15, 0, reg, c1, c0, 0;

        ARM9_MMU_ASM_WAIT();
    }

    return (reg & 0x1);
}

#if defined(COMPILE_THUMB)
#pragma THUMB
#endif

#pragma arm section code

#elif defined(COMPILE_THUMB2)

void CPU_InvalidateTLBs() {
}

void CPU_EnableMMU(void* TTB) {
}

void CPU_DisableMMU() {
}

bool CPU_IsMMUEnabled() {
}

#endif // #if defined(COMPILE_ARM) || defined(COMPILE_THUMB)

extern "C" {
    extern uint32_t Load$$SDRAM$$Base;
    extern uint32_t Image$$SDRAM$$Length;

    extern uint32_t Load$$SRAM$$Base;
    extern uint32_t Image$$SRAM$$Length;

    extern uint32_t Load$$ER_RLP$$Base;
    extern uint32_t Image$$ER_RLP$$Length;

    extern uint32_t Load$$ER_DMA$$Base;
    extern uint32_t Image$$ER_DMA$$Length;

}

static const uint32_t c_Bootstrap_Register_Begin = 0xF0000000;//0xFFF00000;
static const uint32_t c_Bootstrap_Register_End = 0xFFFFFFFF;

static const uint32_t c_RLP_Virtual_Address_Cached = 0xA0000000; // Added for RLP Support of Memory MMU
static const uint32_t c_RLP_Virtual_Address_Uncached = 0xB0000000; // Added for RLP Support of Memory MMU


void AT91SAM9X35_MMU_Initialize() {

    uint32_t c_Bootstrap_SDRAM_Begin = ((uint32_t)&Load$$SDRAM$$Base);
    uint32_t c_Bootstrap_SDRAM_End = c_Bootstrap_SDRAM_Begin + ((uint32_t)&Image$$SDRAM$$Length) - ARM9_MMU::c_TTB_size;
    uint32_t c_Bootstrap_SRAM_Begin = ((uint32_t)&Load$$SRAM$$Base);;
    uint32_t c_Bootstrap_SRAM_End = c_Bootstrap_SRAM_Begin + ((uint32_t)&Image$$SRAM$$Length);
    uint32_t* c_Bootstrap_BaseOfTTBs = (uint32_t*)(c_Bootstrap_SDRAM_End);

    uint32_t c_RLP_Physical_Address = ((uint32_t)&Load$$ER_RLP$$Base);
    uint32_t c_RLP_Size = ((uint32_t)&Image$$ER_RLP$$Length);

    uint32_t c_DMA_Physical_Address = ((uint32_t)&Load$$ER_DMA$$Base);
    uint32_t c_DMA_Size = ((uint32_t)&Image$$ER_DMA$$Length);

    // Fill Translation table with faults.
    ARM9_MMU::InitializeL1(c_Bootstrap_BaseOfTTBs);

    // Direct map for the APB registers (0xFFF00000 ~ 0xFFFFFFFF)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        c_Bootstrap_Register_Begin,                             // mapped address
        c_Bootstrap_Register_Begin,                             // physical address
        c_Bootstrap_Register_End - c_Bootstrap_Register_Begin,  // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        false,                                                  // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended

    // Direct map SDRAM (cachable)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        c_Bootstrap_SDRAM_Begin,                                // mapped address
        c_Bootstrap_SDRAM_Begin,                                // physical address
        c_Bootstrap_SDRAM_End - c_Bootstrap_SDRAM_Begin,        // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        true,                                                   // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended


    // Remap SRAM @0x000000000 (cachable)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        0x00000000,                                             // mapped address
        c_Bootstrap_SRAM_Begin,                                 // physical address
        c_Bootstrap_SRAM_End - c_Bootstrap_SRAM_Begin,          // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        true,                                                   // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended

    // Direct map SRAM (cachable)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        c_Bootstrap_SRAM_Begin,                                 // mapped address
        c_Bootstrap_SRAM_Begin,                                 // physical address
        c_Bootstrap_SRAM_End - c_Bootstrap_SRAM_Begin,          // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        true,                                                   // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended

    // Direct map for the LCD registers(0xF8038000)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        AT91C_BASE_LCDC,                                        // mapped address
        AT91C_BASE_LCDC,                                        // physical address
        ARM9_MMU::c_MMU_L1_size,                                // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        false,                                                  // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended

#ifdef PLATFORM_ARM_SAM9RL64_ANY

        // Direct map for the USBHS buffer registers(0x00500000~0x005FFFFF)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        AT91C_BASE_UDP_DMA,                                     // mapped address
        AT91C_BASE_UDP_DMA,                                     // physical address
        ARM9_MMU::c_MMU_L1_size,                                // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        false,                                                  // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended


#endif



    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        c_RLP_Virtual_Address_Cached,                           // mapped address
        c_RLP_Physical_Address,                                 // physical address
        c_RLP_Size,                                             // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        true,                                                   // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended

    //(uncachable)
    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        c_RLP_Virtual_Address_Uncached,                         // mapped address
        c_RLP_Physical_Address,                                 // physical address
        c_RLP_Size,                                             // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        false,                                                  // Cacheable
        false,                                                  // Buffered
        false);                                                 // Extended

    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        AT91C_BASE_UHP_OHCI,                                    // mapped address
        AT91C_BASE_UHP_OHCI,                                    // physical address
        ARM9_MMU::c_MMU_L1_size,                                // length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        false,                                                  // Cacheable
        false,                                                  // Buffered
        false);

    ARM9_MMU::GenerateL1_Sections(
        c_Bootstrap_BaseOfTTBs,                                 // base of TTBs
        c_DMA_Physical_Address,                                 // mapped address
        c_DMA_Physical_Address,                                 // physical address
        c_DMA_Size,	// length to be mapped
        ARM9_MMU::c_AP__Manager,                                // AP
        0,                                                      // Domain
        false,                                                  // Cacheable
        false,                                                  // Buffered
        false);

    AT91SAM9X35_Cache_FlushCaches();
    AT91SAM9X35_CPU_EnableMMU(c_Bootstrap_BaseOfTTBs);
}