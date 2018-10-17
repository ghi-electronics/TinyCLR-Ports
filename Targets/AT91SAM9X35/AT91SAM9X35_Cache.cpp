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

void AT91SAM9X35_Cache_FlushCaches() {
    uint32_t reg = 0;
#ifdef __GNUC__
    asm("MCR p15, 0, %0, c7,  c6, 0" :: "r" (reg));
    asm("MCR p15, 0, %0, c7, c10, 4" :: "r" (reg));
    asm("MCR p15, 0, %0, c7,  c5, 0" :: "r" (reg));
#else
    __asm
    {
        mcr p15, 0, reg, c7, c6, 0 // invalidate DCache (Write through)
        mcr p15, 0, reg, c7, c10, 4 // Drain write buffer
        mcr p15, 0, reg, c7, c5, 0 // invalidate Icache
    }
#endif
    //  tci_loop:
    //  mrc p15, 0, pc, c7, c14, 3 // test clean & invalidate DCache
    //  bne tci_loop
}

void AT91SAM9X35_Cache_DrainWriteBuffers() {
    uint32_t  reg = 0;

#ifdef __GNUC__
    asm("MCR p15, 0, %0, c7, c10, 4" :: "r" (reg));
#else
    __asm
    {
        mcr     p15, 0, reg, c7, c10, 4 // Drain Write Buffers.
    }
#endif
}

void AT91SAM9X35_Cache_InvalidateCaches() {
    uint32_t reg = 0;

#ifdef __GNUC__
    asm("MCR p15, 0, %0, c7, c7, 0" :: "r" (reg));
#else
    __asm
    {
        mcr     p15, 0, reg, c7, c7, 0 // Invalidate caches.
    }
#endif
}

void AT91SAM9X35_Cache_EnableCaches() {
    uint32_t reg;

    AT91SAM9X35_Cache_InvalidateCaches();

#ifdef __GNUC__
    asm("MRC p15, 0, %0, c1, c0, 0" : "=r" (reg));
    asm("ORR %0, %0, #0x1000"       : "=r" (reg) : "r" (reg));
    asm("ORR %0, %0, #0x0004"       : "=r" (reg) : "r" (reg));
    asm("MCR p15, 0, %0, c1, c0, 0" : : "r" (reg));
#else
    __asm
    {
        mrc     p15, 0, reg, c1, c0, 0
        orr     reg, reg, #0x1000             // Enable ICache
        orr     reg, reg, #0x0004             // Enable DCache
        mcr     p15, 0, reg, c1, c0, 0
    }
#endif
}

void AT91SAM9X35_Cache_DisableCaches() {
    uint32_t reg;

#ifdef __GNUC__
    asm("MRC p15, 0, %0, c1, c0, 0" : "=r" (reg));
    asm("BIC %0, %0, #0x1000"       : "=r" (reg) : "r" (reg));
    asm("BIC %0, %0, #0x0004"       : "=r" (reg) : "r" (reg));
    asm("MCR p15, 0, %0, c1, c0, 0" : : "r" (reg));
#else
    __asm
    {
        mrc     p15, 0, reg, c1, c0, 0
        bic     reg, reg, #0x1000             // Disable ICache
        bic     reg, reg, #0x0004             // Disable DCache
        mcr     p15, 0, reg, c1, c0, 0
    }
#endif

    AT91SAM9X35_Cache_FlushCaches();
}

//--//

template <typename T> void AT91SAM9X35_Cache_InvalidateAddress(T* address) {
    uint32_t reg = 0;

#ifdef __GNUC__
    asm("MCR p15, 0, %0, c7, c10, 4" :: "r" (reg));
    asm("MCR p15, 0, %0, c7,  c5, 1" :: "r" (address));
    asm("MCR p15, 0, %0, c7,  c6, 1" :: "r" (address));
#else
    __asm
    {
        mcr     p15, 0, reg, c7, c10, 4        // Drain Write Buffers.
        mcr     p15, 0, address, c7, c5, 1        // Invalidate ICache.
        mcr     p15, 0, address, c7, c6, 1        // Invalidate DCache.
    }
#endif
}

//--//

size_t AT91SAM9X35_Cache_GetCachableAddress(size_t address) {
    return address;
}

//--//

size_t AT91SAM9X35_Cache_GetUncachableAddress(size_t address) {
    return address;
}

