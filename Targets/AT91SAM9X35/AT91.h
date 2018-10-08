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

#pragma once

#include <stdio.h>
#include <string.h>

#include <TinyCLR.h>
#include <Device.h>

#define SIZEOF_ARRAY(arr) (sizeof(arr) / sizeof(arr[0]))
#define CONCAT2(a, b) a##b
#define CONCAT(a, b) CONCAT2(a, b)
#define CHARIZE2(c) #c
#define CHARIZE(c) (CHARIZE2(c)[0])

//
//  PERIPHERAL ID DEFINITIONS FOR AT91SAM9X35
//
#define AT91C_ID_FIQ        ((unsigned int)  0) // Advanced Interrupt Controller (FIQ)
#define AT91C_ID_SYS        ((unsigned int)  1) // System Interrupt
#define AT91C_ID_PIOA_PIOB    ((unsigned int)  2) // Parallel IO Controller A and B
#define AT91C_ID_PIOC_PIOD    ((unsigned int)  3) // Parallel IO Controller C and D
#define AT91C_ID_SMD        ((unsigned int)  4) // SMD Soft Modem - not yet supported
#define AT91C_ID_USART0        ((unsigned int)  5) // USART 0
#define AT91C_ID_USART1        ((unsigned int)  6) // USART 1
#define AT91C_ID_USART2        ((unsigned int)  7) // USART 2
#define AT91C_ID_TWI        AT91C_ID_TWI0       // only support one I2C
#define AT91C_ID_TWI0        ((unsigned int)  9) // Two-Wire Interface 0
#define AT91C_ID_TWI1        ((unsigned int) 10) // Two-Wire Interface 1
#define AT91C_ID_TWI2        ((unsigned int) 11) // Two-Wire Interface 2
#define AT91C_ID_HSMCI0        ((unsigned int) 12) // High Speed Multimedia Card Interface 0
#define AT91C_ID_SPI0        ((unsigned int) 13) // Serial Peripheral Interface
#define AT91C_ID_SPI1        ((unsigned int) 14) // Serial Peripheral Interface
#define AT91C_ID_UART0        ((unsigned int) 15) // UART 0
#define AT91C_ID_UART1        ((unsigned int) 16) // UART 1
#define AT91C_ID_TC0_TC1    ((unsigned int) 17) // Timer Counter 0, 1, 2, 3, 4, 5
#define AT91C_ID_PWM        ((unsigned int) 18) // Pulse Width Modulation Controller
#define AT91C_ID_ADC        ((unsigned int) 19) // ADC Controller
#define AT91C_ID_DMAC0        ((unsigned int) 20) // DMA Controller 0
#define AT91C_ID_DMAC1        ((unsigned int) 21) // DMA Controller 1
#define AT91C_ID_UHPHS        ((unsigned int) 22) // USB Host High Speed
#define AT91C_ID_UDPHS        ((unsigned int) 23) // USB Device High Speed
#define AT91C_ID_EMAC        ((unsigned int) 24) // Ethernet MAC
#define AT91C_ID_LCDC        ((unsigned int) 25) // LCD Controller
#define AT91C_ID_HSMCI1        ((unsigned int) 26) // High Speed Multimedia Card Interface 1
#define AT91C_ID_SSC        ((unsigned int) 28) // Synchronous Serial Controller
#define AT91C_ID_CAN0        ((unsigned int) 29) // CAN Controller 0
#define AT91C_ID_CAN1        ((unsigned int) 30) // CAN Controller 1
#define AT91C_ID_IRQ0        ((unsigned int) 31) // Advanced Interrupt Controller (IRQ0)

//
// BASE ADDRESS DEFINITIONS FOR AT91SAM9RL64
//
#define AT91C_BASE_SYS          0xFFFFE600 // (SYS) Base Address
#define AT91C_BASE_DMAC0        0xFFFFEC00 // Hydra original address 0xFFFFE600 // (DMAC)Address                        - Not same Memory Address
#define AT91C_BASE_DDRS         0xFFFFE800 // (DDRS) Address
#define AT91C_BASE_SDRAMC       0xFFFFEA00 // (SDRAMC) Base Address
#define AT91C_BASE_SMC          0xFFFFEC00 // (SMC) Base Address
#define AT91C_BASE_MATRIX       0xFFFFEE00 // (MATRIX) Base Address                - Not same Memory Address
#define AT91C_BASE_AIC          0xFFFFF000 // (AIC) Base Address
#define AT91C_BASE_PDC_DBGU     0xFFFFF300 // (PDC_DBGU) Base Address
#define AT91C_BASE_DBGU         0xFFFFF200 // (DBGU) Base Address
#define AT91C_BASE_PIOA         0xFFFFF400 // (PIOA) Base Address
#define AT91C_BASE_PIOB         0xFFFFF600 // (PIOB) Base Address
#define AT91C_BASE_PIOC         0xFFFFF800 // (PIOC) Base Address
#define AT91C_BASE_PIOD         0xFFFFFA00 // (PIOD) Base Address
#define AT91C_BASE_CKGR         0xFFFFFC20 // (CKGR) Base Address
#define AT91C_BASE_PMC          0xFFFFFC00 // (PMC) Base Address
#define AT91C_BASE_RSTC         0xFFFFFE00 // (RSTC) Base Address                - Not same Memory Address
#define AT91C_BASE_RSTC_SR      0xFFFFFD04
#define AT91C_BASE_RSTC_MR      0xFFFFFD08
#define AT91C_BASE_SHDWC        0xFFFFFD10 // (SHDWC) Base Address
#define AT91C_BASE_RTTC             0xFFFFFD20 // (RTTC) Base Address
#define AT91C_BASE_PITC             0xFFFFFE30 // (PITC) Base Address
#define AT91C_BASE_WDTC             0xFFFFFE40 // (WDTC) Base Address
#define AT91C_BASE_SCKCR            0xFFFFFE50 // (SCKCR) Base Address
#define AT91C_BASE_GPBR             0xFFFFFD60 // (GPBR) Base Address
#define AT91C_BASE_TC0                 0xF8008000 // Hydra original address 0xFFFA0000 // (TC0) Base Address
#define AT91C_BASE_TC1              0xFFFA0040 // (TC1) Base Address
#define AT91C_BASE_TC2              0xFFFA0080 // (TC2) Base Address
#define AT91C_BASE_TCB0             0xFFFA0000 // (TCB0) Base Address
#define AT91C_BASE_PDC_MCI          0xFFFA4100 // (PDC_MCI) Base Address
#define AT91C_BASE_MCI              0xF0008000 // Hydra original address 0xFFFA4000 // (MCI) Base Address
#define AT91C_BASE_TWI0             0xF8010000 // Hydra original address 0xFFFA8000 // (TWI1) Base Address
#define AT91C_BASE_TWI1             0xF8014000 // Hydra original address 0xFFFAC000 // (TWI2) Base Address
#define AT91C_BASE_TWI2             0xF8018000 // (TWI3) Base Address
#define AT91C_BASE_TWI              AT91C_BASE_TWI0 // Support one I2C controller only
#define AT91C_BASE_USART0              0xF801C000 // Hydra original address 0xFFFB0000 // (US0) Base Address                - Not same Memory Address
#define AT91C_BASE_USART1              0xF8020000 // Hydra original address 0xFFFB4000 // (US1) Base Address                - Not same Memory Address
#define AT91C_BASE_USART2              0xF8024000 // Hydra original address 0xFFFB8000 // (US2) Base Address                - Not same Memory Address
#define AT91C_BASE_UART0            0xF8040000
#define AT91C_BASE_UART1            0xF8044000
#define AT91C_BASE_PDC_US3          0xFFFBC100 // (PDC_US3) Base Address
#define AT91C_BASE_US3              0xF8008000 // Hydra original address 0xFFFBC000 // (US3) Base Address                - Not same Memory Address
#define AT91C_BASE_PDC_SSC0         0xFFFC0100 // (PDC_SSC0) Base Address
#define AT91C_BASE_SSC0             0xFFFC0000 // (SSC0) Base Address
#define AT91C_BASE_PDC_SSC1         0xFFFC4100 // (PDC_SSC1) Base Address
#define AT91C_BASE_SSC1             0xFFFC4000 // (SSC1) Base Address
#define AT91C_BASE_PWMC             0xF8034000 // Hydra original address 0xFFFC8000 // (PWMC) Base Address                - Not same Memory Address
#define AT91C_BASE_PDC_SPI0         0xFFFCC100 // (PDC_SPI0) Base Address
#define AT91C_BASE_SPI0             0xF0000000 // (SPI0) Base Address // Hydra original address 0xFFFCC000
#define AT91C_BASE_SPI1             0xF0004000 // (SPI1) Base Address
#define AT91C_BASE_TSADCC           0xFFFD0000 // (ADC touch screen) Base Address
#define AT91C_BASE_UDP              0xF803C000 // 0xFFFD4000 // (UDP) Base Address - Changed on G400
#define AT91C_BASE_AC97             0xFFFD8000 // (UDP) Base Address
#define AT91C_BASE_ITCM             0x00100000 // (ITCM) Base Address
#define AT91C_BASE_DTCM             0x00200000 // (DTCM) Base Address
#define AT91C_BASE_LCDC             0xF8038000 // Hydra original address 0x00500000 // (LCDC) Base Address
#define AT91C_BASE_UDP_DMA            0x00500000 // Hydra original address 0x00600000 // (UDP DMA) Base Address
#define AT91C_BASE_EMAC                0xF802C000

// RTSC bit defines
#define AT91C_RTSC__PROCRST     0x01         // processor reset bit
#define AT91C_RTSC__PERRST      0x04         // Peripheral reset bit
#define AT91C_RTSC__EXTRST      0x08         // asserts NRST pin
#define AT91C_RSTC__RESET_KEY   0xA5000000   // reset key
#define AT91C_RTSC_SR__SRCMP    (1ul < 17)   // Software Reset Command in progress
#define AT91C_RTSC_SR__NRSTL    (1ul < 16)   // Registers the NRST Pin Level at Master Clock (MCK)
#define AT91C_RTSC_SR__RSTTYP   (7ul <  8)   // Reset Type mask
#define AT91C_RTSC_SR__BODSTS   (1ul <  1)   // Brownout Detection Status
#define AT91C_RTSC_SR__URSTS    (1ul <  0)   // User Reset Status

// SHDWC bit defines
#define AT91C_SHDWC__SHUTDOWN_KEY   0xA5000000   // reset key
#define AT91C_SHDWC__SHDW           0x01         // processor reset bit

extern const TinyCLR_Api_Manager* apiManager;

struct AT91_PMC {
    static const uint32_t c_Base = AT91C_BASE_PMC;

    volatile uint32_t PMC_SCER;                // System Clock Enable Register

    volatile uint32_t PMC_SCDR;                // System Clock Disable Register

    volatile uint32_t PMC_SCSR;                // System Clock Status Register

    volatile uint32_t Reserved0[1];

    volatile uint32_t PMC_PCER;                // Peripheral Clock Enable Register

    volatile uint32_t PMC_PCDR;                // Peripheral Clock Disable Register

    volatile uint32_t PMC_PCSR;                // Peripheral Clock Status Register

    volatile uint32_t PMC_CKGR_UCKR;            // UTMI Clock Register

    volatile uint32_t PMC_CKGR_MOR;            // Main Oscillator Register
    static const    uint32_t CKGR_MOSCEN = (0x1 << 0);       // (CKGR) Main Oscillator Enable
    static const    uint32_t CKGR_OSCBYPASS = (0x1 << 1);       // (CKGR) Main Oscillator Bypass
    static const    uint32_t CKGR_OSCOUNT = (0xFF << 8);      // (CKGR) Main Oscillator Start-up Time


    volatile uint32_t PMC_CKGR_MCFR;  // Main Clock  Frequency Register
    static const    uint32_t CKGR_MAINF = (0xFFFF << 0);     // (CKGR) Main Clock Frequency
    static const    uint32_t CKGR_MAINRDY = (0x1 << 16);       // (CKGR) Main Clock Ready


    volatile uint32_t PMC_CKGR_PLLAR;  // PLL Register


    static const    uint32_t CKGR_DIV = (0xFF << 0);     // (CKGR) Divider Selected
    static const    uint32_t CKGR_DIV_0 = (0x0);           // (CKGR) Divider output is 0
    static const    uint32_t CKGR_DIV_BYPASS = (0x1);           // (CKGR) Divider is bypassed
    static const    uint32_t CKGR_PLLCOUNT = (0x3F << 8);     // (CKGR) PLL Counter
    static const    uint32_t CKGR_OUT = (0x3 << 14);     // (CKGR) PLL Output Frequency Range
    static const    uint32_t CKGR_OUT_0 = (0x0 << 14);     // (CKGR) Please refer to the PLL datasheet
    static const    uint32_t CKGR_OUT_1 = (0x1 << 14);     // (CKGR) Please refer to the PLL datasheet
    static const    uint32_t CKGR_OUT_2 = (0x2 << 14);     // (CKGR) Please refer to the PLL datasheet
    static const    uint32_t CKGR_OUT_3 = (0x3 << 14);     // (CKGR) Please refer to the PLL datasheet
    static const    uint32_t CKGR_MUL = (0x7FF << 16);   // (CKGR) PLL Multiplier
    static const    uint32_t CKGR_USBDIV = (0x3 << 28);     // (CKGR) Divider for USB Clocks
    static const    uint32_t CKGR_USBDIV_0 = (0x0 << 28);     // (CKGR) Divider output is PLL clock output
    static const    uint32_t CKGR_USBDIV_1 = (0x1 << 28);     // (CKGR) Divider output is PLL clock output divided by 2
    static const    uint32_t CKGR_USBDIV_2 = (0x2 << 28);     // (CKGR) Divider output is PLL clock output divided by 4

    volatile uint32_t Reserved1[1];

    volatile uint32_t PMC_MCKR;  // Master Clock Register
    static const    uint32_t PMC_CSS = (0x3 << 0);   //  (PMC) Programmable Clock Selection
    static const    uint32_t PMC_CSS_SLOW_CLK = (0x0);         //  (PMC) Slow Clock is selected
    static const    uint32_t PMC_CSS_MAIN_CLK = (0x1);         //  (PMC) Main Clock is selected
    static const    uint32_t PMC_CSS_PLLA_CLK = (0x2);         //  (PMC) Clock from PLL1 is selected = (SAM9 only);
    static const    uint32_t PMC_CSS_PLL_CLK = (0x3);         //  (PMC) Clock from PLL is selected
    static const    uint32_t PMC_PRES = (0x7 << 2);   //  (PMC) Programmable Clock Prescaler
    static const    uint32_t PMC_PRES_CLK = (0x0 << 2);   //  (PMC) Selected clock
    static const    uint32_t PMC_PRES_CLK_2 = (0x1 << 2);   //  (PMC) Selected clock divided by 2
    static const    uint32_t PMC_PRES_CLK_4 = (0x2 << 2);   //  (PMC) Selected clock divided by 4
    static const    uint32_t PMC_PRES_CLK_8 = (0x3 << 2);   //  (PMC) Selected clock divided by 8
    static const    uint32_t PMC_PRES_CLK_16 = (0x4 << 2);   //  (PMC) Selected clock divided by 16
    static const    uint32_t PMC_PRES_CLK_32 = (0x5 << 2);   //  (PMC) Selected clock divided by 32
    static const    uint32_t PMC_PRES_CLK_64 = (0x6 << 2);   //  (PMC) Selected clock divided by 64
    static const    uint32_t PMC_MDIV = (0x3 << 8);   //  (PMC) Master Clock Division
    static const    uint32_t PMC_MDIV_1 = (0x0 << 8);   //  (PMC) The master clock and the processor clock are the same
    static const    uint32_t PMC_MDIV_2 = (0x1 << 8);   //  (PMC) The processor clock is twice as fast as the master clock
    static const    uint32_t PMC_MDIV_3 = (0x2 << 8);   //  (PMC) The processor clock is four times faster than the master clock

    volatile uint32_t Reserved3[3];  // TQD: Just a note PMC_USB and PMC_SMD are in here

    volatile uint32_t PMC_PCKR[2];   // Programmable Clock Register

    volatile uint32_t Reserved4[6];

    volatile uint32_t PMC_IER;   // Interrupt Enable Register
    static const    uint32_t PMC_MOSCS = (0x1 << 0);       // (PMC) MOSC Status/Enable/Disable/Mask
    static const    uint32_t PMC_LOCKA = (0x1 << 1);       // (PMC) PLL Status/Enable/Disable/Mask =(SAM9 Only);
    static const    uint32_t PMC_LOCK = (0x1 << 2);       // (PMC) PLL Status/Enable/Disable/Mask
    static const    uint32_t PMC_MCKRDY = (0x1 << 3);       // (PMC) MCK_RDY Status/Enable/Disable/Mask
    static const    uint32_t PMC_PCK0RDY = (0x1 << 8);       // (PMC) PCK0_RDY Status/Enable/Disable/Mask
    static const    uint32_t PMC_PCK1RDY = (0x1 << 9);       // (PMC) PCK1_RDY Status/Enable/Disable/Mask
    static const    uint32_t PMC_PCK2RDY = (0x1 << 10);       // (PMC) PCK2_RDY Status/Enable/Disable/Mask
    static const    uint32_t PMC_PCK3RDY = (0x1 << 11);       // (PMC) PCK3_RDY Status/Enable/Disable/Mask
    static const    uint32_t PMC_LOCKU = (0x1 << 6);           //(PMC) USB clock locked
    volatile uint32_t PMC_IDR;   // Interrupt Disable Register

    volatile uint32_t PMC_SR;    // Status Register

    volatile uint32_t PMC_IMR;   // Interrupt Mask Register

    volatile uint32_t Reserved5[3]; // Reserved

    volatile uint32_t PMC_PLLICPR; // Charge Pump Current Register
    static const    uint32_t PMC_PLLICPR__PLLA = (0x1 << 0);
    static const    uint32_t PMC_PLLICPR__PLLB = (0x1 << 16);

    __inline void EnableSystemClock(uint32_t clkIds) {
        PMC_SCER = clkIds;
    }

    __inline void DisableSystemClock(uint32_t clkIds) {
        PMC_SCDR = clkIds;
    }

    __inline void EnablePeriphClock(uint32_t periphIds) {
        PMC_PCER = (1 << periphIds);
    }

    __inline void DisablePeriphClock(uint32_t periphIds) {
        PMC_PCDR = (1 << periphIds);
    }


};


// MMU
struct ARM9_MMU {
    static const uint32_t c_TTB_size = 0x4000;

    static const uint32_t c_MMU_L1_Fault = 0x00;
    static const uint32_t c_MMU_L1_Coarse = 0x11;
    static const uint32_t c_MMU_L1_Section = 0x12;
    static const uint32_t c_MMU_L1_Fine = 0x13;
    static const uint32_t c_MMU_L1_size = 1 << 20;

    static const uint32_t c_AP__NoAccess = 0;
    static const uint32_t c_AP__Client = 1;
    static const uint32_t c_AP__Reserved = 2;
    static const uint32_t c_AP__Manager = 3;

    //--//

    static uint32_t* GetL1Entry(uint32_t* base, uint32_t address);
    static void    InitializeL1(uint32_t* baseOfTTBs);
    static uint32_t  GenerateL1_Section(uint32_t address, uint32_t AP, uint32_t domain, bool Cachable, bool Buffered, bool Xtended = false);
    static void    GenerateL1_Sections(uint32_t* baseOfTTBs, uint32_t mappedAddress, uint32_t physAddress, int32_t size, uint32_t AP, uint32_t domain, bool Cachable, bool Buffered, bool Xtended = false);
};

void AT91_MMU_Initialize();
void AT91_CPU_InvalidateTLBs();
void AT91_CPU_EnableMMU(void* TTB);
void AT91_CPU_DisableMMU();
bool AT91_CPU_IsMMUEnabled();
void AT91_CPU_BootstrapCode();

// Cache
void AT91_Cache_FlushCaches();
void AT91_Cache_DrainWriteBuffers();
void AT91_Cache_InvalidateCaches();
void AT91_Cache_EnableCaches();
void AT91_Cache_DisableCaches();
template <typename T> void AT91_Cache_InvalidateAddress(T* address);
size_t AT91_Cache_GetCachableAddress(size_t address);
size_t AT91_Cache_GetUncachableAddress(size_t address);

// GPIO

//////////////////////////////////////////////////////////////////////////////
// AT91_GPIO
//
struct AT91_PIO {
    static const uint32_t c_Base = AT91C_BASE_PIOA;

    static const uint32_t c_Base_Offset = 0x200;

    volatile uint32_t PIO_PER;       /**< \brief (Pio Offset: 0x0000) PIO Enable Register */

    volatile uint32_t PIO_PDR;       /**< \brief (Pio Offset: 0x0004) PIO Disable Register */

    volatile uint32_t PIO_PSR;       /**< \brief (Pio Offset: 0x0008) PIO Status Register */

    volatile uint32_t Reserved1[1];

    volatile uint32_t PIO_OER;       /**< \brief (Pio Offset: 0x0010) Output Enable Register */

    volatile uint32_t PIO_ODR;       /**< \brief (Pio Offset: 0x0014) Output Disable Register */

    volatile uint32_t PIO_OSR;       /**< \brief (Pio Offset: 0x0018) Output Status Register */

    volatile uint32_t Reserved2[1];

    volatile uint32_t PIO_IFER;      /**< \brief (Pio Offset: 0x0020) Glitch Input Filter Enable Register */

    volatile uint32_t PIO_IFDR;      /**< \brief (Pio Offset: 0x0024) Glitch Input Filter Disable Register */

    volatile uint32_t PIO_IFSR;      /**< \brief (Pio Offset: 0x0028) Glitch Input Filter Status Register */

    volatile uint32_t Reserved3[1];

    volatile uint32_t PIO_SODR;      /**< \brief (Pio Offset: 0x0030) Set Output Data Register */

    volatile uint32_t PIO_CODR;      /**< \brief (Pio Offset: 0x0034) Clear Output Data Register */

    volatile uint32_t PIO_ODSR;      /**< \brief (Pio Offset: 0x0038) Output Data Status Register */

    volatile uint32_t PIO_PDSR;      /**< \brief (Pio Offset: 0x003C) Pin Data Status Register */

    volatile uint32_t PIO_IER;       /**< \brief (Pio Offset: 0x0040) Interrupt Enable Register */

    volatile uint32_t PIO_IDR;       /**< \brief (Pio Offset: 0x0044) Interrupt Disable Register */

    volatile uint32_t PIO_IMR;       /**< \brief (Pio Offset: 0x0048) Interrupt Mask Register */

    volatile uint32_t PIO_ISR;       /**< \brief (Pio Offset: 0x004C) Interrupt Status Register */

    volatile uint32_t PIO_MDER;      /**< \brief (Pio Offset: 0x0050) Multi-state Enable Register */

    volatile uint32_t PIO_MDDR;      /**< \brief (Pio Offset: 0x0054) Multi-state Disable Register */

    volatile uint32_t PIO_MDSR;      /**< \brief (Pio Offset: 0x0058) Multi-state Status Register */

    volatile uint32_t Reserved4[1];

    volatile uint32_t PIO_PUDR;      /**< \brief (Pio Offset: 0x0060) Pull-up Disable Register */

    volatile uint32_t PIO_PUER;      /**< \brief (Pio Offset: 0x0064) Pull-up Enable Register */

    volatile uint32_t PIO_PUSR;      /**< \brief (Pio Offset: 0x0068) Pad Pull-up Status Register */

    volatile uint32_t Reserved5[1];

    volatile uint32_t PIO_ABCDSR[2]; /**< \brief (Pio Offset: 0x0070) Peripheral Select Register */

    volatile uint32_t Reserved6[2];

    volatile uint32_t PIO_IFSCDR;    /**< \brief (Pio Offset: 0x0080) Input Filter Slow Clock Disable Register */

    volatile uint32_t PIO_IFSCER;    /**< \brief (Pio Offset: 0x0084) Input Filter Slow Clock Enable Register */

    volatile uint32_t PIO_IFSCSR;    /**< \brief (Pio Offset: 0x0088) Input Filter Slow Clock Status Register */

    volatile uint32_t PIO_SCDR;      /**< \brief (Pio Offset: 0x008C) Slow Clock Divider Debouncing Register */

    volatile uint32_t PIO_PPDDR;     /**< \brief (Pio Offset: 0x0090) Pad Pull-down Disable Register */

    volatile uint32_t PIO_PPDER;     /**< \brief (Pio Offset: 0x0094) Pad Pull-down Enable Register */

    volatile uint32_t PIO_PPDSR;     /**< \brief (Pio Offset: 0x0098) Pad Pull-down Status Register */

    volatile uint32_t Reserved7[1];

    volatile uint32_t PIO_OWER;      /**< \brief (Pio Offset: 0x00A0) Output Write Enable */

    volatile uint32_t PIO_OWDR;      /**< \brief (Pio Offset: 0x00A4) Output Write Disable */

    volatile uint32_t PIO_OWSR;      /**< \brief (Pio Offset: 0x00A8) Output Write Status Register */

    volatile uint32_t Reserved8[1];
    volatile uint32_t PIO_AIMER;     /**< \brief (Pio Offset: 0x00B0) Additional Interrupt Modes Enable Register */

    volatile uint32_t PIO_AIMDR;     /**< \brief (Pio Offset: 0x00B4) Additional Interrupt Modes Disables Register */

    volatile uint32_t PIO_AIMMR;     /**< \brief (Pio Offset: 0x00B8) Additional Interrupt Modes Mask Register */

    volatile uint32_t Reserved9[1];

    volatile uint32_t PIO_ESR;       /**< \brief (Pio Offset: 0x00C0) Edge Select Register */

    volatile uint32_t PIO_LSR;       /**< \brief (Pio Offset: 0x00C4) Level Select Register */

    volatile uint32_t PIO_ELSR;      /**< \brief (Pio Offset: 0x00C8) Edge/Level Status Register */

    volatile uint32_t Reserved10[1];

    volatile uint32_t PIO_FELLSR;    /**< \brief (Pio Offset: 0x00D0) Falling Edge/Low Level Select Register */

    volatile uint32_t PIO_REHLSR;    /**< \brief (Pio Offset: 0x00D4) Rising Edge/ High Level Select Register */

    volatile uint32_t PIO_FRLHSR;    /**< \brief (Pio Offset: 0x00D8) Fall/Rise - Low/High Status Register */

    volatile uint32_t Reserved11[1];

    volatile uint32_t PIO_LOCKSR;    /**< \brief (Pio Offset: 0x00E0) Lock Status */

    volatile uint32_t PIO_WPMR;      /**< \brief (Pio Offset: 0x00E4) Write Protect Mode Register */

    volatile uint32_t PIO_WPSR;      /**< \brief (Pio Offset: 0x00E8) Write Protect Status Register */

    volatile uint32_t Reserved12[5];

    volatile uint32_t PIO_SCHMITT;   /**< \brief (Pio Offset: 0x0100) Schmitt Trigger Register */

    volatile uint32_t Reserved13[3];

    volatile uint32_t PIO_DELAYR;    /**< \brief (Pio Offset: 0x0110) IO Delay Register */

    volatile uint32_t PIO_DRIVER1;   /**< \brief (Pio Offset: 0x0114) I/O Drive Register 1 */

    volatile uint32_t PIO_DRIVER2;   /**< \brief (Pio Offset: 0x0118) I/O Drive Register 2 */
};

// --//


enum class AT91_Gpio_PeripheralSelection : uint8_t {
    None = 0,
    PeripheralA = 1,
    PeripheralB = 2,
    PeripheralC = 3,
    PeripheralD = 4
};

enum class AT91_Gpio_Direction : uint8_t {
    Input = 0,
    Output = 1
};

enum class AT91_Gpio_ResistorMode : uint8_t {
    Inactive = 0,
    PullUp = 1,
    PullDown = 2,
};

enum class AT91_Gpio_MultiDriver : uint8_t {
    Disable = 0,
    Enable = 1
};

enum class AT91_Gpio_Filter : uint8_t {
    Disable = 0,
    Enable = 1
};

enum class AT91_Gpio_FilterSlowClock : uint8_t {
    Disable = 0,
    Enable = 1
};

enum class AT91_Gpio_Schmitt : uint8_t {
    Enable = 0,
    Disable = 1
};

enum class AT91_Gpio_DriveSpeed : uint8_t {
    High = 0,
    Medium = 1,
    Low = 2,
    Reserved = 3
};

struct AT91_Gpio_Pin {
    uint32_t number;
    AT91_Gpio_PeripheralSelection peripheralSelection;
};

struct AT91_Gpio_PinConfiguration {
    AT91_Gpio_Direction direction;
    AT91_Gpio_ResistorMode resistorMode;
    AT91_Gpio_MultiDriver multiDriver;
    AT91_Gpio_Filter filter;
    AT91_Gpio_FilterSlowClock filterSlowClock;
    AT91_Gpio_Schmitt schmitt;
    AT91_Gpio_DriveSpeed speed;
    AT91_Gpio_PeripheralSelection peripheralSelection;
    bool outputDirection;
    bool apply;
};

#define PIN(port, pin) ((CHARIZE(port) - 'A') * 32 + pin)
#define PIN_NONE 0xFFFFFFFF
#define PS(num) (CONCAT(AT91_Gpio_PeripheralSelection::Peripheral, num))
#define PS_NONE AT91_Gpio_PeripheralSelection::None

#define INIT(pinDirection, resistorMode, peripheralSelection, outputDirection, apply) { AT91_Gpio_Direction::pinDirection, AT91_Gpio_ResistorMode::resistorMode, AT91_Gpio_MultiDriver::Disable, AT91_Gpio_Filter::Disable, AT91_Gpio_FilterSlowClock::Disable, AT91_Gpio_Schmitt::Disable,  AT91_Gpio_DriveSpeed::High, AT91_Gpio_PeripheralSelection::peripheralSelection, outputDirection, apply }
#define ALTFUN(direction, resistorMode, peripheralSelection) { AT91_Gpio_Direction::direction, AT91_Gpio_ResistorMode::resistorMode, AT91_Gpio_MultiDriver::Disable, AT91_Gpio_Filter::Disable, AT91_Gpio_FilterSlowClock::Disable, AT91_Gpio_Schmitt::Disable,  AT91_Gpio_DriveSpeed::High, AT91_Gpio_PeripheralSelection::peripheralSelection, true }
#define INPUT(resistorMode) { AT91_Gpio_Direction::Input, AT91_Gpio_ResistorMode::resistorMode, AT91_Gpio_MultiDriver::Disable, AT91_Gpio_Filter::Disable, AT91_Gpio_FilterSlowClock::Disable, AT91_Gpio_Schmitt::Disable, AT91_Gpio_DriveSpeed::High, AT91_Gpio_PeripheralSelection::None, true }
#define DEFAULT() INIT(Input, Inactive, None, false, true)
#define NO_INIT() INIT(Input, Inactive, None, false, false)

void AT91_Gpio_Reset();
void AT91_Gpio_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* AT91_Gpio_GetRequiredApi();
TinyCLR_Result AT91_Gpio_Acquire(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result AT91_Gpio_Release(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result AT91_Gpio_EnableAlternatePin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor, uint32_t alternate);
TinyCLR_Result AT91_Gpio_Read(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue& value);
TinyCLR_Result AT91_Gpio_Write(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinValue value);
TinyCLR_Result AT91_Gpio_SetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin, uint64_t debounceTicks);
TinyCLR_Result AT91_Gpio_SetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Result AT91_Gpio_OpenPin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
bool AT91_Gpio_IsDriveModeSupported(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinDriveMode mode);
TinyCLR_Gpio_PinDriveMode AT91_Gpio_GetDriveMode(const TinyCLR_Gpio_Controller* self, uint32_t pin);
uint64_t AT91_Gpio_GetDebounceTimeout(const TinyCLR_Gpio_Controller* self, uint32_t pin);
uint32_t AT91_Gpio_GetPinCount(const TinyCLR_Gpio_Controller* self);
TinyCLR_Result AT91_Gpio_SetPinChangedHandler(const TinyCLR_Gpio_Controller* self, uint32_t pin, TinyCLR_Gpio_PinChangeEdge edge, TinyCLR_Gpio_PinChangedHandler handler);
TinyCLR_Result AT91_Gpio_ClosePin(const TinyCLR_Gpio_Controller* self, uint32_t pin);
void AT91_GpioInternal_EnableOutputPin(int32_t pin, bool initialState);
void AT91_GpioInternal_EnableInputPin(int32_t pin, TinyCLR_Gpio_PinDriveMode resistor);
bool AT91_GpioInternal_OpenPin(int32_t pin);
bool AT91_GpioInternal_ClosePin(int32_t pin);
bool AT91_GpioInternal_ReadPin(int32_t pin);
void AT91_GpioInternal_WritePin(int32_t pin, bool value);
bool AT91_GpioInternal_ConfigurePin(int32_t pin, AT91_Gpio_Direction pinDir, AT91_Gpio_PeripheralSelection peripheralSelection, AT91_Gpio_ResistorMode resistorMode);
bool AT91_GpioInternal_ConfigurePin(int32_t pin, AT91_Gpio_Direction pinDir, AT91_Gpio_PeripheralSelection peripheralSelection, AT91_Gpio_ResistorMode resistorMode, AT91_Gpio_MultiDriver multiDrive, AT91_Gpio_Filter filter, AT91_Gpio_FilterSlowClock filterSlowClock, AT91_Gpio_Schmitt schmitt, AT91_Gpio_DriveSpeed driveSpeed);
bool AT91_GpioInternal_OpenMultiPins(const AT91_Gpio_Pin* pins, size_t count);

// ADC
void AT91_Adc_AddApi(const TinyCLR_Api_Manager* apiManager);
void AT91_Adc_Reset();
int32_t AT91_Adc_GetPin(int32_t channel);
AT91_Gpio_PeripheralSelection AT91_Adc_GetPeripheralSelection(int32_t channel);
TinyCLR_Result AT91_Adc_Acquire(const TinyCLR_Adc_Controller* self);
TinyCLR_Result AT91_Adc_Release(const TinyCLR_Adc_Controller* self);
TinyCLR_Result AT91_Adc_OpenChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Adc_CloseChannel(const TinyCLR_Adc_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Adc_ReadChannel(const TinyCLR_Adc_Controller* self, uint32_t channel, int32_t& value);
uint32_t AT91_Adc_GetChannelCount(const TinyCLR_Adc_Controller* self);
uint32_t AT91_Adc_GetResolutionInBits(const TinyCLR_Adc_Controller* self);
int32_t AT91_Adc_GetMinValue(const TinyCLR_Adc_Controller* self);
int32_t AT91_Adc_GetMaxValue(const TinyCLR_Adc_Controller* self);
TinyCLR_Adc_ChannelMode AT91_Adc_GetChannelMode(const TinyCLR_Adc_Controller* self);
TinyCLR_Result AT91_Adc_SetChannelMode(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);
bool AT91_Adc_IsChannelModeSupported(const TinyCLR_Adc_Controller* self, TinyCLR_Adc_ChannelMode mode);

// CAN
void AT91_Can_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result AT91_Can_Acquire(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_Release(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_SoftReset(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_WriteMessage(const TinyCLR_Can_Controller* self, const TinyCLR_Can_Message* messages, size_t& length);
TinyCLR_Result AT91_Can_ReadMessage(const TinyCLR_Can_Controller* self, TinyCLR_Can_Message* messages, size_t& length);
TinyCLR_Result AT91_Can_SetBitTiming(const TinyCLR_Can_Controller* self, const TinyCLR_Can_BitTiming* timing);
size_t AT91_Can_GetMessagesToRead(const TinyCLR_Can_Controller* self);
size_t AT91_Can_GetMessagesToWrite(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_SetMessageReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_MessageReceivedHandler handler);
TinyCLR_Result AT91_Can_SetErrorReceivedHandler(const TinyCLR_Can_Controller* self, TinyCLR_Can_ErrorReceivedHandler handler);
TinyCLR_Result AT91_Can_SetExplicitFilters(const TinyCLR_Can_Controller* self, const uint32_t* filters, size_t count);
TinyCLR_Result AT91_Can_SetGroupFilters(const TinyCLR_Can_Controller* self, const uint32_t* lowerBounds, const uint32_t* upperBounds, size_t count);
TinyCLR_Result AT91_Can_ClearReadBuffer(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_IsWritingAllowed(const TinyCLR_Can_Controller* self, bool& allowed);
size_t AT91_Can_GetWriteErrorCount(const TinyCLR_Can_Controller* self);
size_t AT91_Can_GetReadErrorCount(const TinyCLR_Can_Controller* self);
uint32_t AT91_Can_GetSourceClock(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_SetReadBufferSize(const TinyCLR_Can_Controller* self, size_t size);
size_t AT91_Can_GetReadBufferSize(const TinyCLR_Can_Controller* self);
size_t AT91_Can_GetWriteBufferSize(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_SetWriteBufferSize(const TinyCLR_Can_Controller* self, size_t size);
TinyCLR_Result AT91_Can_Enable(const TinyCLR_Can_Controller* self);
TinyCLR_Result AT91_Can_Disable(const TinyCLR_Can_Controller* self);
bool AT91_Can_CanWriteMessage(const TinyCLR_Can_Controller* self);
bool AT91_Can_CanReadMessage(const TinyCLR_Can_Controller* self);
void AT91_Can_Reset();

// MCI
typedef struct _AT91S_MCI {
    uint32_t     MCI_CR;
    uint32_t     MCI_MR;
    uint32_t     MCI_DTOR;
    uint32_t     MCI_SDCR;
    uint32_t     MCI_ARGR;
    uint32_t     MCI_CMDR;
    uint32_t     MCI_BLKR;
    uint32_t     MCI_CSTOR;
    uint32_t     MCI_RSPR[4];
    uint32_t     MCI_RDR;
    uint32_t     MCI_TDR;
    uint32_t     Reserved1[2];
    uint32_t     MCI_SR;
    uint32_t     MCI_IER;
    uint32_t     MCI_IDR;
    uint32_t     MCI_IMR;
    uint32_t     MCI_DMA;
    uint32_t     MCI_CFG;
    uint32_t     Reserved2[35];
    uint32_t     MCI_WPMR;
    uint32_t     MCI_WPSR;
    uint32_t     Reserved3[69];
    uint32_t     MCI_FIFO[256];

} AT91S_MCI, *AT91PS_MCI;

//DAC
void AT91_Dac_AddApi(const TinyCLR_Api_Manager* apiManager);
void AT91_Dac_Reset();
TinyCLR_Result AT91_Dac_Acquire(const TinyCLR_Dac_Controller* self);
TinyCLR_Result AT91_Dac_Release(const TinyCLR_Dac_Controller* self);
TinyCLR_Result AT91_Dac_OpenChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Dac_CloseChannel(const TinyCLR_Dac_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Dac_WriteValue(const TinyCLR_Dac_Controller* self, uint32_t channel, int32_t value);
uint32_t AT91_Dac_GetChannelCount(const TinyCLR_Dac_Controller* self);
uint32_t AT91_Dac_GetResolutionInBits(const TinyCLR_Dac_Controller* self);
int32_t AT91_Dac_GetMinValue(const TinyCLR_Dac_Controller* self);
int32_t AT91_Dac_GetMaxValue(const TinyCLR_Dac_Controller* self);

// PWM
struct PwmState {
    int32_t controllerIndex;
    uint32_t *channelModeReg;
    uint32_t *dutyCycleReg;
    uint32_t *channelUpdateReg;

    AT91_Gpio_Pin gpioPin[MAX_PWM_PER_CONTROLLER];

    TinyCLR_Pwm_PulsePolarity invert[MAX_PWM_PER_CONTROLLER];
    bool isOpened[MAX_PWM_PER_CONTROLLER];

    double frequency;
    double dutyCycle[MAX_PWM_PER_CONTROLLER];

    uint16_t initializeCount;
};
void AT91_Pwm_AddApi(const TinyCLR_Api_Manager* apiManager);
void AT91_Pwm_Reset();
void AT91_Pwm_ResetController(int32_t controller);
AT91_Gpio_Pin AT91_Pwm_GetPins(int32_t controller, int32_t channel);
TinyCLR_Result AT91_Pwm_Acquire(const TinyCLR_Pwm_Controller* self);
TinyCLR_Result AT91_Pwm_Release(const TinyCLR_Pwm_Controller* self);
uint32_t AT91_Pwm_GetGpioPinForChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Pwm_SetDesiredFrequency(const TinyCLR_Pwm_Controller* self, double& frequency);
TinyCLR_Result AT91_Pwm_OpenChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Pwm_CloseChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Pwm_EnableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Pwm_DisableChannel(const TinyCLR_Pwm_Controller* self, uint32_t channel);
TinyCLR_Result AT91_Pwm_SetPulseParameters(const TinyCLR_Pwm_Controller* self, uint32_t channel, double dutyCycle, TinyCLR_Pwm_PulsePolarity polarity);
double AT91_Pwm_GetMinFrequency(const TinyCLR_Pwm_Controller* self);
double AT91_Pwm_GetMaxFrequency(const TinyCLR_Pwm_Controller* self);
double AT91_Pwm_GetActualFrequency(const TinyCLR_Pwm_Controller* self);
uint32_t AT91_Pwm_GetChannelCount(const TinyCLR_Pwm_Controller* self);

//RTC
void AT91_Rtc_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result AT91_Rtc_Acquire(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result AT91_Rtc_Release(const TinyCLR_Rtc_Controller* self);
TinyCLR_Result AT91_Rtc_IsValid(const TinyCLR_Rtc_Controller* self, bool& value);
TinyCLR_Result AT91_Rtc_GetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime& value);
TinyCLR_Result AT91_Rtc_SetTime(const TinyCLR_Rtc_Controller* self, TinyCLR_Rtc_DateTime value);

////////////////////////////////////////////////////////////////////////////////
//SD
////////////////////////////////////////////////////////////////////////////////
void AT91_SdCard_AddApi(const TinyCLR_Api_Manager* apiManager);

TinyCLR_Result AT91_SdCard_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result AT91_SdCard_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result AT91_SdCard_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result AT91_SdCard_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result AT91_SdCard_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result AT91_SdCard_Erases(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result AT91_SdCard_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result AT91_SdCard_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result AT91_SdCard_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result AT91_SdCard_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result AT91_SdCard_Close(const TinyCLR_Storage_Controller* self);

TinyCLR_Result AT91_SdCard_Reset();

//SPI
//////////////////////////////////////////////////////////////////////////////
// AT91_SPI
//

struct AT91_SPI {
    static const uint32_t c_Base_1 = AT91C_BASE_SPI0;
    static const uint32_t c_Base_2 = AT91C_BASE_SPI1;


    static const uint32_t c_MAX_SPI = 2;

    volatile uint32_t SPI_CR;         // Control Register
    static const    uint32_t SPI_CR_ENABLE_SPI = (0x1 << 0); // enable SPI
    static const    uint32_t SPI_CR_DISABLE_SPI = (0x1 << 1); // disable SPI
    static const    uint32_t SPI_CR_SW_RESET = (0x1 << 7); // Software reset SPI

    volatile uint32_t SPI_MR;         // Mode Register
    static const    uint32_t SPI_MR_MSTR = (0x1 << 0); // 1--Master mode\0--Slave mode
    static const    uint32_t SPI_MR_PS = (0x1 << 1); // peripheral select 1--varable\0--fixed
    static const    uint32_t SPI_MR_MODFDIS = (0x1 << 4); // peripheral select 1--varable\0--fixed
    static const    uint32_t SPI_MR_CS0 = (0x0 << 16);
    static const    uint32_t SPI_MR_CS1 = (0x1 << 16);
    static const    uint32_t SPI_MR_CS2 = (0x3 << 16);
    static const    uint32_t SPI_MR_CS3 = (0x7 << 16);

    volatile uint32_t SPI_RDR;        // Receive Data Register

    volatile uint32_t SPI_TDR;        // Transmit Data Register

    volatile uint32_t SPI_SR;         // Status Register
    static const    uint32_t SPI_SR_RDRF = (0x1 << 0); // RDR full
    static const    uint32_t SPI_SR_TDRE = (0x1 << 1); // TDR empty
    static const    uint32_t SPI_SR_RXBUFF = (0x1 << 6); // receive buffer full
    static const    uint32_t SPI_SR_TXBUFE = (0x1 << 7); // transmit buffer empty
    static const    uint32_t SPI_SR_NSSR = (0x1 << 8); // Slave mode control
    static const    uint32_t SPI_SR_TXEMPTY = (0x1 << 9); // transmit register and internal shifters are empty
    static const    uint32_t SPI_SR_SPIENS = (0x1 << 16); // SPI enable status

    volatile uint32_t SPI_IER;        // Interrupt Enable Register
    volatile uint32_t SPI_IDR;        // Interrupt Disable Register
    volatile uint32_t SPI_IMR;        // Interrupt MaskRegister
    volatile uint32_t Reserved1[4];

    volatile uint32_t SPI_CSR0;       // Chip Select Register 0
    static const    uint32_t SPI_CSR_CPOL = (0x1 << 0); // clock polarity
    static const    uint32_t SPI_CSR_NCPHA = (0x1 << 1); // clock phase
    static const    uint32_t SPI_CSR_BITS_MASK = (0xF << 4); // bits per transfer
    static const    uint32_t SPI_CSR_8BITS = (0x0 << 4);
    static const    uint32_t SPI_CSR_16BITS = (0x8 << 4);
    static const    uint32_t SPI_CSR_SCBR_MASK = (0xFF << 8); // serial clock baud rate
    static const    uint32_t SPI_CSR_SCBR_SHIFT = (0x8);     // serial clock baud rate
    static const    uint32_t SPI_CSR_DLYBS_MASK = (0xFF << 16); // delay before SPCK
    static const    uint32_t SPI_CSR_DLYBS_SHIFT = (16);     // delay before SPCK
    static const    uint32_t SPI_CSR_DLYBCT_MASK = ((uint32_t)0xFF << 24); // delay between transfer
    static const    uint32_t SPI_CSR_DLYBCT_SHIFT = (24);     // delay between transfer

    volatile uint32_t SPI_CSR1;       // Chip Select Register 1
    volatile uint32_t SPI_CSR2;       // Chip Select Register 2
    volatile uint32_t SPI_CSR3;       // Chip Select Register 3

    __inline static uint32_t ConvertClockRateToDivisor(uint32_t clockKHz) {
        uint32_t mckKHz = AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 1000;
        uint32_t divisor = mckKHz / clockKHz;

        while ((divisor > 0) && ((mckKHz / divisor) > clockKHz)) {
            divisor++;

            if (divisor > 0xFF)
                break;
        }

        if (divisor > 0xFF)
            divisor = 0xFF;

        return divisor;
    }

    // no data in TX FIFO
    __inline bool TransmitBufferEmpty(AT91_SPI & _SPI) {
        return (_SPI.SPI_SR & SPI_SR_TXEMPTY) != 0;
    }
};

void AT91_Spi_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* AT91_Spi_GetRequiredApi();
void AT91_Spi_Reset();
bool AT91_Spi_Transaction_Start(int32_t controller);
bool AT91_Spi_Transaction_Stop(int32_t controller);
bool AT91_Spi_Transaction_nWrite8_nRead8(int32_t controller);
TinyCLR_Result AT91_Spi_Acquire(const TinyCLR_Spi_Controller* self);
TinyCLR_Result AT91_Spi_Release(const TinyCLR_Spi_Controller* self);
TinyCLR_Result AT91_Spi_SetActiveSettings(const TinyCLR_Spi_Controller* self, const TinyCLR_Spi_Settings* settings);
TinyCLR_Result AT91_Spi_Read(const TinyCLR_Spi_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Spi_Write(const TinyCLR_Spi_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Spi_WriteRead(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
TinyCLR_Result AT91_Spi_TransferSequential(const TinyCLR_Spi_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool deselectAfter);
uint32_t AT91_Spi_GetChipSelectLineCount(const TinyCLR_Spi_Controller* self);
uint32_t AT91_Spi_GetMinClockFrequency(const TinyCLR_Spi_Controller* self);
uint32_t AT91_Spi_GetMaxClockFrequency(const TinyCLR_Spi_Controller* self);
TinyCLR_Result AT91_Spi_GetSupportedDataBitLengths(const TinyCLR_Spi_Controller* self, uint32_t* dataBitLengths, size_t& dataBitLengthsCount);

//Uart
//////////////////////////////////////////////////////////////////////////////
// AT91_USART
//
struct AT91_USART {

    static const uint32_t c_Base_dbg = AT91C_BASE_DBGU;
    static const uint32_t c_Base_usart = AT91C_BASE_USART0;
    static const uint32_t c_Base_uart = AT91C_BASE_UART0;
    static const uint32_t c_Base_offset = 0x4000;

    static const uint32_t c_MAX_BAUDRATE = ((AT91_SYSTEM_PERIPHERAL_CLOCK_HZ * 10) / 16);
    static const uint32_t c_MIN_BAUDRATE = 0;

    volatile uint32_t US_CR;          // Control Register
    static const    uint32_t US_RSTRX = ((unsigned int)0x1 << 2); // (DBGU) Reset Receiver
    static const    uint32_t US_RSTTX = ((unsigned int)0x1 << 3); //  (DBGU) Reset Transmitter
    static const    uint32_t US_RXEN = ((unsigned int)0x1 << 4); //  (DBGU) Receiver Enable
    static const    uint32_t US_RXDIS = ((unsigned int)0x1 << 5); //  (DBGU) Receiver Disable
    static const    uint32_t US_TXEN = ((unsigned int)0x1 << 6); //  (DBGU) Transmitter Enable
    static const    uint32_t US_TXDIS = ((unsigned int)0x1 << 7); //  (DBGU) Transmitter Disable
    static const    uint32_t US_RSTSTA = ((unsigned int)0x1 << 8); //  (DBGU) Reset Status Bits
    static const    uint32_t US_STTBRK = ((unsigned int)0x1 << 9); //  (USART) Start Break
    static const    uint32_t US_STPBRK = ((unsigned int)0x1 << 10); //  (USART) Stop Break
    static const    uint32_t US_STTTO = ((unsigned int)0x1 << 11); //  (USART) Start Time-out
    static const    uint32_t US_SENDA = ((unsigned int)0x1 << 12); //  (USART) Send Address
    static const    uint32_t US_RSTIT = ((unsigned int)0x1 << 13); //  (USART) Reset Iterations
    static const    uint32_t US_RSTNACK = ((unsigned int)0x1 << 14); //  (USART) Reset Non Acknowledge
    static const    uint32_t US_RETTO = ((unsigned int)0x1 << 15); //  (USART) Rearm Time-out
    static const    uint32_t US_DTREN = ((unsigned int)0x1 << 16); //  (USART) Data Terminal ready Enable
    static const    uint32_t US_DTRDIS = ((unsigned int)0x1 << 17); //  (USART) Data Terminal ready Disable
    static const    uint32_t US_RTSEN = ((unsigned int)0x1 << 18); //  (USART) Request to Send enable
    static const    uint32_t US_RTSDIS = ((unsigned int)0x1 << 19); //  (USART) Request to Send Disable

    volatile uint32_t US_MR;          // Mode Register
    static const    uint32_t US_USMODE = ((unsigned int)0xF << 0); //  (USART) Usart mode
    static const    uint32_t     US_USMODE_NORMAL = ((unsigned int)0x0); //  (USART) Normal
    static const    uint32_t     US_USMODE_RS485 = ((unsigned int)0x1); //  (USART) RS485
    static const    uint32_t     US_USMODE_HWHSH = ((unsigned int)0x2); //  (USART) Hardware Handshaking
    static const    uint32_t     US_USMODE_MODEM = ((unsigned int)0x3); //  (USART) Modem
    static const    uint32_t     US_USMODE_ISO7816_0 = ((unsigned int)0x4); //  (USART) ISO7816 protocol: T = 0
    static const    uint32_t     US_USMODE_ISO7816_1 = ((unsigned int)0x6); //  (USART) ISO7816 protocol: T = 1
    static const    uint32_t     US_USMODE_IRDA = ((unsigned int)0x8); //  (USART) IrDA
    static const    uint32_t     US_USMODE_SWHSH = ((unsigned int)0xC); //  (USART) Software Handshaking
    static const    uint32_t US_CLKS = ((unsigned int)0x3 << 4); //  (USART) Clock Selection = (Baud Rate generator Input Clock
    static const    uint32_t     US_CLKS_CLOCK = ((unsigned int)0x0 << 4); //  (USART) Clock
    static const    uint32_t     US_CLKS_FDIV1 = ((unsigned int)0x1 << 4); //  (USART) fdiv1
    static const    uint32_t     US_CLKS_SLOW = ((unsigned int)0x2 << 4); //  (USART) slow_clock = (ARM);
    static const    uint32_t     US_CLKS_EXT = ((unsigned int)0x3 << 4); //  (USART) External = (SCK);
    static const    uint32_t US_CHRL = ((unsigned int)0x3 << 6); //  (USART) Clock Selection = (Baud Rate generator Input Clock
    static const    uint32_t     US_CHRL_5_BITS = ((unsigned int)0x0 << 6); //  (USART) Character Length: 5 bits
    static const    uint32_t     US_CHRL_6_BITS = ((unsigned int)0x1 << 6); //  (USART) Character Length: 6 bits
    static const    uint32_t     US_CHRL_7_BITS = ((unsigned int)0x2 << 6); //  (USART) Character Length: 7 bits
    static const    uint32_t     US_CHRL_8_BITS = ((unsigned int)0x3 << 6); //  (USART) Character Length: 8 bits
    static const    uint32_t US_SYNC = ((unsigned int)0x1 << 8); //  (USART) Synchronous Mode Select
    static const    uint32_t US_NBSTOP = ((unsigned int)0x3 << 12); //  (USART) Number of Stop bits
    static const    uint32_t     US_NBSTOP_1_BIT = ((unsigned int)0x0 << 12); //  (USART) 1 stop bit
    static const    uint32_t     US_NBSTOP_15_BIT = ((unsigned int)0x1 << 12); //  (USART) Asynchronous = (SYNC=0); 2 stop bits Synchronous = (SYNC=1); 2 stop bits
    static const    uint32_t     US_NBSTOP_2_BIT = ((unsigned int)0x2 << 12); //  (USART) 2 stop bits
    static const    uint32_t US_MSBF = ((unsigned int)0x1 << 16); //  (USART) Bit Order
    static const    uint32_t US_MODE9 = ((unsigned int)0x1 << 17); //  (USART) 9-bit Character length
    static const    uint32_t US_CKLO = ((unsigned int)0x1 << 18); //  (USART) Clock Output Select
    static const    uint32_t US_OVER = ((unsigned int)0x1 << 19); //  (USART) Over Sampling Mode
    static const    uint32_t US_INACK = ((unsigned int)0x1 << 20); //  (USART) Inhibit Non Acknowledge
    static const    uint32_t US_DSNACK = ((unsigned int)0x1 << 21); //  (USART) Disable Successive NACK
    static const    uint32_t US_MAX_ITER = ((unsigned int)0x1 << 24); //  (USART) Number of Repetitions
    static const    uint32_t US_FILTER = ((unsigned int)0x1 << 28); //  (USART) Receive Line Filter
    static const    uint32_t US_PAR = (0x7 << 9); // = (DBGU); Parity type
    static const    uint32_t     US_PAR_EVEN = (0x0 << 9); // = (DBGU); Even Parity
    static const    uint32_t     US_PAR_ODD = (0x1 << 9); // = (DBGU); Odd Parity
    static const    uint32_t     US_PAR_SPACE = (0x2 << 9); // = (DBGU); Parity forced to 0 = (Space);
    static const    uint32_t     US_PAR_MARK = (0x3 << 9); // = (DBGU); Parity forced to 1 = (Mark);
    static const    uint32_t     US_PAR_NONE = (0x4 << 9); // = (DBGU); No Parity
    static const    uint32_t     US_PAR_MULTI_DROP = (0x6 << 9); // = (DBGU); Multi-drop mode
    static const    uint32_t US_CHMODE = (0x3 << 14); // = (DBGU); Channel Mode
    static const    uint32_t     US_CHMODE_NORMAL = (0x0 << 14); // = (DBGU); Normal Mode: The USART channel operates as an RX/TX USART.
    static const    uint32_t     US_CHMODE_AUTO = (0x1 << 14); // = (DBGU); Automatic Echo: Receiver Data Input is connected to the TXD pin.
    static const    uint32_t     US_CHMODE_LOCAL = (0x2 << 14); // = (DBGU); Local Loopback: Transmitter Output Signal is connected to Receiver Input Signal.
    static const    uint32_t     US_CHMODE_REMOTE = (0x3 << 14); // = (DBGU); Remote Loopback: RXD pin is internally connected to TXD pin.

    volatile uint32_t US_IER;         // Interrupt Enable Register
    static const    uint32_t US_RXBRK = ((unsigned int)0x1 << 2); //  (USART) Break Received/End of Break
    static const    uint32_t US_TIMEOUT = ((unsigned int)0x1 << 8); //  (USART) Receiver Time-out
    static const    uint32_t US_ITERATION = ((unsigned int)0x1 << 10); //  (USART) Max number of Repetitions Reached
    static const    uint32_t US_NACK = ((unsigned int)0x1 << 13); //  (USART) Non Acknowledge
    static const    uint32_t US_RIIC = ((unsigned int)0x1 << 16); //  (USART) Ring INdicator Input Change Flag
    static const    uint32_t US_DSRIC = ((unsigned int)0x1 << 17); //  (USART) Data Set Ready Input Change Flag
    static const    uint32_t US_DCDIC = ((unsigned int)0x1 << 18); //  (USART) Data Carrier Flag
    static const    uint32_t US_CTSIC = ((unsigned int)0x1 << 19); //  (USART) Clear To Send Input Change Flag
    static const    uint32_t US_RXRDY = (0x1 << 0); // = (DBGU); RXRDY Interrupt
    static const    uint32_t US_TXRDY = (0x1 << 1); // = (DBGU); TXRDY Interrupt
    static const    uint32_t US_ENDRX = (0x1 << 3); // = (DBGU); End of Receive Transfer Interrupt
    static const    uint32_t US_ENDTX = (0x1 << 4); // = (DBGU); End of Transmit Interrupt
    static const    uint32_t US_OVRE = (0x1 << 5); // = (DBGU); Overrun Interrupt
    static const    uint32_t US_FRAME = (0x1 << 6); // = (DBGU); Framing Error Interrupt
    static const    uint32_t US_PARE = (0x1 << 7); // = (DBGU); Parity Error Interrupt
    static const    uint32_t US_TXEMPTY = (0x1 << 9); // = (DBGU); TXEMPTY Interrupt
    static const    uint32_t US_TXBUFE = (0x1 << 11); // = (DBGU); TXBUFE Interrupt
    static const    uint32_t US_RXBUFF = (0x1 << 12); // = (DBGU); RXBUFF Interrupt
    static const    uint32_t US_COMM_TX = (0x1 << 30); // = (DBGU); COMM_TX Interrupt
    static const    uint32_t US_COMM_RX = ((uint32_t)0x1 << 31); // = (DBGU); COMM_RX Interrupt

    volatile uint32_t US_IDR;         // Interrupt Disable Register

    volatile uint32_t US_IMR;         // Interrupt Mask Register

    volatile uint32_t US_CSR;         // Channel Status Register
    static const    uint32_t US_RI = ((unsigned int)0x1 << 20); //  (USART) Image of RI Input
    static const    uint32_t US_DSR = ((unsigned int)0x1 << 21); //  (USART) Image of DSR Input
    static const    uint32_t US_DCD = ((unsigned int)0x1 << 22); //  (USART) Image of DCD Input
    static const    uint32_t US_CTS = ((unsigned int)0x1 << 23); //  (USART) Image of CTS Input

    volatile uint32_t US_RHR;         // Receiver Holding Register

    volatile uint32_t US_THR;         // Transmitter Holding Register

    volatile uint32_t US_BRGR;        // Baud Rate Generator Register

    volatile uint32_t US_RTOR;        // Receiver Time-out Register

    volatile uint32_t US_TTGR;        // Transmitter Time-guard Register

    volatile uint32_t Reserved0[5];   //

    volatile uint32_t US_FIDI;        // FI_DI_Ratio Register

    volatile uint32_t US_NER;         // Nb Errors Register

    volatile uint32_t Reserved1[1];   //

    volatile uint32_t US_IF;          // IRDA_FILTER Register

#if defined(PLATFORM_ARM_SAM9RL64_ANY)

    volatile uint32_t US_MAN;          // Manchester Encoder Decoder Register

    volatile uint32_t Reserved2[43];

#else
    volatile uint32_t Reserved2[44];

#endif

    volatile uint32_t US_RPR;         // Receive Pointer Register

    volatile uint32_t US_RCR;         // Receive Counter Register

    volatile uint32_t US_TPR;         // Transmit Pointer Register

    volatile uint32_t US_TCR;         // Transmit Counter Register

    volatile uint32_t US_RNPR;        // Receive Next Pointer Register

    volatile uint32_t US_RNCR;        // Receive Next Counter Register

    volatile uint32_t US_TNPR;        // Transmit Next Pointer Register

    volatile uint32_t US_TNCR;        // Transmit Next Counter Register

    volatile uint32_t US_PTCR;        // PDC Transfer Control Register

    volatile uint32_t US_PTSR;        // PDC Transfer Status Register
};

void AT91_Uart_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* AT91_Uart_GetRequiredApi();
void AT91_Uart_Reset();
int32_t AT91_Uart_GetTxPin(int32_t controller);
int32_t AT91_Uart_GetRxPin(int32_t controller);
int32_t AT91_Uart_GetRtsPin(int32_t controller);
int32_t AT91_Uart_GetCtsPin(int32_t controller);
AT91_Gpio_PeripheralSelection AT91_Uart_GetTxAlternateFunction(int32_t controller);
AT91_Gpio_PeripheralSelection AT91_Uart_GetRxAlternateFunction(int32_t controller);
AT91_Gpio_PeripheralSelection AT91_Uart_GetRtsAlternateFunction(int32_t controller);
AT91_Gpio_PeripheralSelection AT91_Uart_GetCtsAlternateFunction(int32_t controller);
bool AT91_Uart_CanSend(int controller);
void AT91_Uart_TxBufferEmptyInterruptEnable(int controller, bool enable);
void AT91_Uart_RxBufferFullInterruptEnable(int controller, bool enable);

TinyCLR_Result AT91_Uart_Acquire(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_Release(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_Enable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_Disable(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_SetActiveSettings(const TinyCLR_Uart_Controller* self, const TinyCLR_Uart_Settings* settings);
TinyCLR_Result AT91_Uart_Flush(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_Read(const TinyCLR_Uart_Controller* self, uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Uart_Write(const TinyCLR_Uart_Controller* self, const uint8_t* buffer, size_t& length);
TinyCLR_Result AT91_Uart_SetErrorReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ErrorReceivedHandler handler);
TinyCLR_Result AT91_Uart_SetDataReceivedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_DataReceivedHandler handler);
TinyCLR_Result AT91_Uart_GetClearToSendState(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result AT91_Uart_SetClearToSendChangedHandler(const TinyCLR_Uart_Controller* self, TinyCLR_Uart_ClearToSendChangedHandler handler);
TinyCLR_Result AT91_Uart_GetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool& state);
TinyCLR_Result AT91_Uart_SetIsRequestToSendEnabled(const TinyCLR_Uart_Controller* self, bool state);
size_t AT91_Uart_GetReadBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_SetReadBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t AT91_Uart_GetWriteBufferSize(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_SetWriteBufferSize(const TinyCLR_Uart_Controller* self, size_t size);
size_t AT91_Uart_GetBytesToRead(const TinyCLR_Uart_Controller* self);
size_t AT91_Uart_GetBytesToWrite(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_ClearReadBuffer(const TinyCLR_Uart_Controller* self);
TinyCLR_Result AT91_Uart_ClearWriteBuffer(const TinyCLR_Uart_Controller* self);

//Deployment
void AT91_Deployment_AddApi(const TinyCLR_Api_Manager* apiManager);
void AT91_Deployment_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
TinyCLR_Result AT91_Deployment_Acquire(const TinyCLR_Storage_Controller* self);
TinyCLR_Result AT91_Deployment_Release(const TinyCLR_Storage_Controller* self);
TinyCLR_Result AT91_Deployment_Read(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint8_t* data, uint64_t timeout);
TinyCLR_Result AT91_Deployment_Write(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, const uint8_t* data, uint64_t timeout);
TinyCLR_Result AT91_Deployment_Erase(const TinyCLR_Storage_Controller* self, uint64_t address, size_t& count, uint64_t timeout);
TinyCLR_Result AT91_Deployment_IsErased(const TinyCLR_Storage_Controller* self, uint64_t address, size_t count, bool& erased);
TinyCLR_Result AT91_Deployment_GetBytesPerSector(const TinyCLR_Storage_Controller* self, uint32_t address, int32_t& size);
TinyCLR_Result AT91_Deployment_GetDescriptor(const TinyCLR_Storage_Controller* self, const TinyCLR_Storage_Descriptor*& descriptor);
TinyCLR_Result AT91_Deployment_IsPresent(const TinyCLR_Storage_Controller* self, bool& present);
TinyCLR_Result AT91_Deployment_SetPresenceChangedHandler(const TinyCLR_Storage_Controller* self, TinyCLR_Storage_PresenceChangedHandler handler);
TinyCLR_Result AT91_Deployment_Open(const TinyCLR_Storage_Controller* self);
TinyCLR_Result AT91_Deployment_Close(const TinyCLR_Storage_Controller* self);

// Interrupt
//////////////////////////////////////////////////////////////////////////////
// AT91_AIC
//
struct AT91_AIC {
    static const uint32_t c_Base = AT91C_BASE_AIC;

    volatile uint32_t AIC_SMR[32];    // Source Mode Register
    static const    uint32_t AIC_PRIOR = (0x7 << 0); // (AIC) Priority Level
    static const    uint32_t AIC_PRIOR_LOWEST = (0x0); // (AIC) Lowest priority level
    static const    uint32_t AIC_PRIOR_HIGHEST = (0x7); // (AIC) Highest priority level
    static const    uint32_t AIC_SRCTYPE = (0x3 << 5); // (AIC) Interrupt Source Type
    static const    uint32_t AIC_SRCTYPE_INT_HIGH_LEVEL = (0x0 << 5); // (AIC) Internal Sources Code Label High-level Sensitive
    static const    uint32_t AIC_SRCTYPE_EXT_LOW_LEVEL = (0x0 << 5); // (AIC) External Sources Code Label Low-level Sensitive
    static const    uint32_t AIC_SRCTYPE_INT_POSITIVE_EDGE = (0x1 << 5); // (AIC) Internal Sources Code Label Positive Edge triggered
    static const    uint32_t AIC_SRCTYPE_EXT_NEGATIVE_EDGE = (0x1 << 5); // (AIC) External Sources Code Label Negative Edge triggered
    static const    uint32_t AIC_SRCTYPE_HIGH_LEVEL = (0x2 << 5); // (AIC) Internal Or External Sources Code Label High-level Sensitive
    static const    uint32_t AIC_SRCTYPE_POSITIVE_EDGE = (0x3 << 5); // (AIC) Internal Or External Sources Code Label Positive Edge triggered



    volatile uint32_t AIC_SVR[32];    // Source Vector Register

    volatile uint32_t AIC_IVR;        // IRQ Vector Register

    volatile uint32_t AIC_FVR;        // FIQ Vector Register

    volatile uint32_t AIC_ISR;        // Interrupt Status Register

    volatile uint32_t AIC_IPR;        // Interrupt Pending Register

    volatile uint32_t AIC_IMR;        // Interrupt Mask Register

    volatile uint32_t AIC_CISR;       // Core Interrupt Status Register
    static const    uint32_t AIC_NFIQ = (0x1 << 0); // (AIC) NFIQ Status
    static const    uint32_t AIC_NIRQ = (0x1 << 1); // (AIC) NIRQ Status

    volatile uint32_t Reserved6[2];

    volatile uint32_t AIC_IECR;       // Interrupt Enable Command Register

    volatile uint32_t AIC_IDCR;       // Interrupt Disable Command Register
    static const    uint32_t AIC_IDCR_DIABLE_ALL = 0xFFFFFFFF; // disable all

    volatile uint32_t AIC_ICCR;       // Interrupt Clear Command Register
    static const    uint32_t AIC_ICCR_CLEAR_ALL = 0xFFFFFFFF; // clear all

    volatile uint32_t AIC_ISCR;       // Interrupt Set Command Register

    volatile uint32_t AIC_EOICR;      // End of Interrupt Command Register

    volatile uint32_t AIC_SPU;        // Spurious Vector Register

    volatile uint32_t AIC_DCR;        // Debug Control Register (Protect)
    static const    uint32_t AIC_DCR_PROT = (0x1 << 0); // (AIC) Protection Mode
    static const    uint32_t AIC_DCR_GMSK = (0x1 << 1); // (AIC) General Mask

    volatile uint32_t Reserved7[1];

    volatile uint32_t AIC_FFER;       // Fast Forcing Enable Register

    volatile uint32_t AIC_FFDR;       // Fast Forcing Disable Register

    volatile uint32_t AIC_FFSR;       // Fast Forcing Status Register


    bool IsInterruptPending() {
        if (AIC_IPR & AIC_IMR) {
            return true;
        }

        return false;
    }
};

//
// AT91_AIC
//////////////////////////////////////////////////////////////////////////////

typedef void(*AT91_Interrupt_Handler)(void* arg);

struct AT91_Interrupt_Callback {

public:
    void* EntryPoint;
    void* Argument;

public:
    void Initialize(uint32_t* EntryPoint, void* Argument) {
        this->EntryPoint = (void*)EntryPoint;
        this->Argument = Argument;
    }

    void Execute() const {
        AT91_Interrupt_Handler EntryPoint = (AT91_Interrupt_Handler)this->EntryPoint;

        void* Argument = this->Argument;

        if (EntryPoint) {
            EntryPoint(Argument);
        }
    }
};

class AT91_DisableInterrupts_RaiiHelper {
    uint32_t state;

public:
    AT91_DisableInterrupts_RaiiHelper();
    ~AT91_DisableInterrupts_RaiiHelper();

    bool IsDisabled();
    void Acquire();
    void Release();
};

class AT91_InterruptStarted_RaiiHelper {
public:
    AT91_InterruptStarted_RaiiHelper();
    ~AT91_InterruptStarted_RaiiHelper();
};

#define DISABLE_INTERRUPTS_SCOPED(name) AT91_DisableInterrupts_RaiiHelper name
#define INTERRUPT_STARTED_SCOPED(name) AT91_InterruptStarted_RaiiHelper name

bool AT91_InterruptInternal_Activate(uint32_t index, uint32_t* isr, void* isrParam);
bool AT91_InterruptInternal_Deactivate(uint32_t index);

void AT91_Interrupt_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* AT91_Interrupt_GetRequiredApi();
TinyCLR_Result AT91_Interrupt_Initialize(const TinyCLR_Interrupt_Controller* self, TinyCLR_Interrupt_StartStopHandler onInterruptStart, TinyCLR_Interrupt_StartStopHandler onInterruptEnd);
TinyCLR_Result AT91_Interrupt_Uninitialize(const TinyCLR_Interrupt_Controller* self);
void AT91_Interrupt_Enable();
void AT91_Interrupt_Disable();
void AT91_Interrupt_WaitForInterrupt();
bool AT91_Interrupt_IsDisabled();

void AT91_Interrupt_ForceInterrupt(uint32_t Irq_Index);
extern TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Started;
extern TinyCLR_Interrupt_StartStopHandler AT91_Interrupt_Ended;

//////////////////////////////////////////////////////////////////////////////
// AT91_I2C
//
struct AT91_I2C {
    static const uint32_t c_Base0 = AT91C_BASE_TWI0;
    static const uint32_t c_Base1 = AT91C_BASE_TWI1;
    static const uint32_t c_Base2 = AT91C_BASE_TWI2;

    //--//

    volatile uint32_t TWI_CR;         // Control Register
    static const    uint32_t TWI_CR_START = (0x1 << 0); // send START condition
    static const    uint32_t TWI_CR_STOP = (0x1 << 1); // send STOP condition
    static const    uint32_t TWI_CR_MSEN = (0x1 << 2); // Enable Master Transfer
    static const    uint32_t TWI_CR_MSDIS = (0x1 << 3); // Disable Master Transfer
// for SAM9
    static const    uint32_t TWI_CR_SVEN = (0x1 << 4); // enable Slave Transfer
    static const    uint32_t TWI_CR_SVDIS = (0x1 << 5); // Disable Slave Transfer
    static const    uint32_t TWI_CR_QUICK = (0x1 << 6); //
////

    static const    uint32_t TWI_CR_SWRST = (0x1 << 7); // Softwre Reset

    volatile uint32_t TWI_MMR;           // Master Mode Register
    static const    uint32_t TWI_MMR_IADRSZ_0 = (0x0 << 8);  // No internal device address
    static const    uint32_t TWI_MMR_IADRSZ_1 = (0x1 << 8);  // One byte device address
    static const    uint32_t TWI_MMR_IADRSZ_2 = (0x2 << 8);  // Two bytes device address
    static const    uint32_t TWI_MMR_MREAD_W = (0x0 << 12); // Three bytes device address
    static const    uint32_t TWI_MMR_MREAD_R = (0x1 << 12); // Three bytes device address
    static const    uint32_t TWI_MMR_DADR_MASK = 0x00FF0000;  // Subordinate device address mask
    static const    uint32_t TWI_MMR_DADR_SHIFT = 16;           // subordinate address position

    volatile uint32_t Reserved0[1];

    volatile uint32_t TWI_IADR;           // Internal address register
    static const    uint32_t TWI_IADR_MASK_0 = 0x00000000; // No internal device address
    static const    uint32_t TWI_IADR_MASK_1 = 0x000000FF; // One byte device address
    static const    uint32_t TWI_IADR_MASK_2 = 0x0000FFFF; // Two bytes device address
    static const    uint32_t TWI_IADR_MASK_3 = 0x00FFFFFF; // Three bytes device address

    volatile uint32_t TWI_CWGR;           // Clock waveform generator Register
    static const    uint32_t TWI_CWGR_CLDIV_MASK = 0x000000FF; // Clock Low Divider
    static const    uint32_t TWI_CWGR_CHDIV_MASK = 0x0000FF00; // Clock High Divider
    static const    uint32_t TWI_CWGR_CHDIV_SHIFT = 8;         // Clock High Divider shift
    static const    uint32_t TWI_CWGR_CKDIV_MASK = 0x00070000; // Clock Divider
    static const    uint32_t TWI_CWGR_CKDIV_SHIFT = 16;         // Clock Divider shift

    volatile uint32_t Reserved1[3];

    volatile uint32_t TWI_SR;             // Status Register
    static const    uint32_t TWI_SR_TXCOMP = (0x1 << 0); // holding and shift register are empty and STOP condition has been sent
    static const    uint32_t TWI_SR_RXRDY = (0x1 << 1); // Data moved from shifter since last read
    static const    uint32_t TWI_SR_TXRDY = (0x1 << 2); // Data moved to shifter
    static const    uint32_t TWI_SR_NACK = (0x1 << 8); // No acknoledge received

    volatile uint32_t TWI_IER;            // Interrupt Enable Register
    static const    uint32_t TWI_IER_TXCOMP = (0x1 << 0); // Transmission completed
    static const    uint32_t TWI_IER_RXRDY = (0x1 << 1); // receive holding register ready
    static const    uint32_t TWI_IER_TXRDY = (0x1 << 2); // transmit holding register ready
    static const    uint32_t TWI_IER_NACK = (0x1 << 8); // No acknoledge received

    volatile uint32_t TWI_IDR;            // Interrupt Disable Register
    static const    uint32_t TWI_IDR_TXCOMP = (0x1 << 0); // Transmission completed
    static const    uint32_t TWI_IDR_RXRDY = (0x1 << 1); // receive holding register ready
    static const    uint32_t TWI_IDR_TXRDY = (0x1 << 2); // transmit holding register ready
    static const    uint32_t TWI_IDR_NACK = (0x1 << 8); // No acknoledge received

    volatile uint32_t TWI_IMR;            // Interrupt Mask Register
    static const    uint32_t TWI_IMR_TXCOMP = (0x1 << 0); // Transmission completed
    static const    uint32_t TWI_IMR_RXRDY = (0x1 << 1); // receive holding register ready
    static const    uint32_t TWI_IMR_TXRDY = (0x1 << 2); // transmit holding register ready
    static const    uint32_t TWI_IMR_NACK = (0x1 << 8); // No acknoledge received

    volatile uint32_t TWI_RHR;            // Receive Holding Register
    static const    uint32_t TWI_RHR_RXDATA_MASK = 0x000000FF; // data mask

    volatile uint32_t TWI_THR;            // Receive Holding Register
    static const    uint32_t TWI_THR_TXDATA_MASK = 0x000000FF; // data mask
};
//
// AT91_I2C
//////////////////////////////////////////////////////////////////////////////
void AT91_I2c_AddApi(const TinyCLR_Api_Manager* apiManager);
void AT91_I2c_Reset();
TinyCLR_Result AT91_I2c_Acquire(const TinyCLR_I2c_Controller* self);
TinyCLR_Result AT91_I2c_Release(const TinyCLR_I2c_Controller* self);
TinyCLR_Result AT91_I2c_SetActiveSettings(const TinyCLR_I2c_Controller* self, const TinyCLR_I2c_Settings* settings);
TinyCLR_Result AT91_I2c_WriteRead(const TinyCLR_I2c_Controller* self, const uint8_t* writeBuffer, size_t& writeLength, uint8_t* readBuffer, size_t& readLength, bool sendStartCondition, bool sendStopCondition, TinyCLR_I2c_TransferStatus& error);
void AT91_I2c_StartTransaction();
void AT91_I2c_StopTransaction();

// Time
//////////////////////////////////////////////////////////////////////////////
// AT91 Timer Channel
//
struct AT91_TC {

    static const uint32_t c_Base = AT91C_BASE_TC0;

    volatile uint32_t TC_CCR;    // Channel Control Register
    static const    uint32_t TC_CLKEN = (0x1 << 0);// (TC) Counter Clock Enable Command
    static const    uint32_t TC_CLKDIS = (0x1 << 1);// (TC)Counter Clock Disable Command
    static const    uint32_t TC_SWTRG = (0x1 << 2);// (TC)Software Trigger Command

    volatile uint32_t TC_CMR;    // Channel Mode Register (Capture Mode / Waveform Mode)
    static const    uint32_t TC_CLKS = (0x7 << 0);// (TC)Clock Selection
    static const    uint32_t     TC_CLKS_TIMER_DIV1_CLOCK = (0x0);// (TC)Clock selected: TIMER_DIV1_CLOCK
    static const    uint32_t     TC_CLKS_TIMER_DIV2_CLOCK = (0x1);// (TC)Clock selected: TIMER_DIV2_CLOCK
    static const    uint32_t     TC_CLKS_TIMER_DIV3_CLOCK = (0x2);// (TC)Clock selected: TIMER_DIV3_CLOCK
    static const    uint32_t     TC_CLKS_TIMER_DIV4_CLOCK = (0x3);// (TC)Clock selected: TIMER_DIV4_CLOCK
    static const    uint32_t     TC_CLKS_TIMER_DIV5_CLOCK = (0x4);// (TC)Clock selected: TIMER_DIV5_CLOCK
    static const    uint32_t     TC_CLKS_XC0 = (0x5);// (TC)Clock selected: XC0
    static const    uint32_t     TC_CLKS_XC1 = (0x6);// (TC)Clock selected: XC1
    static const    uint32_t     TC_CLKS_XC2 = (0x7);// (TC)Clock selected: XC2
    static const    uint32_t TC_CLKI = (0x1 << 3);// (TC)Clock Invert
    static const    uint32_t TC_BURST = (0x3 << 4);// (TC)Burst Signal Selection
    static const    uint32_t     TC_BURST_NONE = (0x0 << 4);// (TC)The clock is not gated by an external signal
    static const    uint32_t     TC_BURST_XC0 = (0x1 << 4);// (TC)XC0 is ANDed with the selected clock
    static const    uint32_t     TC_BURST_XC1 = (0x2 << 4);// (TC)XC1 is ANDed with the selected clock
    static const    uint32_t     TC_BURST_XC2 = (0x3 << 4);// (TC)XC2 is ANDed with the selected clock
    static const    uint32_t TC_CPCSTOP = (0x1 << 6);// (TC)Counter Clock Stopped with RC Compare
    static const    uint32_t TC_LDBSTOP = (0x1 << 6);// (TC)Counter Clock Stopped with RB Loading
    static const    uint32_t TC_CPCDIS = (0x1 << 7);// (TC)Counter Clock Disable with RC Compare
    static const    uint32_t TC_LDBDIS = (0x1 << 7);// (TC)Counter Clock Disabled with RB Loading
    static const    uint32_t TC_ETRGEDG = (0x3 << 8);// (TC)External Trigger Edge Selection
    static const    uint32_t     TC_ETRGEDG_NONE = (0x0 << 8);// (TC)Edge: None
    static const    uint32_t     TC_ETRGEDG_RISING = (0x1 << 8);// (TC)Edge: rising edge
    static const    uint32_t     TC_ETRGEDG_FALLING = (0x2 << 8);// (TC)Edge: falling edge
    static const    uint32_t     TC_ETRGEDG_BOTH = (0x3 << 8);// (TC)Edge: each edge
    static const    uint32_t TC_EEVTEDG = (0x3 << 8);// (TC)External Event Edge Selection
    static const    uint32_t     TC_EEVTEDG_NONE = (0x0 << 8);// (TC)Edge: None
    static const    uint32_t     TC_EEVTEDG_RISING = (0x1 << 8);// (TC)Edge: rising edge
    static const    uint32_t     TC_EEVTEDG_FALLING = (0x2 << 8);// (TC)Edge: falling edge
    static const    uint32_t     TC_EEVTEDG_BOTH = (0x3 << 8);// (TC)Edge: each edge
    static const    uint32_t TC_EEVT = (0x3 << 10);// (TC)External Event  Selection
    static const    uint32_t     TC_EEVT_TIOB = (0x0 << 10);// (TC)Signal selected as external event: TIOB TIOB direction: input
    static const    uint32_t     TC_EEVT_XC0 = (0x1 << 10);// (TC)Signal selected as external event: XC0 TIOB direction: output
    static const    uint32_t     TC_EEVT_XC1 = (0x2 << 10);// (TC)Signal selected as external event: XC1 TIOB direction: output
    static const    uint32_t     TC_EEVT_XC2 = (0x3 << 10);// (TC)Signal selected as external event: XC2 TIOB direction: output
    static const    uint32_t TC_ABETRG = (0x1 << 10);// (TC)TIOA or TIOB External Trigger Selection
    static const    uint32_t TC_ENETRG = (0x1 << 12);// (TC)External Event Trigger enable
    static const    uint32_t TC_WAVESEL = (0x3 << 13);// (TC)Waveform  Selection
    static const    uint32_t     TC_WAVESEL_UP = (0x0 << 13);// (TC)UP mode without atomatic trigger on RC Compare
    static const    uint32_t     TC_WAVESEL_UPDOWN = (0x1 << 13);// (TC)UPDOWN mode without automatic trigger on RC Compare
    static const    uint32_t     TC_WAVESEL_UP_AUTO = (0x2 << 13);// (TC)UP mode with automatic trigger on RC Compare
    static const    uint32_t     TC_WAVESEL_UPDOWN_AUTO = (0x3 << 13);// (TC)UPDOWN mode with automatic trigger on RC Compare
    static const    uint32_t TC_CPCTRG = (0x1 << 14);// (TC)RC Compare Trigger Enable
    static const    uint32_t TC_WAVE = (0x1 << 15);// (TC)
    static const    uint32_t TC_ACPA = (0x3 << 16);// (TC)RA Compare Effect on TIOA
    static const    uint32_t     TC_ACPA_NONE = (0x0 << 16);// (TC)Effect: none
    static const    uint32_t     TC_ACPA_SET = (0x1 << 16);// (TC)Effect: set
    static const    uint32_t     TC_ACPA_CLEAR = (0x2 << 16);// (TC)Effect: clear
    static const    uint32_t     TC_ACPA_TOGGLE = (0x3 << 16);// (TC)Effect: toggle
    static const    uint32_t TC_LDRA = (0x3 << 16);// (TC)RA Loading Selection
    static const    uint32_t     TC_LDRA_NONE = (0x0 << 16);// (TC)Edge: None
    static const    uint32_t     TC_LDRA_RISING = (0x1 << 16);// (TC)Edge: rising edge of TIOA
    static const    uint32_t     TC_LDRA_FALLING = (0x2 << 16);// (TC)Edge: falling edge of TIOA
    static const    uint32_t     TC_LDRA_BOTH = (0x3 << 16);// (TC)Edge: each edge of TIOA
    static const    uint32_t TC_ACPC = (0x3 << 18);// (TC)RC Compare Effect on TIOA
    static const    uint32_t     TC_ACPC_NONE = (0x0 << 18);// (TC)Effect: none
    static const    uint32_t     TC_ACPC_SET = (0x1 << 18);// (TC)Effect: set
    static const    uint32_t     TC_ACPC_CLEAR = (0x2 << 18);// (TC)Effect: clear
    static const    uint32_t     TC_ACPC_TOGGLE = (0x3 << 18);// (TC)Effect: toggle
    static const    uint32_t TC_LDRB = (0x3 << 18);// (TC)RB Loading Selection
    static const    uint32_t     TC_LDRB_NONE = (0x0 << 18);// (TC)Edge: None
    static const    uint32_t     TC_LDRB_RISING = (0x1 << 18);// (TC)Edge: rising edge of TIOA
    static const    uint32_t     TC_LDRB_FALLING = (0x2 << 18);// (TC)Edge: falling edge of TIOA
    static const    uint32_t     TC_LDRB_BOTH = (0x3 << 18);// (TC)Edge: each edge of TIOA
    static const    uint32_t TC_AEEVT = (0x3 << 20);// (TC)External Event Effect on TIOA
    static const    uint32_t     TC_AEEVT_NONE = (0x0 << 20);// (TC)Effect: none
    static const    uint32_t     TC_AEEVT_SET = (0x1 << 20);// (TC)Effect: set
    static const    uint32_t     TC_AEEVT_CLEAR = (0x2 << 20);// (TC)Effect: clear
    static const    uint32_t     TC_AEEVT_TOGGLE = (0x3 << 20);// (TC)Effect: toggle
    static const    uint32_t TC_ASWTRG = (0x3 << 22);// (TC)Software Trigger Effect on TIOA
    static const    uint32_t     TC_ASWTRG_NONE = (0x0 << 22);// (TC)Effect: none
    static const    uint32_t     TC_ASWTRG_SET = (0x1 << 22);// (TC)Effect: set
    static const    uint32_t     TC_ASWTRG_CLEAR = (0x2 << 22);// (TC)Effect: clear
    static const    uint32_t     TC_ASWTRG_TOGGLE = (0x3 << 22);// (TC)Effect: toggle
    static const    uint32_t TC_BCPB = (0x3 << 24);// (TC)RB Compare Effect on TIOB
    static const    uint32_t     TC_BCPB_NONE = (0x0 << 24);// (TC)Effect: none
    static const    uint32_t     TC_BCPB_SET = (0x1 << 24);// (TC)Effect: set
    static const    uint32_t     TC_BCPB_CLEAR = (0x2 << 24);// (TC)Effect: clear
    static const    uint32_t     TC_BCPB_TOGGLE = (0x3 << 24);// (TC)Effect: toggle
    static const    uint32_t TC_BCPC = (0x3 << 26);// (TC)RC Compare Effect on TIOB
    static const    uint32_t     TC_BCPC_NONE = (0x0 << 26);// (TC)Effect: none
    static const    uint32_t     TC_BCPC_SET = (0x1 << 26);// (TC)Effect: set
    static const    uint32_t     TC_BCPC_CLEAR = (0x2 << 26);// (TC)Effect: clear
    static const    uint32_t     TC_BCPC_TOGGLE = (0x3 << 26);// (TC)Effect: toggle
    static const    uint32_t TC_BEEVT = (0x3 << 28);// (TC)External Event Effect on TIOB
    static const    uint32_t     TC_BEEVT_NONE = (0x0 << 28);// (TC)Effect: none
    static const    uint32_t     TC_BEEVT_SET = (0x1 << 28);// (TC)Effect: set
    static const    uint32_t     TC_BEEVT_CLEAR = (0x2 << 28);// (TC)Effect: clear
    static const    uint32_t     TC_BEEVT_TOGGLE = (0x3 << 28);// (TC)Effect: toggle
    static const    uint32_t TC_BSWTRG = ((uint32_t)0x3 << 30);// (TC)Software Trigger Effect on TIOB
    static const    uint32_t     TC_BSWTRG_NONE = ((uint32_t)0x0 << 30);// (TC)Effect: none
    static const    uint32_t     TC_BSWTRG_SET = ((uint32_t)0x1 << 30);// (TC)Effect: set
    static const    uint32_t     TC_BSWTRG_CLEAR = ((uint32_t)0x2 << 30);// (TC)Effect: clear
    static const    uint32_t     TC_BSWTRG_TOGGLE = ((uint32_t)0x3 << 30);// (TC)Effect: toggle

    volatile uint32_t Reserved0[2];

    volatile uint32_t TC_CV;     // Counter Value

    volatile uint32_t TC_RA;     // Register A

    volatile uint32_t TC_RB;     // Register B

    volatile uint32_t TC_RC;     // Register C

    volatile uint32_t TC_SR;     // Status Register
    static const    uint32_t TC_COVFS = (0x1 << 0);// (TC)Counter Overflow
    static const    uint32_t TC_LOVRS = (0x1 << 1);// (TC)Load Overrun
    static const    uint32_t TC_CPAS = (0x1 << 2);// (TC)RA Compare
    static const    uint32_t TC_CPBS = (0x1 << 3);// (TC)RB Compare
    static const    uint32_t TC_CPCS = (0x1 << 4);// (TC)RC Compare
    static const    uint32_t TC_LDRAS = (0x1 << 5);// (TC)RA Loading
    static const    uint32_t TC_LDRBS = (0x1 << 6);// (TC)RB Loading
    static const    uint32_t TC_ETRGS = (0x1 << 7);// (TC)External Trigger
    static const    uint32_t TC_CLKSTA = (0x1 << 16);// (TC)Clock Enabling
    static const    uint32_t TC_MTIOA = (0x1 << 17);// (TC)TIOA Mirror
    static const    uint32_t TC_MTIOB = (0x1 << 18);// (TC)TIOA Mirror

    volatile uint32_t TC_IER;    // Interrupt Enable Register

    volatile uint32_t TC_IDR;    // Interrupt Disable Register

    volatile uint32_t TC_IMR;    // Interrupt Mask Register
};

//
// AT91 Timer Channel
void AT91_Time_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* AT91_Time_GetRequiredApi();
TinyCLR_Result AT91_Time_Initialize(const TinyCLR_NativeTime_Controller* self);
TinyCLR_Result AT91_Time_Uninitialize(const TinyCLR_NativeTime_Controller* self);
uint64_t AT91_Time_GetTimeForProcessorTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks);
uint64_t AT91_Time_GetProcessorTicksForTime(const TinyCLR_NativeTime_Controller* self, uint64_t time);
uint64_t AT91_Time_MillisecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t ticks);
uint64_t AT91_Time_MicrosecondsToTicks(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
uint64_t AT91_Time_GetCurrentProcessorTicks(const TinyCLR_NativeTime_Controller* self);
uint64_t AT91_Time_GetCurrentProcessorTime();
TinyCLR_Result AT91_Time_SetNextTickCallbackTime(const TinyCLR_NativeTime_Controller* self, uint64_t processorTicks);
TinyCLR_Result AT91_Time_SetTickCallback(const TinyCLR_NativeTime_Controller* self, TinyCLR_NativeTime_Callback callback);
void AT91_Time_Delay(const TinyCLR_NativeTime_Controller* self, uint64_t microseconds);
void AT91_Time_GetDriftParameters(const TinyCLR_NativeTime_Controller* self, int32_t* a, int32_t* b, int64_t* c);
void AT91_Time_DelayNative(const TinyCLR_NativeTime_Controller* self, uint64_t nativeTime);

// Power
void AT91_Power_AddApi(const TinyCLR_Api_Manager* apiManager);
const TinyCLR_Api_Info* AT91_Power_GetRequiredApi();
void AT91_Power_SetHandlers(void(*stop)(), void(*restart)());
TinyCLR_Result AT91_Power_Sleep(const TinyCLR_Power_Controller* self, TinyCLR_Power_SleepLevel level, TinyCLR_Power_SleepWakeSource wakeSource);
TinyCLR_Result AT91_Power_Reset(const TinyCLR_Power_Controller* self, bool runCoreAfter);
TinyCLR_Result AT91_Power_Initialize(const TinyCLR_Power_Controller* self);
TinyCLR_Result AT91_Power_Uninitialize(const TinyCLR_Power_Controller* self);

//UsbClient
const TinyCLR_Api_Info* AT91_UsbDevice_GetRequiredApi();
void AT91_UsbDevice_AddApi(const TinyCLR_Api_Manager* apiManager);
void AT91_UsbDevice_Reset();
void AT91_UsbDevice_PinConfiguration();

struct USB_PACKET64;
struct UsClientState;
typedef void(*USB_NEXT_CALLBACK)(UsClientState*);

void TinyCLR_UsbClient_ClearEvent(UsClientState *usClientState, uint32_t event);
void TinyCLR_UsbClient_ClearEndpoints(UsClientState *usClientState, int32_t endpoint);
USB_PACKET64* TinyCLR_UsbClient_RxEnqueue(UsClientState* usClientState, int32_t endpoint, bool& disableRx);
USB_PACKET64* TinyCLR_UsbClient_TxDequeue(UsClientState* usClientState, int32_t endpoint);
void TinyCLR_UsbClient_StateCallback(UsClientState* usClientState);
uint8_t TinyCLR_UsbClient_ControlCallback(UsClientState* usClientState);

// LCD
void AT91_Display_Reset();
void AT91_Display_AddApi(const TinyCLR_Api_Manager* apiManager);
TinyCLR_Result AT91_Display_Acquire(const TinyCLR_Display_Controller* self);
TinyCLR_Result AT91_Display_Release(const TinyCLR_Display_Controller* self);
TinyCLR_Result AT91_Display_Enable(const TinyCLR_Display_Controller* self);
TinyCLR_Result AT91_Display_Disable(const TinyCLR_Display_Controller* self);
TinyCLR_Result AT91_Display_GetCapabilities(const TinyCLR_Display_Controller* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount);
TinyCLR_Result AT91_Display_GetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration);
TinyCLR_Result AT91_Display_SetConfiguration(const TinyCLR_Display_Controller* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration);
TinyCLR_Result AT91_Display_DrawBuffer(const TinyCLR_Display_Controller* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data);
TinyCLR_Result AT91_Display_DrawPixel(const TinyCLR_Display_Controller* self, uint32_t x, uint32_t y, uint64_t color);
TinyCLR_Result AT91_Display_WriteString(const TinyCLR_Display_Controller* self, const char* buffer, size_t length);

//WatchDog
//////////////////////////////////////////////////////////////////////////////
// WATCHDOG
//
struct AT91_WATCHDOG {
    static const uint32_t c_Base = AT91C_BASE_WDTC;

    //--//

    volatile uint32_t WTDG_CR;
    static const    uint32_t WCR__KEY_mask = 0xFF000000;
    static const    uint32_t WCR__KEY = 0xA5000000;
    static const    uint32_t WCR__RESTART = 0x00000001;

    volatile uint32_t WTDG_MR;
    static const    uint32_t WMR_WDV_mask = 0x00000FFF;
    static const    uint32_t WMR_WDFIEN = 0x00001000;
    static const    uint32_t WMR_WDRSTEN = 0x00002000;

    static const    uint32_t WMR_WDRPROC = 0x00004000;
    static const    uint32_t WMR_WDDIS = 0x00008000;

    static const    uint32_t WMR_WDD_mask = 0x0FFF0000;
    static const    uint32_t WMR_WDDBGHLT = 0x10000000;
    static const    uint32_t WMR_WDIDLEHLT = 0x20000000;


    volatile uint32_t WTDG_SR;
    static const    uint32_t WSR__WDUNF = 0x00000001;
    static const    uint32_t WSR__WDERR = 0x00000002;
};
//
// WATCHDOG
//////////////////////////////////////////////////////////////////////////////

//Startup
void AT91_Startup_Initialize();
void AT91_Startup_GetHeap(uint8_t*& start, size_t& length);
void AT91_Startup_GetDebuggerTransportApi(const TinyCLR_Api_Info*& api, const void*& configuration);
void AT91_Startup_GetRunApp(bool& runApp);
void AT91_Startup_GetDeploymentApi(const TinyCLR_Api_Info*& api, const TinyCLR_Startup_DeploymentConfiguration*& configuration);
const TinyCLR_Startup_DeploymentConfiguration* AT91_Deployment_GetDeploymentConfiguration();
void AT91_Startup_OnSoftReset(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);
void AT91_Startup_OnSoftResetDevice(const TinyCLR_Api_Manager* apiManager, const TinyCLR_Interop_Manager* interopProvider);

struct AT91 {

    static const uint32_t c_UncachableMask = 0x80000000;

    //    static AT91_EIM     & EIM()             { return *(AT91_EIM     *)(size_t)(      AT91_EIM     ::c_Base                                      ); }
    //    static AT91_SC      & SC()              { return *(AT91_SC      *)(size_t)(      AT91_SC      ::c_Base                                      ); }
    //    static AT91_CMU     & CMU  (         )  { return *(AT91_CMU     *)(size_t)(      AT91_CMU     ::c_Base                                      ); }
    //    static AT91_PWM     & PWM()             { return *(AT91_PWM     *)(size_t)(      AT91_PWM     ::c_Base                                      ); }
    //    static AT91_DMA     & DMA()             { return *(AT91_DMA     *)(size_t)(      AT91_DMA     ::c_Base                                      ); }

    static AT91_I2C     & I2C(int sel) { return *(AT91_I2C     *)(size_t)(sel == 0 ? AT91_I2C::c_Base0 : (sel == 1 ? AT91_I2C::c_Base1 : AT91_I2C::c_Base2)); }
    static AT91_AIC     & AIC() { return *(AT91_AIC     *)(size_t)(AT91_AIC::c_Base); }
    static AT91_PIO     & PIO(int sel) { return *(AT91_PIO     *)(size_t)(AT91_PIO::c_Base + AT91_PIO::c_Base_Offset * sel); }
    static AT91_PMC     & PMC() { return *(AT91_PMC     *)(size_t)(AT91_PMC::c_Base); }
    static AT91_SPI     & SPI(int sel) {
        if (sel == 0) return *(AT91_SPI     *)(size_t)(AT91_SPI::c_Base_1);
        else      return *(AT91_SPI     *)(size_t)(AT91_SPI::c_Base_2);
    }

    static AT91_TC      & TIMER(int sel) { return *(AT91_TC*)(size_t)(AT91_TC::c_Base + (sel * 0x40)); }
    static AT91_WATCHDOG& WTDG() { return *(AT91_WATCHDOG*)(size_t)(AT91_WATCHDOG::c_Base); }
    //***************************************************************************************************************************************************************************************************************
    static AT91_USART   & USART(int sel) {
        if (sel == 0)
            return *(AT91_USART*)(size_t)(AT91_USART::c_Base_dbg);
        else if ((sel > 0) && (sel < 4))
            return *(AT91_USART   *)(size_t)(AT91_USART::c_Base_usart + ((sel - 1) * 0x4000));
        else
            return *(AT91_USART   *)(size_t)(AT91_USART::c_Base_uart + ((sel - 4) * 0x4000));
    }
    //***************************************************************************************************************************************************************************************************************
    // static AT91_UDP     & UDP()             { return *(AT91_UDP     *)(size_t)(AT91_UDP     ::c_Base                                      ); }

    // #if defined(PLATFORM_ARM_SAM9261_ANY) || defined(PLATFORM_ARM_SAM9RL64_ANY)
        // static AT91_LCDC    & LCDC()            { return *(AT91_LCDC    *)(size_t)(AT91_LCDC    ::c_Base                                      ); }

        // static AT91_SDRAMC  & SDRAMC()          { return *(AT91_SDRAMC  *)(size_t)(AT91_SDRAMC  ::c_Base                                      ); }
        // static AT91_SMC     & SMCTRL()          { return *(AT91_SMC     *)(size_t)(AT91_SMC     ::c_Base                                      ); }
        // static AT91_MATRIX  & MATRIX()          { return *(AT91_MATRIX  *)(size_t)(AT91_MATRIX  ::c_Base                                      ); }
    // #endif
        // static AT91_DDRS     & DDRS()             { return *(AT91_DDRS     *)(size_t)(AT91_DDRS     ::c_Base                                      ); }
        // static AT91_PIT     & PIT()             { return *(AT91_PIT     *)(size_t)(AT91_PIT     ::c_Base                                      ); }
        //--//

};
