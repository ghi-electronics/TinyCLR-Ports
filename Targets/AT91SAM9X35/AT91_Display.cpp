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

#include "AT91.h"

#ifdef INCLUDE_DISPLAY

typedef struct _PALETTEENTRY_LCD {
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
    unsigned char Flags;
} PALETTEENTRY_LCD, *PPALETTEENTRY_LCD;


struct AT91SAM9X35_LCDC {
    volatile uint32_t LCDC_LCDCFG0;
    volatile uint32_t LCDC_LCDCFG1;
    volatile uint32_t LCDC_LCDCFG2;
    volatile uint32_t LCDC_LCDCFG3;
    volatile uint32_t LCDC_LCDCFG4;
    volatile uint32_t LCDC_LCDCFG5;
    volatile uint32_t LCDC_LCDCFG6;
    volatile uint32_t RESERVED_1;
    volatile uint32_t LCDC_LCDEN;
    volatile uint32_t LCDC_LCDDIS;
    volatile const uint32_t LCDC_LCDSR;
    volatile uint32_t LCDC_LCDIER;
    volatile uint32_t LCDC_LCDIDR;
    volatile uint32_t LCDC_LCDIMR;
    volatile uint32_t LCDC_LCDISR;
    volatile uint32_t RESERVED_2;
    volatile uint32_t LCDC_BASECHER;
    volatile uint32_t LCDC_BASECHDR;
    volatile uint32_t LCDC_BASECHSR;
    volatile uint32_t LCDC_BASEIER;
    volatile uint32_t LCDC_BASEIDR;
    volatile uint32_t LCDC_BASEIMR;
    volatile uint32_t LCDC_BASEISR;
    volatile uint32_t LCDC_BASEHEAD;
    volatile uint32_t LCDC_BASEADDR;
    volatile uint32_t LCDC_BASECTRL;
    volatile uint32_t LCDC_BASENEXT;
    volatile uint32_t LCDC_BASECFG0;
    volatile uint32_t LCDC_BASECFG1;
    volatile uint32_t LCDC_BASECFG2;
    volatile uint32_t LCDC_BASECFG3;
    volatile uint32_t LCDC_BASECFG4;
    volatile uint32_t RESERVED_3[32];
    volatile uint32_t LCDC_OVRCHER1;
    volatile uint32_t LCDC_OVRCHDR1;
    volatile uint32_t LCDC_OVRCHSR1;
    volatile uint32_t LCDC_OVRIER1;
    volatile uint32_t LCDC_OVRIDR1;
    volatile uint32_t LCDC_OVRIMR1;
    volatile uint32_t LCDC_OVRISR1;
    volatile uint32_t LCDC_OVRHEAD1;
    volatile uint32_t LCDC_OVRADDR1;
    volatile uint32_t LCDC_OVRCTRL1;
    volatile uint32_t LCDC_OVRNEXT1;
    volatile uint32_t LCDC_OVRCFG0;
    volatile uint32_t LCDC_OVRCFG1;
    volatile uint32_t LCDC_OVRCFG2;
    volatile uint32_t LCDC_OVRCFG3;
    volatile uint32_t LCDC_OVRCFG4;
    volatile uint32_t LCDC_OVRCFG5;
    volatile uint32_t LCDC_OVRCFG6;
    volatile uint32_t LCDC_OVRCFG7;
    volatile uint32_t LCDC_OVRCFG8;
    volatile uint32_t LCDC_OVRCFG9;
    volatile uint32_t RESERVED_4[75];
    volatile uint32_t LCDC_HEOCHER;
    volatile uint32_t LCDC_HEOCHDR;
    volatile uint32_t LCDC_HEOCHSR;
    volatile uint32_t LCDC_HEOIER;
    volatile uint32_t LCDC_HEOIDR;
    volatile uint32_t LCDC_HEOIMR;
    volatile uint32_t LCDC_HEOISR;
    volatile uint32_t LCDC_HEOHEAD;
    volatile uint32_t LCDC_HEOADDR;
    volatile uint32_t LCDC_HEOCTRL;
    volatile uint32_t LCDC_HEONEXT;
    volatile uint32_t LCDC_HEOUHEAD;
    volatile uint32_t LCDC_HEOUADDR;
    volatile uint32_t LCDC_HEOUCTRL;
    volatile uint32_t LCDC_HEOUNEXT;
    volatile uint32_t LCDC_HEOVHEAD;
    volatile uint32_t LCDC_HEOVADDR;
    volatile uint32_t LCDC_HEOVCTRL;
    volatile uint32_t LCDC_HEOVNEXT;
    volatile uint32_t LCDC_HEOCFG0;
    volatile uint32_t LCDC_HEOCFG1;
    volatile uint32_t LCDC_HEOCFG2;
    volatile uint32_t LCDC_HEOCFG3;
    volatile uint32_t LCDC_HEOCFG4;
    volatile uint32_t LCDC_HEOCFG5;
    volatile uint32_t LCDC_HEOCFG6;
    volatile uint32_t LCDC_HEOCFG7;
    volatile uint32_t LCDC_HEOCFG8;
    volatile uint32_t LCDC_HEOCFG9;
    volatile uint32_t LCDC_HEOCFG10;
    volatile uint32_t LCDC_HEOCFG11;
    volatile uint32_t LCDC_HEOCFG12;
    volatile uint32_t LCDC_HEOCFG13;
    volatile uint32_t LCDC_HEOCFG14;
    volatile uint32_t LCDC_HEOCFG15;
    volatile uint32_t LCDC_HEOCFG16;
    volatile uint32_t RESERVED_5[12];
    volatile uint32_t LCDC_HCRCHER;
    volatile uint32_t LCDC_HCRCHDR;
    volatile uint32_t LCDC_HCRCHSR;
    volatile uint32_t LCDC_HCRIER;
    volatile uint32_t LCDC_HCRIDR;
    volatile uint32_t LCDC_HCRIMR;
    volatile uint32_t LCDC_HCRISR;
    volatile uint32_t LCDC_HCRHEAD;
    volatile uint32_t LCDC_HCRADDR;
    volatile uint32_t LCDC_HCRCTRL;
    volatile uint32_t LCDC_HCRNEXT;
    volatile uint32_t LCDC_HCRCFG0;
    volatile uint32_t LCDC_HCRCFG1;
    volatile uint32_t LCDC_HCRCFG2;
    volatile uint32_t LCDC_HCRCFG3;
    volatile uint32_t LCDC_HCRCFG4;
    volatile uint32_t RESERVED_6;
    volatile uint32_t LCDC_HCRCFG6;
    volatile uint32_t LCDC_HCRCFG7;
    volatile uint32_t LCDC_HCRCFG8;
    volatile uint32_t LCDC_HCRCFG9;
    volatile uint32_t RESERVED_7[27];
    volatile uint32_t LCDC_BASECLUT[256];
    volatile uint32_t LCDC_OVR1CLUT[256];
    volatile uint32_t RESERVED_8[256];
    volatile uint32_t LCDC_HEOCLUT[256];
    volatile uint32_t LCDC_HCRCLUT[256];
    volatile uint32_t RESERVED_9[507];
    volatile uint32_t LCDC_ADDRSIZE;
    volatile uint32_t LCDC_IPNAME1;
    volatile uint32_t LCDC_IPNAME2;
    volatile uint32_t LCDC_FEATURES;
    volatile uint32_t LCDC_VERSION;
};

#define LCDC_BASECFG0_DLBO (0x1u << 8) /**< \brief (LCDC_BASECFG0) Defined Length Burst Only For Channel Bus Transaction. */
#define LCDC_BASECFG0_BLEN_AHB_INCR16 (0x3u << 4) /**< \brief (LCDC_BASECFG0) AHB Access is started as soon as there is enough space in the FIFO to store a total amount of sixteen 32-bit data. An AHB INCR16 Burst is preferred. SINGLE, INCR, INCR4, INCR8 and INCR16 bursts can be used. INCR is used for a burst of 2 and 3 beats. */

/* -------- LCDC_LCDCFG0 : (LCDC Offset: 0x00000000) LCD Controller Configuration Register 0 -------- */
#define LCDC_LCDCFG0_CLKPOL (0x1u << 0) /**< \brief (LCDC_LCDCFG0) LCD Controller Clock Polarity */
#define LCDC_LCDCFG0_CLKSEL (0x1u << 2) /**< \brief (LCDC_LCDCFG0) LCD Controller Clock Source Selection */
#define LCDC_LCDCFG0_CLKPWMSEL (0x1u << 3) /**< \brief (LCDC_LCDCFG0) LCD Controller PWM Clock Source Selection */
#define LCDC_LCDCFG0_CGDISBASE (0x1u << 8) /**< \brief (LCDC_LCDCFG0) Clock Gating Disable Control for the Base Layer */
#define LCDC_LCDCFG0_CGDISOVR1 (0x1u << 9) /**< \brief (LCDC_LCDCFG0) Clock Gating Disable Control for the Overlay 1 Layer */
#define LCDC_LCDCFG0_CGDISHEO (0x1u << 11) /**< \brief (LCDC_LCDCFG0) Clock Gating Disable Control for the High End Overlay */
#define LCDC_LCDCFG0_CGDISHCR (0x1u << 12) /**< \brief (LCDC_LCDCFG0) Clock Gating Disable Control for the Hardware Cursor Layer */
#define LCDC_LCDCFG0_CLKDIV_Pos 16
#define LCDC_LCDCFG0_CLKDIV_Msk (0xffu << LCDC_LCDCFG0_CLKDIV_Pos) /**< \brief (LCDC_LCDCFG0) LCD Controller Clock Divider */
#define LCDC_LCDCFG0_CLKDIV(value) ((LCDC_LCDCFG0_CLKDIV_Msk & ((value) << LCDC_LCDCFG0_CLKDIV_Pos)))
/* -------- LCDC_LCDCFG1 : (LCDC Offset: 0x00000004) LCD Controller Configuration Register 1 -------- */
#define LCDC_LCDCFG1_HSPW_Pos 0
#define LCDC_LCDCFG1_HSPW_Msk (0x3fu << LCDC_LCDCFG1_HSPW_Pos) /**< \brief (LCDC_LCDCFG1) Horizontal Synchronization Pulse Width */
#define LCDC_LCDCFG1_HSPW(value) ((LCDC_LCDCFG1_HSPW_Msk & ((value) << LCDC_LCDCFG1_HSPW_Pos)))
#define LCDC_LCDCFG1_VSPW_Pos 16
#define LCDC_LCDCFG1_VSPW_Msk (0x3fu << LCDC_LCDCFG1_VSPW_Pos) /**< \brief (LCDC_LCDCFG1) Vertical Synchronization Pulse Width */
#define LCDC_LCDCFG1_VSPW(value) ((LCDC_LCDCFG1_VSPW_Msk & ((value) << LCDC_LCDCFG1_VSPW_Pos)))
/* -------- LCDC_LCDCFG2 : (LCDC Offset: 0x00000008) LCD Controller Configuration Register 2 -------- */
#define LCDC_LCDCFG2_VFPW_Pos 0
#define LCDC_LCDCFG2_VFPW_Msk (0x3fu << LCDC_LCDCFG2_VFPW_Pos) /**< \brief (LCDC_LCDCFG2) Vertical Front Porch Width */
#define LCDC_LCDCFG2_VFPW(value) ((LCDC_LCDCFG2_VFPW_Msk & ((value) << LCDC_LCDCFG2_VFPW_Pos)))
#define LCDC_LCDCFG2_VBPW_Pos 16
#define LCDC_LCDCFG2_VBPW_Msk (0x3fu << LCDC_LCDCFG2_VBPW_Pos) /**< \brief (LCDC_LCDCFG2) Vertical Back Porch Width */
#define LCDC_LCDCFG2_VBPW(value) ((LCDC_LCDCFG2_VBPW_Msk & ((value) << LCDC_LCDCFG2_VBPW_Pos)))
/* -------- LCDC_LCDCFG3 : (LCDC Offset: 0x0000000C) LCD Controller Configuration Register 3 -------- */
#define LCDC_LCDCFG3_HFPW_Pos 0
#define LCDC_LCDCFG3_HFPW_Msk (0xffu << LCDC_LCDCFG3_HFPW_Pos) /**< \brief (LCDC_LCDCFG3) Horizontal Front Porch Width */
#define LCDC_LCDCFG3_HFPW(value) ((LCDC_LCDCFG3_HFPW_Msk & ((value) << LCDC_LCDCFG3_HFPW_Pos)))
#define LCDC_LCDCFG3_HBPW_Pos 16
#define LCDC_LCDCFG3_HBPW_Msk (0xffu << LCDC_LCDCFG3_HBPW_Pos) /**< \brief (LCDC_LCDCFG3) Horizontal Back Porch Width */
#define LCDC_LCDCFG3_HBPW(value) ((LCDC_LCDCFG3_HBPW_Msk & ((value) << LCDC_LCDCFG3_HBPW_Pos)))
/* -------- LCDC_LCDCFG4 : (LCDC Offset: 0x00000010) LCD Controller Configuration Register 4 -------- */
#define LCDC_LCDCFG4_PPL_Pos 0
#define LCDC_LCDCFG4_PPL_Msk (0x7ffu << LCDC_LCDCFG4_PPL_Pos) /**< \brief (LCDC_LCDCFG4) Number of Pixels Per Line */
#define LCDC_LCDCFG4_PPL(value) ((LCDC_LCDCFG4_PPL_Msk & ((value) << LCDC_LCDCFG4_PPL_Pos)))
#define LCDC_LCDCFG4_RPF_Pos 16
#define LCDC_LCDCFG4_RPF_Msk (0x7ffu << LCDC_LCDCFG4_RPF_Pos) /**< \brief (LCDC_LCDCFG4) Number of Active Rows Per Frame */
#define LCDC_LCDCFG4_RPF(value) ((LCDC_LCDCFG4_RPF_Msk & ((value) << LCDC_LCDCFG4_RPF_Pos)))
/* -------- LCDC_LCDCFG5 : (LCDC Offset: 0x00000014) LCD Controller Configuration Register 5 -------- */
#define LCDC_LCDCFG5_HSPOL (0x1u << 0) /**< \brief (LCDC_LCDCFG5) Horizontal Synchronization Pulse Polarity */
#define LCDC_LCDCFG5_VSPOL (0x1u << 1) /**< \brief (LCDC_LCDCFG5) Vertical Synchronization Pulse Polarity */
#define LCDC_LCDCFG5_VSPDLYS (0x1u << 2) /**< \brief (LCDC_LCDCFG5) Vertical Synchronization Pulse Start */
#define LCDC_LCDCFG5_VSPDLYE (0x1u << 3) /**< \brief (LCDC_LCDCFG5) Vertical Synchronization Pulse End */
#define LCDC_LCDCFG5_DISPPOL (0x1u << 4) /**< \brief (LCDC_LCDCFG5) Display Signal Polarity */
#define LCDC_LCDCFG5_SERIAL (0x1u << 5) /**< \brief (LCDC_LCDCFG5)  */
#define LCDC_LCDCFG5_DITHER (0x1u << 6) /**< \brief (LCDC_LCDCFG5) LCD Controller Dithering */
#define LCDC_LCDCFG5_DISPDLY (0x1u << 7) /**< \brief (LCDC_LCDCFG5) LCD Controller Display Power Signal Synchronization */
#define LCDC_LCDCFG5_MODE_Pos 8
#define LCDC_LCDCFG5_MODE_Msk (0x3u << LCDC_LCDCFG5_MODE_Pos) /**< \brief (LCDC_LCDCFG5) LCD Controller Output Mode */
#define   LCDC_LCDCFG5_MODE_OUTPUT_12BPP (0x0u << 8) /**< \brief (LCDC_LCDCFG5) LCD output mode is set to 12 bits per pixel */
#define   LCDC_LCDCFG5_MODE_OUTPUT_16BPP (0x1u << 8) /**< \brief (LCDC_LCDCFG5) LCD output mode is set to 16 bits per pixel */
#define   LCDC_LCDCFG5_MODE_OUTPUT_18BPP (0x2u << 8) /**< \brief (LCDC_LCDCFG5) LCD output mode is set to 18 bits per pixel */
#define   LCDC_LCDCFG5_MODE_OUTPUT_24BPP (0x3u << 8) /**< \brief (LCDC_LCDCFG5) LCD output mode is set to 24 bits per pixel */
#define LCDC_LCDCFG5_VSPSU (0x1u << 12) /**< \brief (LCDC_LCDCFG5) LCD Controller Vertical Synchronization Pulse Setup Configuration */
#define LCDC_LCDCFG5_VSPHO (0x1u << 13) /**< \brief (LCDC_LCDCFG5) LCD Controller Vertical Synchronization Pulse Hold Configuration */
#define LCDC_LCDCFG5_GUARDTIME_Pos 16
#define LCDC_LCDCFG5_GUARDTIME_Msk (0x1fu << LCDC_LCDCFG5_GUARDTIME_Pos) /**< \brief (LCDC_LCDCFG5) LCD DISPLAY Guard Time */
#define LCDC_LCDCFG5_GUARDTIME(value) ((LCDC_LCDCFG5_GUARDTIME_Msk & ((value) << LCDC_LCDCFG5_GUARDTIME_Pos)))
/* -------- LCDC_LCDCFG6 : (LCDC Offset: 0x00000018) LCD Controller Configuration Register 6 -------- */
#define LCDC_LCDCFG6_PWMPS_Pos 0
#define LCDC_LCDCFG6_PWMPS_Msk (0x7u << LCDC_LCDCFG6_PWMPS_Pos) /**< \brief (LCDC_LCDCFG6) PWM Clock Prescaler */
#define LCDC_LCDCFG6_PWMPS(value) ((LCDC_LCDCFG6_PWMPS_Msk & ((value) << LCDC_LCDCFG6_PWMPS_Pos)))
#define LCDC_LCDCFG6_PWMPOL (0x1u << 4) /**< \brief (LCDC_LCDCFG6) LCD Controller PWM Signal Polarity */
#define LCDC_LCDCFG6_PWMCVAL_Pos 8
#define LCDC_LCDCFG6_PWMCVAL_Msk (0xffu << LCDC_LCDCFG6_PWMCVAL_Pos) /**< \brief (LCDC_LCDCFG6) LCD Controller PWM Compare Value */
#define LCDC_LCDCFG6_PWMCVAL(value) ((LCDC_LCDCFG6_PWMCVAL_Msk & ((value) << LCDC_LCDCFG6_PWMCVAL_Pos)))
/* -------- LCDC_LCDEN : (LCDC Offset: 0x00000020) LCD Controller Enable Register -------- */
#define LCDC_LCDEN_CLKEN (0x1u << 0) /**< \brief (LCDC_LCDEN) LCD Controller Pixel Clock Enable */
#define LCDC_LCDEN_SYNCEN (0x1u << 1) /**< \brief (LCDC_LCDEN) LCD Controller Horizontal and Vertical Synchronization Enable */
#define LCDC_LCDEN_DISPEN (0x1u << 2) /**< \brief (LCDC_LCDEN) LCD Controller DISP Signal Enable */
#define LCDC_LCDEN_PWMEN (0x1u << 3) /**< \brief (LCDC_LCDEN) LCD Controller Pulse Width Modulation Enable */
/* -------- LCDC_LCDSR : (LCDC Offset: 0x00000028) LCD Controller Status Register -------- */
#define LCDC_LCDSR_CLKSTS (0x1u << 0) /**< \brief (LCDC_LCDSR) Clock Status */
#define LCDC_LCDSR_LCDSTS (0x1u << 1) /**< \brief (LCDC_LCDSR) LCD Controller Synchronization status */
#define LCDC_LCDSR_DISPSTS (0x1u << 2) /**< \brief (LCDC_LCDSR) LCD Controller DISP Signal Status */
#define LCDC_LCDSR_PWMSTS (0x1u << 3) /**< \brief (LCDC_LCDSR) LCD Controller PWM Signal Status */
#define LCDC_LCDSR_SIPSTS (0x1u << 4) /**< \brief (LCDC_LCDSR) Synchronization In Progress */

/** Frequency of the board main oscillator */
#define BOARD_MAINOSC           12000000

/** Master clock frequency (when using board_lowlevel.c) */
#define BOARD_MCK                ((unsigned long)((BOARD_MAINOSC / 3 / 2 / 3) * 200 ))

/** Display width in pixels. */
#define BOARD_LCD_WIDTH             480 // 480 // 800 // 320 // 800 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module
/** Display height in pixels. */
#define BOARD_LCD_HEIGHT            272 // 272 // 480 // 240 // 480 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module

/** Display interface width in bits. */
#define BOARD_LCD_IFWIDTH           16
/** Frame size in words (height * width * bpp / 32) */
#define BOARD_LCD_FRAMESIZE         (BOARD_LCD_WIDTH * BOARD_LCD_HEIGHT * BOARD_LCD_IFWIDTH / 32)

/** Vertical front porch in number of lines. */
#define BOARD_LCD_TIMING_VFP        2 // 2 // 7 //16 // 22 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module
/** Vertical back porch in number of lines. */
#define BOARD_LCD_TIMING_VBP        2 // 2 // 23 // 8 // 21 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module
/** Vertical pulse width in number of lines. */
#define BOARD_LCD_TIMING_VPW        10 // 10 // 1 // 10 // 2 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module
/** Horizontal front porch in LCDDOTCLK cycles. */
#define BOARD_LCD_TIMING_HFP        2 // 2 // 16 // 51 // 64 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module
/** Horizontal back porch in LCDDOTCLK cycles. */
#define BOARD_LCD_TIMING_HBP        2 // 2 // 46 // 27 // 64 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module
/** Horizontal pulse width in LCDDOTCLK cycles. */
#define BOARD_LCD_TIMING_HPW        41 // 41 // 1 // 41 // 128 -- Current Active | ChipworkX Display | 7in CapTouch | T35 | Original board Display Module

/** Frame rate in Hz. */
#define BOARD_LCD_FRAMERATE         40

/** Pixel clock rate in Hz (HS period * VS period * BOARD_LCD_FRAMERATE). */
#define BOARD_LCD_PIXELCLOCK        ((BOARD_LCD_TIMING_HPW+BOARD_LCD_TIMING_HBP+BOARD_LCD_WIDTH+BOARD_LCD_TIMING_HFP)\
                                    *(BOARD_LCD_TIMING_VPW+BOARD_LCD_TIMING_VBP+BOARD_LCD_HEIGHT+BOARD_LCD_TIMING_VFP)\
                                    *BOARD_LCD_FRAMERATE)


// #define AT91_LCDD0		_P(C,0)
// #define AT91_LCDD1		_P(C,1)
// #define AT91_LCDD2		_P(C,2)
// #define AT91_LCDD3		_P(C,3)
// #define AT91_LCDD4		_P(C,4)
// #define AT91_LCDD5		_P(C,5)
// #define AT91_LCDD6		_P(C,6)
// #define AT91_LCDD7		_P(C,7)
// #define AT91_LCDD8		_P(C,8)
// #define AT91_LCDD9		_P(C,9)
// #define AT91_LCDD10		_P(C,10)
// #define AT91_LCDD11		_P(C,11)
// #define AT91_LCDD12		_P(C,12)
// #define AT91_LCDD13		_P(C,13)
// #define AT91_LCDD14		_P(C,14)
// #define AT91_LCDD15		_P(C,15)
// #define AT91_LCDDISP	_P(C,24)
// #define AT91_LCDPWM		_P(C,26)
// #define AT91_LCDVSYNC	_P(C,27)
// #define AT91_LCDHSYNC	_P(C,28)
// #define AT91_LCDDEN		_P(C,29)
// #define AT91_LCDPCK		_P(C,30)

// static const uint8_t LCDPins[] =
// {
    // AT91_LCDD0,
    // AT91_LCDD1,
    // AT91_LCDD2,
    // AT91_LCDD3,
    // AT91_LCDD4,
    // AT91_LCDD5,
    // AT91_LCDD6,
    // AT91_LCDD7,
    // AT91_LCDD8,
    // AT91_LCDD9,
    // AT91_LCDD10,
    // AT91_LCDD11,
    // AT91_LCDD12,
    // AT91_LCDD13,
    // AT91_LCDD14,
    // AT91_LCDD15,
    // AT91_LCDDISP,
    // AT91_LCDPWM,
    // AT91_LCDVSYNC,
    // AT91_LCDHSYNC,
    // AT91_LCDDEN,
    // AT91_LCDPCK,
// };

const uint8_t characters[129][5] = {
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,

0x00,0x00,0x00,0x00,0x00, /* Espace	0x20 */
0x00,0x00,0x4f,0x00,0x00, /* ! */
0x00,0x07,0x00,0x07,0x00, /* " */
0x14,0x7f,0x14,0x7f,0x14, /* # */
0x24,0x2a,0x7f,0x2a,0x12, /* 0x */
0x23,0x13,0x08,0x64,0x62, /* % */
0x36,0x49,0x55,0x22,0x20, /* & */
0x00,0x05,0x03,0x00,0x00, /* ' */
0x00,0x1c,0x22,0x41,0x00, /* ( */
0x00,0x41,0x22,0x1c,0x00, /* ) */
0x14,0x08,0x3e,0x08,0x14, /* // */
0x08,0x08,0x3e,0x08,0x08, /* + */
0x50,0x30,0x00,0x00,0x00, /* , */
0x08,0x08,0x08,0x08,0x08, /* - */
0x00,0x60,0x60,0x00,0x00, /* . */
0x20,0x10,0x08,0x04,0x02, /* / */
0x3e,0x51,0x49,0x45,0x3e, /* 0		0x30 */
0x00,0x42,0x7f,0x40,0x00, /* 1 */
0x42,0x61,0x51,0x49,0x46, /* 2 */
0x21,0x41,0x45,0x4b,0x31, /* 3 */
0x18,0x14,0x12,0x7f,0x10, /* 4 */
0x27,0x45,0x45,0x45,0x39, /* 5 */
0x3c,0x4a,0x49,0x49,0x30, /* 6 */
0x01,0x71,0x09,0x05,0x03, /* 7 */
0x36,0x49,0x49,0x49,0x36, /* 8 */
0x06,0x49,0x49,0x29,0x1e, /* 9 */
0x00,0x36,0x36,0x00,0x00, /* : */
0x00,0x56,0x36,0x00,0x00, /* ; */
0x08,0x14,0x22,0x41,0x00, /* < */
0x14,0x14,0x14,0x14,0x14, /* = */
0x00,0x41,0x22,0x14,0x08, /* > */
0x02,0x01,0x51,0x09,0x06, /* ? */
0x3e,0x41,0x5d,0x55,0x1e, /* @		0x40 */
0x7e,0x11,0x11,0x11,0x7e, /* A */
0x7f,0x49,0x49,0x49,0x36, /* B */
0x3e,0x41,0x41,0x41,0x22, /* C */
0x7f,0x41,0x41,0x22,0x1c, /* D */
0x7f,0x49,0x49,0x49,0x41, /* E */
0x7f,0x09,0x09,0x09,0x01, /* F */
0x3e,0x41,0x49,0x49,0x7a, /* G */
0x7f,0x08,0x08,0x08,0x7f, /* H */
0x00,0x41,0x7f,0x41,0x00, /* I */
0x20,0x40,0x41,0x3f,0x01, /* J */
0x7f,0x08,0x14,0x22,0x41, /* K */
0x7f,0x40,0x40,0x40,0x40, /* L */
0x7f,0x02,0x0c,0x02,0x7f, /* M */
0x7f,0x04,0x08,0x10,0x7f, /* N */
0x3e,0x41,0x41,0x41,0x3e, /* O */
0x7f,0x09,0x09,0x09,0x06, /* P		0x50 */
0x3e,0x41,0x51,0x21,0x5e, /* Q */
0x7f,0x09,0x19,0x29,0x46, /* R */
0x26,0x49,0x49,0x49,0x32, /* S */
0x01,0x01,0x7f,0x01,0x01, /* T */
0x3f,0x40,0x40,0x40,0x3f, /* U */
0x1f,0x20,0x40,0x20,0x1f, /* V */
0x3f,0x40,0x38,0x40,0x3f, /* W */
0x63,0x14,0x08,0x14,0x63, /* X */
0x07,0x08,0x70,0x08,0x07, /* Y */
0x61,0x51,0x49,0x45,0x43, /* Z */
0x00,0x7f,0x41,0x41,0x00, /* [ */
0x02,0x04,0x08,0x10,0x20, /* \ */
0x00,0x41,0x41,0x7f,0x00, /* ] */
0x04,0x02,0x01,0x02,0x04, /* ^ */
0x40,0x40,0x40,0x40,0x40, /* _ */
0x00,0x00,0x03,0x05,0x00, /* `		0x60 */
0x20,0x54,0x54,0x54,0x78, /* a */
0x7F,0x44,0x44,0x44,0x38, /* b */
0x38,0x44,0x44,0x44,0x44, /* c */
0x38,0x44,0x44,0x44,0x7f, /* d */
0x38,0x54,0x54,0x54,0x18, /* e */
0x04,0x04,0x7e,0x05,0x05, /* f */
0x08,0x54,0x54,0x54,0x3c, /* g */
0x7f,0x08,0x04,0x04,0x78, /* h */
0x00,0x44,0x7d,0x40,0x00, /* i */
0x20,0x40,0x44,0x3d,0x00, /* j */
0x7f,0x10,0x28,0x44,0x00, /* k */
0x00,0x41,0x7f,0x40,0x00, /* l */
0x7c,0x04,0x7c,0x04,0x78, /* m */
0x7c,0x08,0x04,0x04,0x78, /* n */
0x38,0x44,0x44,0x44,0x38, /* o */
0x7c,0x14,0x14,0x14,0x08, /* p		0x70 */
0x08,0x14,0x14,0x14,0x7c, /* q */
0x7c,0x08,0x04,0x04,0x00, /* r */
0x48,0x54,0x54,0x54,0x24, /* s */
0x04,0x04,0x3f,0x44,0x44, /* t */
0x3c,0x40,0x40,0x20,0x7c, /* u */
0x1c,0x20,0x40,0x20,0x1c, /* v */
0x3c,0x40,0x30,0x40,0x3c, /* w */
0x44,0x28,0x10,0x28,0x44, /* x */
0x0c,0x50,0x50,0x50,0x3c, /* y */
0x44,0x64,0x54,0x4c,0x44, /* z */
0x08,0x36,0x41,0x41,0x00, /* { */
0x00,0x00,0x77,0x00,0x00, /* | */
0x00,0x41,0x41,0x36,0x08, /* } */
0x08,0x08,0x2a,0x1c,0x08, /* <- */
0x08,0x1c,0x2a,0x08,0x08, /* -> */
0xff,0xff,0xff,0xff,0xff, /* 	 	0x80 */
};

const PALETTEENTRY_LCD c_rgbPalette[256] =
{
    { 0x00, 0x00, 0x00, 0 },    /* 0 Sys Black      gray 0  */
    { 0x80, 0x00, 0x00, 0 },    /* 1 Sys Dk Red  */
    { 0x00, 0x80, 0x00, 0 },    /* 2 Sys Dk Green  */
    { 0x80, 0x80, 0x00, 0 },    /* 3 Sys Dk Yellow  */
    { 0x00, 0x00, 0x80, 0 },    /* 4 Sys Dk Blue  */
    { 0x80, 0x00, 0x80, 0 },    /* 5 Sys Dk Violet  */
    { 0x00, 0x80, 0x80, 0 },    /* 6 Sys Dk Cyan  */
    { 0xc0, 0xc0, 0xc0, 0 },    /* 7 Sys Lt Grey    gray 192  */
    { 0xc0, 0xdc, 0xc0, 0 },    /* 8 Sys 8  */
    { 0xa6, 0xca, 0xf0, 0 },    /* 9 Sys 9 (the first 10 are fixed by Windows)  */
    { 0x04, 0x04, 0x04, 0 },    /* 10       gray 4  */
    { 0x08, 0x08, 0x08, 0 },    /* 11       gray 8  */
    { 0x0c, 0x0c, 0x0c, 0 },    /* 12       gray 12  */
    { 0x11, 0x11, 0x11, 0 },    /* 13       gray 17  */
    { 0x16, 0x16, 0x16, 0 },    /* 14       gray 22  */
    { 0x1c, 0x1c, 0x1c, 0 },    /* 15       gray 28  */
    { 0x22, 0x22, 0x22, 0 },    /* 16       gray 34  */
    { 0x29, 0x29, 0x29, 0 },    /* 17       gray 41  */
    { 0x55, 0x55, 0x55, 0 },    /* 18 swapped so inversions look good       gray 85  */
    { 0x4d, 0x4d, 0x4d, 0 },    /* 19 swapped so inversions look good       gray 77  */
    { 0x42, 0x42, 0x42, 0 },    /* 20 swapped so inversions look good       gray 66  */
    { 0x39, 0x39, 0x39, 0 },    /* 21 swapped so inversions look good       gray 57  */
    { 0xFF, 0x7C, 0x80, 0 },    /* 22 R255 G124 B128  */
    { 0xFF, 0x50, 0x50, 0 },    /* 23 R255 G80  B80  */
    { 0xD6, 0x00, 0x93, 0 },    /* 24 R214 G0   B147  */
    { 0xCC, 0xEC, 0xFF, 0 },    /* 25 R204 G236 B255  */
    { 0xEF, 0xD6, 0xC6, 0 },    /* 26 R239 G214 B198  */
    { 0xE7, 0xE7, 0xD6, 0 },    /* 27 R231 G231 B214  */
    { 0xAD, 0xA9, 0x90, 0 },    /* 28 R173 G169 B144  */
    { 0x33, 0x00, 0x00, 0 },    /* 29  */
    { 0x66, 0x00, 0x00, 0 },    /* 30  */
    { 0x99, 0x00, 0x00, 0 },    /* 31  */
    { 0xcc, 0x00, 0x00, 0 },    /* 32  */
    { 0x00, 0x33, 0x00, 0 },    /* 33  */
    { 0x33, 0x33, 0x00, 0 },    /* 34  */
    { 0x66, 0x33, 0x00, 0 },    /* 35  */
    { 0x99, 0x33, 0x00, 0 },    /* 36  */
    { 0xcc, 0x33, 0x00, 0 },    /* 37  */
    { 0xff, 0x33, 0x00, 0 },    /* 38  */
    { 0x00, 0x66, 0x00, 0 },    /* 39  */
    { 0x33, 0x66, 0x00, 0 },    /* 40  */
    { 0x66, 0x66, 0x00, 0 },    /* 41  */
    { 0x99, 0x66, 0x00, 0 },    /* 42  */
    { 0xcc, 0x66, 0x00, 0 },    /* 43  */
    { 0xff, 0x66, 0x00, 0 },    /* 44  */
    { 0x00, 0x99, 0x00, 0 },    /* 45  */
    { 0x33, 0x99, 0x00, 0 },    /* 46  */
    { 0x66, 0x99, 0x00, 0 },    /* 47  */
    { 0x99, 0x99, 0x00, 0 },    /* 48  */
    { 0xcc, 0x99, 0x00, 0 },    /* 49  */
    { 0xff, 0x99, 0x00, 0 },    /* 50  */
    { 0x00, 0xcc, 0x00, 0 },    /* 51  */
    { 0x33, 0xcc, 0x00, 0 },    /* 52  */
    { 0x66, 0xcc, 0x00, 0 },    /* 53  */
    { 0x99, 0xcc, 0x00, 0 },    /* 54  */
    { 0xcc, 0xcc, 0x00, 0 },    /* 55  */
    { 0xff, 0xcc, 0x00, 0 },    /* 56  */
    { 0x66, 0xff, 0x00, 0 },    /* 57  */
    { 0x99, 0xff, 0x00, 0 },    /* 58  */
    { 0xcc, 0xff, 0x00, 0 },    /* 59  */
    { 0x00, 0x00, 0x33, 0 },    /* 60  */
    { 0x33, 0x00, 0x33, 0 },    /* 61  */
    { 0x66, 0x00, 0x33, 0 },    /* 62  */
    { 0x99, 0x00, 0x33, 0 },    /* 63  */
    { 0xcc, 0x00, 0x33, 0 },    /* 64  */
    { 0xff, 0x00, 0x33, 0 },    /* 65  */
    { 0x00, 0x33, 0x33, 0 },    /* 66  */
    { 0x33, 0x33, 0x33, 0 },    /* 67       gray 51  */
    { 0x66, 0x33, 0x33, 0 },    /* 68  */
    { 0x99, 0x33, 0x33, 0 },    /* 69  */
    { 0xcc, 0x33, 0x33, 0 },    /* 70  */
    { 0xff, 0x33, 0x33, 0 },    /* 71  */
    { 0x00, 0x66, 0x33, 0 },    /* 72  */
    { 0x33, 0x66, 0x33, 0 },    /* 73  */
    { 0x66, 0x66, 0x33, 0 },    /* 74  */
    { 0x99, 0x66, 0x33, 0 },    /* 75  */
    { 0xcc, 0x66, 0x33, 0 },    /* 76  */
    { 0xff, 0x66, 0x33, 0 },    /* 77  */
    { 0x00, 0x99, 0x33, 0 },    /* 78  */
    { 0x33, 0x99, 0x33, 0 },    /* 79  */
    { 0x66, 0x99, 0x33, 0 },    /* 80  */
    { 0x99, 0x99, 0x33, 0 },    /* 81  */
    { 0xcc, 0x99, 0x33, 0 },    /* 82  */
    { 0xff, 0x99, 0x33, 0 },    /* 83  */
    { 0x00, 0xcc, 0x33, 0 },    /* 84  */
    { 0x33, 0xcc, 0x33, 0 },    /* 85  */
    { 0x66, 0xcc, 0x33, 0 },    /* 86  */
    { 0x99, 0xcc, 0x33, 0 },    /* 87  */
    { 0xcc, 0xcc, 0x33, 0 },    /* 88  */
    { 0xff, 0xcc, 0x33, 0 },    /* 89  */
    { 0x33, 0xff, 0x33, 0 },    /* 90  */
    { 0x66, 0xff, 0x33, 0 },    /* 91  */
    { 0x99, 0xff, 0x33, 0 },    /* 92  */
    { 0xcc, 0xff, 0x33, 0 },    /* 93  */
    { 0xff, 0xff, 0x33, 0 },    /* 94  */
    { 0x00, 0x00, 0x66, 0 },    /* 95  */
    { 0x33, 0x00, 0x66, 0 },    /* 96  */
    { 0x66, 0x00, 0x66, 0 },    /* 97  */
    { 0x99, 0x00, 0x66, 0 },    /* 98  */
    { 0xcc, 0x00, 0x66, 0 },    /* 99  */
    { 0xff, 0x00, 0x66, 0 },    /* 100  */
    { 0x00, 0x33, 0x66, 0 },    /* 101  */
    { 0x33, 0x33, 0x66, 0 },    /* 102  */
    { 0x66, 0x33, 0x66, 0 },    /* 103  */
    { 0x99, 0x33, 0x66, 0 },    /* 104  */
    { 0xcc, 0x33, 0x66, 0 },    /* 105  */
    { 0xff, 0x33, 0x66, 0 },    /* 106  */
    { 0x00, 0x66, 0x66, 0 },    /* 107  */
    { 0x33, 0x66, 0x66, 0 },    /* 108  */
    { 0x66, 0x66, 0x66, 0 },    /* 109      gray 102  */
    { 0x99, 0x66, 0x66, 0 },    /* 110  */
    { 0xcc, 0x66, 0x66, 0 },    /* 111  */
    { 0x00, 0x99, 0x66, 0 },    /* 112  */
    { 0x33, 0x99, 0x66, 0 },    /* 113  */
    { 0x66, 0x99, 0x66, 0 },    /* 114  */
    { 0x99, 0x99, 0x66, 0 },    /* 115  */
    { 0xcc, 0x99, 0x66, 0 },    /* 116  */
    { 0xff, 0x99, 0x66, 0 },    /* 117  */
    { 0x00, 0xcc, 0x66, 0 },    /* 118  */
    { 0x33, 0xcc, 0x66, 0 },    /* 119  */
    { 0x99, 0xcc, 0x66, 0 },    /* 120  */
    { 0xcc, 0xcc, 0x66, 0 },    /* 121  */
    { 0xff, 0xcc, 0x66, 0 },    /* 122  */
    { 0x00, 0xff, 0x66, 0 },    /* 123  */
    { 0x33, 0xff, 0x66, 0 },    /* 124  */
    { 0x99, 0xff, 0x66, 0 },    /* 125  */
    { 0xcc, 0xff, 0x66, 0 },    /* 126  */
    { 0xff, 0x00, 0xcc, 0 },    /* 127  */
    { 0xcc, 0x00, 0xff, 0 },    /* 128  */
    { 0x00, 0x99, 0x99, 0 },    /* 129  */
    { 0x99, 0x33, 0x99, 0 },    /* 130  */
    { 0x99, 0x00, 0x99, 0 },    /* 131  */
    { 0xcc, 0x00, 0x99, 0 },    /* 132  */
    { 0x00, 0x00, 0x99, 0 },    /* 133  */
    { 0x33, 0x33, 0x99, 0 },    /* 134  */
    { 0x66, 0x00, 0x99, 0 },    /* 135  */
    { 0xcc, 0x33, 0x99, 0 },    /* 136  */
    { 0xff, 0x00, 0x99, 0 },    /* 137  */
    { 0x00, 0x66, 0x99, 0 },    /* 138  */
    { 0x33, 0x66, 0x99, 0 },    /* 139  */
    { 0x66, 0x33, 0x99, 0 },    /* 140  */
    { 0x99, 0x66, 0x99, 0 },    /* 141  */
    { 0xcc, 0x66, 0x99, 0 },    /* 142  */
    { 0xff, 0x33, 0x99, 0 },    /* 143  */
    { 0x33, 0x99, 0x99, 0 },    /* 144  */
    { 0x66, 0x99, 0x99, 0 },    /* 145  */
    { 0x99, 0x99, 0x99, 0 },    /* 146      gray 153  */
    { 0xcc, 0x99, 0x99, 0 },    /* 147  */
    { 0xff, 0x99, 0x99, 0 },    /* 148  */
    { 0x00, 0xcc, 0x99, 0 },    /* 149  */
    { 0x33, 0xcc, 0x99, 0 },    /* 150  */
    { 0x66, 0xcc, 0x66, 0 },    /* 151  */
    { 0x99, 0xcc, 0x99, 0 },    /* 152  */
    { 0xcc, 0xcc, 0x99, 0 },    /* 153  */
    { 0xff, 0xcc, 0x99, 0 },    /* 154  */
    { 0x00, 0xff, 0x99, 0 },    /* 155  */
    { 0x33, 0xff, 0x99, 0 },    /* 156  */
    { 0x66, 0xcc, 0x99, 0 },    /* 157  */
    { 0x99, 0xff, 0x99, 0 },    /* 158  */
    { 0xcc, 0xff, 0x99, 0 },    /* 159  */
    { 0xff, 0xff, 0x99, 0 },    /* 160  */
    { 0x00, 0x00, 0xcc, 0 },    /* 161  */
    { 0x33, 0x00, 0x99, 0 },    /* 162  */
    { 0x66, 0x00, 0xcc, 0 },    /* 163  */
    { 0x99, 0x00, 0xcc, 0 },    /* 164  */
    { 0xcc, 0x00, 0xcc, 0 },    /* 165  */
    { 0x00, 0x33, 0x99, 0 },    /* 166  */
    { 0x33, 0x33, 0xcc, 0 },    /* 167  */
    { 0x66, 0x33, 0xcc, 0 },    /* 168  */
    { 0x99, 0x33, 0xcc, 0 },    /* 169  */
    { 0xcc, 0x33, 0xcc, 0 },    /* 170  */
    { 0xff, 0x33, 0xcc, 0 },    /* 171  */
    { 0x00, 0x66, 0xcc, 0 },    /* 172  */
    { 0x33, 0x66, 0xcc, 0 },    /* 173  */
    { 0x66, 0x66, 0x99, 0 },    /* 174  */
    { 0x99, 0x66, 0xcc, 0 },    /* 175  */
    { 0xcc, 0x66, 0xcc, 0 },    /* 176  */
    { 0xff, 0x66, 0x99, 0 },    /* 177  */
    { 0x00, 0x99, 0xcc, 0 },    /* 178  */
    { 0x33, 0x99, 0xcc, 0 },    /* 179  */
    { 0x66, 0x99, 0xcc, 0 },    /* 180  */
    { 0x99, 0x99, 0xcc, 0 },    /* 181  */
    { 0xcc, 0x99, 0xcc, 0 },    /* 182  */
    { 0xff, 0x99, 0xcc, 0 },    /* 183  */
    { 0x00, 0xcc, 0xcc, 0 },    /* 184  */
    { 0x33, 0xcc, 0xcc, 0 },    /* 185  */
    { 0x66, 0xcc, 0xcc, 0 },    /* 186  */
    { 0x99, 0xcc, 0xcc, 0 },    /* 187  */
    { 0xcc, 0xcc, 0xcc, 0 },    /* 188      gray 204  */
    { 0xff, 0xcc, 0xcc, 0 },    /* 189  */
    { 0x00, 0xff, 0xcc, 0 },    /* 190  */
    { 0x33, 0xff, 0xcc, 0 },    /* 191  */
    { 0x66, 0xff, 0x99, 0 },    /* 192  */
    { 0x99, 0xff, 0xcc, 0 },    /* 193  */
    { 0xcc, 0xff, 0xcc, 0 },    /* 194  */
    { 0xff, 0xff, 0xcc, 0 },    /* 195  */
    { 0x33, 0x00, 0xcc, 0 },    /* 196  */
    { 0x66, 0x00, 0xff, 0 },    /* 197  */
    { 0x99, 0x00, 0xff, 0 },    /* 198  */
    { 0x00, 0x33, 0xcc, 0 },    /* 199  */
    { 0x33, 0x33, 0xff, 0 },    /* 200  */
    { 0x66, 0x33, 0xff, 0 },    /* 201  */
    { 0x99, 0x33, 0xff, 0 },    /* 202  */
    { 0xcc, 0x33, 0xff, 0 },    /* 203  */
    { 0xff, 0x33, 0xff, 0 },    /* 204  */
    { 0x00, 0x66, 0xff, 0 },    /* 205  */
    { 0x33, 0x66, 0xff, 0 },    /* 206  */
    { 0x66, 0x66, 0xcc, 0 },    /* 207  */
    { 0x99, 0x66, 0xff, 0 },    /* 208  */
    { 0xcc, 0x66, 0xff, 0 },    /* 209  */
    { 0xff, 0x66, 0xcc, 0 },    /* 210  */
    { 0x00, 0x99, 0xff, 0 },    /* 211  */
    { 0x33, 0x99, 0xff, 0 },    /* 212  */
    { 0x66, 0x99, 0xff, 0 },    /* 213  */
    { 0x99, 0x99, 0xff, 0 },    /* 214  */
    { 0xcc, 0x99, 0xff, 0 },    /* 215  */
    { 0xff, 0x99, 0xff, 0 },    /* 216  */
    { 0x00, 0xcc, 0xff, 0 },    /* 217  */
    { 0x33, 0xcc, 0xff, 0 },    /* 218  */
    { 0x66, 0xcc, 0xff, 0 },    /* 219  */
    { 0x99, 0xcc, 0xff, 0 },    /* 220  */
    { 0xcc, 0xcc, 0xff, 0 },    /* 221  */
    { 0xff, 0xcc, 0xff, 0 },    /* 222  */
    { 0x33, 0xff, 0xff, 0 },    /* 223  */
    { 0x66, 0xff, 0xcc, 0 },    /* 224  */
    { 0x99, 0xff, 0xff, 0 },    /* 225  */
    { 0xcc, 0xff, 0xff, 0 },    /* 226  */
    { 0xff, 0x66, 0x66, 0 },    /* 227  */
    { 0x66, 0xff, 0x66, 0 },    /* 228  */
    { 0xff, 0xff, 0x66, 0 },    /* 229  */
    { 0x66, 0x66, 0xff, 0 },    /* 230  */
    { 0xff, 0x66, 0xff, 0 },    /* 231  */
    { 0x66, 0xff, 0xff, 0 },    /* 232  */
    { 0xA5, 0x00, 0x21, 0 },    /* 233      R165 G0 B33  */
    { 0x5f, 0x5f, 0x5f, 0 },    /* 234      gray 95  */
    { 0x77, 0x77, 0x77, 0 },    /* 235      gray 119  */
    { 0x86, 0x86, 0x86, 0 },    /* 236      gray 134  */
    { 0x96, 0x96, 0x96, 0 },    /* 237      gray 150  */
    { 0xcb, 0xcb, 0xcb, 0 },    /* 238      gray 203  */
    { 0xb2, 0xb2, 0xb2, 0 },    /* 239      gray 178  */
    { 0xd7, 0xd7, 0xd7, 0 },    /* 240      gray 215  */
    { 0xdd, 0xdd, 0xdd, 0 },    /* 241      gray 221  */
    { 0xe3, 0xe3, 0xe3, 0 },    /* 242      gray 227  */
    { 0xea, 0xea, 0xea, 0 },    /* 243      gray 234  */
    { 0xf1, 0xf1, 0xf1, 0 },    /* 244      gray 241  */
    { 0xf8, 0xf8, 0xf8, 0 },    /* 245      gray 248  */
    { 0xff, 0xfb, 0xf0, 0 },    /* 246 Sys Reserved  */
    { 0xa0, 0xa0, 0xa4, 0 },    /* 247 Sys Reserved  */
    { 0x80, 0x80, 0x80, 0 },    /* 248 Sys Lt Gray  gray 128  */
    { 0xff, 0x00, 0x00, 0 },    /* 249 Sys Red  */
    { 0x00, 0xff, 0x00, 0 },    /* 250 Sys Green  */
    { 0xff, 0xff, 0x00, 0 },    /* 251 Sys Yellow  */
    { 0x00, 0x00, 0xff, 0 },    /* 252 Sys Blue  */
    { 0xff, 0x00, 0xff, 0 },    /* 253 Sys Violet  */
    { 0x00, 0xff, 0xff, 0 },    /* 254 Sys Cyan  */
    { 0xff, 0xff, 0xff, 0 }     /* 255 Sys White     gray 255  */
};

//////////////////////////////////////////////////////////////////////////////
// AT91_LCDC_Driver
//

#define VIDEO_RAM_SIZE                  (800*600*2) // Maximum LCD screen size times bytes per pixel

#define LCD_MAX_ROW	                    32
#define LCD_MAX_COLUMN                  70

#define BITS_PER_PIXEL              16

/* More or less configurable parameters for LCDC controller*/
#define SIDSAFB_FIFO_SIZE		512
#define SIDSAFB_DMA_BURST_LEN	16
#define SIDSAFB_CRST_VAL        0xc8   // 0xda

typedef struct _LCDCDescriptor {
    uint32_t addr;
    uint32_t ctrl;
    uint32_t next;
} LCDCDescriptor;

/** CULT information */
typedef struct _CLUTInfo {
    uint8_t bpp;
    uint8_t nbColors;
} CLUTInfo;

/** LCDC General Layer information */
typedef struct _Layer {
    void* pBuffer;
    CLUTInfo clut;
    uint16_t  reserved;
    LCDCDescriptor dmaD;
} Layer;

//
// AT91_LCDC_Driver
//////////////////////////////////////////////////////////////////////////////

enum AT91_LCD_Rotation {
    rotateNormal_0,
    rotateCW_90,
    rotate_180,
    rotateCCW_90,
};

int64_t m_AT91_Display_ReservedVitualRamLocation[VIDEO_RAM_SIZE / 8];

uint32_t m_AT91_DisplayWidth = 0;
uint32_t m_AT91_DisplayHeight = 0;
uint32_t m_AT91_DisplayPixelClockRateKHz = 0;
uint32_t m_AT91_DisplayHorizontalSyncPulseWidth = 0;
uint32_t m_AT91_DisplayHorizontalFrontPorch = 0;
uint32_t m_AT91_DisplayHorizontalBackPorch = 0;
uint32_t m_AT91_DisplayVerticalSyncPulseWidth = 0;
uint32_t m_AT91_DisplayVerticalFrontPorch = 0;
uint32_t m_AT91_DisplayVerticalBackPorch = 0;
uint32_t m_AT91_Display_TextRow = 0;
uint32_t m_AT91_Display_TextColumn = 0;

bool m_AT91_DisplayOutputEnableIsFixed = false;
bool m_AT91_DisplayOutputEnablePolarity = false;
bool m_AT91_DisplayPixelPolarity = false;
bool m_AT91_DisplayHorizontalSyncPolarity = false;
bool m_AT91_DisplayVerticalSyncPolarity = false;
bool m_AT91_DisplayEnable = false;

uint16_t* m_AT91_Display_VituralRam;
uint8_t m_AT91_Display_TextBuffer[LCD_MAX_COLUMN][LCD_MAX_ROW];

AT91_LCD_Rotation m_AT91_Display_CurrentRotation = AT91_LCD_Rotation::rotateNormal_0;

bool AT91_Display_Initialize();
bool AT91_Display_Uninitialize();
bool AT91_Display_SetPinConfiguration(bool enable);

void AT91_Display_WriteFormattedChar(uint8_t c);
void AT91_Display_WriteChar(uint8_t c, int32_t row, int32_t col);
void AT91_Display_BitBltEx(int32_t x, int32_t y, int32_t width, int32_t height, uint32_t data[]);
void AT91_Display_PaintPixel(uint32_t x, uint32_t y, uint8_t c);
void AT91_Display_Paint8HorizontalPixels(uint32_t x, uint32_t y, uint8_t p);
void AT91_Display_TextEnterClearMode();
void AT91_Display_PrintChracter(uint32_t x, uint32_t y, uint8_t c);
void AT91_Display_TextShiftColUp();
void AT91_Display_Clear();
void AT91_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight);

int32_t AT91_Display_GetWidth();
int32_t AT91_Display_GetHeight();
int32_t AT91_Display_BitPerPixel();
uint32_t AT91_Display_GetPixelClockDivider();
int32_t AT91_Display_GetOrientation();
uint32_t* AT91_Display_GetFrameBuffer();

static TinyCLR_Display_Provider displayProvider;
static TinyCLR_Api_Info displayApi;

static Layer baseLayer;

void AT91_Display_SetBaseLayerDMA() {
    AT91SAM9X35_LCDC *lcd = (AT91SAM9X35_LCDC*)AT91C_BASE_LCDC;

    Layer *pointerToBaseLayer = &baseLayer;
    LCDCDescriptor *DMApointerForBase = &pointerToBaseLayer->dmaD;

    void *pBaseBuffer = pointerToBaseLayer->pBuffer;

    if (pBaseBuffer)
        pointerToBaseLayer->pBuffer = pBaseBuffer;
    else
        pBaseBuffer = pointerToBaseLayer->pBuffer;

    DMApointerForBase->addr = (uint32_t)((uint32_t*)m_AT91_Display_ReservedVitualRamLocation);
    DMApointerForBase->ctrl = 0x1;
    DMApointerForBase->next = (uint32_t)DMApointerForBase;

    lcd->LCDC_BASEADDR = DMApointerForBase->addr;
    lcd->LCDC_BASECTRL = 0x1;
    lcd->LCDC_BASENEXT = (uint32_t)DMApointerForBase;
    lcd->LCDC_BASECFG4 = 0x100;
    lcd->LCDC_BASECHER = 0x3;
}

static const AT91_Gpio_Pin g_at91_display_pins[] = AT91_DISPLAY_CONTROLLER_PINS;
static const AT91_Gpio_Pin g_at91_display_enable_pin = AT91_DISPLAY_ENABLE_PIN;
static const AT91_Gpio_Pin g_at91_display_backlight_pin = AT91_DISPLAY_BACKLIGHT_PIN;

bool AT91_Display_Initialize() {

    AT91SAM9X35_LCDC *lcd = (AT91SAM9X35_LCDC*)AT91C_BASE_LCDC;
    AT91_PMC &pmc = AT91::PMC();

    if (m_AT91_DisplayPixelClockRateKHz == 0) {
        return false;
    }
    // Enable the LCD clock
    pmc.EnablePeriphClock(AT91C_ID_LCDC);
    *((volatile uint32_t*)0xFFFFFC00) = 1 << 3; // Accessing PMC_SCER register to enable system clock.

    // Disable LCD interrupts
    lcd->LCDC_LCDIDR = 0xFFFFFFFF;

    // Configure channels
    lcd->LCDC_BASECFG0 = LCDC_BASECFG0_DLBO | LCDC_BASECFG0_BLEN_AHB_INCR16;
    lcd->LCDC_BASECFG1 = (3 << 4);

    // Configure channels
    lcd->LCDC_OVRCFG0 = LCDC_BASECFG0_DLBO | LCDC_BASECFG0_BLEN_AHB_INCR16;
    lcd->LCDC_OVRCFG1 = (3 << 4);
    lcd->LCDC_OVRCFG2 = (100 << 16) | 100; //(LCD_GetHeight() << 16) | LCD_GetWidth();
    lcd->LCDC_OVRCFG3 = (50 << 16) | 350; //(LCD_GetHeight() << 16) | LCD_GetWidth();

    // Configure channels
    lcd->LCDC_HEOCFG0 = LCDC_BASECFG0_DLBO | LCDC_BASECFG0_BLEN_AHB_INCR16;
    lcd->LCDC_HEOCFG1 = (3 << 4);
    lcd->LCDC_HEOCFG2 = (150 << 16) | 100; //(LCD_GetHeight() << 16) | LCD_GetWidth();
    lcd->LCDC_HEOCFG3 = (50 << 16) | 350; //(LCD_GetHeight() << 16) | LCD_GetWidth();

    lcd->LCDC_LCDCFG0 = LCDC_LCDCFG0_CLKDIV((AT91_SYSTEM_PERIPHERAL_CLOCK_HZ * 2) / (m_AT91_DisplayPixelClockRateKHz * 1000) - 2)
        | LCDC_LCDCFG0_CGDISHCR
        | LCDC_LCDCFG0_CGDISHEO
        | LCDC_LCDCFG0_CGDISOVR1
        | LCDC_LCDCFG0_CGDISBASE
        | LCDC_LCDCFG0_CLKPWMSEL
        | LCDC_LCDCFG0_CLKSEL
        | m_AT91_DisplayPixelPolarity;

    lcd->LCDC_LCDCFG1 = LCDC_LCDCFG1_VSPW(m_AT91_DisplayVerticalSyncPulseWidth - 1)
        | LCDC_LCDCFG1_HSPW(m_AT91_DisplayHorizontalSyncPulseWidth - 1);

    lcd->LCDC_LCDCFG2 = LCDC_LCDCFG2_VBPW(m_AT91_DisplayVerticalBackPorch)
        | LCDC_LCDCFG2_VFPW(m_AT91_DisplayVerticalFrontPorch - 1);

    lcd->LCDC_LCDCFG3 = LCDC_LCDCFG3_HBPW(m_AT91_DisplayHorizontalBackPorch - 1)
        | LCDC_LCDCFG3_HFPW(m_AT91_DisplayHorizontalFrontPorch - 1);

    lcd->LCDC_LCDCFG4 = LCDC_LCDCFG4_RPF(m_AT91_DisplayHeight - 1)
        | LCDC_LCDCFG4_PPL(m_AT91_DisplayWidth - 1);

    lcd->LCDC_LCDCFG5 = LCDC_LCDCFG5_GUARDTIME(30)
        | (0x01 << 8) // To select 16bpp // LCDC_LCDCFG5_MODE_OUTPUT_24BPP
        | LCDC_LCDCFG5_DISPDLY
        | LCDC_LCDCFG5_VSPDLYS
        | ((m_AT91_DisplayVerticalSyncPolarity ? 0 : 1) << 1)
        | (m_AT91_DisplayHorizontalSyncPolarity ? 0 : 1);

    lcd->LCDC_LCDCFG6 = LCDC_LCDCFG6_PWMCVAL(0xF0)
        | LCDC_LCDCFG6_PWMPOL
        | LCDC_LCDCFG6_PWMPS(6);

    // 2. Enable the Pixel Clock by writing one to the CLKEN field of the LCDC_LCDEN register.
    lcd->LCDC_LCDEN = LCDC_LCDEN_CLKEN;
    // 3. Poll CLKSTS field of the LCDC_LCDSR register to check that the clock is running.
    while (!(lcd->LCDC_LCDSR & LCDC_LCDSR_CLKSTS));

    // 4. Enable Horizontal and Vertical Synchronization by writing one to the SYNCEN field of the LCDC_LCDEN register.
    lcd->LCDC_LCDEN = LCDC_LCDEN_SYNCEN;
    // 5. Poll LCDSTS field of the LCDC_LCDSR register to check that the synchronization is up.
    while (!(lcd->LCDC_LCDSR & LCDC_LCDSR_LCDSTS));
    // 6. Enable the display power signal writing one to the DISPEN field of the LCDC_LCDEN register.
    lcd->LCDC_LCDEN = LCDC_LCDEN_DISPEN;
    // 7. Poll DISPSTS field of the LCDC_LCDSR register to check that the power signal is activated.
    while (!(lcd->LCDC_LCDSR & LCDC_LCDSR_DISPSTS));
    // 8. Enable backlight
    lcd->LCDC_LCDEN = LCDC_LCDEN_PWMEN;

    AT91_Display_SetBaseLayerDMA();


    AT91_Display_Clear();

    return true;
}

bool AT91_Display_Uninitialize() {
    AT91_PMC &pmc = AT91::PMC();

    AT91SAM9X35_LCDC *lcd = (AT91SAM9X35_LCDC*)AT91C_BASE_LCDC;

    lcd->LCDC_LCDEN &= ~(LCDC_LCDEN_CLKEN | LCDC_LCDEN_PWMEN);

    pmc.DisablePeriphClock(AT91C_ID_LCDC);

    m_AT91_DisplayEnable = false;

    return true;
}

//====================================================
void AT91_Display_WriteFormattedChar(uint8_t c) {
    if (m_AT91_DisplayEnable == false)
        return;

    if (c == '\f') {
        AT91_Display_Clear();
        AT91_Display_TextEnterClearMode();
        m_AT91_Display_TextColumn = 0;

        return;
    }

    if (c == '\r') {
        m_AT91_Display_TextColumn = 0;

        return;
    }
    if (c == '\n') {
        m_AT91_Display_TextColumn = 0;

        if (++m_AT91_Display_TextRow >= LCD_MAX_ROW) {
            m_AT91_Display_TextRow = LCD_MAX_ROW - 1;
            AT91_Display_TextShiftColUp();
        }
        // clean the new line
        for (c = 0; c < (LCD_MAX_COLUMN - 1); c++) {
            m_AT91_Display_TextBuffer[c][m_AT91_Display_TextRow] = ' ';
        }

        return;
    }

    AT91_Display_PrintChracter(m_AT91_Display_TextColumn * 6, m_AT91_Display_TextRow * 8, c);
    m_AT91_Display_TextBuffer[m_AT91_Display_TextColumn][m_AT91_Display_TextRow] = c;

    if (++m_AT91_Display_TextColumn >= (LCD_MAX_COLUMN - 1)) {
        m_AT91_Display_TextColumn = 0;

        if (++m_AT91_Display_TextRow >= LCD_MAX_ROW) {
            m_AT91_Display_TextRow = LCD_MAX_ROW - 1;
            AT91_Display_TextShiftColUp();
        }
        else {
            // clean the new line
            for (c = 0; c < LCD_MAX_COLUMN; c++) {
                m_AT91_Display_TextBuffer[c][m_AT91_Display_TextRow] = ' ';
            }
        }
    }
}
//=======================================================
void AT91_Display_PaintPixel(uint32_t x, uint32_t y, uint8_t c) {
    volatile uint16_t * loc;

    if (m_AT91_DisplayEnable == false)
        return;

    if (x >= m_AT91_DisplayWidth)
        return;
    if (y >= m_AT91_DisplayHeight)
        return;

    loc = m_AT91_Display_VituralRam + (y *m_AT91_DisplayWidth) + (x);

    if (c)
        *loc = 0x0fff;
    else
        *loc = 0;
}
//=======================================================
void AT91_Display_Paint8HorizontalPixels(uint32_t x, uint32_t y, uint8_t p) {
    if (m_AT91_DisplayEnable == false)
        return;

    for (int32_t i = 0; i < 8; i++) {
        if (p&(1 << i))
            AT91_Display_PaintPixel(x, y + i, 1);
        else
            AT91_Display_PaintPixel(x, y + i, 0);//clear
    }
}
//===========================================================
void AT91_Display_TextEnterClearMode() {
    uint32_t r, c;

    if (m_AT91_DisplayEnable == false)
        return;

    AT91_Display_Clear();
    m_AT91_Display_TextRow = 0;
    m_AT91_Display_TextColumn = 0;

    for (r = 0; r < LCD_MAX_ROW; r++) {
        for (c = 0; c < (LCD_MAX_COLUMN - 1); c++) {
            m_AT91_Display_TextBuffer[c][r] = '1';
        }
    }
}
//===========================================================
void AT91_Display_PrintChracter(uint32_t x, uint32_t y, uint8_t c) {
    uint8_t i;

    if (m_AT91_DisplayEnable == false)
        return;

    for (i = 0; i < 5; i++)
        AT91_Display_Paint8HorizontalPixels(x + i, y, characters[c][i]);

    AT91_Display_Paint8HorizontalPixels(x + i, y, 0);
}

void AT91_Display_TextShiftColUp() {
    uint32_t r, c;

    if (m_AT91_DisplayEnable == false)
        return;

    // refresh with new data
    AT91_Display_Clear();
    m_AT91_Display_TextRow = 0;
    m_AT91_Display_TextColumn = 0;

    for (r = 0; r < (LCD_MAX_ROW - 1); r++) {
        for (c = 0; c < LCD_MAX_COLUMN - 1; c++) {
            m_AT91_Display_TextBuffer[c][r] = m_AT91_Display_TextBuffer[c][r + 1];
            AT91_Display_WriteFormattedChar(m_AT91_Display_TextBuffer[c][r]);
        }
    }
}

void AT91_Display_Clear() {
    if (m_AT91_DisplayEnable == false)
        return;

    memset((uint32_t*)m_AT91_Display_VituralRam, 0, VIDEO_RAM_SIZE);
}

bool AT91_Display_SetPinConfiguration(bool enable) {
    if (enable) {
        for (uint32_t pin = 0; pin < SIZEOF_ARRAY(g_at91_display_pins); pin++) {
            if (!AT91_Gpio_OpenPin(g_at91_display_pins[pin].number))
                return false;

            AT91_Gpio_ConfigurePin(g_at91_display_pins[pin].number, AT91_Gpio_Direction::Input, g_at91_display_pins[pin].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
        }

        if (!AT91_Gpio_OpenPin(g_at91_display_enable_pin.number))
            return false;

        if (m_AT91_DisplayOutputEnableIsFixed) {
            AT91_Gpio_EnableOutputPin(g_at91_display_enable_pin.number, m_AT91_DisplayOutputEnablePolarity);
        }
        else {
            AT91_Gpio_ConfigurePin(g_at91_display_enable_pin.number, AT91_Gpio_Direction::Input, g_at91_display_enable_pin.peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
        }

        if (g_at91_display_backlight_pin.number != PIN_NONE) {
            if (!AT91_Gpio_OpenPin(g_at91_display_backlight_pin.number))
                return false;

            AT91_Gpio_EnableOutputPin(g_at91_display_backlight_pin.number, true);
        }
    }
    else {

        for (int32_t i = 0; i < SIZEOF_ARRAY(g_at91_display_pins); i++) {
            AT91_Gpio_ClosePin(g_at91_display_pins[i].number);
        }

        AT91_Gpio_ClosePin(g_at91_display_enable_pin.number);

        AT91_Gpio_ClosePin(g_at91_display_backlight_pin.number);
    }

    return true;
}
uint32_t* AT91_Display_GetFrameBuffer() {
    return (uint32_t*)m_AT91_Display_VituralRam;
}

int32_t AT91_Display_GetWidth() {
    int32_t width = m_AT91_DisplayWidth;
    int32_t height = m_AT91_DisplayHeight;

    AT91_Display_GetRotatedDimensions(&width, &height);

    return width;
}

int32_t AT91_Display_GetHeight() {
    int32_t width = m_AT91_DisplayWidth;
    int32_t height = m_AT91_DisplayHeight;

    AT91_Display_GetRotatedDimensions(&width, &height);

    return height;
}

uint32_t AT91_Display_GetPixelClockDivider() {
    return m_AT91_DisplayPixelClockRateKHz;
}

int32_t AT91_Display_GetOrientation() {
    return m_AT91_Display_CurrentRotation;
}

void  AT91_Display_MemCopy(void *dest, void *src, int32_t size) {
    const int32_t MEMCOPY_BYTES_ALIGNED = 8;

    uint64_t *from64 = (uint64_t *)src;
    uint64_t *to64 = (uint64_t *)dest;

    int32_t block = size / MEMCOPY_BYTES_ALIGNED;
    int32_t remainder = size % MEMCOPY_BYTES_ALIGNED;

    while (block > 0) {
        *to64++ = *from64++;
        block--;
    }

    if (remainder > 0) {
        uint8_t *from8 = (uint8_t *)from64;
        uint8_t *to8 = (uint8_t *)to64;

        while (remainder > 0) {
            *to8++ = *from8++;

            remainder--;
        }
    }
}

void AT91_Display_BitBltEx(int32_t x, int32_t y, int32_t width, int32_t height, uint32_t data[]) {

    int32_t xTo, yTo, xFrom, yFrom;
    int32_t xOffset = x;
    int32_t yOffset = y;
    uint16_t *from = (uint16_t *)data;
    uint16_t *to = (uint16_t *)m_AT91_Display_VituralRam;


    int32_t screenWidth = m_AT91_DisplayWidth;
    int32_t screenHeight = m_AT91_DisplayHeight;
    int32_t startPx, toAddition;

    if (m_AT91_DisplayEnable == false)
        return;

    switch (m_AT91_Display_CurrentRotation) {
    case AT91_LCD_Rotation::rotateNormal_0:

        if (xOffset == 0 && yOffset == 0 &&
            width == screenWidth && height == screenHeight) {
            AT91_Display_MemCopy(to, from, (screenWidth*screenHeight * 2));
        }
        else {
            for (yTo = yOffset; yTo < (yOffset + height); yTo++) {
                AT91_Display_MemCopy((void*)(to + yTo * screenWidth + xOffset), (void*)(from + yTo * screenWidth + xOffset), (width * 2));
            }
        }

        break;

    case AT91_LCD_Rotation::rotateCCW_90:

        startPx = yOffset * screenHeight;
        xFrom = xOffset + width;
        yTo = screenHeight - xOffset - width;
        xTo = yOffset;
        to += yTo * screenWidth + xTo;
        toAddition = screenWidth - height;

        for (; yTo < (screenHeight - xOffset); yTo++) {
            xFrom--;
            yFrom = startPx + xFrom;

            for (xTo = yOffset; xTo < (yOffset + height); xTo++) {
                *to++ = from[yFrom];
                yFrom += screenHeight;
            }

            to += toAddition;
        }

        break;

    case AT91_LCD_Rotation::rotateCW_90:

        startPx = (yOffset + height - 1) * screenHeight;
        xFrom = xOffset;

        yTo = xOffset;
        xTo = screenWidth - yOffset - height;
        to += yTo * screenWidth + xTo;
        toAddition = screenWidth - height;

        for (; yTo < (xOffset + width); yTo++) {
            yFrom = startPx + xFrom;

            for (xTo = screenWidth - yOffset - height; xTo < (screenWidth - yOffset); xTo++) {
                *to++ = from[yFrom];
                yFrom -= screenHeight;
            }

            to += toAddition;
            xFrom++;
        }

        break;

    case AT91_LCD_Rotation::rotate_180:

        xFrom = (yOffset + height - 1) * screenWidth + xOffset + width;

        yTo = screenHeight - yOffset - height;
        xTo = screenWidth - xOffset - width;
        to += yTo * screenWidth + xTo;
        toAddition = screenWidth - width;

        for (; yTo < (screenHeight - yOffset); yTo++) {

            for (xTo = screenWidth - xOffset - width; xTo < (screenWidth - xOffset); xTo++) {
                xFrom--;
                *to++ = from[xFrom];
            }

            to += toAddition;
            xFrom -= toAddition;
        }

        break;
    }

}

void AT91_Display_WriteChar(uint8_t c, int32_t row, int32_t col) {
    m_AT91_Display_TextRow = row;
    m_AT91_Display_TextColumn = col;
    AT91_Display_WriteFormattedChar(c);

}
void AT91_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight) {
    switch (m_AT91_Display_CurrentRotation) {
    case AT91_LCD_Rotation::rotateNormal_0:
    case AT91_LCD_Rotation::rotate_180:
        *screenWidth = m_AT91_DisplayWidth;
        *screenHeight = m_AT91_DisplayHeight;
        break;

    case AT91_LCD_Rotation::rotateCCW_90:
    case AT91_LCD_Rotation::rotateCW_90:
        *screenWidth = m_AT91_DisplayHeight;
        *screenHeight = m_AT91_DisplayWidth;
        break;
    }
}

TinyCLR_Result AT91_Display_Acquire(const TinyCLR_Display_Provider* self) {
    m_AT91_Display_CurrentRotation = AT91_LCD_Rotation::rotateNormal_0;

    m_AT91_Display_VituralRam = (uint16_t*)m_AT91_Display_ReservedVitualRamLocation;

    if (!AT91_Display_SetPinConfiguration(true)) {
        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Display_Release(const TinyCLR_Display_Provider* self) {
    AT91_Display_Uninitialize();

    AT91_Display_SetPinConfiguration(false);

    m_AT91_DisplayEnable = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Display_Enable(const TinyCLR_Display_Provider* self) {
    if (m_AT91_DisplayEnable || AT91_Display_Initialize()) {
        m_AT91_DisplayEnable = true;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91_Display_Disable(const TinyCLR_Display_Provider* self) {
    AT91_Display_Uninitialize();

    m_AT91_DisplayEnable = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Display_SetConfiguration(const TinyCLR_Display_Provider* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration) {
    if (dataFormat != TinyCLR_Display_DataFormat::Rgb565) return TinyCLR_Result::NotSupported;

    if (configuration != nullptr) {
        auto& cfg = *(const TinyCLR_Display_ParallelConfiguration*)configuration;

        m_AT91_DisplayWidth = width;
        m_AT91_DisplayHeight = height;

        m_AT91_DisplayOutputEnableIsFixed = cfg.DataEnableIsFixed;
        m_AT91_DisplayOutputEnablePolarity = cfg.DataEnablePolarity;
        m_AT91_DisplayPixelPolarity = cfg.PixelPolarity;

        m_AT91_DisplayPixelClockRateKHz = cfg.PixelClockRate / 1000;

        m_AT91_DisplayHorizontalSyncPolarity = cfg.HorizontalSyncPolarity;

        m_AT91_DisplayHorizontalSyncPulseWidth = cfg.HorizontalSyncPulseWidth;
        m_AT91_DisplayHorizontalFrontPorch = cfg.HorizontalFrontPorch;
        m_AT91_DisplayHorizontalBackPorch = cfg.HorizontalBackPorch;

        m_AT91_DisplayVerticalSyncPolarity = cfg.VerticalSyncPolarity;

        m_AT91_DisplayVerticalSyncPulseWidth = cfg.VerticalSyncPulseWidth;
        m_AT91_DisplayVerticalFrontPorch = cfg.VerticalFrontPorch;
        m_AT91_DisplayVerticalBackPorch = cfg.VerticalBackPorch;
    }

    return  TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Display_GetConfiguration(const TinyCLR_Display_Provider* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration) {
    dataFormat = TinyCLR_Display_DataFormat::Rgb565;
    width = m_AT91_DisplayWidth;
    height = m_AT91_DisplayHeight;

    if (configuration != nullptr) {
        auto& cfg = *(TinyCLR_Display_ParallelConfiguration*)configuration;

        cfg.DataEnableIsFixed = m_AT91_DisplayOutputEnableIsFixed;
        cfg.DataEnablePolarity = m_AT91_DisplayOutputEnablePolarity;
        cfg.PixelPolarity = m_AT91_DisplayPixelPolarity;

        cfg.PixelClockRate = m_AT91_DisplayPixelClockRateKHz * 1000;

        cfg.HorizontalSyncPolarity = m_AT91_DisplayHorizontalSyncPolarity;

        cfg.HorizontalSyncPulseWidth = m_AT91_DisplayHorizontalSyncPulseWidth;
        cfg.HorizontalFrontPorch = m_AT91_DisplayHorizontalFrontPorch;
        cfg.HorizontalBackPorch = m_AT91_DisplayHorizontalBackPorch;

        cfg.VerticalSyncPolarity = m_AT91_DisplayVerticalSyncPolarity;

        cfg.VerticalSyncPulseWidth = m_AT91_DisplayVerticalSyncPulseWidth;
        cfg.VerticalFrontPorch = m_AT91_DisplayVerticalFrontPorch;
        cfg.VerticalBackPorch = m_AT91_DisplayVerticalBackPorch;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data) {
    AT91_Display_BitBltEx(x, y, width, height, (uint32_t*)data);
    return TinyCLR_Result::Success;
}

TinyCLR_Result AT91_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer, size_t length) {
    for (size_t i = 0; i < length; i++)
        AT91_Display_WriteFormattedChar(buffer[i]);

    return TinyCLR_Result::Success;
}

TinyCLR_Display_DataFormat dataFormats[] = { TinyCLR_Display_DataFormat::Rgb565 };

TinyCLR_Result AT91_Display_GetCapabilities(const TinyCLR_Display_Provider* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount) {
    type = TinyCLR_Display_InterfaceType::Parallel;
    supportedDataFormatCount = SIZEOF_ARRAY(dataFormats);
    supportedDataFormats = dataFormats;

    return TinyCLR_Result::Success;
}

const TinyCLR_Api_Info* AT91_Display_GetApi() {
    displayProvider.Parent = &displayApi;
    displayProvider.Index = 0;
    displayProvider.Acquire = &AT91_Display_Acquire;
    displayProvider.Release = &AT91_Display_Release;
    displayProvider.Enable = &AT91_Display_Enable;
    displayProvider.Disable = &AT91_Display_Disable;
    displayProvider.SetConfiguration = &AT91_Display_SetConfiguration;
    displayProvider.GetConfiguration = &AT91_Display_GetConfiguration;
    displayProvider.GetCapabilities = &AT91_Display_GetCapabilities;
    displayProvider.DrawBuffer = &AT91_Display_DrawBuffer;
    displayProvider.WriteString = &AT91_Display_WriteString;

    displayApi.Author = "GHI Electronics, LLC";
    displayApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.DisplayProvider";
    displayApi.Type = TinyCLR_Api_Type::DisplayProvider;
    displayApi.Version = 0;
    displayApi.Count = 1;
    displayApi.Implementation = &displayProvider;

    return &displayApi;
}

void AT91_Display_Reset() {
    AT91_Display_Clear();

    if (m_AT91_DisplayEnable)
        AT91_Display_Release(&displayProvider);

    m_AT91_DisplayEnable = false;
}

#endif // INCLUDE_DISPLAY
