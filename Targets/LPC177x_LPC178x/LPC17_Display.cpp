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

#include <stdio.h>
#include <string.h>

#include "LPC17.h"

#define LCD_MAX_ROW	                32
#define LCD_MAX_COLUMN              70

#define BITS_PER_PIXEL              16

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

/***********************************************************************
 * _BIT(p) sets the bit at position "p"
 **********************************************************************/
#undef _BIT
#define _BIT(p)	(((uint32_t)(1)) << (p))

 /***********************************************************************
  * _BITMASK constructs a symbol with 'field_width' least significant bits set.
  **********************************************************************/
#undef _BITMASK
#define _BITMASK(field_width) (_BIT(field_width) - 1)

  /***********************************************************************
   * _SBF(p,v) sets the bit field starting at position "p" to value "v".
   **********************************************************************/
#undef _SBF
#define _SBF(p,v) (((uint32_t)(v)) << (p))

   /***********************************************************************
    * Color LCD controller horizontal axis plane control register definitions
    **********************************************************************/

    /* LCD controller horizontal axis plane control register pixels per line */
#define CLCDC_LCDTIMING0_PPL_WIDTH 6
#define CLCDC_LCDTIMING0_PPL(n) _SBF(2, (((n) / 16) - 1) & _BITMASK(CLCDC_LCDTIMING0_PPL_WIDTH))
/* LCD controller horizontal axis plane control register HSYNC pulse width */
#define CLCDC_LCDTIMING0_HSW_WIDTH 8
#define CLCDC_LCDTIMING0_HSW(n) _SBF(8, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING0_HSW_WIDTH))
/* LCD controller horizontal axis plane control register horizontal front porch */
#define CLCDC_LCDTIMING0_HFP_WIDTH 8
#define CLCDC_LCDTIMING0_HFP(n)	_SBF(16, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING0_HFP_WIDTH))
/* LCD controller horizontal axis plane control register horizontal back porch */
#define CLCDC_LCDTIMING0_HBP_WIDTH 8
#define CLCDC_LCDTIMING0_HBP(n)	_SBF(24, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING0_HBP_WIDTH))

/***********************************************************************
 * Color LCD controller vertical axis plane control register definitions
 **********************************************************************/

 /* LCD controller vertical axis plane control register lines per panel */
#define CLCDC_LCDTIMING1_LPP_WIDTH 10
#define CLCDC_LCDTIMING1_LPP(n)	_SBF(0, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING1_LPP_WIDTH))
/* LCD controller vertical axis plane control register VSYNC pulse width */
#define CLCDC_LCDTIMING1_VSW_WIDTH 6
#define CLCDC_LCDTIMING1_VSW(n)	_SBF(10, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING1_VSW_WIDTH))
/* LCD controller vertical axis plane control register vertical front porch */
#define CLCDC_LCDTIMING1_VFP_WIDTH 8
#define CLCDC_LCDTIMING1_VFP(n)	_SBF(16, (n) & _BITMASK(CLCDC_LCDTIMING1_VFP_WIDTH))
/* LCD controller vertical axis plane control register vertical back porch */
#define CLCDC_LCDTIMING1_VBP_WIDTH 8
#define CLCDC_LCDTIMING1_VBP(n)	_SBF(24, (n) & _BITMASK(CLCDC_LCDTIMING1_VBP_WIDTH))

/***********************************************************************
 * Color LCD controller clock and signal polarity control register definitions
 **********************************************************************/

 /* LCD controller clock and signal polarity control register panel clock divisor low*/
#define CLCDC_LCDTIMING2_PCD(n)	 CLCDC_LCDTIMING2_PCD_hi(n) | CLCDC_LCDTIMING2_PCD_lo(n)
#define CLCDC_LCDTIMING2_PCD_lo(n)	 (_SBF(0, ((n) - 2 )) & 0x0000001f)
/* LCD controller clock and signal polarity control register clock select */
#define CLCDC_LCDTIMING2_CLKSEL _BIT(5)
/* LCD controller clock and signal polarity control register AC bias pin frequency */
#define CLCDC_LCDTIMING2_ACB_WIDTH 5
#define CLCDC_LCDTIMING2_ACB(n)	_SBF(6, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING2_ACB_WIDTH))
/* LCD controller clock and signal polarity control register invert VSYNC */
#define CLCDC_LCDTIMING2_IVS    _BIT(11)
/* LCD controller clock and signal polarity control register invert HSYNC */
#define CLCDC_LCDTIMING2_IHS    _BIT(12)
/* LCD controller clock and signal polarity control register invert plane clock */
#define CLCDC_LCDTIMING2_IPC    _BIT(13)
/* LCD controller clock and signal polarity control register invert output enable */
#define CLCDC_LCDTIMING2_IOE    _BIT(14)
/* LCD controller clock and signal polarity control register clocks per line */
#define CLCDC_LCDTIMING2_CPL_WIDTH 10
#define CLCDC_LCDTIMING2_CPL(n)	_SBF(16, (n) & _BITMASK(CLCDC_LCDTIMING2_CPL_WIDTH))
/* LCD controller clock and signal polarity control register bypass pixel clock divider */
#define CLCDC_LCDTIMING2_BCD 	_BIT(26)
/* LCD controller clock and signal polarity control register panel clock divisor high*/
#define CLCDC_LCDTIMING2_PCD_hi(n)	 (_SBF(22,((n) - 2 )) & 0xf8000000)

/**********************************************************************
 * Color LCD Controller line end control register definitions
 *********************************************************************/

 /* Line End Signal Delay */
#define CLCDC_LCDTIMING3_LED_WIDTH 7
#define CLCDC_LCDTIMING3_LED(n) _SBF(0, ((n) - 1) & _BITMASK(CLCDC_LCDTIMING3_LED_WIDTH))
/* Line End Enable */
#define CLCDC_LCDTIMING3_LEE    _BIT(16)

/***********************************************************************
 * Color LCD controller control register definitions
 **********************************************************************/

 /* LCD control enable bit */
#define CLCDC_LCDCTRL_ENABLE    _BIT(0)
/* LCD control 1 bit per pixel bit field */
#define CLCDC_LCDCTRL_BPP1      _SBF(1, 0)
/* LCD control 2 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP2      _SBF(1, 1)
/* LCD control 4 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP4      _SBF(1, 2)
/* LCD control 8 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP8      _SBF(1, 3)
/* LCD control 16 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP16     _SBF(1, 4)
/* LCD control 24 bits per pixel bit field */
#define CLCDC_LCDCTRL_BPP24     _SBF(1, 5)
/* LCD control 16 bits (5:6:5 mode) per pixel bit field */
#define CLCDC_LCDCTRL_BPP16_565_MODE _SBF(1, 6)
/* LCD control 12 bits (4:4:4 mode) per pixel bit field */
#define CLCDC_LCDCTRL_BPP12_444_MODE _SBF(1, 7)
/* LCD control mono select bit */
#define CLCDC_LCDCTRL_BW_COLOR  _SBF(4, 0)
#define CLCDC_LCDCTRL_BW_MONO   _SBF(4, 1)
/* LCD controler TFT select bit */
#define CLCDC_LCDCTRL_TFT       _BIT(5)
/* LCD control monochrome LCD has 4-bit/8-bit select bit */
#define CLCDC_LCDCTRL_MON8      _BIT(6)
/* LCD control dual panel select bit */
#define CLCDC_LCDCTRL_DUAL      _BIT(7)
/* LCD control RGB or BGR format select bit */
#define CLCDC_LCDCTRL_RGB       _SBF(8, 0)
#define CLCDC_LCDCTRL_BGR       _SBF(8, 1)
/* LCD control big-endian byte order select bit */
#define CLCDC_LCDCTRL_BEBO      _BIT(9)
/* LCD control big-endian pixel order within a byte select bit */
#define CLCDC_LCDCTRL_BEPO      _BIT(10)
/* LCD control power enable bit */
#define CLCDC_LCDCTRL_PWR       _BIT(11)
/* LCD control VCOMP interrupt is start of VSYNC */
#define CLCDC_LCDCTRL_VCOMP_VS  _SBF(12, 0)
/* LCD control VCOMP interrupt is start of back porch */
#define CLCDC_LCDCTRL_VCOMP_BP  _SBF(12, 1)
/* LCD control VCOMP interrupt is start of active video */
#define CLCDC_LCDCTRL_VCOMP_AV  _SBF(12, 2)
/* LCD control VCOMP interrupt is start of front porch */
#define CLCDC_LCDCTRL_VCOMP_FP  _SBF(12, 3)
/* LCD control watermark level is 8 or more words free bit */
#define CLCDC_LCDCTRL_WATERMARK _BIT(16)

/* PINSEL11 selction for TFT 16 5:6:5 */
#define TFT_16_565 0xB

//////////////////////////////////////////////////////////////////////////////
// LCD Controller
//
struct LPC17xx_LCDC {
    static const uint32_t c_LCDC_Base = 0x20088000;

    /****/ volatile uint32_t LCD_TIMH;      /* LCD horizontal axis plane control register */
    /****/ volatile uint32_t LCD_TIMV;      /* LCD vertical axis plane control register */
    /****/ volatile uint32_t LCD_POL;       /* LCD clock and signal polarity control register */
    /****/ volatile uint32_t LCD_LE;        /* LCD line end control register */
    /****/ volatile uint32_t LCD_UPBASE;    /* LCD upper plane frame base address register */
    /****/ volatile uint32_t LCD_LPBASE;    /* LCD lower plane frame base address register */
    /****/ volatile uint32_t LCD_CTRL;      /* LCD control register */
    /****/ volatile uint32_t LCD_INTMSK;    /* LCD interrupt mask set/clear register */
    /****/ volatile uint32_t LCD_INTRAW;    /* LCD raw interrupt status register */
    /****/ volatile uint32_t LCD_INTSTAT;   /* LCD masked interrupt status register */
    /****/ volatile uint32_t LCD_INTCLR;    /* LCD interrupt clear register */
    /****/ volatile uint32_t LCD_UPCURR;    /* LCD upper panel current address value register */
    /****/ volatile uint32_t LCD_LPCURR;    /* LCD lower panel current address value register */
    /****/ volatile uint32_t dummy0[115];   /* LCD reserved */
    /****/ volatile uint32_t LCD_PAL[128];  /* LCD palette registers */
    /****/ volatile uint32_t dummy1[256];   /* LCD reserved */
    /****/ volatile uint32_t CRSR_IMG[256]; /* LCD cursor image */
    /****/ volatile uint32_t CRSR_CTRL;     /* LCD cursor control register */
    /****/ volatile uint32_t CRSR_CFG;      /* LCD cursor configuration register */
    /****/ volatile uint32_t CRSR_PAL0;     /* LCD cursor palette register */
    /****/ volatile uint32_t CRSR_PAL1;     /* LCD cursor palette register */
    /****/ volatile uint32_t CRSR_XY;       /* LCD cursor xy position register */
    /****/ volatile uint32_t CRSR_CLIP;     /* LCD cursor clip position register */
    /****/ volatile uint32_t dummy2[2];     /* LCD reserved */
    /****/ volatile uint32_t CRSR_INTMSK;   /* LCD cursor interrupt mask set/clear register */
    /****/ volatile uint32_t CRSR_INTCLR;   /* LCD cursor interrupt clear register */
    /****/ volatile uint32_t CRSR_INTRAW;   /* LCD cursor raw interrupt status register */
    /****/ volatile uint32_t CRSR_INTSTAT;  /* LCD cursor masked interrupt status register */
};
// LCD Controller Driver
//
//////////////////////////////////////////////////////////////////////////////

enum LPC17xx_LCD_Rotation {
    rotateNormal_0,
    rotateCW_90,
    rotate_180,
    rotateCCW_90,
};

uint32_t m_LPC17_DisplayWidth = 0;
uint32_t m_LPC17_DisplayHeight = 0;
uint32_t m_LPC17_DisplayPixelClockRateKHz = 0;
uint32_t m_LPC17_DisplayHorizontalSyncPulseWidth = 0;
uint32_t m_LPC17_DisplayHorizontalFrontPorch = 0;
uint32_t m_LPC17_DisplayHorizontalBackPorch = 0;
uint32_t m_LPC17_DisplayVerticalSyncPulseWidth = 0;
uint32_t m_LPC17_DisplayVerticalFrontPorch = 0;
uint32_t m_LPC17_DisplayVerticalBackPorch = 0;
uint32_t m_LPC17_Display_TextRow = 0;
uint32_t m_LPC17_Display_TextColumn = 0;

bool m_LPC17_DisplayOutputEnableIsFixed = false;
bool m_LPC17_DisplayOutputEnablePolarity = false;
bool m_LPC17_DisplayPixelPolarity = false;
bool m_LPC17_DisplayHorizontalSyncPolarity = false;
bool m_LPC17_DisplayVerticalSyncPolarity = false;
bool m_LPC17_DisplayEnable = false;

uint16_t* m_LPC17_Display_VituralRam = nullptr;
size_t m_LPC17_DisplayBufferSize = 0;

uint8_t m_LPC17_Display_TextBuffer[LCD_MAX_COLUMN][LCD_MAX_ROW];

LPC17xx_LCD_Rotation m_LPC17_Display_CurrentRotation = LPC17xx_LCD_Rotation::rotateNormal_0;

bool LPC17_Display_Initialize();
bool LPC17_Display_Uninitialize();
bool LPC17_Display_SetPinConfiguration(bool enable);

void LPC17_Display_WriteFormattedChar(uint8_t c);
void LPC17_Display_WriteChar(uint8_t c, int32_t row, int32_t col);
void LPC17_Display_BitBltEx(int32_t x, int32_t y, int32_t width, int32_t height, uint32_t data[]);
void LPC17_Display_PaintPixel(uint32_t x, uint32_t y, uint8_t c);
void LPC17_Display_Paint8HorizontalPixels(uint32_t x, uint32_t y, uint8_t p);
void LPC17_Display_TextEnterClearMode();
void LPC17_Display_PrintChracter(uint32_t x, uint32_t y, uint8_t c);
void LPC17_Display_TextShiftColUp();
void LPC17_Display_Clear();
void LPC17_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight);

int32_t LPC17_Display_GetWidth();
int32_t LPC17_Display_GetHeight();
int32_t LPC17_Display_BitPerPixel();
uint32_t LPC17_Display_GetPixelClockDivider();
int32_t LPC17_Display_GetOrientation();
uint32_t* LPC17_Display_GetFrameBuffer();

static TinyCLR_Display_Provider displayProvider;
static TinyCLR_Api_Info displayApi;

bool LPC17_Display_Initialize() {
    int32_t i;
    uint32_t * p32;

    uint32_t newFreq;
    uint8_t divider;

    LPC17xx_LCDC & LCDC = *(LPC17xx_LCDC *)LPC17xx_LCDC::c_LCDC_Base;

    /* Power Up the LCD controller. */
    LPC_SC->PCONP |= LPC17xx_SYSCON::ENABLE_LCD;
    LPC17_Time_Delay(nullptr, 1000 * 10);

    /* Disable the display in case it is on */
    LCDC.LCD_CTRL = 0;
    LPC17_Time_Delay(nullptr, 1000 * 10);

    LCDC.LCD_TIMH = ((m_LPC17_DisplayHorizontalBackPorch - 1) << 24) | ((m_LPC17_DisplayHorizontalFrontPorch - 1) << 16) | ((m_LPC17_DisplayHorizontalSyncPulseWidth - 1) << 8) | (((m_LPC17_DisplayWidth / 16) - 1) << 2);//(1<<24)|(16<<16)|(40<<8)|(((480/16)-1)<<2);
    LCDC.LCD_TIMV = ((m_LPC17_DisplayVerticalBackPorch) << 24) | ((m_LPC17_DisplayVerticalFrontPorch) << 16) | ((m_LPC17_DisplayVerticalSyncPulseWidth - 1) << 10) | (m_LPC17_DisplayHeight - 1);//LPC17_Display_VerticalTimingReg;//(2<<24)|(16<<16)|(9<<10)|(271);
    LCDC.LCD_POL = (1 << 26) | ((m_LPC17_DisplayWidth - 1) << 16) | (!m_LPC17_DisplayOutputEnablePolarity << 14) | (m_LPC17_DisplayPixelPolarity << 13) | (!m_LPC17_DisplayHorizontalSyncPolarity << 12) | (!m_LPC17_DisplayVerticalSyncPolarity << 11);//LPC17_Display_PolarityReg;//(1<<26)|(479<<16)|(1<<14)|(0<<13)|(1<<12)|(1<<11);
    LCDC.LCD_LE = 0;//not used

    LCDC.LCD_INTMSK = 0;

    // don ot use grayscaler!!!?? manual
    LCDC.LCD_CTRL = (6 << 1) | (1 << 5) | (1 << 11) | (1 << 8)/*RGB*/ | (0 << 9)/*endian*/ | (0 << 10)/*pixel order*/;

    // clear pallete
    p32 = (uint32_t*)&LCDC.LCD_PAL;
    for (i = 0; i < 128; i++) {
        p32[i] = 0;
    }

    if (m_LPC17_DisplayPixelClockRateKHz == 0) {
        divider = 0xFF;
    }
    else {
        newFreq = (uint32_t)((LPC17_AHB_CLOCK_HZ / 1000) % m_LPC17_DisplayPixelClockRateKHz);

        if (newFreq != 0)
            divider = ((LPC17_AHB_CLOCK_HZ / 1000) / m_LPC17_DisplayPixelClockRateKHz) + 1;
        else
            divider = ((LPC17_AHB_CLOCK_HZ / 1000) / m_LPC17_DisplayPixelClockRateKHz);
    }

    LPC_SC->LCD_CFG = (divider - 1);

    if (m_LPC17_Display_VituralRam == nullptr)
        return false;

    LCDC.LCD_UPBASE = (uint32_t)&m_LPC17_Display_VituralRam[0];

    LPC17_Time_Delay(nullptr, 1000 * 10);

    LCDC.LCD_CTRL |= 1;//enable ....also add power

    LPC17_Time_Delay(nullptr, 1000 * 10);

    LPC17_Display_TextEnterClearMode();

    return true;
}

bool LPC17_Display_Uninitialize() {
    int32_t i;

    LPC17xx_LCDC & LCDC = *(LPC17xx_LCDC *)LPC17xx_LCDC::c_LCDC_Base;

    if (m_LPC17_DisplayEnable == false)
        return true;

    // powerdown
    LCDC.LCD_CTRL &= ~1;
    LPC17_Time_Delay(nullptr, 1000 * 10);
    LCDC.LCD_CTRL = 0;
    LPC17_Time_Delay(nullptr, 1000 * 10);

    return true;
}

//====================================================
void LPC17_Display_WriteFormattedChar(uint8_t c) {
    if (m_LPC17_DisplayEnable == false)
        return;

    if (c == '\f') {
        LPC17_Display_Clear();
        LPC17_Display_TextEnterClearMode();
        m_LPC17_Display_TextColumn = 0;

        return;
    }

    if (c == '\r') {
        m_LPC17_Display_TextColumn = 0;

        return;
    }
    if (c == '\n') {
        m_LPC17_Display_TextColumn = 0;

        if (++m_LPC17_Display_TextRow >= LCD_MAX_ROW) {
            m_LPC17_Display_TextRow = LCD_MAX_ROW - 1;
            LPC17_Display_TextShiftColUp();
        }
        // clean the new line
        for (c = 0; c < (LCD_MAX_COLUMN - 1); c++) {
            m_LPC17_Display_TextBuffer[c][m_LPC17_Display_TextRow] = ' ';
        }

        return;
    }

    LPC17_Display_PrintChracter(m_LPC17_Display_TextColumn * 6, m_LPC17_Display_TextRow * 8, c);
    m_LPC17_Display_TextBuffer[m_LPC17_Display_TextColumn][m_LPC17_Display_TextRow] = c;

    if (++m_LPC17_Display_TextColumn >= (LCD_MAX_COLUMN - 1)) {
        m_LPC17_Display_TextColumn = 0;

        if (++m_LPC17_Display_TextRow >= LCD_MAX_ROW) {
            m_LPC17_Display_TextRow = LCD_MAX_ROW - 1;
            LPC17_Display_TextShiftColUp();
        }
        else {
            // clean the new line
            for (c = 0; c < LCD_MAX_COLUMN; c++) {
                m_LPC17_Display_TextBuffer[c][m_LPC17_Display_TextRow] = ' ';
            }
        }
    }
}
//=======================================================
void LPC17_Display_PaintPixel(uint32_t x, uint32_t y, uint8_t c) {
    volatile uint16_t * loc;

    if (m_LPC17_DisplayEnable == false)
        return;

    if (x >= m_LPC17_DisplayWidth)
        return;
    if (y >= m_LPC17_DisplayHeight)
        return;

    loc = m_LPC17_Display_VituralRam + (y *m_LPC17_DisplayWidth) + (x);

    if (c)
        *loc = 0x0fff;
    else
        *loc = 0;
}
//=======================================================
void LPC17_Display_Paint8HorizontalPixels(uint32_t x, uint32_t y, uint8_t p) {
    if (m_LPC17_DisplayEnable == false)
        return;

    for (int32_t i = 0; i < 8; i++) {
        if (p&(1 << i))
            LPC17_Display_PaintPixel(x, y + i, 1);
        else
            LPC17_Display_PaintPixel(x, y + i, 0);//clear
    }
}
//===========================================================
void LPC17_Display_TextEnterClearMode() {
    uint32_t r, c;

    if (m_LPC17_DisplayEnable == false)
        return;

    LPC17_Display_Clear();
    m_LPC17_Display_TextRow = 0;
    m_LPC17_Display_TextColumn = 0;

    for (r = 0; r < LCD_MAX_ROW; r++) {
        for (c = 0; c < (LCD_MAX_COLUMN - 1); c++) {
            m_LPC17_Display_TextBuffer[c][r] = '1';
        }
    }
}
//===========================================================
void LPC17_Display_PrintChracter(uint32_t x, uint32_t y, uint8_t c) {
    uint8_t i;

    if (m_LPC17_DisplayEnable == false)
        return;

    for (i = 0; i < 5; i++)
        LPC17_Display_Paint8HorizontalPixels(x + i, y, characters[c][i]);

    LPC17_Display_Paint8HorizontalPixels(x + i, y, 0);
}

void LPC17_Display_TextShiftColUp() {
    uint32_t r, c;

    if (m_LPC17_DisplayEnable == false)
        return;

    // refresh with new data
    LPC17_Display_Clear();
    m_LPC17_Display_TextRow = 0;
    m_LPC17_Display_TextColumn = 0;

    for (r = 0; r < (LCD_MAX_ROW - 1); r++) {
        for (c = 0; c < LCD_MAX_COLUMN - 1; c++) {
            m_LPC17_Display_TextBuffer[c][r] = m_LPC17_Display_TextBuffer[c][r + 1];
            LPC17_Display_WriteFormattedChar(m_LPC17_Display_TextBuffer[c][r]);
        }
    }
}

void LPC17_Display_Clear() {
    if (m_LPC17_DisplayEnable == false || m_LPC17_Display_VituralRam == nullptr)
        return;

    memset((uint32_t*)m_LPC17_Display_VituralRam, 0, m_LPC17_DisplayBufferSize);
}

const LPC17_Gpio_Pin g_Display_ControllerPins[] = LPC17_DISPLAY_CONTROLLER_PINS;
const LPC17_Gpio_Pin g_Display_BacklightPin = LPC17_DISPLAY_BACKLIGHT_PIN;
const LPC17_Gpio_Pin g_Display_EnablePin = LPC17_DISPLAY_ENABLE_PIN;

bool  LPC17_Display_SetPinConfiguration(bool enable) {
    if (enable) {
        for (auto i = 0; i < SIZEOF_ARRAY(g_Display_ControllerPins); i++) {
            if (!LPC17_Gpio_OpenPin(g_Display_ControllerPins[i].number)) {
                return false;
            }

            LPC17_Gpio_ConfigurePin(g_Display_ControllerPins[i].number, LPC17_Gpio_Direction::Input, g_Display_ControllerPins[i].pinFunction, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);
        }

        if (!LPC17_Gpio_OpenPin(g_Display_EnablePin.number)) {
            return false;
        }

        if (m_LPC17_DisplayOutputEnableIsFixed)
            LPC17_Gpio_EnableOutputPin(g_Display_EnablePin.number, m_LPC17_DisplayOutputEnablePolarity);
        else
            LPC17_Gpio_ConfigurePin(g_Display_EnablePin.number, LPC17_Gpio_Direction::Input, LPC17_Gpio_PinFunction::PinFunction7, LPC17_Gpio_ResistorMode::Inactive, LPC17_Gpio_Hysteresis::Disable, LPC17_Gpio_InputPolarity::NotInverted, LPC17_Gpio_SlewRate::StandardMode, LPC17_Gpio_OutputType::PushPull);

        if (g_Display_BacklightPin.number != PIN_NONE) {
            if (!LPC17_Gpio_OpenPin(g_Display_BacklightPin.number)) {
                return false;

            }

            LPC17_Gpio_EnableOutputPin(g_Display_BacklightPin.number, true);
        }
    }
    else {
        for (int32_t i = 0; i < SIZEOF_ARRAY(g_Display_ControllerPins); i++) {
            LPC17_Gpio_ClosePin(g_Display_ControllerPins[i].number);
        }

        LPC17_Gpio_ClosePin(g_Display_EnablePin.number);

        LPC17_Gpio_ClosePin(g_Display_BacklightPin.number);
    }

    return true;
}
uint32_t* LPC17_Display_GetFrameBuffer() {
    return (uint32_t*)m_LPC17_Display_VituralRam;
}

int32_t LPC17_Display_GetWidth() {
    int32_t width = m_LPC17_DisplayWidth;
    int32_t height = m_LPC17_DisplayHeight;

    LPC17_Display_GetRotatedDimensions(&width, &height);

    return width;
}

int32_t LPC17_Display_GetHeight() {
    int32_t width = m_LPC17_DisplayWidth;
    int32_t height = m_LPC17_DisplayHeight;

    LPC17_Display_GetRotatedDimensions(&width, &height);

    return height;
}

uint32_t LPC17_Display_GetPixelClockDivider() {
    return m_LPC17_DisplayPixelClockRateKHz;
}
int32_t LPC17_Display_GetOrientation() {
    return m_LPC17_Display_CurrentRotation;
}

void  LPC17_Display_MemCopy(void *dest, void *src, int32_t size) {
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

void LPC17_Display_BitBltEx(int32_t x, int32_t y, int32_t width, int32_t height, uint32_t data[]) {

    int32_t xTo, yTo, xFrom, yFrom;
    int32_t xOffset = x;
    int32_t yOffset = y;
    uint16_t *from = (uint16_t *)data;
    uint16_t *to = (uint16_t *)m_LPC17_Display_VituralRam;


    int32_t screenWidth = m_LPC17_DisplayWidth;
    int32_t screenHeight = m_LPC17_DisplayHeight;
    int32_t startPx, toAddition;

    if (m_LPC17_DisplayEnable == false)
        return;

    switch (m_LPC17_Display_CurrentRotation) {
    case LPC17xx_LCD_Rotation::rotateNormal_0:

        if (xOffset == 0 && yOffset == 0 &&
            width == screenWidth && height == screenHeight) {
            LPC17_Display_MemCopy(to, from, (screenWidth*screenHeight * 2));
        }
        else {
            for (yTo = yOffset; yTo < (yOffset + height); yTo++) {
                LPC17_Display_MemCopy((void*)(to + yTo * screenWidth + xOffset), (void*)(from + yTo * screenWidth + xOffset), (width * 2));
            }
        }

        break;

    case LPC17xx_LCD_Rotation::rotateCCW_90:

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

    case LPC17xx_LCD_Rotation::rotateCW_90:

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

    case LPC17xx_LCD_Rotation::rotate_180:

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

void LPC17_Display_WriteChar(uint8_t c, int32_t row, int32_t col) {
    m_LPC17_Display_TextRow = row;
    m_LPC17_Display_TextColumn = col;
    LPC17_Display_WriteFormattedChar(c);

}
void LPC17_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight) {
    switch (m_LPC17_Display_CurrentRotation) {
    case LPC17xx_LCD_Rotation::rotateNormal_0:
    case LPC17xx_LCD_Rotation::rotate_180:
        *screenWidth = m_LPC17_DisplayWidth;
        *screenHeight = m_LPC17_DisplayHeight;
        break;

    case LPC17xx_LCD_Rotation::rotateCCW_90:
    case LPC17xx_LCD_Rotation::rotateCW_90:
        *screenWidth = m_LPC17_DisplayHeight;
        *screenHeight = m_LPC17_DisplayWidth;
        break;
    }
}

TinyCLR_Result LPC17_Display_Acquire(const TinyCLR_Display_Provider* self, int32_t controller) {
    m_LPC17_Display_CurrentRotation = LPC17xx_LCD_Rotation::rotateNormal_0;

    if (!LPC17_Display_SetPinConfiguration(true)) {
        return TinyCLR_Result::SharingViolation;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Display_Release(const TinyCLR_Display_Provider* self, int32_t controller) {
    LPC17_Display_Uninitialize();

    LPC17_Display_SetPinConfiguration(false);

    m_LPC17_DisplayEnable = false;

    if (m_LPC17_Display_VituralRam != nullptr) {
        auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

        memoryProvider->Free(memoryProvider, m_LPC17_Display_VituralRam);

        m_LPC17_Display_VituralRam = nullptr;
    }

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Display_Enable(const TinyCLR_Display_Provider* self, int32_t controller) {
    if (m_LPC17_DisplayEnable || LPC17_Display_Initialize()) {
        m_LPC17_DisplayEnable = true;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result LPC17_Display_Disable(const TinyCLR_Display_Provider* self, int32_t controller) {
    LPC17_Display_Uninitialize();

    m_LPC17_DisplayEnable = false;

    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Display_SetConfiguration(const TinyCLR_Display_Provider* self, int32_t controller, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration) {
    if (dataFormat != TinyCLR_Display_DataFormat::Rgb565) return TinyCLR_Result::NotSupported;

    if (configuration != nullptr) {
        auto& cfg = *(const TinyCLR_Display_ParallelConfiguration*)configuration;

        m_LPC17_DisplayWidth = width;
        m_LPC17_DisplayHeight = height;

        m_LPC17_DisplayOutputEnableIsFixed = cfg.DataEnableIsFixed;
        m_LPC17_DisplayOutputEnablePolarity = cfg.DataEnablePolarity;
        m_LPC17_DisplayPixelPolarity = cfg.PixelPolarity;

        m_LPC17_DisplayPixelClockRateKHz = cfg.PixelClockRate / 1000;

        m_LPC17_DisplayHorizontalSyncPolarity = cfg.HorizontalSyncPolarity;

        m_LPC17_DisplayHorizontalSyncPulseWidth = cfg.HorizontalSyncPulseWidth;
        m_LPC17_DisplayHorizontalFrontPorch = cfg.HorizontalFrontPorch;
        m_LPC17_DisplayHorizontalBackPorch = cfg.HorizontalBackPorch;

        m_LPC17_DisplayVerticalSyncPolarity = cfg.VerticalSyncPolarity;

        m_LPC17_DisplayVerticalSyncPulseWidth = cfg.VerticalSyncPulseWidth;
        m_LPC17_DisplayVerticalFrontPorch = cfg.VerticalFrontPorch;
        m_LPC17_DisplayVerticalBackPorch = cfg.VerticalBackPorch;

        switch (dataFormat) {
        case TinyCLR_Display_DataFormat::Rgb565:
            m_LPC17_DisplayBufferSize = width * height * 2;
            break;

        default:
            // TODO 8 - 24 - 32 bits
            break;
        }

        auto memoryProvider = (const TinyCLR_Memory_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::MemoryProvider);

        if (m_LPC17_Display_VituralRam != nullptr) {
            memoryProvider->Free(memoryProvider, m_LPC17_Display_VituralRam);

            m_LPC17_Display_VituralRam = nullptr;
        }

        m_LPC17_Display_VituralRam = (uint16_t*)((uint8_t*)memoryProvider->Allocate(memoryProvider, m_LPC17_DisplayBufferSize));

        if (m_LPC17_Display_VituralRam == nullptr) {
            return TinyCLR_Result::OutOfMemory;
        }
    }

    return  TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Display_GetConfiguration(const TinyCLR_Display_Provider* self, int32_t controller, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration) {
    dataFormat = TinyCLR_Display_DataFormat::Rgb565;
    width = m_LPC17_DisplayWidth;
    height = m_LPC17_DisplayHeight;

    if (configuration != nullptr) {
        auto& cfg = *(TinyCLR_Display_ParallelConfiguration*)configuration;

        cfg.DataEnableIsFixed = m_LPC17_DisplayOutputEnableIsFixed;
        cfg.DataEnablePolarity = m_LPC17_DisplayOutputEnablePolarity;
        cfg.PixelPolarity = m_LPC17_DisplayPixelPolarity;

        cfg.PixelClockRate = m_LPC17_DisplayPixelClockRateKHz * 1000;

        cfg.HorizontalSyncPolarity = m_LPC17_DisplayHorizontalSyncPolarity;

        cfg.HorizontalSyncPulseWidth = m_LPC17_DisplayHorizontalSyncPulseWidth;
        cfg.HorizontalFrontPorch = m_LPC17_DisplayHorizontalFrontPorch;
        cfg.HorizontalBackPorch = m_LPC17_DisplayHorizontalBackPorch;

        cfg.VerticalSyncPolarity = m_LPC17_DisplayVerticalSyncPolarity;

        cfg.VerticalSyncPulseWidth = m_LPC17_DisplayVerticalSyncPulseWidth;
        cfg.VerticalFrontPorch = m_LPC17_DisplayVerticalFrontPorch;
        cfg.VerticalBackPorch = m_LPC17_DisplayVerticalBackPorch;

        return TinyCLR_Result::Success;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result LPC17_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t controller, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data) {
    LPC17_Display_BitBltEx(x, y, width, height, (uint32_t*)data);
    return TinyCLR_Result::Success;
}

TinyCLR_Result LPC17_Display_WriteString(const TinyCLR_Display_Provider* self, int32_t controller, const char* buffer, size_t length) {
    for (size_t i = 0; i < length; i++)
        LPC17_Display_WriteFormattedChar(buffer[i]);

    return TinyCLR_Result::Success;
}

TinyCLR_Display_DataFormat dataFormats[] = { TinyCLR_Display_DataFormat::Rgb565 };

TinyCLR_Result LPC17_Display_GetCapabilities(const TinyCLR_Display_Provider* self, int32_t controller, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount) {
    type = TinyCLR_Display_InterfaceType::Parallel;
    supportedDataFormatCount = SIZEOF_ARRAY(dataFormats);
    supportedDataFormats = dataFormats;

    return TinyCLR_Result::Success;
}

const TinyCLR_Api_Info* LPC17_Display_GetApi() {
    displayProvider.ApiInfo = &displayApi;
    displayProvider.Acquire = &LPC17_Display_Acquire;
    displayProvider.Release = &LPC17_Display_Release;
    displayProvider.Enable = &LPC17_Display_Enable;
    displayProvider.Disable = &LPC17_Display_Disable;
    displayProvider.SetConfiguration = &LPC17_Display_SetConfiguration;
    displayProvider.GetConfiguration = &LPC17_Display_GetConfiguration;
    displayProvider.GetCapabilities = &LPC17_Display_GetCapabilities;
    displayProvider.DrawBuffer = &LPC17_Display_DrawBuffer;
    displayProvider.WriteString = &LPC17_Display_WriteString;
    displayProvider.GetControllerCount = &LPC17_Display_GetControllerCount;

    displayApi.Author = "GHI Electronics, LLC";
    displayApi.Name = "GHIElectronics.TinyCLR.NativeApis.LPC17.DisplayProvider";
    displayApi.Type = TinyCLR_Api_Type::DisplayProvider;
    displayApi.Version = 0;
    displayApi.Implementation = &displayProvider;

    m_LPC17_Display_VituralRam = nullptr;

    return &displayApi;
}

void LPC17_Display_Reset() {
    LPC17_Display_Clear();

    if (m_LPC17_DisplayEnable)
        LPC17_Display_Release(&displayProvider, 0);

    m_LPC17_DisplayEnable = false;
}

TinyCLR_Result LPC17_Display_GetControllerCount(const TinyCLR_Display_Provider* self, int32_t& count) {
    count = 1;

    return TinyCLR_Result::Success;
}

