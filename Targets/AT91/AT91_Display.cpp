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

#define VIDEO_RAM_SIZE              800*600*2

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

/////////////////////////////////////////////////////////////////////////////
// LCD Controller
//
// LCD Controller Driver
//
//////////////////////////////////////////////////////////////////////////////

enum LPC17xx_LCD_Rotation {
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

LPC17xx_LCD_Rotation m_AT91_Display_CurrentRotation = LPC17xx_LCD_Rotation::rotateNormal_0;

bool AT91_Display_Initialize();
bool AT91_Display_Uninitialize();
bool AT91_Display_SetPinConfiguration();

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


#define AHBCFG1		(*(volatile unsigned *)0xe01fc188)

bool AT91_Display_Initialize() {
    m_AT91_DisplayEnable = true;

    return true;
}

bool AT91_Display_Uninitialize() {
    int32_t i;

    if (m_AT91_DisplayEnable == false)
        return true;

    for (i = 51; i <= 70; i++) {
        AT91_Gpio_ReleasePin(nullptr, i);
    }

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

bool  AT91_Display_SetPinConfiguration() {

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
        case LPC17xx_LCD_Rotation::rotateNormal_0:

            if (xOffset == 0 && yOffset == 0 &&
                width == screenWidth &&    height == screenHeight) {
                AT91_Display_MemCopy(to, from, (screenWidth*screenHeight * 2));
            }
            else {
                for (yTo = yOffset; yTo < (yOffset + height); yTo++) {
                    AT91_Display_MemCopy((void*)(to + yTo * screenWidth + xOffset), (void*)(from + yTo * screenWidth + xOffset), (width * 2));
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

void AT91_Display_WriteChar(uint8_t c, int32_t row, int32_t col) {
    m_AT91_Display_TextRow = row;
    m_AT91_Display_TextColumn = col;
    AT91_Display_WriteFormattedChar(c);

}
void AT91_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight) {
    switch (m_AT91_Display_CurrentRotation) {
        case LPC17xx_LCD_Rotation::rotateNormal_0:
        case LPC17xx_LCD_Rotation::rotate_180:
            *screenWidth = m_AT91_DisplayWidth;
            *screenHeight = m_AT91_DisplayHeight;
            break;

        case LPC17xx_LCD_Rotation::rotateCCW_90:
        case LPC17xx_LCD_Rotation::rotateCW_90:
            *screenWidth = m_AT91_DisplayHeight;
            *screenHeight = m_AT91_DisplayWidth;
            break;
    }
}

TinyCLR_Result AT91_Display_Acquire(const TinyCLR_Display_Provider* self, uint32_t width, uint32_t height) {
    m_AT91_DisplayWidth = width;
    m_AT91_DisplayHeight = height;

    m_AT91_Display_CurrentRotation = LPC17xx_LCD_Rotation::rotateNormal_0;

    m_AT91_Display_VituralRam = (uint16_t*)m_AT91_Display_ReservedVitualRamLocation;

    if (AT91_Display_SetPinConfiguration())
        return TinyCLR_Result::Success;

    return  TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91_Display_Release(const TinyCLR_Display_Provider* self) {
    if (AT91_Display_Uninitialize())
        return TinyCLR_Result::Success;

    return  TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91_Display_SetLcdConfiguration(const TinyCLR_Display_Provider* self, bool outputEnableIsFixed, bool outputEnablePolarity, bool pixelPolarity, uint32_t pixelClockRate, bool horizontalSyncPolarity, uint32_t horizontalSyncPulseWidth, uint32_t horizontalFrontPorch, uint32_t horizontalBackPorch, bool verticalSyncPolarity, uint32_t verticalSyncPulseWidth, uint32_t verticalFrontPorch, uint32_t verticalBackPorch) {
    m_AT91_DisplayOutputEnableIsFixed = outputEnableIsFixed;
    m_AT91_DisplayOutputEnablePolarity = outputEnablePolarity;
    m_AT91_DisplayPixelPolarity = pixelPolarity;

    m_AT91_DisplayPixelClockRateKHz = pixelClockRate / 1000;

    m_AT91_DisplayHorizontalSyncPolarity = horizontalSyncPolarity;

    m_AT91_DisplayHorizontalSyncPulseWidth = horizontalSyncPulseWidth;
    m_AT91_DisplayHorizontalFrontPorch = horizontalFrontPorch;
    m_AT91_DisplayHorizontalBackPorch = horizontalBackPorch;

    m_AT91_DisplayVerticalSyncPolarity = verticalSyncPolarity;

    m_AT91_DisplayVerticalSyncPulseWidth = verticalSyncPulseWidth;
    m_AT91_DisplayVerticalFrontPorch = verticalFrontPorch;
    m_AT91_DisplayVerticalBackPorch = verticalBackPorch;

    if (AT91_Display_Initialize())
        return TinyCLR_Result::Success;

    return  TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data, TinyCLR_Display_Format dataFormat) {
    switch (dataFormat) {
        case TinyCLR_Display_Format::Rgb565:
            AT91_Display_BitBltEx(x, y, width, height, (uint32_t*)data);
            return TinyCLR_Result::Success;
    }

    return  TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result AT91_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer) {
    while (buffer != '\0') {
        AT91_Display_WriteFormattedChar(buffer[0]);
        buffer++;
    }
    return TinyCLR_Result::Success;
}

int32_t AT91_Display_GetWidth(const TinyCLR_Display_Provider* self) {
    return AT91_Display_GetWidth();;
}

int32_t AT91_Display_GetHeight(const TinyCLR_Display_Provider* self) {
    return AT91_Display_GetHeight();
}

TinyCLR_Display_InterfaceType AT91_Display_GetType(const TinyCLR_Display_Provider* self) {
    return TinyCLR_Display_InterfaceType::Parallel;
}

const TinyCLR_Api_Info* AT91_Display_GetApi() {
    displayProvider.Parent = &displayApi;
    displayProvider.Index = 0;
    displayProvider.Acquire = &AT91_Display_Acquire;
    displayProvider.Release = &AT91_Display_Release;
    displayProvider.SetLcdConfiguration = &AT91_Display_SetLcdConfiguration;
    displayProvider.DrawBuffer = &AT91_Display_DrawBuffer;
    displayProvider.WriteString = &AT91_Display_WriteString;
    displayProvider.GetWidth = &AT91_Display_GetWidth;
    displayProvider.GetHeight = &AT91_Display_GetHeight;
    displayProvider.GetType = &AT91_Display_GetType;

    displayApi.Author = "GHI Electronics, LLC";
    displayApi.Name = "GHIElectronics.TinyCLR.NativeApis.AT91.DisplayProvider";
    displayApi.Type = TinyCLR_Api_Type::DisplayProvider;
    displayApi.Version = 0;
    displayApi.Count = 1;
    displayApi.Implementation = &displayProvider;

    return &displayApi;
}

void AT91_Display_Reset() {
    AT91_Display_Uninitialize();
}

#endif // INCLUDE_DISPLAY
