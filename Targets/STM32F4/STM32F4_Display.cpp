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
#include "STM32F4.h"

#ifdef INCLUDE_DISPLAY

#define MAX_LAYER  2

/**
  * @brief  LTDC color structure definition
  */
typedef struct {
    uint8_t Blue;                    /*!< Configures the blue value.
                                          This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */
    uint8_t Green;                   /*!< Configures the green value.
                                          This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */
    uint8_t Red;                     /*!< Configures the red value.
                                          This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */
    uint8_t Reserved;                /*!< Reserved 0xFF */
} LTDC_ColorTypeDef;

/**
  * @brief  LTDC Init structure definition
  */
typedef struct {
    uint32_t            HSPolarity;                /*!< configures the horizontal synchronization polarity.
                                                        This parameter can be one value of @ref LTDC_HS_POLARITY */
    uint32_t            VSPolarity;                /*!< configures the vertical synchronization polarity.
                                                        This parameter can be one value of @ref LTDC_VS_POLARITY */
    uint32_t            DEPolarity;                /*!< configures the data enable polarity.
                                                        This parameter can be one of value of @ref LTDC_DE_POLARITY */
    uint32_t            PCPolarity;                /*!< configures the pixel clock polarity.
                                                        This parameter can be one of value of @ref LTDC_PC_POLARITY */
    uint32_t            HorizontalSync;            /*!< configures the number of Horizontal synchronization width.
                                                        This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */
    uint32_t            VerticalSync;              /*!< configures the number of Vertical synchronization height.
                                                        This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF. */
    uint32_t            AccumulatedHBP;            /*!< configures the accumulated horizontal back porch width.
                                                        This parameter must be a number between Min_Data = LTDC_HorizontalSync and Max_Data = 0xFFF. */
    uint32_t            AccumulatedVBP;            /*!< configures the accumulated vertical back porch height.
                                                        This parameter must be a number between Min_Data = LTDC_VerticalSync and Max_Data = 0x7FF. */
    uint32_t            AccumulatedActiveW;        /*!< configures the accumulated active width.
                                                        This parameter must be a number between Min_Data = LTDC_AccumulatedHBP and Max_Data = 0xFFF. */
    uint32_t            AccumulatedActiveH;        /*!< configures the accumulated active height.
                                                        This parameter must be a number between Min_Data = LTDC_AccumulatedVBP and Max_Data = 0x7FF. */
    uint32_t            TotalWidth;                /*!< configures the total width.
                                                        This parameter must be a number between Min_Data = LTDC_AccumulatedActiveW and Max_Data = 0xFFF. */
    uint32_t            TotalHeigh;                /*!< configures the total height.
                                                        This parameter must be a number between Min_Data = LTDC_AccumulatedActiveH and Max_Data = 0x7FF. */
    LTDC_ColorTypeDef   Backcolor;                 /*!< Configures the background color. */
} LTDC_InitTypeDef;

/**
  * @brief  LTDC Layer structure definition
  */
typedef struct {
    uint32_t WindowX0;                   /*!< Configures the Window Horizontal Start Position.
                                              This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */
    uint32_t WindowX1;                   /*!< Configures the Window Horizontal Stop Position.
                                              This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */
    uint32_t WindowY0;                   /*!< Configures the Window vertical Start Position.
                                              This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF. */
    uint32_t WindowY1;                   /*!< Configures the Window vertical Stop Position.
                                              This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x7FF. */
    uint32_t PixelFormat;                /*!< Specifies the pixel format.
                                              This parameter can be one of value of @ref LTDC_Pixelformat */
    uint32_t Alpha;                      /*!< Specifies the constant alpha used for blending.
                                              This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */
    uint32_t Alpha0;                     /*!< Configures the default alpha value.
                                              This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */
    uint32_t BlendingFactor1;            /*!< Select the blending factor 1.
                                              This parameter can be one of value of @ref LTDC_BlendingFactor1 */
    uint32_t BlendingFactor2;            /*!< Select the blending factor 2.
                                              This parameter can be one of value of @ref LTDC_BlendingFactor2 */
    uint32_t FBStartAdress;              /*!< Configures the color frame buffer address */
    uint32_t ImageWidth;                 /*!< Configures the color frame buffer line length.
                                              This parameter must be a number between Min_Data = 0x0000 and Max_Data = 0x1FFF. */
    uint32_t ImageHeight;                /*!< Specifies the number of line in frame buffer.
                                              This parameter must be a number between Min_Data = 0x000 and Max_Data = 0x7FF. */
    LTDC_ColorTypeDef   Backcolor;       /*!< Configures the layer background color. */
} LTDC_LayerCfgTypeDef;


/**
  * @brief  LTDC handle Structure definition
  */
typedef struct {
    LTDC_TypeDef                *Instance;                /*!< LTDC Register base address                */
    LTDC_InitTypeDef            Init;                     /*!< LTDC parameters                           */
    LTDC_LayerCfgTypeDef        LayerCfg[MAX_LAYER];      /*!< LTDC Layers parameters                    */
} LTDC_HandleTypeDef;

/**
  * @}
  */

/** @defgroup LTDC_HS_POLARITY LTDC HS POLARITY
* @{
*/
#define LTDC_HSPOLARITY_AL                ((uint32_t)0x00000000)                /*!< Horizontal Synchronization is active low. */
#define LTDC_HSPOLARITY_AH                LTDC_GCR_HSPOL                        /*!< Horizontal Synchronization is active high. */
/**
* @}
*/

/** @defgroup LTDC_VS_POLARITY LTDC VS POLARITY
* @{
*/
#define LTDC_VSPOLARITY_AL                ((uint32_t)0x00000000)                /*!< Vertical Synchronization is active low. */
#define LTDC_VSPOLARITY_AH                LTDC_GCR_VSPOL                        /*!< Vertical Synchronization is active high. */
/**
* @}
*/

/** @defgroup LTDC_DE_POLARITY LTDC DE POLARITY
* @{
*/
#define LTDC_DEPOLARITY_AL                ((uint32_t)0x00000000)                /*!< Data Enable, is active low. */
#define LTDC_DEPOLARITY_AH                LTDC_GCR_DEPOL                        /*!< Data Enable, is active high. */
/**
* @}
*/

/** @defgroup LTDC_PC_POLARITY LTDC PC POLARITY
* @{
*/
#define LTDC_PCPOLARITY_IPC               ((uint32_t)0x00000000)                /*!< input pixel clock. */
#define LTDC_PCPOLARITY_IIPC              LTDC_GCR_PCPOL                        /*!< inverted input pixel clock. */

/** @defgroup LTDC_Pixelformat LTDC Pixel format
* @{
*/
#define LTDC_PIXEL_FORMAT_ARGB8888                  ((uint32_t)0x00000000)      /*!< ARGB8888 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_RGB888                    ((uint32_t)0x00000001)      /*!< RGB888 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_RGB565                    ((uint32_t)0x00000002)      /*!< RGB565 LTDC pixel format   */
#define LTDC_PIXEL_FORMAT_ARGB1555                  ((uint32_t)0x00000003)      /*!< ARGB1555 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_ARGB4444                  ((uint32_t)0x00000004)      /*!< ARGB4444 LTDC pixel format */
#define LTDC_PIXEL_FORMAT_L8                        ((uint32_t)0x00000005)      /*!< L8 LTDC pixel format       */
#define LTDC_PIXEL_FORMAT_AL44                      ((uint32_t)0x00000006)      /*!< AL44 LTDC pixel format     */
#define LTDC_PIXEL_FORMAT_AL88                      ((uint32_t)0x00000007)      /*!< AL88 LTDC pixel format     */
/**
* @}
*/

/** @defgroup LTDC_BlendingFactor1 LTDC Blending Factor1
* @{
*/
#define LTDC_BLENDING_FACTOR1_CA                       ((uint32_t)0x00000400)   /*!< Blending factor : Cte Alpha */
#define LTDC_BLENDING_FACTOR1_PAxCA                    ((uint32_t)0x00000600)   /*!< Blending factor : Cte Alpha x Pixel Alpha*/
/**
* @}
*/

/** @defgroup LTDC_BlendingFactor2 LTDC Blending Factor2
* @{
*/
#define LTDC_BLENDING_FACTOR2_CA                       ((uint32_t)0x00000005)   /*!< Blending factor : Cte Alpha */
#define LTDC_BLENDING_FACTOR2_PAxCA                    ((uint32_t)0x00000007)   /*!< Blending factor : Cte Alpha x Pixel Alpha*/
/**
* @}
*/

#define LTDC_LAYER(__HANDLE__, __LAYER__)         ((LTDC_Layer_TypeDef *)((uint32_t)(((uint32_t)((__HANDLE__)->Instance)) + 0x84 + (0x80*(__LAYER__)))))

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
#define VIDEO_RAM_SIZE (800*600*2) // Maximum LCD screen size times bytes per pixel
#define UNCACHE_LCD_BUFFER_ADDRESS 0xC0E00000
#define VIDEO_RAM_ADDRESS (UNCACHE_LCD_BUFFER_ADDRESS) // Maximum LCD screen size times bytes per pixel

#define LCD_MAX_ROW	32
#define LCD_MAX_COLUMN 70

enum STM32F4xx_LCD_Rotation {
    rotateNormal_0,
    rotateCW_90,
    rotate_180,
    rotateCCW_90,
};

uint32_t m_STM32F4_DisplayWidth = 0;
uint32_t m_STM32F4_DisplayHeight = 0;
uint32_t m_STM32F4_DisplayPixelClockRateKHz = 0;
uint32_t m_STM32F4_DisplayHorizontalSyncPulseWidth = 0;
uint32_t m_STM32F4_DisplayHorizontalFrontPorch = 0;
uint32_t m_STM32F4_DisplayHorizontalBackPorch = 0;
uint32_t m_STM32F4_DisplayVerticalSyncPulseWidth = 0;
uint32_t m_STM32F4_DisplayVerticalFrontPorch = 0;
uint32_t m_STM32F4_DisplayVerticalBackPorch = 0;
uint32_t m_STM32F4_Display_TextRow = 0;
uint32_t m_STM32F4_Display_TextColumn = 0;

bool m_STM32F4_DisplayOutputEnableIsFixed = false;
bool m_STM32F4_DisplayOutputEnablePolarity = false;
bool m_STM32F4_DisplayPixelPolarity = false;
bool m_STM32F4_DisplayHorizontalSyncPolarity = false;
bool m_STM32F4_DisplayVerticalSyncPolarity = false;
bool m_STM32F4_DisplayEnable = false;

uint16_t* m_STM32F4_Display_VituralRam;
uint8_t m_STM32F4_Display_TextBuffer[LCD_MAX_COLUMN][LCD_MAX_ROW];

STM32F4xx_LCD_Rotation m_STM32F4_Display_CurrentRotation = STM32F4xx_LCD_Rotation::rotateNormal_0;

bool STM32F4_Display_Initialize();
bool STM32F4_Display_Uninitialize();
bool STM32F4_Display_SetPinConfiguration();

void STM32F4_Display_WriteFormattedChar(uint8_t c);
void STM32F4_Display_WriteChar(uint8_t c, int32_t row, int32_t col);
void STM32F4_Display_BitBltEx(int32_t x, int32_t y, int32_t width, int32_t height, uint32_t data[]);
void STM32F4_Display_PaintPixel(uint32_t x, uint32_t y, uint8_t c);
void STM32F4_Display_Paint8HorizontalPixels(uint32_t x, uint32_t y, uint8_t p);
void STM32F4_Display_TextEnterClearMode();
void STM32F4_Display_PrintChracter(uint32_t x, uint32_t y, uint8_t c);
void STM32F4_Display_TextShiftColUp();
void STM32F4_Display_Clear();
void STM32F4_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight);

int32_t STM32F4_Display_GetWidth();
int32_t STM32F4_Display_GetHeight();
int32_t STM32F4_Display_BitPerPixel();
uint32_t STM32F4_Display_GetPixelClockDivider();
int32_t STM32F4_Display_GetOrientation();
uint32_t* STM32F4_Display_GetFrameBuffer();

static TinyCLR_Display_Provider displayProvider;
static TinyCLR_Api_Info displayApi;

bool STM32F4_Ltdc_Initialize(LTDC_HandleTypeDef *hltdc) {
    uint32_t tmp = 0, tmp1 = 0;

    /* Check the LTDC peripheral state */
    if (hltdc == nullptr) {
        return false;
    }

    /* Configures the HS, VS, DE and PC polarity */
    hltdc->Instance->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
    hltdc->Instance->GCR |= (uint32_t)(hltdc->Init.HSPolarity | hltdc->Init.VSPolarity | \
        hltdc->Init.DEPolarity | hltdc->Init.PCPolarity);

    /* Sets Synchronization size */
    hltdc->Instance->SSCR &= ~(LTDC_SSCR_VSH | LTDC_SSCR_HSW);
    tmp = (hltdc->Init.HorizontalSync << 16);
    hltdc->Instance->SSCR |= (tmp | hltdc->Init.VerticalSync);

    /* Sets Accumulated Back porch */
    hltdc->Instance->BPCR &= ~(LTDC_BPCR_AVBP | LTDC_BPCR_AHBP);
    tmp = (hltdc->Init.AccumulatedHBP << 16);
    hltdc->Instance->BPCR |= (tmp | hltdc->Init.AccumulatedVBP);

    /* Sets Accumulated Active Width */
    hltdc->Instance->AWCR &= ~(LTDC_AWCR_AAH | LTDC_AWCR_AAW);
    tmp = (hltdc->Init.AccumulatedActiveW << 16);
    hltdc->Instance->AWCR |= (tmp | hltdc->Init.AccumulatedActiveH);

    /* Sets Total Width */
    hltdc->Instance->TWCR &= ~(LTDC_TWCR_TOTALH | LTDC_TWCR_TOTALW);
    tmp = (hltdc->Init.TotalWidth << 16);
    hltdc->Instance->TWCR |= (tmp | hltdc->Init.TotalHeigh);

    /* Sets the background color value */
    tmp = ((uint32_t)(hltdc->Init.Backcolor.Green) << 8);
    tmp1 = ((uint32_t)(hltdc->Init.Backcolor.Red) << 16);
    hltdc->Instance->BCCR &= ~(LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED);
    hltdc->Instance->BCCR |= (tmp1 | tmp | hltdc->Init.Backcolor.Blue);

    /* Enable the transfer Error interrupt */
    hltdc->Instance->IER |= LTDC_IER_TERRIE;

    /* Enable the FIFO underrun interrupt */
    hltdc->Instance->IER |= LTDC_IER_FUIE;

    /* Enable LTDC by setting LTDCEN bit */
    hltdc->Instance->GCR |= LTDC_GCR_LTDCEN;

    return true;
}

void STM32F4_Ltdc_SetConfiguration(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx) {
    uint32_t tmp = 0;
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;

    /* Configures the horizontal start and stop position */
    tmp = ((pLayerCfg->WindowX1 + ((hltdc->Instance->BPCR & LTDC_BPCR_AHBP) >> 16)) << 16);
    LTDC_LAYER(hltdc, LayerIdx)->WHPCR &= ~(LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS);
    LTDC_LAYER(hltdc, LayerIdx)->WHPCR = ((pLayerCfg->WindowX0 + ((hltdc->Instance->BPCR & LTDC_BPCR_AHBP) >> 16) + 1) | tmp);

    /* Configures the vertical start and stop position */
    tmp = ((pLayerCfg->WindowY1 + (hltdc->Instance->BPCR & LTDC_BPCR_AVBP)) << 16);
    LTDC_LAYER(hltdc, LayerIdx)->WVPCR &= ~(LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS);
    LTDC_LAYER(hltdc, LayerIdx)->WVPCR = ((pLayerCfg->WindowY0 + (hltdc->Instance->BPCR & LTDC_BPCR_AVBP) + 1) | tmp);

    /* Specifies the pixel format */
    LTDC_LAYER(hltdc, LayerIdx)->PFCR &= ~(LTDC_LxPFCR_PF);
    LTDC_LAYER(hltdc, LayerIdx)->PFCR = (pLayerCfg->PixelFormat);

    /* Configures the default color values */
    tmp = ((uint32_t)(pLayerCfg->Backcolor.Green) << 8);
    tmp1 = ((uint32_t)(pLayerCfg->Backcolor.Red) << 16);
    tmp2 = (pLayerCfg->Alpha0 << 24);
    LTDC_LAYER(hltdc, LayerIdx)->DCCR &= ~(LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA);
    LTDC_LAYER(hltdc, LayerIdx)->DCCR = (pLayerCfg->Backcolor.Blue | tmp | tmp1 | tmp2);

    /* Specifies the constant alpha value */
    LTDC_LAYER(hltdc, LayerIdx)->CACR &= ~(LTDC_LxCACR_CONSTA);
    LTDC_LAYER(hltdc, LayerIdx)->CACR = (pLayerCfg->Alpha);

    /* Specifies the blending factors */
    LTDC_LAYER(hltdc, LayerIdx)->BFCR &= ~(LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1);
    LTDC_LAYER(hltdc, LayerIdx)->BFCR = (pLayerCfg->BlendingFactor1 | pLayerCfg->BlendingFactor2);

    /* Configures the color frame buffer start address */
    LTDC_LAYER(hltdc, LayerIdx)->CFBAR &= ~(LTDC_LxCFBAR_CFBADD);
    LTDC_LAYER(hltdc, LayerIdx)->CFBAR = (pLayerCfg->FBStartAdress);

    if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888) {
        tmp = 4;
    }
    else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888) {
        tmp = 3;
    }
    else if ((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
        (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565) || \
        (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
        (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88)) {
        tmp = 2;
    }
    else {
        tmp = 1;
    }

    /* Configures the color frame buffer pitch in byte */
    LTDC_LAYER(hltdc, LayerIdx)->CFBLR &= ~(LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP);
    LTDC_LAYER(hltdc, LayerIdx)->CFBLR = (((pLayerCfg->ImageWidth * tmp) << 16) | (((pLayerCfg->WindowX1 - pLayerCfg->WindowX0) * tmp) + 3));

    /* Configures the frame buffer line number */
    LTDC_LAYER(hltdc, LayerIdx)->CFBLNR &= ~(LTDC_LxCFBLNR_CFBLNBR);
    LTDC_LAYER(hltdc, LayerIdx)->CFBLNR = (pLayerCfg->ImageHeight);

    /* Enable LTDC_Layer by setting LEN bit */
    LTDC_LAYER(hltdc, LayerIdx)->CR |= (uint32_t)LTDC_LxCR_LEN;
}


void STM32F4_Ltdc_LayerConfiguration(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx) {
    /* Copy new layer configuration into handle structure */
    hltdc->LayerCfg[LayerIdx] = *pLayerCfg;

    /* Configure the LTDC Layer */
    STM32F4_Ltdc_SetConfiguration(hltdc, pLayerCfg, LayerIdx);

    /* Sets the Reload type */
    hltdc->Instance->SRCR = LTDC_SRCR_IMR;

}

bool STM32F4_Display_Initialize() {
    // InitializeConfiguration
    static LTDC_HandleTypeDef hltdc_F;
    LTDC_LayerCfgTypeDef      pLayerCfg;

    RCC->CR &= ~(RCC_CR_PLLSAION);

    STM32F4_Time_Delay(nullptr, 0x10000);

    RCC->PLLSAICFGR = ((0xC0) << 6) | ((0) << 16) | ((0x4) << 24) | ((0x5) << 28);

    MODIFY_REG(RCC->DCKCFGR, RCC_DCKCFGR_PLLSAIDIVR, (uint32_t)(0x00010000));

    RCC->CR |= (RCC_CR_PLLSAION);

    STM32F4_Time_Delay(nullptr, 0x10000);

    RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;

    //HorizontalSyncPolarity
    if (m_STM32F4_DisplayHorizontalSyncPolarity == false)
        hltdc_F.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    else
        hltdc_F.Init.HSPolarity = LTDC_HSPOLARITY_AH;
    //VerticalSyncPolarity
    if (m_STM32F4_DisplayVerticalSyncPolarity == false)
        hltdc_F.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    else
        hltdc_F.Init.VSPolarity = LTDC_VSPOLARITY_AH;

    //OutputEnablePolarity
    if (m_STM32F4_DisplayOutputEnablePolarity == false)
        hltdc_F.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    else
        hltdc_F.Init.DEPolarity = LTDC_DEPOLARITY_AH;

    //OutputEnablePolarity
    if (m_STM32F4_DisplayPixelPolarity == false)
        hltdc_F.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    else
        hltdc_F.Init.PCPolarity = LTDC_GCR_PCPOL;

    /* The RK043FN48H LCD 480x272 is selected */
    /* Timing Configuration */
    hltdc_F.Init.HorizontalSync = (m_STM32F4_DisplayHorizontalSyncPulseWidth - 1);
    hltdc_F.Init.VerticalSync = (m_STM32F4_DisplayVerticalSyncPulseWidth - 1);
    hltdc_F.Init.AccumulatedHBP = (m_STM32F4_DisplayHorizontalSyncPulseWidth + m_STM32F4_DisplayHorizontalBackPorch - 1);
    hltdc_F.Init.AccumulatedVBP = (m_STM32F4_DisplayVerticalSyncPulseWidth + m_STM32F4_DisplayVerticalBackPorch - 1);
    hltdc_F.Init.AccumulatedActiveH = (m_STM32F4_DisplayHeight + m_STM32F4_DisplayVerticalSyncPulseWidth + m_STM32F4_DisplayVerticalBackPorch - 1);
    hltdc_F.Init.AccumulatedActiveW = (m_STM32F4_DisplayWidth + m_STM32F4_DisplayHorizontalSyncPulseWidth + m_STM32F4_DisplayHorizontalBackPorch - 1);
    hltdc_F.Init.TotalHeigh = (m_STM32F4_DisplayHeight + m_STM32F4_DisplayVerticalSyncPulseWidth + m_STM32F4_DisplayVerticalBackPorch + m_STM32F4_DisplayVerticalFrontPorch - 1);
    hltdc_F.Init.TotalWidth = (m_STM32F4_DisplayWidth + m_STM32F4_DisplayHorizontalSyncPulseWidth + m_STM32F4_DisplayHorizontalBackPorch + m_STM32F4_DisplayHorizontalFrontPorch - 1);
    /* Layer1 Configuration ------------------------------------------------------*/

    /* Windowing configuration */
    /* In this case all the active display area is used to display a picture then :
     Horizontal start = horizontal synchronization + Horizontal back porch = 43
     Vertical start   = vertical synchronization + vertical back porch     = 12
     Horizontal stop = Horizontal start + window width -1 = 43 + 480 -1
     Vertical stop   = Vertical start + window height -1  = 12 + 272 -1      */
    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = m_STM32F4_DisplayWidth;
    pLayerCfg.WindowY0 = 0;
    pLayerCfg.WindowY1 = m_STM32F4_DisplayHeight;

    /* Configure R,G,B component values for LCD background color : all black background */
    hltdc_F.Init.Backcolor.Blue = 0;
    hltdc_F.Init.Backcolor.Green = 0;
    hltdc_F.Init.Backcolor.Red = 0;

    hltdc_F.Instance = LTDC;

    /* Pixel Format configuration*/
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;

    /* Start Address configuration : frame buffer is located at FLASH memory */
    pLayerCfg.FBStartAdress = (uint32_t)VIDEO_RAM_ADDRESS;

    /* Alpha constant (255 == totally opaque) */
    pLayerCfg.Alpha = 255;

    /* Default Color configuration (configure A,R,G,B component values) : no background color */
    pLayerCfg.Alpha0 = 0; /* fully transparent */
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;

    /* Configure blending factors */
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;

    /* Configure the number of lines and number of pixels per line */
    pLayerCfg.ImageWidth = pLayerCfg.WindowX1;
    pLayerCfg.ImageHeight = pLayerCfg.WindowY1;

    /* Configure the LTDC */
    if (!STM32F4_Ltdc_Initialize(&hltdc_F)) {
        return false;
    }

    /* Configure the Layer*/
    STM32F4_Ltdc_LayerConfiguration(&hltdc_F, &pLayerCfg, 1);

    m_STM32F4_DisplayEnable = true;

    return true;
}

bool STM32F4_Display_Uninitialize() {
    m_STM32F4_DisplayEnable = false;

    return true;
}

//====================================================
void STM32F4_Display_WriteFormattedChar(uint8_t c) {
    if (m_STM32F4_DisplayEnable == false)
        return;

    if (c == '\f') {
        STM32F4_Display_Clear();
        STM32F4_Display_TextEnterClearMode();
        m_STM32F4_Display_TextColumn = 0;

        return;
    }

    if (c == '\r') {
        m_STM32F4_Display_TextColumn = 0;

        return;
    }
    if (c == '\n') {
        m_STM32F4_Display_TextColumn = 0;

        if (++m_STM32F4_Display_TextRow >= LCD_MAX_ROW) {
            m_STM32F4_Display_TextRow = LCD_MAX_ROW - 1;
            STM32F4_Display_TextShiftColUp();
        }
        // clean the new line
        for (c = 0; c < (LCD_MAX_COLUMN - 1); c++) {
            m_STM32F4_Display_TextBuffer[c][m_STM32F4_Display_TextRow] = ' ';
        }

        return;
    }

    STM32F4_Display_PrintChracter(m_STM32F4_Display_TextColumn * 6, m_STM32F4_Display_TextRow * 8, c);
    m_STM32F4_Display_TextBuffer[m_STM32F4_Display_TextColumn][m_STM32F4_Display_TextRow] = c;

    if (++m_STM32F4_Display_TextColumn >= (LCD_MAX_COLUMN - 1)) {
        m_STM32F4_Display_TextColumn = 0;

        if (++m_STM32F4_Display_TextRow >= LCD_MAX_ROW) {
            m_STM32F4_Display_TextRow = LCD_MAX_ROW - 1;
            STM32F4_Display_TextShiftColUp();
        }
        else {
            // clean the new line
            for (c = 0; c < LCD_MAX_COLUMN; c++) {
                m_STM32F4_Display_TextBuffer[c][m_STM32F4_Display_TextRow] = ' ';
            }
        }
    }
}
//=======================================================
void STM32F4_Display_PaintPixel(uint32_t x, uint32_t y, uint8_t c) {
    volatile uint16_t * loc;

    if (m_STM32F4_DisplayEnable == false)
        return;

    if (x >= m_STM32F4_DisplayWidth)
        return;
    if (y >= m_STM32F4_DisplayHeight)
        return;

    loc = m_STM32F4_Display_VituralRam + (y *m_STM32F4_DisplayWidth) + (x);

    if (c)
        *loc = 0x0fff;
    else
        *loc = 0;
}
//=======================================================
void STM32F4_Display_Paint8HorizontalPixels(uint32_t x, uint32_t y, uint8_t p) {
    if (m_STM32F4_DisplayEnable == false)
        return;

    for (int32_t i = 0; i < 8; i++) {
        if (p&(1 << i))
            STM32F4_Display_PaintPixel(x, y + i, 1);
        else
            STM32F4_Display_PaintPixel(x, y + i, 0);//clear
    }
}
//===========================================================
void STM32F4_Display_TextEnterClearMode() {
    uint32_t r, c;

    if (m_STM32F4_DisplayEnable == false)
        return;

    STM32F4_Display_Clear();
    m_STM32F4_Display_TextRow = 0;
    m_STM32F4_Display_TextColumn = 0;

    for (r = 0; r < LCD_MAX_ROW; r++) {
        for (c = 0; c < (LCD_MAX_COLUMN - 1); c++) {
            m_STM32F4_Display_TextBuffer[c][r] = '1';
        }
    }
}
//===========================================================
void STM32F4_Display_PrintChracter(uint32_t x, uint32_t y, uint8_t c) {
    uint8_t i;

    if (m_STM32F4_DisplayEnable == false)
        return;

    for (i = 0; i < 5; i++)
        STM32F4_Display_Paint8HorizontalPixels(x + i, y, characters[c][i]);

    STM32F4_Display_Paint8HorizontalPixels(x + i, y, 0);
}

void STM32F4_Display_TextShiftColUp() {
    uint32_t r, c;

    if (m_STM32F4_DisplayEnable == false)
        return;

    // refresh with new data
    STM32F4_Display_Clear();
    m_STM32F4_Display_TextRow = 0;
    m_STM32F4_Display_TextColumn = 0;

    for (r = 0; r < (LCD_MAX_ROW - 1); r++) {
        for (c = 0; c < LCD_MAX_COLUMN - 1; c++) {
            m_STM32F4_Display_TextBuffer[c][r] = m_STM32F4_Display_TextBuffer[c][r + 1];
            STM32F4_Display_WriteFormattedChar(m_STM32F4_Display_TextBuffer[c][r]);
        }
    }
}

void STM32F4_Display_Clear() {
    if (m_STM32F4_DisplayEnable == false)
        return;

    memset((uint32_t*)m_STM32F4_Display_VituralRam, 0, VIDEO_RAM_SIZE);
}

const STM32F4_Gpio_Pin g_Display_ControllerPins[] = STM32F4_DISPLAY_CONTROLLER_PINS;
const STM32F4_Gpio_Pin g_Display_BacklightPin = STM32F4_DISPLAY_BACKLIGHT_PIN;
const STM32F4_Gpio_Pin g_Display_EnablePin = STM32F4_DISPLAY_ENABLE_PIN;

bool  STM32F4_Display_SetPinConfiguration() {
    for (int32_t i = 0; i < SIZEOF_ARRAY(g_Display_ControllerPins); i++) {
        STM32F4_GpioInternal_ConfigurePin(g_Display_ControllerPins[i].number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, g_Display_ControllerPins[i].alternateFunction);
    }

    if (m_STM32F4_DisplayOutputEnableIsFixed) {
        STM32F4_GpioInternal_ConfigurePin(g_Display_EnablePin.number, STM32F4_Gpio_PortMode::GeneralPurposeOutput, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, STM32F4_Gpio_AlternateFunction::AF0);
        STM32F4_GpioInternal_WritePin(g_Display_EnablePin.number, m_STM32F4_DisplayOutputEnablePolarity);
    }
    else {
        STM32F4_GpioInternal_ConfigurePin(g_Display_EnablePin.number, STM32F4_Gpio_PortMode::AlternateFunction, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, g_Display_EnablePin.alternateFunction);
    }

    // backlight D7
    STM32F4_GpioInternal_ConfigurePin(g_Display_BacklightPin.number, STM32F4_Gpio_PortMode::GeneralPurposeOutput, STM32F4_Gpio_OutputType::PushPull, STM32F4_Gpio_OutputSpeed::High, STM32F4_Gpio_PullDirection::None, g_Display_BacklightPin.alternateFunction);
    STM32F4_GpioInternal_WritePin(g_Display_BacklightPin.number, true);

    return true;
}
uint32_t* STM32F4_Display_GetFrameBuffer() {
    return (uint32_t*)m_STM32F4_Display_VituralRam;
}

int32_t STM32F4_Display_GetWidth() {
    int32_t width = m_STM32F4_DisplayWidth;
    int32_t height = m_STM32F4_DisplayHeight;

    STM32F4_Display_GetRotatedDimensions(&width, &height);

    return width;
}

int32_t STM32F4_Display_GetHeight() {
    int32_t width = m_STM32F4_DisplayWidth;
    int32_t height = m_STM32F4_DisplayHeight;

    STM32F4_Display_GetRotatedDimensions(&width, &height);

    return height;
}

uint32_t STM32F4_Display_GetPixelClockDivider() {
    return m_STM32F4_DisplayPixelClockRateKHz;
}
int32_t STM32F4_Display_GetOrientation() {
    return m_STM32F4_Display_CurrentRotation;
}

void  STM32F4_Display_MemCopy(void *dest, void *src, int32_t size) {
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

void STM32F4_Display_BitBltEx(int32_t x, int32_t y, int32_t width, int32_t height, uint32_t data[]) {

    int32_t xTo, yTo, xFrom, yFrom;
    int32_t xOffset = x;
    int32_t yOffset = y;
    uint16_t *from = (uint16_t *)data;
    uint16_t *to = (uint16_t *)m_STM32F4_Display_VituralRam;


    int32_t screenWidth = m_STM32F4_DisplayWidth;
    int32_t screenHeight = m_STM32F4_DisplayHeight;
    int32_t startPx, toAddition;

    if (m_STM32F4_DisplayEnable == false)
        return;

    switch (m_STM32F4_Display_CurrentRotation) {
    case STM32F4xx_LCD_Rotation::rotateNormal_0:

        if (xOffset == 0 && yOffset == 0 &&
            width == screenWidth &&    height == screenHeight) {
            STM32F4_Display_MemCopy(to, from, (screenWidth*screenHeight * 2));
        }
        else {
            for (yTo = yOffset; yTo < (yOffset + height); yTo++) {
                STM32F4_Display_MemCopy((void*)(to + yTo * screenWidth + xOffset), (void*)(from + yTo * screenWidth + xOffset), (width * 2));
            }
        }

        break;

    case STM32F4xx_LCD_Rotation::rotateCCW_90:

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

    case STM32F4xx_LCD_Rotation::rotateCW_90:

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

    case STM32F4xx_LCD_Rotation::rotate_180:

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

void STM32F4_Display_WriteChar(uint8_t c, int32_t row, int32_t col) {
    m_STM32F4_Display_TextRow = row;
    m_STM32F4_Display_TextColumn = col;
    STM32F4_Display_WriteFormattedChar(c);

}
void STM32F4_Display_GetRotatedDimensions(int32_t *screenWidth, int32_t *screenHeight) {
    switch (m_STM32F4_Display_CurrentRotation) {
    case STM32F4xx_LCD_Rotation::rotateNormal_0:
    case STM32F4xx_LCD_Rotation::rotate_180:
        *screenWidth = m_STM32F4_DisplayWidth;
        *screenHeight = m_STM32F4_DisplayHeight;
        break;

    case STM32F4xx_LCD_Rotation::rotateCCW_90:
    case STM32F4xx_LCD_Rotation::rotateCW_90:
        *screenWidth = m_STM32F4_DisplayHeight;
        *screenHeight = m_STM32F4_DisplayWidth;
        break;
    }
}

TinyCLR_Result STM32F4_Display_Acquire(const TinyCLR_Display_Provider* self) {
    m_STM32F4_Display_CurrentRotation = STM32F4xx_LCD_Rotation::rotateNormal_0;

    m_STM32F4_Display_VituralRam = (uint16_t*)VIDEO_RAM_ADDRESS;

    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Display_Release(const TinyCLR_Display_Provider* self) {
    if (STM32F4_Display_Uninitialize())
        return TinyCLR_Result::Success;

    return  TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result STM32F4_Display_Enable(const TinyCLR_Display_Provider* self) {
    if (STM32F4_Display_SetPinConfiguration() && STM32F4_Display_Initialize())
        return TinyCLR_Result::Success;

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result STM32F4_Display_Disable(const TinyCLR_Display_Provider* self) {
    return TinyCLR_Result::NotSupported;
}

TinyCLR_Result STM32F4_Display_SetConfiguration(const TinyCLR_Display_Provider* self, TinyCLR_Display_DataFormat dataFormat, uint32_t width, uint32_t height, const void* configuration) {
    if (dataFormat != TinyCLR_Display_DataFormat::Rgb565) return TinyCLR_Result::NotSupported;

    m_STM32F4_DisplayWidth = width;
    m_STM32F4_DisplayHeight = height;

    if (configuration != nullptr) {
        auto& cfg = *(const TinyCLR_Display_ParallelConfiguration*)configuration;

        m_STM32F4_DisplayOutputEnableIsFixed = cfg.DataEnableIsFixed;
        m_STM32F4_DisplayOutputEnablePolarity = cfg.DataEnablePolarity;
        m_STM32F4_DisplayPixelPolarity = cfg.PixelPolarity;

        m_STM32F4_DisplayPixelClockRateKHz = cfg.PixelClockRate / 1000;

        m_STM32F4_DisplayHorizontalSyncPolarity = cfg.HorizontalSyncPolarity;

        m_STM32F4_DisplayHorizontalSyncPulseWidth = cfg.HorizontalSyncPulseWidth;
        m_STM32F4_DisplayHorizontalFrontPorch = cfg.HorizontalFrontPorch;
        m_STM32F4_DisplayHorizontalBackPorch = cfg.HorizontalBackPorch;

        m_STM32F4_DisplayVerticalSyncPolarity = cfg.VerticalSyncPolarity;

        m_STM32F4_DisplayVerticalSyncPulseWidth = cfg.VerticalSyncPulseWidth;
        m_STM32F4_DisplayVerticalFrontPorch = cfg.VerticalFrontPorch;
        m_STM32F4_DisplayVerticalBackPorch = cfg.VerticalBackPorch;
    }

    return  TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Display_GetConfiguration(const TinyCLR_Display_Provider* self, TinyCLR_Display_DataFormat& dataFormat, uint32_t& width, uint32_t& height, void* configuration) {
    dataFormat = TinyCLR_Display_DataFormat::Rgb565;
    width = m_STM32F4_DisplayWidth;
    height = m_STM32F4_DisplayHeight;

    if (configuration != nullptr) {
        auto& cfg = *(TinyCLR_Display_ParallelConfiguration*)configuration;

        cfg.DataEnableIsFixed = m_STM32F4_DisplayOutputEnableIsFixed;
        cfg.DataEnablePolarity = m_STM32F4_DisplayOutputEnablePolarity;
        cfg.PixelPolarity = m_STM32F4_DisplayPixelPolarity;

        cfg.PixelClockRate = m_STM32F4_DisplayPixelClockRateKHz * 1000;

        cfg.HorizontalSyncPolarity = m_STM32F4_DisplayHorizontalSyncPolarity;

        cfg.HorizontalSyncPulseWidth = m_STM32F4_DisplayHorizontalSyncPulseWidth;
        cfg.HorizontalFrontPorch = m_STM32F4_DisplayHorizontalFrontPorch;
        cfg.HorizontalBackPorch = m_STM32F4_DisplayHorizontalBackPorch;

        cfg.VerticalSyncPolarity = m_STM32F4_DisplayVerticalSyncPolarity;

        cfg.VerticalSyncPulseWidth = m_STM32F4_DisplayVerticalSyncPulseWidth;
        cfg.VerticalFrontPorch = m_STM32F4_DisplayVerticalFrontPorch;
        cfg.VerticalBackPorch = m_STM32F4_DisplayVerticalBackPorch;
    }

    return TinyCLR_Result::InvalidOperation;
}

TinyCLR_Result STM32F4_Display_DrawBuffer(const TinyCLR_Display_Provider* self, int32_t x, int32_t y, int32_t width, int32_t height, const uint8_t* data) {
    STM32F4_Display_BitBltEx(x, y, width, height, (uint32_t*)data);
    return TinyCLR_Result::Success;
}

TinyCLR_Result STM32F4_Display_WriteString(const TinyCLR_Display_Provider* self, const char* buffer, size_t length) {
    for (size_t i = 0; i < length; i++)
        STM32F4_Display_WriteFormattedChar(buffer[i]);

    return TinyCLR_Result::Success;
}

TinyCLR_Display_DataFormat dataFormats[] = { TinyCLR_Display_DataFormat::Rgb565 };

TinyCLR_Result STM32F4_Display_GetCapabilities(const TinyCLR_Display_Provider* self, TinyCLR_Display_InterfaceType& type, const TinyCLR_Display_DataFormat*& supportedDataFormats, size_t& supportedDataFormatCount) {
    type = TinyCLR_Display_InterfaceType::Parallel;
    supportedDataFormatCount = SIZEOF_ARRAY(dataFormats);
    supportedDataFormats = dataFormats;

    return TinyCLR_Result::Success;
}

const TinyCLR_Api_Info* STM32F4_Display_GetApi() {
    displayProvider.Parent = &displayApi;
    displayProvider.Index = 0;
    displayProvider.Acquire = &STM32F4_Display_Acquire;
    displayProvider.Release = &STM32F4_Display_Release;
    displayProvider.Enable = &STM32F4_Display_Enable;
    displayProvider.Disable = &STM32F4_Display_Disable;
    displayProvider.SetConfiguration = &STM32F4_Display_SetConfiguration;
    displayProvider.GetConfiguration = &STM32F4_Display_GetConfiguration;
    displayProvider.GetCapabilities = &STM32F4_Display_GetCapabilities;
    displayProvider.DrawBuffer = &STM32F4_Display_DrawBuffer;
    displayProvider.WriteString = &STM32F4_Display_WriteString;

    displayApi.Author = "GHI Electronics, LLC";
    displayApi.Name = "GHIElectronics.TinyCLR.NativeApis.STM32F4.DisplayProvider";
    displayApi.Type = TinyCLR_Api_Type::DisplayProvider;
    displayApi.Version = 0;
    displayApi.Count = 1;
    displayApi.Implementation = &displayProvider;

    return &displayApi;
}

void STM32F4_Display_Reset() {
    STM32F4_Display_Uninitialize();
}

#endif //INCLUDE_DISPLAY
