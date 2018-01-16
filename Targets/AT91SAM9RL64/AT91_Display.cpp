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
//LUT configurations
typedef struct _PALETTEENTRY_LCD {
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
    unsigned char Flags;
} PALETTEENTRY_LCD, *PPALETTEENTRY_LCD;


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



#define VIDEO_RAM_SIZE                  (800*600*2) // Maximum LCD screen size times bytes per pixel

#define LCD_MAX_ROW	                    32
#define LCD_MAX_COLUMN                  70

#define BITS_PER_PIXEL              16

/* More or less configurable parameters for LCDC controller*/
#define SIDSAFB_FIFO_SIZE		512
#define SIDSAFB_DMA_BURST_LEN	5
#define SIDSAFB_CRST_VAL        0xc8   // 0xda


enum AT91_LCD_Rotation {
    rotateNormal_0,
    rotateCW_90,
    rotate_180,
    rotateCCW_90,
};

int64_t m_AT91_Display_ReservedVitualRamLocation[VIDEO_RAM_SIZE / 8];

//static uint32_t* AT91_LCD_Screen_Buffer = (uint32_t*)0x20F00000;

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
uint32_t m_AT91_Display_PixelClockDivider = 0;
uint32_t m_AT91_Display_BitsPerPixel = 16;
uint32_t m_AT91_Display_EnableTFT = true;

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

static const AT91_Gpio_Pin g_at91_display_dataPins[] = AT91_DISPLAY_DATA_PINS;
static const AT91_Gpio_Pin g_at91_display_controlPins[] = AT91_DISPLAY_CONTROL_PINS;
static const AT91_Gpio_Pin g_at91_display_enablePin = AT91_DISPLAY_ENABLE_PIN;

bool AT91_Display_Initialize() {

    if (m_AT91_DisplayPixelClockRateKHz == 0x0) {

        return true;
    }

    // Pixel Clock Divider is calculated by this equation Frequency = System Clock / ((Divider + 1) * 2)
    // The following equation should calculate Pixel Clock Divider
    // Divider = ((AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 2) - (m_AT91_DisplayPixelClockRateKHz * 1000)) / (m_AT91_DisplayPixelClockRateKHz * 1000);
    if (m_AT91_DisplayPixelClockRateKHz > 25000) // 25000 KHz is maximum supported frequency
        m_AT91_DisplayPixelClockRateKHz = 25000;

    m_AT91_Display_PixelClockDivider = ((AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 2) - (m_AT91_DisplayPixelClockRateKHz * 1000)) / (m_AT91_DisplayPixelClockRateKHz * 1000);

    if (((AT91_SYSTEM_PERIPHERAL_CLOCK_HZ / 2) - (m_AT91_DisplayPixelClockRateKHz * 1000)) % (m_AT91_DisplayPixelClockRateKHz * 1000) > 0)
        m_AT91_Display_PixelClockDivider += 1;

    if (m_AT91_Display_PixelClockDivider > 511) // Pixel clock divider cannot exceed a 9 bit number which is 511 in Decimal.
        m_AT91_Display_PixelClockDivider = 511;


    /////////////////////////////////////////////

    uint32_t pin;
    uint32_t value;

    /* Selected as LCD pins */
    for (pin = 0; pin < SIZEOF_ARRAY(g_at91_display_dataPins); pin++) {
        AT91_Gpio_ConfigurePin(g_at91_display_dataPins[pin].number, AT91_Gpio_Direction::Input, g_at91_display_dataPins[pin].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
    }
    for (pin = 0; pin < SIZEOF_ARRAY(g_at91_display_controlPins); pin++) {
        AT91_Gpio_ConfigurePin(g_at91_display_controlPins[pin].number, AT91_Gpio_Direction::Input, g_at91_display_controlPins[pin].peripheralSelection, AT91_Gpio_ResistorMode::Inactive);
    }

    /* Enable CS for LCD */
    //CPU_GPIO_EnableOutputPin((GPIO_PIN)AT91_LCDC_CS, 0);
    if (m_AT91_DisplayOutputEnableIsFixed)
        AT91_Gpio_EnableOutputPin(g_at91_display_enablePin.number, m_AT91_DisplayOutputEnablePolarity);
    else
        AT91_Gpio_ConfigurePin(g_at91_display_enablePin.number, AT91_Gpio_Direction::Input, g_at91_display_enablePin.peripheralSelection, AT91_Gpio_ResistorMode::Inactive);

    AT91_PMC &pmc = AT91::PMC();
    pmc.EnablePeriphClock(AT91C_ID_LCDC);

    AT91_LCDC &lcdc = AT91::LCDC();

    /* Turn off the LCD controller and the DMA controller */
    lcdc.LCDC_PWRCON = 0x0C;
    lcdc.LCDC_DMACON = 0;

    // Reset LCDC DMA
    lcdc.LCDC_DMACON = AT91_LCDC::LCDC_DMARST;

    /* ...set frame size and burst length = 8 words (?) */
    value = (m_AT91_DisplayHeight * m_AT91_DisplayWidth * m_AT91_Display_BitsPerPixel) >> 5; // / 32;
    value |= 0x04000000;
    lcdc.LCDC_FRMCFG = value;

    /* Set pixel clock divider */
    if (m_AT91_Display_PixelClockDivider == 0)
        lcdc.LCDC_LCDCON1 = AT91_LCDC::LCDC_BYPASS;
    else
        lcdc.LCDC_LCDCON1 = m_AT91_Display_PixelClockDivider << 12;

    /* Set configurations for LCDCON2 */
    if (m_AT91_Display_EnableTFT)
        value = (AT91_LCDC::LCDC_MEMOR_LITTLEIND | AT91_LCDC::LCDC_DISTYPE_TFT | AT91_LCDC::LCDC_CLKMOD);
    else
        value = (AT91_LCDC::LCDC_MEMOR_LITTLEIND | AT91_LCDC::LCDC_CLKMOD);

    if (m_AT91_DisplayHorizontalSyncPolarity == false) // Original was if it is true set 1
        value |= 1 << 10;   /* INVLINE */
    if (m_AT91_DisplayVerticalSyncPolarity == false) // Original was if it is true set 1
        value |= 1 << 9;    /* INVFRAME */

    switch (m_AT91_Display_BitsPerPixel) {
        //Bits per pixel: 0 = 1, 1 = 2, 2 = 4, 3 = 8, 4 = 16, 5 = 24
    case 1:
        value |= (0 << 5);
        break;
    case 2:
        value |= (1 << 5);
        break;
    case 4:
        value |= (2 << 5);
        break;
    case 8:
        value |= (3 << 5);
        break;
    case 16:
        value |= (4 << 5);
        break;
    case 24:
        value |= (5 << 5);
        break;
    default:
        //debug_printf("Unsupported LCD Bus Width : %d\r\n", m_AT91_Display_BitsPerPixel);
        return false;
    };

    lcdc.LCDC_LCDCON2 = value;

    for (uint32_t i = 0; i < 256; i++) {
        lcdc.LCDC_LUT_ENTRY[i] =
            ((c_rgbPalette[i].Red) >> 3 |
            ((c_rgbPalette[i].Green & 0xf8) << 2) |
                ((c_rgbPalette[i].Blue & 0xf8) << 7));
    }

    /* Horizontal timing */
    value = (m_AT91_DisplayHorizontalFrontPorch - 1) << 21;
    value |= (m_AT91_DisplayHorizontalSyncPulseWidth - 1) << 8;
    value |= m_AT91_DisplayHorizontalBackPorch - 1;
    lcdc.LCDC_TIM2 = value;

    /* Vertical timing */
    value = (m_AT91_DisplayVerticalSyncPulseWidth - 1) << 16;




    m_AT91_DisplayVerticalBackPorch += m_AT91_DisplayVerticalSyncPulseWidth; //1;//10; For Hydra or Atmel you need to add the VPW to the VBP in order for the screen to align properly.

    value |= m_AT91_DisplayVerticalBackPorch << 8;

    value |= m_AT91_DisplayVerticalFrontPorch;
    lcdc.LCDC_TIM1 = value;

    value = (m_AT91_DisplayWidth - 1) << 21;
    value |= (m_AT91_DisplayHeight - 1);

    lcdc.LCDC_LCDFRCFG = value;

    /* FIFO Threshold: Use formula from data sheet */
    value = SIDSAFB_FIFO_SIZE - (2 * SIDSAFB_DMA_BURST_LEN + 3);
    lcdc.LCDC_FIFO = value;

    /* Toggle LCD_MODE every frame */
    //lcdc.LCDC_MVAL = 0;

    //STN Dithering
    if (!(m_AT91_Display_EnableTFT)) {
        lcdc.LCDC_DP1_2 = 0xA5;
        lcdc.LCDC_DP4_7 = 0x05AF0FA5;
        lcdc.LCDC_DP3_5 = 0x000A5A5F;
        lcdc.LCDC_DP2_3 = 0x00000A5F;
        lcdc.LCDC_DP5_7 = 0x0FAF5FA5;
        lcdc.LCDC_DP3_4 = 0x0000FAF5;
        lcdc.LCDC_DP4_5 = 0x000FAF5F;
        lcdc.LCDC_DP6_7 = 0x0F5FFAFF;
    }

    /* Disable all interrupts */
    lcdc.LCDC_IDR = ~0UL;
    // Set contrast
    value = AT91_LCDC::LCDC_PS_DIVIDEDBYEIGHT | AT91_LCDC::LCDC_POL_POSITIVEPULSE | AT91_LCDC::LCDC_ENA_PWMGEMENABLED;
    lcdc.LCDC_CTRSTCON = value;
    lcdc.LCDC_CTRSTVAL = 0xDA;

    lcdc.LCDC_BA1 = (uint32_t)m_AT91_Display_ReservedVitualRamLocation;
    lcdc.LCDC_FRMCFG = (4 << 24) + (m_AT91_DisplayHeight * m_AT91_DisplayWidth * m_AT91_Display_BitsPerPixel >> 5);

    // Enable
    lcdc.LCDC_DMACON = AT91_LCDC::LCDC_DMAEN;
    lcdc.LCDC_PWRCON = AT91_LCDC::LCDC_PWR | (0x0F << 1);

    m_AT91_DisplayEnable = true;

    return true;
}

bool AT91_Display_Uninitialize() {
    int32_t i;

    if (m_AT91_DisplayEnable == false)
        return true;

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
    case AT91_LCD_Rotation::rotateNormal_0:

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

TinyCLR_Result AT91_Display_Acquire(const TinyCLR_Display_Provider* self, uint32_t width, uint32_t height) {
    m_AT91_DisplayWidth = width;
    m_AT91_DisplayHeight = height;

    m_AT91_Display_CurrentRotation = AT91_LCD_Rotation::rotateNormal_0;

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
    while (buffer != 0) {
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
