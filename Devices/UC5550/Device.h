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

#define STM32F769xx 1

#include <STM32F7.h>

#define DEVICE_TARGET STM32F7
#define DEVICE_NAME "UC5550"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((1ULL << 48) | (0ULL << 32) | (0ULL << 16) | (10000ULL << 0))

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x500D

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(I, 3)
#define DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::High

#define RUN_APP_PIN PIN(I, 1)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define BOOTLOADER_HOLD_ADDRESS 0x2002FFF8
#define BOOTLOADER_HOLD_VALUE 0xA9DE5BAC

#define DEPLOYMENT_SECTORS { { 0x14, 0x08180000, 0x00020000 }, { 0x15, 0x081A0000, 0x00020000 }, { 0x16, 0x081C0000, 0x00020000 }, { 0x17, 0x081E0000, 0x00020000 } }

#define STM32F7_SYSTEM_CLOCK_HZ 216000000
#define STM32F7_AHB_CLOCK_HZ 216000000
#define STM32F7_APB1_CLOCK_HZ 54000000
#define STM32F7_APB2_CLOCK_HZ 108000000
#define STM32F7_EXT_CRYSTAL_CLOCK_HZ 8000000
#define STM32F7_SUPPLY_VOLTAGE_MV 3300

#define INCLUDE_ADC

#define INCLUDE_CAN
#define TOTAL_CAN_CONTROLLERS 2
#define STM32F7_CAN_BUFFER_DEFAULT_SIZE { 128 , 128 }
#define STM32F7_CAN_TX_PINS { { PIN(H, 13), AF(9) }, { PIN(B, 13), AF(9) } }
#define STM32F7_CAN_RX_PINS { { PIN(I,  9), AF(9) }, { PIN(B, 12), AF(9) } }

#define INCLUDE_DAC

#define INCLUDE_GPIO
#define STM32F7_GPIO_PINS {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15      */\
                           /*PAx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PBx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PCx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PDx*/ NO_INIT(), NO_INIT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                           /*PEx*/ NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                           /*PFx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                           /*PGx*/ NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(),\
                           /*PHx*/ DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PIx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PJx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PKx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                          }

#define INCLUDE_I2C
#define TOTAL_I2C_CONTROLLERS 1
#define STM32F7_I2C_SCL_PINS { { PIN(B, 8), AF(4) } }
#define STM32F7_I2C_SDA_PINS { { PIN(B, 9), AF(4) } }

#define INCLUDE_POWER

#define INCLUDE_PWM
#define TOTAL_PWM_CONTROLLERS 14
#define STM32F7_PWM_PINS {/*              1                        2                        3                        4                     */\
                          /* TIM1  */ { { PIN_NONE  , AF_NONE }, { PIN(A,  9), AF(1)   }, { PIN(A, 10), AF(1)   }, { PIN_NONE  , AF_NONE } },\
                          /* TIM2  */ { { PIN(A, 15), AF(1)   }, { PIN_NONE  , AF_NONE }, { PIN(B, 10), AF(1)   }, { PIN(B, 11), AF(1)   } },\
                          /* TIM3  */ { { PIN(C,  6), AF(2)   }, { PIN(C,  7), AF(2)   }, { PIN(B,  0), AF(2)   }, { PIN(B,  1), AF(2)   } },\
                          /* TIM4  */ { { PIN_NONE  , AF_NONE }, { PIN(B,  7), AF(2)   }, { PIN(B,  8), AF(2)   }, { PIN(B,  9), AF(2)   } },\
                          /* TIM5  */ { { PIN(A,  0), AF(2)   }, { PIN(H, 11), AF(2)   }, { PIN_NONE  , AF_NONE }, { PIN(I,  0), AF(2)   } },\
                          /* TIM6  */ { { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM7  */ { { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM8  */ { { PIN(I,  5), AF(3)   }, { PIN(I,  6), AF(3)   }, { PIN(I,  7), AF(3)   }, { PIN(I,  2), AF(3)   } },\
                          /* TIM9  */ { { PIN_NONE  , AF_NONE }, { PIN(A,  3), AF(3)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM10 */ { { PIN(F,  6), AF(3)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM11 */ { { PIN(F,  7), AF(3)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM12 */ { { PIN(H,  6), AF(9)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM13 */ { { PIN(F,  8), AF(9)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM14 */ { { PIN(F,  9), AF(9)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                         }

#define INCLUDE_RTC

#define INCLUDE_SD
#define STM32F7_SD_DATA0_PINS { { PIN(C, 8), AF(12) } }
#define STM32F7_SD_DATA1_PINS { { PIN(C, 9), AF(12) } }
#define STM32F7_SD_DATA2_PINS { { PIN(C, 10), AF(12) } }
#define STM32F7_SD_DATA3_PINS { { PIN(C, 11), AF(12) } }
#define STM32F7_SD_CLK_PINS { { PIN(C, 12), AF(12) } }
#define STM32F7_SD_CMD_PINS { { PIN(D, 2), AF(12) } }

#define INCLUDE_SIGNALS

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 5
#define STM32F7_SPI_SCLK_PINS { { PIN(B, 3), AF(5) }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN(H, 6), AF(5) } }
#define STM32F7_SPI_MISO_PINS { { PIN(B, 4), AF(5) }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN(F, 8), AF(5) } }
#define STM32F7_SPI_MOSI_PINS { { PIN(B, 5), AF(5) }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN(F, 9), AF(5) } }

#define INCLUDE_STORAGE

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 7
#define STM32F7_UART_DEFAULT_TX_BUFFER_SIZE { 256, 256, 256, 256, 256, 256, 256 }
#define STM32F7_UART_DEFAULT_RX_BUFFER_SIZE { 512, 512, 512, 512, 512, 512, 512 }
#define STM32F7_UART_TX_PINS  { { PIN(A,  9), AF(7)   }, { PIN(D, 5), AF(7) }, { PIN(B, 10), AF(7)   }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN(C, 6), AF(8)   }, { PIN(F, 7), AF(8) } }
#define STM32F7_UART_RX_PINS  { { PIN(A, 10), AF(7)   }, { PIN(D, 6), AF(7) }, { PIN(B, 11), AF(7)   }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN(C, 7), AF(8)   }, { PIN(F, 6), AF(8) } }
#define STM32F7_UART_CTS_PINS { { PIN_NONE  , AF_NONE }, { PIN(D, 3), AF(7) }, { PIN_NONE  , AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE , AF_NONE }, { PIN(F, 8), AF(8) } }
#define STM32F7_UART_RTS_PINS { { PIN_NONE  , AF_NONE }, { PIN(D, 4), AF(7) }, { PIN_NONE  , AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE, AF_NONE }, { PIN_NONE , AF_NONE }, { PIN(F, 9), AF(8) } }

#define INCLUDE_USBCLIENT
#define STM32F7_TOTAL_USB_CONTROLLERS 1
#define STM32F7_USB_PACKET_FIFO_COUNT 64
#define STM32F7_USB_ENDPOINT_SIZE 64
#define STM32F7_USB_ENDPOINT0_SIZE 8
#define STM32F7_USB_ENDPOINT_COUNT 4
#define STM32F7_USB_PIPE_COUNT 4

#define STM32F7_USB_DM_PINS { { PIN(A, 11), AF(10) } }
#define STM32F7_USB_DP_PINS { { PIN(A, 12), AF(10) } }
#define STM32F7_USB_VB_PINS { { PIN(A,  9), AF(10) } }
#define STM32F7_USB_ID_PINS { { PIN(A, 10), AF(10) } }

#define INCLUDE_DISPLAY
#define STM32F7_DISPLAY_CONTROLLER_PINS { { PIN(I, 12), AF(14) }, { PIN(I, 13), AF(14) }, { PIN(I, 14), AF(14) }, { PIN(J, 2), AF(14) },{ PIN(J, 3), AF(14) },{ PIN(J, 4), AF(14) },{ PIN(J, 5), AF(14) },{ PIN(J, 6), AF(14) },{ PIN(J, 9), AF(14) },{ PIN(J, 10), AF(14) },{ PIN(J, 11), AF(14) },{ PIN(J, 15), AF(14) }, { PIN(K, 0), AF(14) },{ PIN(K, 1), AF(14) },{ PIN(K, 2), AF(14) },{ PIN(K, 3), AF(14) },{ PIN(K, 4), AF(14) },{ PIN(K, 5), AF(14) },{ PIN(K, 6), AF(14) } }
#define STM32F7_DISPLAY_BACKLIGHT_PIN  { PIN(D, 7), AF(0) }
#define STM32F7_DISPLAY_ENABLE_PIN  { PIN(K, 7), AF(14) }

#define MEMORY_PROFILE_FACTOR_LEVEL 9