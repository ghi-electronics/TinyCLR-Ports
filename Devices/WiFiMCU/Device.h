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

#define STM32F411xE 1

#include <STM32F4.h>

#define DEVICE_TARGET STM32F4
#define DEVICE_NAME "WiFiMCU"
#define DEVICE_MANUFACTURER "Doit.am"
#define DEVICE_VERSION ((0ULL << 48) | (6ULL << 32) | (0ULL << 16) | (0ULL << 0))

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x5000

#define UART_DEBUGGER_INDEX 1

#define DEBUGGER_FORCE_API		STM32F4_Uart_GetApi()
#define DEBUGGER_FORCE_INDEX	UART_DEBUGGER_INDEX

#define RUN_APP_PIN PIN(B, 1)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define DEPLOYMENT_SECTORS { { 0x06, 0x08040000, 0x00020000 }, { 0x07, 0x08060000, 0x00020000 } }

#define STM32F4_SYSTEM_CLOCK_HZ 96000000
#define STM32F4_AHB_CLOCK_HZ 96000000
#define STM32F4_APB1_CLOCK_HZ 48000000
#define STM32F4_APB2_CLOCK_HZ 96000000
#define STM32F4_EXT_CRYSTAL_CLOCK_HZ 26000000
#define STM32F4_SUPPLY_VOLTAGE_MV 3300

#define INCLUDE_ADC

#define INCLUDE_GPIO
#define STM32F4_GPIO_PINS {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15      */\     3          4          5          6          7          8          9          10         11         12         13         14         15      */\    2               3               4               5               6               7               8               9               10              11              12              13              14              15      */\
                           /*PAx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(),\
                           /*PBx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PCx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                           /*PDx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT()\
                          }

#define INCLUDE_I2C
#define STM32F4_I2C_SCL_PINS { { PIN(B, 8), AF(4) } }
#define STM32F4_I2C_SDA_PINS { { PIN(B, 9), AF(4) } }

#define INCLUDE_PWM
#define STM32F4_PWM_PINS {/*          0                       1                     2                      3                      */\
                          /* TIM1  */ { { PIN(A, 8), AF(1) }, { PIN(A, 9), AF(1) }, { PIN(A, 10), AF(1) }, { PIN_NONE , AF_NONE } },\
                          /* TIM2  */ { { PIN(A, 5), AF(1) }, { PIN(B, 3), AF(1) }, { PIN(A,  2), AF(1) }, { PIN(A, 3), AF(1)   } },\
                          /* TIM3  */ { { PIN(A, 6), AF(2) }, { PIN(C, 7), AF(2) }, { PIN(C,  8), AF(2) }, { PIN(C, 9), AF(2)   } },\
                          /* TIM4  */ { { PIN(B, 6), AF(2) }, { PIN(B, 7), AF(2) }, { PIN(B,  8), AF(2) }, { PIN(B, 9), AF(2)   } },\
                         }

#define INCLUDE_SPI
#define STM32F4_SPI_SCLK_PINS { { PIN(A, 5), AF(5) }, { PIN(B, 3), AF(5) } }
#define STM32F4_SPI_MISO_PINS { { PIN(A, 6), AF(5) }, { PIN(B, 4), AF(5) } }
#define STM32F4_SPI_MOSI_PINS { { PIN(A, 7), AF(5) }, { PIN(B, 5), AF(5) } }

#define INCLUDE_UART
#define STM32F4_UART_DEFAULT_TX_BUFFER_SIZE  { 256, 256 }
#define STM32F4_UART_DEFAULT_RX_BUFFER_SIZE  { 512, 512 }
#define STM32F4_UART_TX_PINS  { { PIN(B,  6), AF(7)   }, { PIN(A, 2), AF(7)   } }
#define STM32F4_UART_RX_PINS  { { PIN(A, 10), AF(7)   }, { PIN(A, 3), AF(7)   } }
#define STM32F4_UART_CTS_PINS { { PIN_NONE  , AF_NONE }, { PIN_NONE , AF_NONE } }
#define STM32F4_UART_RTS_PINS { { PIN_NONE  , AF_NONE }, { PIN_NONE , AF_NONE } }

//#define INCLUDE_USBCLIENT
#define STM32F4_USB_QUEUE_SIZE 16
#define STM32F4_USB_FIFO_BUFFER_SIZE 32
#define STM32F4_USB_DM_PINS { { PIN(A, 11), AF(10) } }
#define STM32F4_USB_DP_PINS { { PIN(A, 12), AF(10) } }
#define STM32F4_USB_VB_PINS { { PIN(A,  9), AF(10) } }
#define STM32F4_USB_ID_PINS { { PIN(A, 10), AF(10) } }