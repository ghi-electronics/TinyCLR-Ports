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

#define STM32F427xx 1

#include <STM32F4.h>

#define DEVICE_TARGET STM32F4
#define DEVICE_NAME "G80"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((0x0000ULL << 32) | (0x0006ULL << 16) | (0x0000ULL << 0))

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x0110

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(E, 15)
#define DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::High

#define RUN_APP_PIN PIN(E, 4)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define BOOTLOADER_HOLD_ADDRESS 0x2002FFF8
#define BOOTLOADER_HOLD_VALUE 0x37D56D4A

#define DEPLOYMENT_SECTOR_ADDRESSES { 0x080C0000, 0x080E0000 }
#define DEPLOYMENT_SECTOR_SIZES { 0x00020000, 0x00020000 }

#define STM32F4_SYSTEM_CLOCK_HZ 180000000
#define STM32F4_AHB_CLOCK_HZ 180000000
#define STM32F4_APB1_CLOCK_HZ 45000000
#define STM32F4_APB2_CLOCK_HZ 90000000
#define STM32F4_CRYSTAL_CLOCK_HZ 12000000
#define STM32F4_SUPPLY_VOLTAGE_MV 3300

#define INCLUDE_ADC

#define INCLUDE_DAC

#define INCLUDE_GPIO
#define STM32F4_GPIO_PINS {/*      0                              1                              2                              3                              4                              5                              6                              7                              8                              9                              10                             11                             12                             13                             14                             15                          */\
                           /*PAx*/ INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp),\
                           /*PBx*/ INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp),\
                           /*PCx*/ INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp),\
                           /*PDx*/ INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp),\
                           /*PEx*/ INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp), INPUT(PushPull, High, PullUp),\
                          }

#define INCLUDE_I2C
#define STM32F4_I2C_SCL_PINS { { PIN(B, 6), AF(4) } }
#define STM32F4_I2C_SDA_PINS { { PIN(B, 7), AF(4) } }

#define INCLUDE_PWM
#define STM32F4_PWM_PINS {/*          0                          1                        2                        3                       */\
                          /* TIM1  */ { { PIN(E,  9), AF(1)   }, { PIN(E, 11), AF(1)   }, { PIN(E, 13), AF(1)   }, { PIN(E, 14), AF(1)   } },\
                          /* TIM2  */ { { PIN(A, 15), AF(1)   }, { PIN(B,  3), AF(1)   }, { PIN(B, 10), AF(1)   }, { PIN(B, 11), AF(1)   } },\
                          /* TIM3  */ { { PIN(B,  4), AF(2)   }, { PIN(B,  5), AF(2)   }, { PIN(B,  0), AF(2)   }, { PIN(B,  1), AF(2)   } },\
                          /* TIM4  */ { { PIN(D, 12), AF(2)   }, { PIN(D, 13), AF(2)   }, { PIN(D, 14), AF(2)   }, { PIN(D, 15), AF(2)   } },\
                          /* TIM5  */ { { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM6  */ { { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM7  */ { { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM8  */ { { PIN(C,  6), AF(3)   }, { PIN(C,  7), AF(3)   }, { PIN(C,  8), AF(3)   }, { PIN(C,  9), AF(3)   } },\
                          /* TIM9  */ { { PIN(A,  2), AF(3)   }, { PIN(A,  3), AF(3)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM10 */ { { PIN(B,  8), AF(3)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM11 */ { { PIN(B,  9), AF(3)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM12 */ { { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM13 */ { { PIN(A,  6), AF(9)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                          /* TIM14 */ { { PIN(A,  7), AF(9)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                         }

#define INCLUDE_SPI
#define STM32F4_SPI_SCLK_PINS { { PIN(B, 3), AF(5) }, { PIN(B, 10), AF(5) } }
#define STM32F4_SPI_MISO_PINS { { PIN(B, 4), AF(5) }, { PIN(C,  2), AF(5) } }
#define STM32F4_SPI_MOSI_PINS { { PIN(B, 5), AF(5) }, { PIN(C,  3), AF(5) } }

#define INCLUDE_UART
#define STM32F4_UART_TX_BUFFER_SIZE 256
#define STM32F4_UART_RX_BUFFER_SIZE 512
#define STM32F4_UART_TX_PINS  { { PIN(A,  9), AF(7)   }, { PIN(D, 5), AF(7) }, { PIN(D,  8), AF(7) }, { PIN(A, 0), AF(8)   } }
#define STM32F4_UART_RX_PINS  { { PIN(A, 10), AF(7)   }, { PIN(D, 6), AF(7) }, { PIN(D,  9), AF(7) }, { PIN(A, 1), AF(8)   } }
#define STM32F4_UART_CTS_PINS { { PIN_NONE  , AF_NONE }, { PIN(D, 3), AF(7) }, { PIN(D, 11), AF(7) }, { PIN_NONE , AF_NONE } }
#define STM32F4_UART_RTS_PINS { { PIN_NONE  , AF_NONE }, { PIN(D, 4), AF(7) }, { PIN(D, 12), AF(7) }, { PIN_NONE , AF_NONE } }

#define INCLUDE_USBCLIENT
#define STM32F4_USB_QUEUE_SIZE 16
#define STM32F4_USB_DM_PINS { { PIN(A, 11), AF(10) } }
#define STM32F4_USB_DP_PINS { { PIN(A, 12), AF(10) } }
#define STM32F4_USB_VB_PINS { { PIN(A,  9), AF(10) } }
#define STM32F4_USB_ID_PINS { { PIN(A, 10), AF(10) } }