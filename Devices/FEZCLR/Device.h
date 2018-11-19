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

#define STM32F401xE 1

#include <STM32F4.h>

#define DEVICE_TARGET STM32F4
#define DEVICE_NAME "FEZCLR"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((1ULL << 48) | (0ULL << 32) | (0ULL << 16) | (10001ULL << 0))
#define DEVICE_MEMORY_PROFILE_FACTOR 5

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x500B

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(B, 10)
#define DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::High

#define RUN_APP_PIN PIN(C, 13)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define BOOTLOADER_HOLD_ADDRESS 0x20017FF8
#define BOOTLOADER_HOLD_VALUE 0x82469E42

#define DEPLOYMENT_SECTORS { { 0x06, 0x08040000, 0x00020000 }, { 0x07, 0x08060000, 0x00020000 } }

#define STM32F4_SYSTEM_CLOCK_HZ 84000000
#define STM32F4_AHB_CLOCK_HZ 84000000
#define STM32F4_APB1_CLOCK_HZ 42000000
#define STM32F4_APB2_CLOCK_HZ 84000000
#define STM32F4_EXT_CRYSTAL_CLOCK_HZ 8000000
#define STM32F4_SUPPLY_VOLTAGE_MV 3300

#define INCLUDE_ADC

#define INCLUDE_DEPLOYMENT

#define INCLUDE_GPIO
#define STM32F4_GPIO_PINS {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15      */\
                           /*PAx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                           /*PBx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                           /*PCx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                           /*PDx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT()\
                          }

#define INCLUDE_I2C
#define TOTAL_I2C_CONTROLLERS 1
#define STM32F4_I2C_PINS {/*            SDA                   SCL*/               \
                          /*I2C0*/  { { PIN(B, 7), AF(4) }, { PIN(B, 6), AF(4) } }\
                         }

#define INCLUDE_POWER

#define INCLUDE_PWM
#define TOTAL_PWM_CONTROLLERS 4
#define STM32F4_PWM_PINS {/*          0                          1                        2                        3                       */\
                          /* TIM1  */ { { PIN(A,  8), AF(1)   }, { PIN(A,  9), AF(1)   }, { PIN(A, 10), AF(1)   }, { PIN_NONE ,  AF_NONE } },\
                          /* TIM2  */ { { PIN(A,  0), AF(1)   }, { PIN(A,  1), AF(1)   }, { PIN(A,  2), AF(1)   }, { PIN(A,  3), AF(1)   } },\
                          /* TIM3  */ { { PIN(C,  6), AF(2)   }, { PIN(B,  5), AF(2)   }, { PIN(C,  8), AF(2)   }, { PIN(C,  9), AF(2)   } },\
                          /* TIM4  */ { { PIN(B,  6), AF(2)   }, { PIN(B,  7), AF(2)   }, { PIN(B,  8), AF(2)   }, { PIN(B,  9), AF(2)   } },\
                         }

#define INCLUDE_RTC

#define INCLUDE_SIGNALS

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 2
#define STM32F4_SPI_PINS {/*         MOSI                    MISO                   CLOCK*/                \
                          /*SPI0*/{ { PIN(B,  5), AF(5) },  { PIN(B,  4), AF(5) }, { PIN(B, 3) , AF(5) } },\
                          /*SPI1*/{ { PIN(B, 15), AF(5) },  { PIN(B, 14), AF(5) }, { PIN(B, 13), AF(5) } } \
                         }

#define INCLUDE_STORAGE

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 2
#define STM32F4_UART_DEFAULT_TX_BUFFER_SIZE  { 256, 256 }
#define STM32F4_UART_DEFAULT_RX_BUFFER_SIZE  { 512, 512 }
#define STM32F4_UART_PINS {/*           TX                       RX                      RTS                       CTS*/                   \
                           /*UART0*/{ { PIN(A,  9), AF(7)   }, { PIN(A, 10), AF(7)   }, { PIN_NONE  , AF_NONE }, { PIN_NONE  , AF_NONE } },\
                           /*UART1*/{ { PIN(A, 2), AF(7)    }, { PIN(A, 3), AF(7)    }, { PIN(A, 1), AF(7)    }, { PIN(A, 0), AF(7)    } } \
                           }

#define INCLUDE_USBCLIENT
#define STM32F4_TOTAL_USB_CONTROLLERS 1
#define STM32F4_USB_PACKET_FIFO_COUNT 16
#define STM32F4_USB_ENDPOINT_SIZE 64
#define STM32F4_USB_ENDPOINT0_SIZE 8
#define STM32F4_USB_ENDPOINT_COUNT 4
#define STM32F4_USB_PIPE_COUNT 4


#define STM32F4_USB_PINS {/*           DM                      DP                      VB                      ID*/                  \
                          /*USBC0*/{ { PIN(A, 11), AF(10) }, { PIN(A, 12), AF(10) }, { PIN(A,  9), AF(10) }, { PIN(A, 10), AF(10) } }\
                         }
