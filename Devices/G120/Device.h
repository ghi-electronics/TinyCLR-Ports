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

#define LPC177x_8x 1

#define DEVICE_TARGET LPC17
#define DEVICE_NAME "G120"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((1ULL << 48) | (0ULL << 32) | (0ULL << 16) | (10001ULL << 0))
#define DEVICE_MEMORY_PROFILE_FACTOR 9

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x5007

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(2, 1)
#define G120_DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define G120_DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::High

#define G120E_DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullDown
#define G120E_DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::Low

#define RUN_APP_PIN PIN(0, 22)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define RAM_BOOTLOADER_HOLD_ADDRESS 0xA0FFFFF8
#define RAM_BOOTLOADER_HOLD_VALUE 0x37D56D4A

#define LPC17_SYSTEM_CLOCK_HZ 120000000
#define LPC17_AHB_CLOCK_HZ 120000000

#define INCLUDE_ADC
#define LPC17_ADC_PINS { { PIN(0, 23), PF(1) }, { PIN(0, 24), PF(1) }, { PIN(0, 25), PF(1) }, { PIN(0, 26), PF(1) }, { PIN(1, 30), PF(3) }, { PIN(1, 31) , PF(3) }, { PIN(0, 12), PF(3) }, { PIN(0, 13), PF(3) } }

#define INCLUDE_CAN
#define TOTAL_CAN_CONTROLLERS 2
#define LPC17_CAN_BUFFER_DEFAULT_SIZE { 128, 128 }
#define LPC17_CAN_PINS {/*          TX                     RX          */       \
                        /*CAN0*/{ { PIN(0, 1), PF(1) },  { PIN(0, 0), PF(1) } },\
                        /*CAN1*/{ { PIN(0, 5), PF(2) },  { PIN(0, 4), PF(2) } } \
                       }

#define INCLUDE_DAC
#define LPC17_DAC_PINS { { PIN(0, 26), PF(2) } }

#define LPC17_DEPLOYMENT_SECTOR_START 20
#define LPC17_DEPLOYMENT_SECTOR_END 57
#define LPC17_DEPLOYMENT_SECTOR_NUM (LPC17_DEPLOYMENT_SECTOR_END - LPC17_DEPLOYMENT_SECTOR_START + 1)

#define LPC17_DEPLOYMENT_SPI_PORT 1
#define LPC17_DEPLOYMENT_SPI_ENABLE_PIN PIN(4,27)

#define INCLUDE_GPIO
#define LPC17_GPIO_PINS {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15         16         17         18         19         20         21         22         23         24         25         26         27         28         29         30         31      */\
                         /*P0x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P1x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P2x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(),\
                         /*P3x*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P4x*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), DEFAULT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT()\
                        }

#define INCLUDE_I2C
#define TOTAL_I2C_CONTROLLERS 1
#define LPC17_I2C_PINS {/*          SDA                   SCL*/                 \
                        /*I2C0*/{ { PIN(0, 27), PF(1) }, { PIN(0, 28), PF(1) } }\
                       }

#define INCLUDE_POWER

#define INCLUDE_PWM
#define TOTAL_PWM_CONTROLLERS 2
#define MAX_PWM_PER_CONTROLLER 6
#define LPC17_G120_PWM_PINS  { { { PIN(1,  2), PF(3) }, { PIN(1,  3), PF(3) }, { PIN(1,  5), PF(3) }, { PIN(1,  6), PF(3) }, { PIN(1,  7), PF(3) }, { PIN(1, 11), PF(3) } }, { { PIN(3, 24), PF(2) }, { PIN(3, 25), PF(2) }, { PIN(3, 26), PF(2) }, { PIN(2,  3), PF(1) }, { PIN(2,  4), PF(1) }, { PIN(2,  5), PF(1) } } }
#define LPC17_G120E_PWM_PINS { { { PIN(3, 16), PF(2) }, { PIN(3, 17), PF(2) }, { PIN(3, 18), PF(2) }, { PIN(3, 19), PF(2) }, { PIN(3, 20), PF(2) }, { PIN(3, 21), PF(2) } }, { { PIN(3, 24), PF(2) }, { PIN(3, 25), PF(2) }, { PIN(3, 26), PF(2) }, { PIN(3, 27), PF(2) }, { PIN(3, 28), PF(2) }, { PIN(3, 29), PF(2) } } }

#define INCLUDE_RTC

#define INCLUDE_SD
#define LPC17_SD_PINS {/*             DATA 0                   DATA 1                  DATA 2                  DATA 3                  CLK                   CMD*/               \
                       /*SDCARD0*/{ { PIN(1, 6), PF(2) },    { PIN(1, 7), PF(2) },   { PIN(1, 11), PF(2) },  { PIN(1, 12), PF(2) } , { PIN(1, 2), PF(2) }, { PIN(1, 3), PF(2) } }\
                      }

#define INCLUDE_SIGNALS

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 3
#define LPC17_SPI_PINS {/*          MOSI                   MISO                   CLOCK*/               \
                        /*SPI0*/{ { PIN(0, 18), PF(2) }, { PIN(0, 17), PF(2) }, { PIN(0, 15), PF(2) } },\
                        /*SPI1*/{ { PIN(0,  9), PF(2) }, { PIN(0,  8), PF(2) }, { PIN(0,  7), PF(2) } },\
                        /*SPI2*/{ { PIN(1,  1), PF(4) }, { PIN(1,  4), PF(4) }, { PIN(1,  0), PF(4) } } \
                       }

#define INCLUDE_STORAGE

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 5
#define LPC17_UART_DEFAULT_TX_BUFFER_SIZE  { 16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024 }
#define LPC17_UART_DEFAULT_RX_BUFFER_SIZE  { 16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024 }

#define LPC17_G120_UART_PINS {/*           TX                       RX                     RTS                      CTS*/                   \
                              /*UART0*/{ { PIN(0,  2), PF(1)   }, { PIN(0,  3), PF(1) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } },\
                              /*UART1*/{ { PIN(2,  0), PF(2)   }, { PIN(0, 16), PF(1) }, { PIN(0, 17), PF(1)   }, { PIN(0,  6), PF(4)   } },\
                              /*UART2*/{ { PIN(0, 10), PF(1)   }, { PIN(0, 11), PF(1) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } },\
                              /*UART3*/{ { PIN(4, 29), PF(2)   }, { PIN(4, 28), PF(2) }, { PIN_NONE , PF_NONE  }, { PIN_NONE , PF_NONE  } },\
                              /*UART4*/{ { PIN(1, 29), PF(5)   }, { PIN(2,  9), PF(3) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } } \
                             } 

#define LPC17_G120E_UART_PINS {/*           TX                     RX                     RTS                      CTS*/                   \
                               /*UART0*/{ { PIN(0,  2), PF(1) }, { PIN(0,  3), PF(1) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } },\
                               /*UART1*/{ { PIN(2,  0), PF(2) }, { PIN(0, 16), PF(1) }, { PIN(0, 17), PF(1)   }, { PIN(0,  6), PF(4)   } },\
                               /*UART2*/{ { PIN(0, 10), PF(1) }, { PIN(0, 11), PF(1) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } },\
                               /*UART3*/{ { PIN(0, 25), PF(3) }, { PIN(0, 26), PF(3) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } },\
                               /*UART4*/{ { PIN(1, 29), PF(5) }, { PIN(2,  9), PF(3) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } } \
                               }

#define INCLUDE_USBCLIENT
#define LPC17_TOTAL_USB_CONTROLLERS 1
#define LPC17_USB_PACKET_FIFO_COUNT 64
#define LPC17_USB_ENDPOINT_SIZE 64
#define LPC17_USB_ENDPOINT0_SIZE 64
#define LPC17_USB_ENDPOINT_COUNT 16
#define LPC17_USB_PIPE_COUNT 16

#define INCLUDE_DISPLAY
#define LPC17_DISPLAY_CONTROLLER_PINS { { { PIN(1, 20), PF(7) }, { PIN(1, 21), PF(7) }, { PIN(1, 22), PF(7) }, { PIN(1, 23), PF(7) }, { PIN(1, 24), PF(7) }, { PIN(1, 25), PF(7) }, { PIN(1, 26), PF(7) }, { PIN(1, 27), PF(7) }, { PIN(1, 28), PF(7)}, { PIN(1, 29), PF(7) }, { PIN(2, 2), PF(7) }, { PIN(2, 3), PF(7) }, { PIN(2, 5), PF(7) }, { PIN(2, 6), PF(7) }, { PIN(2, 7), PF(7) }, { PIN(2, 8), PF(7) }, { PIN(2, 9), PF(7) }, { PIN(2, 12), PF(5) }, { PIN(2, 13), PF(7) } } }
#define LPC17_DISPLAY_ENABLE_PIN  { PIN(2, 4), PF(7) }

#include <LPC17.h>