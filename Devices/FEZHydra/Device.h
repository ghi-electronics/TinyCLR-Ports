// Copyright GHI Electronics, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#define DEVICE_TARGET AT91SAM9Rx64
#define DEVICE_NAME "FEZHydra"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((1ULL << 48) | (0ULL << 32) | (0ULL << 16) | (10001ULL << 0))
#define DEVICE_MEMORY_PROFILE_FACTOR 9

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x5005

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(B, 9)
#define DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::High

#define RUN_APP_PIN PIN(B, 12)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define RAM_BOOTLOADER_HOLD_ADDRESS 0x266FFFF0
#define RAM_BOOTLOADER_HOLD_VALUE 0xCAC6839D

#define AT91SAM9Rx64_AHB_CLOCK_HZ (200*1000*1000) // 200 MHz
#define AT91SAM9Rx64_SYSTEM_PERIPHERAL_CLOCK_HZ (AT91SAM9Rx64_AHB_CLOCK_HZ / 2) // 100MHz (Peripheral Clock - MCK)

#define INCLUDE_GPIO
#define AT91SAM9Rx64_GPIO_PINS  {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15         16         17         18         19         20         21         22         23         24         25         26         27         28         29         30         31      */\
                         /*PAx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                         /*PBx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                         /*PCx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                         /*PDx*/ NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(),\
                         }


#define INCLUDE_ADC
#define AT91SAM9Rx64_ADC_PINS { { PIN(A, 17), PS(A) }, { PIN(A, 18), PS(A) }, { PIN(A, 19), PS(A) }, { PIN(A, 20), PS(A) }, { PIN(D ,6), PS(A) }, { PIN(D, 7), PS(A) } }

#define INCLUDE_DEPLOYMENT

#define AT91SAM9Rx64_DEPLOYMENT_SECTOR_START 640
#define AT91SAM9Rx64_DEPLOYMENT_SECTOR_END 1020
#define AT91SAM9Rx64_DEPLOYMENT_SECTOR_NUM (AT91SAM9Rx64_DEPLOYMENT_SECTOR_END - AT91SAM9Rx64_DEPLOYMENT_SECTOR_START + 1)
#define AT91SAM9Rx64_DEPLOYMENT_SPI_PORT 0
#define AT91SAM9Rx64_DEPLOYMENT_SPI_ENABLE_PIN PIN(A,28)

#define INCLUDE_I2C
#define TOTAL_I2C_CONTROLLERS 1
#define AT91SAM9Rx64_I2C_PINS {/*           SDA                   SCL*/               \
                        /*I2C0*/{ { PIN(A,23), PS(A) }, { PIN(A,24), PS(A) } }\
                      }

#define INCLUDE_POWER

#define INCLUDE_PWM
#define MAX_PWM_PER_CONTROLLER 1
#define TOTAL_PWM_CONTROLLERS 3
#define AT91SAM9Rx64_PWM_PINS { { { PIN(D,14), PS(B) } }, { { PIN(D,15), PS(B) } }, { { PIN(D,16), PS(B) } } }

#define INCLUDE_RTC

#define INCLUDE_SIGNALS

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 1
#define AT91SAM9Rx64_SPI_MISO_PINS { { PIN(A,25), PS(A) } }
#define AT91SAM9Rx64_SPI_MOSI_PINS { { PIN(A,26), PS(A) } }
#define AT91SAM9Rx64_SPI_SCLK_PINS { { PIN(A,27), PS(A) } }
#define AT91SAM9Rx64_SPI_PINS {/*           MOSI                   MISO                   CLOCK*/              \
                       /*SPI0*/ { { PIN(A,25), PS(A) },  { PIN(A,26), PS(A) },  { PIN(A, 27), PS(A) } }\
                      }

#define AT91SAM9Rx64_TIME_DEFAULT_CONTROLLER_ID 0

#define INCLUDE_STORAGE

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 4
#define AT91SAM9Rx64_UART_DEFAULT_TX_BUFFER_SIZE  { 16*1024, 16*1024, 16*1024, 16*1024 }
#define AT91SAM9Rx64_UART_DEFAULT_RX_BUFFER_SIZE  { 16*1024, 16*1024, 16*1024, 16*1024 }
#define AT91SAM9Rx64_UART_PINS {/*            TX                     RX                   RTS                     CTS*/                  \
                        /*UART0*/{  { PIN(A,22), PS(A) }, { PIN(A,21), PS(A) }, { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } },\
                        /*UART1*/{  { PIN(A, 6), PS(A) }, { PIN(A, 7), PS(A) }, { PIN(A, 9), PS(A)   }, { PIN(A, 10), PS(A)  } },\
                        /*UART2*/{  { PIN(A, 11), PS(A)}, { PIN(A, 12), PS(A)}, { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } },\
                        /*UART3*/{  { PIN(A, 13), PS(A)}, { PIN(A, 14), PS(A)}, { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } } \
                       }

#define INCLUDE_USBCLIENT
#define AT91SAM9Rx64_TOTAL_USB_CONTROLLERS 1
#define AT91SAM9Rx64_USB_PACKET_FIFO_COUNT 64
#define AT91SAM9Rx64_USB_ENDPOINT_SIZE 64
#define AT91SAM9Rx64_USB_ENDPOINT0_SIZE 64
#define AT91SAM9Rx64_USB_ENDPOINT_COUNT 16
#define AT91SAM9Rx64_USB_PIPE_COUNT 16

#include <AT91SAM9Rx64.h>
