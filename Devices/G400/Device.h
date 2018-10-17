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

#define DEVICE_TARGET AT91SAM9X35
#define DEVICE_NAME "G400"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((1ULL << 48) | (0ULL << 32) | (0ULL << 16) | (10001ULL << 0))
#define DEVICE_MEMORY_PROFILE_FACTOR 9

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x5008

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(A, 25)
#define DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::High

#define RUN_APP_PIN PIN(A, 4)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define RAM_BOOTLOADER_HOLD_ADDRESS 0x266FFFF0
#define RAM_BOOTLOADER_HOLD_VALUE 0x8DB4DA67

#define AT91SAM9X35_AHB_CLOCK_HZ (400*1000*1000) // 400 MHz
#define AT91SAM9X35_SYSTEM_PERIPHERAL_CLOCK_HZ (AT91SAM9X35_AHB_CLOCK_HZ / 3) // 133MHz (Peripheral Clock - MCK)

#define INCLUDE_GPIO
#define AT91SAM9X35_GPIO_PINS  {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15         16         17         18         19         20         21         22         23         24         25         26         27         28         29         30         31      */\
                         /*PAx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), NO_INIT(), NO_INIT(), NO_INIT(), NO_INIT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*PBx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*PCx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*PDx*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         }


#define INCLUDE_ADC
#define AT91SAM9X35_ADC_PINS { { PIN(B,11), PS(D) }, { PIN(B,12), PS(D) }, { PIN(B,13), PS(D) }, { PIN(B,14), PS(D) }, { PIN(B,15), PS(D) }, { PIN(B,16), PS(D) }, { PIN(A,17), PS(D) }, { PIN(B,6), PS(D) }, { PIN(B,7), PS(D) }, { PIN(B,8), PS(D) }, { PIN(B,9), PS(D) }, { PIN(B,10), PS(D) } }

#define INCLUDE_CAN
#define TOTAL_CAN_CONTROLLERS 2
#define AT91SAM9X35_CAN_BUFFER_DEFAULT_SIZE { 128, 128 }
#define AT91SAM9X35_CAN_PINS {/*        TX                     RX          */           \
                       /*CAN0*/{ { PIN(A, 10), PS(B) }, { PIN(A,  9), PS(B) }, },\
                       /*CAN1*/{ { PIN(A, 6) , PS(B) }, { PIN(A, 5) , PS(B) }  } \
                       }

#define AT91SAM9X35_DEPLOYMENT_SECTOR_START 640
#define AT91SAM9X35_DEPLOYMENT_SECTOR_END 1020
#define AT91SAM9X35_DEPLOYMENT_SECTOR_NUM (AT91SAM9X35_DEPLOYMENT_SECTOR_END - AT91SAM9X35_DEPLOYMENT_SECTOR_START + 1)
#define AT91SAM9X35_DEPLOYMENT_SPI_PORT 0
#define AT91SAM9X35_DEPLOYMENT_SPI_ENABLE_PIN PIN(A,14)

#define INCLUDE_I2C
#define TOTAL_I2C_CONTROLLERS 1
#define AT91SAM9X35_I2C_PINS {/*          SDA                   SCL*/                \
                       /*I2C0*/{  { PIN(A,30), PS(A) }, { PIN(A,31), PS(A) } }\
                      }

#define INCLUDE_POWER

#define INCLUDE_PWM
#define MAX_PWM_PER_CONTROLLER 1
#define TOTAL_PWM_CONTROLLERS 4
#define AT91SAM9X35_PWM_PINS { { { PIN(C,18), PS(C) } }, { { PIN(C,19), PS(C) } }, { { PIN(C,20), PS(C) } }, { { PIN(C,21), PS(C) } } }

#define INCLUDE_RTC

#define INCLUDE_SD
#define AT91SAM9X35_SD_PINS {/*            DATA 0                   DATA 1                 DATA 2                  DATA 3                  CLK                    CMD*/                 \
                      /*SDCARD0*/{ { PIN(A, 15), PS(A) },   { PIN(A, 18), PS(A) }, { PIN(A, 19), PS(A) },  { PIN(A, 20), PS(A) },  { PIN(A, 17), PS(A) }, { PIN(A, 16), PS(A) } }\
                     }

#define INCLUDE_SIGNALS

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 2
#define AT91SAM9X35_SPI_PINS {/*           MOSI                   MISO                   CLOCK*/              \
                       /*SPI0*/{ { PIN(A,11), PS(A) },  { PIN(A,12), PS(A) },  { PIN(A,13), PS(A) } },\
                       /*SPI1*/{ { PIN(A,21), PS(B) },  { PIN(A,22), PS(B) },  { PIN(A,23), PS(B) } } \
                      }

#define AT91SAM9X35_TIME_DEFAULT_CONTROLLER_ID 0

#define INCLUDE_STORAGE

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 6
#define AT91SAM9X35_UART_DEFAULT_TX_BUFFER_SIZE  { 16*1024, 16*1024, 16*1024, 16*1024, 16*1024, 16*1024 }
#define AT91SAM9X35_UART_DEFAULT_RX_BUFFER_SIZE  { 16*1024, 16*1024, 16*1024, 16*1024, 16*1024, 16*1024 }

#define AT91SAM9X35_UART_PINS {/*          TX                      RX                     RTS                     CTS*/                  \
                        /*UART0*/{ { PIN(A, 9), PS(A) } , { PIN(A,10), PS(A) } , { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } },\
                        /*UART1*/{ { PIN(A, 0), PS(A) } , { PIN(A, 1), PS(A) } , { PIN(A, 2), PS(A)   }, { PIN(A, 3), PS(A)   } },\
                        /*UART2*/{ { PIN(A, 5), PS(A) } , { PIN(A, 6), PS(A) } , { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } },\
                        /*UART3*/{ { PIN(A, 7), PS(A) } , { PIN(A, 8), PS(A) } , { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } },\
                        /*UART4*/{ { PIN(C, 8), PS(C) } , { PIN(C, 9), PS(C) } , { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } },\
                        /*UART5*/{ { PIN(C,16), PS(C) } , { PIN(C,17), PS(C) } , { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } } \
                       }

#define INCLUDE_USBCLIENT
#define AT91SAM9X35_TOTAL_USB_CONTROLLERS 1
#define AT91SAM9X35_USB_PACKET_FIFO_COUNT 64
#define AT91SAM9X35_USB_ENDPOINT_SIZE 64
#define AT91SAM9X35_USB_ENDPOINT0_SIZE 64
#define AT91SAM9X35_USB_ENDPOINT_COUNT 16
#define AT91SAM9X35_USB_PIPE_COUNT 16

#define INCLUDE_DISPLAY
#define AT91SAM9X35_DISPLAY_CONTROLLER_RED_PINS   { { PIN(C,0), PS(A) }, { PIN(C,1), PS(A) }, { PIN(C,2), PS(A) }, { PIN(C,3), PS(A) }, { PIN(C,4), PS(A) } }
#define AT91SAM9X35_DISPLAY_CONTROLLER_GREEN_PINS { { PIN(C,5), PS(A) }, { PIN(C,6), PS(A) }, { PIN(C,7), PS(A) }, { PIN(C,8), PS(A) }, { PIN(C,9), PS(A) }, { PIN(C,10), PS(A) } }
#define AT91SAM9X35_DISPLAY_CONTROLLER_BLUE_PINS  { { PIN(C,11), PS(A) }, { PIN(C,12), PS(A) }, { PIN(C,13), PS(A) }, { PIN(C,14), PS(A) }, { PIN(C,15), PS(A) } }
#define AT91SAM9X35_DISPLAY_CONTROLLER_HSYNC_PIN { PIN(C,28), PS(A) }
#define AT91SAM9X35_DISPLAY_CONTROLLER_VSYNC_PIN { PIN(C,27), PS(A) }
#define AT91SAM9X35_DISPLAY_CONTROLLER_CLOCK_PIN { PIN(C,30), PS(A) }
#define AT91SAM9X35_DISPLAY_CONTROLLER_DATA_ENABLE_PIN { PIN(C, 29), PS(A) }

#include <AT91SAM9X35.h>
