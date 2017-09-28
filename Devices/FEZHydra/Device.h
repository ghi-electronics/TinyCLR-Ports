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

#define DEVICE_TARGET AT91
#define DEVICE_NAME "FEZHydra"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((0ULL << 48) | (6ULL << 32) | (0ULL << 16) | (0ULL << 0))

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

#define RAM_BOOTLOADER_HOLD_ADDRESS 0x20FFFFF8
#define RAM_BOOTLOADER_HOLD_VALUE 0x9B3642AE

#define AT91_AHB_CLOCK_HZ (200*1000*1000) // 200 MHz
#define AT91_SYSTEM_PERIPHERAL_CLOCK_HZ (AT91_AHB_CLOCK_HZ / 2) // 100MHz (Peripheral Clock - MCK)

#define SRAM_MEMORY_BASE 0x00300000
#define SRAM_MEMORY_SIZE (32*1024)
#define SDRAM_MEMORY_BASE 0x20000000
#define SDRAM_MEMORY_SIZE (4*1024*1024)

#define FLASH_MEMORY_BASE 0x00000000
#define FLASH_MEMORY_SIZE 0x00420000

#define INCLUDE_GPIO
#define AT91_GPIO_PINS  {/*      0              1              2              3              4              5              6              7              8              9              10             11             12             13             14             15             16             17             18             19             20             21             22             23             24             25             26             27             28             29             30             31          */\
                         /*P0x*/ INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp),\
                         /*P1x*/ INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp),\
                         /*P2x*/ INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp),\
                         /*P3x*/ INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp), INPUT(PullUp),\
                         }


#define INCLUDE_ADC
#define AT91_ADC_PINS { { PIN(B,11), PS(D) }, { PIN(B,12), PS(D) }, { PIN(B,13), PS(D) }, { PIN(B,14), PS(D) }, { PIN(B,15), PS(D) }, { PIN(B,16), PS(D) }, { PIN(A,17), PS(D) }, { PIN(B,6), PS(D) }, { PIN(B,7), PS(D) }, { PIN(B,8), PS(D) }, { PIN(B,9), PS(D) }, { PIN(B,10), PS(D) } }

#define INCLUDE_I2C
#define AT91_I2C_SCL_PINS { { PIN(A,24), PS(A) } }
#define AT91_I2C_SDA_PINS { { PIN(A,23), PS(A) } }

#define INCLUDE_PWM
#define MAX_PWM_PER_CONTROLLER 1
#define TOTAL_PWM_CONTROLLER 4
#define AT91_PWM_PINS { { { PIN(C,18), PS(C) } }, { { PIN(C,19), PS(C) } }, { { PIN(C,20), PS(C) } }, { { PIN(C,21), PS(C) } } }

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 2
#define AT91_SPI_MISO_PINS { { PIN(A,11), PS(A) }, { PIN(A,21), PS(B) } }
#define AT91_SPI_MOSI_PINS { { PIN(A,12), PS(A) }, { PIN(A,22), PS(B) } }
#define AT91_SPI_SCLK_PINS { { PIN(A,13), PS(A) }, { PIN(A,13), PS(B) } }

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 4
#define AT91_UART_TX_BUFFER_SIZE (16*1024)
#define AT91_UART_RX_BUFFER_SIZE (16*1024)
#define AT91_UART_TX_PINS { { PIN(A,22), PS(A) } , { PIN(A, 6), PS(A) }, { PIN(A, 11), PS(A) } , { PIN(A, 13), PS(A) } }
#define AT91_UART_RX_PINS { { PIN(A,21), PS(A) } , { PIN(A, 7), PS(A) }, { PIN(A, 12), PS(A) } , { PIN(A, 14), PS(A) } }
#define AT91_UART_RTS_PINS { { PIN_NONE , PS_NONE }, { PIN(A, 9), PS(A) }, { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } }
#define AT91_UART_CTS_PINS { { PIN_NONE , PS_NONE }, { PIN(A, 10), PS(A) }, { PIN_NONE , PS_NONE }, { PIN_NONE , PS_NONE } }

#define INCLUDE_USBCLIENT
#define TOTAL_USB_CONTROLLER 1
#define AT91_USB_QUEUE_SIZE 16

#define INCLUDE_DISPLAY
#define AT91_DISPLAY_CONTROLLER_PINS { { PIN(C,0), PS(A) }, { PIN(C,1), PS(A) }, { PIN(C,2), PS(A) }, { PIN(C,3), PS(A) }, { PIN(C,4), PS(A) }, { PIN(C,5), PS(A) }, { PIN(C,6), PS(A) }, { PIN(C,7), PS(A) }, { PIN(C,8), PS(A) }, { PIN(C,9), PS(A) }, { PIN(C,10), PS(A) }, { PIN(C,11), PS(A) }, { PIN(C,12), PS(A) }, { PIN(C,13), PS(A) }, { PIN(C,14), PS(A) }, { PIN(C,15), PS(A) }, { PIN(C,24), PS(A) }, { PIN(C,26), PS(A) }, { PIN(C,27), PS(A) }, { PIN(C,28), PS(A) }, { PIN(C,30), PS(A) } }
#define AT91_DISPLAY_BACKLIGHT_PIN { PIN_NONE, PS_NONE }
#define AT91_DISPLAY_ENABLE_PIN { PIN(C, 29), PS(A) }

#include <AT91.h>
