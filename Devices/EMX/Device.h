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

#define DEVICE_TARGET LPC24
#define DEVICE_NAME "EMX"
#define DEVICE_MANUFACTURER "GHI Electronics, LLC"
#define DEVICE_VERSION ((0ULL << 48) | (9ULL << 32) | (0ULL << 16) | (0ULL << 0))

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x5004

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define DEBUGGER_SELECTOR_PIN PIN(3, 23)
#define DEBUGGER_SELECTOR_PULL TinyCLR_Gpio_PinDriveMode::InputPullDown
#define DEBUGGER_SELECTOR_USB_STATE TinyCLR_Gpio_PinValue::Low

#define RUN_APP_PIN PIN(0, 4)
#define RUN_APP_PULL TinyCLR_Gpio_PinDriveMode::InputPullUp
#define RUN_APP_STATE TinyCLR_Gpio_PinValue::High

#define RAM_BOOTLOADER_HOLD_ADDRESS 0xA0FFFFF8
#define RAM_BOOTLOADER_HOLD_VALUE 0xB53D0238

#define SYSTEM_CLOCK_HZ 18000000
#define LPC24_AHB_CLOCK_HZ 72000000

#define INCLUDE_GPIO
#define LPC24_GPIO_PINS {/*      0          1          2          3          4          5          6          7          8          9          10         11         12         13         14         15         16         17         18         19         20         21         22         23         24         25         26         27         28         29         30         31      */\
                         /*P0x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P1x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P2x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P3x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                         /*P4x*/ DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(), DEFAULT(),\
                        }

#define INCLUDE_ADC
#define LPC24_ADC_PINS { { PIN(0,23), PF(1) }, { PIN(0,24), PF(1) }, { PIN(0,25), PF(1) }, { PIN(0,26), PF(1) }, { PIN_NONE, PF_NONE }, { PIN(1,31), PF(3) }, { PIN(0,12), PF(3) }, { PIN(0,13), PF(3) } }

#define INCLUDE_CAN
#define LPC24_CAN_BUFFER_DEFAULT_SIZE { 128, 128 }
#define LPC24_CAN_TX_PINS { { PIN(0, 1), PF(1) }, { PIN(0, 5), PF(2) } }
#define LPC24_CAN_RX_PINS { { PIN(0, 0), PF(1) }, { PIN(0, 4), PF(2) } }

#define LPC24_DEPLOYMENT_SECTOR_START 3
#define LPC24_DEPLOYMENT_SECTOR_END 12
#define LPC24_DEPLOYMENT_SECTOR_NUM (LPC24_DEPLOYMENT_SECTOR_END - LPC24_DEPLOYMENT_SECTOR_START + 1)

#define INCLUDE_DAC
#define LPC24_DAC_PINS { { PIN(0,26), PF(2)} }

#define INCLUDE_I2C
#define LPC24_I2C_SCL_PINS { { PIN(0, 28), PF(1) } }
#define LPC24_I2C_SDA_PINS { { PIN(0, 27), PF(1) } }

#define INCLUDE_PWM
#define TOTAL_PWM_CONTROLLER 2
#define MAX_PWM_PER_CONTROLLER 6
#define LPC24_PWM_PINS  { { { PIN(3, 16), PF(2) }, { PIN(3, 17), PF(2) }, { PIN_NONE, PF_NONE }, { PIN_NONE, PF_NONE }, { PIN_NONE, PF_NONE }, { PIN_NONE  , PF_NONE } }, { { PIN(3, 24), PF(3) }, { PIN_NONE  , PF_NONE }, { PIN(3, 26), PF(3) }, { PIN(3, 27), PF(3) }, { PIN_NONE, PF_NONE }, { PIN(3, 29), PF(3) } } }

#define INCLUDE_RTC

#define INCLUDE_SPI
#define TOTAL_SPI_CONTROLLERS 2
#define LPC24_SPI_SCLK_PINS { { PIN(0, 15), PF(2) }, { PIN(0,  7), PF(2) } }
#define LPC24_SPI_MISO_PINS { { PIN(0, 17), PF(2) }, { PIN(0,  8), PF(2) } }
#define LPC24_SPI_MOSI_PINS { { PIN(0, 18), PF(2) }, { PIN(0,  9), PF(2) } }

#define INCLUDE_UART
#define TOTAL_UART_CONTROLLERS 4
#define LPC24_UART_DEFAULT_TX_BUFFER_SIZE { 16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024 }
#define LPC24_UART_DEFAULT_RX_BUFFER_SIZE { 16 * 1024, 16 * 1024, 16 * 1024, 16 * 1024 }
#define LPC24_UART_TX_PINS              { { PIN(0, 2), PF(1) }, { PIN(2, 0) , PF(2) }, { PIN(4, 22), PF(2) }, { PIN(0, 25), PF(3) } }
#define LPC24_UART_RX_PINS              { { PIN(0, 3), PF(1) }, { PIN(2, 1) , PF(2) }, { PIN(4, 23), PF(2) }, { PIN(0, 26), PF(3) } }
#define LPC24_UART_RTS_PINS             { { PIN_NONE , PF_NONE }, { PIN(3, 30), PF(3) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } }
#define LPC24_UART_CTS_PINS             { { PIN_NONE , PF_NONE }, { PIN(3, 18), PF(3) }, { PIN_NONE  , PF_NONE }, { PIN_NONE  , PF_NONE } }

#define INCLUDE_USBCLIENT
#define TOTAL_USB_CONTROLLER            1
#define LPC24_USB_QUEUE_SIZE 16
#define LPC24_USB_FIFO_BUFFER_SIZE 64

#define INCLUDE_DISPLAY
#define LPC24_DISPLAY_CONTROLLER_PINS { { PIN(1, 20), PF(1) }, { PIN(1, 21), PF(1) }, { PIN(1, 22), PF(1) }, { PIN(1, 23), PF(1) }, { PIN(1, 24), PF(1) }, { PIN(1, 25), PF(1) }, { PIN(1, 26), PF(1) }, { PIN(1, 27), PF(1) }, { PIN(1, 28), PF(1)}, { PIN(1, 29), PF(1) }, { PIN(2, 2), PF(3) }, { PIN(2, 3), PF(3) }, { PIN(2, 5), PF(3) }, { PIN(2, 6), PF(3) }, { PIN(2, 7), PF(3) }, { PIN(2, 8), PF(3) }, { PIN(2, 9), PF(3) }, { PIN(2, 12), PF(1) }, { PIN(2, 13), PF(1) } }
#define LPC24_DISPLAY_BACKLIGHT_PIN  { PIN_NONE, PF_NONE }
#define LPC24_DISPLAY_ENABLE_PIN  { PIN(2, 4), PF(3) }

#include <LPC24.h>