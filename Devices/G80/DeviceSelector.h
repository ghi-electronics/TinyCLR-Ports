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
#define DEVICE_VERSION 0x000600

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

#define USB_DEBUGGER_VENDOR_ID 0x1B9F
#define USB_DEBUGGER_PRODUCT_ID 0x0110

#define MODE_PIN MAKE_PIN(E, 15)
#define MODE_USB_STATE TinyCLR_Gpio_PinValue::High

#define RUNAPP_PIN MAKE_PIN(E, 4)
#define RUNAPP_STATE TinyCLR_Gpio_PinValue::High

#define RAM_BOOTLOADER_HOLD_ADDRESS 0x2002FFF8
#define RAM_BOOTLOADER_HOLD_VALUE 0x37D56D4A

#define INCLUDE_I2C
#define STM32F4_I2C_SCL_PINS { { MAKE_PIN(B, 6), STM32F4_Gpio_AlternateFunction::AF4 } }
#define STM32F4_I2C_SDA_PINS { { MAKE_PIN(B, 7), STM32F4_Gpio_AlternateFunction::AF4 } }

#define INCLUDE_SPI
#define STM32F4_SPI_SCLK_PINS { { MAKE_PIN(B, 3), STM32F4_Gpio_AlternateFunction::AF5 }, { MAKE_PIN(B, 10), STM32F4_Gpio_AlternateFunction::AF5 } }
#define STM32F4_SPI_MISO_PINS { { MAKE_PIN(B, 4), STM32F4_Gpio_AlternateFunction::AF5 }, { MAKE_PIN(C, 2), STM32F4_Gpio_AlternateFunction::AF5 } }
#define STM32F4_SPI_MOSI_PINS { { MAKE_PIN(B, 5), STM32F4_Gpio_AlternateFunction::AF5 }, { MAKE_PIN(C, 3), STM32F4_Gpio_AlternateFunction::AF5 } }

#define INCLUDE_PWM
#define STM32F4_PWM_PINS {\
                             { { MAKE_PIN(E,  9), STM32F4_Gpio_AlternateFunction::AF1 }, { MAKE_PIN(E, 11), STM32F4_Gpio_AlternateFunction::AF1 }, { MAKE_PIN(E, 13), STM32F4_Gpio_AlternateFunction::AF1 }, { MAKE_PIN(E, 14), STM32F4_Gpio_AlternateFunction::AF1 } },\
                             { { MAKE_PIN(A, 15), STM32F4_Gpio_AlternateFunction::AF1 }, { MAKE_PIN(B,  3), STM32F4_Gpio_AlternateFunction::AF1 }, { MAKE_PIN(B, 10), STM32F4_Gpio_AlternateFunction::AF1 }, { MAKE_PIN(B, 11), STM32F4_Gpio_AlternateFunction::AF1 } },\
                             { { MAKE_PIN(B,  4), STM32F4_Gpio_AlternateFunction::AF2 }, { MAKE_PIN(B,  5), STM32F4_Gpio_AlternateFunction::AF2 }, { MAKE_PIN(B,  0), STM32F4_Gpio_AlternateFunction::AF2 }, { MAKE_PIN(B,  1), STM32F4_Gpio_AlternateFunction::AF2 } },\
                             { { MAKE_PIN(D, 12), STM32F4_Gpio_AlternateFunction::AF2 }, { MAKE_PIN(D, 13), STM32F4_Gpio_AlternateFunction::AF2 }, { MAKE_PIN(D, 14), STM32F4_Gpio_AlternateFunction::AF2 }, { MAKE_PIN(D, 15), STM32F4_Gpio_AlternateFunction::AF2 } },\
                             { { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             } },\
                             { { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             } },\
                             { { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             } },\
                             { { MAKE_PIN(C,  6), STM32F4_Gpio_AlternateFunction::AF3 }, { MAKE_PIN(C,  7), STM32F4_Gpio_AlternateFunction::AF3 }, { MAKE_PIN(C,  8), STM32F4_Gpio_AlternateFunction::AF3 }, { MAKE_PIN(C,  9), STM32F4_Gpio_AlternateFunction::AF3 } },\
                             { { MAKE_PIN(A,  2), STM32F4_Gpio_AlternateFunction::AF3 }, { MAKE_PIN(A,  3), STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 } },\
                             { { MAKE_PIN(B,  8), STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 } },\
                             { { MAKE_PIN(B,  9), STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF3 } },\
                             { { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             }, { PIN_NONE       , AF_NONE                             } },\
                             { { MAKE_PIN(A,  6), STM32F4_Gpio_AlternateFunction::AF9 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF9 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF9 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF9 } },\
                             { { MAKE_PIN(A,  7), STM32F4_Gpio_AlternateFunction::AF9 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF9 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF9 }, { PIN_NONE       , STM32F4_Gpio_AlternateFunction::AF9 } }\
                         }

#define INCLUDE_UART
#define STM32F4_UART_TX_BUFFER_SIZE 256
#define STM32F4_UART_RX_BUFFER_SIZE 512
#define STM32F4_UART_TX_PINS { { MAKE_PIN(A, 9), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(D, 5), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(D, 8), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(A, 0), STM32F4_Gpio_AlternateFunction::AF8 } }
#define STM32F4_UART_RX_PINS { { MAKE_PIN(A, 10), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(D, 6), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(D, 9), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(A, 1), STM32F4_Gpio_AlternateFunction::AF8 } }
#define STM32F4_UART_CTS_PINS { { PIN_NONE, AF_NONE }, { MAKE_PIN(D, 3), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(D, 11), STM32F4_Gpio_AlternateFunction::AF7 }, { PIN_NONE, AF_NONE } }
#define STM32F4_UART_RTS_PINS { { PIN_NONE, AF_NONE }, { MAKE_PIN(D, 4), STM32F4_Gpio_AlternateFunction::AF7 }, { MAKE_PIN(D, 12), STM32F4_Gpio_AlternateFunction::AF7 }, { PIN_NONE, AF_NONE } }




#define INCLUDE_ADC
#define INCLUDE_DAC
#define INCLUDE_GPIO
#define INCLUDE_USBCLIENT

// GPIO
#define TOTAL_GPIO_PORT                 (GPIO_PORT_E + 1)
#define TOTAL_GPIO_PINS                 (TOTAL_GPIO_PORT*16)

// ADC
#define STM32F4_ADC                     1
#define STM32F4_AD_CHANNELS             { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}

// USBC
#define TOTAL_USB_CONTROLLER            1
#define USB_MAX_QUEUES                  16


// System clock
#define SYSTEM_CLOCK_HZ                  180000000   // 180 MHz
#define SYSTEM_CYCLE_CLOCK_HZ            180000000   // 18 MHz
#define SYSTEM_APB1_CLOCK_HZ              45000000   //  45 MHz
#define SYSTEM_APB2_CLOCK_HZ              90000000   //  90 MHz
#define SYSTEM_CRYSTAL_CLOCK_HZ           12000000   // 12 MHz external clock
#define SUPPLY_VOLTAGE_MV                     3300   // 3.3V supply
#define CLOCK_COMMON_FACTOR                1000000   // GCD(SYSTEM_CLOCK_HZ, 1M)
#define SLOW_CLOCKS_PER_SECOND           180000000   // 1 MHz
#define SLOW_CLOCKS_TEN_MHZ_GCD            1000000   // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD           1000   // GCD(SLOW_CLOCKS_PER_SECOND, 1k)

// Memory
#define FLASH_MEMORY_Base                   0x08000000
#define FLASH_MEMORY_Size                   0x00100000  // 1 MB
#define SRAM1_MEMORY_Base                   0x20000000
#define SRAM1_MEMORY_Size                   0x00030000  // 192 KB
#define ENABLE_CCM_RAM                      1

#define FLASH_DEPLOYMENT_SECTOR_ADDRESS     {0x080C0000, 0x080E0000}
#define FLASH_DEPLOYMENT_SECTOR_SIZE        {0x00020000, 0x00020000}