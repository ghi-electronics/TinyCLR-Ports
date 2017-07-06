// Copyright 2017 GHI Electronics, LLC
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

#ifndef _DEVICE_NETDUINO_3_H_
#define _DEVICE_NETDUINO_3_H_

#define PLATFORM_ARM_DEFINED

// Macro
#define GLOBAL_LOCK(x)             STM32F4_SmartPtr_IRQ x
#define DISABLE_INTERRUPTS()       STM32F4_SmartPtr_IRQ::ForceDisabled()
#define ENABLE_INTERRUPTS()        STM32F4_SmartPtr_IRQ::ForceEnabled()
#define INTERRUPTS_ENABLED_STATE() STM32F4_SmartPtr_IRQ::GetState()
#define GLOBAL_LOCK_SOCKETS(x)     STM32F4_SmartPtr_IRQ x

#if defined(_DEBUG)
#define ASSERT(x)                  while (!x)
#define ASSERT_IRQ_MUST_BE_OFF()   ASSERT(!STM32F4_SmartPtr_IRQ::GetState())
#define ASSERT_IRQ_MUST_BE_ON()    ASSERT( STM32F4_SmartPtr_IRQ::GetState())
#else
#define ASSERT_IRQ_MUST_BE_OFF()
#define ASSERT_IRQ_MUST_BE_ON()
#endif

#define INTERRUPT_START         STM32F4_Interrupt_Started();
#define INTERRUPT_END           STM32F4_Interrupt_Ended();

// Device
#define HAL_SYSTEM_NAME "Netduino 3"

// System clock
#define SYSTEM_CLOCK_HZ                  180000000   // 180 MHz
#define SYSTEM_CYCLE_CLOCK_HZ            180000000   // 18 MHz
#define SYSTEM_APB1_CLOCK_HZ              45000000   //  45 MHz
#define SYSTEM_APB2_CLOCK_HZ              90000000   //  90 MHz
#define SYSTEM_CRYSTAL_CLOCK_HZ           25000000   // 25 MHz external clock
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

// Macro
#define PORT_PIN(port,pin)              (((int)port)*16 + (pin))
#define _P(port, pin)                   PORT_PIN(GPIO_PORT##port, pin)
#define _P_NONE_                        GPIO_PIN_NONE
#define GPIO_PORTA  0
#define GPIO_PORTB  1
#define GPIO_PORTC  2
#define GPIO_PORTD  3
#define GPIO_PORTE  4

// GPIO
#define TOTAL_GPIO_PORT                 (GPIO_PORTE + 1)
#define TOTAL_GPIO_PINS                 (TOTAL_GPIO_PORT*16)

// I2C
#define STM32F4_ADC                     1
#define STM32F4_AD_CHANNELS             { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}

// PWM
#define MAX_PWM_PER_CONTROLLER           4
#define TOTAL_PWM_CONTROLLER            14
#define STM32F4_PWM                     {   { TIM1  ,  0x12,     { _P(E, 9) , _P(E,11) , _P(E,13) , _P(E,14)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM2  ,  0x12,     { _P(A,15) , _P(B, 3) , _P(B,10) , _P(B,11)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM3  ,  0x22,     { _P(B, 4) , _P(B, 5) , _P(B, 0) , _P(B, 1)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM4  ,  0x22,     { _P(D,12) , _P(D,13) , _P(D,14) , _P(D,15)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { 0x00  ,  0x00,     { _P_NONE_ , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { 0x00  ,  0x00,     { _P_NONE_ , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { 0x00  ,  0x00,     { _P_NONE_ , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM8  ,  0x32,     { _P(C, 6) , _P(C, 7) , _P(C, 8) , _P(C, 9)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM9  ,  0x32,     { _P(A, 2) , _P(A, 3) , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM10 ,  0x32,     { _P(B, 8) , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM11 ,  0x92,     { _P(B, 9) , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { 0x00  ,  0x00,     { _P_NONE_ , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM13 ,  0x92,     { _P(A, 6) , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 },\
                                            { TIM14 ,  0x92,     { _P(A, 7) , _P_NONE_ , _P_NONE_ , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0 }}

// I2C
#define STM32F4_I2C_PORT                1
#define STM32F4_I2C_SCL_PIN             _P(B, 6)
#define STM32F4_I2C_SDA_PIN             _P(B, 7)

// SPI
#define TOTAL_SPI_CONTROLLERS           2
#define STM32F4_SPI_SCLK_PINS           { _P(B, 3), _P(B,10) }
#define STM32F4_SPI_MISO_PINS           { _P(B, 4), _P(C, 2) }
#define STM32F4_SPI_MOSI_PINS           { _P(B, 5), _P(C, 3) }

// UART
#define TOTAL_UART_CONTROLLERS          4
#define STM32F4_UART_TX_BUFFER_SIZE     256
#define STM32F4_UART_RX_BUFFER_SIZE     512
#define STM32F4_UART_RXD_PINS           { _P(A,10), _P(D, 6), _P(D, 9), _P(A, 1) }
#define STM32F4_UART_TXD_PINS           { _P(A, 9), _P(D, 5), _P(D, 8), _P(A, 0) }
#define STM32F4_UART_CTS_PINS           { GPIO_PIN_NONE, _P(D, 3), _P(D,11), GPIO_PIN_NONE }
#define STM32F4_UART_RTS_PINS           { GPIO_PIN_NONE, _P(D, 4), _P(D,12), GPIO_PIN_NONE }

//  USBC

#define TOTAL_USB_CONTROLLER            1
#define USB_MAX_QUEUES                  16
#define USB_VENDOR_ID                   0x1B9F
#define USB_PRODUCT_ID                  0x0110
#define USB_MANUFACTURER_NAME           {'N', 'e', 't', 'd', 'u', 'i', 'n', 'o', ' ', '3'}
#define USB_PRODUCT_NAME                {'N', 'e', 't', 'd', 'u', 'i', 'n', 'o', ' ', '3'}
#define USB_DISPLAY_NAME                USB_PRODUCT_NAME
#define USB_FRIENDLY_NAME               USB_PRODUCT_NAME


// Debug
#define DEBUG_TEXT_PORT                 USB1
#define STDIO                           USB1
#define DEBUGGER_PORT                   USB1
#define MESSAGING_PORT                  USB1

#define LMODE_PIN                       _P(D,0)
#define LMODE_USB_STATE                 TinyCLR_Gpio_PinValue::High

// Loader
#define RUNAPP_PIN                      _P(D,1)
#define RUNAPP_STATE                    TinyCLR_Gpio_PinValue::High

#define UART_DEBUGGER_INDEX 0
#define USB_DEBUGGER_INDEX 0

// OEM information
#define OEM_STRING                      "Netduino 3\0"
#define OEM_VERSION_MAJOR               0
#define OEM_VERSION_MINOR               5
#define OEM_VERSION_PATCH               0


// STM32F4 requires to define specific
#define STM32F4XX                        1
#define STM32F427X                         1

#define INCLUDE_ADC
#define INCLUDE_DAC
#define INCLUDE_GPIO
#define INCLUDE_I2C
#define INCLUDE_PWM
#define INCLUDE_SPI
#define INCLUDE_UART
#define INCLUDE_USBCLIENT

#define TARGET STM32F4

#include <stm32f4xx.h>
#include <STM32F4.h>

#endif