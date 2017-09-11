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

#ifndef _DEVICE_NUCLEO_F411RE_H_
#define _DEVICE_NUCLEO_F411RE_H_

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
#define HAL_SYSTEM_NAME "NUCLEO-F411RE"

// System clock
#define SYSTEM_CLOCK_HZ                  96000000   // 96 MHz
#define SYSTEM_CYCLE_CLOCK_HZ            96000000   // 96 MHz
#define SYSTEM_APB1_CLOCK_HZ             48000000   // 96 MHz
#define SYSTEM_APB2_CLOCK_HZ             96000000   // 96 MHz
#define SYSTEM_CRYSTAL_CLOCK_HZ           8000000   //  8 MHz external clock
#define SUPPLY_VOLTAGE_MV                    3300   // 3.3V supply
#define CLOCK_COMMON_FACTOR               1000000   // GCD(SYSTEM_CLOCK_HZ, 1M)
#define SLOW_CLOCKS_PER_SECOND           96000000   // 96 MHz
#define SLOW_CLOCKS_TEN_MHZ_GCD           1000000   // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000   // GCD(SLOW_CLOCKS_PER_SECOND, 1k)

// Memory
#define FLASH_MEMORY_Base                   0x08000000
#define FLASH_MEMORY_Size                   0x00080000  // 512KB
#define SRAM1_MEMORY_Base                   0x20000000
#define SRAM1_MEMORY_Size                   0x00020000  // 128KB

#define FLASH_DEPLOYMENT_SECTOR_ADDRESS     {0x08040000, 0x08060000}
#define FLASH_DEPLOYMENT_SECTOR_SIZE        {0x00020000, 0x00020000}


// Macro
#define PORT_PIN(port,pin)              (((int)port)*16 + (pin))
#define _P(port, pin)                   PORT_PIN(GPIO_PORT##port, pin)
#define _P_NONE_                        GPIO_PIN_NONE
#define GPIO_PORTA  0
#define GPIO_PORTB  1
#define GPIO_PORTC  2
#define GPIO_PORTD  3


// GPIO
#define TOTAL_GPIO_PORT                 (GPIO_PORTD + 1)
#define TOTAL_GPIO_PINS                 (TOTAL_GPIO_PORT*16)

// I2C
#define STM32F4_ADC 					1
#define STM32F4_AD_CHANNELS 			{ _P(A,0),_P(A,1),_P(A,4),_P(A,5),_P(A,6),_P(A,7),_P(B,0),_P(B,1),_P(C,0),_P(C,1),_P(C,2),_P(C,3),_P(C,4),_P(C,5) }

// PWM
#define MAX_PWM_PER_CONTROLLER               4
#define TOTAL_PWM_CONTROLLER                 4
#define STM32F4_PWM                     {   {TIM1  ,  STM32F4_Gpio_AlternateFunction::AF1,     {  _P(A, 8) , _P(A, 9) , _P(A,10) , _P_NONE_} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0, 1} ,\
                                            {TIM2  ,  STM32F4_Gpio_AlternateFunction::AF1,     { _P(A, 5) , _P(B, 3) , _P(A, 2) , _P(A, 3)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0, 2} ,\
                                            {TIM3  ,  STM32F4_Gpio_AlternateFunction::AF2,     { _P(A, 6) , _P(C, 7) , _P(C, 8) , _P(C, 9)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0, 3} ,\
                                            {TIM4  ,  STM32F4_Gpio_AlternateFunction::AF2,     { _P(B, 6) , _P(B, 7) , _P(B, 8) , _P(B, 9)} ,  { false, false, false, false }, 0.0, 0.0, {0.0, 0.0, 0.0, 0.0}, 0, 0, 4} }

// I2C
#define STM32F4_I2C_PORT                1
#define STM32F4_I2C_SCL_PIN             _P(B, 8)
#define STM32F4_I2C_SDA_PIN             _P(B, 9)

// SPI
#define TOTAL_SPI_CONTROLLERS           2  //SPI1     SPI3
#define STM32F4_SPI_SCLK_PINS           { _P(A, 5), _P(B,3) }
#define STM32F4_SPI_MISO_PINS           { _P(A, 6), _P(B,4) }
#define STM32F4_SPI_MOSI_PINS           { _P(A, 7), _P(B,5) }

// UART
#define TOTAL_UART_CONTROLLERS          2
#define STM32F4_UART_TX_BUFFER_SIZE     256
#define STM32F4_UART_RX_BUFFER_SIZE     512
                                        // UART 1         UART 2
#define STM32F4_UART_RXD_PINS           { _P(A,10),      _P(A, 3) }
#define STM32F4_UART_TXD_PINS           { _P(A, 9),      _P(A, 2) }
#define STM32F4_UART_CTS_PINS           { GPIO_PIN_NONE, GPIO_PIN_NONE }
#define STM32F4_UART_RTS_PINS           { GPIO_PIN_NONE, GPIO_PIN_NONE }

//  USBC
#define TOTAL_USB_CONTROLLER            1
#define USB_MAX_QUEUES                  16
#define USB_VENDOR_ID                   0x1B9F
#define USB_PRODUCT_ID                  0x0110
#define USB_MANUFACTURER_NAME           {'S', 'T', 'M', 'i', 'c', 'r', 'o', 'e', 'l', 'e', 'c', 't', 'r', 'o', 'n', 'i', 'c', 's'}
#define USB_PRODUCT_NAME                {'N', 'U', 'C', 'L', 'E', 'O', '-', 'F', '4', '1', '1', 'R', 'E'}
#define USB_DISPLAY_NAME                USB_PRODUCT_NAME
#define USB_FRIENDLY_NAME               USB_PRODUCT_NAME


// Debug
#define DEBUG_TEXT_PORT                 USB1
#define STDIO                           USB1
#define DEBUGGER_PORT                   USB1
#define MESSAGING_PORT                  USB1

#define LMODE_PIN                       _P(B,1)
#define LMODE_USB_STATE                 TinyCLR_Gpio_PinValue::Low

// Loader
#define RUNAPP_PIN						_P(B,2)
#define RUNAPP_STATE 				    TinyCLR_Gpio_PinValue::High

#define UART_DEBUGGER_INDEX 1
#define USB_DEBUGGER_INDEX 0

// OEM information
#define OEM_STRING                      "STMicroelectronics\0"
#define OEM_VERSION_MAJOR               0
#define OEM_VERSION_MINOR               6
#define OEM_VERSION_PATCH               0


// STM32F4 requires to define specific
#define STM32F4XX						1
#define STM32F401xE                     1 //should be STM32F411xE but do not work

#define INCLUDE_ADC
#define INCLUDE_GPIO
#define INCLUDE_I2C
#define INCLUDE_PWM
#define INCLUDE_SPI
#define INCLUDE_UART
#define INCLUDE_USBCLIENT 

#define TARGET STM32F4

#include <STM32F4.h>

#endif
