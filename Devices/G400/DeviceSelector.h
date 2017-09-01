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

#ifndef _DEVICE_G400_H_
#define _DEVICE_G400_H_

#define PLATFORM_ARM_DEFINED

// Macro
#define GLOBAL_LOCK(x)             AT91_SmartPtr_IRQ x
#define DISABLE_INTERRUPTS()       AT91_SmartPtr_IRQ::ForceDisabled()
#define ENABLE_INTERRUPTS()        AT91_SmartPtr_IRQ::ForceEnabled()

#if defined(_DEBUG)
#define ASSERT(x)                  while (!(x))
#define ASSERT_IRQ_MUST_BE_OFF()   ASSERT(!AT91_SmartPtr_IRQ::GetState())
#define ASSERT_IRQ_MUST_BE_ON()    ASSERT( AT91_SmartPtr_IRQ::GetState())
#else
#define ASSERT_IRQ_MUST_BE_OFF()
#define ASSERT_IRQ_MUST_BE_ON()
#endif

#define INTERRUPT_START         AT91_Interrupt_Started();
#define INTERRUPT_END           AT91_Interrupt_Ended();

// Device
#define HAL_SYSTEM_NAME "G400"

// System clock
#define SYSTEM_CLOCK_HZ                      18000000 // 18 MHz
#define SYSTEM_CYCLE_CLOCK_HZ                72000000 // 72 MHz
#define CLOCK_COMMON_FACTOR                  1000000 // GCD(SYSTEM_CLOCK_HZ, 1M)
#define SLOW_CLOCKS_PER_SECOND               18000000 // SYSTEM_CLOCK_HZ MHz
#define SLOW_CLOCKS_TEN_MHZ_GCD              1000000 // GCD(SLOW_CLOCKS_PER_SECOND, 10M)
#define SLOW_CLOCKS_MILLISECOND_GCD          1000 // GCD(SLOW_CLOCKS_PER_SECOND, 1k)

// external Memory - Flash
#define EXT_FLASH_MEMORY_Base                0x80000000
#define EXT_FLASH_MEMORY_Size                0x00400000 // 4MB ext flash
#define EXT_SDRAM_MEMORY_Base                0xA0000000
#define EXT_SDRAM_MEMORY_Size                0x01000000 // 16MB ext ram

// Macro
#define PORT_PIN(port,pin)                   (((int)port)*32 + (pin))
#define _P(port, pin)                        PORT_PIN(GPIO_PORT##port, pin)
#define _P_NONE_                             GPIO_PIN_NONE
#define GPIO_PORT0                           0
#define GPIO_PORT1                           1
#define GPIO_PORT2                           2
#define GPIO_PORT3                           3
#define GPIO_PORT4                           4

// GPIO
#define TOTAL_GPIO_PORT                      (GPIO_PORT4 + 1)
#define TOTAL_GPIO_PINS                      (TOTAL_GPIO_PORT*32)

// USBC
#define TOTAL_USB_CONTROLLER            1
#define USB_MAX_QUEUES                  16
#define USB_VENDOR_ID                   0x1B9F
#define USB_PRODUCT_ID                  0x0110
#define USB_MANUFACTURER_NAME           {'G', 'H', 'I', ' ', 'E', 'l', 'e', 'c', 't', 'r', 'o', 'n', 'i', 'c', 's', ',', ' ', 'L', 'L', 'C'}
#define USB_PRODUCT_NAME                {'E', 'M', 'X'}
#define USB_DISPLAY_NAME                USB_PRODUCT_NAME
#define USB_FRIENDLY_NAME               USB_PRODUCT_NAME

// UART
#define TOTAL_UART_CONTROLLERS               4
#define AT91_UART_TX_BUFFER_SIZE           (16*1024)
#define AT91_UART_RX_BUFFER_SIZE           (16*1024)

// ADC
#define TOTAL_ADC_CONTROLLERS               8
#define AT91_ADC_PINS                     {_P(0,23),_P(0,24),_P(0,25),_P(0,26),_P_NONE_,_P(1,31),_P(0,12),_P(0,13)}
#define AT91_ADC_ALT_MODE                 {AT91_Gpio_PinFunction::PinFunction1, AT91_Gpio_PinFunction::PinFunction1, AT91_Gpio_PinFunction::PinFunction1, AT91_Gpio_PinFunction::PinFunction1, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3}

// DAC
#define TOTAL_DAC_CONTROLLERS                1
#define AT91_DAC_PINS                     {_P(0,26)}
#define AT91_DAC_ALT_MODE                 {AT91_Gpio_PinFunction::PinFunction2}

// I2C
#define AT91_I2C_PCLK_KHZ                 SYSTEM_CLOCK_HZ/1000
#define AT91_I2C_SCL_PIN                  _P(0,28)
#define AT91_I2C_SDA_PIN                  _P(0,27)
#define AT91_I2C_SCL_ALT_MODE             AT91_Gpio_PinFunction::PinFunction1
#define AT91_I2C_SDA_ALT_MODE             AT91_Gpio_PinFunction::PinFunction1

// PWM
#define AT91_PWM_PCLK                     (SYSTEM_CLOCK_HZ)
#define MAX_PWM_PER_CONTROLLER              6
#define TOTAL_PWM_CONTROLLER                2
#define AT91_PWM                           {   { 0 ,  {0x0, 0x0, 0x0, 0x0, 0x0, 0x0} , {0x0, 0x1, 0x2, 0x3, 0x4, 0x5 } , { _P(3, 16), _P(3, 17), _P_NONE_ , _P_NONE_ , _P_NONE_ , _P_NONE_},   {AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3},  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {PWM0MR1, PWM0MR2, PWM0MR3, PWM0MR4, PWM0MR5, PWM0MR6 }, { false, false, false, false, false, false }, 0.0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  } ,\
                                                { 1 ,  {0x1, 0x1, 0x1, 0x1, 0x1, 0x1} , {0x0, 0x1, 0x2, 0x3, 0x4, 0x5 } , { _P(3, 24), _P_NONE_ , _P(3, 26), _P(3, 27), _P_NONE_ , _P(3, 29)},  {AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction3},  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {PWM1MR1, PWM1MR2, PWM1MR3, PWM1MR4, PWM1MR5, PWM1MR6 }, { false, false, false, false, false, false }, 0.0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  } }
// SPI
#define TOTAL_SPI_CONTROLLERS               2
#define AT91_SPI_PCLK_KHZ                 (SYSTEM_CYCLE_CLOCK_HZ/1000)
#define AT91_SPI_MISO_PINS                {_P(0,17),_P(0,8)}
#define AT91_SPI_MOSI_PINS                {_P(0,18),_P(0,9)}
#define AT91_SPI_CLK_PINS                 {_P(0,15),_P(0,7)}
#define AT91_SPI_MISO_ALT_MODE            {AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction2}
#define AT91_SPI_MOSI_ALT_MODE            {AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction2}
#define AT91_SPI_CLK_ALT_MODE             {AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction2}

// UARTs
#define AT91_UART_PCLK                   SYSTEM_CYCLE_CLOCK_HZ
//UART G400
#define AT91_UART_TX_PINS              {_P(0,2) , _P(2,0) , _P(4,22), _P(0,25) }
#define AT91_UART_RX_PINS              {_P(0,3) , _P(2,1) , _P(4,23), _P(0,26)  }
#define AT91_UART_RTS_PINS             {_P_NONE_, _P(3,30), _P_NONE_, _P_NONE_ }
#define AT91_UART_CTS_PINS             {_P_NONE_, _P(3,18), _P_NONE_, _P_NONE_ }
#define AT91_UART_TX_ALT_MODE          {AT91_Gpio_PinFunction::PinFunction1, AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction2,AT91_Gpio_PinFunction::PinFunction3}
#define AT91_UART_RX_ALT_MODE          {AT91_Gpio_PinFunction::PinFunction1, AT91_Gpio_PinFunction::PinFunction2, AT91_Gpio_PinFunction::PinFunction2,AT91_Gpio_PinFunction::PinFunction3}
#define AT91_UART_RTS_ALT_MODE         {AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction0,AT91_Gpio_PinFunction::PinFunction0}
#define AT91_UART_CTS_ALT_MODE         {AT91_Gpio_PinFunction::PinFunction0, AT91_Gpio_PinFunction::PinFunction3, AT91_Gpio_PinFunction::PinFunction0,AT91_Gpio_PinFunction::PinFunction0}

// Debug
#define DEBUG_TEXT_PORT                      USB1
#define STDIO                                USB1
#define DEBUGGER_PORT                        USB1
#define MESSAGING_PORT                       USB1

#define LMODE_PIN                       	_P(3,23)
#define LMODE_USB_STATE                 	TinyCLR_Gpio_PinValue::Low

// Loader
#define RUNAPP_PIN                          _P(0,4)
#define RUNAPP_STATE                        TinyCLR_Gpio_PinValue::High

#define UART_DEBUGGER_INDEX                 0
#define USB_DEBUGGER_INDEX                  0

#define RAM_BOOTLOADER_HOLD_ADDRESS         (EXT_SDRAM_MEMORY_Base + EXT_SDRAM_MEMORY_Size - 8)
#define RAM_BOOTLOADER_HOLD_VALUE           0xB53D0238

// OEM information
#define OEM_STRING                          "GHI Electronics, LLC\0"
#define OEM_VERSION_MAJOR                   0
#define OEM_VERSION_MINOR                   6
#define OEM_VERSION_PATCH                   0

#define INCLUDE_ADC
#define INCLUDE_DAC
#define INCLUDE_GPIO
#define INCLUDE_I2C
#define INCLUDE_PWM
#define INCLUDE_SPI
#define INCLUDE_UART
#define INCLUDE_USBCLIENT
#define INCLUDE_DISPLAY

#define TARGET AT91

#include <AT91.h>

#endif