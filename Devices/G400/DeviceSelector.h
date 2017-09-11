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
#define SYSTEM_CYCLE_CLOCK_HZ               (400*1000*1000) // 400 MHz
#define SYSTEM_PERIPHERAL_CLOCK_HZ          (SYSTEM_CYCLE_CLOCK_HZ / 3) // 133MHz
#define SYSTEM_CLOCK_HZ                     (SYSTEM_PERIPHERAL_CLOCK_HZ/32) // 4MHz
#define CLOCK_COMMON_FACTOR                 5000 // 333333 //5000
#define SLOW_CLOCKS_PER_SECOND              (SYSTEM_CLOCK_HZ - ((SYSTEM_CLOCK_HZ/45))) //in 4.3.7.8, we use Div = 3 = 133MHz / 3 for slow lock
#define SLOW_CLOCKS_TEN_MHZ_GCD             1000 // 33333 //1000
#define SLOW_CLOCKS_MILLISECOND_GCD         10

// Memory
#define SRAM_MEMORY_Base                    0x00300000
#define SRAM_MEMORY_Size                    (32*1024)
#define SDRAM_MEMORY_Base                   0x20000000
#define SDRAM_MEMORY_Size                   (128*1024*1024)

//Flash
#define FLASH_MEMORY_Base                   0x00000000
#define FLASH_MEMORY_Size                   0x00420000


// Macro
#define PORT_PIN(port,pin)                   (((int)port)*32 + (pin))
#define _P(port, pin)                        PORT_PIN(GPIO_PORT##port, pin)
#define _P_NONE_                             GPIO_PIN_NONE
#define GPIO_PORTA                           0
#define GPIO_PORTB                           1
#define GPIO_PORTC                           2
#define GPIO_PORTD                           3

// GPIO
#define TOTAL_GPIO_PORT                      (GPIO_PORTD + 1)
#define TOTAL_GPIO_PINS                      (TOTAL_GPIO_PORT*32)

// USBC
#define TOTAL_USB_CONTROLLER            1
#define USB_MAX_QUEUES                  16
#define USB_VENDOR_ID                   0x1B9F
#define USB_PRODUCT_ID                  0x0110
#define USB_MANUFACTURER_NAME           {'G', 'H', 'I', ' ', 'E', 'l', 'e', 'c', 't', 'r', 'o', 'n', 'i', 'c', 's', ',', ' ', 'L', 'L', 'C'}
#define USB_PRODUCT_NAME                {'G', '4', '0', '0'}
#define USB_DISPLAY_NAME                USB_PRODUCT_NAME
#define USB_FRIENDLY_NAME               USB_PRODUCT_NAME

// ADC
#define TOTAL_ADC_CONTROLLERS             12
#define AT91_ADC_PINS                     {_P(B,11),_P(B,12),_P(B,13),_P(B,14),_P(B,15),_P(B,16),_P(A,17),_P(B,6),_P(B,7),_P(B,8),_P(B,9),_P(B,10)}
#define AT91_ADC_ALT_MODE                 {AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD, AT91_Gpio_PeripheralSelection::PeripheralD}

// I2C
#define AT91_I2C_SCL_PIN                  _P(A,31)
#define AT91_I2C_SDA_PIN                  _P(A,30)
#define AT91_I2C_SCL_ALT_MODE             AT91_Gpio_PeripheralSelection::PeripheralA
#define AT91_I2C_SDA_ALT_MODE             AT91_Gpio_PeripheralSelection::PeripheralA

// PWM
#define MAX_PWM_PER_CONTROLLER            1
#define TOTAL_PWM_CONTROLLER              4
#define AT91_PWM                          { {PWM_CHANNEL_MODE_REGISTER(0), PWM_DUTY_REGISTER(0), PWM_CHANNEL_UPDATE_REGISTER(0), {_P(C,18)}, {AT91_Gpio_PeripheralSelection::PeripheralC}, { false }, 0.0, {0.0} } ,\
                                            {PWM_CHANNEL_MODE_REGISTER(1), PWM_DUTY_REGISTER(1), PWM_CHANNEL_UPDATE_REGISTER(1), {_P(C,19)}, {AT91_Gpio_PeripheralSelection::PeripheralC}, { false }, 0.0, {0.0} } ,\
                                            {PWM_CHANNEL_MODE_REGISTER(2), PWM_DUTY_REGISTER(2), PWM_CHANNEL_UPDATE_REGISTER(2), {_P(C,20)}, {AT91_Gpio_PeripheralSelection::PeripheralC}, { false }, 0.0, {0.0} } ,\
                                            {PWM_CHANNEL_MODE_REGISTER(3), PWM_DUTY_REGISTER(3), PWM_CHANNEL_UPDATE_REGISTER(3), {_P(C,21)}, {AT91_Gpio_PeripheralSelection::PeripheralC}, { false }, 0.0, {0.0} } }
// SPI
#define TOTAL_SPI_CONTROLLERS              2
#define AT91_SPI_MISO_PINS                {_P(A,11),_P(A,21)}
#define AT91_SPI_MOSI_PINS                {_P(A,12),_P(A,22)}
#define AT91_SPI_CLK_PINS                 {_P(A,13),_P(A,23)}
#define AT91_SPI_MISO_ALT_MODE            {AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralB}
#define AT91_SPI_MOSI_ALT_MODE            {AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralB}
#define AT91_SPI_CLK_ALT_MODE             {AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralB}

// UART
#define TOTAL_UART_CONTROLLERS            6
#define AT91_UART_TX_BUFFER_SIZE          (16*1024)
#define AT91_UART_RX_BUFFER_SIZE          (16*1024)
#define AT91_UART_TX_PINS                 {_P(A, 9), _P(A, 0), _P(A, 5), _P(A, 7), _P(C, 8), _P(C,16) }
#define AT91_UART_RX_PINS                 {_P(A,10), _P(A, 1), _P(A, 6), _P(A, 8), _P(C, 9), _P(C,17) }
#define AT91_UART_RTS_PINS                {_P_NONE_, _P(A, 2), _P_NONE_, _P_NONE_, _P_NONE_, _P_NONE_ }
#define AT91_UART_CTS_PINS                {_P_NONE_, _P(A, 3), _P_NONE_, _P_NONE_, _P_NONE_, _P_NONE_ }
#define AT91_UART_TX_ALT_MODE             {AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralA,AT91_Gpio_PeripheralSelection::PeripheralA ,AT91_Gpio_PeripheralSelection::PeripheralC ,AT91_Gpio_PeripheralSelection::PeripheralC}
#define AT91_UART_RX_ALT_MODE             {AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::PeripheralA,AT91_Gpio_PeripheralSelection::PeripheralA ,AT91_Gpio_PeripheralSelection::PeripheralC ,AT91_Gpio_PeripheralSelection::PeripheralC}
#define AT91_UART_RTS_ALT_MODE            {AT91_Gpio_PeripheralSelection::None       , AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::None       ,AT91_Gpio_PeripheralSelection::None        ,AT91_Gpio_PeripheralSelection::None        ,AT91_Gpio_PeripheralSelection::None}
#define AT91_UART_CTS_ALT_MODE            {AT91_Gpio_PeripheralSelection::None       , AT91_Gpio_PeripheralSelection::PeripheralA, AT91_Gpio_PeripheralSelection::None       ,AT91_Gpio_PeripheralSelection::None        ,AT91_Gpio_PeripheralSelection::None        ,AT91_Gpio_PeripheralSelection::None}

// Debug
#define DEBUG_TEXT_PORT                      USB1
#define STDIO                                USB1
#define DEBUGGER_PORT                        USB1
#define MESSAGING_PORT                       USB1

#define LMODE_PIN                       	_P(A,25)
#define LMODE_USB_STATE                 	TinyCLR_Gpio_PinValue::High

// Loader
#define RUNAPP_PIN                          _P(A,4)
#define RUNAPP_STATE                        TinyCLR_Gpio_PinValue::High

#define UART_DEBUGGER_INDEX                 0
#define USB_DEBUGGER_INDEX                  0

#define RAM_BOOTLOADER_HOLD_ADDRESS         0x266FFFF0
#define RAM_BOOTLOADER_HOLD_VALUE           0x8DB4DA67

// OEM information
#define OEM_STRING                          "GHI Electronics, LLC\0"
#define OEM_VERSION_MAJOR                   0
#define OEM_VERSION_MINOR                   6
#define OEM_VERSION_PATCH                   0

#define INCLUDE_ADC
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