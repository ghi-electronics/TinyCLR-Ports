### TinyCLR Ports for Electron STM32F411RET6 

This repository hosts ports of TinyCLR to many different devices. 
For information on porting TinyCLR of the device or building a firmware for an existing port done based on, see http://docs.ghielectronics.com/tinyclr/porting/intro.html.

|Hardware Info|      Description      |
|-------------|-----------------------|
|Board        |Electron STM32F411CEU6 |
|FLASH        |      512 KB           |
|RAM          |      128 KB           |
|Pinout       |     UFQFPN48          |

|ADC Channel 12 Bit|
|------------------|
|        10        |

###
GPIO Pins
PA1, PA2, PA3, PA4, PA5, PA8, PAB3, PB4, PB5, PB6, PB7

### Analog Pins 
PA2(ADC2), PA3(ADC3), PA4(ADC4), PA5(ADC5)

### PWM
PA1(PWM4), PA2(PWM5), PA3(PWM6), PA8(PWM0), PB6(PWM12), PB7(PWM11)

|SPI INFO| SPI1 |
|--------|------|
|  SCK   | PB3  |
|  MISO  | PB4  |
|  MOSI  | PB5  |


|UART INFO| USART2 |
|---------|--------|
|    TX   |   PA2  | 
|    RX   |   PA3  | 
|   CTX   |  NONE  |
|   RTS   |  NONE  |

|I2C| SCL | SDA |
|---|-----|------|
| / | PB6 | PB7 |



PORTER : BAULAND based on work of JUSTIN and VALON HOTI
