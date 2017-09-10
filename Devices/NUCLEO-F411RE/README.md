TinyCLR Ports for Nucleo STM32F411RET6 

This repository hosts ports of TinyCLR to many different devices. 
For information on porting TinyCLR of the device or building a firmware for an existing port done based on, see http://docs.ghielectronics.com/tinyclr/porting/intro.html.

|Hardware Info| Description|
|------|------|
|Board|Nucleo STM32F411RET6 |
|FLASH| 512 KB |
|RAM| 128 KB |
|Type 1|Arduino Pinout Compatible|
|Type 2|Morp Pinout|

Nucleo STM32F411RET6 Pinout:
### Arduino Headers 
![alt text](https://developer.mbed.org/media/uploads/bcostm/nucleo64_revc_f411re_mbed_pinout_v2_arduino.png "Arduino Headers")

### Morpho Headers 
![alt text]( https://developer.mbed.org/media/uploads/bcostm/nucleo64_revc_f411re_mbed_pinout_v2_morpho.png "Morpho Headers")

|ADC Channel 12 Bit|
|------|
|1|

### Analog Pins 
PA0,PA1,PA4,PA5,PA6,PA7,PB0,PB1,PC0,PC1,PC2,PC3,PC4,PC5 

### PWM
PA0,PA1,PA5,PA6,PA8,PA9,PA10,PA11,PA15,PB3,PB4,PB5,PB6,PB7,PB8,PB9,PB10,PC6,PC7,PC8,PC9 

| SPI INFO| SPI1| SPI3 |
|------|------|------|
|SLK   | PA5  | PB3 |
|MISO| PA6  | PB4 |
|MOSI| PA7  | PB5 | |


|UART INFO| USART1| USART2 |
|------|------|------|
|TX  | PA9  | PA2 | 
|RX| PA10  | PA3 | 
|CTX| NONE  | NONE |
|RTS| NONE  | NONE |

|I2C INFO| SCL| SDA |
|------|------|------|
| / | PB8  | PB9 |


contact : valon.hoti@gmail.com 
