# STM32 Boards
---
The market is full of STM32 boards -- especially from ST Microelectronics.  ST offers several evaluation boards for their STM32 product families. The FEZCLR reference firmware runs on the STM32F4 family of processors, which is their most popular.

> [!Note]
> We do not support, test, or guarantee TinyCLR's operation on devices from other manufactures. We are only sharing the experiences of ourselves and others running TinyCLR on third party boards.

## STM32F4 Discovery
![STM32F411 Discovery Board](images/STM32F411-discovery-board.jpg)

The ST Microelectronics [STM32F411 Discovery board](http://www.st.com/en/evaluation-tools/32f411ediscovery.html) works with the [FEZCLR firmware](../downloads.md#fezclr). Follow the instructions for [Uploading DFU Files](../loaders/stm32-bootloader.md#uploading-dfu-files) to load the bootloader. To load the TinyCLR firmware please go to the [Loading the Firmware](../loaders/ghi-bootloader.md#loading-the-firmware) section of the GHI bootloader page. To build your own firmware from source, please visit our [Porting TinyCLR](../porting/intro.md) page.

## STM32F4 Nucleo
![Nucleo-F401RE](images/nucleo-f401re.jpg)

The ST Microelectronics [Nucleo-F401RE](http://www.st.com/en/evaluation-tools/nucleo-f401re.html) board works with the [FEZCLR firmware](../downloads.md#fezclr). Follow the instructions for [Uploading DFU Files](../loaders/stm32-bootloader.md#uploading-dfu-files) to load the bootloader. To load the TinyCLR firmware please go to the [Loading the Firmware](../loaders/ghi-bootloader.md#loading-the-firmware) section of the GHI bootloader page. To build your own firmware from source, please visit our [Porting TinCLR](../porting/intro.md) page.

> [!Tip]
> The Nucleo board doesn't include a USB connector but has the connections needed to add a USB connector.

Check out this [TinyCLR Fork](https://github.com/valoni/TinyCLR-Ports/tree/master/Devices/NUCLEO-F411RE) started with the FEZCLR port to create a port for the [Nucleo-F411RE](http://www.st.com/en/evaluation-tools/nucleo-f411re.html).

## Other STM32F4 boards
All STM32F4 microcontrollers are very similar. Any board (Discovery, Nucleo and others) with an 8Mhz crystal should simply run the [FEZCLR firmware](../downloads.md#fezclr). However, you can simply change the crystal clock in the port and recompile the firmware. Go to our [Porting TinyCLR](../porting/intro.md) page to get started.

***

Visit our main website at [**www.ghielectronics.com**](http://www.ghielectronics.com) and our community forums at [**forums.ghielectronics.com**](https://forums.ghielectronics.com/).