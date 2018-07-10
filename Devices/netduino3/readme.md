# netduino
## netduino 3

Originally a .NET Micro Framework product, netduino 3 is available in three flavors:

- [netduino 3](http://www.netduino.com/netduino3/specs.htm)
- [netduino 3 Ethernet](http://www.netduino.com/netduino3ethernet/specs.htm)
- [netduino 3 Wi-Fi](http://www.netduino.com/netduino3wifi/specs.htm)

> [!Note]
> We do not support, test, or guarantee TinyCLR's operation on devices from other manufactures. We are only sharing the experiences of ourselves and others running TinyCLR on third party boards.

> [!Tip]
> Currently the firmware is the same for all three devices and doesn't include any networking support.

The netduino3 firmware source is located in the [TinyCLR ports repository](https://github.com/ghi-electronics/TinyCLR-Ports). Please go to our porting guide for instructions on how to build the firmware. To put the netduino in DFU mode, hold its one button down while plugging in the USB cable. Now use the STM32 DFU tool to load the firmware.

> [!Tip]
>The region set aside for RLI is 0x2002F000 - 0x2002FFF8.

## Pin Definitions
This should help in getting to pins available on the netduino3

```
namespace GHIElectronics.TinyCLR.Pins {
    /// <summary>Board definition for the FEZCerbuinoNet.</summary>
    public static class Netduino3 {
        /// <summary>GPIO pin definitions.</summary>
        public static class GpioPin {

            /// <summary>Debug LED definition</summary>
            public const int Led = STM32F4.GpioPin.PA10;
            public const int PowerLed = STM32F4.GpioPin.PC13;

            /// <summary>SD Card Dectect definition</summary>
            public const int SdCardDetect = STM32F4.GpioPin.PB2;
            public const int SdCardPwrCtrl = STM32F4.GpioPin.PB1;

            /// <summary>GPIO pin.</summary>
            public const int D0 = STM32F4.GpioPin.PC7;
            /// <summary>GPIO pin.</summary>
            public const int D1 = STM32F4.GpioPin.PC6;
            /// <summary>GPIO pin.</summary>
            public const int D2 = STM32F4.GpioPin.PA3;
            /// <summary>GPIO pin.</summary>
            public const int D3 = STM32F4.GpioPin.PA2;
            /// <summary>GPIO pin.</summary>
            public const int D4 = STM32F4.GpioPin.PB12;
            /// <summary>GPIO pin.</summary>
            public const int D5 = STM32F4.GpioPin.PB8;
            /// <summary>GPIO pin.</summary>
            public const int D6 = STM32F4.GpioPin.PB9;
            /// <summary>GPIO pin.</summary>
            public const int D7 = STM32F4.GpioPin.PA1;
            /// <summary>GPIO pin.</summary>
            public const int D8 = STM32F4.GpioPin.PA0;
            /// <summary>GPIO pin.</summary>
            public const int D9 = STM32F4.GpioPin.PE5;
            /// <summary>GPIO pin.</summary>
            public const int D10 = STM32F4.GpioPin.PB10;
            /// <summary>GPIO pin.</summary>
            public const int D11 = STM32F4.GpioPin.PB15;
            /// <summary>GPIO pin.</summary>
            public const int D12 = STM32F4.GpioPin.PB14;
            /// <summary>GPIO pin.</summary>
            public const int D13 = STM32F4.GpioPin.PB13;
            /// <summary>GPIO pin.</summary>
            public const int A0 = STM32F4.GpioPin.PC0;
            /// <summary>GPIO pin.</summary>
            public const int A1 = STM32F4.GpioPin.PC1;
            /// <summary>GPIO pin.</summary>
            public const int A2 = STM32F4.GpioPin.PC2;
            /// <summary>GPIO pin.</summary>
            public const int A3 = STM32F4.GpioPin.PC3;
            /// <summary>GPIO pin.</summary>
            public const int A4 = STM32F4.GpioPin.PC4;
            /// <summary>GPIO pin.</summary>
            public const int A5 = STM32F4.GpioPin.PC5;

            /// <summary>Socket definition.</summary>
            public static class GoPort1 {
                /// <summary>Pin definition.</summary>
                public const int Pin3 = STM32F4.GpioPin.PD13;
                /// <summary>Pin definition.</summary>
                public const int Pin4 = STM32F4.GpioPin.PD8;
                /// <summary>Pin definition.</summary>
                public const int Pin5 = STM32F4.GpioPin.PD9;
                /// <summary>Pin definition.</summary>
                public const int Pin6 = STM32F4.GpioPin.PD0;
                /// <summary>LED definition.</summary>
                public const int Led = STM32F4.GpioPin.PE9;
                /// <summary>Power On definition.</summary>
                public const int PwrOn = STM32F4.GpioPin.PD7;
            }

            /// <summary>Socket definition.</summary>
            public static class GoPort2 {
                /// <summary>Pin definition.</summary>
                public const int Pin3 = STM32F4.GpioPin.PD14;
                /// <summary>Pin definition.</summary>
                public const int Pin4 = STM32F4.GpioPin.PE8;
                /// <summary>Pin definition.</summary>
                public const int Pin5 = STM32F4.GpioPin.PE7;
                /// <summary>Pin definition.</summary>
                public const int Pin6 = STM32F4.GpioPin.PD1;
                /// <summary>LED definition.</summary>
                public const int Led = STM32F4.GpioPin.PE11;
                /// <summary>Power On definition.</summary>
                public const int PwrOn = STM32F4.GpioPin.PD10;
            }

            /// <summary>Socket definition.</summary>
            public static class GoPort3 {
                /// <summary>Pin definition.</summary>
                public const int Pin3 = STM32F4.GpioPin.PD15;
                /// <summary>Pin definition.</summary>
                public const int Pin4 = STM32F4.GpioPin.PE1;
                /// <summary>Pin definition.</summary>
                public const int Pin5 = STM32F4.GpioPin.PE0;
                /// <summary>Pin definition.</summary>
                public const int Pin6 = STM32F4.GpioPin.PD2;
                /// <summary>LED definition.</summary>
                public const int Led = STM32F4.GpioPin.PB0;
                /// <summary>Power On definition.</summary>
                public const int PwrOn = STM32F4.GpioPin.PE14;
            }
        }

        /// <summary>Analog channel definition.</summary>
        public static class AdcChannel {
            /// <summary>Pin definition.</summary>
            public const int A0 = STM32F4.AdcChannel.Channel10;
            /// <summary>Pin definition.</summary>
            public const int A1 = STM32F4.AdcChannel.Channel11;
            /// <summary>Pin definition.</summary>
            public const int A2 = STM32F4.AdcChannel.Channel12;
            /// <summary>Pin definition.</summary>
            public const int A3 = STM32F4.AdcChannel.Channel13;
            /// <summary>Pin definition.</summary>
            public const int A4 = STM32F4.AdcChannel.Channel14;
            /// <summary>Pin definition.</summary>
            public const int A5 = STM32F4.AdcChannel.Channel15;
        }

        /// <summary>Uart port definition.</summary>
        public static class UartPort {

            /// <summary>Socket definition.</summary>
            public const string GoPort1 = STM32F4.UartPort.Usart3;
            /// <summary>Socket definition.</summary>
            public const string GoPort2 = STM32F4.UartPort.Uart7;
            /// <summary>Socket definition.</summary>
            public const string GoPort3 = STM32F4.UartPort.Uart8;

            /// <summary>UART D0 (RX) and D1 (TX).</summary>
            public const string Uart6 = STM32F4.UartPort.Uart6;
        }

        /// <summary>SPI Bus definition.</summary>
        public static class SpiBus {

            /// <summary>Socket definition.</summary>
            public const string GoPort1 = STM32F4.SpiBus.Spi4;
            public const string GoPort2 = STM32F4.SpiBus.Spi4;
            public const string GoPort3 = STM32F4.SpiBus.Spi4;
            public const string Spi2 = STM32F4.SpiBus.Spi2;
        }
    }
}
```
## netduino 2
This board uses STM32F2, which is very similar to the STM32F4 used on the netduino 3. Porting TinyCLR OS to netduino 2 should not be very difficult.

## netduino 1
This board uses an Atmel microcontroller. The G400 uses an Atmel processor and can be a good starting point for porting TinyCLR OS.
