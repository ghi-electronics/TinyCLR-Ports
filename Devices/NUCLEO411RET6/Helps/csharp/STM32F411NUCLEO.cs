using System;
using System.Runtime.InteropServices;

namespace STM
{
    public static class NUCLEO
    {
        public static class STM32F411
        {
            public static string name = "STM32F411RET6";
            public static string about = "Valon Hoti\n . Prishtine @20170727";
            public static int PinNumber(char port, byte pin)
            {
                if (port < 'A' || port > 'E') return 0;

                return ((port - 'A') * 16) + pin;
            }

            public static class GpioPin
            {
                public const string Id = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.GpioProvider\0";

                // Not connected
                public static int NONE = Convert.ToInt32("0xFFFFFFFF", 16);

                // pins
                public static int PA0 = 0;
                public static int PA1 = 1;
                public static int PA2 = 2;
                public static int PA3 = 3;
                public static int PA4 = 4;
                public static int PA5 = 5;
                public static int PA6 = 6;
                public static int PA7 = 7;
                public static int PA8 = 8;
                public static int PA9 = 9;
                public static int PA10 = 10;
                public static int PA11 = 11;
                public static int PA12 = 12;
                public static int PA13 = 13;
                public static int PA14 = 14;
                public static int PA15 = 15;

                public static int PB0 = 16;
                public static int PB1 = 17;
                public static int PB2 = 18;
                public static int PB3 = 19;
                public static int PB4 = 20;
                public static int PB5 = 21;
                public static int PB6 = 22;
                public static int PB7 = 23;
                public static int PB8 = 24;
                public static int PB9 = 25;
                public static int PB10 = 26;
                public static int PB11 = 27;
                public static int PB12 = 28;
                public static int PB13 = 29;
                public static int PB14 = 30;
                public static int PB15 = 31;

                public static int PC0 = 32;
                public static int PC1 = 33;
                public static int PC2 = 34;
                public static int PC3 = 35;
                public static int PC4 = 36;
                public static int PC5 = 37;
                public static int PC6 = 38;
                public static int PC7 = 39;
                public static int PC8 = 40;
                public static int PC9 = 41;
                public static int PC10 = 42;
                public static int PC11 = 43;
                public static int PC12 = 44;
                public static int PC13 = 45;
                public static int PC14 = 46;
                public static int PC15 = 47;

                public static int PD2 = 50;

                public static int PH0 = 112;
                public static int PH1 = 113;

                // ADC internal channels
                public static int ADC_TEMP = 240;
                public static int ADC_VREF = 241;
                public static int ADC_VBAT = 242;

            }

            public static class Arduino
            {
                //Arduino Fields
                public static string name = "ArduinoPins";

                public static int A0 = GpioPin.PA0;
                public static int A1 = GpioPin.PA1;
                public static int A2 = GpioPin.PA4;
                public static int A3 = GpioPin.PB0;
                public static int A4 = GpioPin.PC1;
                public static int A5 = GpioPin.PC0;

                public static int D0 = GpioPin.PA3;
                public static int D1 = GpioPin.PA2;
                public static int D2 = GpioPin.PA10;
                public static int D3 = GpioPin.PB3;
                public static int D4 = GpioPin.PB5;
                public static int D5 = GpioPin.PB4;
                public static int D6 = GpioPin.PB10;
                public static int D7 = GpioPin.PA8;
                public static int D8 = GpioPin.PA9;
                public static int D9 = GpioPin.PC7;
                public static int D10 = GpioPin.PB6;
                public static int D11 = GpioPin.PA7;
                public static int D12 = GpioPin.PA6;
                public static int D13 = GpioPin.PA5;

                public static int UserButton = GpioPin.PC13;
                public static int Led1 = GpioPin.PA5;

                public static int I2C_SCL = GpioPin.PB8;
                public static int I2C_SDA = GpioPin.PB9;

                public static int SPI_MOSI = GpioPin.PA7;
                public static int SPI_MISO = GpioPin.PA6;
                public static int SPI_SCK = GpioPin.PA5;
                public static int SPI_CS = GpioPin.PB6;

                public static int PWM_OUT = GpioPin.PB3;
            }

            public static class SerialPort
            {
                public static string Com1 = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.UartProvider\0";
                public static string Com2 = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.UartProvider\1";

                public struct COM1
                {
                    public static string name = "COM1";
                    public static int RX = GpioPin.PA10;
                    public static int TX = GpioPin.PA9;
                    public static int CTS = GpioPin.NONE;
                    public static int RTS = GpioPin.NONE;

                }

                public struct COM2
                {
                    public static string name = "COM2";
                    public static int RX = GpioPin.PA3;
                    public static int TX = GpioPin.PA2;
                    public static int CTS = GpioPin.PA0;
                    public static int RTS = GpioPin.PA1;

                }

            }

            public static class BaudRate
            {
                public static int Baud100 = 100;
                public static int Baud200 = 200;
                public static int Baud300 = 300;
                public static int Baud400 = 400;
                public static int Baud500 = 500;
                public static int Baud600 = 600;
                public static int Baud700 = 700;
                public static int Baud800 = 800;
                public static int Baud900 = 900;
                public static int Baud1000 = 1000;
                public static int Baud1100 = 1100;
                public static int Baud1200 = 1200;
                public static int Baud2400 = 2400;
                public static int Baud3600 = 3600;
                public static int Baud4800 = 4800;
                public static int Baud9600 = 9600;
                public static int Baud19200 = 19200;
                public static int Baud38400 = 38400;
                public static int Baud57600 = 57600;
                public static int Baud96000 = 96000;
                public static int Baud115200 = 115200;
                public static int Baud230400 = 230400;
                public static int Baud460800 = 460800;
                public static int Baud921600 = 921600;

            }

            public static class I2cBus
            {
                // Fields
                public const string I2c1 = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.I2cProvider\0";
            }

            public static class SPIControllers
            {
                public struct SPI1
                {
                    public static string name = "SP1";
                    public static int SCLK = GpioPin.PA5;
                    public static int MISO = GpioPin.PA6;
                    public static int MOSI = GpioPin.PA7;
                }

                public struct SPI2
                {
                    public static string name = "SP2";
                    public static int SCLK = GpioPin.PB13;
                    public static int MISO = GpioPin.PB14;
                    public static int MOSI = GpioPin.PB15;
                }

                public struct SPI3
                {
                    public static string name = "SP3";
                    public static int SCLK = GpioPin.PC10;
                    public static int MISO = GpioPin.PC11;
                    public static int MOSI = GpioPin.PC12;
                }
            }

            public static class SpiBus
            {
                public const string Spi1 = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.SpiProvider\0";
                public const string Spi2 = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.SpiProvider\1";
                public const string Spi3 = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.SpiProvider\2";
            }

            public static class AdcChannel
            {

                public const string Id = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.AdcProvider\0";
                public const int PA0 = 0;
                public const int PA1 = 1;
                public const int PA2 = 2;
                public const int PA3 = 3;
                public const int PA4 = 4;
                public const int PA5 = 5;
                public const int PA6 = 6;
                public const int PA7 = 7;
                public const int PB0 = 8;
                public const int PB1 = 9;
                public const int PC0 = 10;
                public const int PC1 = 11;
                public const int PC2 = 12;
                public const int PC3 = 13;
                public const int PC4 = 14;
                public const int PC5 = 15;

            }

            public static class PwmPin
            {
                // Nested Types
                public static class Controller1
                {
                    // Fields
                    public const string Id = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.PwmProvider\0";
                    public static int PA8 = 0;
                    public static int PA9 = 1;
                    public static int PA10 = 2;
                }

                public static class Controller2
                {
                    // Fields
                    public const string Id = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.PwmProvider\1";
                    public static int PA5 = 0;
                    public static int PB3 = 1;
                    public static int PA2 = 2;
                    public static int PA3 = 3;
                }

                public static class Controller3
                {
                    // Fields
                    public const string Id = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.PwmProvider\2";
                    public static int PA6 = 0;
                    public static int PC7 = 1;
                    public static int PC8 = 2;
                    public static int PC9 = 3;
                }

                public static class Controller4
                {
                    // Fields
                    public const string Id = @"GHIElectronics.TinyCLR.NativeApis.STM32F4.PwmProvider\3";
                    public static int PB6 = 0;
                    public static int PB7 = 1;
                    public static int PB8 = 2;
                    public static int PB9 = 3;
                }
            }

            //addedd 
            //Watchdog IWDG w TinyCLR OS
            //http://kodfilemon.blogspot.com/2017/07/systemruntimeinteropservicesmarshal-w.html

            public static string GetDeviceGuid()
            {
                var uidAddr = new IntPtr(0x1FFF7A10);
                int uid0 = Marshal.ReadInt32(uidAddr, 0);
                int uid1 = Marshal.ReadInt32(uidAddr, 0x04);
                int uid2 = Marshal.ReadInt32(uidAddr, 0x08);

                string result = uid0.ToString("X8")
                                + "-" + (uid1 >> 16).ToString("X4")
                                + "-" + (uid1 & 0xFFFF).ToString("X4")
                                + "-" + "0000"
                                + "-" + uid2.ToString("X8") + "0000";

                return result;
            }

        }

        //addedd 
        //Watchdog IWDG w TinyCLR OS
        //http://kodfilemon.blogspot.com/2017/07/watchdog-iwdg-w-tinyclr-os.html 
        //
        public class WatchDog
        {
            public static bool LastReboot
            {
                get
                {
                    var rccAddr = new IntPtr(0x40023800);
                    int rccCsrValue = Marshal.ReadInt32(rccAddr, 0x74);
                    return IsIwdgRstf(rccCsrValue);
                }
            }

            public static void Start(TimeSpan period)
            {
                ResetLastReboot();
                SetTimings(period);
                WriteIwdgKr(0xCCCC);
            }

            public static void Reset()
            {
                WriteIwdgKr(0xAAAA);
            }

            private static void ResetLastReboot()
            {
                var rccAddr = new IntPtr(0x40023800);
                int rccCsrValue = Marshal.ReadInt32(rccAddr, 0x74);

                if (IsIwdgRstf(rccCsrValue))
                {
                    const int rmvfMask = 0x01000000;
                    rccCsrValue = rccCsrValue | rmvfMask;
                    Marshal.WriteInt32(rccAddr, 0x74, rccCsrValue);
                }
            }

            private static void WriteIwdgKr(int value)
            {
                Marshal.WriteInt32(new IntPtr(0x40003000), value);
            }

            private static bool IsIwdgRstf(int rccCsrValue)
            {
                const int iwdgRstfMask = 0x20000000;
                return (rccCsrValue & iwdgRstfMask) > 0;
            }

            private static void SetTimings(TimeSpan period)
            {
                const int kHzLsi = 32000;

                long usPeriod = ((period.Ticks * 1000) / TimeSpan.TicksPerMillisecond);
                int[] dividers = { 4, 8, 16, 32, 64, 128, 256 };
                for (int i = 0; i < dividers.Length; i++)
                {
                    int usMin = (dividers[i] * 1000 * 1000) / kHzLsi;
                    if (usPeriod >= usMin)
                    {
                        int counter = (int)(usPeriod / usMin - 1);
                        if (counter < 0 || counter > 0xFFF)
                            continue;

                        SetIwdgPrAndRlr(i, counter);
                        return;
                    }
                }

                throw new InvalidOperationException("Invalid period (0.125..32768 ms).");
            }

            private static void SetIwdgPrAndRlr(int prValue, int rlrValue)
            {
                var iwdgKrAddr = new IntPtr(0x40003000);
                Marshal.WriteInt32(iwdgKrAddr, 0x5555);
                Marshal.WriteInt32(iwdgKrAddr, 0x04, prValue);
                Marshal.WriteInt32(iwdgKrAddr, 0x08, rlrValue);
            }

        }
    }
}
