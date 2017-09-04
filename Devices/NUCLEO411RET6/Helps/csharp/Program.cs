using System.Diagnostics;
using System.Threading;
using GHIElectronics.TinyCLR.Devices.Gpio;

namespace BlinkTinyCLR
{
    class Program
    {
        public static void Main()
        {
            var controller = GpioController.GetDefault();
            var pin = controller.OpenPin(STM.NUCLEO.STM32F411.Arduino.Led1);                        
            pin.SetDriveMode(GpioPinDriveMode.Output);
     
            while (true)
            {
                pin.Write(GpioPinValue.High);
                Thread.Sleep(500);

                Debug.WriteLine("Led on ..");

                pin.Write(GpioPinValue.Low);
                Thread.Sleep(500);

                Debug.WriteLine("Led off..");

            }
        }
     }
}
