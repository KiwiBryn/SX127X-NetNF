//---------------------------------------------------------------------------------
// Copyright (c) April 2020, March 2022 devMobile Software
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
//
//---------------------------------------------------------------------------------
//#define NETDUINO3_WIFI   // nanoff --target NETDUINO3_WIFI --update
#define ESP32_WROOM_32_LORA_1_CHANNEL   // nanoff --target ESP32_PSRAM_REV0 --serialport COM7 --update
//#define ST_STM32F769I_DISCOVERY      // nanoff --target ST_STM32F769I_DISCOVERY --update 

namespace devMobile.IoT.SX127x.ShieldSPI
{
   using System;
   using System.Diagnostics;
   using System.Threading;

   using System.Device.Gpio;
   using System.Device.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public class Program
   {
      private const byte RegVersion = 0x42;
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const int SpiBusId = 1;
#endif
#if NETDUINO3_WIFI
      private const int SpiBusId = 2;
#endif
#if ST_STM32F769I_DISCOVERY
      private const int SpiBusId = 2;
#endif

      public static void Main()
      {
         GpioController gpioController = new GpioController();

#if ESP32_WROOM_32_LORA_1_CHANNEL // No reset line for this device as it isn't connected on SX127X
         int ledPinNumber = Gpio.IO17;
         int chipSelectLine = Gpio.IO16;
#endif
#if NETDUINO3_WIFI
         int ledPinNumber = PinNumber('A', 10);
         // Arduino D10->PB10
         int chipSelectLine = PinNumber('B', 10);
         // Arduino D9->PE5
         int resetPinNumber = PinNumber('E', 5);
#endif
#if ST_STM32F769I_DISCOVERY
         int ledPinNumber  = PinNumber('J', 5);
         // Arduino D10->PA11
         int chipSelectLine = PinNumber('A', 11);
         // Arduino D9->PH6
         int resetPinNumber = PinNumber('H', 6);
#endif
         Debug.WriteLine("devMobile.IoT.SX127x.ShieldSPI starting");

         try
         {
#if ESP32_WROOM_32_LORA_1_CHANNEL || NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
            // Setup the onboard LED
            gpioController.OpenPin(ledPinNumber, PinMode.Output);
#endif

#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
            // Setup the reset pin
            gpioController.OpenPin(resetPinNumber, PinMode.Output);
            gpioController.Write(resetPinNumber, PinValue.High);
#endif

#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);
#endif

            var settings = new SpiConnectionSettings(SpiBusId, chipSelectLine)
            {
               ClockFrequency = 1000000,
               Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
               //SharingMode = SpiSharingMode.Shared,
            };

            using (SpiDevice device = SpiDevice.Create(settings))
            {
               Thread.Sleep(500);

               while (true)
               {
                  byte[] writeBuffer = new byte[] { RegVersion, 0x0 };
                  byte[] readBuffer = new byte[writeBuffer.Length];

                  device.TransferFullDuplex(writeBuffer, readBuffer);

                  Debug.WriteLine(String.Format("Register 0x{0:x2} - Value 0X{1:x2}", RegVersion, readBuffer[1]));

#if ESP32_WROOM_32_LORA_1_CHANNEL || NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
                  if ( gpioController.Read(ledPinNumber) == PinValue.High)
						{
                     gpioController.Write(ledPinNumber, PinValue.Low);
                  }
                  else
						{
                     gpioController.Write(ledPinNumber, PinValue.High);
                  }
#endif
                  Thread.Sleep(10000);
               }
            }
         }
         catch (Exception ex)
         {
            Debug.WriteLine(ex.Message);
         }
      }

#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
      static int PinNumber(char port, byte pin)
      {
         if (port < 'A' || port > 'J')
            throw new ArgumentException();

         return ((port - 'A') * 16) + pin;
      }
#endif
   }
}
