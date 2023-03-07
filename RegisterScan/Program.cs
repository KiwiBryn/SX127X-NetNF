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
namespace devMobile.IoT.SX127x.RegisterScan
{
   using System;
   using System.Diagnostics;
   using System.Threading;

   using System.Device.Gpio;
   using System.Device.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public sealed class SX127XDevice
   {
      private readonly SpiDevice SX127XTransceiver;

      public SX127XDevice(int busId, int chipSelectLine)
      {

         var settings = new SpiConnectionSettings(busId, chipSelectLine)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            //SharingMode = SpiSharingMode.Shared
         };

         SX127XTransceiver = new SpiDevice(settings);
      }

      public SX127XDevice(int busId, int chipSelectLine, int resetPin)
      {
         var settings = new SpiConnectionSettings(busId, chipSelectLine)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            //SharingMode = SpiSharingMode.Shared
         };

         SX127XTransceiver = new SpiDevice(settings);

         // Factory reset pin configuration
         GpioController gpioController = new GpioController();
         gpioController.OpenPin(resetPin, PinMode.Output);

         gpioController.Write(resetPin, PinValue.Low);
         Thread.Sleep(20);
         gpioController.Write(resetPin, PinValue.High);
         Thread.Sleep(20);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

         return readBuffer[1];
      }
   }

   public class Program
   {
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
#if ESP32_WROOM_32_LORA_1_CHANNEL
         int chipSelectLine = Gpio.IO16;
#endif
#if NETDUINO3_WIFI
         // Arduino D10->PB10
         int chipSelectLine = PinNumber('B', 10);
         // Arduino D9->PE5
         int resetPinNumber = PinNumber('E', 5);
#endif
#if ST_STM32F769I_DISCOVERY
         // Arduino D10->PA11
         int chipSelectLine = PinNumber('A', 11);
         // Arduino D9->PH6
         int resetPinNumber = PinNumber('H', 6);
#endif

         Debug.WriteLine("devMobile.IoT.SX127x.RegisterScan starting");

         try
         {
#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
            SX127XDevice sx127XDevice = new SX127XDevice(SpiBusId, chipSelectLine, resetPinNumber);
#endif

#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);

            SX127XDevice sx127XDevice = new SX127XDevice(SpiBusId, chipSelectLine);
#endif

            Thread.Sleep(500);

            while (true)
            {
               for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
               {
                  byte registerValue = sx127XDevice.RegisterReadByte(registerIndex);

                  Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
               }
               Debug.WriteLine("");

               Thread.Sleep(10000);
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
