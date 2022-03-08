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
#define NETDUINO3_WIFI   // nanoff --target NETDUINO3_WIFI --update
//#define ESP32_WROOM_32_LORA_1_CHANNEL   // nanoff --target ESP32_PSRAM_REV0 --serialport COM7 --update
//#define ST_STM32F769I_DISCOVERY      // nanoff --target ST_STM32F769I_DISCOVERY --update 
namespace devMobile.IoT.SX127x.RegisterScan
{
   using System;
   using System.Diagnostics;
   using System.Threading;

   using System.Device.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public sealed class SX127xDevice
   {
      private readonly SpiDevice sx127xLoraModem;

      public SX127xDevice(int spiPort, int chipSelectPin)
      {

         var settings = new SpiConnectionSettings(spiPort, chipSelectPin)
         {
            ClockFrequency = 500000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared
         };

         sx127xLoraModem = new SpiDevice(settings);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

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
#if ESP32_WROOM_32_LORA_1_CHANNEL // No reset line for this device as it isn't connected on SX127X
         int chipSelectPinNumber = Gpio.IO16;
#endif
#if NETDUINO3_WIFI
         // Arduino D10->PB10
         int chipSelectPinNumber = PinNumber('B', 10);
         // Arduino D9->PE5
         int resetPinNumber = PinNumber('E', 5);
#endif
#if ST_STM32F769I_DISCOVERY
         // Arduino D10->PA11
         int chipSelectPinNumber = PinNumber('A', 11);
         // Arduino D9->PH6
         int resetPinNumber = PinNumber('H', 6);
#endif
         Debug.WriteLine("devMobile.IoT.SX127x.RegisterScan starting");

         try
         {
#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO14, DeviceFunction.SPI1_CLOCK);
#endif
            SX127xDevice rfm9XDevice = new SX127xDevice(SpiBusId, chipSelectPinNumber);

            Thread.Sleep(500);

            while (true)
            {
               for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
               {
                  byte registerValue = rfm9XDevice.RegisterReadByte(registerIndex);

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

#if ST_STM32F429I_DISCOVERY ||NETDUINO3_WIFI || ST_NUCLEO144_F746ZG
      static int PinNumber(char port, byte pin)
      {
         if (port < 'A' || port > 'J')
            throw new ArgumentException();

         return ((port - 'A') * 16) + pin;
      }
#endif
   }
}
