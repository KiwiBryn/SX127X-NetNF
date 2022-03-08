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
namespace devMobile.IoT.SX127x.RegisterReadAndWrite
{
   using System;
   using System.Diagnostics;
   using System.Threading;

   using System.Device.Gpio;
   using System.Device.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public sealed class SX127xDevice
   {
      private readonly SpiDevice sx127xLoraModem;
      private const byte RegisterAddressReadMask = 0X7f;
      private const byte RegisterAddressWriteMask = 0x80;

      public SX127xDevice(int spiBusId, int chipSelectPin, int resetPin)
      {
         var settings = new SpiConnectionSettings(spiBusId, chipSelectPin)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared
         };

         sx127xLoraModem = new SpiDevice(settings);

         // Factory reset pin configuration
         GpioController gpioController = new GpioController();
         gpioController.OpenPin(resetPin, PinMode.Output);

         gpioController.Write(resetPin, PinValue.Low);
         Thread.Sleep(10);
         gpioController.Write(resetPin, PinValue.High);
         Thread.Sleep(10);
      }

      public SX127xDevice(int spiBusId, int chipSelectPin)
      {
         var settings = new SpiConnectionSettings(spiBusId, chipSelectPin)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            SharingMode = SpiSharingMode.Shared,
         };

         sx127xLoraModem = new SpiDevice(settings);
      }

      public Byte RegisterReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress &= RegisterAddressReadMask, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         return readBuffer[1];
      }

      public ushort RegisterReadWord(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask, 0x0, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         return (ushort)(readBuffer[2] + (readBuffer[1] << 8));
      }

      public byte[] RegisterRead(byte address, int length)
      {
         byte[] writeBuffer = new byte[length + 1];
         byte[] readBuffer = new byte[writeBuffer.Length];
         byte[] replyBuffer = new byte[length];

         writeBuffer[0] = address &= RegisterAddressReadMask;

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);

         Array.Copy(readBuffer, 1, replyBuffer, 0, length);

         return replyBuffer;
      }

      public void RegisterWriteByte(byte address, byte value)
      {
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
         byte[] readBuffer = new byte[writeBuffer.Length];

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void RegisterWriteWord(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
         byte[] readBuffer = new byte[writeBuffer.Length];

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void RegisterWrite(byte address, byte[] bytes)
      {
         byte[] writeBuffer = new byte[1 + bytes.Length];
         byte[] readBuffer = new byte[writeBuffer.Length];

         Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
         writeBuffer[0] = address |= RegisterAddressWriteMask;

         sx127xLoraModem.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void RegisterDump()
      {
         Debug.WriteLine("Register dump");
         for (byte registerIndex = 0; registerIndex <= 0x42; registerIndex++)
         {
            byte registerValue = this.RegisterReadByte(registerIndex);

            Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
         }
      }
   }

   class Program
   {
#if NETDUINO3_WIFI
      private const int SpiBusId = 2;
#endif
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const int SpiBusId = 1;
#endif
#if ST_STM32F769I_DISCOVERY
      private const int SpiBusId = 2;
#endif

      static void Main()
      {
#if ESP32_WROOM_32_LORA_1_CHANNEL
         int chipSelectPinNumber = Gpio.IO16;
#endif
#if NETDUINO3_WIFI
         int chipSelectPinNumber = PinNumber('B', 10);
         int resetPinNumber = PinNumber('E', 5);
#endif
#if ST_STM32F769I_DISCOVERY
         // Arduino D10->PA11
         int chipSelectPinNumber = PinNumber('A', 11);
         // Arduino D9->PH6
         int resetPinNumber = PinNumber('H', 6);
#endif

         Debug.WriteLine("devMobile.IoT.SX127x.RegisterReadAndWrite");

         try
         {
#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(nanoFramework.Hardware.Esp32.Gpio.IO14, DeviceFunction.SPI1_CLOCK);

            SX127xDevice sx127xDevice = new SX127xDevice(SpiBusId, chipSelectPinNumber);
#endif
#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
            SX127xDevice sx127xDevice = new SX127xDevice(SpiBusId, chipSelectPinNumber, resetPinNumber);
#endif
            Thread.Sleep(500);

            sx127xDevice.RegisterDump();

            while (true)
            {
               Debug.WriteLine("Read RegOpMode (read byte)");
               Byte regOpMode1 = sx127xDevice.RegisterReadByte(0x1);
               Debug.WriteLine($"RegOpMode 0x{regOpMode1:x2}");

               Debug.WriteLine("Set LoRa mode and sleep mode (write byte)");
               sx127xDevice.RegisterWriteByte(0x01, 0b10000000);

               Debug.WriteLine("Read RegOpMode (read byte)");
               Byte regOpMode2 = sx127xDevice.RegisterReadByte(0x1);
               Debug.WriteLine($"RegOpMode 0x{regOpMode2:x2}");

               Debug.WriteLine("Read the preamble (read word)");
               ushort preamble = sx127xDevice.RegisterReadWord(0x20);
               Debug.WriteLine($"Preamble 0x{preamble:x2}");

               Debug.WriteLine("Set the preamble to 0x80 (write word)");
               sx127xDevice.RegisterWriteWord(0x20, 0x80);

               Debug.WriteLine("Read the center frequency (read byte array)");
               byte[] frequencyReadBytes = sx127xDevice.RegisterRead(0x06, 3);
               Debug.WriteLine($"Frequency Msb 0x{frequencyReadBytes[0]:x2} Mid 0x{frequencyReadBytes[1]:x2} Lsb 0x{frequencyReadBytes[2]:x2}");

               Debug.WriteLine("Set the center frequency to 915MHz (write byte array)");
               byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 };
               sx127xDevice.RegisterWrite(0x06, frequencyWriteBytes);

               sx127xDevice.RegisterDump();

               Thread.Sleep(30000);
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
