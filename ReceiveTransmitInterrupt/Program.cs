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
namespace devMobile.IoT.SX127x.ReceiveTransmitInterrupt
{
   using System;
   using System.Diagnostics;
   using System.Text;
   using System.Threading;

   using System.Device.Gpio;
   using System.Device.Spi;

#if ESP32_WROOM_32_LORA_1_CHANNEL
   using nanoFramework.Hardware.Esp32;
#endif

   public sealed class SX127XDevice
   {
      private const byte RegisterAddressMinimum = 0X0;
      private const byte RegisterAddressMaximum = 0x42;
      private const byte RegisterAddressReadMask = 0X7f;
      private const byte RegisterAddressWriteMask = 0x80;

      private readonly SpiDevice SX127XTransceiver;
      private readonly GpioController gpioController;

      public SX127XDevice(int busId, int chipSelectLine, int interruptPin, int resetPin)
      {
         var settings = new SpiConnectionSettings(busId, chipSelectLine)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            //SharingMode = SpiSharingMode.Shared
         };

         SX127XTransceiver = new SpiDevice(settings);

         GpioController gpioController = new GpioController();


         // Factory reset pin configuration
         gpioController.OpenPin(resetPin, PinMode.Output);

         gpioController.Write(resetPin, PinValue.Low);
         Thread.Sleep(20);
         gpioController.Write(resetPin, PinValue.High);
         Thread.Sleep(20);

         // Interrupt pin for RX message & TX done notification 
         gpioController.OpenPin(interruptPin, PinMode.InputPullDown);

         gpioController.RegisterCallbackForPinValueChangedEvent(interruptPin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
      }

      public SX127XDevice(int busId, int chipSelectLine, int interruptPin)
      {
         var settings = new SpiConnectionSettings(busId, chipSelectLine)
         {
            ClockFrequency = 1000000,
            Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
            //SharingMode = SpiSharingMode.Shared,
         };

         SX127XTransceiver = new SpiDevice(settings);

         gpioController = new GpioController();

         // Interrupt pin for RX message & TX done notification 
         gpioController.OpenPin(interruptPin, PinMode.InputPullDown);

         gpioController.RegisterCallbackForPinValueChangedEvent(interruptPin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
      }

      private void InterruptGpioPin_ValueChanged(object sender, PinValueChangedEventArgs e)
      {
         byte irqFlags = this.ReadByte(0x12); // RegIrqFlags
         Debug.WriteLine($"RegIrqFlags 0X{irqFlags:x2}");

         if ((irqFlags & 0b01000000) == 0b01000000)  // RxDone 
         {
            Debug.WriteLine("Receive-Message");
            byte currentFifoAddress = this.ReadByte(0x10); // RegFifiRxCurrent
            this.WriteByte(0x0d, currentFifoAddress); // RegFifoAddrPtr

            byte numberOfBytes = this.ReadByte(0x13); // RegRxNbBytes

            // Allocate buffer for message
            byte[] messageBytes = this.ReadBytes(0X0, numberOfBytes);

            // Remove unprintable characters from messages
            for (int index = 0; index < messageBytes.Length; index++)
            {
               if ((messageBytes[index] < 0x20) || (messageBytes[index] > 0x7E))
               {
                  messageBytes[index] = 0x20;
               }
            }

            string messageText = UTF8Encoding.UTF8.GetString(messageBytes, 0, messageBytes.Length);
            Debug.WriteLine($"Received {messageBytes.Length} byte message {messageText}");
         }

         if ((irqFlags & 0b00001000) == 0b00001000)  // TxDone
         {
            this.WriteByte(0x01, 0b10000101); // RegOpMode set LoRa & RxContinuous
            Debug.WriteLine("Transmit-Done");
         }

         this.WriteByte(0x40, 0b00000000); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady
         this.WriteByte(0x12, 0xff);// RegIrqFlags
      }

      public Byte ReadByte(byte registerAddress)
      {
         byte[] writeBuffer = new byte[] { registerAddress &= RegisterAddressReadMask, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

         return readBuffer[1];
      }

      public ushort ReadWord(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask, 0x0, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

         return (ushort)(readBuffer[2] + (readBuffer[1] << 8));
      }

      public ushort ReadWordMsbLsb(byte address)
      {
         byte[] writeBuffer = new byte[] { address &= RegisterAddressReadMask, 0x0, 0x0 };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

         return (ushort)((readBuffer[1] << 8) + readBuffer[2]);
      }

      public byte[] ReadBytes(byte address, byte length)
      {
         byte[] writeBuffer = new byte[length + 1];
         byte[] readBuffer = new byte[writeBuffer.Length];
         byte[] replyBuffer = new byte[length];

         writeBuffer[0] = address &= RegisterAddressReadMask;

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);

         Array.Copy(readBuffer, 1, replyBuffer, 0, length);

         return replyBuffer;
      }

      public void WriteByte(byte address, byte value)
      {
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, value };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void WriteWord(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[0], valueBytes[1] };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void WriteWordMsbLsb(byte address, ushort value)
      {
         byte[] valueBytes = BitConverter.GetBytes(value);
         byte[] writeBuffer = new byte[] { address |= RegisterAddressWriteMask, valueBytes[1], valueBytes[0] };
         byte[] readBuffer = new byte[writeBuffer.Length];

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void WriteBytes(byte address, byte[] bytes)
      {
         byte[] writeBuffer = new byte[1 + bytes.Length];
         byte[] readBuffer = new byte[writeBuffer.Length];

         Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
         writeBuffer[0] = address |= RegisterAddressWriteMask;

         SX127XTransceiver.TransferFullDuplex(writeBuffer, readBuffer);
      }

      public void RegisterDump()
      {
         Debug.WriteLine("Register dump");
         for (byte registerIndex = RegisterAddressMinimum; registerIndex <= RegisterAddressMaximum; registerIndex++)
         {
            byte registerValue = this.ReadByte(registerIndex);

            Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
         }

         Debug.WriteLine("");
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
         int SendCount = 0;
#if ESP32_WROOM_32_LORA_1_CHANNEL // No reset line for this device as it isn't connected on SX127X
         int chipSelectLine = Gpio.IO16;
         int interruptPinNumber = Gpio.IO26;
#endif
#if NETDUINO3_WIFI
         // Arduino D10->PB10
         int chipSelectLine = PinNumber('B', 10);
         // Arduino D9->PE5
         int resetPinNumber = PinNumber('E', 5);
         // Arduino D2 -PA3
         int interruptPinNumber = PinNumber('A', 3);
#endif
#if ST_STM32F769I_DISCOVERY
         // Arduino D10->PA11
         int chipSelectLine = PinNumber('A', 11);
         // Arduino D9->PH6
         int resetPinNumber = PinNumber('H', 6);
         // Arduino D2->PA4
         int interruptPinNumber = PinNumber('J', 1);
#endif

         Debug.WriteLine("devMobile.IoT.SX127x.ReceiveTransmitInterrupt starting");

         try
         {
#if ESP32_WROOM_32_LORA_1_CHANNEL
            Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
            Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
            Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);

            SX127XDevice sx127XDevice = new SX127XDevice(SpiBusId, chipSelectLine, interruptPinNumber);
#endif
#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
            SX127XDevice sx127XDevice = new SX127XDevice(SpiBusId, chipSelectLine, interruptPinNumber, resetPinNumber);
#endif
            Thread.Sleep(500);

            // Put device into LoRa + Sleep mode
            sx127XDevice.WriteByte(0x01, 0b10000000); // RegOpMode 

            // Set the frequency to 915MHz
            byte[] frequencyWriteBytes = { 0xE4, 0xC0, 0x00 }; // RegFrMsb, RegFrMid, RegFrLsb
            sx127XDevice.WriteBytes(0x06, frequencyWriteBytes);

            // More power PA Boost
            sx127XDevice.WriteByte(0x09, 0b10000000); // RegPaConfig

            sx127XDevice.WriteByte(0x01, 0b10000101); // RegOpMode set LoRa & RxContinuous

            while (true)
            {
               // Set the Register Fifo address pointer
               sx127XDevice.WriteByte(0x0E, 0x00); // RegFifoTxBaseAddress 

               // Set the Register Fifo address pointer
               sx127XDevice.WriteByte(0x0D, 0x0); // RegFifoAddrPtr 

               string messageText = $"Hello LoRa {SendCount += 1}!";

               // load the message into the fifo
               byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
               sx127XDevice.WriteBytes(0x0, messageBytes); // RegFifo 

               // Set the length of the message in the fifo
               sx127XDevice.WriteByte(0x22, (byte)messageBytes.Length); // RegPayloadLength
               sx127XDevice.WriteByte(0x40, 0b01000000); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady
               sx127XDevice.WriteByte(0x01, 0b10000011); // RegOpMode 

               Debug.WriteLine($"Sending {messageBytes.Length} bytes message {messageText}");

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
