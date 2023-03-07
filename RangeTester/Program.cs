//---------------------------------------------------------------------------------
// Copyright (c) March 2022, devMobile Software
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
#define ESP32_WROOM_32_LORA_1_CHANNEL   // nanoff --platform ESP32 --serialport COM7 --update
//#define ST_STM32F769I_DISCOVERY      // nanoff --target ST_STM32F769I_DISCOVERY --update 
#define ARDUINO_LORA_DUPLEX
namespace devMobile.IoT.SX127x.RangeTester
{
	using System;
	using System.Text;
	using System.Threading;

	using System.Device.Gpio;
	using System.Device.Spi;

	using devMobile.IoT.SX127xLoRaDevice;

#if ESP32_WROOM_32_LORA_1_CHANNEL
	using nanoFramework.Hardware.Esp32 ;
#endif

	class Program
	{
		private const double Frequency = 915000000.0;
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const int SpiBusId = 1;
      private const int ledPinNumber = Gpio.IO17;     // May 2022 LED won't flash _gpioController.Read always returns false see https://github.com/nanoframework/Home/issues/1036
#endif
#if NETDUINO3_WIFI
		private const int SpiBusId = 2;
		private static int ledPinNumber = PinNumber('A', 10);
#endif
#if ST_STM32F769I_DISCOVERY
		private const int SpiBusId = 2;
		private static int ledPinNumber = PinNumber('J', 5);
#endif
		private static SX127XDevice _sx127XDevice;
		private static GpioController _gpioController;

#if ARDUINO_LORA_DUPLEX
		private const byte MessageHeaderLength = 4;
		private const byte MessageHeaderAddressDestinationByte = 0;
		private const byte MessageHeaderAddressSourceByte = 1;
		private const byte MessageHeaderCountByte = 2;
		private const byte MessageHeaderLengthByte = 3;

		private const byte AddressDestinationBroadcast = 0xff;

		//private const byte AddressDestination = 0xaa;
		private const byte AddressDestination = 0xdd;

        private const byte AddressLocal = 0x10;
#endif

        static void Main(string[] args)
		{
			byte SendCount = 0;
#if ESP32_WROOM_32_LORA_1_CHANNEL // No reset line for this device as it isn't connected on SX127X
			int chipSelectLine = Gpio.IO16;
			int dio0PinNumber = Gpio.IO26;
#endif
#if NETDUINO3_WIFI
			// Arduino D10->PB10
			int chipSelectLine = PinNumber('B', 10);
			// Arduino D9->PE5
			int resetPinNumber = PinNumber('E', 5);
			// Arduino D2 -PA3
			int dio0PinNumber = PinNumber('A', 3);
#endif
#if ST_STM32F769I_DISCOVERY
			// Arduino D10->PA11
			int chipSelectLine = PinNumber('A', 11);
			// Arduino D9->PH6
			int resetPinNumber = PinNumber('H', 6);
			// Arduino D2->PA4
			int dio0PinNumber = PinNumber('J', 1);
#endif
			Console.WriteLine("devMobile.IoT.SX127xLoRaDevice Range Tester starting");

			try
			{
#if ESP32_WROOM_32_LORA_1_CHANNEL
				Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
				Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
				Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);
#endif
				var settings = new SpiConnectionSettings(SpiBusId, chipSelectLine)
				{
					ClockFrequency = 1000000,
					Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
					//SharingMode = SpiSharingMode.Shared
				};

				using (_gpioController = new GpioController())
				using (SpiDevice spiDevice = new SpiDevice(settings))
				{

#if ESP32_WROOM_32_LORA_1_CHANNEL
					_sx127XDevice = new SX127XDevice(spiDevice, _gpioController, dio0Pin: dio0PinNumber);
#endif

#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
					_sx127XDevice = new SX127XDevice(spiDevice, _gpioController, dio0Pin: dio0PinNumber, resetPin:resetPinNumber);
#endif

					_sx127XDevice.Initialise(
								Frequency,
								lnaGain: RegLnaLnaGain.G3,
								lnaBoost: true,
								powerAmplifier: RegPAConfigPASelect.PABoost,
								ocpOn: RegOcp.On,
								ocpTrim: RegOcpTrim._150mA,
								rxPayloadCrcOn: true,
								rxDoneignoreIfCrcMissing: false
								);

					_gpioController.OpenPin(ledPinNumber, PinMode.Output);

#if DEBUG
					_sx127XDevice.RegisterDump();
#endif

					_sx127XDevice.OnReceive += SX127XDevice_OnReceive;
					_sx127XDevice.Receive();
					_sx127XDevice.OnTransmit += SX127XDevice_OnTransmit;

					while (true)
					{
#if ARDUINO_LORA_DUPLEX
						string messageText = $"Hello LoRa from .NET nanoFramework {SendCount += 1}!";

						byte[] messageBytes = new byte[messageText.Length + MessageHeaderLength];
							
						UTF8Encoding.UTF8.GetBytes(messageText,0,messageText.Length, messageBytes, MessageHeaderLength);

						messageBytes[MessageHeaderAddressDestinationByte] = AddressDestination;
						messageBytes[MessageHeaderAddressSourceByte] = AddressLocal;
						messageBytes[MessageHeaderCountByte] = SendCount;
						messageBytes[MessageHeaderLengthByte] = (byte)(messageBytes.Length - MessageHeaderLength);
						Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-TX {messageBytes.Length} byte message {messageText}");
#else
						string messageText = $"Hello LoRa from .NET nanoFramework {SendCount += 1}!";

						byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
						Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-TX {messageBytes.Length} byte message {messageText}");
#endif
						_sx127XDevice.Send(messageBytes);

						Thread.Sleep(5000);
					}
				}
			}
			catch (Exception ex)
			{
				Console.WriteLine(ex.Message);
			}
		}

		private static void SX127XDevice_OnReceive(object sender, SX127XDevice.OnDataReceivedEventArgs e)
		{
			try
			{
#if ARDUINO_LORA_DUPLEX
				if (e.Data.Length < MessageHeaderLength)
				{
					Console.WriteLine("Message length invalid");
					return;
				}

				if ((e.Data[MessageHeaderAddressDestinationByte] != AddressLocal) && (e.Data[MessageHeaderAddressDestinationByte] != AddressDestinationBroadcast))
				{
					Console.WriteLine("Message address invalid");
					return;
				}
#endif

				if (_gpioController.Read(ledPinNumber) != PinValue.High)
				{
					_gpioController.Write(ledPinNumber, PinValue.High);
				}
				else
				{
					_gpioController.Write(ledPinNumber, PinValue.Low);
				}
			}
			catch( Exception ex)
			{
				Console.WriteLine(ex.Message);
			}

			try
			{
#if ARDUINO_LORA_DUPLEX
				for (int index = MessageHeaderLength; index < e.Data.Length; index++)
				{
					if ((e.Data[index] < 0x20) || (e.Data[index] > 0x7E))
					{
						e.Data[index] = 0x7C;
					}
				}

				string messageText = UTF8Encoding.UTF8.GetString(e.Data, MessageHeaderLength, e.Data.Length - MessageHeaderLength);

				Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-RX PacketSnr {e.PacketSnr:0.0} Packet RSSI {e.PacketRssi}dBm RSSI {e.Rssi}dBm = To 0x{e.Data[MessageHeaderAddressDestinationByte]:x} From 0x{e.Data[MessageHeaderAddressSourceByte]:x} Count {e.Data[MessageHeaderCountByte]} {e.Data[MessageHeaderLengthByte]} byte message {messageText}");
#else
				for (int index = 0; index < e.Data.Length; index++)
				{
					if ((e.Data[index] < 0x20) || (e.Data[index] > 0x7E))
					{
						e.Data[index] = 0x7C;
					}
				}

				string messageText = UTF8Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);

				Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-RX PacketSnr {e.PacketSnr:0.0} Packet RSSI {e.PacketRssi}dBm RSSI {e.Rssi}dBm = {e.Data.Length} byte message {messageText}");
#endif

			}
			catch (Exception ex)
			{
				Console.WriteLine(ex.Message);
			}
		}

		private static void SX127XDevice_OnTransmit(object sender, SX127XDevice.OnDataTransmitedEventArgs e)
		{
			_sx127XDevice.Receive();

			Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-TX Done");
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
