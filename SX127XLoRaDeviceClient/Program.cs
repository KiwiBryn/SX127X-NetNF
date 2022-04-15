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
#define NETDUINO3_WIFI   // nanoff --target NETDUINO3_WIFI --update
//#define ESP32_WROOM_32_LORA_1_CHANNEL   // nanoff --target ESP32_PSRAM_REV0 --serialport COM7 --update
//#define ST_STM32F769I_DISCOVERY      // nanoff --target ST_STM32F769I_DISCOVERY --update 
namespace devMobile.IoT.SX127xLoRaDevice
{
	using System;
	using System.Text;
	using System.Threading;

	using System.Device.Gpio;
	using System.Device.Spi;

	class Program
	{
		private const double Frequency = 915000000.0;
#if ESP32_WROOM_32_LORA_1_CHANNEL
      private const int SpiBusId = 1;
#endif
#if NETDUINO3_WIFI
		private const int SpiBusId = 2;
#endif
#if ST_STM32F769I_DISCOVERY
		private const int SpiBusId = 2;
#endif
		private static SX127XDevice sx127XDevice;

		static void Main(string[] args)
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
			Console.WriteLine("devMobile.IoT.SX127xLoRaDevice Client starting");

			try
			{
				var settings = new SpiConnectionSettings(SpiBusId, chipSelectLine)
				{
					ClockFrequency = 1000000,
					Mode = SpiMode.Mode0,// From SemTech docs pg 80 CPOL=0, CPHA=0
					SharingMode = SpiSharingMode.Shared
				};

				using (SpiDevice spiDevice = new SpiDevice(settings))
				using (GpioController gpioController = new GpioController())
				{
#if ESP32_WROOM_32_LORA_1_CHANNEL
					Configuration.SetPinFunction(Gpio.IO12, DeviceFunction.SPI1_MISO);
					Configuration.SetPinFunction(Gpio.IO13, DeviceFunction.SPI1_MOSI);
					Configuration.SetPinFunction(Gpio.IO14, DeviceFunction.SPI1_CLOCK);

					sx127XDevice = new SX127XDevice(spiDevice, gpioController, interruptPinNumber);
#endif
#if NETDUINO3_WIFI || ST_STM32F769I_DISCOVERY
					sx127XDevice = new SX127XDevice(spiDevice, gpioController, interruptPinNumber, resetPinNumber);
#endif

					sx127XDevice.Initialise(
								Frequency,
								lnaGain: Configuration.RegLnaLnaGain.G3,
								lnaBoost: true,
								powerAmplifier: Configuration.RegPAConfigPASelect.PABoost,
								rxPayloadCrcOn: true,
								rxDoneignoreIfCrcMissing: false
								);

#if DEBUG
					sx127XDevice.RegisterDump();
#endif

					sx127XDevice.OnReceive += SX127XDevice_OnReceive;
					sx127XDevice.Receive();
					sx127XDevice.OnTransmit += SX127XDevice_OnTransmit;

					Thread.Sleep(500);

					while (true)
					{
						string messageText = $"Hello LoRa from .NET nanoFramework {SendCount += 1}!";

						byte[] messageBytes = UTF8Encoding.UTF8.GetBytes(messageText);
						Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-TX {messageBytes.Length} byte message {messageText}");
						sx127XDevice.Send(messageBytes);

						Thread.Sleep(50000);
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
				// Remove unprintable characters from messages
				for (int index = 0; index < e.Data.Length; index++)
				{
					if ((e.Data[index] < 0x20) || (e.Data[index] > 0x7E))
					{
						e.Data[index] = 0x7C;
					}
				}

				string messageText = UTF8Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);

				Console.WriteLine($"{DateTime.UtcNow:HH:mm:ss}-RX PacketSnr {e.PacketSnr:0.0} Packet RSSI {e.PacketRssi}dBm RSSI {e.Rssi}dBm = {e.Data.Length} byte message {messageText}");
			}
			catch (Exception ex)
			{
				Console.WriteLine(ex.Message);
			}
		}

		private static void SX127XDevice_OnTransmit(object sender, SX127XDevice.OnDataTransmitedEventArgs e)
		{
			sx127XDevice.SetMode(Configuration.RegOpModeMode.ReceiveContinuous);

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
