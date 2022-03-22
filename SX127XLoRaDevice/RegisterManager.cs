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
namespace devMobile.IoT.SX127xLoRaDevice
{
	using System;

	using System.Device.Spi;

	public sealed class RegisterManager
	{
		private readonly byte _registerAddressReadMask = 0X7f;
		private readonly byte _registerAddressWriteMask = 0x80;

		private readonly SpiDevice _spiDevice = null;

		public RegisterManager(SpiDevice spiDevice, byte registerAddressReadMask, byte registerAddressWriteMask)
		{
			_spiDevice = spiDevice;

			_registerAddressReadMask = registerAddressReadMask;
			_registerAddressWriteMask = registerAddressWriteMask;
		}

		public Byte ReadByte(byte registerAddress)
		{
			byte[] writeBuffer = new byte[] { registerAddress &= _registerAddressReadMask, 0x0 };
			byte[] readBuffer = new byte[writeBuffer.Length];

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

			return readBuffer[1];
		}

		public ushort ReadWord(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= _registerAddressReadMask, 0x0, 0x0 };
			byte[] readBuffer = new byte[writeBuffer.Length];

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

			return (ushort)(readBuffer[2] + (readBuffer[1] << 8));
		}

		public ushort ReadWordMsbLsb(byte address)
		{
			byte[] writeBuffer = new byte[] { address &= _registerAddressReadMask, 0x0, 0x0 };
			byte[] readBuffer = new byte[writeBuffer.Length];

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

			return (ushort)((readBuffer[1] << 8) + readBuffer[2]);
		}

		public byte[] ReadBytes(byte address, byte length)
		{
			byte[] writeBuffer = new byte[length + 1];
			byte[] readBuffer = new byte[writeBuffer.Length];
			byte[] replyBuffer = new byte[length];

			writeBuffer[0] = address &= _registerAddressReadMask;

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

			Array.Copy(readBuffer, 1, replyBuffer, 0, length);

			return replyBuffer;
		}

		public void WriteByte(byte address, byte value)
		{
			byte[] writeBuffer = new byte[] { address |= _registerAddressWriteMask, value };
			byte[] readBuffer = new byte[writeBuffer.Length];

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);
		}

		public void WriteWord(byte address, ushort value)
		{
			byte[] valueBytes = BitConverter.GetBytes(value);
			byte[] writeBuffer = new byte[] { address |= _registerAddressWriteMask, valueBytes[0], valueBytes[1] };
			byte[] readBuffer = new byte[writeBuffer.Length];

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);
		}

		public void WriteWordMsbLsb(byte address, ushort value)
		{
			byte[] valueBytes = BitConverter.GetBytes(value);
			byte[] writeBuffer = new byte[] { address |= _registerAddressWriteMask, valueBytes[1], valueBytes[0] };
			byte[] readBuffer = new byte[writeBuffer.Length];

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);
		}

		public void WriteBytes(byte address, byte[] bytes)
		{
			byte[] writeBuffer = new byte[1 + bytes.Length];
			byte[] readBuffer = new byte[writeBuffer.Length];

			Array.Copy(bytes, 0, writeBuffer, 1, bytes.Length);
			writeBuffer[0] = address |= _registerAddressWriteMask;

			_spiDevice.TransferFullDuplex(writeBuffer, readBuffer);
		}
	}
}
