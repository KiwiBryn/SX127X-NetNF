//---------------------------------------------------------------------------------
// Copyright (c) May 2022, devMobile Software
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
	// RegInvertIq settings from Semtech SX127X Datasheet
	public enum InvertIqRx : byte
	{
		On = 0b01000000,
		Off = 0b00000000,
		Default = Off
	}

	public enum InvertIqTx : byte
	{
		On = 0b00000001,
		Default = On,
		Off = 0b00000000
	}

	internal enum RegInvertIq2
	{
		On = 0x19,
		Off = 0x1D,
		Default = Off
	}
}