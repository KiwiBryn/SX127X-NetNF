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
	using System;

	// RegPaConfig + RegPaDac settings from Semtech SX127X Datasheet with complexity hidden from user 
	[Flags]
	public enum RegPAConfigPASelect : byte
	{
		Rfo = 0b00000000,
		PABoost = 0b10000000,
		Default = Rfo
	}

	internal enum RegPAConfigMaxPower : byte
	{
		Min = 0b00000000,
		Max = 0b01110000,
		Default = 0b01000000
	}

	// RegPaDac more power
	[Flags]
	internal enum RegPaDac
	{
		Normal = 0x84,
		Boost = 0x87,
		Default = Normal
	}
}