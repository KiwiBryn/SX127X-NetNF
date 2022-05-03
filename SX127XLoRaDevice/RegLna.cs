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

	// RegLna settings from Semtech SX127X Datasheet
	[Flags]
	public enum RegLnaLnaGain : byte
	{
		G1 = 0b00100000,
		Default = G1,
		G2 = 0b01000000,
		G3 = 0b01100000,
		G4 = 0b10000000,
		G5 = 0b10100000,
		G6 = 0b11000000
	}
	// TODO : Fix lnaBoost default as there must be a better way of doing this
	// public const bool LnaBoostDefault = false; 

	[Flags]
	internal enum RegLnaLnaBoost : byte
	{
		LfOn = 0b00011000,
		LfOff = 0b00000000,
		LfDefault = LfOff,
		HfOn = 0b00000011,
		HfOff = 0b00000000,
		HfDefault = HfOff
	}
}