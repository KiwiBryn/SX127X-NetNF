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

	// RegOpMode bit flags from Semtech SX127X Datasheet
	[Flags]
	internal enum RegOpModeModeFlags : byte
	{
		LongRangeModeLoRa = 0b10000000,
		LongRangeModeFskOok = 0b00000000,
		LongRangeModeDefault = LongRangeModeFskOok,
		AcessSharedRegLoRa = 0b00000000,
		AcessSharedRegFsk = 0b01000000,
		AcessSharedRegDefault = AcessSharedRegLoRa,
		LowFrequencyModeOnHighFrequency = 0b00000000,
		LowFrequencyModeOnLowFrequency = 0b00001000,
		LowFrequencyModeOnDefault = LowFrequencyModeOnLowFrequency
	}

	internal enum RegOpModeMode : byte
	{
		Sleep = 0b00000000,
		StandBy = 0b00000001,
		FrequencySynthesisTX = 0b00000010,
		Transmit = 0b00000011,
		FrequencySynthesisRX = 0b00000100,
		ReceiveContinuous = 0b00000101,
		ReceiveSingle = 0b00000110,
		ChannelActivityDetection = 0b00000111,
	};
}