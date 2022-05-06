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

	// RegIrqFlags settings from Semtech SX127X Datasheet
	[Flags]
	internal enum RegIrqFlagsMask : byte
	{
		RxTimeoutMask = 0b10000000,
		RxDoneMask = 0b01000000,
		PayLoadCrcErrorMask = 0b00100000,
		ValidHeaderMask = 0b00010000,
		TxDoneMask = 0b00001000,
		CadDoneMask = 0b00000100,
		FhssChangeChannelMask = 0b00000010,
		CadDetectedMask = 0b00000001,
	}

	[Flags]
	internal enum RegIrqFlags : byte
	{
		ClearNone = 0b00000000,
		RxTimeout = 0b10000000,
		RxDone = 0b01000000,
		PayLoadCrcError = 0b00100000,
		ValidHeader = 0b00010000,
		TxDone = 0b00001000,
		CadDone = 0b00000100,
		FhssChangeChannel = 0b00000010,
		CadDetected = 0b00000001,
	}
}
