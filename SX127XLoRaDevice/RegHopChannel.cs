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

	[Flags]
	internal enum RegHopChannelFlags : byte
	{
		PllTimeout = 0b10000000,
		CrcOnPayload = 0b01000000,
	}

	internal enum RegHopChannelMask : byte
	{
		PllTimeout = 0b10000000,
		CrcOnPayload = 0b01000000,
		FhssPresentChannel = 0b01111111,
	}
}