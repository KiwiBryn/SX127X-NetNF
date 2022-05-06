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

	// RegDioMapping from table 18 for Dio0..Dio3 in LoRaMode
	[Flags]
	internal enum RegDioMapping1
	{
		Dio0None = 0b11000000,
		Dio0RxDone = 0b00000000,
		Dio0TxDone = 0b01000000,
		Dio0CadDone = 0b10000000,
		Dio0Default = 0b00000000,
		Dio0Mask = 0b11000000,

		Dio1None = 0b00110000,
		Dio1RxTimeout = 0b00000000,
		Dio1FhssChangeChannel = 0b00010000,
		Dio1CadDetect = 0b00100000,
		Dio1Default = 0b00000000,
		Dio1Mask = 0b00110000,

		Dio2None = 0b00001100,
		Dio2FhssChangeChannel1 = 0b00000000,
		Dio2FhssChangeChannel2 = 0b00000100,
		Dio2FhssChangeChannel3 = 0b00001000,
		Dio2Default = 0b00000000,
		Dio2Mask = 0b00001100,

		Dio3None = 0b00000011,
		Dio3CadDone = 0b00000000,
		Dio3ValidHeader = 0b00000001,
		Dio3PayloadCrcError = 0b00000010,
		Dio3Default = 0b00000000,
		Dio3Mask = 0b00000011,
	}

	// RegDioMapping2 from table 18 for Dio4..Dio5 in LoRaMode. Not certain what todo about MapPreambleDetect problem for future me
	[Flags]
	internal enum RegDioMapping2
	{
		Dio4None = 0b11000000,
		Dio4CadDetected = 0b00000000,
		Dio4PllLock1 = 0b01000000,
		Dio4PllLock2 = 0b10000000,
		Dio4Default = 0b00000000,
		Di04Mask = 0b11000000,

		Dio5None = 0b00110000,
		Dio5ModeReady = 0b00000000,
		Dio5ClkOut1 = 0b00010000,
		Dio5ClkOut2 = 0b00100000,
		Dio5Default = 0b00000000,
		Dio5Mask = 0b00110000,
	}

}