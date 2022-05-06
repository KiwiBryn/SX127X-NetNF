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
	// Registers from Semtech SX127X Datasheet
	internal enum Registers : byte
	{
		Minimum = RegOpMode,

		RegFifo = 0x0,
		RegOpMode = 0x01,
		//Reserved 0x02-0x05 
		RegFrMsb = 0x06,
		RegFrMid = 0x7,
		RegFrLsb = 0x08,
		RegPAConfig = 0x09,
		//RegPARamp = 0x0A, appears to be for FSK only
		RegOcp = 0x0B,
		RegLna = 0x0C,
		RegFifoAddrPtr = 0x0D,
		RegFifoTxBaseAddr = 0x0E,
		RegFifoRxBaseAddr = 0x0F,
		RegFifoRxCurrent = 0x10,
		RegIrqFlagsMask = 0x11,
		RegIrqFlags = 0x12,
		RegRxNbBytes = 0x13,
		// RegRxHeaderCntValueMsb=0x14
		// RegRxHeaderCntValueLsb=0x15
		// RegRxPacketCntValueMsb=0x16
		// RegRxPacketCntValueMsb=0x17
		RegModemStat = 0x18,
		RegPktSnrValue = 0x19,
		RegPktRssiValue = 0x1A,
		RegRssiValue = 0x1B,
		RegHopChannel = 0x1C,
		RegModemConfig1 = 0x1D,
		RegModemConfig2 = 0x1E,
		RegSymbTimeoutLsb = 0x1F,
		RegPreambleMsb = 0x20,
		RegPreambleLsb = 0x21,
		RegPayloadLength = 0x22,
		RegMaxPayloadLength = 0x23,
		RegHopPeriod = 0x24,
		// RegFifoRxByteAddr = 0x25
		RegModemConfig3 = 0x26,
		RegPpmCorrection = 0x27,
		// RegFeiMsb = 0x28
		// RegFeiMid = 0x29
		// RegFeiLsb = 0x2A
		// Reserved 0x2B
		RegRssiWideband = 0x2C, // Useful for random number generation
		// Reserved 0x2D-0x2E
		// RegIifFreq2 = 0x2F
		// RegIifFreq1 = 0x30
		RegDetectOptimize = 0x31,
		// Reserved 0x32
		RegInvertIq = 0x33,
		// Reserved 0x34-0x35
		RegHighBwOptimise1 = 0x36,
		RegDetectionThreshold = 0x37,
		// Reserved 0x38
		RegSyncWord = 0x39,
		RegHighBwOptimise2 = 0x3A,
		RegInvertIq2 = 0x3B,
		// Reserved 0x3C-0x3F
		RegDioMapping1 = 0x40,
		RegDioMapping2 = 0x41,
		RegVersion = 0x42,
		RegPaDac = 0x4d,

		Maximum = RegPaDac,
	}
}