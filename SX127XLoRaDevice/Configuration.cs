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

	public static class Configuration
	{
		// Registers from SemTech SX127X Datasheet
		public enum Registers : byte
		{
			MinValue = RegOpMode,

			RegFifo = 0x0,
			RegOpMode = 0x01,
			//Reserved 0x02-0x05 
			RegFrMsb = 0x06,
			RegFrMid = 0x7,
			RegFrLsb = 0x08,
			RegPAConfig = 0x09,
			//RegPARamp = 0x0A,
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
			RegInvertIQ = 0x33,
			// Reserved 0x34-0x35
			RegHighBwOptimise1 = 0x36,
			RegDetectionThreshold = 0x37,
			// Reserved 0x38
			RegSyncWord = 0x39,
			RegHighBwOptimise2 = 0x3A,
			RegInvertIQ2 = 0x3B,
			// Reserved 0x3C-0x3F
			RegDioMapping1 = 0x40,
			RegDioMapping2 = 0x41,
			RegVersion = 0x42,
			RegPaDac = 0x4d,

			MaxValue = RegPaDac,
		}

		// RegOpMode mode flags
		[Flags]
		public enum RegOpModeModeFlags : byte
		{
			LongRangeModeLoRa = 0b10000000,
			LongRangeModeFskOok = 0b00000000,
			LongRangeModeDefault = LongRangeModeFskOok,
			AcessSharedRegLoRa = 0b00000000,
			AcessSharedRegFsk = 0b01000000,
			AcessSharedRegDefault = AcessSharedRegLoRa,
			LowFrequencyModeOnHighFrequency = 0b00000000,
			LowFrequencyModeOnLowFrequency = 0b00001000,
			LowFrequencyModeOnDefault= LowFrequencyModeOnLowFrequency
		}

		public enum RegOpModeMode : byte
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

		// Frequency configuration magic numbers from Semtech SX127X specs
		public const double SX127X_FXOSC = 32000000.0;
		public const double SX127X_FSTEP = SX127X_FXOSC / 524288.0; 
		public const double SX127XMidBandThreshold = 525000000.0; // Search for RF_MID_BAND_THRESH GitHub LoRaNet LoRaMac-node/src/boards/sx1276-board.h
		public const int RssiAdjustmentHF = -157;
		public const int RssiAdjustmentLF = -164;

		// RegFrMsb, RegFrMid, RegFrLsb
		public const double FrequencyDefault = 434000000.0;

		// RegPAConfig based RegPAConfigPADac with complexity hidden from user 
		public const Byte RegPAConfigPASelectRfo = 0b00000000;
		public const Byte RegPAConfigPASelectPABoost = 0b10000000;
		public const byte RegPAConfigMaxPowerMax = 0b01110000;

		public enum PowerAmplifier
		{
			Rfo,
			PABoost
		}
		public const PowerAmplifier PowerAmplifierDefault = PowerAmplifier.Rfo;

		public const sbyte OutputPowerDefault = 0x0F;

		// Validation constants for outputpower param
		public const sbyte OutputPowerPABoostMin = 5;
		public const sbyte OutputPowerPABoostMax = 23;
		public const sbyte OutputPowerRfoMin = -1;
		public const sbyte OutputPowerRfoMax = 14;

		// RegPaRamp appears to be for FSK only ?

		// RegOcp
		public const byte RegOcpOn = 0b00100000;
		public const byte RegOcpOff = 0b00000000;
		public const bool RegOcpDefault = true;

		public const byte RegOcpOcpTrimMin = 0b00000000;
		public const byte RegOcpOcpTrimMax = 0b00011111;
		public const byte RegOcpOcpTrimDefault = 0b00001011;

		// RegLna
		[Flags]
		public enum RegLnaLnaGain : byte
		{
			G1 = 0b00100000,
			G2 = 0b01000000,
			G3 = 0b01100000,
			G4 = 0b10000000,
			G5 = 0b10100000,
			G6 = 0b11000000
		}
		public const RegLnaLnaGain LnaGainDefault = RegLnaLnaGain.G1;
		public const bool LnaBoostDefault = false;

		public const byte RegLnaLnaBoostLfOn = 0b00011000;
		public const byte RegLnaLnaBoostLfOff = 0b00000000;
		public const byte RegLnaLnaBoostLfDefault = RegLnaLnaBoostLfOff;

		public const byte RegLnaLnaBoostHfOn = 0b00000011;
		public const byte RegLnaLnaBoostHfOff = 0b00000000;
		public const byte RegLnaLnaBoostHfDefault = RegLnaLnaBoostHfOff;

		[Flags]
		public enum RegIrqFlagsMask : byte
		{
			RxTimeoutMask = 0b10000000,
			RxDoneMask = 0b01000000,
			PayLoadCrcErrorMask = 0b00100000,
			ValidHeadrerMask = 0b00010000,
			TxDoneMask = 0b00001000,
			CadDoneMask = 0b00000100,
			FhssChangeChannelMask = 0b00000010,
			CadDetectedMask = 0b00000001,
		}

		[Flags]
		public enum RegIrqFlags : byte
		{
			RxTimeout = 0b10000000,
			RxDone = 0b01000000,
			PayLoadCrcError = 0b00100000,
			ValidHeadrer = 0b00010000,
			TxDone = 0b00001000,
			CadDone = 0b00000100,
			FhssChangeChannel = 0b00000010,
			CadDetected = 0b00000001,
			ClearAll = 0b11111111,
		}

		[Flags]
		public enum RegHopChannelFlags : byte
		{
			PllTimeout = 0b10000000,
			CrcOnPayload = 0b01000000,
		}

		public enum RegHopChannelMask : byte
		{
			PllTimeout = 0b10000000,
			CrcOnPayload = 0b01000000,
			FhssPresentChannel = 0b01111111,
		}

		// RegModemConfig1
		public enum RegModemConfigBandwidth : byte
		{
			_7_8KHz = 0b00000000,
			_10_4KHz = 0b00010000,
			_15_6KHz = 0b00100000,
			_20_8KHz = 0b00110000,
			_31_25KHz = 0b01000000,
			_41_7KHz = 0b01010000,
			_62_5KHz = 0b01100000,
			_125KHz = 0b01110000,
			_250KHz = 0b10000000,
			_500KHz = 0b10010000
		}
		public const RegModemConfigBandwidth RegModemConfigBandwidthDefault = RegModemConfigBandwidth._125KHz;

		public enum RegModemConfigCodingRate
		{
			_4of5 = 0b00000010,
			_4of6 = 0b00000100,
			_4of7 = 0b00000110,
			_4of8 = 0b00001000,
		}
		public const RegModemConfigCodingRate RegModemConfigCodingRateDefault = RegModemConfigCodingRate._4of5;

		public enum RegModemConfigImplicitHeaderModeOn
		{
			ExplicitHeaderMode = 0b00000000,
			ImplicitHeaderMode = 0b00000001,
		}
		public const RegModemConfigImplicitHeaderModeOn RegModemConfigImplicitHeaderModeOnDefault = RegModemConfigImplicitHeaderModeOn.ExplicitHeaderMode;

		// RegModemConfig2
		public enum RegModemConfig2SpreadingFactor : byte
		{
			_64ChipsPerSymbol = 0b01100000,
			_128ChipsPerSymbol = 0b01110000,
			_256ChipsPerSymbol = 0b10000000,
			_512ChipsPerSymbol = 0b10010000,
			_1024ChipsPerSymbol = 0b10100000,
			_2048ChipsPerSymbol = 0b10110000,
			_4096ChipsPerSymbol = 0b11000000,
		}
		public const RegModemConfig2SpreadingFactor RegModemConfig2SpreadingFactorDefault = RegModemConfig2SpreadingFactor._128ChipsPerSymbol;

		public const byte RegModemConfig2TxContinuousModeOn = 0b00001000;
		public const byte RegModemConfig2TxContinuousModeOff = 0b00000000;
		public const byte RegModemConfig2TxContinuousModeDefault = RegModemConfig2TxContinuousModeOff;

		public const byte RegModemConfig2RxPayloadCrcOn = 0b00000100;
		public const byte RegModemConfig2RxPayloadCrcOff = 0b00000000;
		public const byte RegModemConfig2RxPayloadCrcDefault = RegModemConfig2RxPayloadCrcOff;

		// RegModemConfig2 for MSb RegSymbTimeoutLsb for LSB
		public const ushort SymbolTimeoutDefault = 0x64;
		public const ushort symbolTimeoutMin = 0x0;
		public const ushort symbolTimeoutMax = 0x1023;

		public const byte SymbolTimeoutMsbMask = 0b0011;

		// RegReambleMsb & RegReambleLsb
		public const ushort PreambleLengthDefault = 0x08;

		// RegPayloadLength
		public const byte PayloadLengthDefault = 0x01;

		// RegMaxPayloadLength
		public const byte PayloadMaxLengthDefault = 0xff;

		// RegHopPeriod
		public const byte FreqHoppingPeriodDefault = 0x0;

		public const bool LowDataRateOptimizeDefault = false;

		public const bool AgcAutoOnDefault = false;

		// RegModemConfig3
		public const byte RegModemConfig3LowDataRateOptimizeOn = 0b00001000;
		public const byte RegModemConfig3LowDataRateOptimizeOff = 0b00000000;
		public const byte RegModemConfig3LowDataRateOptimizeDefault = RegModemConfig3LowDataRateOptimizeOff;

		public const byte RegModemConfig3AgcAutoOn = 0b00000100;
		public const byte RegModemConfig3AgcAutoOff = 0b00000000;
		public const byte RegModemConfig3AgcAutoDefault = RegModemConfig3AgcAutoOff;

		// RegPpmCorrection
		public const byte ppmCorrectionDefault = 0x0;

		// RegDetectOptimize
		public enum RegDetectOptimizeDectionOptimize
		{
			SF7toSF12 = 0x03,
			SF6 = 0x05,
		};
		public const RegDetectOptimizeDectionOptimize RegDetectOptimizeDectionOptimizeDefault = RegDetectOptimizeDectionOptimize.SF7toSF12;

		// RegInvertId
		public const byte RegInvertIdDefault = 0b00100110;
		public const byte InvertIqRXOn = 0b01000000;
		public const byte InvertIqRXOff = 0b00000000;
		public const bool InvertIqRXDefault = false;

		public const byte InvertIqTXOn = 0b00000001;
		public const byte InvertIqTXOff = 0b00000000;
		public const bool InvertIqTXDefault = true;

		public const byte RegInvertIq2On = 0x19;
		public const byte RegInvertIq2Off = 0x1D;

		// RegDetectionThreshold
		public enum RegisterDetectionThreshold
		{
			SF7toSF12 = 0x0A,
			SF6 = 0x0c,
		}
		public const RegisterDetectionThreshold RegisterDetectionThresholdDefault = RegisterDetectionThreshold.SF7toSF12;

		// RegSyncWord Syncword default for public networks
		public const byte RegSyncWordDefault = 0x12;

		// RegDioMapping1 
		[Flags]
		public enum RegDioMapping1
		{
			Dio0RxDone = 0b00000000,
			Dio0TxDone = 0b01000000,
			Dio0CadDone = 0b1000000,
		}

		// The Semtech ID Relating to the Silicon revision
		public const byte RegVersionValueExpected = 0x12;

		// RegPaDac more power
		[Flags]
		public enum RegPaDac
		{
			Normal = 0b10000100,
			Boost = 0b10000111,
		}
		public const byte RegPaDacPABoostThreshold = 20;
	}
}
