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
		// RegOpMode mode flags
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
		internal const double SX127X_FXOSC = 32000000.0;
		internal const double SX127X_FSTEP = SX127X_FXOSC / 524288.0;
		internal const double SX127XMidBandThreshold = 525000000.0; // Search for RF_MID_BAND_THRESH GitHub LoRaNet LoRaMac-node/src/boards/sx1276-board.h
		internal const int RssiAdjustmentHF = -157;
		internal const int RssiAdjustmentLF = -164;

		// RegFrMsb, RegFrMid, RegFrLsb
		public const double FrequencyDefault = 434000000.0;

		// RegPAConfig based RegPaDac with complexity hidden from user 
		public enum RegPAConfigPASelect:byte
		{
			Rfo = 0b00000000,
			PABoost = 0b10000000,
			Default = Rfo
		}

		internal enum RegPAConfigMaxPower:byte
		{
			Min = 0b00000000,
			Max = 0b01110000,
			Default = 0b01000000
		}
		public const sbyte OutputPowerDefault = 13;

		// Validation constants for outputpower param
		internal const sbyte OutputPowerPABoostMin = 2;
		internal const sbyte OutputPowerPABoostMax = 20;
		internal const sbyte OutputPowerPABoostPaDacThreshhold = 17;

		internal const sbyte OutputPowerRfoMin = -4;
		internal const sbyte OutputPowerRfoMax = 15;
		internal const sbyte OutputPowerRfoThreshhold = 0;

		// RegPaRamp appears to be for FSK only ?

		// RegOcp
		public enum RegOcp
		{
			On = 0b00100000,
			Off = 0b00000000,
			Default = On
		}

		// RegOcpTrim
		public enum RegOcpTrim
		{
			Minimum  = _45mA,
			_45mA = 0x00,
			_50mA = 0x01,
			_55mA = 0x02,
			_60mA = 0x03,
			_65mA = 0x04,
			_70mA = 0x05,
			_75mA = 0x06,
			_80mA = 0x07,
			_85mA = 0x08,
			_90mA = 0x09,
			_95mA = 0x0A,
			_100mA = 0x0B,  
			Default = _100mA,
			_105mA = 0x0C,
			_110mA = 0x0D,
			_115mA = 0x0E,
			_120mA = 0x0F,
			_130mA = 0x10,
			_140mA = 0x11,
			_150mA = 0x12,
			_160mA = 0x13,
			_170mA = 0x14,
			_180mA = 0x15,
			_190mA = 0x16,
			_200mA = 0x17,
			_210mA = 0x18,
			_220mA = 0x19,
			_230mA = 0x1A,
			_240mA = 0x1B,
			Maximum = _240mA
		};

		// RegLna
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
		public const bool LnaBoostDefault = false;

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
			Default = _125KHz,
			_250KHz = 0b10000000,
			_500KHz = 0b10010000
		}

		public enum RegModemConfigCodingRate
		{
			_4of5 = 0b00000010,
			Default = _4of5,
			_4of6 = 0b00000100,
			_4of7 = 0b00000110,
			_4of8 = 0b00001000,
		}

		public enum RegModemConfigImplicitHeaderModeOn
		{
			ExplicitHeaderMode = 0b00000000,
			Default = ExplicitHeaderMode,
			ImplicitHeaderMode = 0b00000001,
		}

		// RegModemConfig2
		public enum RegModemConfig2SpreadingFactor : byte
		{
			_64ChipsPerSymbol = 0b01100000,
			_128ChipsPerSymbol = 0b01110000,
			Default = _128ChipsPerSymbol,
			_256ChipsPerSymbol = 0b10000000,
			_512ChipsPerSymbol = 0b10010000,
			_1024ChipsPerSymbol = 0b10100000,
			_2048ChipsPerSymbol = 0b10110000,
			_4096ChipsPerSymbol = 0b11000000,
		}

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
		internal const byte RegInvertIqDefault = 0b00100110;

		public enum InvertIqRx :byte
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

		// RegDetectionThreshold
		public enum RegisterDetectionThreshold
		{
			SF7toSF12 = 0x0A,
			SF6 = 0x0c,
		}
		public const RegisterDetectionThreshold RegisterDetectionThresholdDefault = RegisterDetectionThreshold.SF7toSF12;

		// RegSyncWord Syncword default for public networks
		public const byte RegSyncWordDefault = 0x12;

		// RegDioMapping1 from table 18 for Dio0..Dio3 in LoRaMode
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
			Dio1Default= 0b00000000,
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

		// The Semtech ID Relating to the Silicon revision
		internal const byte RegVersionValueExpected = 0x12;

		// RegPaDac more power
		[Flags]
		internal enum RegPaDac
		{
			Normal = 0x84,
			Boost = 0x87,
			Default = Normal
		}
		internal const byte RegPaDacPABoostThreshold = 20;
	}
}
