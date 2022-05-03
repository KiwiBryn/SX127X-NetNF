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
