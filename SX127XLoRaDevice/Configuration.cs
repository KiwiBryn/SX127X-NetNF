﻿//---------------------------------------------------------------------------------
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

		public const sbyte OutputPowerDefault = 13;

		// Validation constants for outputpower param
		internal const sbyte OutputPowerPABoostMin = 2;
		internal const sbyte OutputPowerPABoostMax = 20;
		internal const sbyte OutputPowerPABoostPaDacThreshhold = 17;

		internal const sbyte OutputPowerRfoMin = -4;
		internal const sbyte OutputPowerRfoMax = 15;
		internal const sbyte OutputPowerRfoThreshhold = 0;

		// RegPaRamp appears to be for FSK only ?

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

		// RegDetectionThreshold
		public enum RegisterDetectionThreshold
		{
			SF7toSF12 = 0x0A,
			SF6 = 0x0c,
		}
		public const RegisterDetectionThreshold RegisterDetectionThresholdDefault = RegisterDetectionThreshold.SF7toSF12;

		// RegSyncWord Syncword default for public networks
		public const byte RegSyncWordDefault = 0x12;

		// The Semtech ID Relating to the Silicon revision
		internal const byte RegVersionValueExpected = 0x12;

		internal const byte RegPaDacPABoostThreshold = 20;
	}
}
