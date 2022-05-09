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
	using System.Diagnostics;
	using System.Threading;

	using System.Device.Gpio;
	using System.Device.Spi;

	public sealed class SX127XDevice
	{
		// Frequency configuration magic numbers from Semtech SX127X specs used to calculate RegFrMsb, RegFrMid, RegFrLsb 
		private const double SX127X_FXOSC = 32000000.0;
		private const double SX127X_FSTEP = SX127X_FXOSC / 524288.0;

		public const double FrequencyDefault = 434000000.0;

		public const byte MessageLengthMinimum = 0;
		public const byte MessageLengthMaximum = 128;

		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

		// RegPAConfig constants for outputpower param validation and RFO to PABoost tipping point.
		public const sbyte OutputPowerDefault = 13;

		private const sbyte OutputPowerPABoostMin = 2;
		private const sbyte OutputPowerPABoostMax = 20;
		private const sbyte OutputPowerPABoostPaDacThreshhold = 17;

		private const sbyte OutputPowerRfoMin = -4;
		private const sbyte OutputPowerRfoMax = 15;
		private const sbyte OutputPowerRfoThreshhold = 0;

		//RegRssiValue magic numbers for calculating low/high band RSSI
		private const double SX127XMidBandThreshold = 525000000.0; // Search for RF_MID_BAND_THRESH GitHub LoRaNet LoRaMac-node/src/boards/sx1276-board.h
		private const int RssiAdjustmentHF = -157;
		private const int RssiAdjustmentLF = -164;

		// RegModemConfig2 for MSb RegSymbTimeoutLsb for LSB
		public const ushort SymbolTimeoutDefault = 0x64;
		private const ushort SymbolTimeoutMin = 0x0;
		private const ushort SymbolTimeoutMax = 0x1023;
		private const byte SymbolTimeoutMsbMask = 0b0011;

		// RegReambleMsb & RegReambleLsb
		public const ushort PreambleLengthDefault = 0x08;

		// RegPayloadLength
		public const byte PayloadLengthDefault = 0x01;

		// RegMaxPayloadLength
		public const byte PayloadMaxLengthDefault = 0xff;

		// RegHopPeriod
		public const byte FreqHoppingPeriodDefault = 0x0;

		// RegSyncWord Syncword default for public networks
		public const byte RegSyncWordDefault = 0x12;

		// RegPpmCorrection
		public const byte ppmCorrectionDefault = 0x0;

		// The Semtech ID Relating to the Silicon revision in RegVersion
		private const byte RegVersionValueExpected = 0x12;

		public class OnRxTimeoutEventArgs : EventArgs
		{
		}
		public delegate void onRxTimeoutEventHandler(Object sender, OnRxTimeoutEventArgs e);
		public event onRxTimeoutEventHandler OnRxTimeout;

		public class OnDataReceivedEventArgs : EventArgs
		{
			public float PacketSnr { get; set; }
			public int PacketRssi { get; set; }
			public int Rssi { get; set; }
			public byte[] Data { get; set; }
		}
		public delegate void onReceivedEventHandler(Object sender, OnDataReceivedEventArgs e);
		public event onReceivedEventHandler OnReceive;

		public class OnPayloadCrcErrorEventArgs : EventArgs
		{
		}
		public delegate void onPayloadCrcErrorEventHandler(Object sender, OnPayloadCrcErrorEventArgs e);
		public event onPayloadCrcErrorEventHandler OnPayloadCrcError;

		public class OnValidHeaderEventArgs : EventArgs
		{
		}
		public delegate void onValidHeaderEventHandler(Object sender, OnValidHeaderEventArgs e);
		public event onValidHeaderEventHandler OnValidHeader;

		public class OnDataTransmitedEventArgs : EventArgs
		{
		}
		public delegate void onTransmittedEventHandler(Object sender, OnDataTransmitedEventArgs e);
		public event onTransmittedEventHandler OnTransmit;

		public class OnChannelActivityDetectionDoneEventArgs : EventArgs
		{
		}
		public delegate void onChannelActivityDetectionDoneEventHandler(Object sender, OnChannelActivityDetectionDoneEventArgs e);
		public event onChannelActivityDetectionDoneEventHandler OnChannelActivityDetectionDone;

		public class OnFhssChangeChannelEventArgs : EventArgs
		{
		}
		public delegate void OnFhssChangeChannelEventHandler(Object sender, OnFhssChangeChannelEventArgs e);
		public event OnFhssChangeChannelEventHandler OnFhssChangeChannel;

		public class OnChannelActivityDetectedEventArgs : EventArgs
		{
		}
		public delegate void onChannelActivityDetectedEventHandler(Object sender, OnChannelActivityDetectedEventArgs e);
		public event onChannelActivityDetectedEventHandler OnChannelActivityDetected;

		// Hardware configuration support
		private readonly int _resetPin;
		private readonly GpioController _gpioController = null;
		private readonly RegisterManager _registerManager = null;
		private readonly object _regFifoLock = new object();
		private double _frequency = FrequencyDefault;
		private bool _rxDoneIgnoreIfCrcMissing = true;
		private bool _rxDoneIgnoreIfCrcInvalid = true;

		public SX127XDevice(SpiDevice spiDevice, GpioController gpioController,
			int dio0Pin,
			int dio1Pin = 0,
			int dio2Pin = 0,
			int dio3Pin = 0,
			int dio4Pin = 0,
			int dio5Pin = 0,
			int resetPin = 0
			)
		{
			_gpioController = gpioController;

			// Factory reset pin configuration
			if (resetPin != 0)
			{
				_resetPin = resetPin;
				_gpioController.OpenPin(resetPin, PinMode.Output);

				_gpioController.Write(resetPin, PinValue.Low);
				Thread.Sleep(20);
				_gpioController.Write(resetPin, PinValue.High);
				Thread.Sleep(50);
			}

			_registerManager = new RegisterManager(spiDevice, RegisterAddressReadMask, RegisterAddressWriteMask);

			// Once the pins setup check that SX127X chip is present
			Byte regVersionValue = _registerManager.ReadByte((byte)Registers.RegVersion);
			if (regVersionValue != RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			// See Table 18 DIO Mapping LoRa® Mode
			RegDioMapping1 regDioMapping1Value = RegDioMapping1.Dio0None;
			regDioMapping1Value |= RegDioMapping1.Dio1None;
			regDioMapping1Value |= RegDioMapping1.Dio2None;
			regDioMapping1Value |= RegDioMapping1.Dio3None;
			_registerManager.WriteByte((byte)Registers.RegDioMapping1, (byte)regDioMapping1Value);

			// Currently no easy way to test this with available hardware
			//Configuration.RegDioMapping2 regDioMapping2Value = Configuration.RegDioMapping2.Dio4None;
			//regDioMapping2Value = Configuration.RegDioMapping2.Dio5None;
			//_registerManager.WriteByte((byte)Configuration.Registers.RegDioMapping2, (byte)regDioMapping2Value);

			// Interrupt pin for RXDone, TXDone, and CadDone notification 
			_gpioController.OpenPin(dio0Pin, PinMode.InputPullDown);
			_gpioController.RegisterCallbackForPinValueChangedEvent(dio0Pin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);

			// RxTimeout, FhssChangeChannel, and CadDetected
			if (dio1Pin != 0)
			{
				_gpioController.OpenPin(dio1Pin, PinMode.InputPullDown);
				_gpioController.RegisterCallbackForPinValueChangedEvent(dio1Pin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
			}

			// FhssChangeChannel, FhssChangeChannel, and FhssChangeChannel
			if (dio2Pin != 0)
			{
				_gpioController.OpenPin(dio2Pin, PinMode.InputPullDown);
				_gpioController.RegisterCallbackForPinValueChangedEvent(dio2Pin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
			}

			// CadDone, ValidHeader, and PayloadCrcError
			if (dio3Pin != 0)
			{
				_gpioController.OpenPin(dio3Pin, PinMode.InputPullDown);
				_gpioController.RegisterCallbackForPinValueChangedEvent(dio3Pin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
			}

			// CadDetected, PllLock and PllLock
			if (dio4Pin != 0)
			{
				_gpioController.OpenPin(dio4Pin, PinMode.InputPullDown);
				_gpioController.RegisterCallbackForPinValueChangedEvent(dio4Pin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
			}

			// ModeReady, ClkOut and ClkOut
			if (dio5Pin != 0)
			{
				_gpioController.OpenPin(dio5Pin, PinMode.InputPullDown);
				_gpioController.RegisterCallbackForPinValueChangedEvent(dio5Pin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
			}
		}

		private void SetMode(RegOpModeMode mode)
		{
			RegOpModeModeFlags regOpModeValue;

			regOpModeValue = RegOpModeModeFlags.LongRangeModeLoRa;
			regOpModeValue |= RegOpModeModeFlags.AcessSharedRegLoRa;
			if (_frequency > SX127XMidBandThreshold)
			{
				regOpModeValue |= RegOpModeModeFlags.LowFrequencyModeOnHighFrequency;
			}
			else
			{
				regOpModeValue |= RegOpModeModeFlags.LowFrequencyModeOnLowFrequency;
			}
			regOpModeValue |= (RegOpModeModeFlags)mode;
			_registerManager.WriteByte((byte)Registers.RegOpMode, (byte)regOpModeValue);
		}

		public void Initialise(double frequency = FrequencyDefault, // RegFrMsb, RegFrMid, RegFrLsb
			bool rxDoneignoreIfCrcMissing = true, bool rxDoneignoreIfCrcInvalid = true,
			sbyte outputPower = OutputPowerDefault, RegPAConfigPASelect powerAmplifier = RegPAConfigPASelect.Default, // RegPAConfig & RegPaDac
			RegOcp ocpOn = RegOcp.Default, RegOcpTrim ocpTrim = RegOcpTrim.Default, // RegOcp
			RegLnaLnaGain lnaGain = RegLnaLnaGain.Default, bool lnaBoost = false, // RegLna
			RegModemConfig1Bandwidth bandwidth = RegModemConfig1Bandwidth.Default, RegModemConfig1CodingRate codingRate = RegModemConfig1CodingRate.Default, RegModemConfig1ImplicitHeaderModeOn implicitHeaderModeOn = RegModemConfig1ImplicitHeaderModeOn.Default, //RegModemConfig1
			RegModemConfig2SpreadingFactor spreadingFactor = RegModemConfig2SpreadingFactor.Default, bool txContinuousMode = false, bool rxPayloadCrcOn = false,
			ushort symbolTimeout = SymbolTimeoutDefault,
			ushort preambleLength = PreambleLengthDefault,
			byte payloadLength = PayloadLengthDefault,
			byte payloadMaxLength = PayloadMaxLengthDefault,
			byte freqHoppingPeriod = FreqHoppingPeriodDefault,
			RegModemConfig3LowDataRateOptimise lowDataRateOptimize = RegModemConfig3LowDataRateOptimise.Default, RegModemConfig3AgcAutoOn agcAutoOn = RegModemConfig3AgcAutoOn.Default,
			byte ppmCorrection = ppmCorrectionDefault,
			RegDetectOptimizeDetectionOptimize detectionOptimize = RegDetectOptimizeDetectionOptimize.Default,
			InvertIqRx invertIqRX = InvertIqRx.Default, InvertIqTx invertIqTX = InvertIqTx.Default,
			RegisterDetectionThreshold detectionThreshold = RegisterDetectionThreshold.Default,
			byte syncWord = RegSyncWordDefault)
		{

			_frequency = frequency; // Store this away for RSSI adjustments
			_rxDoneIgnoreIfCrcMissing = rxDoneignoreIfCrcMissing;
			_rxDoneIgnoreIfCrcInvalid = rxDoneignoreIfCrcInvalid;

			// Strobe Reset pin briefly to factory reset SX127X chip
			if (_resetPin != 0)
			{
				_gpioController.Write(_resetPin, PinValue.Low);
				Thread.Sleep(20);
				_gpioController.Write(_resetPin, PinValue.High);
				Thread.Sleep(50);
			}

			// Put the device into sleep mode so registers can be changed
			SetMode(RegOpModeMode.Sleep);

			// Configure RF Carrier frequency 
			if (frequency != FrequencyDefault)
			{
				byte[] bytes = BitConverter.GetBytes((long)(frequency / SX127X_FSTEP));
				_registerManager.WriteByte((byte)Registers.RegFrMsb, bytes[2]);
				_registerManager.WriteByte((byte)Registers.RegFrMid, bytes[1]);
				_registerManager.WriteByte((byte)Registers.RegFrLsb, bytes[0]);
			}

			// Set RegPAConfig & RegPaDac if powerAmplifier/OutputPower settings not defaults
			if ((powerAmplifier != RegPAConfigPASelect.Default) || (outputPower != OutputPowerDefault))
			{
				if (powerAmplifier == RegPAConfigPASelect.PABoost)
				{
					byte regPAConfigValue = (byte)RegPAConfigPASelect.PABoost;

					// Validate the minimum and maximum PABoost outputpower
					if ((outputPower < OutputPowerPABoostMin) || (outputPower > OutputPowerPABoostMax))
					{
						throw new ApplicationException($"PABoost {outputPower}dBm Min power {OutputPowerPABoostMin} to Max power {OutputPowerPABoostMax}");
					}

					if (outputPower <= OutputPowerPABoostPaDacThreshhold)
					{
						// outputPower 0..15 so pOut is 2=17-(15-0)...17=17-(15-15)
						regPAConfigValue |= (byte)RegPAConfigMaxPower.Default;
						regPAConfigValue |= (byte)(outputPower - 2);

						_registerManager.WriteByte((byte)Registers.RegPAConfig, regPAConfigValue);
						_registerManager.WriteByte((byte)Registers.RegPaDac, (byte)RegPaDac.Normal);
					}
					else
					{
						// outputPower 0..15 so pOut is 5=20-(15-0)...20=20-(15-15) // See https://github.com/adafruit/RadioHead/blob/master/RH_RF95.cpp around line 411 could be 23dBm
						regPAConfigValue |= (byte)RegPAConfigMaxPower.Default;
						regPAConfigValue |= (byte)(outputPower - 5);

						_registerManager.WriteByte((byte)Registers.RegPAConfig, regPAConfigValue);
						_registerManager.WriteByte((byte)Registers.RegPaDac, (byte)RegPaDac.Boost);
					}
				}
				else
				{
					byte regPAConfigValue = (byte)RegPAConfigPASelect.Rfo;

					// Validate the minimum and maximum RFO outputPower
					if ((outputPower < OutputPowerRfoMin) || (outputPower > OutputPowerRfoMax))
					{
						throw new ApplicationException($"RFO {outputPower}dBm Min power {OutputPowerRfoMin} to Max power {OutputPowerRfoMax}");
					}

					// Set MaxPower and Power calculate pOut = PMax-(15-outputPower), pMax=10.8 + 0.6*MaxPower 
					if (outputPower > OutputPowerRfoThreshhold)
					{
						// pMax 15=10.8+0.6*7 with outputPower 0...15 so pOut is 15=pMax-(15-0)...0=pMax-(15-15) 
						regPAConfigValue |= (byte)RegPAConfigMaxPower.Max;
						regPAConfigValue |= (byte)(outputPower + 0);
					}
					else
					{
						// pMax 10.8=10.8+0.6*0 with output power 0..15 so pOut is -4=10-(15-0)...10.8=10.8-(15-15)
						regPAConfigValue |= (byte)RegPAConfigMaxPower.Min;
						regPAConfigValue |= (byte)(outputPower + 4);
					}

					_registerManager.WriteByte((byte)Registers.RegPAConfig, regPAConfigValue);
					_registerManager.WriteByte((byte)Registers.RegPaDac, (byte)RegPaDac.Normal);
				}
			}

			// Set RegOcp if any of the settings not defaults
			if ((ocpOn != RegOcp.Default) || (ocpTrim != RegOcpTrim.Default))
			{
				byte regOcpValue = (byte)ocpTrim;

				regOcpValue |= (byte)ocpOn;

				_registerManager.WriteByte((byte)Registers.RegOcp, regOcpValue);
			}

			// Set RegLna if any of the settings not defaults
			if ((lnaGain != RegLnaLnaGain.Default) || (lnaBoost != false)) // TODO : Fix lnaBoost default as there must be a better way of doing this
			{
				byte regLnaValue = (byte)lnaGain;

				regLnaValue |= (byte)RegLnaLnaBoost.LfDefault;
				regLnaValue |= (byte)RegLnaLnaBoost.HfDefault;

				if (lnaBoost)
				{
					if (_frequency > SX127XMidBandThreshold)
					{
						regLnaValue |= (byte)RegLnaLnaBoost.HfOn;
					}
					else
					{
						regLnaValue |= (byte)RegLnaLnaBoost.LfOn;
					}
				}
				_registerManager.WriteByte((byte)Registers.RegLna, regLnaValue);
			}

			// Set regModemConfig1 if any of the settings not defaults
			if ((bandwidth != RegModemConfig1Bandwidth.Default) || (codingRate != RegModemConfig1CodingRate.Default) || (implicitHeaderModeOn != RegModemConfig1ImplicitHeaderModeOn.Default))
			{
				byte regModemConfig1Value = (byte)bandwidth;
				regModemConfig1Value |= (byte)codingRate;
				regModemConfig1Value |= (byte)implicitHeaderModeOn;
				_registerManager.WriteByte((byte)Registers.RegModemConfig1, regModemConfig1Value);
			}

			if ((symbolTimeout < SymbolTimeoutMin) || (symbolTimeout > SymbolTimeoutMax))
			{
				throw new ArgumentException($"symbolTimeout must be between {SymbolTimeoutMin} and {SymbolTimeoutMax}", nameof(symbolTimeout));
			}

			// Set regModemConfig2 if any of the settings not defaults
			if ((spreadingFactor != RegModemConfig2SpreadingFactor.Default) || (txContinuousMode != false) | (rxPayloadCrcOn != false) || (symbolTimeout != SymbolTimeoutDefault))
			{
				byte RegModemConfig2Value = (byte)spreadingFactor;
				if (txContinuousMode)
				{
					RegModemConfig2Value |= (byte)RegModemConfig2TxContinuousMode.On;
				}
				if (rxPayloadCrcOn)
				{
					RegModemConfig2Value |= (byte)RegModemConfig2RxPayloadCrc.On;
				}
				// Get the MSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);

				// Only the zeroth & second bit of byte matter
				symbolTimeoutBytes[1] &= SymbolTimeoutMsbMask;
				RegModemConfig2Value |= symbolTimeoutBytes[1];
				_registerManager.WriteByte((byte)Registers.RegModemConfig2, RegModemConfig2Value);
			}

			// RegModemConfig2.SymbTimout + RegSymbTimeoutLsb
			if (symbolTimeout != SymbolTimeoutDefault)
			{
				// Get the LSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				_registerManager.WriteByte((byte)Registers.RegSymbTimeoutLsb, symbolTimeoutBytes[0]);
			}

			// RegPreambleMsb + RegPreambleLsb
			if (preambleLength != PreambleLengthDefault)
			{
				_registerManager.WriteWordMsbLsb((Byte)Registers.RegPreambleMsb, preambleLength);
			}

			// RegPayloadLength
			if (payloadLength != PayloadLengthDefault)
			{
				_registerManager.WriteByte((byte)Registers.RegPayloadLength, payloadLength);
			}

			// RegMaxPayloadLength
			if (payloadMaxLength != PayloadMaxLengthDefault)
			{
				_registerManager.WriteByte((byte)Registers.RegMaxPayloadLength, payloadMaxLength);
			}

			// RegHopPeriod
			if (freqHoppingPeriod != FreqHoppingPeriodDefault)
			{
				_registerManager.WriteByte((byte)Registers.RegHopPeriod, freqHoppingPeriod);
			}

			// RegModemConfig3
			if ((lowDataRateOptimize != RegModemConfig3LowDataRateOptimise.Default) || (agcAutoOn != RegModemConfig3AgcAutoOn.Default))
			{
				byte regModemConfig3Value = 0; // Buts 7-4 & 1-0 unused

				regModemConfig3Value |= (byte)lowDataRateOptimize;

				regModemConfig3Value |= (byte)agcAutoOn;

				_registerManager.WriteByte((byte)Registers.RegModemConfig3, regModemConfig3Value);
			}

			// RegPpmCorrection
			if (ppmCorrection != ppmCorrectionDefault)
			{
				_registerManager.WriteByte((byte)Registers.RegPpmCorrection, ppmCorrection);
			}

			// RegDetectOptimize
			if (detectionOptimize != RegDetectOptimizeDetectionOptimize.Default)
			{
				_registerManager.WriteByte((byte)Registers.RegDetectOptimize, (byte)detectionOptimize);
			}

			// TX & RX inversion plus optimisation specialness
			if ((invertIqRX != InvertIqRx.Default) || (invertIqTX != InvertIqTx.Default))
			{
				byte regInvertIqValue = 0b00100110; // Bits 5-1 are reserved and are 0x13

				if (invertIqRX == InvertIqRx.On)
				{
					regInvertIqValue |= (byte)InvertIqRx.On;
				}

				if (invertIqTX == InvertIqTx.On)
				{
					regInvertIqValue |= (byte)InvertIqTx.On;
				}

				_registerManager.WriteByte((byte)Registers.RegInvertIq, regInvertIqValue);

				if ((invertIqRX == InvertIqRx.On) || (invertIqTX == InvertIqTx.On))
				{
					_registerManager.WriteByte((byte)Registers.RegInvertIq2, (byte)RegInvertIq2.On);
				}
				else
				{
					_registerManager.WriteByte((byte)Registers.RegInvertIq2, (byte)RegInvertIq2.Off);
				}
			}

			// RegSyncWordDefault 
			if (syncWord != RegSyncWordDefault)
			{
				_registerManager.WriteByte((byte)Registers.RegSyncWord, syncWord);
			}
		}

		private void InterruptGpioPin_ValueChanged(object sender, PinValueChangedEventArgs pinValueChangedEventArgs)
		{
			Byte regIrqFlagsToClear = (byte)RegIrqFlags.ClearNone;

			// Read RegIrqFlags to see what caused the interrupt
			Byte irqFlags = _registerManager.ReadByte((byte)Registers.RegIrqFlags);

			//Console.WriteLine($"IrqFlags 0x{irqFlags:x} Pin:{pinValueChangedEventArgs.PinNumber}");

			// Check RxTimeout for inbound message
			if ((irqFlags & (byte)RegIrqFlagsMask.RxTimeoutMask) == (byte)RegIrqFlags.RxTimeout)
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.RxTimeout;

				ProcessRxTimeout(irqFlags);
			}

			// Check RxDone for inbound message
			if ((irqFlags & (byte)RegIrqFlagsMask.RxDoneMask) == (byte)RegIrqFlags.RxDone)
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.RxDone;

				ProcessRxDone(irqFlags);
			}

			// Check PayLoadCrcError for inbound message
			if ((irqFlags & (byte)RegIrqFlagsMask.PayLoadCrcErrorMask) == (byte)RegIrqFlags.PayLoadCrcError)
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.PayLoadCrcError;

				ProcessPayloadCrcError(irqFlags);
			}

			// Check ValidHeader for inbound message
			if ((irqFlags & (byte)RegIrqFlagsMask.ValidHeaderMask) == (byte)RegIrqFlags.ValidHeader)
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.ValidHeader;

				ProcessValidHeader(irqFlags);
			}

			// Check TxDone for outbound message
			if ((irqFlags & (byte)RegIrqFlagsMask.TxDoneMask) == (byte)RegIrqFlags.TxDone)
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.TxDone;

				ProcessTxDone(irqFlags);
			}

			// Check Channel Activity Detection done 
			if (((irqFlags & (byte)RegIrqFlagsMask.CadDoneMask) == (byte)RegIrqFlags.CadDone))
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.CadDone;

				ProcessChannelActivityDetectionDone(irqFlags);
			}

			// Check FhssChangeChannel for inbound message
			if ((irqFlags & (byte)RegIrqFlagsMask.FhssChangeChannelMask) == (byte)RegIrqFlags.FhssChangeChannel)
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.FhssChangeChannel;

				ProcessFhssChangeChannel(irqFlags);
			}

			// Check Channel Activity Detected 
			if (((irqFlags & (byte)RegIrqFlagsMask.CadDetectedMask) == (byte)RegIrqFlags.CadDetected))
			{
				regIrqFlagsToClear |= (byte)RegIrqFlags.CadDetected;

				ProcessChannelActivityDetected(irqFlags);
			}

			_registerManager.WriteByte((byte)Registers.RegIrqFlags, regIrqFlagsToClear);
		}

		private void ProcessRxTimeout(byte irqFlags)
		{
			OnRxTimeoutEventArgs onRxTimeoutArgs = new OnRxTimeoutEventArgs();

			OnRxTimeout?.Invoke(this, onRxTimeoutArgs);
		}

		private void ProcessRxDone(byte irqFlags)
		{
			byte[] payloadBytes;

			// Check to see if payload has CRC 
			if (_rxDoneIgnoreIfCrcMissing)
			{
				byte regHopChannel = _registerManager.ReadByte((byte)Registers.RegHopChannel);
				if ((regHopChannel & (byte)RegHopChannelMask.CrcOnPayload) != (byte)RegHopChannelFlags.CrcOnPayload)
				{
					return;
				}
			}

			// Check to see if payload CRC is valid
			if (_rxDoneIgnoreIfCrcInvalid)
			{
				if ((irqFlags & (byte)RegIrqFlagsMask.PayLoadCrcErrorMask) == (byte)RegIrqFlagsMask.PayLoadCrcErrorMask)
				{
					return;
				}
			}

			// Extract the message from the RFM9X fifo, try and keep lock in place for the minimum possible time
			lock (_regFifoLock)
			{
				byte currentFifoAddress = _registerManager.ReadByte((byte)Registers.RegFifoRxCurrent);

				_registerManager.WriteByte((byte)Registers.RegFifoAddrPtr, currentFifoAddress);

				byte numberOfBytes = _registerManager.ReadByte((byte)Registers.RegRxNbBytes);

				payloadBytes = _registerManager.ReadBytes((byte)Registers.RegFifo, numberOfBytes);
			}

			// Get the RSSI HF vs. LF port adjustment section 5.5.5 RSSI and SNR in LoRa Mode
			float packetSnr = _registerManager.ReadByte((byte)Registers.RegPktSnrValue) * 0.25f;

			int rssi = _registerManager.ReadByte((byte)Registers.RegRssiValue);
			if (_frequency > SX127XMidBandThreshold)
			{
				rssi = RssiAdjustmentHF + rssi;
			}
			else
			{
				rssi = RssiAdjustmentLF + rssi;
			}

			int packetRssi = _registerManager.ReadByte((byte)Registers.RegPktRssiValue);
			if (_frequency > SX127XMidBandThreshold)
			{
				packetRssi = RssiAdjustmentHF + packetRssi;
			}
			else
			{
				packetRssi = RssiAdjustmentLF + packetRssi;
			}

			OnDataReceivedEventArgs receiveArgs = new OnDataReceivedEventArgs
			{
				PacketSnr = packetSnr,
				Rssi = rssi,
				PacketRssi = packetRssi,
				Data = payloadBytes,
			};

			OnReceive?.Invoke(this, receiveArgs);
		}

		private void ProcessPayloadCrcError(byte irqFlags)
		{
			OnPayloadCrcErrorEventArgs payloadCrcErrorActivityDetectedArgs = new OnPayloadCrcErrorEventArgs();

			OnPayloadCrcError?.Invoke(this, payloadCrcErrorActivityDetectedArgs);
		}

		private void ProcessValidHeader(byte irqFlags)
		{
			OnValidHeaderEventArgs validHeaderArgs = new OnValidHeaderEventArgs();

			OnValidHeader?.Invoke(this, validHeaderArgs);
		}

		private void ProcessTxDone(byte irqFlags)
		{
			OnDataTransmitedEventArgs transmitArgs = new OnDataTransmitedEventArgs();

			OnTransmit?.Invoke(this, transmitArgs);
		}

		private void ProcessChannelActivityDetectionDone(byte irqFlags)
		{
			OnChannelActivityDetectionDoneEventArgs channelActivityDetectionDoneArgs = new OnChannelActivityDetectionDoneEventArgs();

			OnChannelActivityDetectionDone?.Invoke(this, channelActivityDetectionDoneArgs);
		}

		private void ProcessFhssChangeChannel(byte irqFlags)
		{
			OnFhssChangeChannelEventArgs fhssChangeChanneldArgs = new OnFhssChangeChannelEventArgs();

			OnFhssChangeChannel?.Invoke(this, fhssChangeChanneldArgs);
		}

		private void ProcessChannelActivityDetected(byte irqFlags)
		{
			OnChannelActivityDetectedEventArgs channelActivityDetectedArgs = new OnChannelActivityDetectedEventArgs();

			OnChannelActivityDetected?.Invoke(this, channelActivityDetectedArgs);
		}

		public void Receive()
		{
			_registerManager.WriteByte((byte)Registers.RegDioMapping1, (byte)RegDioMapping1.Dio0RxDone);

			SetMode(RegOpModeMode.ReceiveContinuous);
		}

		public void Send(byte[] messageBytes)
		{
			Debug.Assert(messageBytes != null);
			Debug.Assert(messageBytes.Length >= MessageLengthMinimum);
			Debug.Assert(messageBytes.Length <= MessageLengthMaximum);

			lock (_regFifoLock)
			{
				_registerManager.WriteByte((byte)Registers.RegFifoTxBaseAddr, 0x0);

				// Set the Register Fifo address pointer
				_registerManager.WriteByte((byte)Registers.RegFifoAddrPtr, 0x0);

				_registerManager.WriteBytes((byte)Registers.RegFifo, messageBytes);

				// Set the length of the message in the fifo
				_registerManager.WriteByte((byte)Registers.RegPayloadLength, (byte)messageBytes.Length);
			}

			_registerManager.WriteByte((byte)Registers.RegDioMapping1, (byte)RegDioMapping1.Dio0TxDone);
			SetMode(RegOpModeMode.Transmit);
		}

		public void ChannelActivityDetect()
		{
			_registerManager.WriteByte((byte)Registers.RegDioMapping1, (byte)RegDioMapping1.Dio1CadDetect);

			SetMode(RegOpModeMode.ChannelActivityDetection);
		}

		public byte Random()
		{
			return _registerManager.ReadByte((byte)Registers.RegRssiWideband);
		}

		public void RegisterDump()
		{
			Debug.WriteLine("Register dump");
			for (byte registerIndex = (byte)Registers.Minimum; registerIndex <= (byte)Registers.Maximum; registerIndex++)
			{
				byte registerValue = _registerManager.ReadByte(registerIndex);

				Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
			}

			Debug.WriteLine("");
		}
	}
}
