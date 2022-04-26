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
		public const byte MessageLengthMinimum = 0;
		public const byte MessageLengthMaximum = 128;

		private const byte RegisterAddressReadMask = 0X7f;
		private const byte RegisterAddressWriteMask = 0x80;

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
		private readonly Object _regFifoLock = new object();
		private double _frequency = Configuration.FrequencyDefault;
		private bool _rxDoneIgnoreIfCrcMissing = true;
		private bool _rxDoneIgnoreIfCrcInvalid = true;

		public SX127XDevice(SpiDevice spiDevice, GpioController gpioController,
			int dio0Pin,
			int resetPin = 0, // Odd order so as not to break exisiting code
			int dio1Pin = 0,
			int dio2Pin = 0,
			int dio3Pin = 0,
			int dio4Pin = 0,
			int dio5Pin = 0
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
			Byte regVersionValue = _registerManager.ReadByte((byte)Configuration.Registers.RegVersion);
			if (regVersionValue != Configuration.RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			// See Table 18 DIO Mapping LoRa® Mode
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

		public void SetMode(Configuration.RegOpModeMode mode)
		{
			Configuration.RegOpModeModeFlags regOpModeValue;

			regOpModeValue = Configuration.RegOpModeModeFlags.LongRangeModeLoRa;
			regOpModeValue |= Configuration.RegOpModeModeFlags.AcessSharedRegLoRa;
			if (_frequency > Configuration.SX127XMidBandThreshold)
			{
				regOpModeValue |= Configuration.RegOpModeModeFlags.LowFrequencyModeOnHighFrequency;
			}
			else
			{
				regOpModeValue |= Configuration.RegOpModeModeFlags.LowFrequencyModeOnLowFrequency;
			}
			regOpModeValue |= (Configuration.RegOpModeModeFlags)mode;
			_registerManager.WriteByte((byte)Configuration.Registers.RegOpMode, (byte)regOpModeValue);
		}

		public void Initialise(double frequency = Configuration.FrequencyDefault, // RegFrMsb, RegFrMid, RegFrLsb
			bool rxDoneignoreIfCrcMissing = true, bool rxDoneignoreIfCrcInvalid = true,
			sbyte outputPower = Configuration.OutputPowerDefault, Configuration.RegPAConfigPASelect powerAmplifier = Configuration.RegPAConfigPASelect.Default, // RegPAConfig & RegPaDac
			Configuration.RegOcp ocpOn = Configuration.RegOcp.Default, Configuration.RegOcpTrim ocpTrim = Configuration.RegOcpTrim.Default, // RegOcp
			Configuration.RegLnaLnaGain lnaGain = Configuration.RegLnaLnaGain.Default, bool lnaBoost = Configuration.LnaBoostDefault, // RegLna
			Configuration.RegModemConfigBandwidth bandwidth = Configuration.RegModemConfigBandwidth.Default, Configuration.RegModemConfigCodingRate codingRate = Configuration.RegModemConfigCodingRate.Default, Configuration.RegModemConfigImplicitHeaderModeOn implicitHeaderModeOn = Configuration.RegModemConfigImplicitHeaderModeOn.Default, //RegModemConfig1
			Configuration.RegModemConfig2SpreadingFactor spreadingFactor = Configuration.RegModemConfig2SpreadingFactor.Default, bool txContinuousMode = false, bool rxPayloadCrcOn = false,
			ushort symbolTimeout = Configuration.SymbolTimeoutDefault,
			ushort preambleLength = Configuration.PreambleLengthDefault,
			byte payloadLength = Configuration.PayloadLengthDefault,
			byte payloadMaxLength = Configuration.PayloadMaxLengthDefault,
			byte freqHoppingPeriod = Configuration.FreqHoppingPeriodDefault,
			bool lowDataRateOptimize = Configuration.LowDataRateOptimizeDefault, bool agcAutoOn = Configuration.AgcAutoOnDefault,
			byte ppmCorrection = Configuration.ppmCorrectionDefault,
			Configuration.RegDetectOptimizeDectionOptimize detectionOptimize = Configuration.RegDetectOptimizeDectionOptimizeDefault,
			Configuration.InvertIqRx invertIqRX = Configuration.InvertIqRx.Default, Configuration.InvertIqTx invertIqTX = Configuration.InvertIqTx.Default,
			Configuration.RegisterDetectionThreshold detectionThreshold = Configuration.RegisterDetectionThresholdDefault,
			byte syncWord = Configuration.RegSyncWordDefault)
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
			SetMode(Configuration.RegOpModeMode.Sleep);

			// Configure RF Carrier frequency 
			if (frequency != Configuration.FrequencyDefault)
			{
				byte[] bytes = BitConverter.GetBytes((long)(frequency / Configuration.SX127X_FSTEP));
				_registerManager.WriteByte((byte)Configuration.Registers.RegFrMsb, bytes[2]);
				_registerManager.WriteByte((byte)Configuration.Registers.RegFrMid, bytes[1]);
				_registerManager.WriteByte((byte)Configuration.Registers.RegFrLsb, bytes[0]);
			}

			// Set RegPAConfig & RegPaDac if powerAmplifier/OutputPower settings not defaults
			if ((powerAmplifier != Configuration.RegPAConfigPASelect.Default) || (outputPower != Configuration.OutputPowerDefault))
			{
				if (powerAmplifier == Configuration.RegPAConfigPASelect.PABoost)
				{
					byte regPAConfigValue = (byte)Configuration.RegPAConfigPASelect.PABoost;

					// Validate the minimum and maximum PABoost outputpower
					if ((outputPower < Configuration.OutputPowerPABoostMin) || (outputPower > Configuration.OutputPowerPABoostMax))
					{
						throw new ApplicationException($"PABoost {outputPower}dBm Min power {Configuration.OutputPowerPABoostMin} to Max power {Configuration.OutputPowerPABoostMax}");
					}

					if (outputPower < Configuration.OutputPowerPABoostPaDacThreshhold)
					{
						// outputPower 0..15 so pOut is 2=17-(15-0)...17=17-(15-15)
						regPAConfigValue |= (byte)Configuration.RegPAConfigMaxPower.Default;
						regPAConfigValue |= (byte)(outputPower - 2);

						_registerManager.WriteByte((byte)Configuration.Registers.RegPAConfig, regPAConfigValue);
						_registerManager.WriteByte((byte)Configuration.Registers.RegPaDac, (byte)Configuration.RegPaDac.Normal);
					}
					else
					{
						// outputPower 0..15 so pOut is 5=20-(15-0)...20=20-(15-15) // See https://github.com/adafruit/RadioHead/blob/master/RH_RF95.cpp around line 411 could be 23dBm
						regPAConfigValue |= (byte)Configuration.RegPAConfigMaxPower.Default;
						regPAConfigValue |= (byte)(outputPower - 5);

						_registerManager.WriteByte((byte)Configuration.Registers.RegPAConfig, regPAConfigValue);
						_registerManager.WriteByte((byte)Configuration.Registers.RegPaDac, (byte)Configuration.RegPaDac.Boost);
					}
				}
				else
				{
					byte regPAConfigValue = (byte)Configuration.RegPAConfigPASelect.Rfo;

					// Validate the minimum and maximum RFO outputPower
					if ((outputPower < Configuration.OutputPowerRfoMin) || (outputPower > Configuration.OutputPowerRfoMax))
					{
						throw new ApplicationException($"RFO {outputPower}dBm Min power {Configuration.OutputPowerRfoMin} to Max power {Configuration.OutputPowerRfoMax}");
					}

					// Set MaxPower and Power calculate pOut = PMax-(15-outputPower), pMax=10.8 + 0.6*MaxPower 
					if (outputPower > Configuration.OutputPowerRfoThreshhold)
					{
						// pMax 15=10.8+0.6*7 with outputPower 0...15 so pOut is 15=pMax-(15-0)...0=pMax-(15-15) 
						regPAConfigValue |= (byte)Configuration.RegPAConfigMaxPower.Max;
						regPAConfigValue |= (byte)(outputPower + 0);
					}
					else
					{
						// pMax 10.8=10.8+0.6*0 with output power 0..15 so pOut is -4=10-(15-0)...10.8=10.8-(15-15)
						regPAConfigValue |= (byte)Configuration.RegPAConfigMaxPower.Min;
						regPAConfigValue |= (byte)(outputPower + 4);
					}

					_registerManager.WriteByte((byte)Configuration.Registers.RegPAConfig, regPAConfigValue);
					_registerManager.WriteByte((byte)Configuration.Registers.RegPaDac, (byte)Configuration.RegPaDac.Normal);
				}
			}

			// Set RegOcp if any of the settings not defaults
			if ((ocpOn != Configuration.RegOcp.Default) || (ocpTrim != Configuration.RegOcpTrim.Default))
			{
				byte regOcpValue = (byte)ocpTrim;

				regOcpValue |= (byte)ocpOn;

				_registerManager.WriteByte((byte)Configuration.Registers.RegOcp, regOcpValue);
			}

			// Set RegLna if any of the settings not defaults
			if ((lnaGain != Configuration.RegLnaLnaGain.Default) || (lnaBoost != Configuration.LnaBoostDefault))
			{
				byte regLnaValue = (byte)lnaGain;

				regLnaValue |= (byte)Configuration.RegLnaLnaBoost.LfDefault;
				regLnaValue |= (byte)Configuration.RegLnaLnaBoost.HfDefault;

				if (lnaBoost)
				{
					if (_frequency > Configuration.SX127XMidBandThreshold)
					{
						regLnaValue |= (byte)Configuration.RegLnaLnaBoost.HfOn;
					}
					else
					{
						regLnaValue |= (byte)Configuration.RegLnaLnaBoost.LfOn;
					}
				}
				_registerManager.WriteByte((byte)Configuration.Registers.RegLna, regLnaValue);
			}

			// Set regModemConfig1 if any of the settings not defaults
			if ((bandwidth != Configuration.RegModemConfigBandwidth.Default) || (codingRate != Configuration.RegModemConfigCodingRate.Default) || (implicitHeaderModeOn != Configuration.RegModemConfigImplicitHeaderModeOn.Default))
			{
				byte regModemConfig1Value = (byte)bandwidth;
				regModemConfig1Value |= (byte)codingRate;
				regModemConfig1Value |= (byte)implicitHeaderModeOn;
				_registerManager.WriteByte((byte)Configuration.Registers.RegModemConfig1, regModemConfig1Value);
			}

			if ((symbolTimeout < Configuration.symbolTimeoutMin) || (symbolTimeout > Configuration.symbolTimeoutMax))
			{
				throw new ArgumentException($"symbolTimeout must be between {Configuration.symbolTimeoutMin} and {Configuration.symbolTimeoutMax}", nameof(symbolTimeout));
			}

			// Set regModemConfig2 if any of the settings not defaults
			if ((spreadingFactor != Configuration.RegModemConfig2SpreadingFactor.Default) || (txContinuousMode != false) | (rxPayloadCrcOn != false) || (symbolTimeout != Configuration.SymbolTimeoutDefault))
			{
				byte RegModemConfig2Value = (byte)spreadingFactor;
				if (txContinuousMode)
				{
					RegModemConfig2Value |= Configuration.RegModemConfig2TxContinuousModeOn;
				}
				if (rxPayloadCrcOn)
				{
					RegModemConfig2Value |= Configuration.RegModemConfig2RxPayloadCrcOn;
				}
				// Get the MSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);

				// Only the zeroth & second bit of byte matter
				symbolTimeoutBytes[1] &= Configuration.SymbolTimeoutMsbMask;
				RegModemConfig2Value |= symbolTimeoutBytes[1];
				_registerManager.WriteByte((byte)Configuration.Registers.RegModemConfig2, RegModemConfig2Value);
			}

			// RegModemConfig2.SymbTimout + RegSymbTimeoutLsb
			if (symbolTimeout != Configuration.SymbolTimeoutDefault)
			{
				// Get the LSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				_registerManager.WriteByte((byte)Configuration.Registers.RegSymbTimeoutLsb, symbolTimeoutBytes[0]);
			}

			// RegPreambleMsb + RegPreambleLsb
			if (preambleLength != Configuration.PreambleLengthDefault)
			{
				_registerManager.WriteWordMsbLsb((Byte)Configuration.Registers.RegPreambleMsb, preambleLength);
			}

			// RegPayloadLength
			if (payloadLength != Configuration.PayloadLengthDefault)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegPayloadLength, payloadLength);
			}

			// RegMaxPayloadLength
			if (payloadMaxLength != Configuration.PayloadMaxLengthDefault)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegMaxPayloadLength, payloadMaxLength);
			}

			// RegHopPeriod
			if (freqHoppingPeriod != Configuration.FreqHoppingPeriodDefault)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegHopPeriod, freqHoppingPeriod);
			}

			// RegModemConfig3
			if ((lowDataRateOptimize != false) || (agcAutoOn != false))
			{
				byte regModemConfig3Value = 0;
				if (lowDataRateOptimize)
				{
					regModemConfig3Value |= Configuration.RegModemConfig3LowDataRateOptimizeOn;
				}
				if (agcAutoOn)
				{
					regModemConfig3Value |= Configuration.RegModemConfig3AgcAutoOn;
				}
				_registerManager.WriteByte((byte)Configuration.Registers.RegModemConfig3, regModemConfig3Value);
			}

			// RegPpmCorrection
			if (ppmCorrection != Configuration.ppmCorrectionDefault)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegPpmCorrection, ppmCorrection);
			}

			// RegDetectOptimize
			if (detectionOptimize != Configuration.RegDetectOptimizeDectionOptimizeDefault)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegDetectOptimize, (byte)detectionOptimize);
			}

			// TX & RX inversion plus optimisation specialness
			if ((invertIqRX != Configuration.InvertIqRx.Default) || (invertIqTX != Configuration.InvertIqTx.Default))
			{
				byte regInvertIqValue = Configuration.RegInvertIqDefault;

				if (invertIqRX == Configuration.InvertIqRx.On)
				{
					regInvertIqValue |= (byte)Configuration.InvertIqRx.On;
				}

				if (invertIqTX == Configuration.InvertIqTx.On)
				{
					regInvertIqValue |= (byte)Configuration.InvertIqTx.On;
				}

				_registerManager.WriteByte((byte)Configuration.Registers.RegInvertIq, regInvertIqValue);

				if ((invertIqRX == Configuration.InvertIqRx.On) || (invertIqTX == Configuration.InvertIqTx.On))
				{
					_registerManager.WriteByte((byte)Configuration.Registers.RegInvertIq2, (byte)Configuration.RegInvertIq2.On);
				}
				else
				{
					_registerManager.WriteByte((byte)Configuration.Registers.RegInvertIq2, (byte)Configuration.RegInvertIq2.Off);
				}
			}

			// RegSyncWordDefault 
			if (syncWord != Configuration.RegSyncWordDefault)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegSyncWord, syncWord);
			}
		}

		private void InterruptGpioPin_ValueChanged(object sender, PinValueChangedEventArgs pinValueChangedEventArgs)
		{
			Byte regIrqFlagsToClear = (byte)Configuration.RegIrqFlags.ClearNone;

			// Read RegIrqFlags to see what caused the interrupt
			Byte irqFlags = _registerManager.ReadByte((byte)Configuration.Registers.RegIrqFlags);

			//Console.WriteLine($"IrqFlags 0x{irqFlags:x} Pin:{pinValueChangedEventArgs.PinNumber}");

			// Check RxTimeout for inbound message
			if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.RxTimeoutMask) == (byte)Configuration.RegIrqFlags.RxTimeout)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegIrqFlags, (byte)Configuration.RegIrqFlags.RxTimeout);

				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.RxTimeout;

				ProcessRxTimeout(irqFlags);
			}

			// Check RxDone for inbound message
			if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.RxDoneMask) == (byte)Configuration.RegIrqFlags.RxDone)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegIrqFlags, (byte)Configuration.RegIrqFlags.RxDone);

				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.RxDone;

				ProcessRxDone(irqFlags);
			}

			// Check PayLoadCrcError for inbound message
			if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.PayLoadCrcErrorMask) == (byte)Configuration.RegIrqFlags.PayLoadCrcError)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegIrqFlags, (byte)Configuration.RegIrqFlags.PayLoadCrcError);

				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.PayLoadCrcError;

				ProcessPayloadCrcError(irqFlags);
			}

			// Check ValidHeader for inbound message
			if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.ValidHeaderMask) == (byte)Configuration.RegIrqFlags.ValidHeader)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegIrqFlags, (byte)Configuration.RegIrqFlags.ValidHeader);

				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.ValidHeader;

				ProcessValidHeader(irqFlags);
			}

			// Check TxDone for outbound message
			if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.TxDoneMask) == (byte)Configuration.RegIrqFlags.TxDone)
			{
				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.TxDone;

				ProcessTxDone(irqFlags);
			}

			// Check Channel Activity Detection done 
			if (((irqFlags & (byte)Configuration.RegIrqFlagsMask.CadDoneMask) == (byte)Configuration.RegIrqFlags.CadDone))
			{
				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.CadDone;

				ProcessChannelActivityDetectionDone(irqFlags);
			}

			// Check FhssChangeChannel for inbound message
			if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.FhssChangeChannelMask) == (byte)Configuration.RegIrqFlags.FhssChangeChannel)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegIrqFlags, (byte)Configuration.RegIrqFlags.FhssChangeChannel);

				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.FhssChangeChannel;

				ProcessFhssChangeChannel(irqFlags);
			}

			// Check Channel Activity Detected 
			if (((irqFlags & (byte)Configuration.RegIrqFlagsMask.CadDetectedMask) == (byte)Configuration.RegIrqFlags.CadDetected))
			{
				regIrqFlagsToClear |= (byte)Configuration.RegIrqFlags.TxDone;

				ProcessChannelActivityDetected(irqFlags);
			}

			_registerManager.WriteByte((byte)Configuration.Registers.RegIrqFlags, regIrqFlagsToClear);
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
				byte regHopChannel = _registerManager.ReadByte((byte)Configuration.Registers.RegHopChannel);
				if ((regHopChannel & (byte)Configuration.RegHopChannelMask.CrcOnPayload) != (byte)Configuration.RegHopChannelFlags.CrcOnPayload)
				{
					return;
				}
			}

			// Check to see if payload CRC is valid
			if (_rxDoneIgnoreIfCrcInvalid)
			{
				if ((irqFlags & (byte)Configuration.RegIrqFlagsMask.PayLoadCrcErrorMask) == (byte)Configuration.RegIrqFlagsMask.PayLoadCrcErrorMask)
				{
					return;
				}
			}

			// Extract the message from the RFM9X fifo, try and keep lock in place for the minimum possible time
			lock (_regFifoLock)
			{
				byte currentFifoAddress = _registerManager.ReadByte((byte)Configuration.Registers.RegFifoRxCurrent);

				_registerManager.WriteByte((byte)Configuration.Registers.RegFifoAddrPtr, currentFifoAddress);

				byte numberOfBytes = _registerManager.ReadByte((byte)Configuration.Registers.RegRxNbBytes);

				payloadBytes = _registerManager.ReadBytes((byte)Configuration.Registers.RegFifo, numberOfBytes);
			}

			// Get the RSSI HF vs. LF port adjustment section 5.5.5 RSSI and SNR in LoRa Mode
			float packetSnr = _registerManager.ReadByte((byte)Configuration.Registers.RegPktSnrValue) * 0.25f;

			int rssi = _registerManager.ReadByte((byte)Configuration.Registers.RegRssiValue);
			if (_frequency > Configuration.SX127XMidBandThreshold)
			{
				rssi = Configuration.RssiAdjustmentHF + rssi;
			}
			else
			{
				rssi = Configuration.RssiAdjustmentLF + rssi;
			}

			int packetRssi = _registerManager.ReadByte((byte)Configuration.Registers.RegPktRssiValue);
			if (_frequency > Configuration.SX127XMidBandThreshold)
			{
				packetRssi = Configuration.RssiAdjustmentHF + packetRssi;
			}
			else
			{
				packetRssi = Configuration.RssiAdjustmentLF + packetRssi;
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
			_registerManager.WriteByte((byte)Configuration.Registers.RegDioMapping1, (byte)Configuration.RegDioMapping1.Dio0RxDone);

			SetMode(Configuration.RegOpModeMode.ReceiveContinuous);
		}

		public void Send(byte[] messageBytes)
		{
			Debug.Assert(messageBytes != null);
			Debug.Assert(messageBytes.Length >= MessageLengthMinimum);
			Debug.Assert(messageBytes.Length <= MessageLengthMaximum);

			lock (_regFifoLock)
			{
				_registerManager.WriteByte((byte)Configuration.Registers.RegFifoTxBaseAddr, 0x0);

				// Set the Register Fifo address pointer
				_registerManager.WriteByte((byte)Configuration.Registers.RegFifoAddrPtr, 0x0);

				_registerManager.WriteBytes((byte)Configuration.Registers.RegFifo, messageBytes);

				// Set the length of the message in the fifo
				_registerManager.WriteByte((byte)Configuration.Registers.RegPayloadLength, (byte)messageBytes.Length);
			}

			_registerManager.WriteByte((byte)Configuration.Registers.RegDioMapping1, (byte)Configuration.RegDioMapping1.Dio0TxDone);
			SetMode(Configuration.RegOpModeMode.Transmit);
		}

    public void ChannelActivityDetect()
		{
			_registerManager.WriteByte((byte)Configuration.Registers.RegDioMapping1, (byte)Configuration.RegDioMapping1.Dio1CadDetect);

			SetMode(Configuration.RegOpModeMode.ChannelActivityDetection);
    }

		public byte Random()
		{
			return _registerManager.ReadByte((byte)Configuration.Registers.RegRssiWideband);
		}

		public void RegisterDump()
		{
			Debug.WriteLine("Register dump");
			for (byte registerIndex = (byte)Configuration.Registers.Minimum; registerIndex <= (byte)Configuration.Registers.Maximum; registerIndex++)
			{
				byte registerValue = _registerManager.ReadByte(registerIndex);

				Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
			}

			Debug.WriteLine("");
		}
	}
}
