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

		public class OnDataReceivedEventArgs : EventArgs
		{
			public float PacketSnr { get; set; }
			public int PacketRssi { get; set; }
			public int Rssi { get; set; }
			public byte[] Data { get; set; }
		}
		public delegate void onReceivedEventHandler(Object sender, OnDataReceivedEventArgs e);
		public event onReceivedEventHandler OnReceive;

		public class OnDataTransmitedEventArgs : EventArgs
		{
			public byte[] Data { get; set; }
		}
		public delegate void onTransmittedEventHandler(Object sender, OnDataTransmitedEventArgs e);
		public event onTransmittedEventHandler OnTransmit;


		// Hardware configuration support
		private readonly int _resetPin;
		private readonly GpioController _gpioController = null;
		private readonly RegisterManager _registerManager = null;
		private readonly Object _regFifoLock = new object();
		private double _frequency = SX127xConfiguration.FrequencyDefault;
		private bool _rxDoneIgnoreIfCrcMissing = true;
		private bool _rxDoneIgnoreIfCrcInvalid = true;

		public SX127XDevice(SpiDevice spiDevice, GpioController gpioController, int interruptPin, int resetPin)
		{
			_registerManager = new RegisterManager(spiDevice, RegisterAddressReadMask, RegisterAddressWriteMask);

			// As soon as ChipSelectLine/ChipSelectLogicalPinNumber check that SX127X chip is present
			Byte regVersionValue = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegVersion);
			if (regVersionValue != SX127xConfiguration.RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			_gpioController = gpioController;

			// Factory reset pin configuration
			_resetPin = resetPin;
			_gpioController.OpenPin(resetPin, PinMode.Output);

			_gpioController.Write(resetPin, PinValue.Low);
			Thread.Sleep(20);
			_gpioController.Write(resetPin, PinValue.High);
			Thread.Sleep(20);

			// Interrupt pin for RX message & TX done notification 
			_gpioController.OpenPin(interruptPin, PinMode.InputPullDown);

			_gpioController.RegisterCallbackForPinValueChangedEvent(interruptPin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
		}

		public SX127XDevice(SpiDevice spiDevice, GpioController gpioController, int interruptPin)
		{
			_registerManager = new RegisterManager(spiDevice, RegisterAddressReadMask, RegisterAddressWriteMask);

			_gpioController = gpioController;

			// As soon as ChipSelectLine/ChipSelectLogicalPinNumber check that SX127X chip is present
			Byte regVersionValue = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegVersion);
			if (regVersionValue != SX127xConfiguration.RegVersionValueExpected)
			{
				throw new ApplicationException("Semtech SX127X not found");
			}

			// Interrupt pin for RX message & TX done notification 
			_gpioController.OpenPin(interruptPin, PinMode.InputPullDown);

			_gpioController.RegisterCallbackForPinValueChangedEvent(interruptPin, PinEventTypes.Rising, InterruptGpioPin_ValueChanged);
		}

		public void SetMode(SX127xConfiguration.RegOpModeMode mode)
		{
			byte regOpModeValue;

			regOpModeValue = SX127xConfiguration.RegOpModeLongRangeModeLoRa;
			regOpModeValue |= SX127xConfiguration.RegOpModeAcessSharedRegLoRa;
			if (_frequency > SX127xConfiguration.SX127XMidBandThreshold)
			{
				regOpModeValue |= SX127xConfiguration.RegOpModeLowFrequencyModeOnHighFrequency;
			}
			else
			{
				regOpModeValue |= SX127xConfiguration.RegOpModeLowFrequencyModeOnLowFrequency;
			}
			regOpModeValue |= (byte)mode;
			_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegOpMode, regOpModeValue);
		}

		public void Initialise(SX127xConfiguration.RegOpModeMode modeAfterInitialise, // RegOpMode
			double frequency = SX127xConfiguration.FrequencyDefault, // RegFrMsb, RegFrMid, RegFrLsb
			bool rxDoneignoreIfCrcMissing = true, bool rxDoneignoreIfCrcInvalid = true,
			sbyte outputPower = SX127xConfiguration.OutputPowerDefault, SX127xConfiguration.PowerAmplifier powerAmplifier = SX127xConfiguration.PowerAmplifierDefault, // RegPAConfig & RegPaDac
			bool ocpOn = SX127xConfiguration.RegOcpDefault, byte ocpTrim = SX127xConfiguration.RegOcpOcpTrimDefault, // RegOcp
			SX127xConfiguration.RegLnaLnaGain lnaGain = SX127xConfiguration.LnaGainDefault, bool lnaBoost = SX127xConfiguration.LnaBoostDefault, // RegLna
			SX127xConfiguration.RegModemConfigBandwidth bandwidth = SX127xConfiguration.RegModemConfigBandwidthDefault, SX127xConfiguration.RegModemConfigCodingRate codingRate = SX127xConfiguration.RegModemConfigCodingRateDefault, SX127xConfiguration.RegModemConfigImplicitHeaderModeOn implicitHeaderModeOn = SX127xConfiguration.RegModemConfigImplicitHeaderModeOnDefault, //RegModemConfig1
			SX127xConfiguration.RegModemConfig2SpreadingFactor spreadingFactor = SX127xConfiguration.RegModemConfig2SpreadingFactorDefault, bool txContinuousMode = false, bool rxPayloadCrcOn = false,
			ushort symbolTimeout = SX127xConfiguration.SymbolTimeoutDefault,
			ushort preambleLength = SX127xConfiguration.PreambleLengthDefault,
			byte payloadLength = SX127xConfiguration.PayloadLengthDefault,
			byte payloadMaxLength = SX127xConfiguration.PayloadMaxLengthDefault,
			byte freqHoppingPeriod = SX127xConfiguration.FreqHoppingPeriodDefault,
			bool lowDataRateOptimize = SX127xConfiguration.LowDataRateOptimizeDefault, bool agcAutoOn = SX127xConfiguration.AgcAutoOnDefault,
			byte ppmCorrection = SX127xConfiguration.ppmCorrectionDefault,
			SX127xConfiguration.RegDetectOptimizeDectionOptimize detectionOptimize = SX127xConfiguration.RegDetectOptimizeDectionOptimizeDefault,
			bool invertIQRX = SX127xConfiguration.InvertIqRXDefault, bool invertIQTX = SX127xConfiguration.InvertIqTXDefault,
			SX127xConfiguration.RegisterDetectionThreshold detectionThreshold = SX127xConfiguration.RegisterDetectionThresholdDefault,
			byte syncWord = SX127xConfiguration.RegSyncWordDefault)
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
				Thread.Sleep(20);
			}

			// Put the device into sleep mode so registers can be changed
			SetMode(SX127xConfiguration.RegOpModeMode.Sleep);

			// Configure RF Carrier frequency 
			if (frequency != SX127xConfiguration.FrequencyDefault)
			{
				byte[] bytes = BitConverter.GetBytes((long)(frequency / SX127xConfiguration.SX127X_FSTEP));
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegFrMsb, bytes[2]);
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegFrMid, bytes[1]);
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegFrLsb, bytes[0]);
			}

			// Validate the OutputPower
			if (powerAmplifier == SX127xConfiguration.PowerAmplifier.Rfo)
			{
				if ((outputPower < SX127xConfiguration.OutputPowerRfoMin) || (outputPower > SX127xConfiguration.OutputPowerRfoMax))
				{
					throw new ArgumentException($"outputPower must be between {SX127xConfiguration.OutputPowerRfoMin} and {SX127xConfiguration.OutputPowerRfoMax}", nameof(outputPower));
				}
			}
			if (powerAmplifier == SX127xConfiguration.PowerAmplifier.PABoost)
			{
				if ((outputPower < SX127xConfiguration.OutputPowerPABoostMin) || (outputPower > SX127xConfiguration.OutputPowerPABoostMax))
				{
					throw new ArgumentException($"outputPower must be between {SX127xConfiguration.OutputPowerPABoostMin} and {SX127xConfiguration.OutputPowerPABoostMax}", nameof(outputPower));
				}
			}

			if ((powerAmplifier != SX127xConfiguration.PowerAmplifierDefault) || (outputPower != SX127xConfiguration.OutputPowerDefault))
			{
				byte regPAConfigValue = SX127xConfiguration.RegPAConfigMaxPowerMax;

				if (powerAmplifier == SX127xConfiguration.PowerAmplifier.Rfo)
				{
					regPAConfigValue |= SX127xConfiguration.RegPAConfigPASelectRfo;

					regPAConfigValue |= (byte)(outputPower + 1);

					_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPAConfig, regPAConfigValue);
				}

				if (powerAmplifier == SX127xConfiguration.PowerAmplifier.PABoost)
				{
					regPAConfigValue |= SX127xConfiguration.RegPAConfigPASelectPABoost;

					if (outputPower > SX127xConfiguration.RegPaDacPABoostThreshold)
					{
						_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPaDac, (byte)SX127xConfiguration.RegPaDac.Boost);

						regPAConfigValue |= (byte)(outputPower - 8);

						_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPAConfig, regPAConfigValue);
					}
					else
					{
						_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPaDac, (byte)SX127xConfiguration.RegPaDac.Normal);

						regPAConfigValue |= (byte)(outputPower - 5);

						_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPAConfig, regPAConfigValue);
					}
				}
			}

			// Set RegOcp if any of the settings not defaults
			if ((ocpOn != true) || (ocpTrim != SX127xConfiguration.RegOcpOcpTrimDefault))
			{
				byte regOcpValue = ocpTrim;
				if (ocpOn)
				{
					regOcpValue |= SX127xConfiguration.RegOcpOn;
				}
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegOcp, regOcpValue);
			}

			// Set RegLna if any of the settings not defaults
			if ((lnaGain != SX127xConfiguration.LnaGainDefault) || (lnaBoost != false))
			{
				byte regLnaValue = (byte)lnaGain;
				if (lnaBoost)
				{
					if (_frequency > SX127xConfiguration.SX127XMidBandThreshold)
					{
						regLnaValue |= SX127xConfiguration.RegLnaLnaBoostHfOn;
					}
					else
					{
						regLnaValue |= SX127xConfiguration.RegLnaLnaBoostLfOn;
					}
				}
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegLna, regLnaValue);
			}

			// Set regModemConfig1 if any of the settings not defaults
			if ((bandwidth != SX127xConfiguration.RegModemConfigBandwidthDefault) || (codingRate != SX127xConfiguration.RegModemConfigCodingRateDefault) || (implicitHeaderModeOn != SX127xConfiguration.RegModemConfigImplicitHeaderModeOnDefault))
			{
				byte regModemConfig1Value = (byte)bandwidth;
				regModemConfig1Value |= (byte)codingRate;
				regModemConfig1Value |= (byte)implicitHeaderModeOn;
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegModemConfig1, regModemConfig1Value);
			}

			if ((symbolTimeout < SX127xConfiguration.symbolTimeoutMin) || (symbolTimeout > SX127xConfiguration.symbolTimeoutMax))
			{
				throw new ArgumentException($"symbolTimeout must be between {SX127xConfiguration.symbolTimeoutMin} and {SX127xConfiguration.symbolTimeoutMax}", nameof(symbolTimeout));
			}

			// Set regModemConfig2 if any of the settings not defaults
			if ((spreadingFactor != SX127xConfiguration.RegModemConfig2SpreadingFactorDefault) || (txContinuousMode != false) | (rxPayloadCrcOn != false) || (symbolTimeout != SX127xConfiguration.SymbolTimeoutDefault))
			{
				byte RegModemConfig2Value = (byte)spreadingFactor;
				if (txContinuousMode)
				{
					RegModemConfig2Value |= SX127xConfiguration.RegModemConfig2TxContinuousModeOn;
				}
				if (rxPayloadCrcOn)
				{
					RegModemConfig2Value |= SX127xConfiguration.RegModemConfig2RxPayloadCrcOn;
				}
				// Get the MSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);

				// Only the zeroth & second bit of byte matter
				symbolTimeoutBytes[1] &= SX127xConfiguration.SymbolTimeoutMsbMask;
				RegModemConfig2Value |= symbolTimeoutBytes[1];
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegModemConfig2, RegModemConfig2Value);
			}

			// RegModemConfig2.SymbTimout + RegSymbTimeoutLsb
			if (symbolTimeout != SX127xConfiguration.SymbolTimeoutDefault)
			{
				// Get the LSB of SymbolTimeout
				byte[] symbolTimeoutBytes = BitConverter.GetBytes(symbolTimeout);
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegSymbTimeout, symbolTimeoutBytes[0]);
			}

			// RegPreambleMsb + RegPreambleLsb
			if (preambleLength != SX127xConfiguration.PreambleLengthDefault)
			{
				_registerManager.WriteWordMsbLsb((Byte)SX127xConfiguration.Registers.RegPreambleMsb, preambleLength);
			}

			// RegPayloadLength
			if (payloadLength != SX127xConfiguration.PayloadLengthDefault)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPayloadLength, payloadLength);
			}

			// RegMaxPayloadLength
			if (payloadMaxLength != SX127xConfiguration.PayloadMaxLengthDefault)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegMaxPayloadLength, payloadMaxLength);
			}

			// RegHopPeriod
			if (freqHoppingPeriod != SX127xConfiguration.FreqHoppingPeriodDefault)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegHopPeriod, freqHoppingPeriod);
			}

			// RegModemConfig3
			if ((lowDataRateOptimize != false) || (agcAutoOn != false))
			{
				byte regModemConfig3Value = 0;
				if (lowDataRateOptimize)
				{
					regModemConfig3Value |= SX127xConfiguration.RegModemConfig3LowDataRateOptimizeOn;
				}
				if (agcAutoOn)
				{
					regModemConfig3Value |= SX127xConfiguration.RegModemConfig3AgcAutoOn;
				}
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegModemConfig3, regModemConfig3Value);
			}

			// RegPpmCorrection
			if (ppmCorrection != SX127xConfiguration.ppmCorrectionDefault)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPpmCorrection, ppmCorrection);
			}

			// RegDetectOptimize
			if (detectionOptimize != SX127xConfiguration.RegDetectOptimizeDectionOptimizeDefault)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegDetectOptimize, (byte)detectionOptimize);
			}

			if ((invertIQRX != SX127xConfiguration.InvertIqRXDefault) || (invertIQTX != SX127xConfiguration.InvertIqTXDefault))
			{
				byte regInvertIQValue = SX127xConfiguration.RegInvertIdDefault;

				if (invertIQRX)
				{
					regInvertIQValue |= SX127xConfiguration.InvertIqRXOn;
				}

				if (invertIQTX)
				{
					regInvertIQValue |= SX127xConfiguration.InvertIqTXOn;
				}

				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegInvertIQ, regInvertIQValue);

				if (invertIQRX || invertIQTX)
				{
					_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegInvertIQ2, SX127xConfiguration.RegInvertIq2On);
				}
				else
				{
					_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegInvertIQ2, SX127xConfiguration.RegInvertIq2Off);
				}
			}

			// RegSyncWordDefault 
			if (syncWord != SX127xConfiguration.RegSyncWordDefault)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegSyncWord, syncWord);
			}

			// TODO revist this split & move to onReceive function
			_registerManager.WriteByte(0x40, 0b00000000); // RegDioMapping1 0b00000000 DI0 RxReady & TxReady

			// Configure RegOpMode before returning
			SetMode(modeAfterInitialise);
		}

		private void ProcessTxDone(byte IrqFlags)
		{
			Debug.Assert(IrqFlags != 0);

			OnDataTransmitedEventArgs transmitArgs = new OnDataTransmitedEventArgs();

			OnTransmit?.Invoke(this, transmitArgs);
		}

		private void ProcessRxDone(byte IrqFlags)
		{
			byte[] payloadBytes;
			Debug.Assert(IrqFlags != 0);

			// Check to see if payload has CRC 
			if (_rxDoneIgnoreIfCrcMissing)
			{
				byte regHopChannel = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegHopChannel);
				if ((regHopChannel & (byte)SX127xConfiguration.RegHopChannelMask.CrcOnPayload) != (byte)SX127xConfiguration.RegHopChannelFlags.CrcOnPayload)
				{
					return;
				}
			}

			// Check to see if payload CRC is valid
			if (_rxDoneIgnoreIfCrcInvalid)
			{
				if ((IrqFlags & (byte)SX127xConfiguration.RegIrqFlagsMask.PayLoadCrcErrorMask) == (byte)SX127xConfiguration.RegIrqFlagsMask.PayLoadCrcErrorMask)
				{
					return;
				}
			}

			// Extract the message from the RFM9X fifo, try and keep lock in place for the minimum possible time
			lock (_regFifoLock)
			{
				byte currentFifoAddress = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegFifoRxCurrent);

				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegFifoAddrPtr, currentFifoAddress);

				byte numberOfBytes = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegRxNbBytes);

				payloadBytes = _registerManager.ReadBytes((byte)SX127xConfiguration.Registers.RegFifo, numberOfBytes);
			}

			// Get the RSSI HF vs. LF port adjustment section 5.5.5 RSSI and SNR in LoRa Mode
			float packetSnr = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegPktSnrValue) * 0.25f;

			int rssi = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegRssiValue);
			if (_frequency > SX127xConfiguration.SX127XMidBandThreshold)
			{
				rssi = SX127xConfiguration.RssiAdjustmentHF + rssi;
			}
			else
			{
				rssi = SX127xConfiguration.RssiAdjustmentLF + rssi;
			}

			int packetRssi = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegPktRssiValue);
			if (_frequency > SX127xConfiguration.SX127XMidBandThreshold)
			{
				packetRssi = SX127xConfiguration.RssiAdjustmentHF + packetRssi;
			}
			else
			{
				packetRssi = SX127xConfiguration.RssiAdjustmentLF + packetRssi;
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

		private void InterruptGpioPin_ValueChanged(object sender, PinValueChangedEventArgs pinValueChangedEventArgs)
		{
			// Read RegIrqFlags to see what caused the interrupt
			Byte IrqFlags = _registerManager.ReadByte((byte)SX127xConfiguration.Registers.RegIrqFlags);

			// Check RxDone for inbound message
			if ((IrqFlags & (byte)SX127xConfiguration.RegIrqFlagsMask.RxDoneMask) == (byte)SX127xConfiguration.RegIrqFlags.RxDone)
			{
				ProcessRxDone(IrqFlags);
			}

			// Check TxDone for outbound message
			if ((IrqFlags & (byte)SX127xConfiguration.RegIrqFlagsMask.TxDoneMask) == (byte)SX127xConfiguration.RegIrqFlags.TxDone)
			{
				ProcessTxDone(IrqFlags);
			}

			_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegDioMapping1, (byte)SX127xConfiguration.RegDioMapping1.Dio0RxDone);
			_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegIrqFlags, (byte)SX127xConfiguration.RegIrqFlags.ClearAll);
		}

		public void Receive()
		{
			SetMode(SX127xConfiguration.RegOpModeMode.ReceiveContinuous);
		}

		public void Send(byte[] messageBytes)
		{
			Debug.Assert(messageBytes != null);
			Debug.Assert(messageBytes.Length >= MessageLengthMinimum);
			Debug.Assert(messageBytes.Length <= MessageLengthMaximum);

			lock (_regFifoLock)
			{
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegFifoTxBaseAddr, 0x0);

				// Set the Register Fifo address pointer
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegFifoAddrPtr, 0x0);

				_registerManager.WriteBytes((byte)SX127xConfiguration.Registers.RegFifo, messageBytes);

				// Set the length of the message in the fifo
				_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegPayloadLength, (byte)messageBytes.Length);
			}

			_registerManager.WriteByte((byte)SX127xConfiguration.Registers.RegDioMapping1, (byte)SX127xConfiguration.RegDioMapping1.Dio0TxDone);
			SetMode(SX127xConfiguration.RegOpModeMode.Transmit);
		}

		public void RegisterDump()
		{
			Debug.WriteLine("Register dump");
			for (byte registerIndex = (byte)SX127xConfiguration.Registers.MinValue; registerIndex <= (byte)SX127xConfiguration.Registers.MaxValue; registerIndex++)
			{
				byte registerValue = _registerManager.ReadByte(registerIndex);

				Debug.WriteLine($"Register 0x{registerIndex:x2} - Value 0X{registerValue:x2}");
			}

			Debug.WriteLine("");
		}
	}
}
