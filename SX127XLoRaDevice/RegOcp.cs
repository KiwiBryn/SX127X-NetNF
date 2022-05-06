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
	// RegOcp settings from Semtech SX127X Datasheet
	public enum RegOcp
	{
		On = 0b00100000,
		Off = 0b00000000,
		Default = On
	}

	// RegOcpTrim - precalculating seemed a lot easier than doing on the fly with two different ranges
	public enum RegOcpTrim
	{
		Minimum = _45mA,
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
}