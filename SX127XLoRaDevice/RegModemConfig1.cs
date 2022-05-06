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
	// RegModemConfig1 & RegModemConfig2 settings from Semtech SX127X Datasheet
	public enum RegModemConfig1Bandwidth : byte
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

	public enum RegModemConfig1CodingRate
	{
		_4of5 = 0b00000010,
		Default = _4of5,
		_4of6 = 0b00000100,
		_4of7 = 0b00000110,
		_4of8 = 0b00001000,
	}

	public enum RegModemConfig1ImplicitHeaderModeOn
	{
		ExplicitHeaderMode = 0b00000000,
		Default = ExplicitHeaderMode,
		ImplicitHeaderMode = 0b00000001,
	}
}