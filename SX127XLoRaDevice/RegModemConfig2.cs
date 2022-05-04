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

}