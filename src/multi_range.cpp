/*
 * Copyright (c) 2017, Amaury Negre
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "multi_range.h"
//#include "printf.h"

MultiRange* multirangePtr = 0;

void _multirange_range_cb(uint16_t result, bool success)
{
	if(multirangePtr)
		multirangePtr->range_callback(result, success);
}

MultiRange::MultiRange()
{
	multirangePtr = this;
}

bool MultiRange::init(I2C *i2c, uint8_t base_address)
{
	sensor_present_ = false;
	is_updating_ = false;
	nbSensors_ = 0;
	last_update_ms_ = millis();
	
	if(!oneWire_.init(i2c))
	{
		return false;
	}
	oneWire_.deviceReset();
	if(!oneWire_.wireReset())
	{
		return false;
	}

	// switch off all devices
	oneWire_.wireResetSearch();
	while(nbSensors_<MULTI_RANGE_MAX_SENSORS && oneWire_.wireSearch(sensors_[nbSensors_].rom))
	{
		if(sensors_[nbSensors_].rom[0]==0x3A)
		{
			oneWire_.wireReset();
			oneWire_.wireSelect(sensors_[nbSensors_].rom);
			oneWire_.wireWriteByte(0x5A);
			oneWire_.wireWriteByte(0xFD);
			oneWire_.wireWriteByte(0x02);
			oneWire_.wireReset();
			nbSensors_++;
		}
	}
	delay(50);
	
  // switch on and configure each devices
	for(uint8_t k=0; k<nbSensors_; k++)
	{
// 		printf("init device [");
// 		for(int i=0; i<8; i++)
// 		{
// 			printf("%02X", (int) sensors_[k].rom[i]);
// 		}
// 		printf("]\n");
		
		oneWire_.wireReset();
		oneWire_.wireSelect(sensors_[k].rom);
		oneWire_.wireWriteByte(0x5A);
		oneWire_.wireWriteByte(0xFE);
		oneWire_.wireWriteByte(0x01);
		oneWire_.wireReset();

		delay(50);
		sensors_[k].init(i2c);
		sensors_[k].setAddress(base_address+k);
		sensors_[k].setTimeout(200);
		sensors_[k].setVcselPulsePeriod(VcselPeriodPreRange, 18);
		sensors_[k].setVcselPulsePeriod(VcselPeriodFinalRange, 14);
		sensors_[k].setMeasurementTimingBudget(50000);
		
		sensors_[k].startContinuous();
		
		ranges_[k] = 0;
	}
	
	sensor_present_ = nbSensors_>0;
	
	return sensor_present_;
}


bool MultiRange::present()
{
	return sensor_present_;
}

void MultiRange::update()
{
	uint64_t now=millis();
	if((!is_updating_) && (nbSensors_>0) && (now > (last_update_ms_ + MULTIRANGE_UPDATE_WAIT_MILLIS)))
  {
		is_updating_ = true;
		cur_sensor_ = 0;
		last_update_ms_ = now;
		sensors_[cur_sensor_].asyncReadRangeContinuous(_multirange_range_cb);
	}
}

void MultiRange::range_callback(uint16_t result, bool success)
{
	//printf("range_callback : %d\n", result);
	
	ranges_[cur_sensor_] = result;
	cur_sensor_++;
	if(cur_sensor_<nbSensors_)
	{
		sensors_[cur_sensor_].asyncReadRangeContinuous(_multirange_range_cb);
	}
	else
	{
		is_updating_ = false;
	}
}

