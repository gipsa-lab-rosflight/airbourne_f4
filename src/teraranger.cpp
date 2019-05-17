/*
 * Copyright (c) 2017, James Jackson amd Trey Henrichsen
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

#include "teraranger.h"

TeraRanger* terarangerPtr;

void _TeraRanger_start_read_cb(uint8_t result)
{
  (void)result;
  terarangerPtr->cb_start_read(result);
}

void _TeraRanger_finished_read_cb(uint8_t result)
{
  (void)result;
  terarangerPtr->cb_finished_read(result);
}

TeraRanger::TeraRanger()
{
  terarangerPtr = this;
}

bool TeraRanger::init(I2C *_i2c)
{
  i2c_ = _i2c;
  new_data_ = false;
  value_ = 0;
  last_update_ms_ = millis();
  ready_to_ping_ = true;

  uint8_t id=0;
  uint8_t res=0;
  if (i2c_->write(TERARANGER_DEFAULT_ADDRESS, 0xFF, (uint8_t)TERARANGER_TRIGGER_READING) == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    last_callback_ms_ = millis();
  }
  else
  {
    sensor_present_ = false;
    last_callback_ms_ = 0;
  }
  return sensor_present_;
}

bool TeraRanger::present()
{
  if (sensor_present_ && millis() > last_callback_ms_ + 500)
  {
    sensor_present_ = false;
  }
  return sensor_present_;
}

// Tries to either start a measurement, or read it from the sensor
// Does nothing if it has done something in the last UPDATE_WAIT_MILLIS ms
// Feel free to call it more often, though.
void TeraRanger::update()
{
  uint64_t now=millis();
  if (now > (last_update_ms_ + TERARANGER_UPDATE_WAIT_MILLIS))
  {
    last_update_ms_ = now;
    if (ready_to_ping_)
    {
      i2c_->write(TERARANGER_DEFAULT_ADDRESS, 0xFF, TERARANGER_TRIGGER_READING, &_TeraRanger_start_read_cb);
    }else{
      i2c_->read(TERARANGER_DEFAULT_ADDRESS, 0xFF, 3, buffer_, &_TeraRanger_finished_read_cb);
    }
  }
}

//Returns the most recent reading
//It is during this method that the reading is converted to meters, as well
//If there has not yet been a successful reading, returns 0
float TeraRanger::read()
{
  if (new_data_)
  {
    // todo check CRC
    
    uint16_t millimeters = buffer_[0] << 8 | buffer_[1];
    
    value_=(float)millimeters * 0.001;
    
    new_data_=false;
  }
  return value_;
}

//callback after the measure command has been sent to the sensor
void TeraRanger::cb_start_read(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    last_callback_ms_ = millis();
    ready_to_ping_ = false;
  }else{
    ready_to_ping_ = true;    
  }
}

//callback after reading from the sensor has finished
void TeraRanger::cb_finished_read(uint8_t result)
{
  if (result == I2C::RESULT_SUCCESS)
  {
    sensor_present_ = true;
    new_data_ = true;
    last_callback_ms_ = millis();
    ready_to_ping_ = true;
  }else{
    ready_to_ping_ = true;    
  }
}
