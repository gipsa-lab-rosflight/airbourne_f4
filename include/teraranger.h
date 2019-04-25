/*
 * Copyright (c) 2017, James Jackson and Trey Henrichsen
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


/*
 * Driver for the Maxbotix I2CXL-MaxSonar-EZ series sonar.
 * This has been made for and tested with the MB1242 sonar module,
 * but should work for the MB1202, MB1212, MB1222, and MB1232.
 */
#ifndef TERARANGER_H
#define TERARANGER_H

#include <cstdint>

#include "i2c.h"

// These 3 constants come from the spec sheet for the sonar
#define TERARANGER_DEFAULT_ADDRESS 0x31
#define TERARANGER_TRIGGER_READING 0x00
#define TERARANGER_WHO_AM_I 0x01
#define TERARANGER_ID 0xA1


#define TERARANGER_UPDATE_WAIT_MILLIS 1 // minimum time between calls of async_update that actually do something

class TeraRanger
{
private:
    uint32_t last_update_ms_; // The last time that async_update was called
    uint32_t last_callback_ms_; // The last time the sensor responded
    float value_; // the latest reading from the sensor
    bool new_data_; // Whether or not new data is ready to be returned
    I2C *i2c_; // The i2c object used for communication
    uint8_t buffer_[3]; // for recieving data from the sensor
    bool sensor_present_; // Flag of whether we have received data from the sensor
    bool ready_to_ping_; // Whether the sensor is ready to make another measurement
    
public:
    TeraRanger();
    bool init(I2C *_i2c);
    bool present();
    float read(); // Returns the most recent reading, converted to meters, or 0 if there is none
    void update(); // Tries to either start a measurement, or read it from the sensor
    // update will do nothing if it has done something in the last TERARANGER_UPDATE_WAIT_MILLIS ms
    // Calling it more frequently won't break anything

    // Callbacks. For internal use only, but public so the I2C peripheral can call them
    void cb_start_read(uint8_t result); // callback after the measure command has been sent to the sensor
    void cb_finished_read(uint8_t result); // callback after reading from the sensor has finished
};


#endif
