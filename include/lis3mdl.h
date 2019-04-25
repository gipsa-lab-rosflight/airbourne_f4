/*
 * Copyright (c) 2019 James Jackson, Amaury Negre
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

#pragma once

#include "system.h"
#include "i2c.h"
#define LIS3MDL_WHO_AM_I    0x0F
#define LIS3MDL_CTRL_REG1   0x20
#define LIS3MDL_CTRL_REG2   0x21
#define LIS3MDL_CTRL_REG3   0x22
#define LIS3MDL_CTRL_REG4   0x23
#define LIS3MDL_CTRL_REG5   0x24
#define LIS3MDL_STATUS_REG  0x27
#define LIS3MDL_OUT_X_L     0x28
#define LIS3MDL_OUT_X_H     0x29
#define LIS3MDL_OUT_Y_L     0x2A
#define LIS3MDL_OUT_Y_H     0x2B
#define LIS3MDL_OUT_Z_L     0x2C
#define LIS3MDL_OUT_Z_H     0x2D
#define LIS3MDL_TEMP_OUT_L  0x2E
#define LIS3MDL_TEMP_OUT_H  0x2F
#define LIS3MDL_INT_CFG     0x30
#define LIS3MDL_INT_SRC     0x31
#define LIS3MDL_INT_THS_L   0x32
#define LIS3MDL_INT_THS_H   0x33

#define LIS3MDL_SA1_HIGH_ADDRESS  0x1E
#define LIS3MDL_SA1_LOW_ADDRESS   0x1C

#define LIS3MDL_WHO_ID  0x3D

class LIS3MDL
{
public:
  bool init(I2C* i2c_drv, uint8_t addr = LIS3MDL_SA1_LOW_ADDRESS);
  void update();
  bool read(float mag_data[]);
  bool present();
  void cb(uint8_t result);
  
private:
  I2C* i2c_;
  uint8_t addr_;
  uint8_t i2c_buf_[6];
  volatile float data_[3];
  uint32_t last_update_ms_;
  uint32_t next_update_ms_;
  bool mag_present_;
};
