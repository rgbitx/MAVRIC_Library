/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file sonar_i2cxl.c
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This file is the driver for the integrated 3axis gyroscope and
 * accelerometer LSM330DLC
 *
 ******************************************************************************/


#include "drivers/mpu_6050.hpp"
#include "hal/common/time_keeper.hpp"

//TODO remove
#include <libopencm3/stm32/gpio.h>


const uint8_t MPU_6050_ADDRESS      = 0xD0;
const uint8_t WHO_ARE_YOU_COMMAND   = 0x75;
const uint8_t I_AM_MPU_6050         = 0x68;
const uint8_t MPU_6050_GET_ACC      = 0x3B;
const uint8_t MPU_6050_GET_GYROS    = 0x43;

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mpu_6050::Mpu_6050(I2c& i2c):
    i2c_(i2c),
    gyro_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    acc_data_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    temperature_(0.0f)
{}


bool Mpu_6050::init(void)
{
    bool success = false;

    //check if device connected
    if (!i2c_.probe(MPU_6050_ADDRESS))
    {
     gpio_set(GPIOD, GPIO15);
     return success;
    }

    //who I am ?
    uint8_t data;
    i2c_.write(&WHO_ARE_YOU_COMMAND, 1, MPU_6050_ADDRESS);
    i2c_.read(&data, 1, MPU_6050_ADDRESS);
    
    if (data != I_AM_MPU_6050)
    {
     gpio_set(GPIOD, GPIO15);
     return success;
    }

    //wakeup MPU
    wake_up_MPU6050(MPU_6050_ADDRESS);
    
    success = true;
    return success;
}


bool Mpu_6050::update_acc(void)
{
    bool success = true;

    uint8_t accel_buffer[6] = {0, 0, 0, 0, 0, 0};

    // // Read data from accelero sensor
    i2c_.write(&MPU_6050_GET_ACC, 1, MPU_6050_ADDRESS);
    i2c_.read(accel_buffer, 6, MPU_6050_ADDRESS);
    // success &= i2c_.write(&LSM_ACC_DATA_BEGIN, 1, LSM330_ACC_SLAVE_ADDRESS);
    // success &= i2c_.read((uint8_t*)accel_buffer, 7, LSM330_ACC_SLAVE_ADDRESS);

    acc_data_[0] = (float)((int16_t)(accel_buffer[0] << 8 | accel_buffer[1]));
    acc_data_[1] = (float)((int16_t)(accel_buffer[2] << 8 | accel_buffer[3]));
    acc_data_[2] = (float)((int16_t)(accel_buffer[4] << 8 | accel_buffer[5]));

    // Save last update time
    last_update_us_ = time_keeper_get_us();

    return success;
}


bool Mpu_6050::update_gyr(void)
{
    bool success = true;

    uint8_t gyro_buffer[6] = {0, 0, 0, 0, 0, 0};

    // // Read data from gyro sensor
    i2c_.write(&MPU_6050_GET_GYROS, 1, MPU_6050_ADDRESS);
    i2c_.read(gyro_buffer, 6, MPU_6050_ADDRESS);
    // success &= i2c_.write(&LSM_GYRO_DATA_BEGIN, 1, LSM330_GYRO_SLAVE_ADDRESS);
    // success &= i2c_.read((uint8_t*)gyro_buffer, 8, LSM330_GYRO_SLAVE_ADDRESS);

    // sensor temperature
    // temperature_  = (float)((int16_t)gyro_buffer[0]);
    
    gyro_data_[0] = (float)((int16_t)(gyro_buffer[0] << 8 | gyro_buffer[1]));
    gyro_data_[1] = (float)((int16_t)(gyro_buffer[2] << 8 | gyro_buffer[3]));
    gyro_data_[2] = (float)((int16_t)(gyro_buffer[4] << 8 | gyro_buffer[5]));

    // Save last update time
    last_update_us_ = time_keeper_get_us();

    return success;
}


const float& Mpu_6050::last_update_us(void) const
{
    return last_update_us_;
}


const std::array<float, 3>& Mpu_6050::gyro(void) const
{
    return gyro_data_;
}


const float& Mpu_6050::gyro_X(void) const
{
    return gyro_data_[0];
}


const float& Mpu_6050::gyro_Y(void) const
{
    return gyro_data_[1];
}


const float& Mpu_6050::gyro_Z(void) const
{
    return gyro_data_[2];
}


const std::array<float, 3>& Mpu_6050::acc(void) const
{
    return acc_data_;
}


const float& Mpu_6050::acc_X(void) const
{
    return acc_data_[0];
}


const float& Mpu_6050::acc_Y(void) const
{
    return acc_data_[1];
}


const float& Mpu_6050::acc_Z(void) const
{
    return acc_data_[2];
}


const float& Mpu_6050::temperature(void) const
{
    return temperature_;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Mpu_6050::wake_up_MPU6050(const uint8_t address)
{
    uint8_t buffer[2] = {0x6B, 0x00};

    i2c_.write(buffer, 2, address);
}
