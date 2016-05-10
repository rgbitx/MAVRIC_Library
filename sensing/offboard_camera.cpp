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
 * \file offboard_camera.c
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief Offboard camera control
 *
 ******************************************************************************/


#include "sensing/offboard_camera.hpp"
#include "sample_projects/LEQuad/tasks.hpp"
#include "sample_projects/LEQuad/central_data.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "util/constants.h"
#include "util/maths.h"
}



Offboard_Camera::Offboard_Camera(offboard_camera_conf_t config):
    camera_id_(config.camera_id),
    is_camera_running_(config.initial_camera_state),
    last_update_us_(time_keeper_get_us()),
    allowable_horizontal_tag_offset_sqr_(config.allowable_horizontal_tag_offset_sqr),
    max_acc_drone_height_from_camera_mm_(config.max_acc_drone_height_from_camera_mm),
    tag_search_timeout_us_(config.tag_search_timeout_us),
    camera_res_({config.camera_res_x, config.camera_res_y}),
    camera_rotation_(config.camera_rotation),
    camera_fov_({config.camera_fov_x, config.camera_fov_y})

{
    // Set picture count to 0
    picture_count_ = 0.0f;
}


bool Offboard_Camera::update(const Scheduler* scheduler)
{
    bool success = true;
    

    // TEMPORARY...
    // Switch camera on and off
    is_camera_running_ = true;
    // ...TEMPORARY

    // Update timing
    uint32_t t      = time_keeper_get_us();
    //dt_s_           = (float)(t - last_update_us_) / 1000000.0f;
    last_update_us_ = t;

    // Send the message now
    Scheduler_task* camera_send_message_task = scheduler->get_task_by_id(MAVLINK_MSG_ID_COMMAND_LONG);
    camera_send_message_task->run_now();

    return success;
}


const float& Offboard_Camera::last_update_us(void) const
{
    return last_update_us_;
}

const bool& Offboard_Camera::is_camera_running() const
{
    return is_camera_running_;
}

int Offboard_Camera::camera_id() const
{
    return camera_id_;
}

float  Offboard_Camera::allowable_horizontal_tag_offset_sqr() const
{
    return allowable_horizontal_tag_offset_sqr_;
}

float Offboard_Camera::max_acc_drone_height_from_camera_mm() const
{
    return max_acc_drone_height_from_camera_mm_;
}

const float& Offboard_Camera::tag_search_timeout_us() const
{
    return tag_search_timeout_us_;
}

void Offboard_Camera::update_last_update_us()
{
    last_update_us_ = time_keeper_get_us();
}

int Offboard_Camera::camera_x_resolution() const
{
    return camera_res_[0];
}

int Offboard_Camera::camera_y_resolution() const
{
    return camera_res_[1];
}

float Offboard_Camera::camera_rotation() const
{
    return camera_rotation_;
}

float Offboard_Camera::camera_x_fov() const
{
    return camera_fov_[0];
}

float Offboard_Camera::camera_y_fov() const
{
    return camera_fov_[1];
}

local_position_t& Offboard_Camera::tag_location()
{
    return tag_location_;
}

const int& Offboard_Camera::picture_count() const
{
    return picture_count_;
}

void Offboard_Camera::increment_picture_count()
{
    picture_count_++;
}