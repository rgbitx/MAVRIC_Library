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
 * \file position_estimation_telemetry.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the position estimation
 *
 ******************************************************************************/


#ifndef POSIION_ESTIMATION_TELEMETRY_H_
#define POSIION_ESTIMATION_TELEMETRY_H_

#include "communication/mavlink_stream.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "sensing/position_estimation.hpp"

/**
 * \brief   Initialize the MAVLink communication module for the position estimation
 *
 * \param   pos_est                 The pointer to the position estimation structure
 * \param   mavlink_handler         The pointer to the MAVLink message handler
 *
 * \return  True if the init succeed, false otherwise
 */
bool position_estimation_telemetry_init(Position_estimation* pos_est, Mavlink_message_handler* mavlink_handler);

/**
 * \brief   Function to send the MAVLink position estimation message
 *
 * \param   pos_est                 The pointer to the position estimation structure
 * \param   mavlink_stream          The pointer to the MAVLink stream structure
 * \param   msg                     The pointer to the MAVLink message
 */
void position_estimation_telemetry_send_position(const Position_estimation* pos_est, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);


/**
 * \brief   Function to send the MAVLink GPS global position message
 *
 * \param   pos_est                 The pointer to the position estimation structure
 * \param   mavlink_stream          The pointer to the MAVLink stream structure
 * \param   msg                     The pointer to the MAVLink message
 */
void position_estimation_telemetry_send_global_position(const Position_estimation* pos_est, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);


#endif /* POSIION_ESTIMATION_TELEMETRY_H_ */