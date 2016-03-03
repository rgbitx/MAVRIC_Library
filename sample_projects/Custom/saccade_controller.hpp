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
 * \file saccade_controller.hpp
 *
 * \author Darius Merk
 *
 * \brief   Saccade controller for indoors collision free navigation based on optic flow
 *
 ******************************************************************************/


#ifndef SACCADE_CONTROLLER_H_
#define SACCADE_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control/control_command.h"
#include "util/quaternions.h"
#include "util/coord_conventions.h"

#ifdef __cplusplus
}

#include "drivers/flow.hpp"


/**
 * \brief Configuration structure
 */
typedef struct
{
    float pitch;
    float gain;
    float threshold;
    float goal_direction;
} saccade_controller_conf_t;


static inline saccade_controller_conf_t saccade_controller_default_config(void);

/**
 * \Saccade controller structure
 */
class Saccade_controller
{
public:
    
    flow_t                      flow_left;           ///< Left optic flow camera output
    flow_t                      flow_right;          ///< Right optic flow camera output
    float                       pitch;              ///< Pitch command for forward motion
    float                       gain;               ///< Gain for importance of CAN
    float                       threshold;          ///< Threshold for importance of CAN
    float                       goal_direction;     ///< Goal direction for drone
    attitude_command_t*         attitude_command;   ///< Attitude command given by the necessary saccade
  
    
    /**
     * \brief                       Constructor
     *
     * \param   serial_flow_left    Serial port for left optic flow camera
     * \param   serial_flow_right   Serial port for right optic flow cameras
     * \param   config              Configuration structure
     */
    Saccade_controller(Serial& serial_flow_left, Serial& serial_flow_right, saccade_controller_conf_t config);


    /**
     * \brief   Init function
     * 
     * \return  success
     */
    bool init(void);


    /**
     * \brief                       Update of the saccade control
     *
     */
    bool update(void);
    
};



static inline saccade_controller_conf_t saccade_controller_default_config(void)
{
    saccade_controller_conf_t conf;

    conf.pitch          = 0.0f;
    conf.gain           = 1.0f;
    conf.threshold      = 1.0f;
    conf.goal_direction = 0.0f;

    return conf;
};





#endif

#endif /* ALTITUDE_CONTROLLER_H_ */