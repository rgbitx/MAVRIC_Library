/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/** 
 * \file simulation.c
 *  
 * This file is an onboard Hardware-in-the-loop simulation. 
 * Simulates quad-copter dynamics based on simple aerodynamics/physics model and generates simulated raw IMU measurements
 */


#include "conf_constants.h"
#include "time_keeper.h"
#include "coord_conventions.h"
#include "quaternions.h"

#include "central_data.h"
#include "maths.h"

/**
 * \brief	Changes between simulation to and from reality
 *
 * \param	sim				The pointer to the simulation model structure
 * \param	packet			The pointer to the decoded mavlink command long message
 */
static void simulation_set_new_home_position(simulation_model_t *sim, mavlink_command_long_t* packet);

/**
 * \brief	Computes the forces in the local frame of a "cross" quadrotor configuration
 *
 * \warning	This function is not implemented
 *
 * \param	sim		The pointer to the simulation structure
 * \param	servos	The pointer to the servos structure
 */
void forces_from_servos_cross_quad(simulation_model_t *sim, servo_output_t *servos);

/**
 * \brief	Computer the forces in the local frame for a "diagonal" quadrotor configuration
 *
 * \param	sim		The pointer to the simulation structure
 * \param	servos	The pointer to the servos structure
 */
void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output_t *servos);

void simulation_init(simulation_model_t* sim, const simulation_config_t* sim_config, ahrs_t* attitude_estimation, imu_t* imu, position_estimator_t* pos_est, pressure_data_t* pressure, gps_Data_type_t* gps, state_structure_t* state_structure, servo_output_t* servos, bool* waypoint_set, mavlink_message_handler_t *message_handler)
{
	int32_t i;
	
	sim->vehicle_config = *sim_config;
	
	sim->imu = imu;
	sim->pos_est = pos_est;
	sim->pressure = pressure;
	sim->gps = gps;
	sim->state_structure = state_structure;
	sim->servos = servos;
	sim->waypoint_set = waypoint_set;
	
	// set initial conditions to a given attitude_filter
	sim->estimated_attitude = attitude_estimation;
	sim->attitude_estimation = *attitude_estimation;

	print_util_dbg_print("Attitude:");
	print_util_dbg_print_num(sim->attitude_estimation.qe.s*100,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->attitude_estimation.qe.v[0]*100,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->attitude_estimation.qe.v[1]*100,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->attitude_estimation.qe.v[2]*100,10);
	print_util_dbg_print(")\r");
	

	for (i = 0; i < ROTORCOUNT; i++)
	{
		sim->rotorspeeds[i] = 0.0f;			
	}
	sim->last_update = time_keeper_get_micros();
	sim->dt = 0.01f;
	
	simulation_reset_simulation(sim);
	simulation_calib_set(sim);
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id    = MAV_CMD_DO_SET_HOME; // 179
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&simulation_set_new_home_position;
	callbackcmd.module_struct =										sim;
	mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
	
	print_util_dbg_print("HIL simulation initialized. \n");
}

void simulation_calib_set(simulation_model_t *sim)
{
	int32_t i;
	
	for (i = 0;i < 3;i++)
	{
		//we take in sim the inverse of the imu scale_factor to limit number of division
		//while feeding raw_sensor.data[i]
		sim->calib_gyro.scale_factor[i]			= 1.0f/sim->imu->calib_gyro.scale_factor[i];
		sim->calib_accelero.scale_factor[i]		= 1.0f/sim->imu->calib_accelero.scale_factor[i];
		sim->calib_compass.scale_factor[i]		= 1.0f/sim->imu->calib_compass.scale_factor[i];
		
		sim->calib_gyro.bias[i]					= sim->imu->calib_gyro.bias[i];
		sim->calib_accelero.bias[i]				= sim->imu->calib_accelero.bias[i];
		sim->calib_compass.bias[i]				= sim->imu->calib_compass.bias[i];
		
		sim->calib_gyro.orientation[i]			= sim->imu->calib_gyro.orientation[i];
		sim->calib_accelero.orientation[i]		= sim->imu->calib_accelero.orientation[i];
		sim->calib_compass.orientation[i]		= sim->imu->calib_compass.orientation[i];
	}
}

void simulation_reset_simulation(simulation_model_t *sim)
{
	int32_t i;
	
	for (i = 0; i < 3; i++)
	{
		sim->rates_bf[i] = 0.0f;
		sim->torques_bf[i] = 0.0f;
		sim->lin_forces_bf[i] = 0.0f;
		sim->vel_bf[i] = 0.0f;
		sim->vel[i] = 0.0f;
	}
	
	sim->localPosition = sim->pos_est->localPosition;
	
	sim->attitude_estimation = *sim->estimated_attitude;
	
	print_util_dbg_print("(Re)setting simulation. Origin: (");
	print_util_dbg_print_num(sim->pos_est->localPosition.origin.latitude*10000000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->localPosition.origin.longitude*10000000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->localPosition.origin.altitude*1000,10);
	print_util_dbg_print("), Position: (x1000) (");
	print_util_dbg_print_num(sim->pos_est->localPosition.pos[0]*1000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->localPosition.pos[1]*1000,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(sim->pos_est->localPosition.pos[2]*1000,10);
	print_util_dbg_print(")\n");
	
	//sim->localPosition.origin.latitude = HOME_LATITUDE;
	//sim->localPosition.origin.longitude = HOME_LONGITUDE;
	//sim->localPosition.origin.altitude = HOME_ALTITUDE;
	
	//sim->localPosition.origin = sim->pos_est->localPosition.origin;
	//sim->localPosition.heading = sim->pos_est->localPosition.heading;
}

/** 
 * \brief Inverse function of mix_to_servos in stabilization to recover torques and forces
 *
 * \param	sim					The pointer to the simulation model structure
 * \param	rpm					The rotation per minute of the rotor
 * \param	sqr_lat_airspeed	The square of the lateral airspeed
 * \param	axial_airspeed		The axial airspeed
 *
 * \return The value of the lift / drag value without the lift / drag coefficient
 */
static inline float lift_drag_base(simulation_model_t *sim, float rpm, float sqr_lat_airspeed, float axial_airspeed) {
	if (rpm < 0.1f)
	{
		return 0.0f;
	}
	float mean_vel = sim->vehicle_config.rotor_diameter *PI * rpm / 60.0f;
	float exit_vel = rpm / 60.0f *sim->vehicle_config.rotor_pitch;
	           
	return (0.5f * AIR_DENSITY * (mean_vel * mean_vel +sqr_lat_airspeed) * sim->vehicle_config.rotor_foil_area  * (1.0f - (axial_airspeed / exit_vel)));
}


void forces_from_servos_diag_quad(simulation_model_t *sim, servo_output_t *servos){
	int32_t i;
	float motor_command[4];
	float rotor_lifts[4], rotor_drags[4], rotor_inertia[4];
	float ldb;
	UQuat_t wind_gf = {.s = 0, .v = {sim->vehicle_config.wind_x, sim->vehicle_config.wind_y, 0.0f}};
	UQuat_t wind_bf = quaternions_global_to_local(sim->attitude_estimation.qe, wind_gf);
	
	float sqr_lateral_airspeed = SQR(sim->vel_bf[0] + wind_bf.v[0]) + SQR(sim->vel_bf[1] + wind_bf.v[1]);
	float lateral_airspeed = sqrt(sqr_lateral_airspeed);
	
	float old_rotor_speed;
	for (i = 0; i < 4; i++)
	{
		motor_command[i] = (float)servos[i].value / SERVO_SCALE - sim->vehicle_config.rotor_rpm_offset;
		if (motor_command[i] < 0.0f) 
		{
			motor_command[i] = 0;
		}
		
		// temporarily save old rotor speeds
		old_rotor_speed = sim->rotorspeeds[i];
		// estimate rotor speeds by low - pass filtering
		//sim->rotorspeeds[i] = (sim->vehicle_config.rotor_lpf) * sim->rotorspeeds[i] + (1.0f - sim->vehicle_config.rotor_lpf) * (motor_command[i] * sim->vehicle_config.rotor_rpm_gain);
		sim->rotorspeeds[i] = (motor_command[i] * sim->vehicle_config.rotor_rpm_gain);
		
		// calculate torque created by rotor inertia
		rotor_inertia[i] = (sim->rotorspeeds[i] - old_rotor_speed) / sim->dt * sim->vehicle_config.rotor_momentum;
		
		ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, -sim->vel_bf[Z]);
		//ldb = lift_drag_base(sim, sim->rotorspeeds[i], sqr_lateral_airspeed, 0.0f);
		
		rotor_lifts[i] = ldb * sim->vehicle_config.rotor_cl;
		rotor_drags[i] = ldb * sim->vehicle_config.rotor_cd;
	}
	
	float mpos_x = sim->vehicle_config.rotor_arm_length / 1.4142f;
	float mpos_y = sim->vehicle_config.rotor_arm_length / 1.4142f;
	
	// torque around x axis (roll)
	sim->torques_bf[ROLL]  = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_REAR_LEFT]  ) 
						    - (rotor_lifts[M_FRONT_RIGHT] + rotor_lifts[M_REAR_RIGHT] )) * mpos_y;;

	// torque around y axis (pitch)
	sim->torques_bf[PITCH] = ((rotor_lifts[M_FRONT_LEFT]  + rotor_lifts[M_FRONT_RIGHT] )
							- (rotor_lifts[M_REAR_LEFT]   + rotor_lifts[M_REAR_RIGHT] ))*  mpos_x;

	sim->torques_bf[YAW]   = (M_FL_DIR * (10.0f * rotor_drags[M_FRONT_LEFT] + rotor_inertia[M_FRONT_LEFT])  + M_FR_DIR * (10.0f * rotor_drags[M_FRONT_RIGHT] + rotor_inertia[M_FRONT_RIGHT])
							+ M_RL_DIR * (10.0f * rotor_drags[M_REAR_LEFT] + rotor_inertia[M_REAR_LEFT])   + M_RR_DIR * (10.0f * rotor_drags[M_REAR_RIGHT] + rotor_inertia[M_REAR_RIGHT] ))*  sim->vehicle_config.rotor_diameter;
	

	
	sim->lin_forces_bf[X] = -(sim->vel_bf[X] - wind_bf.v[0]) * lateral_airspeed * sim->vehicle_config.vehicle_drag;  
	sim->lin_forces_bf[Y] = -(sim->vel_bf[Y] - wind_bf.v[1]) * lateral_airspeed * sim->vehicle_config.vehicle_drag;
	sim->lin_forces_bf[Z] = -(rotor_lifts[M_FRONT_LEFT]+ rotor_lifts[M_FRONT_RIGHT] +rotor_lifts[M_REAR_LEFT] +rotor_lifts[M_REAR_RIGHT]);

}


void forces_from_servos_cross_quad(simulation_model_t *sim, servo_output_t *servos){
	//int32_t i;
	//float motor_command[4];
	
	//TODO: implement the correct forces
/*	motor_command[M_FRONT] = control->thrust + control->rpy[PITCH] + M_FRONT_DIR * control->rpy[YAW];
	motor_command[M_RIGHT] = control->thrust - control->rpy[ROLL] + M_RIGHT_DIR * control->rpy[YAW];
	motor_command[M_REAR]  = control->thrust - control->rpy[PITCH] + M_REAR_DIR * control->rpy[YAW];
	motor_command[M_LEFT]  = control->thrust + control->rpy[ROLL] + M_LEFT_DIR * control->rpy[YAW];
	for (i = 0; i < 4; i++) {
		if (motor_command[i] < MIN_THRUST) motor_command[i] = MIN_THRUST;
		if (motor_command[i] > MAX_THRUST) motor_command[i] = MAX_THRUST;
	}

	for (i = 0; i < 4; i++) {
		centralData->servos[i].value = SERVO_SCALE * motor_command[i];
	}
	*/
}

void simulation_update(simulation_model_t *sim)
{
	int32_t i;
	UQuat_t qtmp1, qvel_bf,  qed;
	const UQuat_t front = {.s = 0.0f, .v = {1.0f, 0.0f, 0.0f}};
	const UQuat_t up = {.s = 0.0f, .v = {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z}};
	
	
	uint32_t now = time_keeper_get_micros();
	sim->dt = (now - sim->last_update) / 1000000.0f;
	if (sim->dt > 0.1f)
	{
		sim->dt = 0.1f;
	}
	
	sim->last_update = now;
	// compute torques and forces based on servo commands
	#ifdef CONF_DIAG
	forces_from_servos_diag_quad(sim, sim->servos);
	#endif
	#ifdef CONF_CROSS
	forces_from_servos_cross_quad(sim, sim->servos);
	#endif
	
	// pid_control_integrate torques to get simulated gyro rates (with some damping)
	sim->rates_bf[0] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[0] + sim->dt * sim->torques_bf[0] / sim->vehicle_config.roll_pitch_momentum, 10.0f);
	sim->rates_bf[1] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[1] + sim->dt * sim->torques_bf[1] / sim->vehicle_config.roll_pitch_momentum, 10.0f);
	sim->rates_bf[2] = maths_clip((1.0f - 0.1f * sim->dt) * sim->rates_bf[2] + sim->dt * sim->torques_bf[2] / sim->vehicle_config.yaw_momentum, 10.0f);
	
	
	for (i = 0; i < 3; i++)
	{
			qtmp1.v[i] = sim->rates_bf[i];
	}
	
	qtmp1.s = 0;

	// apply step rotation 
	qed = quaternions_multiply(sim->attitude_estimation.qe,qtmp1);

	// TODO: correct this formulas when using the right scales
	sim->attitude_estimation.qe.s = sim->attitude_estimation.qe.s + qed.s * sim->dt;
	sim->attitude_estimation.qe.v[0] += qed.v[0] * sim->dt;
	sim->attitude_estimation.qe.v[1] += qed.v[1] * sim->dt;
	sim->attitude_estimation.qe.v[2] += qed.v[2] * sim->dt;

	sim->attitude_estimation.qe = quaternions_normalise(sim->attitude_estimation.qe);
	sim->attitude_estimation.up_vec = quaternions_global_to_local(sim->attitude_estimation.qe, up);
	
	sim->attitude_estimation.north_vec = quaternions_global_to_local(sim->attitude_estimation.qe, front);	

	// velocity and position integration
	
	// check altitude - if it is lower than 0, clamp everything (this is in NED, assuming negative altitude)
	if (sim->localPosition.pos[Z] >0)
	{
		sim->vel[Z] = 0.0f;
		sim->localPosition.pos[Z] = 0.0f;

		// simulate "acceleration" caused by contact force with ground, compensating gravity
		for (i = 0; i < 3; i++)
		{
			sim->lin_forces_bf[i] = sim->attitude_estimation.up_vec.v[i] * sim->vehicle_config.total_mass * sim->vehicle_config.sim_gravity;
		}
				
		// slow down... (will make velocity slightly inconsistent until next update cycle, but shouldn't matter much)
		for (i = 0; i < 3; i++)
		{
			sim->vel_bf[i] = 0.95f * sim->vel_bf[i];
		}
		
		//upright
		sim->rates_bf[0] =  sim->attitude_estimation.up_vec.v[1]; 
		sim->rates_bf[1] =  - sim->attitude_estimation.up_vec.v[0];
		sim->rates_bf[2] = 0;
	}
	
	sim->attitude_estimation.qe = quaternions_normalise(sim->attitude_estimation.qe);
	sim->attitude_estimation.up_vec = quaternions_global_to_local(sim->attitude_estimation.qe, up);
	
	sim->attitude_estimation.north_vec = quaternions_global_to_local(sim->attitude_estimation.qe, front);	
	for (i = 0; i < 3; i++)
	{
			qtmp1.v[i] = sim->vel[i];
	}
	qtmp1.s = 0.0f;
	qvel_bf = quaternions_global_to_local(sim->attitude_estimation.qe, qtmp1);
	
	float acc_bf[3];
	for (i = 0; i < 3; i++)
	{
		sim->vel_bf[i] = qvel_bf.v[i];
		
		// following the convention in the IMU, this is the acceleration due to force, as measured
		sim->attitude_estimation.linear_acc[i] = sim->lin_forces_bf[i] / sim->vehicle_config.total_mass;
		
		// this is the "clean" acceleration without gravity
		acc_bf[i] = sim->attitude_estimation.linear_acc[i] - sim->attitude_estimation.up_vec.v[i] * sim->vehicle_config.sim_gravity;
		
		sim->vel_bf[i] = sim->vel_bf[i] + acc_bf[i] * sim->dt;
	}
	
	// calculate velocity in global frame
	// vel = qe *vel_bf * qe - 1
	qvel_bf.s = 0.0f; qvel_bf.v[0] = sim->vel_bf[0]; qvel_bf.v[1] = sim->vel_bf[1]; qvel_bf.v[2] = sim->vel_bf[2];
	qtmp1 = quaternions_local_to_global(sim->attitude_estimation.qe, qvel_bf);
	sim->vel[0] = qtmp1.v[0]; sim->vel[1] = qtmp1.v[1]; sim->vel[2] = qtmp1.v[2];
	
	for (i = 0; i < 3; i++)
	{
		sim->localPosition.pos[i] = sim->localPosition.pos[i] + sim->vel[i] * sim->dt;
	}

	// fill in simulated IMU values
	
	for (i = 0;i < 3; i++)
	{
		sim->imu->raw_gyro.data[i] = (sim->rates_bf[i] * sim->calib_gyro.scale_factor[i] + sim->calib_gyro.bias[i]) * sim->calib_gyro.orientation[i];
		sim->imu->raw_accelero.data[i] = ((sim->lin_forces_bf[i] / sim->vehicle_config.total_mass / sim->vehicle_config.sim_gravity) * sim->calib_accelero.scale_factor[i] + sim->calib_accelero.bias[i]) * sim->calib_accelero.orientation[i];
		sim->imu->raw_compass.data[i] = ((sim->attitude_estimation.north_vec.v[i] ) * sim->calib_compass.scale_factor[i] + sim->calib_compass.bias[i])* sim->calib_compass.orientation[i];
	}
	
	//sim->imu->raw_gyro.data[IMU_X] = sim->rates_bf[0] * sim->simu_raw_scale[GYRO_OFFSET + IMU_X] + sim->simu_raw_biais[GYRO_OFFSET + IMU_X];
	//sim->imu->raw_gyro.data[IMU_Y] = sim->rates_bf[1] * sim->simu_raw_scale[GYRO_OFFSET + IMU_Y] + sim->simu_raw_biais[GYRO_OFFSET + IMU_Y];
	//sim->imu->raw_gyro.data[IMU_Z] = sim->rates_bf[2] * sim->simu_raw_scale[GYRO_OFFSET + IMU_Z] + sim->simu_raw_biais[GYRO_OFFSET + IMU_Z];

	//sim->imu->raw_accelero.data[IMU_X] = (sim->lin_forces_bf[0] / sim->vehicle_config.total_mass / GRAVITY) * sim->simu_raw_scale[ACC_OFFSET + IMU_X] + sim->simu_raw_biais[ACC_OFFSET + IMU_X];
	//sim->imu->raw_accelero.data[IMU_Y] = (sim->lin_forces_bf[1] / sim->vehicle_config.total_mass / GRAVITY) * sim->simu_raw_scale[ACC_OFFSET + IMU_Y] + sim->simu_raw_biais[ACC_OFFSET + IMU_Y];
	//sim->imu->raw_accelero.data[IMU_Z] = (sim->lin_forces_bf[2] / sim->vehicle_config.total_mass / GRAVITY) * sim->simu_raw_scale[ACC_OFFSET + IMU_Z] + sim->simu_raw_biais[ACC_OFFSET + IMU_Z];
	//// cheating... provide true upvector instead of simulated forces
	////sim->imu->raw_compass.data[IMU_X] = sim->attitude_estimation.up_vec.v[0] * imu->calib_accelero.scale_factor[IMU_X] + imu->calib_accelero.bias[IMU_X];
	////sim->imu->raw_compass.data[IMU_Y] = sim->attitude_estimation.up_vec.v[1] * imu->calib_accelero.scale_factor[IMU_Y] + imu->calib_accelero.bias[IMU_Y];
	////sim->imu->raw_compass.data[IMU_Z] = sim->attitude_estimation.up_vec.v[2] * imu->calib_accelero.scale_factor[IMU_Z] + imu->calib_accelero.bias[IMU_Z];
	
	//sim->imu->raw_compass.data[IMU_X] = (sim->attitude_estimation.north_vec.v[0] ) * sim->simu_raw_scale[MAG_OFFSET + IMU_X] + sim->simu_raw_biais[MAG_OFFSET + IMU_X];
	//sim->imu->raw_compass.data[IMU_Y] = (sim->attitude_estimation.north_vec.v[1] ) * sim->simu_raw_scale[MAG_OFFSET + IMU_Y] + sim->simu_raw_biais[MAG_OFFSET + IMU_Y];
	//sim->imu->raw_compass.data[IMU_Z] = (sim->attitude_estimation.north_vec.v[2] ) * sim->simu_raw_scale[MAG_OFFSET + IMU_Z] + sim->simu_raw_biais[MAG_OFFSET + IMU_Z];
	
	//sim->imu->dt = sim->dt;

	sim->localPosition.heading = coord_conventions_get_yaw(sim->attitude_estimation.qe);
	//sim->pos_est->localPosition = sim->localPosition;
}

void simulation_simulate_barometer(simulation_model_t *sim)
{
	sim->pressure->altitude = sim->localPosition.origin.altitude - sim->localPosition.pos[Z];
	sim->pressure->vario_vz = sim->vel[Z];
	sim->pressure->last_update = time_keeper_get_millis();
	sim->pressure->altitude_offset = 0;
}
	
void simulation_simulate_gps(simulation_model_t *sim)
{
	global_position_t gpos = coord_conventions_local_to_global_position(sim->localPosition);
	
	sim->gps->altitude = gpos.altitude;
	sim->gps->latitude = gpos.latitude;
	sim->gps->longitude = gpos.longitude;
	sim->gps->time_last_msg = time_keeper_get_millis();
	sim->gps->status = GPS_OK;
}

void simulation_fake_gps_fix(simulation_model_t* sim, uint32_t timestamp_ms)
{
	local_coordinates_t fake_pos;
	
	fake_pos.pos[X] = 10.0f;
	fake_pos.pos[Y] = 10.0f;
	fake_pos.pos[Z] = 0.0f;
	fake_pos.origin.latitude = sim->vehicle_config.home_coordinates[0];
	fake_pos.origin.longitude = sim->vehicle_config.home_coordinates[1];
	fake_pos.origin.altitude = sim->vehicle_config.home_coordinates[2];
	fake_pos.timestamp_ms = timestamp_ms;

	global_position_t gpos = coord_conventions_local_to_global_position(fake_pos);
	
	sim->gps->latitude = gpos.latitude;
	sim->gps->longitude = gpos.longitude;
	sim->gps->altitude = gpos.altitude;
	sim->gps->time_last_msg = time_keeper_get_millis();
	sim->gps->status = GPS_OK;
}

void simulation_switch_between_reality_n_simulation(simulation_model_t *sim)
{
	uint32_t i;
	
	// From simulation to reality
	if (sim->state_structure->simulation_mode == REAL_MODE)
	{
		sim->pos_est->localPosition.origin = sim->localPosition.origin;
		for (i = 0;i < 3;i++)
		{
			sim->pos_est->localPosition.pos[i] = 0.0f;
		}
		sim->pos_est->init_gps_position = false;
		sim->state_structure->mav_state = MAV_STATE_STANDBY;
		sim->state_structure->mav_mode = MAV_MODE_MANUAL_DISARMED;
		servo_pwm_failsafe(sim->servos);
	}

	// From reality to simulation
	if (sim->state_structure->simulation_mode == SIMULATION_MODE)
	{	
		simulation_reset_simulation(sim);
		simulation_calib_set(sim);
		sim->pos_est->init_gps_position = false;
	}
}

static void simulation_set_new_home_position(simulation_model_t *sim, mavlink_command_long_t* packet)
{
	if (packet->param1 == 1)
	{
		// Set new home position to actual position
		print_util_dbg_print("Set new home location to actual position.\n");
		sim->localPosition.origin = coord_conventions_local_to_global_position(sim->localPosition);

		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(sim->localPosition.origin.latitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->localPosition.origin.longitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->localPosition.origin.altitude * 1000.0f,10);
		print_util_dbg_print(")\n");
	}
	else
	{
		// Set new home position from msg
		print_util_dbg_print("Set new home location. \n");

		sim->localPosition.origin.latitude = packet->param5;
		sim->localPosition.origin.longitude = packet->param6;
		sim->localPosition.origin.altitude = packet->param7;

		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(sim->localPosition.origin.latitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->localPosition.origin.longitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(sim->localPosition.origin.altitude * 1000.0f,10);
		print_util_dbg_print(")\n");
	}

	*sim->waypoint_set = false;
}

task_return_t simulation_send_data(simulation_model_t* sim_model)
{
	Aero_Attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(sim_model->attitude_estimation.qe);

	global_position_t gpos = coord_conventions_local_to_global_position(sim_model->localPosition);
	
	mavlink_msg_hil_state_send(	MAVLINK_COMM_0,
								time_keeper_get_micros(),
								aero_attitude.rpy[0],
								aero_attitude.rpy[1],
								aero_attitude.rpy[2],
								sim_model->rates_bf[ROLL],
								sim_model->rates_bf[PITCH],
								sim_model->rates_bf[YAW],
								gpos.latitude * 10000000,
								gpos.longitude * 10000000,
								gpos.altitude * 1000.0f,
								100 * sim_model->vel[X],
								100 * sim_model->vel[Y],
								100 * sim_model->vel[Z],
								1000 * sim_model->lin_forces_bf[0],
								1000 * sim_model->lin_forces_bf[1],
								1000 * sim_model->lin_forces_bf[2] 	);
	
	mavlink_msg_hil_state_quaternion_send(	MAVLINK_COMM_0,
											time_keeper_get_micros(),
											(float*) &sim_model->attitude_estimation.qe,
											aero_attitude.rpy[ROLL],
											aero_attitude.rpy[PITCH],
											aero_attitude.rpy[YAW],
											gpos.latitude * 10000000,
											gpos.longitude * 10000000,
											gpos.altitude * 1000.0f,
											100 * sim_model->vel[X],
											100 * sim_model->vel[Y],
											100 * sim_model->vel[Z],
											100 * vectors_norm(sim_model->vel),
											0.0f,
											sim_model->attitude_estimation.linear_acc[X],
											sim_model->attitude_estimation.linear_acc[Y],
											sim_model->attitude_estimation.linear_acc[Z]	);
	
	//mavlink_msg_named_value_int_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"rolltorque",
										//sim_model->torques_bf[0]	);
//
	//mavlink_msg_named_value_int_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"pitchtorque",
										//sim_model->torques_bf[1]	);
//
	//mavlink_msg_named_value_int_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"yawtorque",
										//sim_model->torques_bf[2]);

	//mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"rpm1",
										//sim_model->rotorspeeds[0]);
//
	//mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"rpm2",
										//sim_model->rotorspeeds[1]);
//
	//mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"rpm3",
										//sim_model->rotorspeeds[2]);
//
	//mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										//time_keeper_get_millis(),
										//"rpm4",
										//sim_model->rotorspeeds[3]);
	return TASK_RUN_SUCCESS;
}