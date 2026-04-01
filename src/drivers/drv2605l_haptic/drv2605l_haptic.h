/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

 /**
 * @file drv2605l_haptic.h
 * Haptic feedback driver for DRV2605L on ARK FPV - Regular Task Version
 *
 * @author Evelena
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include <parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_sensor.h>
#include <matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
//#include <uORB/topics/sensor_avs.h>
#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/sensor_avs_lite.h>
#include <uORB/topics/sensor_avs_lite_ext.h>

// Device type for DRV2605L
#define DRV_HAPTIC_DEVTYPE_DRV2605L 0x60

// PCA9548A I2C Multiplexer
#define PCA9548A_ADDR           0x70

// DRV2605L Register addresses
#define DRV2605L_ADDR           0x5A
#define DRV2605L_REG_STATUS     0x00
#define DRV2605L_REG_MODE       0x01
#define DRV2605L_REG_RTPIN      0x02
#define DRV2605L_REG_LIBRARY    0x03
#define DRV2605L_REG_WAVESEQ1   0x04
#define DRV2605L_REG_GO         0x0C
#define DRV2605L_REG_FEEDBACK   0x1A

// Mode register values
#define DRV2605L_MODE_INTTRIG   0x00  // Internal trigger mode
#define DRV2605L_MODE_STANDBY   0x40  // Standby mode

// Library values
#define DRV2605L_LIB_ERM        0x01  // ERM library
#define DRV2605L_LIB_LRA        0x06  // LRA library


//ModuleBase<DRV2605L> → gives it PX4's module framework (start/stop/status commands)
//device::I2C → gives it I2C communication capabilities (transfer(), bus setup, etc).
class DRV2605L : public ModuleParams, public ModuleBase<DRV2605L>, public device::I2C
{
public:
	//lifecycle
	DRV2605L(int bus, int drv_address, int multiplex_address, int multiplex_channel1, int multiplex_channel2);
	virtual ~DRV2605L();

	/** @see ModuleBase */
	// PX4 integration
	static int task_spawn(int argc, char *argv[]); // called by the module framework when you run 'drv2605l_haptic start'
	static DRV2605L *instantiate(int argc, char *argv[]); //
	static int custom_command(int argc, char *argv[]); //
	static int print_usage(const char *reason = nullptr);
	void run() override; // core logic
	int print_status() override;

private:
	// member functions (methods) - performs actions/ things the object does
	//lifecycle
	int init();
	int probe() override;

	void parameters_update();

	int select_multiplex_channel(uint8_t channel);

	// I2C comm
	int write_register(uint8_t reg, uint8_t value);
	int read_register(uint8_t reg, uint8_t &value);

	int trigger_effect(uint8_t channel, uint8_t effect); //core logic
	int init_drv2605l(uint8_t channel);
	bool check_act_int_threshold(float active_int);
	bool check_q_factor_threshold(float q_factor);
	bool check_histogram_threshold(float histogram);
	char is_azimuth_in_range(float azimuth);
	char is_elevation_in_range(float elevation);
	char is_yaw_in_range(float yaw_deg);
	char is_roll_in_range(float roll);
	char is_pitch_in_range(float pitch);

	// subsribe to both instances of sensor_avs_lite_ext (account fot 2 AVS sensors)
	uORB::Subscription _sensor_avs_lite_ext_sub_0{ORB_ID(sensor_avs_lite_ext),0};
	uORB::Subscription _sensor_avs_lite_ext_sub_1{ORB_ID(sensor_avs_lite_ext),1};

	//param_update subscription to update parameters
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	bool _initialized{false};
	bool _use_multiplex{false};

	uint8_t _multiplex_addr{PCA9548A_ADDR};
	uint8_t _multiplex_channel1{0};  // Channel for first DRV2605L
	uint8_t _multiplex_channel2{1};  // Channel for second DRV2605L
	uint32_t _loop_interval_us{100000}; // 100ms
	//

	DEFINE_PARAMETERS(

		(ParamBool<px4::params::HAP_MULTIPLEX>) _multiplexer_flag,
		(ParamInt<px4::params::HAP_MODE>) _mode,
		(ParamInt<px4::params::HAP_IMU_UP_DOWN>) _up_down_motion,

		(ParamFloat<px4::params::HAP_OFFSET_AVS_R>) _offset_avs_r,
		(ParamFloat<px4::params::HAP_OFFSET_AVS_L>) _offset_avs_l,
		(ParamFloat<px4::params::HAP_OFFSET_IMU>) _offset_imu,

		(ParamInt<px4::params::HAP_SENSE_AVS_R>) _sense_avs_r,
		(ParamInt<px4::params::HAP_SENSE_AVS_L>) _sense_avs_l,
		(ParamInt<px4::params::HAP_SENSE_IMU>) _sense_imu,

		(ParamFloat<px4::params::HAP_YAW_MIN>) _yaw_min,
		(ParamFloat<px4::params::HAP_YAW_MAX>) _yaw_max,
		(ParamFloat<px4::params::HAP_ACT_INT>) _act_int,
		(ParamFloat<px4::params::HAP_Q_FACTOR>) _q_factor,
		(ParamFloat<px4::params::HAP_HISTOGRAM>) _histogram,
		(ParamFloat<px4::params::HAP_ELEV_MAX>) _elevation_max,
		(ParamFloat<px4::params::HAP_ELEV_MIN>) _elevation_min,
		(ParamFloat<px4::params::HAP_PITCH_MAX>) _pitch_max,
		(ParamFloat<px4::params::HAP_PITCH_MIN>) _pitch_min,
		(ParamFloat<px4::params::HAP_ROLL_MAX>) _roll_max,
		(ParamFloat<px4::params::HAP_ROLL_MIN>) _roll_min,
		(ParamFloat<px4::params::HAP_AZIMUTH_MIN>) _azimuth_min,
		(ParamFloat<px4::params::HAP_AZIMUTH_MAX>) _azimuth_max,
		(ParamInt<px4::params::HAP_DRV_EFFECT_B>) _drv_effect_b,
		(ParamInt<px4::params::HAP_DRV_EFFECT_T>) _drv_effect_t,
		(ParamFloat<px4::params::HAP_TRIG_TIMER>) _trig_timer
	)


	param_t _param_left_node_id {PARAM_INVALID};
	param_t _param_right_node_id {PARAM_INVALID};
};
