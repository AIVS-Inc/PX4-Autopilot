/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file drv2605l_haptic.cpp
 * Haptic feedback driver for DRV2605L on ARK FPV - Regular Task Version
 *
 * @author Evelena
 */

 // preprocessor directives - instructions that are processed before the actual compilation
 // of your code begins. They start with # and don't end with a semicolon.

 // #include directive tells the preprocessor to insert the contents of another file into your code.
 // < > For system/standard library headers
 // " " For user-defined headers
 // #define directive creates macros - text replacements that happen before compilation.
// 	#define IDENTIFIER replacement_text
// 	#define MACRO(parameters) replacement_text

//#include "drv2605l_haptic.hpp"

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
#include <uORB/topics/sensor_avs.h>
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

// Waveform effects
// #define DRV2605L_EFFECT_STRONG_CLICK    1
// #define DRV2605L_EFFECT_MEDIUM_CLICK    10
// #define DRV2605L_EFFECT_BUZZ            47


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
	static int task_spawn(int argc, char *argv[]);
	static DRV2605L *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
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
	//float get_active_intensity(const sensor_avs_s &sensor_avs_data);
	float get_elevation(const sensor_avs_s &sensor_avs_data);
	float get_q_factor(const sensor_avs_s &sensor_avs_data);
	float get_azimuth(const sensor_avs_s &sensor_avs_data);
	bool check_act_int_threshold(float active_int);
	float get_yaw_from_quaternion(const vehicle_attitude_s &att);
	//float get_pitch_from_quaternion(const vehicle_attitude_s &att);
	float get_roll_from_quaternion(const vehicle_attitude_s &att);
	char is_azimuth_in_range(float azimuth);
	char is_elevation_in_range(float elevation);
	char is_yaw_in_range(float yaw_deg);
	char is_roll_in_range(float roll);
	//char is_pitch_in_range(float pitch);

	// uORB subscription for vehicle attitude and sensor avs data
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _sensor_avs_sub{ORB_ID(sensor_avs)};
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
		(ParamFloat<px4::params::HAP_OFFSET>) _offset,
		(ParamInt<px4::params::HAP_SENSE>) _sense,
		(ParamFloat<px4::params::HAP_YAW_MIN>) _yaw_min,
		(ParamFloat<px4::params::HAP_YAW_MAX>) _yaw_max,
		(ParamFloat<px4::params::HAP_ACT_INT>) _act_int,
		(ParamFloat<px4::params::HAP_ELEV_MAX>) _elevation_max,
		(ParamFloat<px4::params::HAP_ELEV_MIN>) _elevation_min,
		(ParamFloat<px4::params::HAP_PITCH_MAX>) _pitch_max,
		(ParamFloat<px4::params::HAP_PITCH_MIN>) _pitch_min,
		(ParamFloat<px4::params::HAP_ROLL_MAX>) _roll_max,
		(ParamFloat<px4::params::HAP_ROLL_MIN>) _roll_min,
		(ParamFloat<px4::params::HAP_AZIMUTH_MIN>) _azimuth_min,
		(ParamFloat<px4::params::HAP_AZIMUTH_MAX>) _azimuth_max,
		(ParamFloat<px4::params::HAP_Q_FACTOR>) _q_factor,
		(ParamInt<px4::params::HAP_DRV_EFFECT_B>) _drv_effect_b,
		(ParamInt<px4::params::HAP_DRV_EFFECT_T>) _drv_effect_t,
		(ParamFloat<px4::params::HAP_TRIG_TIMER>) _trig_timer
	)
};

DRV2605L::DRV2605L(int bus, int drv_address, int multiplex_address, int multiplex_channel1, int multiplex_channel2) :
	ModuleParams(nullptr), I2C(DRV_HAPTIC_DEVTYPE_DRV2605L, MODULE_NAME, bus, drv_address, 400000),
	_multiplex_addr(multiplex_address),
	_multiplex_channel1(multiplex_channel1),
	_multiplex_channel2(multiplex_channel2)
{
	// Enable multiplexer if valid channels are specified
	if (_multiplexer_flag.get() && multiplex_channel1 >= 0 && multiplex_channel2 >= 0) {
		_use_multiplex = true;
		PX4_INFO("Multiplexer mode enabled (HAP_MULTIPLEX=true)");
	}else {
		_use_multiplex = false;
		PX4_INFO("Single driver mode (HAP_MULTIPLEX=false or channels not specified)");
	}

}

DRV2605L::~DRV2605L()
{
	// Ensure we put both devices in standby before exit
	if (_initialized && _use_multiplex) { //executes when both conditions are true
		select_multiplex_channel(_multiplex_channel1);
		write_register(DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY);

		select_multiplex_channel(_multiplex_channel2);
		write_register(DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY);
	}else if (_initialized && !_use_multiplex) {
		// Single driver mode - put it in standby
		write_register(DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY);
	}
}

void DRV2605L::parameters_update()
{
	// Update all parameters from storage
	updateParams();

	// add validation logic ??
}

int DRV2605L::select_multiplex_channel(uint8_t channel)
{	// validation check
	if (!_use_multiplex || channel > 7) {
		return OK;  // No multiplex or invalid channel
	}

	// create channel selection byte (1 bit)
	uint8_t channel_byte = (1 << channel);

	uint8_t drv_temp_addr = get_device_address(); // save drv2605l address
	set_device_address(_multiplex_addr); // changes I2C address to multiplexer address temporarily

	int ret = transfer(&channel_byte, 1, nullptr, 0); // sends/receives data over I2C; send channel selection

	// Restore DRV2605L address
	set_device_address(drv_temp_addr);

	if (ret != OK) {
		PX4_ERR("Failed to select multiplex channel %d", channel);
	}

	px4_usleep(100); // settling delay for multiplexer

	return ret;
}

int DRV2605L::probe()
{
	if (_use_multiplex) {
		// Probe both channels
		PX4_INFO("Probing DRV2605L devices via multiplexer...");

		// Channel 1
		if (select_multiplex_channel(_multiplex_channel1) != OK) {
			return -EIO;
		}

		uint8_t status1;
		int ret1 = read_register(DRV2605L_REG_STATUS, status1);

		if (ret1 == OK) {
			PX4_INFO("DRV2605L #1 found on channel %d", _multiplex_channel1); //, status: 0x%02X , status1);
		} else {
			PX4_ERR("DRV2605L #1 not found on channel %d", _multiplex_channel1);
			return -EIO;
		}

		// Channel 2
		if (select_multiplex_channel(_multiplex_channel2) != OK) {
			return -EIO;
		}

		uint8_t status2;
		int ret2 = read_register(DRV2605L_REG_STATUS, status2);

		if (ret2 == OK) {
			PX4_INFO("DRV2605L #2 found on channel %d", _multiplex_channel2); //status: 0x%02X",, status2);
		} else {
			PX4_ERR("DRV2605L #2 not found on channel %d", _multiplex_channel2);
			return -EIO;
		}

		return OK;
	}
	//return OK
	else {
		// Single driver, no multiplexer
		uint8_t status;
		int ret = read_register(DRV2605L_REG_STATUS, status);

		if (ret == OK) {
			PX4_INFO("DRV2605L found (single mode)");
			return OK;
		}

		PX4_ERR("DRV2605L not found");
		return -EIO;
	}
}

int DRV2605L::write_register(uint8_t reg, uint8_t value)
{
	uint8_t cmd[2] = {reg, value};
	return transfer(cmd, 2, nullptr, 0);
}

int DRV2605L::read_register(uint8_t reg, uint8_t &value)
{
	return transfer(&reg, 1, &value, 1);
}

int DRV2605L::init_drv2605l(uint8_t channel)
{
	if (_use_multiplex) {
		int ret = select_multiplex_channel(channel);
		if (ret != OK) {
			return ret;
		}
	}

	// Take out of standby mode
	int ret = write_register(DRV2605L_REG_MODE, DRV2605L_MODE_INTTRIG);

	if (ret != OK) {
		PX4_ERR("Failed to set mode on channel %d", channel);
		return ret;
	}

	px4_usleep(1000);

	// Set to ERM library (change to DRV2605L_LIB_LRA if using LRA motor)
	ret = write_register(DRV2605L_REG_LIBRARY, DRV2605L_LIB_ERM);

	if (ret != OK) {
		PX4_ERR("Failed to set library on channel %d", channel);
		return ret;
	}

	// Configure feedback control (optional, for ERM)
	ret = write_register(DRV2605L_REG_FEEDBACK, 0x36);

	if (ret != OK) {
		PX4_WARN("Failed to set feedback control on channel %d", channel);
	}

	if (_use_multiplex) {
		PX4_INFO("DRV2605L #%d initialized successfully", channel + 1);
	} else {
		PX4_INFO("DRV2605L initialized successfully (single mode)");
	}

	return OK;

}

int DRV2605L::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	if (_use_multiplex) {
		// Initialize both DRV2605L devices
		ret = init_drv2605l(_multiplex_channel1);
		if (ret != OK) {
			return ret;
		}

		ret = init_drv2605l(_multiplex_channel2);
		if (ret != OK) {
			return ret;
		}

		PX4_INFO("Dual DRV2605L configuration complete");
	} else {
		// Initialize single device
		ret = init_drv2605l(0);
		if (ret != OK) {
			return ret;
		}
		PX4_INFO("Single DRV2605L configuration complete");
	}

	_initialized = true;

	return OK;
}

int DRV2605L::trigger_effect(uint8_t channel, uint8_t effect)
{
	if (_use_multiplex) {
		int ret = select_multiplex_channel(channel);
		if (ret != OK) {
			return ret;
		}
	}

	// Set waveform sequence
	int ret = write_register(DRV2605L_REG_WAVESEQ1, effect);

	if (ret != OK) {
		return ret;
	}

	// End waveform sequence
	ret = write_register(DRV2605L_REG_WAVESEQ1 + 1, 0);

	if (ret != OK) {
		return ret;
	}

	// Trigger the GO command
	ret = write_register(DRV2605L_REG_GO, 1);

	return ret;
}

// int DRV2605L::determine_mode()
// {
// 	if (_mode.get() == 0){
// 		_use_imu_mode = true;
// 		//_use_avs_mode == false;
// 		PX4_INFO("using IMU mode");
// 		return 0;

// 	}else{
// 		_use_avs_mode = true;
// 		//_use_imu_mode == false;
// 		PX4_INFO("using AVS mode");
// 		return 1;
// 	}
// }

bool DRV2605L::check_act_int_threshold(float active_int)
{
	// Check if active intensity greater than 70
	return (active_int >= _act_int.get());
}

float DRV2605L::get_elevation(const sensor_avs_s &sensor_avs_data)
{
	float elevation= sensor_avs_data.elevation_deg;
	return elevation;
}

char DRV2605L::is_elevation_in_range(float elevation)  // up/down
{
	//float elevation= sensor_avs_data.elevation_deg;

	// Check if elevation is within range
	char c = 'N'; // declare once

	// if btwn -45 to +45
	if (elevation >= _elevation_min.get() && elevation <= _elevation_max.get()){
		c = 'T'; //assign
	} else {
		c = 'N'; // no haptic effect
	}
	return c;
}


float DRV2605L::get_azimuth(const sensor_avs_s &sensor_avs_data)
{
	float azimuth= sensor_avs_data.azimuth_deg;

	// // Normalize to 0-360 range
	// if (azimuth < 0.0f) {
	// 	azimuth += 360.0f;
	// }

	// Normalize to 0-360° range using modulus
	// apply offset and sense correction
	//azimuth = fmodf(azimuth + 360.0f, 360.0f);

	azimuth = fmodf((azimuth - _offset.get()) * _sense.get() + 360.0f, 360.0f);

	return azimuth;
}

char DRV2605L::is_azimuth_in_range(float azimuth) //float azimuth
{
	//float azimuth= sensor_avs_data.azimuth_deg;

	// Check if azimuth is in range
	char c = 'N'; // declare once

	// if btwn -45 to +45
	if (azimuth >= _azimuth_min.get() && azimuth <= _azimuth_max.get()){
		c = 'B'; //assign
	} else {
		c = 'N'; // no haptic effect
	}
	return c;
}

float DRV2605L::get_q_factor(const sensor_avs_s &sensor_avs_data)
{
	float q_factor= sensor_avs_data.q_factor;
	return q_factor;
}

float DRV2605L::get_yaw_from_quaternion(const vehicle_attitude_s &att)
{
	// Convert quaternion to Euler angles using PX4 matrix library
	matrix::Quatf q(att.q);
	matrix::Eulerf euler(q);

	// euler.psi() returns yaw in radians, convert to degrees
	// Range: -180 to +180 degrees
	float yaw_deg = math::degrees(euler.psi());

	// Normalize to 0-360 range
	// if (yaw_deg < 0.0f) {
	// 	yaw_deg += 360.0f;
	// }
	yaw_deg = fmodf(yaw_deg + 360.0f, 360.0f);

	return yaw_deg;
}


char DRV2605L::is_yaw_in_range(float yaw_deg)  // bool DRV2605L
{
	// Check if yaw is within range
	char c = 'N'; // declare once

	// if btwn 135-225
	if (yaw_deg >= _yaw_min.get() && yaw_deg <= _yaw_max.get()){
		c = 'B'; //assign
	} else {
		c = 'N'; // no haptic effect
	}
	return c;
}

// float DRV2605L::get_pitch_from_quaternion(const vehicle_attitude_s &att)
// {
// 	// Convert quaternion to Euler angles using PX4 matrix library
// 	matrix::Quatf q(att.q);
// 	matrix::Eulerf euler(q);

// 	// euler.theta() returns pitch in radians, convert to degrees
// 	//rotation around Y-axis
// 	// Range: -180 to +180 degrees
// 	float pitch = math::degrees(euler.theta());

// 	// // Normalize to 0-360 range
// 	// if (pitch < 0.0f) {
// 	// 	pitch += 360.0f;
// 	// }
// 	return pitch;
// }

// char DRV2605L::is_pitch_in_range(float pitch)  // bool DRV2605L
// {
// 	// Check if pitch is within range
// 	char c = 'N'; // declare once

// 	// if btwn 135-225
// 	if (pitch >= _pitch_min.get() && pitch <= _pitch_max.get()){
// 		c = 'T'; //assign
// 	} else {
// 		c = 'N'; // no haptic effect
// 	}
// 	return c;
// }

float DRV2605L::get_roll_from_quaternion(const vehicle_attitude_s &att)
{
	// for our case, "roll" is up/down motion (aka elevation)
	// typically this up/down is pitch

	// Convert quaternion to Euler angles using PX4 matrix library
	matrix::Quatf q(att.q);
	matrix::Eulerf euler(q);

	// euler.phi() returns roll (rotation around x-axis)
	//in radians, convert to degrees
	float roll = math::degrees(euler.phi());

	return roll;
}

char DRV2605L::is_roll_in_range(float roll)  // bool DRV2605L
{
	// for our case, "roll" is up/down motion (aka elevation)

	// Check if pitch is within range
	char c = 'N'; // declare once

	// if btwn 135-225
	if (roll >= _roll_min.get() && roll <= _roll_max.get()){
		c = 'T'; //assign
	} else {
		c = 'N'; // no haptic effect
	}
	return c;
}

void DRV2605L::run()
{
	// Initialize the device(s)
	if (init() != OK) {
		PX4_ERR("Initialization failed");
		return;
	}

	// Load initial parameter values
	parameters_update();

	PX4_INFO("Haptic task started");

	// Main loop
	while (!should_exit()) {

		// Check for parameter updates
		if (_parameter_update_sub.updated()) {
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			parameters_update();
			PX4_INFO("Parameters updated");
		}

		// Check for new vehicle attitude data
		vehicle_attitude_s attitude;
		sensor_avs_s sensor_avs_data;
		bool att_updated = _vehicle_attitude_sub.update(&attitude);
		bool avs_updated = _sensor_avs_sub.update(&sensor_avs_data);

		// Static variables to preserve latest values across loop iterations
		static char back_side = 'N';
		static char top_side = 'N';
		static bool greater_threshold = false;

		float trig_timer = _trig_timer.get();
		//static float q_factor = 0.0f;

		if (att_updated)  {

			float yaw_deg = get_yaw_from_quaternion(attitude); 	// Get yaw angle from quaternion
			//float pitch = get_pitch_from_quaternion(attitude); 	// Get pitch angle from quaternion
			float roll = get_roll_from_quaternion(attitude); 	// Get roll angle from quaternion

			if (_mode.get() == 0){
				back_side = is_yaw_in_range(yaw_deg); 		// Check and determine if yaw is in target range
				top_side = is_roll_in_range(roll); 		// Check and determine if roll is in target range
				//PX4_INFO("using IMU mode");
				PX4_INFO("Yaw: %.2f deg | Roll: %.2f deg | Haptic Back Side: %c | Haptic Top Side: %c", (double)yaw_deg,  (double)roll, back_side, top_side);
			}
			// Print yaw continuously with haptic side status
			// Haptic: %s.  (back_side != 'N') ? "ACTIVE" : "INACTIVE"
			//PX4_INFO("Yaw: %.2f deg | Roll: %.2f deg | Haptic Back Side: %c | Haptic Top Side: %c", (double)yaw_deg,  (double)roll, back_side, top_side);
		}

		if (avs_updated){

			float active_int = sensor_avs_data.active_intensity;  // get active intensity value
			float elevation = get_elevation(sensor_avs_data); // get elevation value
    			float azimuth = get_azimuth(sensor_avs_data);  // get azimuth value
			greater_threshold = check_act_int_threshold(active_int); // check if greater than threshold value

			PX4_INFO("Active Intensity: %.2f",(double)active_int);

			if (_mode.get() == 1){

			//if (_use_avs_mode){
				back_side = is_azimuth_in_range(azimuth);  // Check and determine if azimuth is in range
				top_side = is_elevation_in_range(elevation); // Check and determine if elevation is in range
				//PX4_INFO("using AVS mode");
				PX4_INFO("Azimuth: %.2f | Elevation: %.2f | Haptic Back Side: %c | Haptic Top Side: %c", (double)azimuth, (double)elevation, back_side, top_side);

			}
			//float q_factor = get_q_factor(sensor_avs_data);
			//| Q Factor: %.2f", (double)q_factor(double)active_int,
			//PX4_INFO("Active Intensity: %.2f",(double)active_int)
			//PX4_INFO("Active Intensity: %.2f | Azimuth: %.2f | Elevation: %.2f", (double)azimuth,(double)active_int, (double)elevation);
		}
		//
		//Only trigger if in haptic yaw range, haptic pitch range, & exceed active intensity threshold
		if ((back_side != 'N' || top_side != 'N') && greater_threshold ) {

			// Get the effect number from parameter
			uint8_t effectT = static_cast<uint8_t>(_drv_effect_t.get());
			uint8_t effectB = static_cast<uint8_t>(_drv_effect_b.get());

			// Print the effect value being used
			//PX4_INFO("Triggering haptic effect right: %d | left: %d", effectR, effectL);

			if (_use_multiplex){ // if using multiplexer
				if (back_side == 'B'){
					// Trigger backside haptic
					int ret1 = trigger_effect(_multiplex_channel1, effectB);
					//int ret2 = trigger_effect(_multiplex_channel2, effectT);

					if (ret1 != OK) {
						PX4_ERR("Failed to trigger effect");
					}
					px4_usleep((useconds_t)(trig_timer * 1000000));
				}
				if (top_side == 'T'){
					// Trigger backside haptic
					int ret2 = trigger_effect(_multiplex_channel2, effectT);
					//int ret2 = trigger_effect(_multiplex_channel2, effectT);

					if (ret2 != OK) {
						PX4_ERR("Failed to trigger effect");
					}
					px4_usleep((useconds_t)(trig_timer * 1000000));
				}
			} else {  // if no multiplexer just a single driver
				uint8_t effect = effectB; // Default to backside effect for single mode
				int ret = trigger_effect(0, effect);
				if (ret != OK) {
					PX4_ERR("Failed to trigger effect");
				}
				px4_usleep((useconds_t)(trig_timer * 1000000));
			}
		}
		// Sleep for the loop interval
		px4_usleep(_loop_interval_us);
	}

	PX4_INFO("Haptic task exiting");
}


int DRV2605L::print_status()
{
	PX4_INFO("Running: %s", _initialized ? "YES" : "NO");
	PX4_INFO("HAP_MULTIPLEX parameter: %s", _multiplexer_flag.get() ? "true" : "false");
	PX4_INFO("Using multiplexer: %s", _use_multiplex ? "YES" : "NO");

	if (_use_multiplex) {
		PX4_INFO("Multiplexer address: 0x%02X", _multiplex_addr);
		PX4_INFO("DRV2605L #1 on channel: %d", _multiplex_channel1);
		PX4_INFO("DRV2605L #2 on channel: %d", _multiplex_channel2);
	} else {
		PX4_INFO("Single driver (no multiplexer)");
	}

	PX4_INFO("Loop interval: %" PRIu32 " us", _loop_interval_us);
	PX4_INFO("I2C bus: %d, DRV address: 0x%02X", get_device_bus(), get_device_address());
	//PX4_INFO("Yaw trigger range: %.0f-%.0f degrees", (double)_yaw_min.get(), (double)_yaw_max.get());
	//PX4_INFO("Pitch trigger range: %.0f-%.0f degrees", (double)_pitch_min.get(), (double)_pitch_max.get());

	if (_mode.get()== 1){
		PX4_INFO("using AVS mode");
	}

	if (_mode.get() == 0){
	PX4_INFO("using IMU mode");
	}

	return OK;
}

DRV2605L *DRV2605L::instantiate(int argc, char *argv[])
{
	int bus = 1;  // Default to external I2C bus 1 (GPS port)
	int drv_address = DRV2605L_ADDR;
	int multiplex_address = PCA9548A_ADDR;
	int multiplex_channel1 = -1;  // -1 means no multiplex
	int multiplex_channel2 = -1;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:m:c:C:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			bus = atoi(myoptarg);
			break;

		case 'd':
			drv_address = strtol(myoptarg, nullptr, 16);
			break;

		case 'm':
			multiplex_address = strtol(myoptarg, nullptr, 16);
			break;

		case 'c':
			multiplex_channel1 = atoi(myoptarg);
			break;

		case 'C':
			multiplex_channel2 = atoi(myoptarg);
			break;

		default:
			return nullptr;
		}
	}

	return new DRV2605L(bus, drv_address, multiplex_address, multiplex_channel1, multiplex_channel2);
}

int DRV2605L::task_spawn(int argc, char *argv[])
{
	DRV2605L *instance = instantiate(argc, argv);

	if (instance == nullptr) {
		PX4_ERR("Failed to allocate DRV2605L");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = px4_task_spawn_cmd("drv2605l_haptic",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_object.store(nullptr);
		delete instance;
		PX4_ERR("Task start failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int DRV2605L::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DRV2605L::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dual DRV2605L Haptic Feedback Driver for ARK FPV via PCA9548A Multiplexer

This driver supports two DRV2605L haptic motors connected through a PCA9548A
I2C multiplexer, or a single DRV2605L without multiplexer. Continuously prints
yaw angle and triggers haptic motor(s) when yaw is in range and active intensity
threshold is met.

The HAP_MULTIPLEX parameter controls the operating mode:
  HAP_MULTIPLEX = true: Use dual motors via multiplexer (default)
  HAP_MULTIPLEX = false: Use single motor without multiplexer

### Examples
Dual mode with multiplexer (default, channels 0 and 1):
$ param set HAP_MULTIPLEX 1
$ drv2605l_haptic start -b 1 -c 0 -C 1

Single mode without multiplexer:
$ param set HAP_MULTIPLEX 0
$ drv2605l_haptic start -b 1

With custom multiplexer address:
$ drv2605l_haptic start -b 1 -m 0x71 -c 0 -C 1

Stop the driver:
$ drv2605l_haptic stop

Check status:
$ drv2605l_haptic status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("drv2605l_haptic", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 0, 10, "I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_INT('d', 0x5A, 0, 0xFF, "DRV2605L I2C address (hex)", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', 0x70, 0, 0xFF, "PCA9548A multiplex address (hex)", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', -1, -1, 7, "multiplex channel for DRV2605L #1 (-1 = no multiplex)", true);
	PRINT_MODULE_USAGE_PARAM_INT('C', -1, -1, 7, "multiplex channel for DRV2605L #2 (-1 = no multiplex)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

extern "C" __EXPORT int drv2605l_haptic_main(int argc, char *argv[])
{
	return DRV2605L::main(argc, argv);
}
