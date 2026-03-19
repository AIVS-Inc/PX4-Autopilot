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

#include "drv2605l_haptic.h"


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

	// string search once at startup
	_param_left_node_id  = param_find("AVS_BOT_NODE_ID");
	_param_right_node_id = param_find("AVS_TOP_NODE_ID");
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

bool DRV2605L::check_act_int_threshold(float active_int)
{
	// Check if active intensity greater than 70
	return (active_int >= _act_int.get());
}

char DRV2605L::is_elevation_in_range(float elevation)  // up/down
{
	// Check if elevation is within range
	char c = 'N'; // declare once

	if (elevation >= _elevation_min.get() && elevation <= _elevation_max.get()){
		c = 'T'; //assign
	} else {
		c = 'N'; // no haptic effect
	}
	return c;
}


char DRV2605L::is_azimuth_in_range(float azimuth) //float azimuth
{
	// Check if azimuth is in range
	char c = 'N'; // declare once

	if (azimuth >= _azimuth_min.get() && azimuth <= _azimuth_max.get()){
		c = 'B'; //assign
	} else {
		c = 'N'; // no haptic effect
	}
	return c;
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

	// initial read of top/bottom node IDs
	int32_t left_node_id = 44, right_node_id = 41;
	param_get(_param_left_node_id, &left_node_id);
	param_get(_param_right_node_id, &right_node_id);

	// Main loop
	while (!should_exit()) {

		// Check for parameter updates
		if (_parameter_update_sub.updated()) { // if any update to param_update uorb message (don't know which param changed yet)
			parameter_update_s param_update;  // if updated, then copy it and update parameters to clear pending update
			_parameter_update_sub.copy(&param_update);


			parameters_update();

			// if updated, get new top/bottom node IDs
			param_get(_param_left_node_id, &left_node_id);
			param_get(_param_right_node_id, &right_node_id);

			PX4_INFO("Parameters updated");
		}

		//Check for new AVS data on both instances
		sensor_avs_lite_ext_s data0{}, data1{}; // {} initializes all fields to zero
		bool avs_updated0 = _sensor_avs_lite_ext_sub_0.update(&data0);
		bool avs_updated1 = _sensor_avs_lite_ext_sub_1.update(&data1);

		// Static variables to preserve latest values across loop iterations
		static char back_side0 = 'N';
		static char top_side0 = 'N';
		static char back_side1 = 'N';
		static char top_side1 = 'N';
		static bool greater_threshold = false;

		float trig_timer = _trig_timer.get();

		if (avs_updated0){

			uint32_t node = data0.device_id;
			float yaw  = data0.yaw;
    			float roll = data0.roll;

			float active_int = data0.active_intensity;  // get active intensity value
			float elevation = data0.elevation_deg; // get elevation value
    			float azimuth = data0.azimuth_deg;  // get azimuth value
			greater_threshold = check_act_int_threshold(active_int); // check if greater than threshold value

			PX4_INFO("Active Intensity: %.2f",(double)active_int);

			if (_mode.get() == 0){  //IMU
				back_side0 = is_yaw_in_range(yaw); 		// Check and determine if yaw is in target range
				top_side0 = is_roll_in_range(roll); 		// Check and determine if roll is in target range
				//PX4_INFO("using IMU mode");

				PX4_INFO("Node: %lu |Yaw: %.2f deg | Roll: %.2f deg | Haptic Back Side: %c | Haptic Top Side: %c", (unsigned long)node,(double)yaw,  (double)roll, back_side0, top_side0);
			}

			if (_mode.get() == 1){  //AVS
				back_side0 = is_azimuth_in_range(azimuth);  // Check and determine if azimuth is in range
				top_side0 = is_elevation_in_range(elevation); // Check and determine if elevation is in range
				//PX4_INFO("using AVS mode");
				PX4_INFO("Node: %lu | Azimuth: %.2f | Elevation: %.2f | Haptic Back Side: %c | Haptic Top Side: %c", (unsigned long)node ,(double)azimuth, (double)elevation, back_side0, top_side0);
			}
		}

		if (avs_updated1){

			uint32_t node = data1.device_id;
			float active_int = data1.active_intensity;  // get active intensity value
			float elevation = data1.elevation_deg; // get elevation value
    			float azimuth = data1.azimuth_deg;  // get azimuth value
			//greater_threshold = check_act_int_threshold(active_int); // check if greater than threshold value

			PX4_INFO("Active Intensity: %.2f",(double)active_int);

			// if (_mode.get() == 0){  //IMU
			// 	back_side = is_yaw_in_range(yaw); 		// Check and determine if yaw is in target range
			// 	top_side = is_roll_in_range(roll); 		// Check and determine if roll is in target range
			// 	//PX4_INFO("using IMU mode");

			// 	PX4_INFO("Node: %lu |Yaw: %.2f deg | Roll: %.2f deg | Haptic Back Side: %c | Haptic Top Side: %c", (unsigned long)node,(double)yaw,  (double)roll, back_side, top_side);
			// }

			if (_mode.get() == 1){  //AVS
				back_side1 = is_azimuth_in_range(azimuth);  // Check and determine if azimuth is in range
				top_side1 = is_elevation_in_range(elevation); // Check and determine if elevation is in range
				//PX4_INFO("using AVS mode");
				PX4_INFO("Node: %lu | Azimuth: %.2f | Elevation: %.2f | Haptic Back Side: %c | Haptic Top Side: %c", (unsigned long)node ,(double)azimuth, (double)elevation, back_side1, top_side1);
			}
		}

		//Only trigger if in haptic yaw range, haptic pitch range, & exceed active intensity threshold
		//if ((back_side != 'N' || top_side != 'N') && greater_threshold ) {
		if ((back_side0 != 'N' || top_side0 != 'N' || back_side1 != 'N' || top_side1 != 'N') && greater_threshold ) {

			// Get the effect number from parameter
			uint8_t effectT = static_cast<uint8_t>(_drv_effect_t.get());
			uint8_t effectB = static_cast<uint8_t>(_drv_effect_b.get());

			// Print the effect value being used
			//PX4_INFO("Triggering haptic effect right: %d | left: %d", effectR, effectL);

			if (_use_multiplex){ // if using multiplexer
				if (back_side0 == 'B' and back_side1 == 'B'){  // if either instance triggers backside haptic
					// Trigger backside haptic
					int ret1 = trigger_effect(_multiplex_channel1, effectB);
					//int ret2 = trigger_effect(_multiplex_channel2, effectT);

					if (ret1 != OK) {
						PX4_ERR("Failed to trigger effect");
					}
					px4_usleep((useconds_t)(trig_timer * 1000000));
				}
				if (top_side0 == 'T' and top_side1 == 'T'){  // if either instance triggers topside haptic
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
