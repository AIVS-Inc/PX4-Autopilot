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

#include "ares_avs.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_avs.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include <uORB/topics/sensor_gps.h>
//#include <uORB/topics/sensor_avs_adc.h>
#include <uORB/topics/sensor_avs_evt_control.h>


int AresAvs::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int AresAvs::custom_command(int argc, char *argv[])
{
	AresAvs *object;

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "send")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				return object->send_command();	// update event params in ARES, enable/disable FFT
			} else {
				PX4_INFO("evt send: task not running");
				return 1;
			}
		}
	} else {
		return print_usage("unknown command");
	}
	return 0;
}


int AresAvs::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ares",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}
	return 0;
}

AresAvs *AresAvs::instantiate(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	uint8_t nodeID_top = 0;	// default if node not present
	uint8_t nodeID_bot = 0;	// default if node not present

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "t:b:", &myoptind, &myoptarg)) != EOF) {
		PX4_INFO("parse arg: %c", *myoptarg);

		switch (ch) {
		case 't':	// top sensor node ID
			nodeID_top = atoi(myoptarg);
			break;

		case 'b':	// bottom sensor node ID
			nodeID_bot = atoi(myoptarg);
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}
	if (error_flag) {
		return nullptr;
	}

	AresAvs *instance = new AresAvs(nodeID_top, nodeID_bot);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	} else {
		PX4_INFO("ares_avs started");
	}
	return instance;
}

AresAvs::AresAvs(uint8_t nodeID_top, uint8_t nodeID_bot)
	: ModuleParams(nullptr)
{
	aresNodeId_top = nodeID_top;
	aresNodeId_bot = nodeID_bot;
}

void AresAvs::run()
{
	// run the loop synchronized to topic publications
	int sensor_avs_sub = orb_subscribe(ORB_ID(sensor_avs));
	int sensor_gnss_relative_sub = orb_subscribe(ORB_ID(sensor_gnss_relative));
	int sensor_gps_sub = orb_subscribe(ORB_ID(sensor_gps));
	//int sensor_avs_adc_sub = orb_subscribe(ORB_ID(sensor_avs_adc));

	px4_pollfd_struct_t fds[] = {
		{.fd = sensor_avs_sub, 		 .events = POLLIN},
		{.fd = sensor_gnss_relative_sub, .events = POLLIN},
		{.fd = sensor_gps_sub, 		 .events = POLLIN}
		//{.fd = sensor_avs_adc_sub, 	 .events = POLLIN}
	};

	// initialize parameters
	parameters_update(true);

	int error_counter = 0;
	while (!should_exit()) {

		/* wait for sensor update of 1 file descriptor for 1000 ms */
		int pret = px4_poll(fds, 1, 1000);

		if (pret == 0) {
			/*Timeout: let the loop run anyway, none of our providers are sending data */
			// don't do `continue` here
			//PX4_ERR("No data within a second...");

		} else if (pret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d, %d", pret, errno);
			}
			error_counter++;
		}
		else if (fds[0].revents & POLLIN) {
			struct sensor_avs_s sensor_avs;
			orb_copy(ORB_ID(sensor_avs), sensor_avs_sub, &sensor_avs);
			// TODO: do something with the data...
			PX4_INFO("got sensor_avs, node: %lu, time: %lld", sensor_avs.device_id, sensor_avs.timestamp);
		}
		else if (fds[1].revents & POLLIN) {
			struct sensor_gnss_relative_s sensor_gnss_relative;
			orb_copy(ORB_ID(sensor_gnss_relative), sensor_gnss_relative_sub, &sensor_gnss_relative);
			// TODO: do something with the data...
			PX4_INFO("got sensor_gnss_relative, node: %lu, time: %lld", sensor_gnss_relative.device_id, sensor_gnss_relative.timestamp);
		}
		else if (fds[2].revents & POLLIN) {
			struct sensor_gps_s sensor_gps;
			orb_copy(ORB_ID(sensor_gps), sensor_gps_sub, &sensor_gps);
			// TODO: do something with the data...
			PX4_INFO("got sensor_gps, node: %lu, time: %lld", sensor_gps.device_id, sensor_gps.timestamp);
		}
		// else if (fds[3].revents & POLLIN) {
		// 	struct sensor_avs_adc_s sensor_avs_adc;
		// 	orb_copy(ORB_ID(sensor_avs_adc), sensor_avs_adc_sub, &sensor_avs_adc);
		// 	// TODO: do something with the data...
		// 	PX4_INFO("got sensor_avs_adc, node: %lu, tID: %lu, time: %lld", sensor_avs_adc.device_id, sensor_avs_adc.transfer_id, sensor_avs_adc.timestamp);
		// }
		parameters_update();
	}
	orb_unsubscribe(sensor_avs_sub);
	orb_unsubscribe(sensor_gnss_relative_sub);
	orb_unsubscribe(sensor_gps_sub);
	//orb_unsubscribe(sensor_avs_adc_sub);
}

void AresAvs::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int AresAvs::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ares_drone is a background task with start/stop/send/status functionality.

### Implementation
Reads uORB messages send in response to received ARES CAN data.
Also allows "send"ing of ARES event parameters when given the node IDs on the CAN bus
Event parameters must be set as desired prior to invocation of the send command

### Examples
CLI usage (parameter should be one of the following):
$ ares_drone help/start/status/send/stop

With optional arguments:
$ ares_drone start -d -t 6 -b 24

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ares_drone", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("send");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_PARAM_INT('t', 6, 1, 127, "AVS node ID (top)", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 24, 1, 127, "AVS node ID (bottom)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int AresAvs::send_command()		// update event params in ARES, enable/disable FFT
{
	int32_t val;

	/* advertise avs_evt_control topic */
	struct sensor_avs_evt_control_s evt;
	memset(&evt, 0, sizeof(evt));
	orb_advert_t evt_pub = orb_advertise(ORB_ID(sensor_avs_evt_control), &evt);

	param_get(param_find("EVT_NUM_SRC"), &val); evt.num_sources = (uint16_t)val;
	param_get(param_find("EVT_BG_TC"),   &val); evt.bg_timeconst = (uint16_t)val;
	param_get(param_find("FFT_ENABLE"),  &val); evt.fft_enable = (uint8_t)val;
	param_get(param_find("EVT_REL_DB"),  &val); evt.relative_db = (int8_t)val;
	param_get(param_find("EVT_ANG_RES"), &val); evt.angular_resln = (uint8_t)val;
	param_get(param_find("EVT_EVT_WIN"), &val); evt.event_window = (uint8_t)val;

	PX4_INFO("Send event parameters to ARES nodes:");
	orb_publish(ORB_ID(sensor_avs_evt_control), evt_pub, &evt);

	return 0;
}

int ares_avs_main(int argc, char *argv[])
{
	return AresAvs::main(argc, argv);
}
