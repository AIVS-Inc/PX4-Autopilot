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

int AresAvs::print_status()
{
	PX4_INFO("ares_avs state: %s, hb_count: %hhu", avs_state_str::statestr[current_state], hb_count);

	return 0;
}

int AresAvs::custom_command(int argc, char *argv[])
{
	AresAvs *object;

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "event")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->event_command();	// update event params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("event: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "peak")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->peak_command();	// update peak params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("peak: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "dec")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->dec_command();	// update decimation params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("dec: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "len")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->len_command();	// update length params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("len: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "lin")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->lin_command();	// update  linear params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("lin: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "win")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->win_command();	// update window params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("win: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "enc")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->enc_command();	// update encryption params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("enc: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "sync")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					object->set_next_state(AVS_SYNC_ACK);
					object->bot_node_sync = false;
					object->top_node_sync = false;
					return object->sync_command_now();	// sync now (+5 sec)
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("sync: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "ena1")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->ena_command(true);	// enable FFT
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("ena1: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "ena0")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->ena_command(false);	// disable FFT
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("ena0: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "begin")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->begin_command();	// begin a flight sequence
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("begin: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "end")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->end_command();	// end a flight sequence
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("end: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "cal")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				if (object->current_state <= AVS_ARM_WAIT) {
					return object->cal_command();	// update cal params
				}
				else {
					PX4_INFO("must disarm system to perform this action");
					return 0;
				}
			} else {
				PX4_INFO("cal: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "cap1")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				return object->cap_command(true);	// capture on
			} else {
				PX4_INFO("cap1: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "cap0")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				return object->cap_command(false);	// capture off
			} else {
				PX4_INFO("cap0: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "rtcm1")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				return object->rtcm_command(true);	// rtcm on
			} else {
				PX4_INFO("rtcm1: task not running");
				return 1;
			}
		}
	}
	else if (!strcmp(argv[0], "rtcm0")) {
		if (is_running()) {
			object = _object.load();

			if (object) {
				return object->rtcm_command(false);	// rtcm off
			} else {
				PX4_INFO("rtcm0: task not running");
				return 1;
			}
		}
	}
	else {
		return print_usage("unknown command");
	}
	return 0;
}


int AresAvs::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ares",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
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

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "t:b:", &myoptind, &myoptarg)) != EOF) {

		switch (ch) {
		// case 't':	// top sensor node ID
		// 	nodeID_top = (uint8_t)strtoul(myoptarg, nullptr, 10);
		// 	break;

		// case 'b':	// bottom sensor node ID
		// 	nodeID_bot = (uint8_t)strtoul(myoptarg, nullptr, 10);
		// 	break;

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
	AresAvs *instance = new AresAvs();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	} else {
		PX4_INFO("ares_avs started");
	}
	return instance;
}

AresAvs::AresAvs()
	: ModuleParams(nullptr)
{
	fftEnable = false;		// FFT mode off is the default powerup state
	top_node_hb_reported = false;	// haven't received node heartbeats yet
	bot_node_hb_reported = false;
	top_node_sync = false;
	bot_node_sync = false;
}

void AresAvs::run()
{
	// run the loop synchronized to topic publications
	int sensor_avs_sub = orb_subscribe(ORB_ID(sensor_avs));
	int sensor_avs_mel_sub = orb_subscribe(ORB_ID(sensor_avs_mel));
	int sensor_gnss_relative_sub = orb_subscribe(ORB_ID(sensor_gnss_relative));
	int sensor_gps_sub = orb_subscribe(ORB_ID(sensor_gps));
	int cyphal_param_sub = orb_subscribe(ORB_ID(uavcan_parameter_value));
	int vehicle_command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));
	int manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	px4_pollfd_struct_t fds[] = {
		{.fd = sensor_avs_sub, 		 .events = POLLIN},
		{.fd = sensor_avs_mel_sub, 	 .events = POLLIN},
		{.fd = sensor_gnss_relative_sub, .events = POLLIN},
		{.fd = sensor_gps_sub, 		 .events = POLLIN},
		{.fd = cyphal_param_sub,	 .events = POLLIN},	// heartbeat or sync
		{.fd = vehicle_command_ack_sub,	 .events = POLLIN},
		{.fd = manual_control_sub,	 .events = POLLIN},
		{.fd = vehicle_status_sub,	 .events = POLLIN}
	};
	struct timespec ts = {};
	bool command_ack = false;
	bool manual_control = false;
	bool veh_status = false;

	struct vehicle_command_ack_s cmd_ack;
	struct manual_control_setpoint_s setpoint;
	struct vehicle_status_s stat;

	// initialize parameters
	parameters_update(true);

	// the first time we get an AVS packet from either node indicates that ARES GPS clock is solid for that node
	int error_counter = 0;
	while (!should_exit()) {

		/* wait for sensor update of 1 file descriptor for 250 ms */
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 250);

		if (pret == 0) {
			/*Timeout: let the loop run anyway, none of our providers are sending data */
			// don't do `continue` here
			//PX4_ERR("No data within 1/4 second...");

		} else if (pret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d, %d", pret, errno);
			}
			error_counter++;
		}
		else if (fds[0].revents & POLLIN) {
			struct sensor_avs_s sensor_avs;		// this is the main measurement result
			orb_copy(ORB_ID(sensor_avs), sensor_avs_sub, &sensor_avs);
			//PX4_INFO("got sensor_avs, node: %lu, time: %llu", sensor_avs.device_id, sensor_avs.time_utc_usec);
		}
		else if (fds[1].revents & POLLIN) {
			struct sensor_avs_mel_s sensor_avs_mel;
			orb_copy(ORB_ID(sensor_avs_mel), sensor_avs_mel_sub, &sensor_avs_mel);
			//PX4_INFO("got sensor_gnss_relative, node: %lu, time: %llu", sensor_gnss_relative.device_id, sensor_gnss_relative.timestamp);
		}
		else if (fds[2].revents & POLLIN) {
			struct sensor_gnss_relative_s sensor_gnss_relative;
			orb_copy(ORB_ID(sensor_gnss_relative), sensor_gnss_relative_sub, &sensor_gnss_relative);
			//PX4_INFO("got sensor_gnss_relative, node: %lu, time: %llu", sensor_gnss_relative.device_id, sensor_gnss_relative.timestamp);
		}
		else if (fds[3].revents & POLLIN) {
			struct sensor_gps_s sensor_gps;
			orb_copy(ORB_ID(sensor_gps), sensor_gps_sub, &sensor_gps);
			//PX4_INFO("got sensor_gps, node: %lu, time: %llu", sensor_gps.device_id, sensor_gps.timestamp);
		}
		else if (fds[4].revents & POLLIN) {
			struct uavcan_parameter_value_s cyphal_param;	// hijacking this message for heartbeat communication
			orb_copy(ORB_ID(uavcan_parameter_value), cyphal_param_sub, &cyphal_param);

			if (cyphal_param.param_type == AVS_HEARTBEAT) {
				//PX4_INFO("got heartbeat, node: %hhu, mode: %llu, time: %llu", cyphal_param.node_id, cyphal_param.int_value, cyphal_param.timestamp);
				if (cyphal_param.node_id > 0) {
					if (aresNodeId_bot == 0) {
						int32_t val;
						param_get(param_find("AVS_BOT_NODE_ID"),  &val);
						if (cyphal_param.node_id == (uint8_t)val) {
							aresNodeId_bot = (uint8_t)val;
							PX4_INFO("node %hhu registered as bottom", aresNodeId_bot);
						}
						else if (aresNodeId_top == 0) {
							aresNodeId_top = (uint8_t)val;
							PX4_INFO("node %hhu registered as top", aresNodeId_top);
						}
					}
					if (bot_node_hb_reported == false)
					{
						if (cyphal_param.int_value == 0 ){		// operational mode, has transitioned from init
							if (aresNodeId_top == cyphal_param.node_id) {
								top_node_hb_reported = true;
								PX4_INFO("node %hhu is operational", aresNodeId_top);
							}
							if (aresNodeId_bot == cyphal_param.node_id) {
								bot_node_hb_reported = true;
								PX4_INFO("node %hhu is operational", aresNodeId_bot);
							}
						}
					}
					if (aresNodeId_bot == cyphal_param.node_id) {
						hb_count++;	// increment only based on bottom node heartbeat
					}
				}
			}
			else if (cyphal_param.param_type == AVS_SYNC_OCCURRED) {
				//PX4_INFO("got sync_state, node: %hhu, state: %llu, time: %llu", cyphal_param.node_id, cyphal_param.int_value, cyphal_param.timestamp);
				// FRC (free running counter) at sync should be only a few counts away from 0 on a 75 MHz timer
				if (sync_reported() == false) {
					if ((aresNodeId_bot > 0) && (cyphal_param.node_id == aresNodeId_bot)) {
						bot_node_sync = true;
						PX4_INFO("bottom node %hhu FRC at sync: %lu", cyphal_param.node_id, (uint32_t)cyphal_param.int_value);
					}
					if ((aresNodeId_top > 0) && (cyphal_param.node_id == aresNodeId_top)) {
						top_node_sync = true;
						PX4_INFO("top node %hhu FRC at sync: %lu", cyphal_param.node_id, (uint32_t)cyphal_param.int_value);
					}
				}
			}
			//else if (cyphal_param.param_type == AVS_SYNC_MSG) {
			//	PX4_INFO("sync msg, node: %hhu, FRC: %lu", cyphal_param.node_id, (uint32_t)cyphal_param.int_value);
			//}
		}
		else if (fds[5].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_command_ack), vehicle_command_ack_sub, &cmd_ack);
			command_ack = true;
		}
		else if (fds[6].revents & POLLIN) {
			orb_copy(ORB_ID(manual_control_setpoint), manual_control_sub, &setpoint);
			manual_control = true;
		}
		else if (fds[7].revents & POLLIN) {
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &stat);
			veh_status = true;
		}
		parameters_update();

		// Run ARES measurement state machine
		switch (current_state) {
		case AVS_INIT:
			px4_clock_gettime(CLOCK_REALTIME, &ts);
			if (abs(ts.tv_sec) > 1717719180) {
				PX4_INFO("GNSS time update occurred: %lu, wait for nodes operational",ts.tv_sec);
				set_next_state(AVS_OPERATIONAL);
			}
			break;
		case AVS_OPERATIONAL:
			if (nodes_operational() == true) {
				PX4_INFO("Nodes operational, wait for RTCM ack");
				rtcm_command(true);
				set_next_state(AVS_RTCM_ON);
			}
			break;
		case AVS_RTCM_ON:
			if (command_ack == true) {
				if (nodes_reported(cmd_ack, ARES_SUBJECT_ID_GNSS_PARAMS)) {
					PX4_INFO("RTCM acknowledged, wait for preflight checks to pass");
					set_next_state(AVS_PREFLIGHT);
				}
				command_ack = false;
			}
			break;
		case AVS_PREFLIGHT:
			if (veh_status == true) {
				if (stat.pre_flight_checks_pass == true) {
					PX4_INFO("System passes preflight checks, wait for RC or manual command to start");
					set_next_state(AVS_BEGIN);
				}
				veh_status = false;
			}
			else if (orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &stat) == PX4_OK) {
				// Check if the preflight check passed
				if (stat.pre_flight_checks_pass == true) {
					set_next_state(AVS_BEGIN);
				}
			}
			break;
		case AVS_BEGIN:
			if (manual_control == true) {
				if (setpoint.aux1 > 0.3f) {
					PX4_INFO("System commanded by RC to start mission, initiate capture");
					cap_command( true);
					set_next_state(AVS_CAPTURE_ON);

					// TMP
					// Capture On is not properly acknowledged, so skip to do sync
					bot_node_sync = false;
					top_node_sync = false;
					sync_command_now();
					set_next_state(AVS_SYNC_ACK);
					// end TMP
				}
				manual_control = false;
			}
			break;
		case AVS_CAPTURE_ON:
			if (hb_count > 1) {	// delay before sync
				if (command_ack == true) {
					if (nodes_reported(cmd_ack, ARES_SUBJECT_ID_STORAGE_CONTROL)) {
						PX4_INFO("Capture command acknowledged, initiate sync");
						bot_node_sync = false;
						top_node_sync = false;
						sync_command_now();
						set_next_state(AVS_SYNC_ACK);
					}
					command_ack = false;
				}
			}
			break;
		case AVS_SYNC_ACK:
			if (command_ack == true) {
				if (nodes_reported(cmd_ack, ARES_SUBJECT_ID_ADC_SYNC)) {
					PX4_INFO("Got sync ack, wait for sync confirmation");
					set_next_state(AVS_SYNC_WAIT);
				}
				command_ack = false;
			}
			break;
		case AVS_SYNC_WAIT:	// sync is at least a 5 second process
			if (hb_count > 6) {	// eventually, wait for ARES to denote SyncState
				if (sync_reported() == true) {
					PX4_INFO("Sync confirmed, wait for arm");
					set_next_state(AVS_ARM_WAIT);
				}
			}
			break;
		case AVS_ARM_WAIT:
			if (veh_status == true) {
				if (stat.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					PX4_INFO("manual (ground control) command to arm");
					set_next_state(AVS_ARMED);
				}
				veh_status = false;
			}
			break;
		case AVS_ARMED:
			//PX4_INFO("AVS armed");
			set_next_state(arm_action(manual_control, setpoint, veh_status, stat));
			break;
		case AVS_TAKEOFF:
			//PX4_INFO("AVS takeoff");
			set_next_state(arm_action(manual_control, setpoint, veh_status, stat));
			break;
		case AVS_HOLD:
			//PX4_INFO("AVS hold");
			set_next_state(arm_action(manual_control, setpoint, veh_status, stat));
			break;
		case AVS_POSITION:
			//PX4_INFO("AVS position");
			set_next_state(arm_action(manual_control, setpoint, veh_status, stat));
			break;
		case AVS_LAND:
			//PX4_INFO("AVS land");
			set_next_state(arm_action(manual_control, setpoint, veh_status, stat));
			break;
		case AVS_DISARMED:
			PX4_INFO("Disarmed after mission, go to capture off");
			cap_command( false);
			set_next_state(AVS_CAPTURE_OFF);

			// TMP - workaround non-acknowledgement of "sd cap off"
			set_next_state(AVS_PREFLIGHT);
			// TMP end
			break;
		case AVS_CAPTURE_OFF:
			if (hb_count > 2) {
				if (command_ack == true) {
					if (nodes_reported(cmd_ack, ARES_SUBJECT_ID_STORAGE_CONTROL)) {
						set_next_state(AVS_PREFLIGHT);
					}
					command_ack = false;
				}
			}
			break;
		case AVS_END:
			if (manual_control == true) {
				if (setpoint.aux1 > 0.3f) {
					PX4_INFO("Waiting for command to end switch state");
				}
				else if (fabs(setpoint.aux1) < 0.1) {
					PX4_INFO("Mission sequence ends here, wait for BEGIN");
					set_next_state(AVS_BEGIN);
				}
				manual_control = false;
			}
			break;
		}
	}
	orb_unsubscribe(sensor_avs_sub);
	orb_unsubscribe(sensor_avs_mel_sub);
	orb_unsubscribe(sensor_gnss_relative_sub);
	orb_unsubscribe(sensor_gps_sub);
	orb_unsubscribe(cyphal_param_sub);
	orb_unsubscribe(vehicle_command_ack_sub);
	orb_unsubscribe(manual_control_sub);
	orb_unsubscribe(vehicle_status_sub);
}

void AresAvs::set_next_state(int32_t state)
{
	current_state = state;
	hb_count = 0;
}

bool AresAvs::sync_reported(void)
{
	if ((aresNodeId_bot > 0) && (aresNodeId_top > 0) && (bot_node_sync == true) && (top_node_sync == true)) {
		return true;
	}
	else if ((aresNodeId_bot > 0) && (bot_node_sync == true)) {
		return true;
	}
	else if ((aresNodeId_top > 0) && (top_node_sync == true)) {
		return true;
	}
	else {
		return false;
	}
}

bool AresAvs::nodes_operational(void)
{
	bool operational = false;

	if ((aresNodeId_bot > 0) && (aresNodeId_top > 0)) {
		if (top_node_hb_reported && bot_node_hb_reported) {
			operational = true;
		}
	}
	else if ((aresNodeId_bot > 0) && bot_node_hb_reported) {
		operational = true;
	}
	else if ((aresNodeId_top > 0) && top_node_hb_reported) {
		operational = true;
	}
	return operational;
}

bool AresAvs::nodes_reported(vehicle_command_ack_s recvd_ack, const uint32_t subjectID)
{
	bool ack = false;

	if ((aresNodeId_bot > 0) &&
	    (recvd_ack.command == subjectID) &&
	    (recvd_ack.target_system == aresNodeId_bot) &&
	    (recvd_ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED)) {
		bot_node_ack_reported = true;
	}
	if ((aresNodeId_top > 0) &&
	    (recvd_ack.command == subjectID) &&
	    (recvd_ack.target_system == aresNodeId_top) &&
	    (recvd_ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED)) {
		top_node_ack_reported = true;
	}
	if ((aresNodeId_bot > 0) && (aresNodeId_top > 0)) {
		if ((bot_node_ack_reported == true) && (top_node_ack_reported == true)) {
			ack = true;
		}
	}
	else if (aresNodeId_bot > 0) {
		if (bot_node_ack_reported == true) {
			ack = true;
		}
	}
	else if (aresNodeId_top > 0) {
		if (top_node_ack_reported == true) {
			ack = true;
		}
	}
	if (ack) {
		bot_node_ack_reported = false;
		top_node_ack_reported = false;
		return true;
	}
	else {
		return false;
	}
}

int32_t AresAvs::get_next_nav_state(uint8_t nav_state )
{
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		return(AVS_TAKEOFF);
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		return(AVS_HOLD);
		break;
	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
	case vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW:
		return(AVS_POSITION);
		break;
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		return(AVS_LAND);
		break;
	default:
		PX4_ERR("Unexpected nav_state %hhd", nav_state);
		return(AVS_HOLD);
	}
}

int32_t AresAvs::arm_action(bool manual_control, manual_control_setpoint_s setpoint, bool veh_status, vehicle_status_s stat)
{
	int32_t next_state = current_state;

	if (veh_status == true) {
		if (stat.arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
			PX4_INFO("Disarmed after armed, go to capture off");
			next_state = AVS_DISARMED;
		}
		else if (stat.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			next_state = get_next_nav_state(stat.nav_state);
			if (next_state != current_state) {
				PX4_INFO("System armed, nav_state: %hhd", stat.nav_state);
			}
		}
		veh_status = false;
	}
	return next_state;
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
ares_avs is a background task with start/stop/send/status functionality.

### Implementation
Reads uORB messages send in response to received ARES CAN data.
Also allows sending of ARES event parameters when given the node IDs on the CAN bus
Event parameters must be set as desired prior to invocation of the send command

### Examples
CLI usage (parameter should be one of the following):
$ ares_avs help		// display this help
	   start	// start the ARES app, begin monitoring CAN messages
	   stop		// stop the ARES application
	   begin	// begin a flight sequence
	   end		// end the current flight sequence
	   status	// reply with run status
	   cap1		// enable SD raw data capture
	   cap0		// disable SD raw data capture
	   rtcm1	// enable rtcm data for base and rover
	   rtcm0	// disable rtcm data for base and rover
	   event	// send event parameters to ARES
	   peak 	// send peak parameters to ARES
	   dec		// send decimation and spatial filter params
	   len		// send FFT length
	   lin		// set FFT linear bin start, number bins
	   win		// set Hann window on/off
	   enc		// set data encryption on/off
	   sync		// sync ARES ADCs
	   ena1		// enable event detection
	   ena0		// disable event detection
	   cal		// recalculate FFT corrections on all node_ids

There are no optional arguments, e.g., :
$ ares_avs start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ares_avs", "template");
	PRINT_MODULE_USAGE_COMMAND("start");	// start app
	PRINT_MODULE_USAGE_COMMAND("stop");	// stop app
	PRINT_MODULE_USAGE_COMMAND("begin");	// begin flight sequence
	PRINT_MODULE_USAGE_COMMAND("end");	// end flight sequence
	PRINT_MODULE_USAGE_COMMAND("status");	// reply if running or not
	PRINT_MODULE_USAGE_COMMAND("cap1");	// SD capture on
	PRINT_MODULE_USAGE_COMMAND("cap0");	// SD capture off
	PRINT_MODULE_USAGE_COMMAND("rtcm1");	// RTCM on
	PRINT_MODULE_USAGE_COMMAND("rtcm0");	// RTCM off
	PRINT_MODULE_USAGE_COMMAND("event");	// send event params to all node_ids
	PRINT_MODULE_USAGE_COMMAND("peak");	// send peak params to all node_ids
	PRINT_MODULE_USAGE_COMMAND("dec");	// send FFT decimation (overlap)
	PRINT_MODULE_USAGE_COMMAND("len");	// send FFT length
	PRINT_MODULE_USAGE_COMMAND("lin");	// send FFT linear bin start, number bins
	PRINT_MODULE_USAGE_COMMAND("win");	// send Hann window on/off
	PRINT_MODULE_USAGE_COMMAND("enc");	// send data encryption on/off
	PRINT_MODULE_USAGE_COMMAND("sync");	// sync ARES ADCs
	PRINT_MODULE_USAGE_COMMAND("ena1");  	// enable FFT on all node_ids
	PRINT_MODULE_USAGE_COMMAND("ena0");	// disable FFT on all node_ids
	PRINT_MODULE_USAGE_COMMAND("cal");	// recalculate FFT corrections on all node_ids
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int AresAvs::event_command()		// update event params in ARES, enable/disable FFT
{
	int32_t val;

	if (fftEnable == true){
		ena_command( false);
	}

	/* advertise avs_evt_control topic */
	struct sensor_avs_evt_control_s evt;
	memset(&evt, 0, sizeof(evt));
	orb_advert_t evt_pub = orb_advertise(ORB_ID(sensor_avs_evt_control), &evt);

	param_get(param_find("AVS_EVT_NUM_SRC"), &val); evt.num_sources = (uint16_t)val;
	param_get(param_find("AVS_EVT_BG_TC"),   &val); evt.bg_timeconst = (uint16_t)val;
	param_get(param_find("AVS_EVT_REL_DB"),  &val); evt.relative_db = (int8_t)val;
	param_get(param_find("AVS_EVT_ANG_RES"), &val); evt.angular_resln = (uint8_t)val;
	param_get(param_find("AVS_EVT_EVT_WIN"), &val); evt.event_window = (uint8_t)val;
	param_get(param_find("AVS_EVT_BGSIL"),   &val); evt.self_measure_bg = (bool)val;
	param_get(param_find("AVS_EVT_BGSIL_DB"),&val); evt.bg_db_threshold = (uint8_t)val;

	evt.fft_param_id = ares_fft_ParamId_EventDetector;
	evt.node_top = aresNodeId_top;
	evt.node_bot = aresNodeId_bot;
	evt.fft_enable = fftEnable;

	PX4_INFO("Send event parameters to ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	orb_publish(ORB_ID(sensor_avs_evt_control), evt_pub, &evt);

	return 0;
}

int AresAvs::peak_command()		// update event params in ARES, enable/disable FFT
{
	int32_t val;

	if (fftEnable == true){
		ena_command( false);
	}

	/* advertise avs_peak_control topic */
	struct sensor_avs_peak_control_s peak;
	memset(&peak, 0, sizeof(peak));
	orb_advert_t peak_pub = orb_advertise(ORB_ID(sensor_avs_peak_control), &peak);

	param_get(param_find("AVS_PK_NUM_HARM"), &val); peak.max_harmonics = (uint8_t)val;
	param_get(param_find("AVS_PK_NUM_PEAK"), &val); peak.num_peaks = (uint8_t)val;
	param_get(param_find("AVS_PK_MIN_HGHT"), &val); peak.min_peak_height = (float)val;
	param_get(param_find("AVS_PK_MIN_D_DB"), &val); peak.min_peak_change = (float)val;
	param_get(param_find("AVS_PK_MIN_BLNK"), &val); peak.min_blanking = (float)val;

	peak.fft_param_id = ares_fft_ParamId_PeakDetector;
	peak.node_top = aresNodeId_top;
	peak.node_bot = aresNodeId_bot;
	peak.fft_enable = fftEnable;

	PX4_INFO("Send event parameters to ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	orb_publish(ORB_ID(sensor_avs_peak_control), peak_pub, &peak);

	return 0;
}

int AresAvs::send_fft_params( ares_fft_ParamId param_id)
{
	int32_t val;

	if (fftEnable == true){
		ena_command( false);	// FFT must be diabled to make these changes
	}
	/* advertise avs_fft_params topic */
	struct sensor_avs_fft_params_s params;
	memset(&params, 0, sizeof(params));
	orb_advert_t params_pub = orb_advertise(ORB_ID(sensor_avs_fft_params), &params);

	param_get(param_find("AVS_FFT_DEC"), &val); params.fft_dec = (uint8_t)val;
	param_get(param_find("AVS_SPATIAL_FILT"), &val); params.fft_spatial = (uint8_t)val;
	param_get(param_find("AVS_FFT_LONG"), &val);
	if (val == 1){	// true
		params.fft_max_bins = FFT_LONG_NUM_BINS;
		params.fft_num_blocks = FFT_LONG_NUM_BUFFERS;
	} else {
		params.fft_max_bins = FFT_SHORT_NUM_BINS;
		params.fft_num_blocks = FFT_SHORT_NUM_BUFFERS;
	}
	param_get(param_find("AVS_FFT_ENCRYPT"), &val); params.fft_encrypt = (float)val;
	param_get(param_find("AVS_FFT_STRT_BIN"), &val); params.fft_start_bin = (float)val;
	param_get(param_find("AVS_FFT_NUM_BINS"), &val); params.fft_num_bins = (float)val;
	param_get(param_find("AVS_FFT_WINDOW"), &val); params.fft_window = (float)val;

	params.fft_param_id = param_id;
	params.node_top = aresNodeId_top;
	params.node_bot = aresNodeId_bot;
	params.fft_enable = fftEnable;

	PX4_INFO("Send FFT parameters to ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	orb_publish(ORB_ID(sensor_avs_fft_params), params_pub, &params);

	return 0;
}

int AresAvs::dec_command()		// send FFT decimation
{
	return send_fft_params(ares_fft_ParamId_OutputDecimator);
}

int AresAvs::len_command()		// sendo FFT length
{
	return send_fft_params(ares_fft_ParamId_Length);
}

int AresAvs::lin_command()		// send FFT linear bin start, num_bins
{
	return send_fft_params(ares_fft_ParamId_Linear);
}

int AresAvs::win_command()		// send FFT win on/off
{
	return send_fft_params(ares_fft_ParamId_HannWindow);
}

int AresAvs::enc_command()		// send FFT encryption on/off
{
	return send_fft_params(ares_fft_ParamId_EncryptOutput);
}

int AresAvs::sync_command_now()
{
	//get system time at the last whole second
	struct timespec ts = {};
	px4_clock_gettime(CLOCK_REALTIME, &ts);

	return sync_command( ts.tv_sec);
}

int AresAvs::sync_command( time_t time_s)	// send an ADC sync to ARES at a specific time
{
	// do a system sync 5 seconds from the last whole second
	time_s += 5;
	char buf[80];	// display the planned sync time
	struct tm date_time;
	localtime_r(&time_s, &date_time);
	strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &date_time);

	PX4_INFO("Sync @ epoch time: %ld", (long)time_s);
	PX4_INFO("Sync time of day: %s", buf);

	/* advertise avs_sync_control topic */
	struct sensor_avs_sync_control_s sync;
	memset(&sync, 0, sizeof(sync));
	orb_advert_t sync_pub = orb_advertise(ORB_ID(sensor_avs_sync_control), &sync);

	sync.node_top = aresNodeId_top;
	sync.node_bot = aresNodeId_bot;
	sync.sync_utc_sec = (uint64_t) time_s;

	PX4_INFO("Send SYNC to ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	orb_publish(ORB_ID(sensor_avs_sync_control), sync_pub, &sync);

	/* Record for logging purposes */
	int32_t sync_secs = (int32_t)sync.sync_utc_sec;
	param_set(param_find("AVS_TARGET_SYNC"), &sync_secs);	// FIX, wraps in year 2038
	return 0;
}

int AresAvs::ena_command( bool flag)		// update event params in ARES, enable/disable FFT
{
	/* advertise avs_fft_control topic */
	struct sensor_avs_fft_control_s fft;
	memset(&fft, 0, sizeof(fft));
	orb_advert_t fft_pub = orb_advertise(ORB_ID(sensor_avs_fft_control), &fft);

	fft.node_top = aresNodeId_top;
	fft.node_bot = aresNodeId_bot;
	fft.fft_param_id = FftControlId_Enable;
	fftEnable = flag;
	fft.fft_enable = flag;

	if (flag == true)
		PX4_INFO("Enable FFT on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	else
		PX4_INFO("Disable FFT on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);

	orb_publish(ORB_ID(sensor_avs_fft_control), fft_pub, &fft);

	return 0;
}

int AresAvs::cal_command()			// recompute FFT correction vectors
{
	if (fftEnable == true){
		ena_command( false);	// FFT must be diabled to make these changes
	}
	/* advertise avs_fft_control topic */
	struct sensor_avs_fft_control_s fft;
	memset(&fft, 0, sizeof(fft));
	orb_advert_t fft_pub = orb_advertise(ORB_ID(sensor_avs_fft_control), &fft);

	fft.node_top = aresNodeId_top;
	fft.node_bot = aresNodeId_bot;
	fft.fft_param_id = FftControlId_CalibrateFromFile;
	fft.fft_enable = fftEnable;

	PX4_INFO("Recalculate FFT correction vectors on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);

	orb_publish(ORB_ID(sensor_avs_fft_control), fft_pub, &fft);

	return 0;
}

int AresAvs::begin_command()			// begin a flight sequence manually
{
	PX4_INFO("Start new AVS flight sequence by manual command, initiate capture");
	cap_command( true);
	set_next_state(AVS_CAPTURE_ON);
	return 0;
}

int AresAvs::end_command()			// end a flight sequence
{
	PX4_INFO("End AVS flight sequence by manual command");
	set_next_state(AVS_END);
	return 0;
}

int AresAvs::cap_command( bool flag)		// update event params in ARES, enable/disable FFT
{
	/* advertise avs_sd_control topic */
	struct sensor_avs_sd_control_s sd;
	memset(&sd, 0, sizeof(sd));
	orb_advert_t sd_pub = orb_advertise(ORB_ID(sensor_avs_sd_control), &sd);

	sd.node_top = aresNodeId_top;
	sd.node_bot = aresNodeId_bot;
	sd.sd_capture = flag;

	if (flag == true)
		PX4_INFO("Enable SD capture on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	else
		PX4_INFO("Disable SD capture on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);

	orb_publish(ORB_ID(sensor_avs_sd_control), sd_pub, &sd);

	return 0;
}

int AresAvs::rtcm_command( bool flag)		// update event params in ARES, enable/disable FFT
{
	/* advertise avs_rtcm_control topic */
	struct sensor_avs_gnss_control_s rtcm;
	memset(&rtcm, 0, sizeof(rtcm));
	orb_advert_t rtcm_pub = orb_advertise(ORB_ID(sensor_avs_gnss_control), &rtcm);

	rtcm.node_top = aresNodeId_top;
	rtcm.node_bot = aresNodeId_bot;
	rtcm.rtcm_mode = flag;

	if (aresNodeId_top > 0)
	{
		if (flag == true)
			PX4_INFO("Enable Moving baseline on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
		else
			PX4_INFO("Disable Moving baseline on ARES nodes: %hd, %hd", aresNodeId_top, aresNodeId_bot);
	}
	if (aresNodeId_bot > 0)
	{
		if (flag == true)
			PX4_INFO("Enable Ntrip corrections on ARES base node: %hd", aresNodeId_bot);
		else
			PX4_INFO("Disable Ntrip corrections on ARES base node: %hd", aresNodeId_bot);
	}
	orb_publish(ORB_ID(sensor_avs_gnss_control), rtcm_pub, &rtcm);

	return 0;
}

int ares_avs_main(int argc, char *argv[])
{
	return AresAvs::main(argc, argv);
}
