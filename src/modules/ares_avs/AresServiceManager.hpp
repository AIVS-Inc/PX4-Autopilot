/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file AresServiceManager.hpp
 *
 * Manages the AFES service subscriptions
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include "../../drivers/cyphal/CanardInterface.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/uavcan_parameter_value.h>
#include <uORB/topics/sensor_avs_cmd_ack.h>
#include <uavcan/node/Heartbeat_1_0.h>
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"
#include "AresFftParamServiceRequest.hpp"
#include "AresFftControlServiceRequest.hpp"
#include "AresSdCapControlServiceRequest.hpp"
#include "AresGnssControlServiceRequest.hpp"
#include "AresSyncControlServiceRequest.hpp"
#include "UavCanId.h"
#include <containers/List.hpp>

class AresNode : public ListNode<AresNode *>
{
public:
	uint8_t node_id {0};
	uint32_t uptime {0};
	uint32_t hb_cnt {0};		// counts heartbeats for this node
	hrt_abstime last_hb {0};	// absolute time of last heartbeat
};

class AresServiceManager : public UavcanBaseSubscriber, public UavcanServiceRequestInterface
{
public:
	AresServiceManager( CanardHandle &handle ) : 	UavcanBaseSubscriber(handle, "", "Heartbeat", 0),
							_canard_handle(handle),
						 	_fft_param(handle, this),	// this->response_callback handles responses
							_fft_control(handle, this),
							_sd_cap_control(handle, this),
							_gnss_control(handle, this),
							_sync_control(handle, this) { };

	void subscribe() override
	{
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
					   uavcan_node_Heartbeat_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		// advertise publication of detected ARES node IDs
		memset(&this->heartbeat, 0, sizeof(this->heartbeat));
		this->heartbeat_pub = orb_advertise(ORB_ID(uavcan_parameter_value), &this->heartbeat);
		PX4_INFO("subscribed to Heartbeat msgs, port %u", uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);

		// Subscribe to responses to various resquests
		_fft_param.subscribe();
		_fft_control.subscribe();
		_sd_cap_control.subscribe();
		_gnss_control.subscribe();
		_sync_control.subscribe();
	}
	void callback(const CanardRxTransfer &receive) override
	{
		// heartbeat subscription callback
		uavcan_node_Heartbeat_1_0 hb{};
		size_t msg_size_in_bits = receive.payload_size;
		uavcan_node_Heartbeat_1_0_deserialize_(&hb, (const uint8_t *)receive.payload, &msg_size_in_bits);

		heartbeat.timestamp = hrt_absolute_time();
		heartbeat.node_id = (uint8_t) receive.metadata.remote_node_id;
		heartbeat.int_value = (int64_t) hb.mode.value;
		heartbeat.param_type = (uint8_t) AVS_HEARTBEAT;

		// Check if this node is on the list
		bool found_node = false;
		hrt_abstime current_time = heartbeat.timestamp; // microseconds

		for (auto &n : _nodes) {
			if (n->node_id == heartbeat.node_id) {
				found_node = true;
				n->last_hb = current_time;
			}
		}
		if (found_node == false) {
			// add a new node to the list
			AresNode *n = new AresNode();
			n->node_id = heartbeat.node_id;
			n->uptime = hb.uptime;
			n->hb_cnt = 0;
			n->last_hb = current_time;

			_nodes.add(n);
			PX4_INFO("add ARES node %hd: ", n->node_id);
		}
		// Remove any nodes that are no longer reporting heartbeats
		for (auto &n : _nodes) {
			if ((n->last_hb > 0) && (current_time - 2000000ULL) > n->last_hb) {	// 2 sec
				_nodes.remove(n);
				PX4_WARN("remove ARES node %hd: ", n->node_id);
			}
		}
		// Check that all active commands have been processed. If not acknowledged, raise warning.
		hrt_abstime cmd_time;

		for (auto &n : _nodes)
		{
			cmd_time = _fft_control.get_command_active(n->node_id);
			if (cmd_time > 0) {
				if ((current_time - cmd_time) > 2000000ULL) {
					PX4_WARN("Timeout on FFT Control command, current_time: %lld, cmd_time: %lld", current_time, cmd_time);
					_fft_control.reset_command_active(n->node_id);
				}
			}
			cmd_time = _fft_param.get_command_active(n->node_id);
			if (cmd_time > 0) {
				if ((current_time - cmd_time) > 2000000ULL) {
					PX4_WARN("Timeout on FFT Param request, current_time: %lld, cmd_time: %lld", current_time, cmd_time);
					_fft_param.reset_command_active(n->node_id);
				}
			}
			cmd_time = _sd_cap_control.get_command_active(n->node_id);
			if (cmd_time > 0) {
				if ((current_time - cmd_time) > 2000000ULL) {
					PX4_WARN("Timeout on SD Capture request, current_time: %lld, cmd_time: %lld", current_time, cmd_time);
					_sd_cap_control.reset_command_active(n->node_id);
				}
			}
			cmd_time = _gnss_control.get_command_active(n->node_id);
			if (cmd_time > 0) {
				if ((current_time - cmd_time) > 2000000ULL) {
					PX4_WARN("Timeout on GNSS Control request, current_time: %lld, cmd_time: %lld", current_time, cmd_time);
					_gnss_control.reset_command_active(n->node_id);
				}
			}
			cmd_time = _sync_control.get_command_active(n->node_id);
			if (cmd_time > 0) {
				if ((current_time - cmd_time) > 2000000ULL) {
					PX4_WARN("Timeout on Sync Control request, current_time: %lld, cmd_time: %lld", current_time, cmd_time);
					_sync_control.reset_command_active(n->node_id);
				}
			}
		}
		orb_publish( ORB_ID(uavcan_parameter_value), this->heartbeat_pub, &this->heartbeat);	///< uORB pub for AVS events
	}

	void HandleAresResponse(const CanardRxTransfer &receive);

	void response_callback(const CanardRxTransfer &receive) override
	{
		HandleAresResponse(receive);
	}

	void update()
	{
		// Update various uORB subscriptions
		_fft_param.update();
		_fft_control.update();
		_sd_cap_control.update();
		_gnss_control.update();
		_sync_control.update();
	}

private:
	CanardHandle &_canard_handle;
	AresFftParamServiceRequest _fft_param;
	AresFftControlServiceRequest _fft_control;
	AresSdCapControlServiceRequest _sd_cap_control;
	AresGnssControlServiceRequest _gnss_control;
	AresSyncControlServiceRequest _sync_control;
	List <AresNode *> _nodes;
	struct uavcan_parameter_value_s heartbeat;
	orb_advert_t heartbeat_pub;
	uORB::Publication<sensor_avs_cmd_ack_s> sensor_avs_cmd_ack_pub{ORB_ID(sensor_avs_cmd_ack)};
};
