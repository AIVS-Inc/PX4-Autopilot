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

#pragma once

#include "../../drivers/cyphal/Publishers/BasePublisher.hpp"
#include "../../drivers/cyphal/Services/ServiceRequest.hpp"

#include "ares/EventParams_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_evt_control.h>

class AresEventServiceRequest : public BasePublisher
{
public:
	AresEventServiceRequest(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		BasePublisher(handle, "ares", "eventparams", instance), _portID(portID)  { };

	~AresEventServiceRequest() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	void update()
	{
		int32_t result;

		if (_evt_sub.updated()) {

			PX4_INFO("ARES event param update");
			sensor_avs_evt_control_s evt {};
			_evt_sub.update(&evt);

			size_t cmd_payload_size = ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t cmd_payload_buffer[ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_EventParams_0_1 cmd {};

			cmd.m_paramId = ares_fft_ParamId_EventDetector;
			cmd.m_relativeDb = (float) evt.relative_db;
			cmd.m_numSources = (uint16_t) evt.num_sources;
			cmd.m_angularRes = (uint8_t) evt.angular_resln;
			cmd.m_bkgndSILtc = (uint16_t) evt.bg_timeconst;
			cmd.m_eventWindow = (uint8_t) evt.event_window;
			cmd.m_selfMeasureBg = (bool) evt.self_measure_bg;
			cmd.m_bgDbThreshold = (float) evt.bg_db_threshold;

			if (evt.fft_enable == false && evt.node_top > 0) {

				const CanardTransferMetadata cmd_transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = evt.node_top,
					.transfer_id    = _transfer_id_evt_en_top,
				};
				result = ares_EventParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

				if (result == 0) {
					++_transfer_id_evt_en_top;
					result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer);
				}
				PX4_INFO("ARES event param update - complete, node %hd", evt.node_top);
			}
			else if (evt.node_top > 0) {
				PX4_INFO("ARES event param update on node %hd failed since FFT is still enabled", evt.node_top);
			}
			if (evt.fft_enable == false && evt.node_bot > 0) {

				const CanardTransferMetadata cmd_transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = evt.node_bot,
					.transfer_id    = _transfer_id_evt_en_bot,
				};
				result = ares_EventParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

				if (result == 0) {
					++_transfer_id_evt_en_bot;
					result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer);
				}
				PX4_INFO("ARES event param update - complete, node %hd", evt.node_bot);
			}
			else if (evt.node_bot > 0) {
				PX4_INFO("ARES event param update on node %hd failed since FFT is still enabled", evt.node_bot);
			}
		}
	};

private:

	uORB::Subscription _evt_sub{ORB_ID(sensor_avs_evt_control)};
	CanardTransferID _transfer_id_evt_en_top {0};
	CanardTransferID _transfer_id_evt_en_bot {0};
	CanardPortID _portID;

	//UavcanServiceRequestInterface *_response_callback = nullptr;

};
