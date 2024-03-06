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

#include "ares/SDcontrol_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_sd_control.h>

class AresSdCapControlServiceRequest : public BasePublisher
{
public:
	AresSdCapControlServiceRequest(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		BasePublisher(handle, "ares", "sdcontrol", instance), _portID(portID)  { };

	~AresSdCapControlServiceRequest() override = default;

	// Update the uORB Subscription and send a UAVCAN service request
	void update()
	{
		int32_t result;

		if (_sd_sub.updated()) {

			PX4_INFO("ARES SD control update");
			sensor_avs_sd_control_s sdctl {};
			_sd_sub.update(&sdctl);

			size_t sd_payload_size = ares_SDcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t sd_payload_buffer[ares_SDcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_SDcontrol_0_1 sd {};

			sd.m_u16ControlId = StorageControlId_FileStart;
			sd.m_u16Length = 1;
			sd.m_u8Command[0] = sdctl.sd_capture;

			if (sdctl.node_top > 0) {
				const CanardTransferMetadata sd_transfer_metadata_0 = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = sdctl.node_top,
					.transfer_id    = _transfer_id_sd_cap_top,
				};
				result = ares_SDcontrol_0_1_serialize_(&sd, sd_payload_buffer, &sd_payload_size);

				if (result == 0) {
					++_transfer_id_sd_cap_top;
					result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&sd_transfer_metadata_0,
								sd_payload_size,
								&sd_payload_buffer);	// no response handler
				}
			}
			if (sdctl.node_bot > 0) {
				const CanardTransferMetadata sd_transfer_metadata_0 = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = sdctl.node_bot,
					.transfer_id    = _transfer_id_sd_cap_bot,
				};
				result = ares_SDcontrol_0_1_serialize_(&sd, sd_payload_buffer, &sd_payload_size);

				if (result == 0) {
					++_transfer_id_sd_cap_bot;
					result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&sd_transfer_metadata_0,
								sd_payload_size,
								&sd_payload_buffer);	// no response handler
				}
			}
		}
	}

protected:
	uORB::Subscription _sd_sub{ORB_ID(sensor_avs_sd_control)};
	CanardTransferID _transfer_id_sd_cap_top {0};
	CanardTransferID _transfer_id_sd_cap_bot {0};
	CanardPortID _portID;

	//UavcanServiceRequestInterface *_response_callback = nullptr;

};
