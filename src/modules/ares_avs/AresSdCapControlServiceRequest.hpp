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

#include "AresPublisher.hpp"
#include "../../drivers/cyphal/Services/ServiceRequest.hpp"
#include <px4_platform_common/time.h>
#include "AresServiceRequest.hpp"
#include "ares/SDcontrol_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_sd_control.h>

class AresSdCapControlServiceRequest : public AresServiceRequest
{
public:
	AresSdCapControlServiceRequest(CanardHandle &handle, UavcanServiceRequestInterface *response_handler) :
		AresServiceRequest(handle, "ares.", "sdcontrol", ARES_SUBJECT_ID_STORAGE_CONTROL, ares_SDcontrol_0_1_EXTENT_BYTES_)
		{
			_response_callback = response_handler;
		};

	~AresSdCapControlServiceRequest() override = default;

	// Update the uORB Subscription and send a UAVCAN service request
	void update()
	{
		int32_t result;

		if (_sd_sub.updated()) {

			PX4_INFO("ARES SD control update");
			sensor_avs_sd_control_s sdctl {};
			_sd_sub.update(&sdctl);
			_node_id_top = sdctl.node_top;
			_node_id_bot = sdctl.node_bot;

			size_t sd_payload_size = ares_SDcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t sd_payload_buffer[ares_SDcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_SDcontrol_0_1 sd {};

			sd.m_u16ControlId = StorageControlId_FileStart;
			sd.m_u16Length = 1;
			sd.m_u8Command[0] = sdctl.sd_capture;

			if (sdctl.node_top > 0) {
				const CanardTransferMetadata sd_transfer_metadata_top = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = sdctl.node_top,
					.transfer_id    = _transfer_id_top,
				};
				result = ares_SDcontrol_0_1_serialize_(&sd, sd_payload_buffer, &sd_payload_size);

				if (result == 0) {
					++_transfer_id_top;
					_active_top = hrt_absolute_time();
					request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&sd_transfer_metadata_top,
							sd_payload_size,
							&sd_payload_buffer,
					       		_response_callback);
				}
				PX4_INFO("set SD capture on node %hd to %hd", sdctl.node_top, sdctl.sd_capture);
			}
			if (sdctl.node_bot > 0) {
				const CanardTransferMetadata sd_transfer_metadata_bot = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = sdctl.node_bot,
					.transfer_id    = _transfer_id_bot,
				};
				result = ares_SDcontrol_0_1_serialize_(&sd, sd_payload_buffer, &sd_payload_size);

				if (result == 0) {
					++_transfer_id_bot;
					_active_bot = hrt_absolute_time();
					request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&sd_transfer_metadata_bot,
							sd_payload_size,
							&sd_payload_buffer,
					       		_response_callback);
				}
				PX4_INFO("set SD capture on node %hd to %hd", sdctl.node_bot, sdctl.sd_capture);
			}
		}
	}

protected:
	uORB::Subscription _sd_sub{ORB_ID(sensor_avs_sd_control)};
};
