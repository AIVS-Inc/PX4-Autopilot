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
#include "ares/GnssControl_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_gnss_control.h>

class AresGnssControlServiceRequest : public AresServiceRequest
{
public:
	AresGnssControlServiceRequest(CanardHandle &handle, UavcanServiceRequestInterface *response_handler) :
		AresServiceRequest(handle, "ares", "gnsscontrol", ARES_SUBJECT_ID_GNSS_PARAMS, ares_GnssControl_0_1_EXTENT_BYTES_)
		{
			_response_callback = response_handler;
		};

	~AresGnssControlServiceRequest() override = default;

	// Update the uORB Subscription and send a UAVCAN service request
	void update()
	{
		int32_t result;

		if (_gnss_sub.updated()) {

			PX4_INFO("ARES GNSS control update");
			sensor_avs_gnss_control_s gnssctl {};
			_gnss_sub.update(&gnssctl);
			_node_id_top = gnssctl.node_top;
			_node_id_bot = gnssctl.node_bot;

			size_t gnss_payload_size = ares_GnssControl_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t gnss_payload_buffer[ares_GnssControl_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_GnssControl_0_1 gnss {};

			gnss.m_u16ParamId = GnssParamId_RtcmMode;
			uint8_t rtcm_mode_top = GnssRtcmMode_None;
			uint8_t rtcm_mode_bot = GnssRtcmMode_None;

			if ((gnssctl.rtcm_mode == GnssRtcmMode_Rover) || (gnssctl.rtcm_mode == GnssRtcmMode_Base)) {
				rtcm_mode_top = GnssRtcmMode_Rover;
				rtcm_mode_bot = GnssRtcmMode_Base;
			}
			if (gnssctl.node_top > 0) {
				const CanardTransferMetadata gnss_transfer_metadata_top = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = gnssctl.node_top,
					.transfer_id    = _transfer_id_top,
				};
				gnss.m_u8Command[0] = rtcm_mode_top;
				result = ares_GnssControl_0_1_serialize_(&gnss, gnss_payload_buffer, &gnss_payload_size);

				if (result == 0) {
					++_transfer_id_top;
					_active_top = hrt_absolute_time();
					request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&gnss_transfer_metadata_top,
							gnss_payload_size,
							&gnss_payload_buffer,
							_response_callback);
				}
				PX4_INFO("set RTCM node %hd to %hd", gnssctl.node_top, gnss.m_u8Command[0]);
			}
			if (gnssctl.node_bot > 0) {
				const CanardTransferMetadata gnss_transfer_metadata_bot = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = gnssctl.node_bot,
					.transfer_id    = _transfer_id_bot,
				};
				gnss.m_u8Command[0] = rtcm_mode_bot;
				result = ares_GnssControl_0_1_serialize_(&gnss, gnss_payload_buffer, &gnss_payload_size);

				if (result == 0) {
					++_transfer_id_bot;
					_active_bot = hrt_absolute_time();
					request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&gnss_transfer_metadata_bot,
							gnss_payload_size,
							&gnss_payload_buffer,
							_response_callback);
				}
				PX4_INFO("set RTCM node %hd to %hd", gnssctl.node_bot, gnss.m_u8Command[0]);
			}
		}
	}

protected:
	uORB::Subscription _gnss_sub{ORB_ID(sensor_avs_gnss_control)};
};
