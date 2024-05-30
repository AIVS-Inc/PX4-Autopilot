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
#include "ares/Rtcm_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gps_inject_data.h>

/*
 * This module acts as a bridge for rtcm correction data from a base station, or NTRIP source,
 * to Cyphal-CAN for use by ARES nodes designated as moving base stations.
 */
class GnssRtcmPublisher : public BasePublisher
{
public:
	GnssRtcmPublisher(CanardHandle &handle, uint8_t instance = 0) :
		BasePublisher(handle, "ares", "rtcm", instance)
	{
	};

	~GnssRtcmPublisher() override = default;

	// Update the uORB Subscription and broadcast a cyphal message
	void update() override
	{
		if (_rtcm_sub.updated()) {

			gps_inject_data_s rtcm {};
			_rtcm_sub.update(&rtcm);

			size_t dest_payload_size = ares_Rtcm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t dest_payload_buffer[ares_Rtcm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			ares_Rtcm_0_1 dest {};
			memcpy(dest.m_u8Data.elements,rtcm.data,rtcm.len);
			dest.m_u8Data.count = rtcm.len;

			const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = ARES_SUBJECT_ID_GNSS_NTRIP,
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
			};
			int32_t result = ares_Rtcm_0_1_serialize_(&dest, dest_payload_buffer, &dest_payload_size);
			//PX4_INFO("copy %d RTCM bytes to CAN, result %ld", rtcm.len, result);

			if (result == 0) {
				++_transfer_id;
				result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&transfer_metadata,
							dest_payload_size,
							&dest_payload_buffer);
			}
		}
	};

private:
	uORB::Subscription _rtcm_sub{ORB_ID(gps_inject_data)};
	CanardTransferID _transfer_id {0};
};
