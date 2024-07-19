/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file MelIntensitySubscriber.hpp
 *
 * Defines functionality of ARES Cyphal Mel Intensity message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/sensor_avs_mel.h>
#include "ares/MelIntensity_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class MelIntensitySubscriber : public UavcanBaseSubscriber
{
	struct sensor_avs_mel_s mel;
	orb_advert_t avs_pub;
public:
	MelIntensitySubscriber(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "melintensity", instance), _portID(portID) { };

	void subscribe() override
	{
		// Subscribe to CAN message
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _portID,
					   ares_MelIntensity_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		/* advertise mel topic */
		memset(&this->mel, 0, sizeof(this->mel));
		this->avs_pub = orb_advertise(ORB_ID(sensor_avs_mel), &this->mel);
		PX4_INFO("subscribed to MelIntensity, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("MelIntensityCallback");

		ares_MelIntensity_0_1 mel_intensity {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_MelIntensity_0_1_deserialize_(&mel_intensity, (const uint8_t *)receive.payload, &msg_size_in_bits);

		uint64_t utc_us = mel_intensity.m_u64JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		mel.timestamp = hrt_absolute_time();
		mel.device_id = receive.metadata.remote_node_id;
		mel.time_utc_usec = utc_us;

		for (unsigned int i = 0; i < ares_MelIntensity_0_1_m_f32ActiveI_ARRAY_CAPACITY_; i++) {
			mel.active_intensity[i] = mel_intensity.m_f32ActiveI[i];
		}

		orb_publish( ORB_ID(sensor_avs_mel), this->avs_pub, &this->mel);	///< uORB pub for AVS events
	};
private:
	CanardPortID _portID;
};
