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
 * @file AresEventSubscriber.hpp
 *
 * Defines functionality of ARES Cyphal GNSS-IMU message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/sensor_avs.h>
#include "ares/Bearings_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class AresEventSubscriber : public UavcanBaseSubscriber
{
	struct sensor_avs_s bearings;
	orb_advert_t avs_pub;
public:
	AresEventSubscriber(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "bearings", instance), _portID(portID) { };

	void subscribe() override
	{
		// Subscribe to CAN message
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _portID,
					   ares_Bearings_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		/* advertise bearings topic */
		memset(&this->bearings, 0, sizeof(this->bearings));
		this->avs_pub = orb_advertise(ORB_ID(sensor_avs), &this->bearings);
		PX4_INFO("subscribed to BearingAngles, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("AresEventCallback");

		ares_Bearings_0_1 aresevent {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_Bearings_0_1_deserialize_(&aresevent, (const uint8_t *)receive.payload, &msg_size_in_bits);

		uint64_t utc_us = aresevent.m_u32JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		uint32_t idx = aresevent.m_iSourceIndex;
		double spl = aresevent.m_fSplDb;
		double sil = aresevent.m_fSilDb;
		double bgsil = aresevent.m_fSilBgDb;
		double acti = aresevent.m_fActiveI;
		double azim = aresevent.m_fAzimuth;
		double elev = aresevent.m_fElevation;
		uint32_t node = receive.metadata.remote_node_id;

		//PX4_INFO("node:%lu,idx:%lu,usec:%llu,spl:%.2f,sil:%.2f,bgsil:%.2f,acti:%.2f,az:%.2f,el:%.2f",
		//  	  node,idx,utc_us,spl,sil,bgsil,acti,azim,elev );

		bearings.timestamp = hrt_absolute_time();
		bearings.device_id = node;
		bearings.time_utc_usec = utc_us;
		bearings.spl = spl;
		bearings.sil = sil;
		bearings.bkgrnd_sil = bgsil;
		bearings.active_intensity = acti;
		bearings.azimuth_deg = azim;
		bearings.elevation_deg = elev;
		bearings.timestamp_sample = aresevent.m_u32SampleIndex;
		bearings.source_index = idx;

		orb_publish( ORB_ID(sensor_avs), this->avs_pub, &this->bearings);	///< uORB pub for AVS events
	};
private:
	CanardPortID _portID;
};
