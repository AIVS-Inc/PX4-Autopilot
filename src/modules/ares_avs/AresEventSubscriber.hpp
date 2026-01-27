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
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs.h>
#include <uORB/topics/sensor_avs_lite.h>
#include <uORB/topics/sensor_avs_lite_ext.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include "ares/Bearings_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"
#include <lib/matrix/matrix/math.hpp>

class AresEventSubscriber : public UavcanBaseSubscriber
{
	struct sensor_avs_s bearings;
	struct sensor_avs_lite_s bearings_lite;
	struct sensor_avs_lite_ext_s bearings_lite_ext;
	orb_advert_t avs_pub;
	orb_advert_t avs_lite_pub;
	orb_advert_t avs_lite_ext_pub;

	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
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

		/* advertise bearings lite topic */
		memset(&this->bearings_lite, 0, sizeof(this->bearings_lite));
		this->avs_lite_pub = orb_advertise(ORB_ID(sensor_avs_lite), &this->bearings_lite);

		/* advertise bearings lite ext topic */
		memset(&this->bearings_lite_ext, 0, sizeof(this->bearings_lite_ext));
		this->avs_lite_ext_pub = orb_advertise(ORB_ID(sensor_avs_lite_ext), &this->bearings_lite_ext);


		PX4_INFO("subscribed to BearingAngles, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("AresEventCallback");

		ares_Bearings_0_1 aresevent {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_Bearings_0_1_deserialize_(&aresevent, (const uint8_t *)receive.payload, &msg_size_in_bits);

		//uint64_t utc_us = aresevent.m_u64JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds

		// Get UTC time from PX4's synchronized clock instead of ARES sensor's Julian time
		// from SYSTEM_TIME.hpp (line 58-63)
		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);
		uint64_t utc_us = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		uint16_t idx = aresevent.m_iSourceIndex;
		uint16_t cnt = aresevent.m_iHistogramCnt;
		double qfac = aresevent.m_fQfac;
		double acti = aresevent.m_fActiveI;
		double azim = aresevent.m_fAzimuth;
		double elev = aresevent.m_fElevation;
		uint32_t node = receive.metadata.remote_node_id;

		// PX4_INFO("node:%lu,idx:%hu,cnt:%hu,usec:%llu,spl:%.2f,sil:%.2f,q-fac:%.2f,acti:%.2f,az:%.2f,el:%.2f",
		//   	  node,idx,cnt,utc_us,spl,sil,qfac,acti,azim,elev );

		bearings.timestamp = hrt_absolute_time();
		bearings.device_id = node;
		bearings.time_utc_usec = utc_us;
		bearings.q_factor = qfac;
		bearings.active_intensity = acti;
		bearings.azimuth_deg = azim;
		bearings.elevation_deg = elev;
		bearings.timestamp_sample = aresevent.m_u32SampleIndex;
		bearings.source_index = idx;
		bearings.histogram_count = cnt;

		for (unsigned int i = 0; i < ares_Bearings_0_1_m_fMelI_ARRAY_CAPACITY_; i++) {
			if (aresevent.m_fMelI[i] > 0.0f)
				bearings.mel_intensity[i] = aresevent.m_fMelI[i];
			else
				bearings.mel_intensity[i] = 0.0f;
		}
	orb_publish( ORB_ID(sensor_avs), this->avs_pub, &this->bearings);	///< uORB pub for AVS events

	// Publish lite version
	bearings_lite.device_id = node;
	bearings_lite.time_utc_usec = utc_us;
	bearings_lite.timestamp = bearings.timestamp;
	bearings_lite.timestamp_sample = bearings.timestamp_sample;
	bearings_lite.azimuth_deg = bearings.azimuth_deg;
	bearings_lite.elevation_deg = bearings.elevation_deg;
	bearings_lite.active_intensity = bearings.active_intensity;
	bearings_lite.q_factor = bearings.q_factor;
	bearings_lite.histogram_count = bearings.histogram_count;
	orb_publish(ORB_ID(sensor_avs_lite), this->avs_lite_pub, &this->bearings_lite);

	// Publish lite extended version
	bearings_lite_ext.device_id = node;
	bearings_lite_ext.time_utc_usec = utc_us;
	bearings_lite_ext.timestamp = bearings.timestamp;
	bearings_lite_ext.timestamp_sample = bearings.timestamp_sample;
	bearings_lite_ext.azimuth_deg = bearings.azimuth_deg;
	bearings_lite_ext.elevation_deg = bearings.elevation_deg;
	bearings_lite_ext.active_intensity = bearings.active_intensity;
	bearings_lite_ext.q_factor = bearings.q_factor;
	bearings_lite_ext.histogram_count = bearings.histogram_count;

	// Vehicle attitude (roll, pitch, yaw from quaternion)
	vehicle_attitude_s att{};
	if (_att_sub.copy(&att)) {
		const matrix::Eulerf euler = matrix::Quatf(att.q);
		bearings_lite_ext.roll = euler.phi();
		bearings_lite_ext.pitch = euler.theta();
		bearings_lite_ext.yaw = euler.psi();
	}

	// Local position NED (north, east, down)
	vehicle_local_position_s lpos{};
	if (_lpos_sub.copy(&lpos)) {
		bearings_lite_ext.north = lpos.x;
		bearings_lite_ext.east = lpos.y;
		bearings_lite_ext.down = lpos.z;
	}

	orb_publish(ORB_ID(sensor_avs_lite_ext), this->avs_lite_ext_pub, &this->bearings_lite_ext);
	};
private:
	CanardPortID _portID;
};
