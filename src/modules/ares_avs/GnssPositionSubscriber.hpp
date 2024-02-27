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
 * @file GnssPositionSubscriber.hpp
 *
 * Defines functionality of ARES Cyphal GNSS-IMU message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>
#include "ares/GnssPos_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class GnssPositionSubscriber : public UavcanBaseSubscriber
{
	struct sensor_gps_s report;
	orb_advert_t gps_pub;
public:
	GnssPositionSubscriber(CanardHandle &handle, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "gnsspos", instance) { };

	void subscribe() override
	{
		// Subscribe to messages
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   ARES_SUBJECT_ID_GNSS_POSITION,
					   ares_GnssPos_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
		/* advertise gnss topic */
		memset(&this->report, 0, sizeof(this->report));
		this->gps_pub = orb_advertise(ORB_ID(sensor_gps), &this->report);
		PX4_INFO("subscribed to GnssPosition");
	};

	void callback(const CanardRxTransfer &receive) override
	{
		PX4_INFO("GnssPositionCallback");

		ares_GnssPos_0_1 gnsspos {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_GnssPos_0_1_deserialize_(&gnsspos, (const uint8_t *)receive.payload, &msg_size_in_bits);

		//uint64_t utc_us = gnsspos.m_u32JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		uint32_t tow = gnsspos.m_u32Itow;
		double lat = gnsspos.m_dLatitude;
		double lon = gnsspos.m_dLongitude;
		double alt = gnsspos.m_dAltitudeMsl;
		uavcan_si_unit_length_Scalar_1_0 deltah = gnsspos.m_fHorizontalAccuracy;
		uavcan_si_unit_length_Scalar_1_0 deltav = gnsspos.m_fVerticalAccuracy;
		uint32_t node = receive.metadata.remote_node_id;

		PX4_INFO("node:%lu,itow: %lu, lat: %.9f, lon: %.9f, alt: %.6f m, (+- h: %.4f m, v: %.4f m)",
			  node,tow, lat, lon, alt, (double)deltah.meter, (double)deltav.meter);

		report.device_id = node;
		report.latitude_deg = lat;
		report.latitude_deg = lon;
		report.altitude_ellipsoid_m = alt;
		report.eph = deltah.meter;
		report.epv = deltav.meter;
		orb_publish( ORB_ID(sensor_gps), this->gps_pub, &this->report);	///< uORB pub for gps position
	};
};
