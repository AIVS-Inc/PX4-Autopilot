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

#include <px4_platform_common/posix.h>
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
	GnssPositionSubscriber(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "gnsspos", instance), _portID(portID) { };

	void subscribe() override
	{
		// Subscribe to messages
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _portID,
					   ares_GnssPos_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
		/* advertise gnss topic */
		memset(&this->report, 0, sizeof(this->report));
		this->gps_pub = orb_advertise(ORB_ID(sensor_gps), &this->report);
		PX4_INFO("subscribed to GnssPosition, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("GnssPositionCallback");

		ares_GnssPos_0_1 gnsspos {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_GnssPos_0_1_deserialize_(&gnsspos, (const uint8_t *)receive.payload, &msg_size_in_bits);

		double lat = gnsspos.m_dLatitude;
		double lon = gnsspos.m_dLongitude;
		float alt = gnsspos.m_fAltitudeEllipsoid;
		float deltah = gnsspos.m_fHorizontalAccuracy;
		float deltav = gnsspos.m_fVerticalAccuracy;
		uint32_t node = receive.metadata.remote_node_id;

		report.timestamp = hrt_absolute_time();
		report.time_utc_usec = gnsspos.m_u64utcUsec;
		report.device_id = node;
		report.fix_type = gnsspos.m_u8FixType;
		report.latitude_deg = 0.5 * lat + 0.5 * lat_last;	// average the positions from both antennas
		report.longitude_deg = 0.5 * lon + 0.5 * lon_last;
		report.altitude_msl_m = 0.5f * alt + 0.5f * alt_last;	// ARES seems to duplicate MSL and ALT
		report.altitude_ellipsoid_m = 0.5f * alt + 0.5f * alt_last;
		report.eph = 0.5f * deltah + 0.5f * sigx_last;
		report.epv = 0.5f * deltav + 0.5f * sigv_last;
		report.hdop = (float) gnsspos.m_u8PDOP / 10.0f;		// uBlox PVT message only has PDOP
		report.vdop = report.hdop * deltav/deltah;		// FIX
		report.cog_rad = (float) NAN;					// heading shall come from the RTK system
		report.vel_m_s = gnsspos.m_fGroundSpeed;
		report.vel_n_m_s = gnsspos.m_fNorthVel;
		report.vel_e_m_s = gnsspos.m_fEastVel;
		report.vel_d_m_s = gnsspos.m_fDownVel;
		report.satellites_used = gnsspos.m_u8SIV;
		report.vel_ned_valid = true;

		// PX4_INFO("node:%lu, lat: %.9f, lon: %.9f, alt: %.2f m, (+- h: %.2f m, v: %.2f m)",
		// 	  node, report.latitude_deg, report.longitude_deg, (double)report.altitude_msl_m, (double)report.eph, (double)report.epv);

		lat_last = report.latitude_deg;
		lon_last = report.longitude_deg;
		alt_last = report.altitude_ellipsoid_m;
		sigx_last = report.eph;
		sigv_last = report.epv;

		orb_publish( ORB_ID(sensor_gps), this->gps_pub, &this->report);	///< uORB pub for gps position

		if ((report.timestamp/1000000 > 120) && (report.fix_type >= 3)) {
			// get current system time to check if it is valid (after 120 sec uptime)
			struct timespec ts = {};
			px4_clock_gettime(CLOCK_REALTIME, &ts);
			time_t time_s = (time_t) (gnsspos.m_u64utcUsec/1000000);
			if (abs(ts.tv_sec - (uint32_t)time_s) > 1) {
				// set system time from GPS time
				ts.tv_sec = time_s;
				ts.tv_nsec = (gnsspos.m_u64utcUsec - (uint64_t)(time_s * 1000000)) * 1000;
				int res = px4_clock_settime(CLOCK_REALTIME, &ts);

				if (res == 0) {
					PX4_INFO("Successfully set system time, %lu: %lu", (uint32_t)ts.tv_sec, ts.tv_nsec);
				} else {
					PX4_ERR("Failed to set system time (%i)", res);
				}
			}
		}
	};
private:
	CanardPortID _portID;
	double lat_last = 47.5894;	// initialized to allow fast position convergence, probably not necessary
	double lon_last = -122.29315;
	float alt_last = 70.7;
	float sigx_last = 1.6;
	float sigv_last = 1.9;
};
