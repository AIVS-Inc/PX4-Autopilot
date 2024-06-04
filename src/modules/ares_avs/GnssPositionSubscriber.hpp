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
		ares_GnssPos_0_1 gnsspos {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_GnssPos_0_1_deserialize_(&gnsspos, (const uint8_t *)receive.payload, &msg_size_in_bits);

		if (node[active_node] == 0) {
			node[active_node] = receive.metadata.remote_node_id;
			msg_cnt = 1;
		}
		else if (node[active_node] == receive.metadata.remote_node_id) {
			msg_cnt = 0;	// this node is still alive, so reset msg_cnt
		}
		else {	// message received from the other non-active node
			msg_cnt++;		// increment count of how many missed
			if (msg_cnt > 10) {
				active_node = (active_node == 0) ? 1 : 0;
				node[active_node] = receive.metadata.remote_node_id;
				msg_cnt = 1;
			}
			PX4_INFO("switch active [%hu] GNSS to node %lu", active_node, node[active_node]);
		}

		double lat = gnsspos.m_dLatitude;
		double lon = gnsspos.m_dLongitude;
		float alt = gnsspos.m_fAltitudeEllipsoid;
		float deltah = gnsspos.m_fHorizontalAccuracy;
		float deltav = gnsspos.m_fVerticalAccuracy;

		report.timestamp = hrt_absolute_time();
		report.time_utc_usec = gnsspos.m_u64utcUsec;
		report.device_id = node[active_node];
		report.fix_type = gnsspos.m_u8FixType;
		report.latitude_deg = lat;	// average the positions from both antennas
		report.longitude_deg = lon;
		report.altitude_msl_m = alt;	// ARES seems to duplicate MSL and ALT
		report.altitude_ellipsoid_m = alt;	// FIX: are both msl and alt in PVT?
		report.eph = deltah;
		report.epv = deltav;
		report.hdop = (float) gnsspos.m_u8PDOP / 10.0f;		// uBlox PVT message only has PDOP
		report.vdop = report.hdop * deltav/deltah;		// FIX
		report.cog_rad = (float) NAN;					// heading shall come from the RTK system
		report.vel_m_s = gnsspos.m_fGroundSpeed;
		report.vel_n_m_s = gnsspos.m_fNorthVel;
		report.vel_e_m_s = gnsspos.m_fEastVel;
		report.vel_d_m_s = gnsspos.m_fDownVel;
		report.satellites_used = gnsspos.m_u8SIV;
		report.vel_ned_valid = true;

		//PX4_INFO("node:%lu, lat: %.9f, lon: %.9f, alt: %.2f m, (+- h: %.2f m, v: %.2f m)",
		//	  node[active_node], report.latitude_deg, report.longitude_deg, (double)report.altitude_msl_m, (double)report.eph, (double)report.epv);

		if (report.time_utc_usec > 0)
		{
			orb_publish( ORB_ID(sensor_gps), this->gps_pub, &this->report);	///< uORB pub for gps position

			if ((report.timestamp/1000000 > 180) && (report.fix_type >= 3)) {
				// get current system time to check if it is valid (after 3 minute uptime)
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
		}
	};
private:
	CanardPortID _portID;
	uint32_t node[2] = {0};		// two reporting nodes max
	uint8_t msg_cnt = 0;		// allows redundant switchover in case one node GNSS fails to arrive
	uint8_t active_node = 0;	// which of up to two nodes is active
 };
