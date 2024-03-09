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
	struct sensor_gps_s gps_report;
	orb_advert_t gps_pub;
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

		/* advertise gnss topic */
		memset(&this->gps_report, 0, sizeof(this->gps_report));
		this->gps_pub = orb_advertise(ORB_ID(sensor_gps), &this->gps_report);
		PX4_INFO("subscribed to BearingAngles, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("AresEventCallback");

		ares_Bearings_0_1 aresevent {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_Bearings_0_1_deserialize_(&aresevent, (const uint8_t *)receive.payload, &msg_size_in_bits);

		uint64_t utc_us = aresevent.m_u32JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		reg_udral_physics_kinematics_geodetic_Point_0_1 geo = aresevent.m_glGnssLlh;
		double lat = geo.latitude;
		double lon = geo.longitude;
		double alt = geo.altitude.meter;
		uavcan_si_unit_length_Scalar_1_0 dh = aresevent.m_fHorizontalAccuracy;
		uavcan_si_unit_length_Scalar_1_0 dv = aresevent.m_fVerticalAccuracy;
		uavcan_si_unit_angle_Scalar_1_0 pitch = aresevent.m_fPitch;
		uavcan_si_unit_angle_Scalar_1_0 roll = aresevent.m_fRoll;
		uavcan_si_unit_angle_Scalar_1_0 yaw = aresevent.m_fYaw;
		double spl = aresevent.m_fSplDb;
		double sil = aresevent.m_fSilDb;
		double acti = aresevent.m_fActiveI;
		double azim = aresevent.m_fAzimuth;
		double elev = aresevent.m_fElevation;
		uint32_t idx = aresevent.m_iSourceIndex;
		uint32_t node = receive.metadata.remote_node_id;

		//PX4_INFO("node:%lu,idx:%lu,usec:%llu,lat:%.9f,lon:%.9f,alt:%.2f,dh:%.2f,dv:%.2f,pitch:%.2f,roll:%.2f,yaw:%.2f,spl:%.2f,sil:%.2f,acti:%.2f,az:%.2f,el:%.2f",
		//  	  node,idx,utc_us,lat,lon,alt,(double)dh.meter,(double)dv.meter,(double)pitch.radian,(double)roll.radian,(double)yaw.radian,spl,sil,acti,azim,elev );

		bearings.timestamp = hrt_absolute_time();
		bearings.device_id = node;
		bearings.time_utc_usec = utc_us;
		bearings.latitude_deg = lat;
		bearings.longitude_deg = lon;
		bearings.altitude_ellipsoid_m = alt;
		bearings.eph = dh.meter;
		bearings.epv = dv.meter;
		bearings.pitch = pitch.radian;
		bearings.roll = roll.radian;
		bearings.yaw = yaw.radian;
		bearings.spl = spl;
		bearings.sil = sil;
		bearings.active_intensity = acti;
		bearings.azimuth_deg = azim;
		bearings.elevation_deg = elev;
		bearings.timestamp_sample = aresevent.m_u32SampleIndex;
		bearings.source_index = idx;

		gps_report.timestamp = bearings.timestamp;
		gps_report.time_utc_usec = utc_us;
		gps_report.latitude_deg = lat;
		gps_report.longitude_deg = lon;
		gps_report.altitude_ellipsoid_m = alt;
		gps_report.eph = dh.meter;
		gps_report.epv = dv.meter;

		orb_publish( ORB_ID(sensor_avs), this->avs_pub, &this->bearings);	///< uORB pub for AVS events
		orb_publish( ORB_ID(sensor_gps), this->gps_pub, &this->gps_report);	///< uORB pub for AVS events
	};
private:
	CanardPortID _portID;
};
