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

// UDRAL Specification Messages
#include "ares/Bearings_0_1.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class AresEventSubscriber : public UavcanBaseSubscriber
{k
public:
	AresEventSubscriber(CanardHandle &handle, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, pmgr, "ares.", "aresevent", instance) { };

	void subscribe() override
	{
		// Subscribe to messages reg.drone.physics.kinematics.geodetic.Point.0.1
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   ares_Bearings_0_1_FIXED_PORT_ID_,
					   ares_Bearings_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		// Test with Yakut:
		// export YAKUT_TRANSPORT="pyuavcan.transport.can.CANTransport(pyuavcan.transport.can.media.slcan.SLCANMedia('/dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_23002B000E514E413431302000000000-if00', 8, 115200), 42)"
		// yakut pub 1500.reg.drone.physics.kinematics.geodetic.Point.0.1 '{latitude: 1.234, longitude: 2.34, altitude: {meter: 0.5}}'
		PX4_INFO("AresEventCallback");

		ares_Bearings_0_1 aresevent {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_AresEvent_0_1_deserialize_(&aresevent, (const uint8_t *)receive.payload, &msg_size_in_bits);

		uint64_t utc_us = aresevent.m_u32JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		reg_udral_physics_kinematics_geodetic_Point_0_1 geo = aresevent.m_glGnssLlh;
		double lat = geo.latitude;
		double lon = geo.longitude;
		double alt = geo.altitude.meter;
		uavcan_si_unit_length_Scalar_1_0 deltah = aresevent.m_fHorizontalAccuracy;
		uavcan_si_unit_length_Scalar_1_0 deltav = aresevent.m_fVerticalAccuracy;
		uavcan_si_unit_angle_Scalar_1_0 pitch = aresevent.m_fPitch;
		uavcan_si_unit_angle_Scalar_1_0 roll = aresevent.m_fRoll;
		uavcan_si_unit_angle_Scalar_1_0 yaw = aresevent.m_fYaw;
		uint32_t sampliIdx = aresevent.m_u32SampleIndex;
		float splDb = aresevent.m_fSplDb;
		float silDb = aresevent.m_fSilDb;
		float activeI = aresevent.m_fActiveI;
		float azim = aresevent.m_fAzimuth;
		float elev = aresevent.m_fElevation;

		PX4_INFO("Latitude: %f, Longitude: %f, Altitude: %f", lat, lon, alt);
		/// do something with the data
	};
};
