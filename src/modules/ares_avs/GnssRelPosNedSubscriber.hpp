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
 * @file GnssRelPosNedSubscriber.hpp
 *
 * Defines functionality of ARES Cyphal GNSS-IMU message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gnss_relative.h>
#include "ares/GnssRelPosNed_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class GnssRelPosNedSubscriber : public UavcanBaseSubscriber
{
	struct sensor_gnss_relative_s report;
	orb_advert_t gnss_relative_pub;
public:
	GnssRelPosNedSubscriber(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "gnssrelposned", instance), _portID(portID) { };

	void subscribe() override
	{
		// Subscribe to messages
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _portID,
					   ares_GnssRelPosNed_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
		/* advertise gnss topic */
		memset(&this->report, 0, sizeof(this->report));
		this->gnss_relative_pub = orb_advertise(ORB_ID(sensor_gnss_relative), &this->report);
		PX4_INFO("subscribed to GnssRelPosNed, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("GnssRelPosNedCallback");

		ares_GnssRelPosNed_0_1 relposned {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_GnssRelPosNed_0_1_deserialize_(&relposned, (const uint8_t *)receive.payload, &msg_size_in_bits);

		float relPosN = relposned.m_relPosN;
		float relPosE = relposned.m_relPosE;
		float relPosD = relposned.m_relPosD;
		float relPosHeading = relposned.m_relPosHeading;
		float relPosLength = relposned.m_relPosLength;
		float accN = relposned.m_accN;
		float accE = relposned.m_accE;
		float accD = relposned.m_accD;
		float accH = relposned.m_accH;
		float accL = relposned.m_accL;
		uint8_t relPosQuality = relposned.m_relPosQuality;
		uint32_t node = receive.metadata.remote_node_id;

		PX4_INFO("node:%lu, fix: %s, hd: %.2f (+- %.2f) deg, len: %.2f (+- %.2f) m, N: %.3f, E: %.3f, D: %.3f m, (+- n: %.3f, e: %.3f, d: %.3f m)\r\n", node,
					( GnssCarrierPhaseStatus_Floating == (relPosQuality & 0x03) ) ? "float" :
					( GnssCarrierPhaseStatus_Fixed == (relPosQuality & 0x03) ) ? "fixed" : "none",
					(double)relPosHeading, (double)accH, (double)relPosLength, (double)accL, (double)relPosN,
					(double)relPosE, (double)relPosD, (double)accN, (double)accE, (double)accD);

		report.timestamp = hrt_absolute_time();
		report.device_id = node;
		report.position[0] = relPosN;
		report.position[1] = relPosE;
		report.position[2] = relPosD;
		report.position_accuracy[0] = accN;
		report.position_accuracy[1] = accE;
		report.position_accuracy[2] = accD;
		report.heading = relPosHeading;
		report.heading_accuracy = accH;
		report.position_length = relPosLength;
		report.accuracy_length = accL;
		report.moving_base_mode = true;
		report.carrier_solution_floating = (relPosQuality & 0x01) ? true: false;
		report.carrier_solution_fixed = (relPosQuality & 0x02) ? true: false;
		report.heading_valid = (relPosQuality & 0x03) ? true: false;
		report.gnss_fix_ok = (relPosQuality & 0x02) ? true: false;
		report.relative_position_valid = report.gnss_fix_ok;
		report.differential_solution = report.gnss_fix_ok;

		orb_publish( ORB_ID(sensor_gnss_relative), this->gnss_relative_pub, &this->report);	///< uORB pub for gps position
	};
private:
	CanardPortID _portID;
};
