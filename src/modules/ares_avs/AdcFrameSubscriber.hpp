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
 * @file AdcFrameSubscriber.hpp
 *
 * Defines functionality of ARES Cyphal ADC Frame message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/sensor_avs_adc.h>
#include "ares/AdcFrame_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class AdcFrameSubscriber : public UavcanBaseSubscriber
{
	struct sensor_avs_adc_s timedata;
	orb_advert_t adc_pub;
	struct sensor_gps_s gps_report;
	orb_advert_t gps_pub;
public:
	AdcFrameSubscriber(CanardHandle &handle, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "adcframe", instance) { };

	void subscribe() override
	{
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   ARES_SUBJECT_ID_FFT_ADC_FRAME,
					   ares_AdcFrame_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		/* advertise timedata topic */
		memset(&this->timedata, 0, sizeof(this->timedata));
		this->adc_pub = orb_advertise(ORB_ID(sensor_avs_adc), &this->timedata);

		/* advertise gnss topic */
		memset(&this->gps_report, 0, sizeof(this->gps_report));
		this->gps_pub = orb_advertise(ORB_ID(sensor_gps), &this->gps_report);
		PX4_INFO("subscribed to AdcFrame");
	};

	void callback(const CanardRxTransfer &receive) override
	{
		PX4_INFO("AdcFrameCallback");

		ares_AdcFrame_0_1 adcframe {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_AdcFrame_0_1_deserialize_(&adcframe, (const uint8_t *)receive.payload, &msg_size_in_bits);

		uint64_t utc_us = adcframe.m_u32JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		reg_udral_physics_kinematics_geodetic_Point_0_1 geo = adcframe.m_glGnssLlh;
		double lat = geo.latitude;
		double lon = geo.longitude;
		double alt = geo.altitude.meter;
		uavcan_si_unit_length_Scalar_1_0 dh = adcframe.m_fHorizontalAccuracy;
		uavcan_si_unit_length_Scalar_1_0 dv = adcframe.m_fVerticalAccuracy;
		uavcan_si_unit_angle_Scalar_1_0 pitch = adcframe.m_fPitch;
		uavcan_si_unit_angle_Scalar_1_0 roll = adcframe.m_fRoll;
		uavcan_si_unit_angle_Scalar_1_0 yaw = adcframe.m_fYaw;
		uint32_t node = receive.metadata.remote_node_id;
		uint32_t tID = receive.metadata.remote_node_id;

		PX4_INFO("node:%lu,id:%lu,usec:%llu,lat:%f,lon:%f,alt:%f,dh:%f,dv:%f,pitch:%f,roll:%f,yaw:%f",
		  	  node,tID,utc_us,lat,lon,alt,(double)dh.meter,(double)dv.meter,(double)pitch.radian,(double)roll.radian,(double)yaw.radian );

		timedata.device_id = node;
		timedata.transfer_id = tID;
		timedata.time_utc_usec = utc_us;
		timedata.latitude_deg = lat;
		timedata.latitude_deg = lon;
		timedata.altitude_ellipsoid_m = alt;
		timedata.eph = dh.meter;
		timedata.epv = dv.meter;
		timedata.pitch = pitch.radian;
		timedata.roll = roll.radian;
		timedata.yaw = yaw.radian;
		timedata.timestamp_sample = adcframe.m_u32SampleIndex;
		memcpy(timedata.adc_frame, adcframe.m_ai32Data, sizeof(adcframe.m_ai32Data));

		gps_report.time_utc_usec = utc_us;
		gps_report.latitude_deg = lat;
		gps_report.latitude_deg = lon;
		gps_report.altitude_ellipsoid_m = alt;
		gps_report.eph = dh.meter;
		gps_report.epv = dv.meter;

		orb_publish( ORB_ID(sensor_avs_adc), this->adc_pub, &this->timedata);	///< uORB pub for AVS events
		orb_publish( ORB_ID(sensor_gps), this->gps_pub, &this->gps_report);	///< uORB pub for AVS events
	};
};
