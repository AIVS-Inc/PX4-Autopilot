/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef SENSOR_AVS_HPP
#define SENSOR_AVS_HPP

#include <uORB/topics/sensor_avs.h>  //placed in: build/nxp_fmuk66-v3_default/uORB/topics

class MavlinkStreamSensorAvs : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorAvs(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_AVS";  }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SENSOR_AVS; }


	const char *get_name() const override { return MavlinkStreamSensorAvs::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }


	unsigned get_size() override
	{
	return MAVLINK_MSG_ID_SENSOR_AVS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
private:
	explicit MavlinkStreamSensorAvs(Mavlink *mavlink) : MavlinkStream(mavlink) {}
    	uORB::Subscription _sensor_avs_sub{ORB_ID(sensor_avs)};

	bool send() override
    	{
	sensor_avs_s sensor_avs_data;

	if (_sensor_avs_sub.update(&sensor_avs_data)) {
		mavlink_sensor_avs_t msg{};

		msg.timestamp= sensor_avs_data.timestamp;
		msg.time_utc_usec= sensor_avs_data.time_utc_usec;
		msg.timestamp_sample= sensor_avs_data.timestamp_sample;
		msg.device_id= sensor_avs_data.device_id;
		msg.azimuth_deg= sensor_avs_data.azimuth_deg;
		msg.elevation_deg= sensor_avs_data.elevation_deg;
		msg.active_intensity= sensor_avs_data.active_intensity;
		msg.q_factor= sensor_avs_data.q_factor;
		msg.source_index= sensor_avs_data.source_index;
		msg.histogram_count= sensor_avs_data.histogram_count;
		//msg.mel_intensity= sensor_avs_data.mel_intensity;

		for (unsigned i = 0; i < 16; ++i) {
			msg.mel_intensity[i] = sensor_avs_data.mel_intensity[i];
		}

        	mavlink_msg_sensor_avs_send_struct(_mavlink->get_channel(),&msg);
			return true;
		}
	return false;

	}
};
#endif // SENSOR_AVS_HPP
