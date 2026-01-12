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

#ifndef SENSOR_AVS_LITE_HPP
#define SENSOR_AVS_LITE_HPP

#include <uORB/topics/sensor_avs_lite.h>  //placed in: build/nxp_fmuk66-v3_default/uORB/topics

class MavlinkStreamSensorAvsLite : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorAvsLite(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_AVS_LITE";  }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SENSOR_AVS_LITE; }


	const char *get_name() const override { return MavlinkStreamSensorAvsLite::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }


	unsigned get_size() override
	{
	return MAVLINK_MSG_ID_SENSOR_AVS_LITE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}
private:
	explicit MavlinkStreamSensorAvsLite(Mavlink *mavlink) : MavlinkStream(mavlink) {}
    	uORB::Subscription _sensor_avs_lite_sub{ORB_ID(sensor_avs_lite)};

	bool send() override
    	{
	sensor_avs_lite_s sensor_avs_lite_data;

	if (_sensor_avs_lite_sub.update(&sensor_avs_lite_data)) {
		mavlink_sensor_avs_lite_t msg{};

		msg.time_utc_usec= sensor_avs_lite_data.time_utc_usec;
		msg.timestamp = sensor_avs_lite_data.timestamp;
		msg.timestamp_sample = sensor_avs_lite_data.timestamp_sample;
		msg.azimuth_deg= sensor_avs_lite_data.azimuth_deg;
		msg.elevation_deg= sensor_avs_lite_data.elevation_deg;
		msg.active_intensity= sensor_avs_lite_data.active_intensity;
		msg.q_factor= sensor_avs_lite_data.q_factor;
		msg.histogram_count= sensor_avs_lite_data.histogram_count;

        	mavlink_msg_sensor_avs_lite_send_struct(_mavlink->get_channel(),&msg);
			return true;
		}
	return false;

	}
};
#endif // SENSOR_AVS_LITE_HPP
