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

#ifndef SENSOR_AVS_LITE_EXT_HPP
#define SENSOR_AVS_LITE_EXT_HPP

#include <uORB/topics/sensor_avs_lite_ext.h>  //placed in: build/nxp_fmuk66-v3_default/uORB/topics

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>


class MavlinkStreamSensorAvsLiteExt : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSensorAvsLiteExt(mavlink); }

	static constexpr const char *get_name_static() { return "SENSOR_AVS_LITE_EXT";  }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SENSOR_AVS_LITE_EXT; }

	const char *get_name() const override { return MavlinkStreamSensorAvsLiteExt::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_avs_lite_ext_sub.advertised() ?  MAVLINK_MSG_ID_SENSOR_AVS_LITE_EXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamSensorAvsLiteExt(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_avs_lite_ext_sub{ORB_ID(sensor_avs_lite_ext)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		sensor_avs_lite_ext_s sensor_avs_lite_ext_data;

		if (_sensor_avs_lite_ext_sub.update(&sensor_avs_lite_ext_data)) {
			mavlink_sensor_avs_lite_ext_t msg{};

			// AVS lite sensor data
			msg.device_id = sensor_avs_lite_ext_data.device_id;
			msg.time_utc_usec = sensor_avs_lite_ext_data.time_utc_usec;
			msg.timestamp = sensor_avs_lite_ext_data.timestamp;
			msg.timestamp_sample = sensor_avs_lite_ext_data.timestamp_sample;
			msg.azimuth_deg = sensor_avs_lite_ext_data.azimuth_deg;
			msg.elevation_deg = sensor_avs_lite_ext_data.elevation_deg;
			msg.active_intensity = sensor_avs_lite_ext_data.active_intensity;
			msg.q_factor = sensor_avs_lite_ext_data.q_factor;
			msg.histogram_count = sensor_avs_lite_ext_data.histogram_count;

			// Vehicle attitude (roll, pitch, yaw from quaternion)
			vehicle_attitude_s att{};
			if (_att_sub.copy(&att)) {
				const matrix::Eulerf euler = matrix::Quatf(att.q);
				msg.roll = euler.phi();
				msg.pitch = euler.theta();
				msg.yaw = euler.psi();
			}

			// Local position NED (x, y, z)
			vehicle_local_position_s lpos{};
			if (_lpos_sub.copy(&lpos)) {
				msg.north = lpos.x;
				msg.east = lpos.y;
				msg.down = lpos.z;
			}

			mavlink_msg_sensor_avs_lite_ext_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // SENSOR_AVS_LITE_EXT_HPP
