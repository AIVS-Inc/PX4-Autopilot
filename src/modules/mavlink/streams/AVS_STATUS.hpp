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

#ifndef AVS_STATUS_HPP
#define AVS_STATUS_HPP

#include <uORB/topics/sensor_avs.h>

class MavlinkStreamAvsStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAvsStatus(mavlink); }

	static constexpr const char *get_name_static() { return "AVS_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVS_STATUS; }

	const char *get_name() const override { return MavlinkStreamAvsStatus::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_avs_sub.advertised() ? MAVLINK_MSG_ID_AVS_STATUS + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAvsStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_avs_sub{ORB_ID(sensor_avs)};

	bool send() override
	{
		sensor_avs_s sensor_avs_status;

		if (_sensor_avs_sub.update(&sensor_avs_status)) {
			mavlink_avs_status_t msg{};

			msg.time_usec = sensor_avs_status.time_utc_usec;
			msg.sample_index = sensor_avs_status.timestamp_sample;
			msg.source_index = sensor_avs_status.source_index;
			msg.node_id = sensor_avs_status.device_id;
			msg.spl = sensor_avs_status.spl;
			msg.sil = sensor_avs_status.sil;
			msg.azimuth = sensor_avs_status.azimuth_deg;
			msg.elevation = sensor_avs_status.elevation_deg;
			msg.intensity = sensor_avs_status.active_intensity;
			msg.bg_sil = sensor_avs_status.bkgrnd_sil;

			mavlink_msg_avs_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // AVS_STATUS_HPP
