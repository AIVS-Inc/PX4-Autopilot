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

#ifndef AVS_MFC_HPP
#define AVS_MFC_HPP

#include <uORB/topics/sensor_avs_mel.h>

class MavlinkStreamAvsMfc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAvsMfc(mavlink); }

	static constexpr const char *get_name_static() { return "AVS_MFC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_AVS_MFC; }

	const char *get_name() const override { return MavlinkStreamAvsMfc::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_avs_mfc_sub.advertised() ? MAVLINK_MSG_ID_AVS_MFC + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAvsMfc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_avs_mfc_sub{ORB_ID(sensor_avs)};

	bool send() override
	{
		sensor_avs_mel_s sensor_avs_mel;

		if (_sensor_avs_mfc_sub.update(&sensor_avs_mel)) {
			mavlink_avs_mfc_t msg{};

			msg.time_boot_ms = sensor_avs_mel.timestamp / 1000;
			msg.time_usec = sensor_avs_mel.time_utc_usec;
			msg.sample_index = sensor_avs_mel.timestamp_sample;
			msg.node_id = sensor_avs_mel.device_id;

			for (unsigned int i = 0; i < sensor_avs_mel.FFT_MEL_BANDS; i++) {
				msg.intensity[i] = sensor_avs_mel.active_intensity[i];
			}
			mavlink_msg_avs_mfc_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // AVS_MFC_HPP
