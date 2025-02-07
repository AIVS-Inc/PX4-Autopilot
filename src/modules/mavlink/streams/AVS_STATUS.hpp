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
		return _sensor_avs_sub.advertised() ? MAVLINK_MSG_ID_AVS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAvsStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_avs_sub{ORB_ID(sensor_avs)};

	bool send() override
	{
		sensor_avs_s sensor_avs_status;
		int32_t sync_time;

		if (_sensor_avs_sub.update(&sensor_avs_status)) {
			mavlink_avs_status_t msg{};

			// Use time_usec to communicate planned sync time prior to the sync
			// There is a 3-5 second period when the sync time will be transmitted
			param_get(param_find("AVS_TARGET_SYNC"), &sync_time);
			if ((sync_time == 0) || (sensor_avs_status.time_utc_usec/1e6 > sync_time))
			{
				msg.time_usec = sensor_avs_status.time_utc_usec;		// uint64
				msg.adcidx = sensor_avs_status.timestamp_sample;		// uint32
			}
			else
			{
				msg.time_usec = sync_time*1e6;
				msg.adcidx = 0;
			}
			msg.time_boot_ms = sensor_avs_status.timestamp / 1000;			// uint32
			msg.histcnt = (uint16_t) sensor_avs_status.histogram_count;		// uint16
			msg.srcidx = (uint8_t) sensor_avs_status.source_index;			// uint8
			msg.nodeid = (uint8_t)sensor_avs_status.device_id;			// uint8
			msg.azim = sensor_avs_status.azimuth_deg;				// float
			msg.elev = sensor_avs_status.elevation_deg;				// float
			msg.intensity = (uint8_t) (sensor_avs_status.active_intensity * 2);	// uint8 [0 128 dB in 0.5 dB steps]
			msg.qfac = (uint8_t) (sensor_avs_status.q_factor + 0.5f);		// uint8

			for (unsigned int i = 0; i < sensor_avs_status.FFT_MEL_BANDS; i++) {
				if (sensor_avs_status.mel_intensity[i] < 128)
					msg.melint[i] = (uint8_t) (sensor_avs_status.mel_intensity[i] * 2);  // uint8 [0 128 dB in 0.5 dB steps]
				else
					msg.melint[i] = (uint8_t) 128;
			}
			mavlink_msg_avs_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // AVS_STATUS_HPP
