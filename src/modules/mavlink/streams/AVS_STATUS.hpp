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
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uint32_t lastMsec = 0;	// remember the last timestamp
	uint8_t bg_lin = 2;	// number of linear average outputs before which which the exponential average will be seeded
	float acti_sum = 0.0f;	// the last averager output
	float bg_threshold = 0.0f;
	float bg_db_threshold = 0.0f;
	float initial_bg_db_threshold = 0.0f;
	float fSmoothBG = 0.0f;
	bool is_armed = false;
	int32_t trigger_hold_cnt;
	uint16_t average_cnt;

	bool send() override
	{
		bool sendMsg = false;

		sensor_avs_s sensor_avs_status;
		int32_t sync_time;
		int32_t bg_time_constant;
		int32_t threshold;
		int32_t bg_lvl_mode;
		int32_t bg_rel_dB;
		float bg_db_threshold_fixed;
		float using_this_threshold = 0;
		float rel_dB;
		float acti;
		bool arm_status_change;

		vehicle_status_s vehicle_status{};
		arm_status_change = _vehicle_status_sub.update(&vehicle_status);

		if (arm_status_change) {
    			is_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			if (!is_armed) {
				if (lastMsec != 0) {
					lastMsec = 0;
					PX4_INFO("Disarmed, not sending AVS_STATUS");
				}
				return sendMsg;
			}
		}
		if (is_armed) {
			if (lastMsec == 0) {
				PX4_INFO("Armed, sending AVS_STATUS after BG noise measurement");
			}
			// We are armed and should send the AVS packet
			param_get(param_find("AVS_EVT_REL_DB"), &bg_rel_dB);
			param_get(param_find("AVS_EVT_BGSIL"), &bg_lvl_mode);	// average mode (0=fixed threshold, 1=meas BG continuously, 2=meas BG once)
			param_get(param_find("AVS_EVT_BGSIL_DB"), &threshold);
			bg_db_threshold_fixed = (float)threshold;
			rel_dB = (float)bg_rel_dB;

			if (bg_lvl_mode == 1) {
				param_get(param_find("AVS_EVT_BG_TC"), &bg_time_constant);
				fSmoothBG = 1 - expf( -0.05f / (float)bg_time_constant);	// assumes event window = 50 msec
			}
			float IntDbRef= 1e-12f;

			if (_sensor_avs_sub.update(&sensor_avs_status)) {
				mavlink_avs_status_t msg{};

				// pause exponential averaging if there has been a large gap since the last packet
				if (sensor_avs_status.timestamp / 1000 - lastMsec <= 250) {
					lastMsec = sensor_avs_status.timestamp / 1000;

					// Always measure the background noise so that it can be reported in the data packet
					acti = IntDbRef*powf(10.f, sensor_avs_status.active_intensity / 10.f);
					bg_threshold = bg_threshold + fSmoothBG * (acti - bg_threshold);
					bg_db_threshold = 10.0f * log10f(bg_threshold / IntDbRef);

					if (bg_lin == 0) {
						// Limit mavlink Tx by sending pkts only above a specified dB threshold
						switch (bg_lvl_mode) {
							case 0:
								using_this_threshold = bg_db_threshold_fixed;
								break;
							case 1:
								using_this_threshold = bg_db_threshold;
								break;
							case 2:
								using_this_threshold = initial_bg_db_threshold;
								break;
						}
						if (sensor_avs_status.active_intensity > using_this_threshold + rel_dB) {
							sendMsg = true;
							param_get(param_find("AVS_EVT_TRG_HOLD"), &trigger_hold_cnt);
						}
						else if (--trigger_hold_cnt > 0) {
							sendMsg = true;
						}
						if (sendMsg) {
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
							msg.bgint = (uint8_t) (bg_db_threshold * 2);				// uint8 [0 128 dB in 0.5 dB steps]
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
						}
					}
					else {
						// do a 3-second linear average which will serve as the starting point of the exponential average
						acti_sum = acti_sum + IntDbRef*powf(10.f, sensor_avs_status.active_intensity / 10.f);
						if (++average_cnt == 60) {
							bg_lin--;
							if (bg_lin > 0) {
								// Do the linear average again
								acti_sum = 0.0f;
								average_cnt = 0;
							}
							else {
								// Compute the new (or starting) background threshold in dB
								bg_threshold = acti_sum / 60;
								bg_db_threshold = 10.0f * log10f(acti_sum / IntDbRef);
								initial_bg_db_threshold = bg_db_threshold;
								PX4_INFO("Initial Background SIL = %.1f dB", (double)initial_bg_db_threshold);
								trigger_hold_cnt = 0;
							}
						}
					}
				}
				else {
					bg_lin = 2;	// need to do two short linear averages. The first captures and discards the startup FFT glitch upon enable
							// the second seeds the exponential average
					acti_sum= 0.0f;
					average_cnt = 0;
					PX4_INFO("last msec, this msec = %ld, %lld", lastMsec, sensor_avs_status.timestamp / 1000);
					lastMsec = sensor_avs_status.timestamp / 1000;
				}
			}
		}
		return sendMsg;
	}
};

#endif // AVS_STATUS_HPP
