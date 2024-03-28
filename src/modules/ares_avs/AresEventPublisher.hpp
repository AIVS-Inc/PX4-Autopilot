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

#pragma once

#include "../../drivers/cyphal/Publishers/BasePublisher.hpp"
#include "ares/EscRpm_0_1.h"
#include "ares/FFTcontrol_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>

/*
 * This module publishes measured RPM and other information to Cyphal-CAN  for
 * use by ARES nodes in acoustic noise rejection.
 */
class AresEventPublisher : public BasePublisher
{
public:
	AresEventPublisher(CanardHandle &handle, uint8_t instance = 0) :
		BasePublisher(handle, "ares", "esc", instance)
	{
		int32_t val;
		param_get(param_find("AVS_RPM_AVG_MSEC"), &val); desired_rpm_avg_msec = (uint32_t) val;
	};

	~AresEventPublisher() override = default;

	// Update the uORB Subscription and broadcast a cyphal message
	void update() override
	{
		if (_esc_sub.updated()) {

			esc_status_s esc_sta {};
			_esc_sub.update(&esc_sta);
			if (esc_sta.esc_count > 4)	// TODO: should handle up to 8
				esc_sta.esc_count = 4;

			for (int i=0; i<esc_sta.esc_count; i++ ) {
				if (esc_sta.esc[i].esc_rpm > 0) {
					rpm[i] += esc_sta.esc[i].esc_rpm;
					rpm_squared[i] += esc_sta.esc[i].esc_rpm * esc_sta.esc[i].esc_rpm;
					rpm_avg_cnt[i]++;
				}
			}
			uint32_t deltaT = (uint32_t)(esc_sta.timestamp/1000 - last_update_msec);
			if ( deltaT >= desired_rpm_avg_msec) {

				int32_t result;
				size_t cmd_payload_size = ares_EscRpm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_EscRpm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

				ares_EscRpm_0_1 cmd {};
				cmd.m_period = (uint16_t) deltaT;
				cmd.m_timeUtcUsec = esc_sta.timestamp;
				uint16_t mask = 0x0000;
				bool skip_msg = false;
				float meanRpm[8];
				float meanSqRpm[8];

				for (int i=0; i<esc_sta.esc_count; i++) {

					meanRpm[i] = rpm[i] / rpm_avg_cnt[i];
					if (((meanRpm[i] > 60) && (meanRpm[i] < 60000)) == false) {
						skip_msg = true;
					}
					meanSqRpm[i] = rpm_squared[i] / rpm_avg_cnt[i];
					rpm[i] = 0;
					rpm_squared[i] = 0;
				}
				if (skip_msg == false) {
					for (int i=0; i<esc_sta.esc_count; i++) {
						cmd.m_rps[i%4] = meanRpm[i] / (float)60;
						cmd.m_var[i%4] = (meanSqRpm[i] - meanRpm[i] * meanRpm[i]) / (60*60);
						mask = mask | (1 << i);

						if (i==3 || i==esc_sta.esc_count-1) {	// send one or two RPM messages, depending on motor count

							cmd.m_motorMask = mask;
							const CanardTransferMetadata cmd_transfer_metadata = {
								.priority       = CanardPriorityNominal,
								.transfer_kind  = CanardTransferKindMessage,
								.port_id        = ARES_SUBJECT_ID_FFT_RPM,
								.remote_node_id = CANARD_NODE_ID_UNSET,
								.transfer_id    = _transfer_id,
							};
							result = ares_EscRpm_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

							if (result == 0) {
								++_transfer_id;
								result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
											&cmd_transfer_metadata,
											cmd_payload_size,
											&cmd_payload_buffer);
							}
							mask = 0x0000;
						}
						// PX4_INFO("time: %llu, delta: %hu, esc: %d, cnt: %lu, rps: %0.0f, var: %0.0f", cmd.m_timeUtcUsec, cmd.m_period, i, rpm_avg_cnt[i], (double)cmd.m_rps[i%4], (double)cmd.m_var[i%4]);
					}
				}
				for (int i=0; i<esc_sta.esc_count; i++ ) {
					rpm_avg_cnt[i] = 0;
				}
				last_update_msec = esc_sta.timestamp/1000;
			}
		}
	};

private:
	uORB::Subscription _esc_sub{ORB_ID(esc_status)};
	CanardTransferID _transfer_id {0};
	float rpm[8];			// RPM sum used for average accumulation
	float rpm_squared[8];		// RPM squared sum
	uint64_t last_update_msec = 0;
	uint32_t rpm_avg_cnt[8] = {0};
	uint32_t desired_rpm_avg_msec;
};
