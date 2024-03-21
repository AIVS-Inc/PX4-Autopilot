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
		param_get(param_find("AVS_RPM_AVG_LEN"), &val); rpm_avg_len = (uint8_t)val;
	};

	~AresEventPublisher() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	void update() override
	{
		int32_t result;

		if (_esc_sub.updated()) {

			PX4_INFO("ARES event param update");
			esc_status_s esc {};
			_esc_sub.update(&esc);

			for (int i; i<esc.esc_count; i++ ) {
				rpm[i] += esc.esc[i].esc_rpm;
				rpm_squared[i] += esc.esc[i].esc_rpm * esc.esc[i].esc_rpm;
			}

			if (++rpm_avg_cnt == rpm_avg_len) {

				size_t cmd_payload_size = ares_EscRpm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_EscRpm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
				ares_EscRpm_0_1 cmd {};

				//get system time
				struct timespec ts = {};
				px4_clock_gettime(CLOCK_REALTIME, &ts);
				uint64_t u64utc = ts.tv_sec + ts.tv_nsec/1000;

				cmd.m_period = (uint16_t) 25000;		// TODO, measurement timeframe of the RPM average
				cmd.m_timeUtcUsec = (uint64_t) u64utc;
				uint8_t mask = 0;

				for (int i=0; i<esc.esc_count; i++) {

					float meanRpm = rpm[i] / rpm_avg_len;
					float meanSqRpm = rpm_squared[i] / rpm_avg_len;
					rpm[i] = 0;
					rpm_squared[i] = 0;

					cmd.m_rps[i%4] = meanRpm / (float)60;
					cmd.m_var[i%4] = meanSqRpm - meanRpm * meanRpm;
					mask = mask | (1 << i);

					if (i==3 || i==esc.esc_count-1) {	// send one or two RPM messages, depending on motor count

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
						mask = 0;
					}
				}
				rpm_avg_cnt = 0;
				//PX4_INFO("ARES RPM update - complete");
			}
		}
	};

private:

	uORB::Subscription _esc_sub{ORB_ID(esc_status)};
	CanardTransferID _transfer_id {0};
	float rpm[8];			// RPM sum used for average accumulation
	float rpm_squared[8];		// RPM squared sum
	uint8_t rpm_avg_cnt {0};	// index of the RPM average component
	uint8_t rpm_avg_len;
};
