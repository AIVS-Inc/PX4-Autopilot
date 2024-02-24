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
#include "ares/EventParams_0_1.h"
#include "ares/FFTcontrol_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_evt_control.h>

class AresEventPublisher : public BasePublisher
{
public:
	AresEventPublisher(CanardHandle &handle, uint8_t instance = 0) :
		BasePublisher(handle, "ares", "eventparams", instance)
	{

	};

	~AresEventPublisher() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	void update() override
	{
		int32_t result;

		if (_evt_sub.updated()) {
			sensor_avs_evt_control_s evt {};
			_evt_sub.update(&evt);

			// disable the FFT before updating event parameters
			size_t fft_payload_size = ares_FFTcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t fft_payload_buffer[ares_FFTcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_FFTcontrol_0_1 fft {};

			fft.m_u16ControlId = FftControlId_Enable;
			fft.m_u16Length = 1;
			fft.m_u8Command[0] = 0;		// 0=disable, 1=enable

			const CanardTransferMetadata fft_transfer_metadata_0 = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = ARES_SUBJECT_ID_FFT_CONTROL,
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id_fft_en,
			};
			result = ares_FFTcontrol_0_1_serialize_(&fft, fft_payload_buffer, &fft_payload_size);

			if (result == 0) {
				++_transfer_id_fft_en;
				result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							       &fft_transfer_metadata_0,
							       fft_payload_size,
							       &fft_payload_buffer);
			}

			size_t cmd_payload_size = ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t cmd_payload_buffer[ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_EventParams_0_1 cmd {};

			cmd.m_paramId = ares_fft_ParamId_EventParam;
			cmd.m_relativeDb = (float) evt.relative_db;
			cmd.m_numSources = (uint32_t) evt.num_sources;
			cmd.m_angularRes = (float) evt.angular_resln;
			cmd.m_bkgndSILtc = (float) evt.bg_timeconst;
			cmd.m_eventWindow = (float) evt.event_window;

			const CanardTransferMetadata cmd_transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = ARES_SUBJECT_ID_FFT_PARAMS,
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
			};
			result = ares_EventParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

			if (result == 0) {
				++_transfer_id;
				result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							       &cmd_transfer_metadata,
							       cmd_payload_size,
							       &cmd_payload_buffer);
			}

			if (evt.fft_enable == 1) {	// enable FFT after setting event parameters
				fft.m_u16ControlId = FftControlId_Enable;
				fft.m_u16Length = 1;
				fft.m_u8Command[0] = 1;	// 0=disable, 1=enable

				const CanardTransferMetadata fft_transfer_metadata_1 = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = ARES_SUBJECT_ID_FFT_CONTROL,
					.remote_node_id = CANARD_NODE_ID_UNSET,
					.transfer_id    = _transfer_id_fft_en,
				};
				result = ares_FFTcontrol_0_1_serialize_(&fft, fft_payload_buffer, &fft_payload_size);

				if (result == 0) {
					++_transfer_id_fft_en;
					result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&fft_transfer_metadata_1,
								fft_payload_size,
								&fft_payload_buffer);
				}
			}
		}
	};

private:

	uORB::Subscription _evt_sub{ORB_ID(sensor_avs_evt_control)};
	CanardTransferID _transfer_id_fft_en {0};
};
