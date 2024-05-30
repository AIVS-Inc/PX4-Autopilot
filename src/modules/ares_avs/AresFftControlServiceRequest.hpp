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
#include "../../drivers/cyphal/Services/ServiceRequest.hpp"
#include <px4_platform_common/time.h>
#include "AresServiceRequest.hpp"
#include "ares/FFTcontrol_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_fft_control.h>

class AresFftControlServiceRequest : public AresServiceRequest
{
public:
	AresFftControlServiceRequest(CanardHandle &handle, UavcanServiceRequestInterface *response_handler) :
		AresServiceRequest(handle, "ares", "fftcontrol", ARES_SUBJECT_ID_FFT_CONTROL, ares_FFTcontrol_0_1_EXTENT_BYTES_)
		{
			_response_callback = response_handler;
		};

	~AresFftControlServiceRequest() override = default;

	// Update the uORB Subscription and send a UAVCAN service request
	void update()
	{
		int32_t result;

		if (_fft_sub.updated()) {

			PX4_INFO("ARES fft control update");
			sensor_avs_fft_control_s fftctl {};
			_fft_sub.update(&fftctl);
			_node_id_top = fftctl.node_top;
			_node_id_bot = fftctl.node_bot;

			size_t fft_payload_size = ares_FFTcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t fft_payload_buffer[ares_FFTcontrol_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_FFTcontrol_0_1 fft {};

			fft.m_u16ControlId = fftctl.fft_param_id;

			if (fftctl.fft_param_id == FftControlId_Enable) {
				fft.m_u16Length = 1;
				fft.m_u8Command[0] = fftctl.fft_enable;

				if (fftctl.node_top > 0) {
					const CanardTransferMetadata fft_transfer_metadata_0 = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = fftctl.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTcontrol_0_1_serialize_(&fft, fft_payload_buffer, &fft_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&fft_transfer_metadata_0,
							fft_payload_size,
							&fft_payload_buffer,
					       		_response_callback);
					}
				}
				if (fftctl.node_bot > 0) {
					const CanardTransferMetadata fft_transfer_metadata_0 = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = fftctl.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTcontrol_0_1_serialize_(&fft, fft_payload_buffer, &fft_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&fft_transfer_metadata_0,
							fft_payload_size,
							&fft_payload_buffer,
					       		_response_callback);
					}
				}
			}
			else if (fftctl.fft_param_id == FftControlId_CalibrateFromFile) {

				if (fftctl.node_top > 0) {
					sprintf((char *)fft.m_u8Command,"0:/system/cal/SN11%02d.json", fftctl.node_top);
					fft.m_u16Length = strlen((char *)fft.m_u8Command);

					const CanardTransferMetadata fft_transfer_metadata_0 = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = fftctl.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTcontrol_0_1_serialize_(&fft, fft_payload_buffer, &fft_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&fft_transfer_metadata_0,
							fft_payload_size,
							&fft_payload_buffer,
					       		_response_callback);
					}
				}
				if (fftctl.node_bot > 0) {
					sprintf((char *)fft.m_u8Command,"0:/system/cal/SN11%02d.json", fftctl.node_bot);
					fft.m_u16Length = strlen((char *)fft.m_u8Command);

					const CanardTransferMetadata fft_transfer_metadata_0 = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = fftctl.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTcontrol_0_1_serialize_(&fft, fft_payload_buffer, &fft_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
					       		&fft_transfer_metadata_0,
					       		fft_payload_size,
					       		&fft_payload_buffer,
					       		_response_callback);
					}
				}
			}
		}
	}

private:
	uORB::Subscription _fft_sub{ORB_ID(sensor_avs_fft_control)};
};
