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

#include "AresPublisher.hpp"
#include "../../drivers/cyphal/Services/ServiceRequest.hpp"
#include <px4_platform_common/time.h>
#include "AresServiceRequest.hpp"
#include "ares/EventParams_0_1.h"
#include "ares/PeakParams_0_1.h"
#include "ares/FFTlength_0_1.h"
#include "ares/FFTwindow_0_1.h"
#include "ares/FFTspatial_0_1.h"
#include "ares/FFTlinear_0_1.h"
#include "ares/FFTencrypt_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_avs_evt_control.h>
#include <uORB/topics/sensor_avs_peak_control.h>
#include <uORB/topics/sensor_avs_fft_params.h>

class AresFftParamServiceRequest : public AresServiceRequest
{
public:
	AresFftParamServiceRequest(CanardHandle &handle, UavcanServiceRequestInterface *response_handler) :
		AresServiceRequest(handle, "ares.", "fftparams", ARES_SUBJECT_ID_FFT_PARAMS, ares_EventParams_0_1_EXTENT_BYTES_)
		{
			_response_callback = response_handler;
		};

	~AresFftParamServiceRequest() override = default;

	// Update the uORB Subscription and broadcast a UAVCAN message
	void update()
	{
		int32_t result;

		if (_evt_sub.updated()) {

			PX4_INFO("ARES event param update");
			sensor_avs_evt_control_s evt {};
			_evt_sub.update(&evt);

			size_t cmd_payload_size = ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t cmd_payload_buffer[ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_EventParams_0_1 cmd {};
			_node_id_top = evt.node_top;
			_node_id_bot = evt.node_bot;

			cmd.m_paramId = evt.fft_param_id;
			cmd.m_relativeDb = (float) evt.relative_db;
			cmd.m_numSources = (uint16_t) evt.num_sources;
			cmd.m_angularRes = (uint8_t) evt.angular_resln;
			cmd.m_bkgndSILtc = (uint16_t) evt.bg_timeconst;
			cmd.m_eventWindow = (uint8_t) evt.event_window;
			cmd.m_selfMeasureBg = (bool) evt.self_measure_bg;
			cmd.m_bgDbThreshold = (float) evt.bg_db_threshold;

			if (evt.fft_enable == false && evt.node_top > 0) {

				const CanardTransferMetadata cmd_transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = evt.node_top,
					.transfer_id    = _transfer_id_top,
				};
				result = ares_EventParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

				if (result == 0) {
					++_transfer_id_top;
					_active_top = hrt_absolute_time();
					request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&cmd_transfer_metadata,
							cmd_payload_size,
							&cmd_payload_buffer,
					       		_response_callback);
				}
				PX4_INFO("ARES event param update - complete, node %hd", evt.node_top);
			}
			else if (evt.node_top > 0) {
				PX4_INFO("ARES event param update on node %hd failed since FFT is still enabled", evt.node_top);
			}
			if (evt.fft_enable == false && evt.node_bot > 0) {

				const CanardTransferMetadata cmd_transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = evt.node_bot,
					.transfer_id    = _transfer_id_bot,
				};
				result = ares_EventParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

				if (result == 0) {
					++_transfer_id_bot;
					_active_bot = hrt_absolute_time();
					request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&cmd_transfer_metadata,
							cmd_payload_size,
							&cmd_payload_buffer,
					       		_response_callback);
				}
				PX4_INFO("ARES event param update - complete, node %hd", evt.node_bot);
			}
			else if (evt.node_bot > 0) {
				PX4_INFO("ARES event param update on node %hd failed since FFT is still enabled", evt.node_bot);
			}
		}
		if (_pk_sub.updated()) {

			PX4_INFO("ARES peak param update");
			sensor_avs_peak_control_s pk {};
			_pk_sub.update(&pk);

			size_t cmd_payload_size = ares_PeakParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t cmd_payload_buffer[ares_PeakParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			ares_PeakParams_0_1 cmd {};
			_node_id_top = pk.node_top;
			_node_id_bot = pk.node_bot;


			cmd.m_paramId = pk.fft_param_id;
			cmd.m_iMaxHarmonics = (uint8_t) pk.max_harmonics;
			cmd.m_iNumberOfPeaks = (uint8_t) pk.num_peaks;
			cmd.m_fMinimumPeakHeight = (float) pk.min_peak_height;
			cmd.m_fMinimumPeakChange = (float) pk.min_peak_change;
			cmd.m_fMinimumPeakBlanking = (float) pk.min_blanking;

			if (pk.fft_enable == false && pk.node_top > 0) {

				const CanardTransferMetadata cmd_transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = pk.node_top,
					.transfer_id    = _transfer_id_top,
				};
				result = ares_PeakParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

				if (result == 0) {
					++_transfer_id_top;
					_active_top = hrt_absolute_time();
					request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&cmd_transfer_metadata,
							cmd_payload_size,
							&cmd_payload_buffer,
					       		_response_callback);
				}
				PX4_INFO("ARES peak param update - complete, node %hd", pk.node_top);
			}
			else if (pk.node_top > 0) {
				PX4_INFO("ARES peak param update on node %hd failed since FFT is still enabled", pk.node_top);
			}
			if (pk.fft_enable == false && pk.node_bot > 0) {

				const CanardTransferMetadata cmd_transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindRequest,
					.port_id        = _portID,
					.remote_node_id = pk.node_bot,
					.transfer_id    = _transfer_id_bot,
				};
				result = ares_PeakParams_0_1_serialize_(&cmd, cmd_payload_buffer, &cmd_payload_size);

				if (result == 0) {
					++_transfer_id_bot;
					_active_bot = hrt_absolute_time();
					request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							&cmd_transfer_metadata,
							cmd_payload_size,
							&cmd_payload_buffer,
					       		_response_callback);
				}
				PX4_INFO("ARES peak param update - complete, node %hd", pk.node_bot);
			}
			else if (pk.node_bot > 0) {
				PX4_INFO("ARES peak param update on node %hd failed since FFT is still enabled", pk.node_bot);
			}
		}
		if (_param_sub.updated()) {

			PX4_INFO("ARES FFT param update");
			sensor_avs_fft_params_s params {};
			_param_sub.update(&params);
			_node_id_top = params.node_top;
			_node_id_bot = params.node_bot;

			if (params.fft_param_id == ares_fft_ParamId_OutputDecimator)
			{
				size_t cmd_payload_size = ares_FFTspatial_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_FFTspatial_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
				ares_FFTspatial_0_1 spatial {};
				spatial.m_FftParamId = params.fft_param_id;
				spatial.m_FftDec = params.fft_dec;

				spatial.m_SpatialFilt = params.fft_spatial;
				if (params.node_top > 0) {
					if (params.fft_spatial > 0) {
						spatial.m_SpatialFilt = 1;	// top node
					}
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTspatial_0_1_serialize_(&spatial, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES spatial filter update - complete, node %hd", params.node_top);
				}
				spatial.m_SpatialFilt = params.fft_spatial;
				if (params.node_bot > 0) {
					if (params.fft_spatial > 0) {
						spatial.m_SpatialFilt = 2;	// bottom node
					}
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTspatial_0_1_serialize_(&spatial, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES spatial filter update - complete, node %hd", params.node_bot);
				}
			}
			else if (params.fft_param_id == ares_fft_ParamId_Length)
			{
				size_t cmd_payload_size = ares_FFTlength_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_FFTlength_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
				ares_FFTlength_0_1  length {};
				length.m_FftParamId = params.fft_param_id;
				length.m_FftNumBins = params.fft_max_bins;
				length.m_FftNumBlocks = params.fft_num_blocks;


				if (params.node_top > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTlength_0_1_serialize_(&length, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT length update - complete, node %hd", params.node_top);
				}
				if (params.node_bot > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTlength_0_1_serialize_(&length, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT length update - complete, node %hd", params.node_bot);
				}
			}
			else if (params.fft_param_id == ares_fft_ParamId_Linear)
			{
				size_t cmd_payload_size = ares_FFTlinear_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_FFTlinear_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
				ares_FFTlinear_0_1  linear {};
				linear.m_FftParamId = params.fft_param_id;
				linear.m_FftStartBin = params.fft_start_bin;
				linear.m_FftNumBins = params.fft_num_bins;


				if (params.node_top > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTlinear_0_1_serialize_(&linear, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT linear update - complete, node %hd", params.node_top);
				}
				if (params.node_bot > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTlinear_0_1_serialize_(&linear, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT linear update - complete, node %hd", params.node_bot);
				}
			}
			else if (params.fft_param_id == ares_fft_ParamId_HannWindow)
			{
				size_t cmd_payload_size = ares_FFTwindow_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_FFTwindow_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
				ares_FFTwindow_0_1  window {};
				window.m_FftParamId = params.fft_param_id;
				window.m_FftWinEnable = params.fft_window;

				if (params.node_top > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTwindow_0_1_serialize_(&window, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT window update - complete, node %hd", params.node_top);
				}
				if (params.node_bot > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTwindow_0_1_serialize_(&window, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT window update - complete, node %hd", params.node_bot);
				}
			}
			else if (params.fft_param_id == ares_fft_ParamId_EncryptOutput)
			{
				size_t cmd_payload_size = ares_FFTencrypt_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
				uint8_t cmd_payload_buffer[ares_FFTencrypt_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
				ares_FFTencrypt_0_1 encrypt {};
				encrypt.m_FftParamId = params.fft_param_id;
				encrypt.m_FftEncrypt = params.fft_encrypt;

				if (params.node_top > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_top,
						.transfer_id    = _transfer_id_top,
					};
					result = ares_FFTencrypt_0_1_serialize_(&encrypt, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_top;
						_active_top = hrt_absolute_time();
						request(_active_top + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT encrypt update - complete, node %hd", params.node_top);
				}
				if (params.node_bot > 0) {
					const CanardTransferMetadata cmd_transfer_metadata = {
						.priority       = CanardPriorityNominal,
						.transfer_kind  = CanardTransferKindRequest,
						.port_id        = _portID,
						.remote_node_id = params.node_bot,
						.transfer_id    = _transfer_id_bot,
					};
					result = ares_FFTencrypt_0_1_serialize_(&encrypt, cmd_payload_buffer, &cmd_payload_size);

					if (result == 0) {
						++_transfer_id_bot;
						_active_bot = hrt_absolute_time();
						request(_active_bot + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&cmd_transfer_metadata,
								cmd_payload_size,
								&cmd_payload_buffer,
					       			_response_callback);
					}
					PX4_INFO("ARES FFT encrypt update - complete, node %hd", params.node_bot);
				}
			}
		}
	};

private:
	uORB::Subscription _evt_sub{ORB_ID(sensor_avs_evt_control)};
	uORB::Subscription _pk_sub{ORB_ID(sensor_avs_peak_control)};
	uORB::Subscription _param_sub{ORB_ID(sensor_avs_fft_params)};
};
