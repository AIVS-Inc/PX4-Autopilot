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
/**
 * @file AresServiceManager.cpp
 *
 * Manages the ARES service subscriptions
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#include "AresServiceManager.hpp"

void AresServiceManager::HandleAresResponse(const CanardRxTransfer &receive)
{
	// response callback, confirm commands and parameters
	int32_t i32val;
	float fVal;
	bool fft_param_active;

	ares_FFTcontrol_0_1 fft {};
	ares_EventParams_0_1 event {};
	ares_PeakParams_0_1 peak {};
	ares_GnssControl_0_1 gnss {};
	ares_SDcontrol_0_1 sd {};
	ares_SyncControl_0_1 sync {};
	ares_FFTlength_0_1  length {};
	ares_FFTspatial_0_1 spatial {};
	ares_FFTlinear_0_1  linear {};
	ares_FFTwindow_0_1  window {};
	ares_FFTencrypt_0_1  encrypt {};

	size_t size_bytes = receive.payload_size;

	switch (receive.metadata.port_id) {
	  case ARES_SUBJECT_ID_FFT_CONTROL:
	  {
		ares_FFTcontrol_0_1_deserialize_(&fft, (const uint8_t *)receive.payload, &size_bytes);
		bool fft_control_active = (_fft_control.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
		PX4_INFO("FftControl response: node %d, active: %hd", receive.metadata.remote_node_id, fft_control_active);
		if (fft_control_active) {
			_fft_control.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
		}
		break;
	  }
	  case ARES_SUBJECT_ID_FFT_PARAMS:
	  {
		switch (size_bytes) {
		case ares_EventParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_:
			ares_EventParams_0_1_deserialize_(&event, (const uint8_t *)receive.payload, &size_bytes);
			fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
			if (fft_param_active && (ares_fft_ParamId_EventDetector == event.m_paramId)) {
				_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
				param_get(param_find("AVS_EVT_NUM_SRC"), &i32val);
				if ((uint16_t)i32val != event.m_numSources) {PX4_INFO("evt.m_numSources not confirmed, id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_EVT_BG_TC"),   &i32val);
				if ((uint16_t)i32val != event.m_bkgndSILtc) {PX4_INFO("evt.m_bkgndSILtc not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_EVT_REL_DB"),  &fVal);
				if (abs(fVal - event.m_relativeDb) > 1e-2) {PX4_INFO("evt.m_relativeDb not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_EVT_ANG_RES"), &i32val);
				if ((uint8_t)i32val != event.m_angularRes) {PX4_INFO("evt.m_angularRes not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_EVT_EVT_WIN"), &i32val);
				if ((uint8_t)i32val != event.m_eventWindow) {PX4_INFO("evt.m_eventWindow not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_EVT_BGSIL"),   &i32val);
				if ((bool)i32val != event.m_selfMeasureBg) {PX4_INFO("evt.m_selfMeasureBg not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_EVT_BGSIL_DB"),&fVal);
				if (abs(fVal - event.m_bgDbThreshold) > 1e-2) {PX4_INFO("evt.m_bgDbThreshold not confirmed id: %d", receive.metadata.remote_node_id);}
			}
			break;
		case ares_PeakParams_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_:
			ares_PeakParams_0_1_deserialize_(&peak, (const uint8_t *)receive.payload, &size_bytes);
			fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
			if (fft_param_active && (ares_fft_ParamId_PeakDetector == peak.m_paramId)) {
				_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
				param_get(param_find("AVS_PK_NUM_HARM"), &i32val);
				if ((uint8_t)i32val != peak.m_iMaxHarmonics) {PX4_INFO("peak.m_iMaxHarmonics not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_PK_NUM_PEAK"), &i32val);
				if ((uint8_t)i32val != peak.m_iNumberOfPeaks) {PX4_INFO("peak.m_iNumberOfPeaks not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_PK_MIN_HGHT"), &fVal);
				if (abs(fVal - peak.m_fMinimumPeakHeight) > 1e-2) {PX4_INFO("peak.m_fMinimumPeakHeight not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_PK_MIN_D_DB"), &fVal);
				if (abs(fVal - peak.m_fMinimumPeakChange) > 1e-2) {PX4_INFO("peak.m_fMinimumPeakChange not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_PK_MIN_BLNK"), &fVal);
				if (abs(fVal - peak.m_fMinimumPeakBlanking) > 1e-2) {PX4_INFO("not confirmed id: %d", receive.metadata.remote_node_id);}
			}
			break;
		case ares_FFTspatial_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_:
			ares_FFTspatial_0_1_deserialize_(&spatial, (const uint8_t *)receive.payload, &size_bytes);
			fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
			if (fft_param_active && (ares_fft_ParamId_OutputDecimator == spatial.m_FftParamId)) {
				_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
				param_get(param_find("AVS_FFT_DEC"), &i32val);
				if ((uint8_t)i32val != spatial.m_FftDec) {PX4_INFO("spatial.m_FftDec not confirmed id: %d", receive.metadata.remote_node_id);}
				param_get(param_find("AVS_SPATIAL_FILT"), &i32val);
				if ((uint8_t)i32val != spatial.m_SpatialFilt) {PX4_INFO("spatial.m_SpatialFilt not confirmed id: %d", receive.metadata.remote_node_id);}
			}
			break;
		case ares_FFTlength_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_:
			ares_FFTlength_0_1_deserialize_(&length, (const uint8_t *)receive.payload, &size_bytes);

			// length and linear share the same payload length, so check paramId
			if (length.m_FftParamId == ares_fft_ParamId_Linear) {
				ares_FFTlinear_0_1_deserialize_(&linear, (const uint8_t *)receive.payload, &size_bytes);
				fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
				if (fft_param_active && (ares_fft_ParamId_Linear == linear.m_FftParamId)) {
					_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
					param_get(param_find("AVS_FFT_STRT_BIN"), &i32val);
					if ((uint16_t)i32val != linear.m_FftStartBin) {PX4_INFO("linear.m_FftStartBin not confirmed id: %d", receive.metadata.remote_node_id);}
					param_get(param_find("AVS_FFT_NUM_BINS"), &i32val);
					if ((uint16_t)i32val != linear.m_FftNumBins) {PX4_INFO("linear.m_FftNumBins not confirmed id: %d", receive.metadata.remote_node_id);}
				}
			} else {
				fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
				if (fft_param_active && (ares_fft_ParamId_Length == length.m_FftParamId)) {
					_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
					param_get(param_find("AVS_FFT_LONG"), &i32val);
					if (((bool)i32val == true) && (length.m_FftNumBins != 576)) {PX4_INFO("length.m_FftNumBins not confirmed id: %d", receive.metadata.remote_node_id);}
				}
			}
			break;
		case ares_FFTwindow_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_:
			ares_FFTwindow_0_1_deserialize_(&window, (const uint8_t *)receive.payload, &size_bytes);

			// window and encrypt share the same payload length, so check paramId
			if (window.m_FftParamId == ares_fft_ParamId_EncryptOutput) {
				ares_FFTencrypt_0_1_deserialize_(&encrypt, (const uint8_t *)receive.payload, &size_bytes);
				fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
				if (fft_param_active && (ares_fft_ParamId_EncryptOutput == encrypt.m_FftParamId)) {
					_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
					param_get(param_find("AVS_FFT_ENCRYPT"), &i32val);
					if ((uint8_t) i32val != encrypt.m_FftEncrypt) {PX4_INFO("encrypt.m_FftEncrypt not confirmed id: %d", receive.metadata.remote_node_id);}
				}
			} else {
				fft_param_active = (_fft_param.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
				if (fft_param_active && (ares_fft_ParamId_HannWindow == window.m_FftParamId)) {
					_fft_param.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
					param_get(param_find("AVS_FFT_WINDOW"), &i32val);
					if ((uint8_t) i32val != window.m_FftWinEnable) {PX4_INFO("window.m_FftWinEnable not confirmed id: %d", receive.metadata.remote_node_id);}
				}
			}
			break;
		}
		break;
	  }
	  case ARES_SUBJECT_ID_GNSS_PARAMS:
	  {
		ares_GnssControl_0_1_deserialize_(&gnss, (const uint8_t *)receive.payload, &size_bytes);
		bool gnss_control_active = (_gnss_control.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
		PX4_INFO("GnssControl response: node %d, active: %hd", receive.metadata.remote_node_id, gnss_control_active);
		if (gnss_control_active) {
			_gnss_control.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
		}
		break;
	  }
	  case ARES_SUBJECT_ID_STORAGE_CONTROL:
	  {
		ares_SDcontrol_0_1_deserialize_(&sd, (const uint8_t *)receive.payload, &size_bytes);
		bool sd_control_active = (_sd_cap_control.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
		PX4_INFO("SdControl response: node %d, active: %hd", receive.metadata.remote_node_id, sd_control_active);
		if (sd_control_active) {
			_sd_cap_control.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
		}
		break;
	  }
	  case ARES_SUBJECT_ID_ADC_SYNC:
	  {
		ares_SyncControl_0_1_deserialize_(&sync, (const uint8_t *)receive.payload, &size_bytes);
		bool sync_control_active = (_sync_control.get_command_active( receive.metadata.remote_node_id) > 0) ? true : false;
		PX4_INFO("SyncControl response: node %d, active: %hd", receive.metadata.remote_node_id, sync_control_active);
		if (sync_control_active) {
			_sync_control.reset_command_active( receive.metadata.remote_node_id);	// everything good, we got the response we were looking for
		}
		break;
	  }
	}
}

