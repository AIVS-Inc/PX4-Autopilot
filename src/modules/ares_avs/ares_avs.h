/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

//#include "ares/AdcFrame_0_1.h"
#include "ares/GnssImu_0_1.h"
#include "ares/Bearings_0_1.h"
#include "ares/GnssPos_0_1.h"
#include "ares/GnssRelPosNed_0_1.h"
#include "ares/FFTcontrol_0_1.h"
#include "ares/EventParams_0_1.h"
#include "ares/Rtcm_0_1.h"

#include "UavCanId.h"

using namespace time_literals;

extern "C" __EXPORT int ares_avs_main(int argc, char *argv[]);


class AresAvs : public ModuleBase<AresAvs>, public ModuleParams
{
public:
	AresAvs(uint8_t nodeID_top, uint8_t nodeID_bot);

	virtual ~AresAvs() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static AresAvs *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int send_fft_params( ares_fft_ParamId param_id);

	int event_command();

	int peak_command();

	int dec_command();

	int len_command();

	int lin_command();

	int win_command();

	int enc_command();

	int cal_command();

	int sync_command_now();

	int sync_command( time_t time_sec);

	int ena_command(bool flag);

	int rtcm_command(bool flag);

	int cap_command(bool flag);
private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);
	uint8_t aresNodeId_top = 0;
	uint8_t aresNodeId_bot = 0;
	bool top_node_reported = false;
	bool bot_node_reported = false;
	bool fftEnable = false;
	bool rtcmActive = false;
	bool captureActive = false;

	DEFINE_PARAMETERS(

		(ParamInt<px4::params::AVS_FFT_DEC>)  	 _fft_dec,
		(ParamInt<px4::params::AVS_FFT_LONG>)  	 _fft_long,
		(ParamInt<px4::params::AVS_FFT_ENCRYPT>) _fft_encrypt,
		(ParamInt<px4::params::AVS_FFT_STRT_BIN>)_fft_start_bin,
		(ParamInt<px4::params::AVS_FFT_NUM_BINS>)_fft_num_bins,
		(ParamInt<px4::params::AVS_FFT_WINDOW>)	 _fft_window,

		(ParamInt<px4::params::AVS_EVT_REL_DB>)  _evt_rel_dB,
		(ParamInt<px4::params::AVS_EVT_NUM_SRC>) _evt_num_src,
		(ParamInt<px4::params::AVS_EVT_ANG_RES>) _evt_ang_res,
		(ParamInt<px4::params::AVS_EVT_BG_TC>) 	 _evt_bg_tc,
		(ParamInt<px4::params::AVS_EVT_EVT_WIN>) _evt_evt_win,
		(ParamInt<px4::params::AVS_EVT_BGSIL>)   _evt_self_bgsil,
		(ParamInt<px4::params::AVS_EVT_BGSIL_DB>)_evt_bgsil_db,

		(ParamInt<px4::params::AVS_PK_NUM_HARM>) _avs_pk_num_harm,
		(ParamInt<px4::params::AVS_PK_NUM_PEAK>) _avs_pk_num_peak,
		(ParamInt<px4::params::AVS_PK_MIN_HGHT>) _avs_pk_min_height,
		(ParamInt<px4::params::AVS_PK_MIN_D_DB>) _avs_pk_min_chg_db,
		(ParamInt<px4::params::AVS_PK_MIN_BLNK>) _avs_pk_min_blanking,

		(ParamInt<px4::params::AVS_SPATIAL_FILT>)_avs_spatial_filt,
		(ParamInt<px4::params::AVS_TARGET_SYNC>) _avs_target_sync,
		(ParamInt<px4::params::AVS_RPM_AVG_MSEC>)_avs_rpm_avg_len,

		(ParamFloat<px4::params::AVS_ANT_DX>)	 _avs_antenna_dx,
		(ParamFloat<px4::params::AVS_ANT_DY>)	 _avs_antenna_dy
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	bool sync_done = false;	// for system sync if there are two nodes

};

