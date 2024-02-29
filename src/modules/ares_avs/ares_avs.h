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

	int send_command();

	int enable_command();

	int disable_command();
private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);
	uint8_t aresNodeId_top;
	uint8_t aresNodeId_bot;
	bool fftEnable;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) 	_param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,  /**< another parameter */
		(ParamInt<px4::params::EVT_REL_DB>) 	_evt_rel_dB,
		(ParamInt<px4::params::EVT_NUM_SRC>) 	_evt_num_src,
		(ParamInt<px4::params::EVT_ANG_RES>) 	_evt_ang_res,
		(ParamInt<px4::params::EVT_BG_TC>) 	_evt_bg_tc,
		(ParamInt<px4::params::EVT_EVT_WIN>) 	_evt_evt_win
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

};

