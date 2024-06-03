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
 * @file AresSubManager.hpp
 *
 * Manages the Ares Cyphal subscriptions
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include "../../drivers/cyphal/CanardInterface.hpp"

#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"
#include "AresEventSubscriber.hpp"
#include "MelIntensitySubscriber.hpp"
#include "AdcFrameSubscriber.hpp"
#include "GnssPositionSubscriber.hpp"
#include "GnssRelPosNedSubscriber.hpp"

typedef struct {
	UavcanBaseSubscriber *(*create_sub)(CanardHandle &handle) {};
	const char *subject_name;
	CanardPortID id;
	const uint8_t instance;
} AresSubBinder;

class AresSubManager
{
public:
	AresSubManager(CanardHandle &handle) : _canard_handle(handle) {}
	~AresSubManager();

	void subscribe();
	void updateParams();
	void printInfo();
private:
	void updateBaseSubscriptions();
	void updateBaseParams();

	CanardHandle &_canard_handle;
	List<UavcanBaseSubscriber *> _basesubscribers;

	const AresSubBinder _cyphal_base_subs[4]
	{
		{
			[](CanardHandle & handle) -> UavcanBaseSubscriber *
			{
				return new GnssPositionSubscriber(handle, ARES_SUBJECT_ID_GNSS_POSITION, 0);
			},
			"ares.gnsspos",
			0
		},
		{
			[](CanardHandle & handle) -> UavcanBaseSubscriber *
			{
				return new GnssRelPosNedSubscriber(handle, ARES_SUBJECT_ID_GNSS_RELPOSNED, 0);
			},
			"ares.gnssrelposned",
			0
		},
		{
			[](CanardHandle & handle) -> UavcanBaseSubscriber *
			{
				return new AresEventSubscriber(handle, ARES_SUBJECT_ID_FFT_BEARING_ANGLES, 0);
			},
			"ares.bearings",
			0
		},
		{
			[](CanardHandle & handle) -> UavcanBaseSubscriber *
			{
				return new MelIntensitySubscriber(handle, ARES_SUBJECT_ID_FFT_MEL_INTENSITY, 0);
			},
			"ares.melintensity",
			0
		},
		// {
		// 	[](CanardHandle & handle) -> UavcanBaseSubscriber *
		// 	{
		// 		return new AdcFrameSubscriber(handle, ARES_SUBJECT_ID_FFT_ADC_FRAME, 0);
		// 	},
		// 	"ares.adcframe",
		// 	0
		// },
	};
};
