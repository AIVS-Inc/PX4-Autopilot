/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file AresPubManager.hpp
 *
 * Manages the dynamic (run-time configurable) UAVCAN publications
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>
#include "AresPublisher.hpp"
#include "../../drivers/cyphal/CanardInterface.hpp"
#include "../../drivers/cyphal/Publishers/Publisher.hpp"

#include "AresEventPublisher.hpp"
#include "GnssRtcmPublisher.hpp"
#include "UavCanId.h"

typedef struct {
	AresPublisher *(*create_pub)(CanardHandle &handle) {};
	const char *subject_name;
	CanardPortID id;
	const uint8_t instance;
} AresPubBinder;

class AresPubManager
{
public:
	AresPubManager(CanardHandle &handle) : _canard_handle(handle) {}
	~AresPubManager();

	void update();
	void printInfo();
	void updateParams();

	UavcanPublisher *getPublisher(const char *subject_name);

private:
	void updateBasePublications();

	CanardHandle &_canard_handle;
	List<AresPublisher *> _basepublishers;


	const AresPubBinder _uavcan_base_pubs[2] {
		{
			[](CanardHandle & handle) -> AresPublisher *
			{
				return new AresEventPublisher(handle, 0);
			},
			"ares.esc",
			0
		},
		{
			[](CanardHandle & handle) -> AresPublisher *
			{
				return new GnssRtcmPublisher(handle, 0);
			},
			"ares.rtcm",
			0
		},
	};
};
