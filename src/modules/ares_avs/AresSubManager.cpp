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
 * @file AresSubManager.cpp
 *
 * Manages the ARES Cyphal subscriptions
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */


#include "AresSubManager.hpp"

AresSubManager::~AresSubManager()
{
	//_dynsubscribers.clear();
	_basesubscribers.clear();
}

void AresSubManager::subscribe()
{
	updateBaseSubscriptions();
}

void AresSubManager::updateBaseSubscriptions()
{
	for (auto &sub : _cyphal_base_subs) {

		bool found_subscriber = false;

		for (auto &basesub : _basesubscribers) {
			// Check if subscriber has already been created
			char full_subj_name[200];
			snprintf(full_subj_name, sizeof(full_subj_name), "%s%s", basesub->getSubjectPrefix(), basesub->getSubjectName());
			const uint8_t instance = basesub->getInstance();

			if (strcmp(full_subj_name, sub.subject_name) == 0 && instance == sub.instance) {
				found_subscriber = true;
				break;
			}
		}
		if (found_subscriber) {
			continue;
		}
		UavcanBaseSubscriber *basesub = sub.create_sub(_canard_handle);

		if (basesub == nullptr) {
			PX4_ERR("Out of memory");
			return;
		}
		_basesubscribers.add(basesub);
	}
}


void AresSubManager::printInfo()
{
	for (auto &basesub : _basesubscribers) {
		basesub->printInfo();
	}
}

void AresSubManager::updateBaseParams()
{
	// Check for any newly-enabled subscriptions
	updateBaseSubscriptions();
}

void AresSubManager::updateParams()
{
	updateBaseParams();
}
