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
 * @file SubscriptionManager.cpp
 *
 * Manages the UAVCAN subscriptions
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */


#include "SubscriptionManager.hpp"

#include "ParamManager.hpp"

SubscriptionManager::~SubscriptionManager()
{
	//_dynsubscribers.clear();
	_basesubscribers.clear();
}

void SubscriptionManager::subscribe()
{
	_heartbeat_sub.subscribe();

#if CONFIG_CYPHAL_GETINFO_RESPONDER
	_getinfo_rsp.subscribe();
#endif

	_access_rsp.subscribe();
	_list_rsp.subscribe();

	//updateDynamicSubscriptions();
	updateBaseSubscriptions();
}

void SubscriptionManager::updateDynamicSubscriptions()
{
	for (auto &sub : _cyphal_dyn_subs) {

		bool found_subscriber = false;

		// for (auto &dynsub : _dynsubscribers) {
		// 	// Check if subscriber has already been created
		// 	char full_subj_name[200];
		// 	snprintf(full_subj_name, sizeof(full_subj_name), "%s%s", dynsub->getSubjectPrefix(), dynsub->getSubjectName());
		// 	const uint8_t instance = dynsub->getInstance();

		// 	if (strcmp(full_subj_name, sub.subject_name) == 0 && instance == sub.instance) {
		// 		found_subscriber = true;
		// 		break;
		// 	}
		// }

		if (found_subscriber) {
			continue;
		}

		char cyphal_param[90];
		snprintf(cyphal_param, sizeof(cyphal_param), "cyphal.sub.%s.%d.id", sub.subject_name, sub.instance);
		uavcan_register_Value_1_0 value;

		if (_param_manager.GetParamByName(cyphal_param, value)) {
			uint16_t port_id = value.natural16.value.elements[0];

			if (port_id <= CANARD_PORT_ID_MAX) { // PortID is set, create a subscriber
				// UavcanDynamicPortSubscriber *dynsub = sub.create_sub(_canard_handle, _param_manager);

				// if (dynsub == nullptr) {
				// 	PX4_ERR("Out of memory");
				// 	return;
				// }

				// _dynsubscribers.add(dynsub);

				// dynsub->updateParam();
			}

		} else {
			PX4_ERR("Port ID param for subscriber %s.%u not found", sub.subject_name, sub.instance);
			return;
		}
	}
}

void SubscriptionManager::updateBaseSubscriptions()
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

		basesub->updateParam();
	}
}


void SubscriptionManager::printInfo()
{
	// for (auto &dynsub : _dynsubscribers) {
	// 	dynsub->printInfo();
	// }
	for (auto &basesub : _basesubscribers) {
		basesub->printInfo();
	}
}

void SubscriptionManager::updateDynParams()
{
	// for (auto &dynsub : _dynsubscribers) {
	// 	dynsub->updateParam();
	// }
	// // Check for any newly-enabled subscriptions
	// updateDynamicSubscriptions();
}

void SubscriptionManager::updateBaseParams()
{
	for (auto &basesub : _basesubscribers) {
		basesub->updateParam();
	}
	// Check for any newly-enabled subscriptions
	updateBaseSubscriptions();
}

void SubscriptionManager::updateParams()
{
	// updateDynParams();
	updateBaseParams();
}
