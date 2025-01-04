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
 * @file AresPubManager.cpp
 *
 * Manages the Ares Cyphal publications
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */


#include "AresPubManager.hpp"

AresPubManager::~AresPubManager()
{
	_basepublishers.clear();
}

void AresPubManager::updateBasePublications()
{
	for (auto &sub : _uavcan_base_pubs) {

		bool found_publisher = false;

		for (auto &basepub : _basepublishers) {
			// Check if publisher has already been created
			char full_subj_name[80];
			snprintf(full_subj_name, sizeof(full_subj_name), "%s%s", basepub->getPrefixName(), basepub->getSubjectName());
			const uint8_t instance = basepub->getInstance();

			if (strcmp(full_subj_name, sub.subject_name) == 0 && instance == sub.instance) {
				found_publisher = true;
				break;
			}
		}
		if (found_publisher) {
			continue;
		}
		AresPublisher *basepub = sub.create_pub(_canard_handle);

		if (basepub == nullptr) {
			PX4_ERR("Out of memory");
			return;
		}
		_basepublishers.add(basepub);

		basepub->updateParam();
	}
}

void AresPubManager::printInfo()
{
	for (auto &basepub : _basepublishers) {
		basepub->printInfo();
	}
}

void AresPubManager::updateParams()
{
	for (auto &basepub : _basepublishers) {
		basepub->updateParam();
	}
	// Check for any newly-enabled publication
	updateBasePublications();
}

UavcanPublisher *AresPubManager::getPublisher(const char *subject_name)
{
	return NULL;
}


void AresPubManager::update()
{
	for (auto &basepub : _basepublishers) {
		basepub->update();
	}
}
