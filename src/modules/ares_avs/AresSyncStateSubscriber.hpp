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
 * @file AresSyncStateSubscriber.hpp
 *
 * Defines functionality of ARES Cyphal Sync message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/uavcan_parameter_value.h>
#include "ares/TimeSync_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"

class AresSyncStateSubscriber : public UavcanBaseSubscriber
{
	struct uavcan_parameter_value_s sync;
	orb_advert_t sync_pub;
public:
	AresSyncStateSubscriber(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "sync", instance), _portID(portID) { };

	void subscribe() override
	{
		// Subscribe to CAN message
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _portID,
					   ares_TimeSync_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		/* advertise sync topic */
		memset(&this->sync, 0, sizeof(this->sync));
		this->sync_pub = orb_advertise(ORB_ID(uavcan_parameter_value), &this->sync);
		PX4_INFO("subscribed to TimeSync, port %d", _portID);
	};

	void callback(const CanardRxTransfer &receive) override
	{
		//PX4_INFO("AresSyncStateCallback");

		ares_TimeSync_0_1 timesync {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_TimeSync_0_1_deserialize_(&timesync, (const uint8_t *)receive.payload, &msg_size_in_bits);

		if (timesync.m_bSyncOccurred == true) {
			sync.param_type = AVS_SYNC_OCCURRED;
		}
		else {
			sync.param_type = AVS_SYNC_MSG;
		}
		sync.timestamp = hrt_absolute_time();
		sync.node_id = (uint8_t) receive.metadata.remote_node_id;
		sync.int_value = (int64_t) timesync.m_u32MasterFrcAtPps;	// should make ARES changes to use m_u8SyncState

		orb_publish( ORB_ID(uavcan_parameter_value), this->sync_pub, &this->sync);	///< uORB pub for AVS events
	};
private:
	CanardPortID _portID;
};
