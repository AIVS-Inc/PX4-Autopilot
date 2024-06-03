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

#include <px4_platform_common/defines.h>
#include "AresPublisher.hpp"
#include "../../drivers/cyphal/Services/ServiceRequest.hpp"

class AresServiceRequest : public UavcanServiceRequest
{
public:
	AresServiceRequest(CanardHandle &handle, const char *prefix_name, const char *subject_name, CanardPortID portID, size_t extent ) :
		UavcanServiceRequest(handle, prefix_name, subject_name, portID, extent) { };

	bool request(const CanardMicrosecond             tx_deadline_usec,
		     const CanardTransferMetadata        *transfer_metadata,
		     const size_t                        payload_size,
		     const void *const                   payload,
		     UavcanServiceRequestInterface       *handler)
	{
		if (_node_id_top == transfer_metadata->remote_node_id)
			request_transfer_id = _transfer_id_top;
		else if (_node_id_bot == transfer_metadata->remote_node_id)
			request_transfer_id = _transfer_id_bot;
		else
			request_transfer_id = 0;

		return UavcanServiceRequest::request(tx_deadline_usec, transfer_metadata, payload_size, payload, handler);
	}

	void callback(const CanardRxTransfer &receive) override
	{
		remote_node_id = receive.metadata.remote_node_id;

		if (remote_node_id == _node_id_top)
			request_transfer_id = _transfer_id_top + 1;	// ARES sends an incremented transfer_id as a response
		else if (remote_node_id == _node_id_bot)		// i.e., doesn't meet the Cyphal spec, FIX this
			request_transfer_id = _transfer_id_bot + 1;
		else
			request_transfer_id = 0;

		PX4_INFO("AresService Response: node %d, transfer_id %d", receive.metadata.remote_node_id, request_transfer_id);
		UavcanServiceRequest::callback( receive);
	};

	hrt_abstime get_command_active( uint8_t node_id)
	{
		if (node_id == _node_id_top)
			return _active_top;
		else if (node_id == _node_id_bot)
			return _active_bot;
		else
			return 0;
	}

	void reset_command_active( uint8_t node_id)
	{
		if (node_id == _node_id_top)
			_active_top = 0;
		else if (node_id == _node_id_bot)
			_active_bot = 0;
	}

protected:
	CanardTransferID _transfer_id_top {0};
	CanardTransferID _transfer_id_bot {0};
	uint8_t _node_id_top {0};
	uint8_t _node_id_bot {0};
	hrt_abstime _active_top {0};		// remember active mode for later confirmation of response
	hrt_abstime _active_bot {0};
};
