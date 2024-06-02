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

#include "../../drivers/cyphal/Publishers/BasePublisher.hpp"
#include "ares/Rtcm_0_1.h"
#include "UavCanId.h"
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gps_inject_data.h>

/*
 * This module acts as a bridge for rtcm correction data from a base station, or NTRIP source,
 * to Cyphal-CAN for use by an ARES node designated as a moving base.
 */
class GnssRtcmPublisher : public BasePublisher
{
public:
	GnssRtcmPublisher(CanardHandle &handle, uint8_t instance = 0) :
		BasePublisher(handle, "ares", "rtcm", instance)
	{
	};

	~GnssRtcmPublisher() override = default;

	// Update the uORB Subscription and broadcast a cyphal message
	void update() override
	{
		if (_rtcm_sub.updated()) {

			ares_Rtcm_0_1 dest {};
			size_t dest_payload_size = ares_Rtcm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
			uint8_t dest_payload_buffer[ares_Rtcm_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
			bool send_msg = false;

			gps_inject_data_s rtcm {};
			_rtcm_sub.update(&rtcm);

			if (rtcm.flags & 0x01) {
				// this is a fragment
				if (_rtcm_offset == 0) {
					// Start of a new fragmented message
					if (rtcm.data[0] == 0xD3) {
						// start of fragmented message
						_rtcm_len = ( rtcm.data[1] & 0x03 ) << 8; // Set the last two bits of this byte. Bits 8 and 9 of 10-bit length
						_rtcm_len |= rtcm.data[2]; 		  // Bits 0-7 of packet length
						_rtcm_len += 6;        			  // 6 additional bytes of header, msgType, CRC
						if (_rtcm_len > 300) {
							PX4_ERR("RTCM message size exceeds allocated buffer size");
						}
						// Copy the fragment into the local buffer
						memcpy(_rtcm_buf,rtcm.data,rtcm.len);
						_rtcm_offset += rtcm.len;
					}
					else {
						PX4_INFO("Unknown byte in expected start of RTCM packet, discarding packet");
					}
				}
				else if (_rtcm_len > 0) {
					// Continuation of fragmented message that had a valid header with message length
					if (_rtcm_offset + rtcm.len > 300) {
						PX4_ERR("Fragmented message growth exceeds allocated buffer size");
						_rtcm_offset = 0;
						_rtcm_len = 0;
					}
					else {
						memcpy(_rtcm_buf+_rtcm_offset,rtcm.data,rtcm.len);
						_rtcm_offset += rtcm.len;
					}
					if (_rtcm_offset == _rtcm_len) {
						// Message is now complete
						memcpy(dest.m_u8Data.elements,_rtcm_buf,_rtcm_len);
						dest.m_u8Data.count = _rtcm_len;
						_rtcm_offset = 0;
						_rtcm_len = 0;
						send_msg = true;
					}
				}
				else {
					PX4_ERR("Detected zero length RTCM packet");
				}
			}
			else {
				// This is a complete RTCM message with no fragmentation
				if (_rtcm_offset != 0) {
					PX4_ERR("Last fragmented RTCM packet was not complete, discarding...");
					_rtcm_offset = 0;
					_rtcm_len = 0;
				}
				else {
					memcpy(dest.m_u8Data.elements,rtcm.data,rtcm.len);
					dest.m_u8Data.count = rtcm.len;
					send_msg = true;
				}
			}
			if (send_msg == true) {
				const CanardTransferMetadata transfer_metadata = {
					.priority       = CanardPriorityNominal,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = ARES_SUBJECT_ID_GNSS_NTRIP,
					.remote_node_id = CANARD_NODE_ID_UNSET,
					.transfer_id    = _transfer_id,
				};
				int32_t result = ares_Rtcm_0_1_serialize_(&dest, dest_payload_buffer, &dest_payload_size);
				//PX4_INFO("copy %d RTCM bytes to CAN, transfer_id %ld", dest.m_u8Data.count, _transfer_id);

				if (result == 0) {
					++_transfer_id;
					result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
								&transfer_metadata,
								dest_payload_size,
								&dest_payload_buffer);
				}
			}
		}
	};

private:
	uORB::Subscription _rtcm_sub{ORB_ID(gps_inject_data)};
	uint8_t _rtcm_buf[300] = {0};			// buffers a fragmented message
	uint16_t _rtcm_len {0};				// length of the RTCM message being received, possible in parts
	uint16_t _rtcm_offset {0};			// current offset into a fragmented message
	CanardTransferID _transfer_id {0};
};
