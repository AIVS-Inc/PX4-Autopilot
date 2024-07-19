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

#ifndef REL_POS_NED_HPP
#define REL_POS_NED_HPP

#include <uORB/topics/sensor_gnss_relative.h>

class MavlinkStreamRelPosNed : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRelPosNed(mavlink); }

	static constexpr const char *get_name_static() { return "REL_POS_NED"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_REL_POS_NED; }

	const char *get_name() const override { return MavlinkStreamRelPosNed::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_gnss_relative_sub.advertised() ? MAVLINK_MSG_ID_REL_POS_NED + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamRelPosNed(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gnss_relative_sub{ORB_ID(sensor_gnss_relative)};

	bool send() override
	{
		sensor_gnss_relative_s relposned;

		if (_sensor_gnss_relative_sub.update(&relposned)) {
			mavlink_rel_pos_ned_t msg{};

			msg.north = relposned.position[0];
			msg.east = relposned.position[1];
			msg.down = relposned.position[2];
			msg.acc_n = relposned.position_accuracy[0];
			msg.acc_e = relposned.position_accuracy[1];
			msg.acc_d = relposned.position_accuracy[2];
			msg.head = relposned.heading;
			msg.acc_h = relposned.heading_accuracy;
			msg.len = relposned.position_length;
			msg.acc_l = relposned.accuracy_length;
			msg.node_id = (uint8_t) relposned.device_id;
			msg.fixed = (uint8_t) relposned.carrier_solution_fixed;

			mavlink_msg_rel_pos_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // REL_POS_NED_HPP
