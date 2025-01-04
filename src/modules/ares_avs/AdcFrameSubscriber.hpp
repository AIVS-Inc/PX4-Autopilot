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
 * @file AdcFrameSubscriber.hpp
 *
 * Defines functionality of ARES Cyphal ADC Frame message subscription
 *
 * @author Jim Waite <jim.waite@aivs.us>
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/sensor_avs_adc.h>
#include <uORB/topics/mavlink_tunnel.h>
#include "ares/AdcFrame_0_1.h"
#include "UavCanId.h"
#include "../../drivers/cyphal/Subscribers/BaseSubscriber.hpp"
#include "../../platforms/nuttx/NuttX/nuttx/net/udp/udp.h"
#include "../../platforms/nuttx/NuttX/nuttx/net/tcp/tcp.h"
#include <px4_platform_common/module_params.h>

class AdcFrameSubscriber : public UavcanBaseSubscriber
{
	struct sensor_avs_adc_s timedata;
	struct mavlink_tunnel_s tunneldata;

	orb_advert_t adc_pub;
public:
	AdcFrameSubscriber(CanardHandle &handle, CanardPortID portID, uint8_t instance = 0) :
		UavcanBaseSubscriber(handle, "ares.", "adcframe", instance), _portID(portID) { };

	void subscribe() override
	{
		// Subscribe to CAN message
		_canard_handle.RxSubscribe(CanardTransferKindMessage,
					   _portID,
					   ares_AdcFrame_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

		PX4_INFO("subscribed to AdcFrame, port %d", _portID);
		param_get(param_find("AVS_IP_XFER_TYPE"), &_ip_xfer_type);
		param_get(param_find("AVS_IP_XFER_ADDR"), &_ip_xfer_addr);
		param_get(param_find("AVS_IP_XFER_PORT"), &_ip_xfer_port);

		unsigned char octets[4];
		octets[0] = (_ip_xfer_addr >> 24) & 0xFF;
		octets[1] = (_ip_xfer_addr >> 16) & 0xFF;
		octets[2] = (_ip_xfer_addr >> 8) & 0xFF;
		octets[3] = _ip_xfer_addr & 0xFF;
    		sprintf(ipString, "%d.%d.%d.%d", octets[0], octets[1], octets[2], octets[3]);
		PX4_INFO("Selected IP address: %s", ipString);

		if (_ip_xfer_type == ARES_UDP_BROADCAST) {
			// Initialize UDP socket
			_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
			if (_udp_socket < 0) {
				PX4_ERR("Failed to create UDP socket");
				return;
			}

			// Set up UDP destination address
			_udp_dest_addr.sin_family = AF_INET;
			_udp_dest_addr.sin_port = htons(_ip_xfer_port); 			// Destination port
			inet_pton(AF_INET, ipString, &_udp_dest_addr.sin_addr); 		// Destination IP

		} else if (_ip_xfer_type == ARES_TCP_CLIENT)  {
			// Initialize TCP socket
			_tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
			if (_tcp_socket < 0) {
				PX4_ERR("Failed to create TCP socket");
				return;
			}
			// Set up server address
			_tcp_dest_addr.sin_family = AF_INET;
			_tcp_dest_addr.sin_port = htons(_ip_xfer_port); // server port
			if (inet_pton(AF_INET, "10.208.78.173", &_tcp_dest_addr.sin_addr) <= 0) {
				PX4_ERR("Invalid destination address");
				return;
			}
			// Connect to the server
			if (connect(_tcp_socket, (struct sockaddr *)&_tcp_dest_addr, sizeof(_tcp_dest_addr)) < 0) {
				PX4_ERR("Failed to connect to TCP server");
				close(_tcp_socket);
				return;
			}
			PX4_INFO("Connected to TCP/IP server");

		} else if (_ip_xfer_type == ARES_TCP_CLIENT_TEST) {
			// Initialize TCP socket
			_tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
			if (_tcp_socket < 0) {
				PX4_ERR("Failed to create TCP socket");
				return;
			}
			// Set up server address
			_tcp_dest_addr.sin_family = AF_INET;
			_tcp_dest_addr.sin_port = htons(_ip_xfer_port); // server port
			if (inet_pton(AF_INET, "10.208.78.173", &_tcp_dest_addr.sin_addr) <= 0) {
				PX4_ERR("Invalid destination address");
				return;
			}
			// Connect to the server
			if (connect(_tcp_socket, (struct sockaddr *)&_tcp_dest_addr, sizeof(_tcp_dest_addr)) < 0) {
				PX4_ERR("Failed to connect to TCP server");
				close(_tcp_socket);
				return;
			}
			PX4_INFO("Connected to TCP/IP server");

		} else if (_ip_xfer_type == ARES_TCP_SERVER) {
			// Initialize TCP server socket
			_tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0);
			if (_tcp_server_socket < 0) {
				PX4_ERR("Failed to create TCP server socket");
				return;
			}
			// Set up server address
			struct sockaddr_in server_addr;
			server_addr.sin_family = AF_INET;
			server_addr.sin_port = htons(_ip_xfer_port); // Server port
			inet_pton(AF_INET, "0.0.0.0", &server_addr.sin_addr); // Bind to all available network interfaces

			// Bind the server socket
			if (bind(_tcp_server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
				PX4_ERR("Failed to bind TCP server socket");
				close(_tcp_server_socket);
				return;
			}

			// Listen for incoming connections
			if (listen(_tcp_server_socket, 1) < 0) {
				PX4_ERR("Failed to listen on TCP server socket");
				close(_tcp_server_socket);
				return;
			}
			PX4_INFO("TCP server listening on port %ld", _ip_xfer_port);

		} else if (_ip_xfer_type == ARES_UORB_TIME_XFER) {
			/* advertise timedata topic */
			memset(&this->timedata, 0, sizeof(this->timedata));
			this->adc_pub = orb_advertise(ORB_ID(sensor_avs_adc), &this->timedata);

		} else if (_ip_xfer_type == ARES_MAV_TUNNEL_XFER) {
			/* advertise tunneldata topic */
			memset(&this->tunneldata, 0, sizeof(this->tunneldata));
			this->adc_pub = orb_advertise(ORB_ID(mavlink_tunnel), &this->tunneldata);
		}
	};

	void callback(const CanardRxTransfer &receive) override
	{
		int i,j;
		int channel_offset = 96;	// channels are not interleaved, this is 4-byte words between channels
		int samples_per_chunk = 32;	// samples

		ares_AdcFrame_0_1 adcframe {};
		size_t msg_size_in_bits = receive.payload_size;
		ares_AdcFrame_0_1_deserialize_(&adcframe, (const uint8_t *)receive.payload, &msg_size_in_bits);

		uint64_t utc_us = adcframe.m_u32JulianMicrosecond - 3506716800000000;	// difference between modified Julian and UTC microseconds
		reg_udral_physics_kinematics_geodetic_Point_0_1 geo = adcframe.m_glGnssLlh;
		double lat = geo.latitude;
		double lon = geo.longitude;
		double alt = geo.altitude.meter;
		uavcan_si_unit_length_Scalar_1_0 dh = adcframe.m_fHorizontalAccuracy;
		uavcan_si_unit_length_Scalar_1_0 dv = adcframe.m_fVerticalAccuracy;
		uavcan_si_unit_angle_Scalar_1_0 pitch = adcframe.m_fPitch;
		uavcan_si_unit_angle_Scalar_1_0 roll = adcframe.m_fRoll;
		uavcan_si_unit_angle_Scalar_1_0 yaw = adcframe.m_fYaw;
		uint32_t node = receive.metadata.remote_node_id;
		uint32_t tID = receive.metadata.transfer_id;

		PX4_INFO("node:%lu,id:%lu,usec:%llu",   //,lat:%f,lon:%f,alt:%f,dh:%f,dv:%f,pitch:%f,roll:%f,yaw:%f
		 	  node,tID,utc_us); 		//,lat,lon,alt,(double)dh.meter,(double)dv.meter,(double)pitch.radian,(double)roll.radian,(double)yaw.radian );

		timedata.timestamp = hrt_absolute_time();
		timedata.device_id = node;
		timedata.transfer_id = tID;
		timedata.time_utc_usec = utc_us;
		timedata.latitude_deg = lat;
		timedata.latitude_deg = lon;
		timedata.altitude_ellipsoid_m = alt;
		timedata.eph = dh.meter;
		timedata.epv = dv.meter;
		timedata.pitch = pitch.radian;
		timedata.roll = roll.radian;
		timedata.yaw = yaw.radian;
		timedata.timestamp_sample = adcframe.m_u32SampleIndex;
		memcpy(timedata.adc_frame, adcframe.m_ai32Data, sizeof(adcframe.m_ai32Data));

		if (_ip_xfer_type == ARES_UORB_TIME_XFER) {
			// uORB publication
			orb_publish( ORB_ID(sensor_avs_adc), this->adc_pub, &this->timedata);

		} else if (_ip_xfer_type == ARES_MAV_TUNNEL_XFER) {
			// Create the tunnel message
			tunneldata.timestamp = hrt_absolute_time();			// 8 bytes
			tunneldata.target_system = 0;  // Target ground station (or set to 0 for broadcast)
			tunneldata.target_component = 0;
			tunneldata.payload_type = 1;   // Custom payload type
			tunneldata.payload_length = sizeof(tunnel_hdr) + samples_per_chunk*4;

			/* Copy data (4ch * 96samp * 4bytes/samp + header) into the mavlink payload (128 bytes).
			 * Do this in a double nested loop (channels, 32 samples/chan).
			 * First copy in the common 16-byte header, then the 96 byte data payload (total 112):
			 */
			tunnel_hdr.device_id = node;				// 2 bytes
			tunnel_hdr.tID = tID;					// 2 bytes
			tunnel_hdr.utc_usec = utc_us;				// 8 bytes
			tunnel_hdr.sample_idx = adcframe.m_u32SampleIndex;	// 4 bytes
			memcpy(tunneldata.payload, &tunnel_hdr, sizeof(tunnel_hdr));

			for (i=0; i<4; i++) {
				for (j=0; j<4; j++) {
					memcpy(tunneldata.payload+sizeof(tunnel_hdr), adcframe.m_ai32Data+channel_offset*i+samples_per_chunk*j, samples_per_chunk*4);
					orb_publish( ORB_ID(mavlink_tunnel), this->adc_pub, &this->tunneldata);
				}
			}

		} else if (_ip_xfer_type == ARES_UDP_BROADCAST) {
		        // Send sensor data over UDP
		        sendto(_udp_socket, &timedata, sizeof(timedata), 0, (struct sockaddr *)&_udp_dest_addr, sizeof(_udp_dest_addr));

		} else if (_ip_xfer_type == ARES_TCP_CLIENT) {
			// Send sensor data over TCP
			ssize_t bytes_sent = send(_tcp_socket, &timedata, sizeof(timedata), 0);
			if (bytes_sent != sizeof(timedata)) {
				PX4_ERR("Failed to send data over TCP");
			}
			//else {
			//	PX4_INFO("sent %d bytes to TCP server", bytes_sent);
			//}
			// Close the TCP socket (need to do this after test is done)
			// close(_tcp_socket);

		} else if (_ip_xfer_type == ARES_TCP_SERVER) {
			_client_len = sizeof(_client_addr);

			// Accept incoming connection
			_tcp_client_socket = accept(_tcp_server_socket, (struct sockaddr *)&_client_addr, &_client_len);
			if (_tcp_client_socket < 0) {
				return;

			} else {
				// Send sensor data over TCP
				ssize_t bytes_sent = send(_tcp_client_socket, &timedata, sizeof(timedata), 0);
				if (bytes_sent!= sizeof(timedata)) {
					PX4_ERR("Failed to send data over TCP");
				}
				// Close the TCP client socket (need to do this after test is done)
				// close(_tcp_client_socket);
			}

		} else if (_ip_xfer_type == ARES_TCP_CLIENT_TEST) {
			uint32_t ramp[] = {0,1,2,3,4,5,6,7,8,9};
			ssize_t bytes_sent = send(_tcp_socket, &ramp, sizeof(ramp), 0);
			if (bytes_sent != sizeof(ramp)) {
				PX4_ERR("Failed to send data over TCP");
			}
		}
	};

private:
	CanardPortID _portID;
	int32_t _ip_xfer_type;
	int32_t _ip_xfer_addr;
	int32_t _ip_xfer_port;
	char ipString[32];

	int _udp_socket;
	struct sockaddr_in _udp_dest_addr;

	int _tcp_socket;
	struct sockaddr_in _tcp_dest_addr;

	int _tcp_server_socket;
	int _tcp_client_socket;
	struct sockaddr_in _client_addr;
	socklen_t _client_len;

	mvhdr_t tunnel_hdr;
    	static constexpr uint16_t MAX_TUNNEL_PAYLOAD = 128;  // Max size for tunnel message
};
