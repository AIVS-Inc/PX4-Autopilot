############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(LIBCANARD_DIR ../../drivers/cyphal/libcanard)

px4_add_module(
	MODULE modules__ares_avs
	MAIN ares_avs
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	INCLUDES
		${LIBCANARD_DIR}/libcanard/
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_BINARY_DIR}
		"/home/ares/PX4-Autopilot/build/px4_fmu-v6xrt_default/src/drivers/cyphal/dsdlc_generated/"
	STACK_MAIN 2048
	SRCS
		ares_avs.cpp
		ares_avs.h
		ares/AdcFrame.0.1.dsdl
		ares/GnssImu.0.1.dsdl
		ares/Bearings.0.1.dsdl
		ares/GnssPos.0.1.dsdl
		ares/GnssRelPosNed.0.1.dsdl
		ares/Rtcm.0.1.dsdl
		ares/EventParams.0.1.dsdl
		ares/FFTcontrol.0.1.dsdl
		ares/SDcontrol.0.1.dsdl
		ares/GnssControl.0.1.dsdl
		ares/FFTlength.0.1.dsdl
		ares/FFTlinear.0.1.dsdl
		ares/FFTspatial.0.1.dsdl
		ares/FFTwindow.0.1.dsdl
		ares/FFTencrypt.0.1.dsdl
		AresEventPublisher.hpp
		GnssRtcmPublisher.hpp
		AresEventSubscriber.hpp
		MelIntensitySubscriber.hpp
		GnssRelPosNedSubscriber.hpp
		GnssPositionSubscriber.hpp
		AdcFrameSubscriber.hpp
		AresSyncStateSubscriber.hpp
		AresFftParamServiceRequest.hpp
		AresFftControlServiceRequest.hpp
		AresSdCapControlServiceRequest.hpp
		AresGnssControlServiceRequest.hpp
		AresSyncControlServiceRequest.hpp
		AresPublisher.hpp
		AresPubManager.hpp
		AresPubManager.cpp
		AresSubManager.hpp
		AresSubManager.cpp
		UavCanId.h
	MODULE_CONFIG
		module.yaml
	DEPENDS
		uorb_msgs
	)

