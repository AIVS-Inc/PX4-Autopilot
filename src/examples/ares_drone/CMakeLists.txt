############################################################################
#
#   Copyright (c) 2023 AIVS Inc. All rights reserved.
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

# set(ares_all)
# list(APPEND ares_all
# 	Application
# 	Accelerometer
# 	Adc
# 	CanBus
# 	Clock
# 	Common
# 	Console
# 	Fft
# 	Gnss
# 	Heartbeat
# 	HinnantDate
# 	LibCanard
# 	Phasor
# 	Storage
# )
# add_subdirectory(Application)
# add_subdirectory(Accelerometer)
# add_subdirectory(Adc)
# add_subdirectory(CanBus)
# add_subdirectory(Clock)
# add_subdirectory(Common)
# add_subdirectory(Console)
# add_subdirectory(Fft)
# add_subdirectory(Gnss)
# add_subdirectory(Heartbeat)
# add_subdirectory(HinnantDate)
# add_subdirectory(LibCanard)
# add_subdirectory(Phasor)
# add_subdirectory(Storage)

find_program(NNVG_PATH nnvg)
if(NNVG_PATH)
	message("Generating Cyphal ARES DSDL headers using Nunavut")
	execute_process(COMMAND ${NNVG_PATH} ares --outdir ${CMAKE_CURRENT_SOURCE_DIR} --target-language c -I /home/ares/PX4-Autopilot/src/drivers/cyphal/public_regulated_data_types/uavcan -I /home/ares/PX4-Autopilot/src/drivers/cyphal/public_regulated_data_types/reg )
else()
	message(FATAL_ERROR "UAVCAN Nunavut nnvg not found")
endif()

px4_add_module(
	MODULE examples__ares_drone
	MAIN ares_drone
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_BINARY_DIR}
		"/home/ares/PX4-Autopilot/build/px4_fmu-v6xrt_default/src/drivers/cyphal/dsdlc_generated/"
	STACK_MAIN 4096
	SRCS
		ares_drone_main.cpp
		ares_drone.h
		ares/AdcFrame.0.1.dsdl
		ares/GnssImu.0.1.dsdl
		ares/Bearings.0.1.dsdl
		ares/GnssPos.0.1.dsdl
		ares/GnssRelPosNed.0.1.dsdl
		ares/Rtcm.0.1.dsdl
		ares/EventParams.0.1.dsdl
		ares/FFTcontrol.0.1.dsdl
		AresEventSubscriber.hpp
		AresEventPublisher.hpp
		GnssPositionSubscriber.hpp
		GnssRelPosNedSubscriber.hpp
		AdcFrameSubscriber.hpp
		UavCanId.h
	MODULE_CONFIG
		module.yaml
	DEPENDS
		uorb_msgs
	)
