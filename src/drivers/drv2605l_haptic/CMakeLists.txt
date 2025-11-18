
px4_add_module(
	MODULE drivers__drv2605l_haptic
	MAIN drv2605l_haptic
	SRCS
		drv2605l_haptic.cpp
		#drv2605l_params.c
		#drv2605l.cpp
	MODULE_CONFIG
		module.yaml

)



# Ensure the module can see PX4 headers like <parameters/param.h>
#include_directories(${PX4_SOURCE_DIR}/src)


# The px4_add_module() method builds a static library from a module description.

#     The MODULE block is the Firmware-unique name of the module (by convention the module name is prefixed by parent directories back to src).
#     The MAIN block lists the entry point of the module, which registers the command with NuttX so that it can be called from the PX4 shell or SITL console.


