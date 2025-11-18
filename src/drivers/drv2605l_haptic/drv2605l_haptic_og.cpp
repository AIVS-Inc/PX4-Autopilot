// /****************************************************************************
//  *
//  *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// /**
//  * @file drv2605l_haptic.cpp
//  * Haptic feedback driver for DRV2605L on ARK FPV - Regular Task Version
//  *
//  * @author Evelena
//  */

//  // preprocessor directives - instructions that are processed before the actual compilation
//  // of your code begins. They start with # and don't end with a semicolon.

//  // #include directive tells the preprocessor to insert the contents of another file into your code.
//  // < > For system/standard library headers
//  // " " For user-defined headers
//  // #define directive creates macros - text replacements that happen before compilation.
// // 	#define IDENTIFIER replacement_text
// // 	#define MACRO(parameters) replacement_text

//  //#include "drv2605l_haptic.hpp"
// #include <px4_platform_common/px4_config.h>
// #include <px4_platform_common/defines.h>
// #include <px4_platform_common/module.h>
// #include <px4_platform_common/getopt.h>
// #include <px4_platform_common/log.h>
// #include <drivers/device/i2c.h>
// #include <drivers/drv_sensor.h>
// #include <lib/parameters/param.h>
// #include <matrix/math.hpp>
// #include <uORB/Subscription.hpp>
// #include <uORB/topics/vehicle_attitude.h>

// // Device type for DRV2605L
// #define DRV_HAPTIC_DEVTYPE_DRV2605L 0x60

// // DRV2605L Register addresses
// #define DRV2605L_ADDR           0x5A
// #define DRV2605L_REG_STATUS     0x00
// #define DRV2605L_REG_MODE       0x01
// #define DRV2605L_REG_RTPIN      0x02
// #define DRV2605L_REG_LIBRARY    0x03
// #define DRV2605L_REG_WAVESEQ1   0x04
// #define DRV2605L_REG_GO         0x0C
// #define DRV2605L_REG_FEEDBACK   0x1A

// // Mode register values
// #define DRV2605L_MODE_INTTRIG   0x00  // Internal trigger mode
// #define DRV2605L_MODE_STANDBY   0x40  // Standby mode

// // Library values
// #define DRV2605L_LIB_ERM        0x01  // ERM library
// #define DRV2605L_LIB_LRA        0x06  // LRA library

// // Waveform effects
// #define DRV2605L_EFFECT_STRONG_CLICK    1
// #define DRV2605L_EFFECT_MEDIUM_CLICK    10
// #define DRV2605L_EFFECT_BUZZ            47

// // Yaw range for haptic feedback (degrees)
// #define YAW_MIN_DEG  135.0f
// #define YAW_MAX_DEG  225.0f

// // declares new class DRV2605L
// class DRV2605L : public ModuleBase<DRV2605L>, public device::I2C
// {
// public:
// // Public Members (accessible from outside the class)

// 	DRV2605L(int bus, int address); //constructor
// 	virtual ~DRV2605L(); // destructor

// 	/** @see ModuleBase */
// 	// static factory/ management methods

// 	// int argc: argument count that tells you how many command-line arguments were passed to your program
// 	// char *argv[]: argument vector - an array of C-style string (character pointers) containing actual
// 	// 	command-line arguments
// 	// const char *reason = nullptr  is a function parameter with a default value:
// 	//	const char *reason - A pointer to a constant character string
// 	// 	= nullptr - If the caller doesn't provide this argument, it defaults to nullptr (null pointer)
// 	static int task_spawn(int argc, char *argv[]);
// 	static DRV2605L *instantiate(int argc, char *argv[]);
// 	static int custom_command(int argc, char *argv[]);
// 	static int print_usage(const char *reason = nullptr);

// 	//overriden virtual methods
// 	void run() override;
// 	int print_status() override;

// private:
// // Private Members (only accessible within the class)
// 	// methods
// 	int init();
// 	int probe() override;
// 	int write_register(uint8_t reg, uint8_t value);
// 	int read_register(uint8_t reg, uint8_t &value);
// 	int trigger_effect(uint8_t effect);

// 	float get_yaw_from_quaternion(const vehicle_attitude_s &att);
// 	bool is_yaw_in_range(float yaw_deg);
// 	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)}; // uORB subscription for vehicle attitude

// 	// data members
// 	bool _initialized{false};
// 	bool _haptic_active{false};  // Track if haptic is currently active
// 	uint32_t _loop_interval_us{100000}; // 100ms
// };

// // class name (the scope) :: constructor name (always matches the class name)
// // :: scope resolution operator


// // constructor for DRV2605L class
// // this constructor sets up the I2C communication interface for the DRV2605L haptic driver,
// // configuring it to communicate at 400 kHz on the specified bus and address.
// DRV2605L::DRV2605L(int bus, int address) :  //The colon (:) starts the member initialization list, which calls the constructor of the parent class I2C
// 	// The DRV2605L class inherits from I2C, and this initializes the base class
// 	I2C(DRV_HAPTIC_DEVTYPE_DRV2605L, MODULE_NAME, bus, address, 400000)
// 		// DRV_HAPTIC_DEVTYPE_DRV2605L - device type identifier (likely a constant)
// {
// }

// //deconstructor for DRV2605L class
// // ~DRV2605L() ensures the haptic device is safely shut down whenever the object is destroyed
// DRV2605L::~DRV2605L()  // prefix: ~ declares the destructor
// {
// 	// Ensure we put device in standby before exit
// 	if (_initialized) {  // _initialized is a private boolean member variable set during initialization
// 		write_register(DRV2605L_REG_MODE, DRV2605L_MODE_STANDBY); //Writes to MODE register of the DRV2605L chip & Sets device to STANDBY mode
// 	}
// }

// // check if device is present and communicating properly on I2C bus
// // called during system startup to verify the haptic driver chip is
// // connected and responsive before attempting further operations.
// int DRV2605L::probe()
// {
// 	uint8_t status; // store status register
// 	int ret = read_register(DRV2605L_REG_STATUS, status);

// 	if (ret == OK) {
// 		PX4_INFO("DRV2605L found, status: 0x%02X", status);
// 		return OK;
// 	}

// 	PX4_ERR("DRV2605L not found");
// 	return -EIO; // Input/Output error code
// }

// // Writes a value to a specific register on the chip
// // cmd is a local array variable that holds data to be sent via I2C
// int DRV2605L::write_register(uint8_t reg, uint8_t value)
// {
// 	uint8_t cmd[2] = {reg, value}; //Creates a 2-byte array;
// 		// cmd[0] = register address you want to write to
// 		// cmd[1] = the value you want to write
// 		// write_register(0x01, 0x05) would write the value 0x05 to register 0x01
// 	//Calls transfer() to send these 2 bytes to the device
// 		return transfer(cmd, 2, nullptr, 0); //nullptr, 0 means "don't read any data back"

// 	// ## Example
// 	// If you call `write_register(0x1C, 0x64)`:
// 	// ```
// 	// cmd[0] = 0x1C  (register address)
// 	// cmd[1] = 0x64  (value to write, which is 100 in decimal)
// }

// // Reads the current value from a specific register
// int DRV2605L::read_register(uint8_t reg, uint8_t &value)
// {
// 	// transfer() function handles the I2C protocol of "write register address, then read data"
// 	return transfer(&reg, 1, &value, 1);
// 		//read_register(0x01, myValue) would read register 0x01 and store the result in myValue.
// }

// // Write: Sends 2 bytes (address + data), reads nothing
// // Read: Sends 1 byte (address), reads 1 byte (the register's value)

// // Both return an int status code (likely 0 for success, negative for errors)
// // from the underlying transfer() function.


// // ---------------------------------------------------------------------------------

// // Initializes the DRV2605L chip over I2C communication and configures it for operation
// // initializing hardware peripherals
// int DRV2605L::init()
// {
// 	int ret = I2C::init(); // Initializes the I2C bus communication

// 	if (ret != OK) {
// 		PX4_ERR("I2C init failed");
// 		return ret;
// 	}

// 	// Probe the device
// 	ret = probe();  // Verifies the DRV2605L chip is physically connected and responding

// 	if (ret != OK) {
// 		return ret;
// 	}

// 	// take out of standby mode
// 	// writes to MODE register
// 	ret = write_register(DRV2605L_REG_MODE, DRV2605L_MODE_INTTRIG);

// 	if (ret != OK) {
// 		PX4_ERR("Failed to set mode");
// 		return ret;
// 	}

// 	px4_usleep(1000); //1ms sleep afterward gives the chip time to wake up

// 	// select motor library
// 	// set to ERM library (change to DRV2605L_LIB_LRA if using LRA motor)
// 	ret = write_register(DRV2605L_REG_LIBRARY, DRV2605L_LIB_ERM);

// 	if (ret != OK) {
// 		PX4_ERR("Failed to set library");
// 		return ret;
// 	}

// 	// Configure feedback control (optional, for ERM)
// 	ret = write_register(DRV2605L_REG_FEEDBACK, 0x36);

// 	if (ret != OK) {
// 		PX4_WARN("Failed to set feedback control");
// 	}

// 	_initialized = true; // Sets an internal flag that the driver is ready to use
// 	PX4_INFO("DRV2605L initialized successfully");
// 	PX4_INFO("Haptic will trigger when yaw is between %.0f-%.0f degrees", (double)YAW_MIN_DEG, (double)YAW_MAX_DEG);

// 	return OK;
// }

// //  function controls a DRV2605L haptic motor driver
// int DRV2605L::trigger_effect(uint8_t effect) // plays a single vibration effect on the haptic motor.
// {
// 	// Set waveform sequence (effect to play)
// 	int ret = write_register(DRV2605L_REG_WAVESEQ1, effect);

// 	if (ret != OK) {
// 		return ret;
// 	}

// 	// End waveform sequence
// 	ret = write_register(DRV2605L_REG_WAVESEQ1 + 1, 0);

// 	if (ret != OK) {
// 		return ret;
// 	}

// 	// Trigger the GO command
// 	// starts playback
// 	ret = write_register(DRV2605L_REG_GO, 1); // Writing 1 to the GO register triggers the haptic motor

// 	return ret;

// 	// trigger_effect(47);  // Play effect #47 (sharp click)
// }

// float DRV2605L::get_yaw_from_quaternion(const vehicle_attitude_s &att)
// {
// 	// Convert quaternion to Euler angles using PX4 matrix library
// 	matrix::Quatf q(att.q);
// 	matrix::Eulerf euler(q);

// 	// euler.psi() returns yaw in radians, convert to degrees
// 	// Range: -180 to +180 degrees
// 	float yaw_deg = math::degrees(euler.psi());

// 	// Normalize to 0-360 range
// 	if (yaw_deg < 0.0f) {
// 		yaw_deg += 360.0f;
// 	}

// 	return yaw_deg;
// }

// bool DRV2605L::is_yaw_in_range(float yaw_deg)
// {
// 	// Check if yaw is within the target range (135-225 degrees)
// 	return (yaw_deg >= YAW_MIN_DEG && yaw_deg <= YAW_MAX_DEG);
// }

// // main task loop function
// // continuous haptic response
// void DRV2605L::run()
// {
// 	// Initialize the device
// 	if (init() != OK) {   //Calls init() to set up the haptic driver hardware
// 		PX4_ERR("Initialization failed");
// 		return;
// 	}

// 	PX4_INFO("Haptic task started - monitoring yaw angle");

// 	// Main loop
// 	while (!should_exit()) {  //should_exit() - Checks if the task should terminate (set by external commands or system shutdown)

// 		// // Trigger continuous haptic feedback with a strong click effect
// 		// int ret = trigger_effect(DRV2605L_EFFECT_STRONG_CLICK);

// 		// if (ret != OK) {
// 		// 	PX4_ERR("Failed to trigger effect");
// 		// }

// 		// Check for new vehicle attitude data and print yaw
// 		vehicle_attitude_s attitude; //Declares a variable to hold the vehicle's attitude (orientation) data;
// 					     // The _s suffix indicates this is a struct type in PX4 conventions
// 		if (_vehicle_attitude_sub.update(&attitude)) { // _vehicle_attitude_sub:uORB subscription object
// 							       // .update(&attitude) attempts to fetch the latest attitude data and store it in the attitude variable
// 			// Get yaw angle from quaternion
// 			// Calls a helper function to extract the yaw angle from the attitude data
// 			// PX4 stores orientation as quaternions (4D representation), so this converts it to a simple yaw angle in degrees
// 			// Yaw represents rotation around the vertical axis (like a compass heading)
// 			float yaw_deg = get_yaw_from_quaternion(attitude);


// 			// Check if yaw is in target range
// 			bool in_range = is_yaw_in_range(yaw_deg);

// 			// Print yaw continuously with haptic status
// 			PX4_INFO("Yaw: %.2f deg | Haptic: %s", (double)yaw_deg, in_range ? "ACTIVE" : "INACTIVE");

// 			// Trigger haptic only if in range
// 			if (in_range) {
// 				int ret = trigger_effect(DRV2605L_EFFECT_STRONG_CLICK);

// 				if (ret != OK) {
// 					PX4_ERR("Failed to trigger effect");
// 				}
// 			}
// 		}

// 		// Sleep for the loop interval
// 		px4_usleep(_loop_interval_us);
// 	}

// 	PX4_INFO("Haptic task exiting");
// }

// // prints diagnostic status information for the DRV2605L
// // typical diagnostic/debug method used to verify the driver
// // is configured correctly and communicating with the hardware.
// int DRV2605L::print_status()
// {	// PX4_INFO is a logging macro from the PX4 autopilot framework
// 	PX4_INFO("Running: %s", _initialized ? "YES" : "NO"); //prints if device intialized
// 	PX4_INFO("Loop interval: %" PRIu32 " us", _loop_interval_us); //prints control loop timing
// 	//PRIu32 is a format specifier for printing 32-bit unsigned integers (portable across platforms)
// 	PX4_INFO("I2C bus: %d, address: 0x%02X", get_device_bus(), get_device_address()); // print I2C bus info
// 	PX4_INFO("Yaw trigger range: %.0f-%.0f degrees", (double)YAW_MIN_DEG, (double)YAW_MAX_DEG);

// 	// Get and display current yaw if available
// 	vehicle_attitude_s attitude;
// 	if (_vehicle_attitude_sub.copy(&attitude)) {
// 		float yaw_deg = get_yaw_from_quaternion(attitude);
// 		PX4_INFO("Current yaw: %.1f degrees", (double)yaw_deg);
// 	}
// 	return OK;
// }

// // This is a factory method that creates and returns a new instance of the DRV2605L class
// // Creates a DRV2605L object with configurable I2C bus and address parameters, parsing command-line arguments.
// DRV2605L *DRV2605L::instantiate(int argc, char *argv[])
// {
// 	int bus = 1;  // Default to external I2C bus 1 (GPS port)
// 	int address = DRV2605L_ADDR;  // Default I2C address (likely 0x5A)

// 	// state variables needed for px4_getopt() function to parse command-line arguments
// 	int myoptind = 1; //Option Index - tracks the current position in the argv array
// 	int ch;  	  //Current Character - stores the option character returned by px4_getopt()
// 	const char *myoptarg = nullptr; //Option Argument - pointer to the string value of the current option's argument

// 	while ((ch = px4_getopt(argc, argv, "b:a:", &myoptind, &myoptarg)) != EOF) {
// 		//Uses px4_getopt (PX4 autopilot's version of getopt) to parse command-line options
// 		//"b:a:" means it accepts two options, both requiring arguments (: indicates required argument)
// 		// -b : specify bus number, -a : specify I2C address
// 		switch (ch) {
// 		case 'b':
// 			bus = atoi(myoptarg); // Convert string to integer
// 			break;

// 		case 'a':
// 			address = strtol(myoptarg, nullptr, 16); // Convert hex string to integer
// 			break;

// 		default:
// 			return nullptr; // Invalid option = abort instantiation
// 		}
// 		// -b uses atoi() for decimal conversion
// 		// -a uses strtol(..., 16) for hexadecimal conversion (since I2C addresses are typically in hex)
// 	}

// 	return new DRV2605L(bus, address); //Creates the object on the heap with the specified/default parameters.
// }

// // task spawning function
// // Creates and launches a new task (thread) to run the DRV2605L haptic feedback device driver
// // This is a typical PX4 driver initialization pattern that safely creates a driver instance, launches it in its
// // own thread, and handles errors at each step with proper cleanup
// int DRV2605L::task_spawn(int argc, char *argv[])
// {
// 	DRV2605L *instance = instantiate(argc, argv); // creates new instance

// 	// Error handling: if instantiation fails (memory allocation issue, invalid parameters, etc.),
// 	// logs an error and returns failure
// 	if (instance == nullptr) {
// 		PX4_ERR("Failed to allocate DRV2605L");
// 		return PX4_ERROR;
// 	}

// 	_object.store(instance); // Stores the instance pointer in an atomic variable _object
// 	// This allows other parts of the code to access this driver instance safely across threads
// 	_task_id = px4_task_spawn_cmd("drv2605l_haptic",
// 				      SCHED_DEFAULT,
// 				      SCHED_PRIORITY_DEFAULT,
// 				      2000,
// 				      (px4_main_t)&run_trampoline,
// 				      (char *const *)argv);

// 	// Spawns a new task with:

// 	// Name: "drv2605l_haptic"
// 	// Scheduler: Default scheduling policy
// 	// Priority: Default priority
// 	// Stack size: 2000 bytes
// 	// Entry point: run_trampoline function (typically calls the instance's run() method)
// 	// Arguments: Original command-line arguments

// 	if (_task_id < 0) {
// 		_object.store(nullptr);
// 		delete instance;
// 		PX4_ERR("Task start failed");
// 		return PX4_ERROR;
// 	}
// 	// If task creation fails, cleans up by:

// 	// Clearing the stored instance pointer
// 	// Deleting the allocated instance
// 	// Logging an error
// 	// Returning failure

// 	return PX4_OK;
// }

// // code implements the command-line interface for a PX4 drone firmware driver.
// // a handler for unrecognized subcommands. When someone types a command that
// // the driver doesn't understand (anything other than start, stop, status),
// // this function catches it and calls print_usage() with an error message
// int DRV2605L::custom_command(int argc, char *argv[])
// {
// 	return print_usage("unknown command");
// }

// int DRV2605L::print_usage(const char *reason)
// {
// 	if (reason) {
// 		PX4_WARN("%s\n", reason); // shows the error
// 	}

// 	// displays module description
// 	// R"DESCR_STR(...)DESCR_STR" is a raw string literal containing
// 	// documentation about what the driver does and usage examples
// 	PRINT_MODULE_DESCRIPTION(
// 		R"DESCR_STR(
// ### Description
// DRV2605L Haptic Feedback Driver for ARK FPV

// This driver provides continuous haptic feedback using the DRV2605L haptic driver chip.
// Connected via I2C on the remapped GPS port. Runs as a dedicated task.

// ### Examples
// Start the driver on I2C bus 1 (GPS port):
// $ drv2605l_haptic start -b 1

// Start on custom bus and address:
// $ drv2605l_haptic start -b 2 -a 0x5A

// Stop the driver:
// $ drv2605l_haptic stop

// Check status:
// $ drv2605l_haptic status
// )DESCR_STR");

// 	PRINT_MODULE_USAGE_NAME("drv2605l_haptic", "driver");  //sets command name
// 	PRINT_MODULE_USAGE_COMMAND("start"); //shows the start subcommand
// 	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 0, 10, "I2C bus", true); // I2C bus number (default: 1, range: 0-10)
// 	PRINT_MODULE_USAGE_PARAM_INT('a', 0x5A, 0, 0xFF, "I2C address (hex)", true); // I2C address in hex (default: 0x5A, range: 0-0xFF)
// 	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();  //Adds standard commands like stop, status

// 	return PX4_OK;
// }

// extern "C" __EXPORT int drv2605l_haptic_main(int argc, char *argv[])
// // extern "C" makes it compatible with C code (important for firmware integration)
// //__EXPORT makes it visible to the module loading system
// // It forwards all arguments to the DRV2605L::main() static method (likely inherited from a base class that handles command parsing)
// {
// 	return DRV2605L::main(argc, argv);
// }

// // In practice: When you type drv2605l_haptic xyz in the PX4 console, it triggers this flow:
// // drv2605l_haptic_main() → DRV2605L::main() → custom_command() → print_usage("unknown command") → help text displayed.
