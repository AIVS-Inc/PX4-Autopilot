# Acoustic Vector Sensor Lite event

uint32 device_id		# unique device ID for the sensor that does not change between power cycles
uint64 time_utc_usec            # Timestamp (microseconds, UTC)
uint64 timestamp		# time since system start (microseconds)
uint32 timestamp_sample         # AVS sensor sample index since system SYNC</field>
float32 azimuth_deg		# azimuth in degrees
float32 elevation_deg		# elevation in degrees
float32 active_intensity	# active intensity level re: 1 pW
float32 q_factor		# Q-factor of the histogram peak for this detected source
uint16  histogram_count		# number of hits for this azim/elev histogram bin

# Local position NED
float32 north		# in meters NED
float32 east		# in meters NED
float32 down		# in meters NED


# Vehicle Attitude
float32 yaw
float32 pitch
float32 roll
