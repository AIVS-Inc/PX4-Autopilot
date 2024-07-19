# Acoustic Vector Sensor Mel Intensity

uint64 timestamp		# time since system start (microseconds)
uint32 timestamp_sample		# AVS sensor sample index since system SYNC
uint64 time_utc_usec            # Timestamp (microseconds, UTC)
uint32 device_id		# unique device ID for the sensor that does not change between power cycles

float32[15] active_intensity	# MEL Intensity spectrum
