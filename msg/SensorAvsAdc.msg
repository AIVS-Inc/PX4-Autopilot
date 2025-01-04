# Acoustic Vector Sensor ADC time data at the currently selected sample rate

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# AVS sensor sample index since system SYNC
uint64 time_utc_usec            # Timestamp (microseconds, UTC)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles
uint32 transfer_id		# incrementing counter to ensure continuity of sampled ADC frames

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
float32 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters
float32 eph			# GPS horizontal position accuracy (metres)
float32 epv			# GPS vertical position accuracy (metres)

float32 pitch			# AVS sensor pitch, radians
float32 roll 			# AVS sensor roll, radians
float32 yaw			# AVS sensor yaw, radians

int32[384] adc_frame		# 4 real ADC columns x 96 samples, segmented in rows (not interleaved)
