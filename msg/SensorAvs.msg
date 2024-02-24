# Acoustic Vector Sensor event

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# AVS sensor sample index since system SYNC
uint64 time_utc_usec            # Timestamp (microseconds, UTC)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
float32 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters
float32 eph			# GPS horizontal position accuracy (metres)
float32 epv			# GPS vertical position accuracy (metres)

float32 pitch			# AVS sensor pitch, radians
float32 roll 			# AVS sensor roll, radians
float32 yaw			# AVS sensor yaw, radians

float32 spl 			# sound pressure level re: 20 uPa
float32 sil 			# sound intensity level re: 1 pW
float32 azimuth_deg		# azimuth in degrees
float32 elevation_deg		# elevation in degrees
float32 active_intensity	# active intensity level re: 1 pW
float32 bkgrnd_sil		# background SIL used for thresholding
uint32  source_index		# index of detected source_index

