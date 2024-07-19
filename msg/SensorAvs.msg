# Acoustic Vector Sensor event

uint64 timestamp		# time since system start (microseconds)
uint32 timestamp_sample		# AVS sensor sample index since system SYNC
uint64 time_utc_usec            # Timestamp (microseconds, UTC)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles

float32 spl 			# sound pressure level re: 20 uPa
float32 sil 			# sound intensity level re: 1 pW
float32 azimuth_deg		# azimuth in degrees
float32 elevation_deg		# elevation in degrees
float32 active_intensity	# active intensity level re: 1 pW
float32 bkgrnd_sil		# background SIL used for thresholding
uint32  source_index		# index of detected source_index

