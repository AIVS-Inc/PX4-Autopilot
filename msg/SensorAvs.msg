# Acoustic Vector Sensor event

uint8  FFT_MEL_BANDS = 16	# currently support on 16 Mel bands, from 60 Hz to 1.9 kHz
uint64 timestamp		# time since system start (microseconds)
uint32 timestamp_sample		# AVS sensor sample index since system SYNC
uint64 time_utc_usec            # Timestamp (microseconds, UTC)
uint32 device_id		# unique device ID for the sensor that does not change between power cycles
float32 azimuth_deg		# azimuth in degrees
float32 elevation_deg		# elevation in degrees
float32 active_intensity	# active intensity level re: 1 pW
float32 q_factor		# Q-factor of the histogram peak for this detected source
uint16  source_index		# index of detected source_index
uint16  histogram_count		# number of hits for this azim/elev histogram bin
float32[16] mel_intensity	# MEL Intensity spectrum

