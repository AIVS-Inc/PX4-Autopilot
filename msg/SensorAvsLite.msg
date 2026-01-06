# Acoustic Vector Sensor Lite event

uint64 timestamp		# time since system start (microseconds)
uint32 timestamp_sample         # AVS sensor sample index since system SYNC</field>
float32 azimuth_deg		# azimuth in degrees
float32 elevation_deg		# elevation in degrees
float32 active_intensity	# active intensity level re: 1 pW
float32 q_factor		# Q-factor of the histogram peak for this detected source
uint16  histogram_count		# number of hits for this azim/elev histogram bin


