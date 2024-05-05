# Acoustic Vector Sensor peak control

uint64 timestamp			# time since system start (microseconds)

bool fft_enable
uint8 node_top
uint8 node_bot
uint16 fft_param_id

uint8 max_harmonics
uint8 num_peaks
float32 min_peak_height			# dB
float32 min_peak_change			# dB
float32 min_blanking			# degrees
