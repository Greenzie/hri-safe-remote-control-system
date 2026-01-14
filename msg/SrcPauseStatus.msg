int32 src_inactivity_pause_time     # Time before the SRC goes into pause mode due to inactivity. Time = this value + 1 minutes Valid Values: 0-10
bool src_auto_off_enabled           # SRC Auto Off after 2 minutes past an inactivity timeout
bool src_orientation_pause_enabled  # SRC Pause when remote detects orientation fault
bool src_free_fall_pause_enabled    # SRC pause when remote detects free fall
bool src_inactivity_pause_enabled   # SRC pause when inactivity timeout occurs