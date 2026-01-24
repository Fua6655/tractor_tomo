# =========================
# EMERGENCY OVERRIDE
# =========================

bool active                 # true = emergency ON
string reason               # optional ("web", "hw", "timeout")

uint8 level
uint8 LEVEL_SOFT=0
uint8 LEVEL_HARD=1

builtin_interfaces/Time stamp
