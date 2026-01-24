# =========================
# TOMO OUTPUT STATE
# =========================

# ---- METADATA ----
string active_source    # "PS4", "WEB", "AUTO"

# ---- SYSTEM STATE ----
bool emergency
bool armed
bool power_mode
bool light_mode

# ---- EVENTS ----
bool engine_start
bool engine_stop
bool clutch_active
bool brake_active
bool move_allowed

# ---- LIGHTS ----
bool front_position
bool front_short
bool front_long
bool back_position
bool left_blink
bool right_blink


builtin_interfaces/Time stamp
