# =========================
# TOMO OUTPUT STATE
# =========================

# ---- SOURCE IDENTIFICATION ----
uint8 source
uint8 SOURCE_PS4=1
uint8 SOURCE_WEB=2
uint8 SOURCE_AUTO=3

# ---- SYSTEM STATE ----
bool emergency
bool armed_state
bool power_state
bool light_state

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
