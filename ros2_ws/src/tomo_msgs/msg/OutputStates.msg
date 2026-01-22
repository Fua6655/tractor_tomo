# =========================
# TOMO OUTPUT STATE
# =========================

# ---- TARGET ESP ----
string esp_ip           # e.g. "192.168.1.50"
uint16 esp_port         # optional (UDP/TCP)

# ---- SYSTEM STATE ----
bool emergency
bool armed
bool power_mode
bool light_mode

# ---- EVENTS ----
bool engine_start
bool clutch_down
bool high_speed
bool move_allowed

# ---- LIGHTS ----
bool front_position
bool front_short
bool front_long
bool back_light
bool left_blink
bool right_blink

# ---- MOTION ----
float32 linear
float32 angular

# ---- HARDWARE MAPPING ----
# Index = logical output, value = GPIO pin on ESP
uint8[] gpio_map

# Example:
# gpio_map[LIGHT_FP] = 12
# gpio_map[ENGINE_START] = 5

# ---- METADATA ----
string active_source    # "PS4", "WEB", "AUTO"
builtin_interfaces/Time stamp
