# =========================
# CONTROL EVENT (INPUT)
# =========================

# ---- SOURCE IDENTIFICATION ----
uint8 source
uint8 SOURCE_PS4=1
uint8 SOURCE_WEB=2
uint8 SOURCE_AUTO=3

# ---- EVENT CATEGORY ----
uint8 category
uint8 CATEGORY_STATE=1        # armed, power, light
uint8 CATEGORY_EVENT=2        # engine_start, clutch, move_allowed
uint8 CATEGORY_LIGHT=3        # lights, blinkers
uint8 CATEGORY_SYSTEM=4       # source switch, reset, etc

# ---- EVENT TYPE (MEANING DEPENDS ON CATEGORY) ----
uint8 type

# STATE TYPES
uint8 STATE_ARMED=1
uint8 STATE_POWER=2
uint8 STATE_LIGHT=3

# EVENT TYPES
uint8 ENGINE_START=10
uint8 ENGINE_STOP=11
uint8 CLUTCH_ACTIVE=12
uint8 BRAKE_ACTIVE=13
uint8 MOVE_ALLOWED=14

# LIGHT TYPES
uint8 FRONT_POSITION=20
uint8 FRONT_SHORT=21
uint8 FRONT_LONG=22
uint8 BACK_POSITION=23
uint8 LEFT_BLINK=24
uint8 RIGHT_BLINK=25

# SYSTEM TYPES
uint8 SYS_FORCE_SOURCE=40
uint8 SYS_RESET=41

# ---- VALUE ----
# Meaning:
# - toggle → 0/1
# - force → 0/1
# - analog (future) → scaled int
int32 value

# ---- TIMESTAMP ----
builtin_interfaces/Time stamp
