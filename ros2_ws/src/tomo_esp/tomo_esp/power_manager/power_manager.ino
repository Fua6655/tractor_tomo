#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

// =====================================================
// ================= WIFI / UDP CONFIG =================
// =====================================================
static char WIFI_SSID[] = "Villa_Milano";
static char WIFI_PASS[] = "10203040";

const uint16_t UDP_LISTEN_PORT = 5005;
const uint16_t UDP_DEFAULT_TX_PORT = 5006;

const unsigned long STATUS_PERIOD_MS = 500;
const unsigned long PING_PERIOD_MS = 1000;
const unsigned long SHUTDOWN_MSG_PERIOD_MS = 1000;
const unsigned long WIFI_RETRY_PERIOD_MS = 3000;

WiFiUDP udp;
IPAddress linux_ip(0, 0, 0, 0);
uint16_t linux_port = UDP_DEFAULT_TX_PORT;
bool udp_started = false;

// =====================================================
// ================= GPIO ==============================
// =====================================================
// Inputs (active-low with INPUT_PULLUP)
const int POWER_BUTTON_PIN = 32;
const int ESTOP_PIN = 33;

// Status LEDs
const int LED1_PIN = 25;
const int LED2_PIN = 26;

// Relay outputs (boot-safe pins)
const int RELAY_CH1_PS_ON = 18;
const int RELAY_CH2_MICROROS_PWR = 19;
const int RELAY_CH3_FAN = 21;
const int RELAY_CH4_POWER_OUTPUTS = 22;  // uses former CH4 channel

const bool INPUT_ACTIVE_LOW = true;
const bool RELAY_ACTIVE_HIGH = true;

// =====================================================
// ================= TIMERS ============================
// =====================================================
const unsigned long DEBOUNCE_MS = 30;
const unsigned long LONG_PRESS_MS = 3000;
const unsigned long POWER_ON_SETTLE_MS = 1500;
const unsigned long RPI_BOOT_DISCOVERY_TIMEOUT_MS = 30000;
const unsigned long ROS_READY_TIMEOUT_MS = 60000;
const unsigned long AGENT_READY_TIMEOUT_MS = 45000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;
const unsigned long SAFE_SHUTDOWN_TIMEOUT_MS = 90000;

const float FAN_ON_TEMP_C = 55.0f;
const float FAN_OFF_TEMP_C = 50.0f;

// =====================================================
// ================= FSM ===============================
// =====================================================
enum PMState {
  OFF = 0,
  POWERING_ON,
  WAITING_RPI_BOOT,
  WAITING_ROS_READY,
  WAITING_AGENT_READY,
  RUNNING,
  SHUTDOWN_REQUESTED,
  WAITING_SAFE_SHUTDOWN,
  ERROR_STATE,
};

PMState state = OFF;
unsigned long state_enter_ms = 0;

// =====================================================
// ================= INPUT STATE =======================
// =====================================================
bool power_button_pressed = false;
bool estop_active = false;

bool last_button_raw = false;
bool last_estop_raw = false;
unsigned long last_button_change_ms = 0;
unsigned long last_estop_change_ms = 0;
unsigned long button_press_start_ms = 0;
bool shutdown_long_press_latched = false;

// =====================================================
// ================= RPI STATE =========================
// =====================================================
bool rpi_seen = false;
bool ros_ready = false;
bool agent_ready = false;
bool safe_to_power_off = false;
bool shutdown_agent_stopped = false;
bool shutdown_ros_stopped = false;
bool shutdown_last_message = false;

float rpi_load = 0.0f;
float rpi_temp = 0.0f;

unsigned long last_heartbeat_ms = 0;
unsigned long last_status_sent_ms = 0;
unsigned long last_ping_sent_ms = 0;
unsigned long last_shutdown_sent_ms = 0;
unsigned long last_wifi_retry_ms = 0;

// =====================================================
// ================= EVENT LOG =========================
// =====================================================
const int LOG_CAPACITY = 48;
char log_buffer[LOG_CAPACITY][96];
int log_head = 0;
int log_count = 0;

void add_log(const char *msg) {
  char line[96];
  snprintf(line, sizeof(line), "%lu %s", millis(), msg);
  strncpy(log_buffer[log_head], line, sizeof(log_buffer[log_head]) - 1);
  log_buffer[log_head][sizeof(log_buffer[log_head]) - 1] = '\0';
  log_head = (log_head + 1) % LOG_CAPACITY;
  if (log_count < LOG_CAPACITY) {
    log_count++;
  }
  Serial.println(line);
}

const char *state_name(PMState s) {
  switch (s) {
    case OFF: return "OFF";
    case POWERING_ON: return "POWERING_ON";
    case WAITING_RPI_BOOT: return "WAITING_RPI_BOOT";
    case WAITING_ROS_READY: return "WAITING_ROS_READY";
    case WAITING_AGENT_READY: return "WAITING_AGENT_READY";
    case RUNNING: return "RUNNING";
    case SHUTDOWN_REQUESTED: return "SHUTDOWN_REQUESTED";
    case WAITING_SAFE_SHUTDOWN: return "WAITING_SAFE_SHUTDOWN";
    case ERROR_STATE: return "ERROR_STATE";
    default: return "UNKNOWN";
  }
}

void set_state(PMState next, const char *reason) {
  if (state == next) {
    return;
  }

  state = next;
  state_enter_ms = millis();

  char msg[128];
  snprintf(msg, sizeof(msg), "STATE %s (%s)", state_name(state), reason);
  add_log(msg);
}

// =====================================================
// ================= LOW LEVEL HELPERS =================
// =====================================================
bool read_input_active(int pin) {
  int raw = digitalRead(pin);
  return INPUT_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
}

void set_relay(int pin, bool on) {
  digitalWrite(pin, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

void set_all_relays_off() {
  set_relay(RELAY_CH1_PS_ON, false);
  set_relay(RELAY_CH2_MICROROS_PWR, false);
  set_relay(RELAY_CH3_FAN, false);
  set_relay(RELAY_CH4_POWER_OUTPUTS, false);
}

void set_leds(bool led1, bool led2) {
  digitalWrite(LED1_PIN, led1 ? HIGH : LOW);
  digitalWrite(LED2_PIN, led2 ? HIGH : LOW);
}

void fan_control_from_temp() {
  if (rpi_temp >= FAN_ON_TEMP_C) {
    set_relay(RELAY_CH3_FAN, true);
  } else if (rpi_temp <= FAN_OFF_TEMP_C) {
    set_relay(RELAY_CH3_FAN, false);
  }
}

bool wifi_ready() {
  return WiFi.status() == WL_CONNECTED;
}

bool udp_ready() {
  return wifi_ready() && udp_started;
}

void ensure_wifi_udp() {
  unsigned long now = millis();

  if (!wifi_ready() && (now - last_wifi_retry_ms >= WIFI_RETRY_PERIOD_MS)) {
    last_wifi_retry_ms = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    add_log("WiFi retry");
  }

  if (wifi_ready() && !udp_started) {
    udp.begin(UDP_LISTEN_PORT);
    udp_started = true;
    add_log("UDP started");
  }
}

void send_udp_text(const char *text) {
  if (!udp_ready()) {
    return;
  }
  if (linux_ip == IPAddress(0, 0, 0, 0)) {
    return;
  }

  udp.beginPacket(linux_ip, linux_port);
  udp.write((const uint8_t *)text, strlen(text));
  udp.endPacket();
}

void send_status(bool force = false) {
  unsigned long now = millis();
  if (!force && (now - last_status_sent_ms < STATUS_PERIOD_MS)) {
    return;
  }
  last_status_sent_ms = now;

  if (!udp_ready() || linux_ip == IPAddress(0, 0, 0, 0)) {
    return;
  }

  char msg[320];
  snprintf(
      msg,
      sizeof(msg),
      "{\"src\":\"ESP_PM\",\"state\":\"%s\",\"btn\":%d,\"estop\":%d,"
      "\"ros\":%d,\"agent\":%d,\"load\":%.2f,\"temp\":%.2f,\"fan\":%d,"
      "\"ch1\":%d,\"ch2\":%d,\"esp_power\":%d,\"uptime_ms\":%lu}",
      state_name(state),
      power_button_pressed ? 1 : 0,
      estop_active ? 1 : 0,
      ros_ready ? 1 : 0,
      agent_ready ? 1 : 0,
      rpi_load,
      rpi_temp,
      digitalRead(RELAY_CH3_FAN) == HIGH ? 1 : 0,
      digitalRead(RELAY_CH1_PS_ON) == HIGH ? 1 : 0,
      digitalRead(RELAY_CH2_MICROROS_PWR) == HIGH ? 1 : 0,
      digitalRead(RELAY_CH4_POWER_OUTPUTS) == HIGH ? 1 : 0,
      now);
  send_udp_text(msg);
}

void send_ping_periodic() {
  if (!udp_ready()) {
    return;
  }
  unsigned long now = millis();
  if (now - last_ping_sent_ms < PING_PERIOD_MS) {
    return;
  }
  last_ping_sent_ms = now;
  send_udp_text("PING");
}

void send_shutdown_periodic() {
  if (!udp_ready()) {
    return;
  }
  unsigned long now = millis();
  if (now - last_shutdown_sent_ms < SHUTDOWN_MSG_PERIOD_MS) {
    return;
  }
  last_shutdown_sent_ms = now;
  send_udp_text("SHUTDOWN");
}

void send_ack(const char *ack_msg) {
  send_udp_text(ack_msg);
}

bool json_bool(const String &p, const String &key, bool &out) {
  int k = p.indexOf(key);
  if (k < 0) return false;
  int colon = p.indexOf(':', k + key.length());
  if (colon < 0) return false;
  String rem = p.substring(colon + 1);
  rem.trim();
  rem.toLowerCase();
  if (rem.startsWith("true") || rem.startsWith("1")) {
    out = true;
    return true;
  }
  if (rem.startsWith("false") || rem.startsWith("0")) {
    out = false;
    return true;
  }
  return false;
}

bool json_float(const String &p, const String &key, float &out) {
  int k = p.indexOf(key);
  if (k < 0) return false;
  int colon = p.indexOf(':', k + key.length());
  if (colon < 0) return false;
  int end = p.indexOf(',', colon + 1);
  if (end < 0) end = p.indexOf('}', colon + 1);
  if (end < 0) end = p.length();
  String token = p.substring(colon + 1, end);
  token.trim();
  out = token.toFloat();
  return true;
}

void parse_heartbeat_json(const String &payload) {
  bool b = false;
  float f = 0.0f;

  if (json_bool(payload, "\"ros\"", b)) {
    ros_ready = b;
  }
  if (json_bool(payload, "\"agent\"", b)) {
    agent_ready = b;
  }
  if (json_float(payload, "\"load\"", f)) {
    rpi_load = f;
  }
  if (json_float(payload, "\"temp\"", f)) {
    rpi_temp = f;
  }
}

void handle_udp_packet(String payload) {
  payload.trim();
  if (payload.length() == 0) {
    return;
  }

  last_heartbeat_ms = millis();
  rpi_seen = true;

  String upper = payload;
  upper.toUpperCase();

  if (upper == "ROS_READY") {
    ros_ready = true;
    add_log("ROS_READY");
    return;
  }
  if (upper == "AGENT_READY") {
    agent_ready = true;
    add_log("AGENT_READY");
    return;
  }
  if (upper == "SAFE_TO_POWER_OFF") {
    shutdown_last_message = true;
    safe_to_power_off = true;
    add_log("SAFE_TO_POWER_OFF");
    send_ack("ACK_SAFE_TO_POWER_OFF");
    return;
  }
  if (upper == "AGENT_STOPPED") {
    agent_ready = false;
    shutdown_agent_stopped = true;
    add_log("AGENT_STOPPED");
    send_ack("ACK_AGENT_STOPPED");
    return;
  }
  if (upper == "ROS_STOPPED") {
    ros_ready = false;
    shutdown_ros_stopped = true;
    add_log("ROS_STOPPED");
    send_ack("ACK_ROS_STOPPED");
    return;
  }
  if (upper == "LAST_MESSAGE") {
    shutdown_last_message = true;
    add_log("LAST_MESSAGE");
    send_ack("ACK_LAST_MESSAGE");
    return;
  }
  if (upper == "HEARTBEAT") {
    return;
  }
  if (payload.startsWith("{")) {
    parse_heartbeat_json(payload);
    return;
  }
}

void service_udp_rx() {
  if (!udp_ready()) {
    return;
  }

  int packet_size = udp.parsePacket();
  if (packet_size <= 0) {
    return;
  }

  char buf[384];
  int len = udp.read(buf, sizeof(buf) - 1);
  if (len <= 0) {
    return;
  }
  buf[len] = '\0';

  linux_ip = udp.remoteIP();
  linux_port = udp.remotePort() > 0 ? udp.remotePort() : UDP_DEFAULT_TX_PORT;

  handle_udp_packet(String(buf));
}

// =====================================================
// ================= INPUT HANDLING ====================
// =====================================================
void service_inputs() {
  unsigned long now = millis();
  bool button_raw = read_input_active(POWER_BUTTON_PIN);
  bool estop_raw = read_input_active(ESTOP_PIN);

  if (button_raw != last_button_raw) {
    last_button_raw = button_raw;
    last_button_change_ms = now;
  }
  if (estop_raw != last_estop_raw) {
    last_estop_raw = estop_raw;
    last_estop_change_ms = now;
  }

  if (now - last_button_change_ms > DEBOUNCE_MS) {
    bool prev = power_button_pressed;
    power_button_pressed = button_raw;
    if (!prev && power_button_pressed) {
      button_press_start_ms = now;
      shutdown_long_press_latched = false;
    }
    if (prev && !power_button_pressed) {
      button_press_start_ms = 0;
      shutdown_long_press_latched = false;
    }
  }

  if (now - last_estop_change_ms > DEBOUNCE_MS) {
    estop_active = estop_raw;
  }
}

bool short_press_event() {
  static bool last_button = false;
  bool event = false;
  if (!last_button && power_button_pressed) {
    event = true;
  }
  last_button = power_button_pressed;
  return event;
}

bool long_press_event() {
  if (!power_button_pressed || shutdown_long_press_latched || button_press_start_ms == 0) {
    return false;
  }
  if (millis() - button_press_start_ms >= LONG_PRESS_MS) {
    shutdown_long_press_latched = true;
    return true;
  }
  return false;
}

// =====================================================
// ================= LED PATTERNS ======================
// =====================================================
void apply_led_pattern() {
  unsigned long now = millis();
  bool slow = ((now / 600) % 2) == 0;
  bool fast = ((now / 150) % 2) == 0;

  if (state == OFF) {
    set_leds(false, false);
    return;
  }

  if (estop_active) {
    set_leds(false, fast);
    return;
  }

  if (state == RUNNING) {
    set_leds(true, true);
    return;
  }

  if (state == WAITING_AGENT_READY) {
    set_leds(true, slow);
    return;
  }

  if (state == WAITING_SAFE_SHUTDOWN || state == SHUTDOWN_REQUESTED) {
    if (shutdown_last_message) {
      set_leds(false, false);
    } else if (shutdown_ros_stopped) {
      set_leds(slow, false);
    } else if (shutdown_agent_stopped) {
      set_leds(true, false);
    } else {
      set_leds(true, true);
    }
    return;
  }

  if (state == ERROR_STATE) {
    set_leds(fast, fast);
    return;
  }

  set_leds(slow, false);
}

// =====================================================
// ================= POWER ACTIONS =====================
// =====================================================
void begin_power_on() {
  ros_ready = false;
  agent_ready = false;
  safe_to_power_off = false;
  shutdown_agent_stopped = false;
  shutdown_ros_stopped = false;
  shutdown_last_message = false;
  rpi_seen = false;

  set_relay(RELAY_CH1_PS_ON, true);        // CH1: ATX PS_ON
  set_relay(RELAY_CH2_MICROROS_PWR, false); // CH2 delayed until AGENT_READY
  set_relay(RELAY_CH3_FAN, false);
  set_relay(RELAY_CH4_POWER_OUTPUTS, true);        // power micro-ROS ESP rail on startup

  set_state(POWERING_ON, "button press");
}

void hard_power_off() {
  set_relay(RELAY_CH4_POWER_OUTPUTS, false);
  set_relay(RELAY_CH2_MICROROS_PWR, false);
  delay(150);
  set_relay(RELAY_CH1_PS_ON, false);
  set_relay(RELAY_CH3_FAN, false);
  ros_ready = false;
  agent_ready = false;
  safe_to_power_off = false;
  shutdown_agent_stopped = false;
  shutdown_ros_stopped = false;
  shutdown_last_message = false;
  rpi_seen = false;
  set_state(OFF, "power rails off");
}

void request_shutdown() {
  safe_to_power_off = false;
  shutdown_agent_stopped = false;
  shutdown_ros_stopped = false;
  shutdown_last_message = false;
  set_state(SHUTDOWN_REQUESTED, "long press");
}

// =====================================================
// ================= FSM UPDATE ========================
// =====================================================
void check_heartbeat_timeout() {
  if (state == OFF) {
    return;
  }
  if (!rpi_seen) {
    return;
  }
  if (millis() - last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS) {
    if (state == RUNNING || state == WAITING_AGENT_READY || state == WAITING_ROS_READY) {
      set_state(ERROR_STATE, "heartbeat timeout");
    }
  }
}

void update_fsm() {
  unsigned long in_state_ms = millis() - state_enter_ms;

  if (estop_active && state != OFF) {
    // Emergency cuts only dedicated output power relay.
    set_relay(RELAY_CH4_POWER_OUTPUTS, false);
  }

  if (state == OFF) {
    if (short_press_event()) {
      begin_power_on();
    }
    return;
  }

  if (long_press_event() && state != SHUTDOWN_REQUESTED && state != WAITING_SAFE_SHUTDOWN) {
    request_shutdown();
  }

  switch (state) {
    case POWERING_ON:
      if (in_state_ms >= POWER_ON_SETTLE_MS) {
        set_state(WAITING_RPI_BOOT, "psu on settled");
      }
      break;

    case WAITING_RPI_BOOT:
      send_ping_periodic();
      if (rpi_seen) {
        set_state(WAITING_ROS_READY, "rpi discovered");
      } else if (in_state_ms > RPI_BOOT_DISCOVERY_TIMEOUT_MS) {
        set_state(ERROR_STATE, "rpi not discovered");
      }
      break;

    case WAITING_ROS_READY:
      send_ping_periodic();
      if (ros_ready) {
        set_state(WAITING_AGENT_READY, "ros ready");
      } else if (in_state_ms > ROS_READY_TIMEOUT_MS) {
        set_state(ERROR_STATE, "ros ready timeout");
      }
      break;

    case WAITING_AGENT_READY:
      send_ping_periodic();
      if (agent_ready) {
        set_relay(RELAY_CH2_MICROROS_PWR, true);  // CH2 only after AGENT_READY
        if (!estop_active) {
          set_relay(RELAY_CH4_POWER_OUTPUTS, true);
        }
        set_state(RUNNING, "agent ready");
      } else if (in_state_ms > AGENT_READY_TIMEOUT_MS) {
        set_state(ERROR_STATE, "agent ready timeout");
      }
      break;

    case RUNNING:
      if (!agent_ready) {
        set_state(WAITING_AGENT_READY, "agent lost");
        break;
      }
      if (!estop_active) {
        set_relay(RELAY_CH4_POWER_OUTPUTS, true);
      }
      fan_control_from_temp();
      break;

    case SHUTDOWN_REQUESTED:
      send_shutdown_periodic();
      if (in_state_ms > 300) {
        set_state(WAITING_SAFE_SHUTDOWN, "shutdown sent");
      }
      break;

    case WAITING_SAFE_SHUTDOWN:
      send_shutdown_periodic();
      if (safe_to_power_off) {
        hard_power_off();
      } else if (in_state_ms > SAFE_SHUTDOWN_TIMEOUT_MS) {
        add_log("safe shutdown timeout, forcing power off");
        hard_power_off();
      }
      break;

    case ERROR_STATE:
      send_ping_periodic();
      if (long_press_event()) {
        request_shutdown();
      }
      break;

    default:
      break;
  }
}

// =====================================================
// ================= SETUP / LOOP ======================
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  pinMode(RELAY_CH1_PS_ON, OUTPUT);
  pinMode(RELAY_CH2_MICROROS_PWR, OUTPUT);
  pinMode(RELAY_CH3_FAN, OUTPUT);
  pinMode(RELAY_CH4_POWER_OUTPUTS, OUTPUT);

  set_leds(false, false);
  set_all_relays_off();

  last_button_raw = read_input_active(POWER_BUTTON_PIN);
  last_estop_raw = read_input_active(ESTOP_PIN);
  power_button_pressed = last_button_raw;
  estop_active = last_estop_raw;
  last_button_change_ms = millis();
  last_estop_change_ms = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  add_log("Boot complete, waiting button");
  set_state(OFF, "startup");
}

void loop() {
  ensure_wifi_udp();
  service_udp_rx();
  service_inputs();
  check_heartbeat_timeout();
  update_fsm();
  apply_led_pattern();
  send_status();
}
