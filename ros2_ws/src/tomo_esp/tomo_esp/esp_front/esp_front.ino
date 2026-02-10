#include <WiFi.h>
#include <WiFiUdp.h>

// =====================================================
// ================= WIFI CONFIG =======================
// =====================================================
const char* ssid = "Villa_Milano";
const char* pass = "10203040";

// =====================================================
// ================= UDP CONFIG ========================
// =====================================================
WiFiUDP udp;
const uint16_t LOCAL_PORT = 8888;
const uint16_t WEB_PORT   = 9999;

IPAddress WEB_IP;

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000;
bool failsafeActive = false;

// =====================================================
// ================= GPIO ==============================
// =====================================================
const int CLUTCH_PIN         = 9;

const int FRONT_POSITION_PIN = 2;
const int FRONT_SHORT_PIN    = 4;
const int FRONT_LONG_PIN     = 3;
const int LEFT_BLINK_PIN     = 6;
const int RIGHT_BLINK_PIN    = 7;

// ---- THROTTLE ----
const int THR_EN  = 10;   // PWM
const int THR_IN1 = 11;
const int THR_IN2 = 12;

// ---- STEERING ----
const int STR_EN  = 13;   // PWM
const int STR_IN1 = 14;
const int STR_IN2 = 15;

// =====================================================
// ================= PWM CONFIG ========================
// =====================================================
const int PWM_FREQ = 1000;      // Hz
const int PWM_RES_BITS = 8;     // 0-255
const int THR_CH = 0;
const int STR_CH = 1;

// =====================================================
// ================= STATE TRACKING ====================
// =====================================================
bool clutchPrev=false;
bool fpPrev=false, fsPrev=false, flPrev=false;
bool lbPrev=false, rbPrev=false;

// =====================================================
// ================= ACTUATOR STATE ====================
// =====================================================
float thrValue = 0.0;
float strValue = 0.0;

unsigned long lastThrTime = 0;
unsigned long lastStrTime = 0;
const unsigned long ACT_TIMEOUT_MS = 500;

// =====================================================
// ================= BUFFER ============================
// =====================================================
char udpBuffer[256];

// =====================================================
// ================= SETUP =============================
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  pinMode(THR_EN, OUTPUT);
  pinMode(THR_IN1, OUTPUT);
  pinMode(THR_IN2, OUTPUT);

  pinMode(STR_EN, OUTPUT);
  pinMode(STR_IN1, OUTPUT);
  pinMode(STR_IN2, OUTPUT);

  ledcAttachChannel(THR_EN, PWM_FREQ, PWM_RES_BITS, THR_CH);
  ledcAttachChannel(STR_EN, PWM_FREQ, PWM_RES_BITS, STR_CH);

  allOutputsLow();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(200);

  udp.begin(LOCAL_PORT);

  sendLog("ESP FRONT READY");
  sendState("IP", WiFi.localIP().toString());
}

// =====================================================
// ================= LOOP ==============================
// =====================================================
void loop() {
  handleUDP();
  handleFailsafe();
  handleActuatorWatchdog();
}

// =====================================================
// ================= UDP RX ============================
// =====================================================
void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  int len = udp.read(udpBuffer, sizeof(udpBuffer)-1);
  if (len <= 0) return;
  udpBuffer[len] = '\0';

  String msg = String(udpBuffer);
  msg.trim();

  lastPacketTime = millis();
  WEB_IP = udp.remoteIP();

  if (msg.startsWith("HEARTBEAT")) {
    clearFailsafe();
    return;
  }

  if (!msg.startsWith("CMD")) return;

  int seq;
  sscanf(msg.c_str(), "CMD,%d", &seq);
  sendRaw("ACK," + String(seq));
  clearFailsafe();

  int idx = msg.indexOf(",OUT,");
  if (idx > 0) {
    processOUT(msg.substring(idx + 1));
  }

  if (msg.indexOf(",THR,") > 0) {
    processTHR(msg.substring(msg.indexOf(",THR,") + 1));
    return;
  }

  if (msg.indexOf(",STR,") > 0) {
    processSTR(msg.substring(msg.indexOf(",STR,") + 1));
    return;
  }
}

// =====================================================
// ================= OUT PARSER ========================
// =====================================================
void processOUT(const String& out) {

  int armed, power, light;
  int eng_start, eng_stop, clutch, brake, move;
  int fp, fs, fl, bp, lb, rb;

  int parsed = sscanf(
    out.c_str(),
    "OUT,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    &armed,&power,&light,
    &eng_start,&eng_stop,&clutch,&brake,&move,
    &fp,&fs,&fl,&bp,&lb,&rb
  );

  if (parsed != 14) {
    sendLog("OUT parse failed");
    return;
  }

  // ---------- EDGE LOGGING ----------
  logEdge("CLUTCH", clutch, clutchPrev);

  logEdge("FP", fp, fpPrev);
  logEdge("FS", fs, fsPrev);
  logEdge("FL", fl, flPrev);
  logEdge("LB", lb, lbPrev);
  logEdge("RB", rb, rbPrev);

  // ---------- DIGITAL OUTPUTS ----------
  digitalWrite(CLUTCH_PIN, clutch);

  digitalWrite(FRONT_POSITION_PIN, fp);
  digitalWrite(FRONT_SHORT_PIN, fs);
  digitalWrite(FRONT_LONG_PIN, fl);
  digitalWrite(LEFT_BLINK_PIN, lb);
  digitalWrite(RIGHT_BLINK_PIN, rb);
}

// =====================================================
// ================= THROTTLE ==========================
// =====================================================
void processTHR(const String& msg) {
  float v;
  if (sscanf(msg.c_str(), "THR,%f", &v) != 1) return;

  thrValue = constrain(v, -1.0, 1.0);
  lastThrTime = millis();

  applyThrottle(thrValue);
  sendState("THR", String(thrValue, 3));
}

void applyThrottle(float v) {
  int pwm = abs(v) * 255;

  if (v > 0.01) {
    digitalWrite(THR_IN1, HIGH); // add gas
    digitalWrite(THR_IN2, LOW);
  } else if (v < -0.01) {
    digitalWrite(THR_IN1, LOW);  // release gas
    digitalWrite(THR_IN2, HIGH);
  } else {
    digitalWrite(THR_IN1, LOW);
    digitalWrite(THR_IN2, LOW);
  }

  ledcWrite(THR_EN, pwm);
}

// =====================================================
// ================= STEERING ==========================
// =====================================================
void processSTR(const String& msg) {
  float v;
  if (sscanf(msg.c_str(), "STR,%f", &v) != 1) return;

  strValue = constrain(v, -1.0, 1.0);
  lastStrTime = millis();

  applySteering(strValue);
  sendState("STR", String(strValue, 3));
}

void applySteering(float v) {
  int pwm = abs(v) * 255;

  if (v > 0.01) {
    digitalWrite(STR_IN1, HIGH);
    digitalWrite(STR_IN2, LOW);
  } else if (v < -0.01) {
    digitalWrite(STR_IN1, LOW);
    digitalWrite(STR_IN2, HIGH);
  } else {
    digitalWrite(STR_IN1, LOW);
    digitalWrite(STR_IN2, LOW);
  }

  ledcWrite(STR_EN, pwm);
}

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
void handleFailsafe() {
  if (!failsafeActive && millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    failsafeActive = true;
    allOutputsLow();
    sendState("FAILSAFE", "1");
    sendLog("FAILSAFE ON");
  }
}

void clearFailsafe() {
  if (failsafeActive) {
    failsafeActive = false;
    sendState("FAILSAFE", "0");
    sendLog("FAILSAFE OFF");
    publishAllStates();
  }
}

// =====================================================
// ================= WATCHDOG ==========================
// =====================================================
void handleActuatorWatchdog() {
  unsigned long now = millis();

  if (now - lastThrTime > ACT_TIMEOUT_MS) {
    applyThrottle(0.0);
  }

  if (now - lastStrTime > ACT_TIMEOUT_MS) {
    applySteering(0.0);
  }
}

// =====================================================
// ================= WEB TX HELPERS ====================
// =====================================================
void sendRaw(const String& msg) {
  if (!WEB_IP) return;
  udp.beginPacket(WEB_IP, WEB_PORT);
  udp.print(msg);
  udp.endPacket();
}

void sendLog(const String& text) {
  sendRaw("LOG," + text);
}

void sendState(const String& name, const String& value) {
  sendRaw("STATE," + name + "," + value);
}

void logEdge(const char* name, bool now, bool &prev) {
  if (now != prev) {
    prev = now;
    sendState(name, now ? "1" : "0");
  }
}

void publishAllStates() {
  sendState("CLUTCH", clutchPrev ? "1" : "0");

  sendState("FP", fpPrev ? "1" : "0");
  sendState("FS", fsPrev ? "1" : "0");
  sendState("FL", flPrev ? "1" : "0");
  sendState("LB", lbPrev ? "1" : "0");
  sendState("RB", rbPrev ? "1" : "0");

  sendState("THR", String(thrValue, 3));
  sendState("STR", String(strValue, 3));
}

// =====================================================
// ================= UTIL ==============================
// =====================================================
void allOutputsLow() {
  thrValue = 0.0;
  strValue = 0.0;

  ledcWrite(THR_EN, 0);
  ledcWrite(STR_EN, 0);

  digitalWrite(THR_IN1, LOW);
  digitalWrite(THR_IN2, LOW);
  digitalWrite(STR_IN1, LOW);
  digitalWrite(STR_IN2, LOW);

  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
}
