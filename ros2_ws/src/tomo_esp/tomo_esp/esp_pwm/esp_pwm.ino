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
// ================= L298N PINS ========================
// =====================================================
// ---- THROTTLE ----
const int THR_EN  = 10;   // PWM
const int THR_IN1 = 11;
const int THR_IN2 = 12;

// ---- STEERING ----
const int STR_EN  = 13;   // PWM
const int STR_IN1 = 14;
const int STR_IN2 = 15;

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

  pinMode(THR_EN, OUTPUT);
  pinMode(THR_IN1, OUTPUT);
  pinMode(THR_IN2, OUTPUT);

  pinMode(STR_EN, OUTPUT);
  pinMode(STR_IN1, OUTPUT);
  pinMode(STR_IN2, OUTPUT);

  stopAllActuators();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(200);

  udp.begin(LOCAL_PORT);

  sendLog("ESP READY");
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
  if (!packetSize) return;

  int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
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

  analogWrite(THR_EN, pwm);
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

  analogWrite(STR_EN, pwm);
}

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
void handleFailsafe() {
  if (!failsafeActive && millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    failsafeActive = true;
    stopAllActuators();
    sendState("FAILSAFE", "1");
    sendLog("FAILSAFE ON");
  }
}

void clearFailsafe() {
  if (failsafeActive) {
    failsafeActive = false;
    sendState("FAILSAFE", "0");
    sendLog("FAILSAFE OFF");
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

// =====================================================
// ================= UTIL ==============================
// =====================================================
void stopAllActuators() {
  analogWrite(THR_EN, 0);
  analogWrite(STR_EN, 0);

  digitalWrite(THR_IN1, LOW);
  digitalWrite(THR_IN2, LOW);
  digitalWrite(STR_IN1, LOW);
  digitalWrite(STR_IN2, LOW);
}
