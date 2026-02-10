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
const int BRAKE_PIN          = 10;
const int BACK_POSITION_PIN  = 5;
const int LEFT_BLINK_PIN     = 6;
const int RIGHT_BLINK_PIN    = 7;

// =====================================================
// ================= STATE TRACKING ====================
// =====================================================
bool brakePrev=false;
bool bpPrev=false;
bool lbPrev=false, rbPrev=false;

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

  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(BACK_POSITION_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  allOutputsLow();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(200);

  udp.begin(LOCAL_PORT);

  sendLog("ESP BACK READY");
  sendState("IP", WiFi.localIP().toString());
}

// =====================================================
// ================= LOOP ==============================
// =====================================================
void loop() {
  handleUDP();
  handleFailsafe();
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
  logEdge("BRAKE", brake, brakePrev);
  logEdge("BP", bp, bpPrev);
  logEdge("LB", lb, lbPrev);
  logEdge("RB", rb, rbPrev);

  // ---------- DIGITAL OUTPUTS ----------
  digitalWrite(BRAKE_PIN, brake);
  digitalWrite(BACK_POSITION_PIN, bp);
  digitalWrite(LEFT_BLINK_PIN, lb);
  digitalWrite(RIGHT_BLINK_PIN, rb);
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
  sendState("BRAKE", brakePrev ? "1" : "0");
  sendState("BP", bpPrev ? "1" : "0");
  sendState("LB", lbPrev ? "1" : "0");
  sendState("RB", rbPrev ? "1" : "0");
}

// =====================================================
// ================= UTIL ==============================
// =====================================================
void allOutputsLow() {
  digitalWrite(BRAKE_PIN, LOW);
  digitalWrite(BACK_POSITION_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
}
