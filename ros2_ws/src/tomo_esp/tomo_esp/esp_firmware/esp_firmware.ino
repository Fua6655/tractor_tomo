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
const int ENGINE_START_PIN   = 8;
const int CLUTCH_PIN         = 9;
const int THROTTLE_PIN       = 10;

const int FRONT_POSITION_PIN = 2;
const int FRONT_SHORT_PIN    = 4;
const int FRONT_LONG_PIN     = 3;
const int BACK_POSITION_PIN  = 5;
const int LEFT_BLINK_PIN     = 6;
const int RIGHT_BLINK_PIN    = 7;

// =====================================================
// ================= STATE TRACKING ====================
// =====================================================
bool armedPrev=false, powerPrev=false, lightPrev=false;
bool engineStartPrev=false, engineStopPrev=false;
bool clutchPrev=false, brakePrev=false, movePrev=false;
bool fpPrev=false, fsPrev=false, flPrev=false;
bool bpPrev=false, lbPrev=false, rbPrev=false;

// =====================================================
// ================= BUFFER ============================
// =====================================================
char udpBuffer[256];

// =====================================================
// ================= SETUP =============================
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ESP32 BOOT ===");

  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(BACK_POSITION_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  allOutputsLow();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  udp.begin(LOCAL_PORT);

  sendLog("ESP READY");
  sendState("IP", WiFi.localIP().toString());
  Serial.print("WEB IP set to: ");
  Serial.println(WEB_IP);

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

  if (msg.startsWith("CMD")) {
    int seq;
    sscanf(msg.c_str(), "CMD,%d", &seq);
    sendRaw("ACK," + String(seq));
    clearFailsafe();

    int idx = msg.indexOf(",OUT,");
    if (idx > 0) {
      processOUT(msg.substring(idx + 1));
    }
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
  logEdge("ARMED", armed, armedPrev);
  logEdge("POWER", power, powerPrev);
  logEdge("LIGHT", light, lightPrev);

  logEdge("ENGINE_START", eng_start, engineStartPrev);
  logEdge("ENGINE_STOP", eng_stop, engineStopPrev);
  logEdge("CLUTCH", clutch, clutchPrev);
  logEdge("BRAKE", brake, brakePrev);
  logEdge("MOVE", move, movePrev);

  logEdge("FP", fp, fpPrev);
  logEdge("FS", fs, fsPrev);
  logEdge("FL", fl, flPrev);
  logEdge("BP", bp, bpPrev);
  logEdge("LB", lb, lbPrev);
  logEdge("RB", rb, rbPrev);

  // ---------- DIGITAL OUTPUTS ----------
  digitalWrite(ENGINE_START_PIN, eng_start);
  digitalWrite(CLUTCH_PIN, clutch);
  digitalWrite(THROTTLE_PIN, brake);

  digitalWrite(FRONT_POSITION_PIN, fp);
  digitalWrite(FRONT_SHORT_PIN, fs);
  digitalWrite(FRONT_LONG_PIN, fl);
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
    Serial.print("FAILSAFE ON");
  }
}

void clearFailsafe() {
  if (failsafeActive) {
    failsafeActive = false;
    sendState("FAILSAFE", "0");
    sendLog("FAILSAFE OFF");
    Serial.print("FAILSAFE OFF");
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
  sendState("ARMED", armedPrev ? "1" : "0");
  sendState("POWER", powerPrev ? "1" : "0");
  sendState("LIGHT", lightPrev ? "1" : "0");

  sendState("ENGINE_START", engineStartPrev ? "1" : "0");
  sendState("ENGINE_STOP", engineStopPrev ? "1" : "0");
  sendState("CLUTCH", clutchPrev ? "1" : "0");
  sendState("BRAKE", brakePrev ? "1" : "0");
  sendState("MOVE", movePrev ? "1" : "0");

  sendState("FP", fpPrev ? "1" : "0");
  sendState("FS", fsPrev ? "1" : "0");
  sendState("FL", flPrev ? "1" : "0");
  sendState("BP", bpPrev ? "1" : "0");
  sendState("LB", lbPrev ? "1" : "0");
  sendState("RB", rbPrev ? "1" : "0");
}

// =====================================================
// ================= UTIL ==============================
// =====================================================
void allOutputsLow() {
  digitalWrite(ENGINE_START_PIN, LOW);
  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(THROTTLE_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(BACK_POSITION_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
}
