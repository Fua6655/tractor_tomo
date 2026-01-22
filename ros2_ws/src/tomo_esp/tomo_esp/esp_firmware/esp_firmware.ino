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
const int BACK_PIN           = 5;
const int LEFT_BLINK_PIN     = 6;
const int RIGHT_BLINK_PIN    = 7;

// =====================================================
// ================= STATE TRACKING ====================
// =====================================================
bool armedPrev=false, powerPrev=false, lightPrev=false;
bool enginePrev=false, clutchPrev=false, speedPrev=false, movePrev=false;
bool fpPrev=false, fsPrev=false, flPrev=false, backPrev=false, lbPrev=false, rbPrev=false;

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

  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(BACK_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);

  allOutputsLow();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) delay(300);

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

  int a,p,l,e,c,s,m,fp,fs,fl,b,lb,rb;
  float lin, ang;

  sscanf(out.c_str(),
    "OUT,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f",
    &a,&p,&l,&e,&c,&s,&m,
    &fp,&fs,&fl,&b,&lb,&rb,
    &lin,&ang
  );

  edge("ARMED", a, armedPrev);
  edge("POWER", p, powerPrev);
  edge("LIGHT", l, lightPrev);

  edge("ENGINE", e, enginePrev);
  edge("CLUTCH", c, clutchPrev);
  edge("SPEED", s, speedPrev);
  edge("MOVE", m, movePrev);

  edge("FP", fp, fpPrev);
  edge("FS", fs, fsPrev);
  edge("FL", fl, flPrev);
  edge("BACK", b, backPrev);
  edge("LB", lb, lbPrev);
  edge("RB", rb, rbPrev);

  digitalWrite(ENGINE_START_PIN, e);
  digitalWrite(CLUTCH_PIN, c);
  digitalWrite(THROTTLE_PIN, s);

  digitalWrite(FRONT_POSITION_PIN, fp);
  digitalWrite(FRONT_SHORT_PIN, fs);
  digitalWrite(FRONT_LONG_PIN, fl);
  digitalWrite(BACK_PIN, b);
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
  }
}

// =====================================================
// ================= HELPERS ===========================
// =====================================================
void edge(const char* name, bool now, bool &prev) {
  if (now != prev) {
    prev = now;
    sendState(name, now ? "1" : "0");
  }
}

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

void allOutputsLow() {
  digitalWrite(ENGINE_START_PIN, LOW);
  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(THROTTLE_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(BACK_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
}
