#include <Arduino.h>
#include <WiFi.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <tomo_msgs/msg/output_states.h>

// =====================================================
// ================= WIFI CONFIG =======================
// =====================================================
static char ssid[] = "Villa_Milano";
static char pass[] = "10203040";

// micro-ROS agent (PC) IP + UDP port
static char AGENT_IP[] = "192.168.0.185";
const uint16_t AGENT_PORT = 8888;

// =====================================================
// ================= GPIO ==============================
// =====================================================
const int ENGINE_START_PIN = 40;
const int CLUTCH_PIN = 41;
const int BRAKE_PIN = 42;

const int FRONT_POSITION_PIN = 15;
const int FRONT_SHORT_PIN = 16;
const int FRONT_LONG_PIN = 17;

const int BACK_POSITION_PIN = 9;
const int LEFT_BLINK_PIN = 10;
const int RIGHT_BLINK_PIN = 11;

// ---- THROTTLE ----
const int THR_EN = 4;   // PWM
const int THR_IN1 = 5;
const int THR_IN2 = 6;

// ---- STEERING ----
const int STR_EN = 12;   // PWM
const int STR_IN1 = 13;
const int STR_IN2 = 14;

// =====================================================
// ================= PWM CONFIG ========================
// =====================================================
const int PWM_FREQ = 1000;   // Hz
const int PWM_RES_BITS = 8;  // 0-255
const int THR_CH = 0;
const int STR_CH = 1;

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000;
bool failsafeActive = false;

// =====================================================
// ================= ACTUATOR STATE ====================
// =====================================================
float thrValue = 0.0;
float strValue = 0.0;

unsigned long lastThrTime = 0;
unsigned long lastStrTime = 0;
const unsigned long ACT_TIMEOUT_MS = 500;
const unsigned long HEARTBEAT_PERIOD_MS = 500;
unsigned long lastHeartbeatTime = 0;

// =====================================================
// ================= micro-ROS =========================
// =====================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_subscription_t sub_out;
rcl_subscription_t sub_thr;
rcl_subscription_t sub_str;
rcl_publisher_t pub_alive;

tomo_msgs__msg__OutputStates out_msg;
std_msgs__msg__Float32 thr_msg;
std_msgs__msg__Float32 str_msg;
std_msgs__msg__Bool alive_msg;

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

  digitalWrite(ENGINE_START_PIN, LOW);
  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(BRAKE_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(BACK_POSITION_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
}

void applyThrottle(float v) {
  int pwm = abs(v) * 255;

  if (v > 0.01) {
    digitalWrite(THR_IN1, HIGH);  // add gas
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
// ================= micro-ROS callbacks ===============
// =====================================================
void output_cb(const void *msgin) {
  const tomo_msgs__msg__OutputStates *msg =
      static_cast<const tomo_msgs__msg__OutputStates *>(msgin);

  lastPacketTime = millis();

  digitalWrite(ENGINE_START_PIN, msg->engine_start);
  digitalWrite(CLUTCH_PIN, msg->clutch_active);
  digitalWrite(BRAKE_PIN, msg->brake_active);

  digitalWrite(FRONT_POSITION_PIN, msg->front_position);
  digitalWrite(FRONT_SHORT_PIN, msg->front_short);
  digitalWrite(FRONT_LONG_PIN, msg->front_long);
  digitalWrite(BACK_POSITION_PIN, msg->back_position);
  digitalWrite(LEFT_BLINK_PIN, msg->left_blink);
  digitalWrite(RIGHT_BLINK_PIN, msg->right_blink);
}

void throttle_cb(const void *msgin) {
  const std_msgs__msg__Float32 *msg =
      static_cast<const std_msgs__msg__Float32 *>(msgin);

  lastPacketTime = millis();
  lastThrTime = millis();

  thrValue = constrain(msg->data, -1.0f, 1.0f);
  applyThrottle(thrValue);
}

void steer_cb(const void *msgin) {
  const std_msgs__msg__Float32 *msg =
      static_cast<const std_msgs__msg__Float32 *>(msgin);

  lastPacketTime = millis();
  lastStrTime = millis();

  strValue = constrain(msg->data, -1.0f, 1.0f);
  applySteering(strValue);
}

void timer_cb(rcl_timer_t *, int64_t) {
  unsigned long now = millis();

  if (!failsafeActive && now - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    failsafeActive = true;
    allOutputsLow();
  }

  if (now - lastThrTime > ACT_TIMEOUT_MS) {
    applyThrottle(0.0);
  }

  if (now - lastStrTime > ACT_TIMEOUT_MS) {
    applySteering(0.0);
  }

  if (now - lastHeartbeatTime > HEARTBEAT_PERIOD_MS) {
    alive_msg.data = true;
    rcl_publish(&pub_alive, &alive_msg, NULL);
    lastHeartbeatTime = now;
  }
}

// =====================================================
// ================= SETUP =============================
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[ESP] boot");

  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(BACK_POSITION_PIN, OUTPUT);
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

  Serial.print("[ESP] WiFi SSID: ");
  Serial.println(ssid);
  Serial.print("[ESP] Agent IP: ");
  Serial.println(AGENT_IP);
  Serial.print("[ESP] Agent port: ");
  Serial.println(AGENT_PORT);

  set_microros_wifi_transports(ssid, pass, AGENT_IP, AGENT_PORT);
  delay(2000);
  Serial.println("[ESP] micro-ROS transport set");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  Serial.println("[ESP] rclc_support_init ok");
  rclc_node_init_default(&node, "esp_microros", "", &support);
  Serial.println("[ESP] node init ok");

  rclc_subscription_init_best_effort(
      &sub_out,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(tomo_msgs, msg, OutputStates),
      "tomo/states");
  Serial.println("[ESP] sub tomo/states ok");

  rclc_subscription_init_best_effort(
      &sub_thr,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "tomo/engine_cmd");
  Serial.println("[ESP] sub tomo/engine_cmd ok");

  rclc_subscription_init_best_effort(
      &sub_str,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "tomo/steer_cmd");
  Serial.println("[ESP] sub tomo/steer_cmd ok");

  rclc_publisher_init_best_effort(
      &pub_alive,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "tomo/esp_alive");
  Serial.println("[ESP] pub tomo/esp_alive ok");

  rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(100),
      timer_cb);
  Serial.println("[ESP] timer init ok");

  rclc_executor_init(&executor, &support.context, 4, &allocator);
  Serial.println("[ESP] executor init ok");
  rclc_executor_add_subscription(&executor, &sub_out, &out_msg, &output_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_thr, &thr_msg, &throttle_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_str, &str_msg, &steer_cb, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  lastPacketTime = millis();
  lastThrTime = millis();
  lastStrTime = millis();
  lastHeartbeatTime = millis();
}

// =====================================================
// ================= LOOP ==============================
// =====================================================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
