#include <Arduino.h>
#include <WiFi.h>
#include <math.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

// =====================================================
// ================= WIFI CONFIG =======================
// =====================================================
static char ssid[] = "Villa_Milano";
static char pass[] = "10203040";

static char AGENT_IP[] = "192.168.0.185";
const uint16_t AGENT_PORT = 8888;

// =====================================================
// ================= GPIO (drive only) =================
// =====================================================
const int THR_EN = 5;   // PWM
const int THR_IN1 = 6;
const int THR_IN2 = 7;

const int STR_EN = 12;  // PWM
const int STR_IN1 = 13;
const int STR_IN2 = 14;

// =====================================================
// ================= PWM CONFIG ========================
// =====================================================
const int PWM_FREQ = 1000;
const int PWM_RES_BITS = 8;
const int THR_CH = 0;
const int STR_CH = 1;

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000;
const unsigned long ACT_TIMEOUT_MS = 500;
const unsigned long HEARTBEAT_PERIOD_MS = 500;
unsigned long lastThrTime = 0;
unsigned long lastStrTime = 0;
unsigned long lastHeartbeatTime = 0;
bool failsafeActive = false;

float thrValue = 0.0f;
float strValue = 0.0f;

// =====================================================
// ================= micro-ROS =========================
// =====================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_subscription_t sub_thr;
rcl_subscription_t sub_str;
rcl_publisher_t pub_alive;

std_msgs__msg__Float32 thr_msg;
std_msgs__msg__Float32 str_msg;
std_msgs__msg__Bool alive_msg;

void applyThrottle(float v) {
  int pwm = (int)(fabsf(v) * 255.0f);

  if (v > 0.01f) {
    digitalWrite(THR_IN1, HIGH);
    digitalWrite(THR_IN2, LOW);
  } else if (v < -0.01f) {
    digitalWrite(THR_IN1, LOW);
    digitalWrite(THR_IN2, HIGH);
  } else {
    digitalWrite(THR_IN1, LOW);
    digitalWrite(THR_IN2, LOW);
  }

  ledcWrite(THR_EN, pwm);
}

void applySteering(float v) {
  int pwm = (int)(fabsf(v) * 255.0f);

  if (v > 0.01f) {
    digitalWrite(STR_IN1, HIGH);
    digitalWrite(STR_IN2, LOW);
  } else if (v < -0.01f) {
    digitalWrite(STR_IN1, LOW);
    digitalWrite(STR_IN2, HIGH);
  } else {
    digitalWrite(STR_IN1, LOW);
    digitalWrite(STR_IN2, LOW);
  }

  ledcWrite(STR_EN, pwm);
}

void allOutputsLow() {
  thrValue = 0.0f;
  strValue = 0.0f;
  applyThrottle(0.0f);
  applySteering(0.0f);
}

void throttle_cb(const void *msgin) {
  const std_msgs__msg__Float32 *msg =
      static_cast<const std_msgs__msg__Float32 *>(msgin);

  lastPacketTime = millis();
  lastThrTime = millis();
  failsafeActive = false;

  thrValue = constrain(msg->data, -1.0f, 1.0f);
  applyThrottle(thrValue);
}

void steer_cb(const void *msgin) {
  const std_msgs__msg__Float32 *msg =
      static_cast<const std_msgs__msg__Float32 *>(msgin);

  lastPacketTime = millis();
  lastStrTime = millis();
  failsafeActive = false;

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
    applyThrottle(0.0f);
  }

  if (now - lastStrTime > ACT_TIMEOUT_MS) {
    applySteering(0.0f);
  }

  if (now - lastHeartbeatTime > HEARTBEAT_PERIOD_MS) {
    alive_msg.data = true;
    rcl_publish(&pub_alive, &alive_msg, NULL);
    lastHeartbeatTime = now;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[ESP2] boot (drive)");

  pinMode(THR_EN, OUTPUT);
  pinMode(THR_IN1, OUTPUT);
  pinMode(THR_IN2, OUTPUT);
  pinMode(STR_EN, OUTPUT);
  pinMode(STR_IN1, OUTPUT);
  pinMode(STR_IN2, OUTPUT);

  ledcAttachChannel(THR_EN, PWM_FREQ, PWM_RES_BITS, THR_CH);
  ledcAttachChannel(STR_EN, PWM_FREQ, PWM_RES_BITS, STR_CH);
  allOutputsLow();

  set_microros_wifi_transports(ssid, pass, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp_microros_2", "", &support);

  rclc_subscription_init_best_effort(
      &sub_thr,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "tomo/engine_cmd");

  rclc_subscription_init_best_effort(
      &sub_str,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "tomo/steer_cmd");

  rclc_publisher_init_best_effort(
      &pub_alive,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "tomo/esp2_alive");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_cb);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &sub_thr, &thr_msg, &throttle_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_str, &str_msg, &steer_cb, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  lastPacketTime = millis();
  lastThrTime = millis();
  lastStrTime = millis();
  lastHeartbeatTime = millis();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
