#include <Arduino.h>
#include <WiFi.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <tomo_msgs/msg/output_states.h>

// =====================================================
// ================= WIFI CONFIG =======================
// =====================================================
static char ssid[] = "Villa_Milano";
static char pass[] = "10203040";

static char AGENT_IP[] = "192.168.0.185";
const uint16_t AGENT_PORT = 8888;

// =====================================================
// ================= GPIO (states only) ===============
// =====================================================
const int ENGINE_START_PIN = 36;
const int CLUTCH_PIN = 37;
const int BRAKE_PIN = 38;

const int FRONT_POSITION_PIN = 9;
const int FRONT_SHORT_PIN = 10;
const int FRONT_LONG_PIN = 11;
const int BACK_POSITION_PIN = 15;
const int LEFT_BLINK_PIN = 16;
const int RIGHT_BLINK_PIN = 17;
const int HORN_PIN = 18;

// ---- EXTRA STATUS RELAYS ----
const int ARMED_PIN = 5;
const int ENGINE_ON_PIN = 6;
const int MOVE_ALLOWED_PIN = 7;

// =====================================================
// ================= FAILSAFE ==========================
// =====================================================
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 1000;
const unsigned long HEARTBEAT_PERIOD_MS = 500;
unsigned long lastHeartbeatTime = 0;
bool failsafeActive = false;

// =====================================================
// ================= micro-ROS =========================
// =====================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_subscription_t sub_out;
rcl_publisher_t pub_alive;

tomo_msgs__msg__OutputStates out_msg;
std_msgs__msg__Bool alive_msg;

void allOutputsLow() {
  digitalWrite(ENGINE_START_PIN, LOW);
  digitalWrite(CLUTCH_PIN, LOW);
  digitalWrite(BRAKE_PIN, LOW);
  digitalWrite(FRONT_POSITION_PIN, LOW);
  digitalWrite(FRONT_SHORT_PIN, LOW);
  digitalWrite(FRONT_LONG_PIN, LOW);
  digitalWrite(BACK_POSITION_PIN, LOW);
  digitalWrite(LEFT_BLINK_PIN, LOW);
  digitalWrite(RIGHT_BLINK_PIN, LOW);
  digitalWrite(HORN_PIN, LOW);
  digitalWrite(ARMED_PIN, LOW);
  digitalWrite(ENGINE_ON_PIN, LOW);
  digitalWrite(MOVE_ALLOWED_PIN, LOW);
}

void output_cb(const void *msgin) {
  const tomo_msgs__msg__OutputStates *msg =
      static_cast<const tomo_msgs__msg__OutputStates *>(msgin);

  lastPacketTime = millis();
  failsafeActive = false;

  digitalWrite(ENGINE_START_PIN, msg->engine_start);
  digitalWrite(CLUTCH_PIN, msg->clutch_active);
  digitalWrite(BRAKE_PIN, msg->brake_active);

  digitalWrite(FRONT_POSITION_PIN, msg->front_position);
  digitalWrite(FRONT_SHORT_PIN, msg->front_short);
  digitalWrite(FRONT_LONG_PIN, msg->front_long);
  digitalWrite(BACK_POSITION_PIN, msg->back_position);
  digitalWrite(LEFT_BLINK_PIN, msg->left_blink);
  digitalWrite(RIGHT_BLINK_PIN, msg->right_blink);
  digitalWrite(HORN_PIN, msg->horn);
  digitalWrite(ARMED_PIN, msg->armed_state);
  digitalWrite(ENGINE_ON_PIN, msg->engine_state);
  digitalWrite(MOVE_ALLOWED_PIN, msg->move_allowed);
}

void timer_cb(rcl_timer_t *, int64_t) {
  unsigned long now = millis();

  if (!failsafeActive && now - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
    failsafeActive = true;
    allOutputsLow();
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
  Serial.println("\n[ESP1] boot (states)");

  pinMode(ENGINE_START_PIN, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(FRONT_POSITION_PIN, OUTPUT);
  pinMode(FRONT_SHORT_PIN, OUTPUT);
  pinMode(FRONT_LONG_PIN, OUTPUT);
  pinMode(BACK_POSITION_PIN, OUTPUT);
  pinMode(LEFT_BLINK_PIN, OUTPUT);
  pinMode(RIGHT_BLINK_PIN, OUTPUT);
  pinMode(HORN_PIN, OUTPUT);
  pinMode(ARMED_PIN, OUTPUT);
  pinMode(ENGINE_ON_PIN, OUTPUT);
  pinMode(MOVE_ALLOWED_PIN, OUTPUT);
  allOutputsLow();

  set_microros_wifi_transports(ssid, pass, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp_microros_1", "", &support);

  rclc_subscription_init_best_effort(
      &sub_out,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(tomo_msgs, msg, OutputStates),
      "tomo/states");

  rclc_publisher_init_best_effort(
      &pub_alive,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "tomo/esp1_alive");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_cb);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_out, &out_msg, &output_cb, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  lastPacketTime = millis();
  lastHeartbeatTime = millis();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
