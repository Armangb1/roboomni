#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <std_msgs/msg/float32_multi_array.h>  // or your actual message type
#include <sensor_msgs/msg/joint_state.h>       // if using JointState
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <driver/ledc.h>

// ============================
// === MOTOR & ENCODER SETUP ===
// ============================

// Motor Pins
const int motor_in1[] = {5, 18, 4, 15};
const int motor_in2[] = {17, 19, 16, 2};

// Encoder Pins
const int enc_a[] = {12, 27, 32, 25};
const int enc_b[] = {13, 14, 33, 26};

// Globals
volatile long encoder_counts[4] = {0};
int pwm_values[4] = {0};

// =======================
// === ROS + micro-ROS ===
// =======================
rcl_publisher_t joint_pub;
sensor_msgs__msg__JointState joint_msg;

rcl_subscription_t pwm_sub;
std_msgs__msg__Int16MultiArray pwm_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t timer;

void pwm_callback(const void *msgin);
void publish_encoder_cb(rcl_timer_t *timer, int64_t last_call_time);

// ===============
// === PWM SETUP ===
// ===============
void setup_motors() {
  for (int i = 0; i < 4; i++) {
    pinMode(motor_in1[i], OUTPUT);
    pinMode(motor_in2[i], OUTPUT);
    ledcAttachPin(motor_in1[i], i);  // PWM on IN1
    ledcSetup(i, 20000, 16);         // 20kHz, 16-bit
  }
}

void update_motor_outputs() {
  for (int i = 0; i < 4; i++) {
    int pwm = pwm_values[i];
    int speed = abs(pwm);
    if (speed > 65535) speed = 65535;

    digitalWrite(motor_in2[i], pwm >= 0 ? LOW : HIGH);  // IN2 = DIR
    ledcWrite(i, speed);                                // IN1 = PWM
  }
}

// ================
// === ENCODERS ===
// ================
void IRAM_ATTR encoder_isr(int i) {
  int a = digitalRead(enc_a[i]);
  int b = digitalRead(enc_b[i]);
  if (a == b) encoder_counts[i]++;
  else encoder_counts[i]--;
}

void IRAM_ATTR encoder_isr_0() { encoder_isr(0); }
void IRAM_ATTR encoder_isr_1() { encoder_isr(1); }
void IRAM_ATTR encoder_isr_2() { encoder_isr(2); }
void IRAM_ATTR encoder_isr_3() { encoder_isr(3); }

void setup_encoders() {
  for (int i = 0; i < 4; i++) {
    pinMode(enc_a[i], INPUT_PULLUP);
    pinMode(enc_b[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(enc_a[0]), encoder_isr_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_a[1]), encoder_isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_a[2]), encoder_isr_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_a[3]), encoder_isr_3, CHANGE);
}

// ===================
// === ROS Setup  ===
// ===================
void pwm_callback(const void *msgin) {
  const std_msgs__msg__Int16MultiArray *msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  for (int i = 0; i < 4 && i < msg->data.size; i++) {
    pwm_values[i] = msg->data.data[i];
  }
}

void publish_encoder_cb(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  int64_t now = rmw_uros_epoch_millis();
  joint_msg.header.stamp.sec = now / 1000;
  joint_msg.header.stamp.nanosec = (now % 1000) * 1000000;
  for (int i = 0; i < 4; i++) {
    joint_msg.position.data[i] = (double)encoder_counts[i];
  }
  rcl_publish(&joint_pub, &joint_msg, NULL);
}

void setup_ros() {
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Subscriber
  rclc_subscription_init_default(
    &pwm_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/pwm_cmds"
  );

  // Publisher
  rclc_publisher_init_default(
    &joint_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/encoder"
  );

  // Timer
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(20),  // 50Hz
    publish_encoder_cb
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &pwm_sub, &pwm_msg, &pwm_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  // Init joint state message
  joint_msg.name.size = 4;
  joint_msg.name.capacity = 4;
  joint_msg.name.data = (rosidl_runtime_c__String *)malloc(4 * sizeof(rosidl_runtime_c__String));
  joint_msg.position.data = (double *)malloc(4 * sizeof(double));
  joint_msg.position.size = 4;
  joint_msg.position.capacity = 4;
  for (int i = 0; i < 4; i++) {
    joint_msg.name.data[i] = (rosidl_runtime_c__String){};
    rosidl_runtime_c__String__assign(&joint_msg.name.data[i], ("wheel_" + String(i)).c_str());
  }

  // Init pwm message
  pwm_msg.data.data = (int16_t *)malloc(4 * sizeof(int16_t));
  pwm_msg.data.size = 4;
  pwm_msg.data.capacity = 4; 
}

// ======================
// === FreeRTOS Tasks ===
// ======================

void ros_spin_task(void *pvParameters) {
  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(pdMS_TO_TICKS(5));  // 200Hz
  }
}

void motor_control_task(void *pvParameters) {
  while (1) {
    update_motor_outputs();  // Apply latest PWM
    vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
  }
}

// ===================
// === Arduino Init ===
// ===================
void setup() {
  Serial.begin(115200);
  delay(2000);
  setup_motors();
  setup_encoders();
  setup_ros();

  // Tasks
  xTaskCreatePinnedToCore(ros_spin_task, "ros_spin", 4000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 2048, NULL, 1, NULL, 0);
}

void loop() {
  // Nothing here — all handled in FreeRTOS
}
