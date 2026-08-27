#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/point.h>

// ----------------------------- Hardware defaults -----------------------------
// Change these only if the PCB wiring changes. Runtime tuning is done with
// `ros2 param set /gantry_controller <name> <value>`.
static constexpr int X1_STEP_PIN = 19;
static constexpr int X1_DIR_PIN  = 18;
static constexpr int X2_STEP_PIN = 22;
static constexpr int X2_DIR_PIN  = 23;
static constexpr int Y_STEP_PIN  = 26;
static constexpr int Y_DIR_PIN   = 25;

static constexpr int X1_HOME_PIN = 14;   // NC to GND: LOW = normal, HIGH = pressed/open/fault
static constexpr int X2_HOME_PIN = 13;   // NC to GND: LOW = normal, HIGH = pressed/open/fault
static constexpr int Y_HOME_PIN = 27;    // NC to GND: LOW = normal, HIGH = pressed/open/fault
static constexpr int ESTOP_PIN = 32;     // NC E-stop monitor only: INPUT_PULLUP, HIGH = pressed/open/fault
static constexpr int ENABLE_PIN = -1;   // set GPIO if all drivers share enable

// ----------------------------- Safety defaults -----------------------------
static constexpr float DEFAULT_FULL_STEPS_PER_MM_X = 1.0f;
static constexpr float DEFAULT_FULL_STEPS_PER_MM_Y = 40.0f;
static constexpr int DEFAULT_MICROSTEPS_X = 32;
static constexpr int DEFAULT_MICROSTEPS_Y = 8;
static constexpr float DEFAULT_MAX_X_MM = 1650.0f;
static constexpr float DEFAULT_MAX_Y_MM = 590.0f;
static constexpr float DEFAULT_SPEED_MM_S = 30.0f;
static constexpr float DEFAULT_ACCEL_MM_S2 = 150.0f;

static constexpr uint32_t ROS_SPIN_PERIOD_MS = 10;
static constexpr uint32_t FEEDBACK_PERIOD_MS = 100;
static constexpr uint32_t MOTION_PERIOD_MS = 5;

// ----------------------------- ROS objects -----------------------------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t position_pub;
rcl_subscription_t target_sub;
rcl_subscription_t x_sub;
rcl_subscription_t y_sub;
rcl_subscription_t speed_sub;
rcl_subscription_t accel_sub;
rcl_subscription_t calibrate_sub;
rclc_parameter_server_t param_server;

geometry_msgs__msg__Point target_msg;
std_msgs__msg__Float32 x_msg;
std_msgs__msg__Float32 y_msg;
std_msgs__msg__Float32 speed_msg;
std_msgs__msg__Float32 accel_msg;
std_msgs__msg__Bool calibrate_msg;
geometry_msgs__msg__Point current_position_msg;

// ----------------------------- Motion objects -----------------------------
FastAccelStepperEngine engine;
FastAccelStepper *x1_motor = nullptr;
FastAccelStepper *x2_motor = nullptr;
FastAccelStepper *y_motor = nullptr;

portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
Preferences prefs;


struct Config {
  float full_steps_per_mm_x = DEFAULT_FULL_STEPS_PER_MM_X;
  float full_steps_per_mm_y = DEFAULT_FULL_STEPS_PER_MM_Y;
  int microsteps_x = DEFAULT_MICROSTEPS_X;
  int microsteps_y = DEFAULT_MICROSTEPS_Y;
  float max_x_mm = DEFAULT_MAX_X_MM;
  float max_y_mm = DEFAULT_MAX_Y_MM;
  float min_x_mm = 0.0f;
  float min_y_mm = 0.0f;
  float speed_mm_s = DEFAULT_SPEED_MM_S;
  float acceleration_mm_s2 = DEFAULT_ACCEL_MM_S2;
  float calibration_fast_mm_s = 25.0f;
  float calibration_slow_mm_s = 2.0f;
  float calibration_backoff_mm = 5.0f;
  float calibration_search_mm = 2000.0f;
  float x_calibration_offset_mm = 0.0f;
  float y_calibration_offset_mm = 0.0f;
  bool invert_x1_dir = false;
  bool invert_x2_dir = true;   // mirrored X motor commonly needs opposite dir
  bool invert_y_dir = true;
  bool enable_homing = true;
  bool enable_estop_monitor = true;
  bool enable_motors = true;
  bool reset_saved_config = false;
};

Config cfg;
float target_x_mm = 0.0f;
float target_y_mm = 0.0f;
float current_x_mm = 0.0f;
float current_y_mm = 0.0f;
bool estop_active = false;
volatile bool config_update_active = false;
float active_target_x_mm = 0.0f;
float active_target_y_mm = 0.0f;
bool pending_motion_command = false;
bool x_limit_fault = false;
bool y_limit_fault = false;
volatile bool calibration_requested = false;
volatile bool calibration_active = false;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int clamp_microsteps(int value) {
  if (value <= 1) return 1;
  if (value <= 2) return 2;
  if (value <= 4) return 4;
  if (value <= 8) return 8;
  if (value <= 16) return 16;
  if (value <= 32) return 32;
  if (value <= 64) return 64;
  return 128;
}

static float effective_steps_per_mm_x() { return cfg.full_steps_per_mm_x * (float)cfg.microsteps_x; }
static float effective_steps_per_mm_y() { return cfg.full_steps_per_mm_y * (float)cfg.microsteps_y; }
static int32_t mm_to_steps_x(float mm) { return lroundf(mm * effective_steps_per_mm_x()); }
static int32_t mm_to_steps_y(float mm) { return lroundf(mm * effective_steps_per_mm_y()); }
static float steps_to_mm_x(int32_t steps) { return ((float)steps) / effective_steps_per_mm_x(); }
static float steps_to_mm_y(int32_t steps) { return ((float)steps) / effective_steps_per_mm_y(); }

static void apply_motor_config() {
  const uint32_t speed_x = (uint32_t)lroundf(clampf(cfg.speed_mm_s, 0.1f, 500.0f) * effective_steps_per_mm_x());
  const uint32_t speed_y = (uint32_t)lroundf(clampf(cfg.speed_mm_s, 0.1f, 500.0f) * effective_steps_per_mm_y());
  const uint32_t accel_x = (uint32_t)lroundf(clampf(cfg.acceleration_mm_s2, 1.0f, 5000.0f) * effective_steps_per_mm_x());
  const uint32_t accel_y = (uint32_t)lroundf(clampf(cfg.acceleration_mm_s2, 1.0f, 5000.0f) * effective_steps_per_mm_y());

  if (x1_motor) {
    x1_motor->setSpeedInHz(speed_x);
    x1_motor->setAcceleration(accel_x);
    x1_motor->setDirectionPin(X1_DIR_PIN, cfg.invert_x1_dir);
    if (ENABLE_PIN >= 0) x1_motor->setEnablePin(ENABLE_PIN, false);
    cfg.enable_motors ? x1_motor->enableOutputs() : x1_motor->disableOutputs();
  }
  if (x2_motor) {
    x2_motor->setSpeedInHz(speed_x);
    x2_motor->setAcceleration(accel_x);
    x2_motor->setDirectionPin(X2_DIR_PIN, cfg.invert_x2_dir);
    if (ENABLE_PIN >= 0) x2_motor->setEnablePin(ENABLE_PIN, false);
    cfg.enable_motors ? x2_motor->enableOutputs() : x2_motor->disableOutputs();
  }
  if (y_motor) {
    y_motor->setSpeedInHz(speed_y);
    y_motor->setAcceleration(accel_y);
    y_motor->setDirectionPin(Y_DIR_PIN, cfg.invert_y_dir);
    if (ENABLE_PIN >= 0) y_motor->setEnablePin(ENABLE_PIN, false);
    cfg.enable_motors ? y_motor->enableOutputs() : y_motor->disableOutputs();
  }
}

static void emergency_stop_outputs() {
  if (x1_motor) { x1_motor->forceStop(); x1_motor->disableOutputs(); }
  if (x2_motor) { x2_motor->forceStop(); x2_motor->disableOutputs(); }
  if (y_motor) { y_motor->forceStop(); y_motor->disableOutputs(); }
}

static bool read_estop_active() {
  return cfg.enable_estop_monitor && (digitalRead(ESTOP_PIN) == HIGH);
}

static void set_target(float x_mm, float y_mm) {
  portENTER_CRITICAL(&state_mux);
  target_x_mm = clampf(x_mm, cfg.min_x_mm, cfg.max_x_mm);
  target_y_mm = clampf(y_mm, cfg.min_y_mm, cfg.max_y_mm);
  pending_motion_command = true;
  portEXIT_CRITICAL(&state_mux);
}

static void update_current_from_motors() {
  if (!x1_motor || !y_motor) return;
  portENTER_CRITICAL(&state_mux);
  current_x_mm = steps_to_mm_x(x1_motor->getCurrentPosition());
  current_y_mm = steps_to_mm_y(y_motor->getCurrentPosition());
  portEXIT_CRITICAL(&state_mux);
}

static void target_callback(const void *msg_in) {
  const auto *msg = (const geometry_msgs__msg__Point *)msg_in;
  set_target((float)msg->x, (float)msg->y);
}

static void x_callback(const void *msg_in) {
  const auto *msg = (const std_msgs__msg__Float32 *)msg_in;
  set_target(msg->data, target_y_mm);
}

static void y_callback(const void *msg_in) {
  const auto *msg = (const std_msgs__msg__Float32 *)msg_in;
  set_target(target_x_mm, msg->data);
}

static void speed_callback(const void *msg_in) {
  const auto *msg = (const std_msgs__msg__Float32 *)msg_in;
  portENTER_CRITICAL(&state_mux);
  cfg.speed_mm_s = clampf(msg->data, 0.1f, 500.0f);
  portEXIT_CRITICAL(&state_mux);
  apply_motor_config();
}

static void accel_callback(const void *msg_in) {
  const auto *msg = (const std_msgs__msg__Float32 *)msg_in;
  portENTER_CRITICAL(&state_mux);
  cfg.acceleration_mm_s2 = clampf(msg->data, 1.0f, 5000.0f);
  portEXIT_CRITICAL(&state_mux);
  apply_motor_config();
}

static void calibrate_callback(const void *msg_in) {
  const auto *msg = (const std_msgs__msg__Bool *)msg_in;
  if (msg->data) {
    calibration_requested = true;
  }
}


static void load_config_from_nvs() {
  if (!prefs.begin("gantry", true)) return;
  cfg.full_steps_per_mm_x = prefs.getFloat("full_spm_x", cfg.full_steps_per_mm_x);
  cfg.full_steps_per_mm_y = prefs.getFloat("full_spm_y", cfg.full_steps_per_mm_y);
  cfg.microsteps_x = clamp_microsteps(prefs.getInt("micro_x", cfg.microsteps_x));
  cfg.microsteps_y = clamp_microsteps(prefs.getInt("micro_y", cfg.microsteps_y));
  cfg.max_x_mm = prefs.getFloat("max_x", cfg.max_x_mm);
  cfg.max_y_mm = prefs.getFloat("max_y", cfg.max_y_mm);
  cfg.min_x_mm = prefs.getFloat("min_x", cfg.min_x_mm);
  cfg.min_y_mm = prefs.getFloat("min_y", cfg.min_y_mm);
  cfg.speed_mm_s = prefs.getFloat("speed", cfg.speed_mm_s);
  cfg.acceleration_mm_s2 = prefs.getFloat("accel", cfg.acceleration_mm_s2);
  cfg.calibration_fast_mm_s = prefs.getFloat("cal_fast", cfg.calibration_fast_mm_s);
  cfg.calibration_slow_mm_s = prefs.getFloat("cal_slow", cfg.calibration_slow_mm_s);
  cfg.calibration_backoff_mm = prefs.getFloat("cal_back", cfg.calibration_backoff_mm);
  cfg.calibration_search_mm = prefs.getFloat("cal_search", cfg.calibration_search_mm);
  cfg.x_calibration_offset_mm = prefs.getFloat("x_cal_off", cfg.x_calibration_offset_mm);
  cfg.y_calibration_offset_mm = prefs.getFloat("y_cal_off", cfg.y_calibration_offset_mm);
  cfg.invert_x1_dir = prefs.getBool("inv_x1", cfg.invert_x1_dir);
  cfg.invert_x2_dir = prefs.getBool("inv_x2", cfg.invert_x2_dir);
  cfg.invert_y_dir = prefs.getBool("inv_y", cfg.invert_y_dir);
  cfg.enable_homing = prefs.getBool("homing", cfg.enable_homing);
  cfg.enable_estop_monitor = prefs.getBool("estop_mon", cfg.enable_estop_monitor);
  cfg.enable_motors = prefs.getBool("motors", cfg.enable_motors);
  prefs.end();

  cfg.full_steps_per_mm_x = clampf(cfg.full_steps_per_mm_x, 0.001f, 10000.0f);
  cfg.full_steps_per_mm_y = clampf(cfg.full_steps_per_mm_y, 0.001f, 10000.0f);
  cfg.max_x_mm = clampf(cfg.max_x_mm, 1.0f, 100000.0f);
  cfg.max_y_mm = clampf(cfg.max_y_mm, 1.0f, 100000.0f);
  cfg.speed_mm_s = clampf(cfg.speed_mm_s, 0.1f, 500.0f);
  cfg.acceleration_mm_s2 = clampf(cfg.acceleration_mm_s2, 1.0f, 5000.0f);
  cfg.calibration_fast_mm_s = clampf(cfg.calibration_fast_mm_s, 0.1f, 200.0f);
  cfg.calibration_slow_mm_s = clampf(cfg.calibration_slow_mm_s, 0.1f, 50.0f);
  cfg.calibration_backoff_mm = clampf(cfg.calibration_backoff_mm, 0.1f, 100.0f);
  cfg.calibration_search_mm = clampf(cfg.calibration_search_mm, 1.0f, 100000.0f);
  target_x_mm = clampf(target_x_mm, cfg.min_x_mm, cfg.max_x_mm);
  target_y_mm = clampf(target_y_mm, cfg.min_y_mm, cfg.max_y_mm);
}

static void save_config_to_nvs() {
  if (!prefs.begin("gantry", false)) return;
  prefs.putFloat("full_spm_x", cfg.full_steps_per_mm_x);
  prefs.putFloat("full_spm_y", cfg.full_steps_per_mm_y);
  prefs.putInt("micro_x", cfg.microsteps_x);
  prefs.putInt("micro_y", cfg.microsteps_y);
  prefs.putFloat("max_x", cfg.max_x_mm);
  prefs.putFloat("max_y", cfg.max_y_mm);
  prefs.putFloat("min_x", cfg.min_x_mm);
  prefs.putFloat("min_y", cfg.min_y_mm);
  prefs.putFloat("speed", cfg.speed_mm_s);
  prefs.putFloat("accel", cfg.acceleration_mm_s2);
  prefs.putFloat("cal_fast", cfg.calibration_fast_mm_s);
  prefs.putFloat("cal_slow", cfg.calibration_slow_mm_s);
  prefs.putFloat("cal_back", cfg.calibration_backoff_mm);
  prefs.putFloat("cal_search", cfg.calibration_search_mm);
  prefs.putFloat("x_cal_off", cfg.x_calibration_offset_mm);
  prefs.putFloat("y_cal_off", cfg.y_calibration_offset_mm);
  prefs.putBool("inv_x1", cfg.invert_x1_dir);
  prefs.putBool("inv_x2", cfg.invert_x2_dir);
  prefs.putBool("inv_y", cfg.invert_y_dir);
  prefs.putBool("homing", cfg.enable_homing);
  prefs.putBool("estop_mon", cfg.enable_estop_monitor);
  prefs.putBool("motors", cfg.enable_motors);
  prefs.end();
}

static void clear_config_nvs() {
  if (!prefs.begin("gantry", false)) return;
  prefs.clear();
  prefs.end();
}


static void freeze_motion_at_current_position(float x_mm, float y_mm) {
  const int32_t x_steps = mm_to_steps_x(x_mm);
  const int32_t y_steps = mm_to_steps_y(y_mm);

  if (x1_motor) x1_motor->forceStopAndNewPosition(x_steps);
  if (x2_motor) x2_motor->forceStopAndNewPosition(x_steps);
  if (y_motor) y_motor->forceStopAndNewPosition(y_steps);

  portENTER_CRITICAL(&state_mux);
  current_x_mm = x_mm;
  current_y_mm = y_mm;
  target_x_mm = clampf(x_mm, cfg.min_x_mm, cfg.max_x_mm);
  target_y_mm = clampf(y_mm, cfg.min_y_mm, cfg.max_y_mm);
  active_target_x_mm = target_x_mm;
  active_target_y_mm = target_y_mm;
  pending_motion_command = false;
  portEXIT_CRITICAL(&state_mux);
}

static bool param_callback(const Parameter *old_param, const Parameter *new_param, void *context) {
  (void)old_param;
  (void)context;
  bool ok = true;
  if (new_param == nullptr) return false;
  config_update_active = true;

  // Parameter updates must not re-trigger the previous motion target.
  // Capture physical position using the OLD calibration before applying changes.
  update_current_from_motors();
  float freeze_x_mm;
  float freeze_y_mm;
  portENTER_CRITICAL(&state_mux);
  freeze_x_mm = current_x_mm;
  freeze_y_mm = current_y_mm;
  portEXIT_CRITICAL(&state_mux);

  portENTER_CRITICAL(&state_mux);
  if (strcmp(new_param->name.data, "reset_saved_config") == 0) {
    cfg.reset_saved_config = new_param->value.bool_value;
  }
  else if (strcmp(new_param->name.data, "steps_per_mm_x") == 0) cfg.full_steps_per_mm_x = clampf(new_param->value.double_value, 1.0f, 10000.0f) / (float)cfg.microsteps_x;
  else if (strcmp(new_param->name.data, "steps_per_mm_y") == 0) cfg.full_steps_per_mm_y = clampf(new_param->value.double_value, 1.0f, 10000.0f) / (float)cfg.microsteps_y;
  else if (strcmp(new_param->name.data, "full_steps_per_mm_x") == 0) cfg.full_steps_per_mm_x = clampf(new_param->value.double_value, 0.001f, 10000.0f);
  else if (strcmp(new_param->name.data, "full_steps_per_mm_y") == 0) cfg.full_steps_per_mm_y = clampf(new_param->value.double_value, 0.001f, 10000.0f);
  else if (strcmp(new_param->name.data, "microsteps_x") == 0) cfg.microsteps_x = clamp_microsteps(new_param->value.integer_value);
  else if (strcmp(new_param->name.data, "microsteps_y") == 0) cfg.microsteps_y = clamp_microsteps(new_param->value.integer_value);
  else if (strcmp(new_param->name.data, "max_x_mm") == 0) cfg.max_x_mm = clampf(new_param->value.double_value, 1.0f, 100000.0f);
  else if (strcmp(new_param->name.data, "max_y_mm") == 0) cfg.max_y_mm = clampf(new_param->value.double_value, 1.0f, 100000.0f);
  else if (strcmp(new_param->name.data, "min_x_mm") == 0) cfg.min_x_mm = new_param->value.double_value;
  else if (strcmp(new_param->name.data, "min_y_mm") == 0) cfg.min_y_mm = new_param->value.double_value;
  else if (strcmp(new_param->name.data, "speed_mm_s") == 0) cfg.speed_mm_s = clampf(new_param->value.double_value, 0.1f, 500.0f);
  else if (strcmp(new_param->name.data, "acceleration_mm_s2") == 0) cfg.acceleration_mm_s2 = clampf(new_param->value.double_value, 1.0f, 5000.0f);
  else if (strcmp(new_param->name.data, "calibration_fast_mm_s") == 0) cfg.calibration_fast_mm_s = clampf(new_param->value.double_value, 0.1f, 200.0f);
  else if (strcmp(new_param->name.data, "calibration_slow_mm_s") == 0) cfg.calibration_slow_mm_s = clampf(new_param->value.double_value, 0.1f, 50.0f);
  else if (strcmp(new_param->name.data, "calibration_backoff_mm") == 0) cfg.calibration_backoff_mm = clampf(new_param->value.double_value, 0.1f, 100.0f);
  else if (strcmp(new_param->name.data, "calibration_search_mm") == 0) cfg.calibration_search_mm = clampf(new_param->value.double_value, 1.0f, 100000.0f);
  else if (strcmp(new_param->name.data, "x_calibration_offset_mm") == 0) cfg.x_calibration_offset_mm = new_param->value.double_value;
  else if (strcmp(new_param->name.data, "y_calibration_offset_mm") == 0) cfg.y_calibration_offset_mm = new_param->value.double_value;
  else if (strcmp(new_param->name.data, "invert_x1_dir") == 0) cfg.invert_x1_dir = new_param->value.bool_value;
  else if (strcmp(new_param->name.data, "invert_x2_dir") == 0) cfg.invert_x2_dir = new_param->value.bool_value;
  else if (strcmp(new_param->name.data, "invert_y_dir") == 0) cfg.invert_y_dir = new_param->value.bool_value;
  else if (strcmp(new_param->name.data, "enable_homing") == 0) cfg.enable_homing = new_param->value.bool_value;
  else if (strcmp(new_param->name.data, "enable_estop_monitor") == 0) cfg.enable_estop_monitor = new_param->value.bool_value;
  else if (strcmp(new_param->name.data, "enable_motors") == 0) cfg.enable_motors = new_param->value.bool_value;
  else ok = false;
  target_x_mm = clampf(target_x_mm, cfg.min_x_mm, cfg.max_x_mm);
  target_y_mm = clampf(target_y_mm, cfg.min_y_mm, cfg.max_y_mm);
  portEXIT_CRITICAL(&state_mux);

  if (ok && cfg.reset_saved_config) {
    clear_config_nvs();
    cfg.reset_saved_config = false;
  } else if (ok) {
    save_config_to_nvs();
  }

  if (ok) {
    freeze_motion_at_current_position(freeze_x_mm, freeze_y_mm);
  }
  apply_motor_config();
  config_update_active = false;
  return ok;
}


static void stop_x_axis() {
  if (x1_motor) x1_motor->forceStop();
  if (x2_motor) x2_motor->forceStop();
}

static void stop_y_axis() {
  if (y_motor) y_motor->forceStop();
}

static bool moving_toward_x_limit(float requested_x_mm) {
  return requested_x_mm <= current_x_mm;
}

static bool moving_toward_y_limit(float requested_y_mm) {
  return requested_y_mm <= current_y_mm;
}

static void start_direct_move(float tx, float ty) {
  apply_motor_config();

  const int32_t x_steps = mm_to_steps_x(tx);
  const int32_t y_steps = mm_to_steps_y(ty);

  if (x1_motor) x1_motor->moveTo(x_steps);
  if (x2_motor) x2_motor->moveTo(x_steps);
  if (y_motor) y_motor->moveTo(y_steps);

  portENTER_CRITICAL(&state_mux);
  active_target_x_mm = tx;
  active_target_y_mm = ty;
  portEXIT_CRITICAL(&state_mux);
}


static void set_calibration_speed(float speed_mm_s) {
  const uint32_t speed_x = (uint32_t)lroundf(clampf(speed_mm_s, 0.1f, 200.0f) * effective_steps_per_mm_x());
  const uint32_t speed_y = (uint32_t)lroundf(clampf(speed_mm_s, 0.1f, 200.0f) * effective_steps_per_mm_y());
  const uint32_t accel_x = (uint32_t)lroundf(clampf(cfg.acceleration_mm_s2, 1.0f, 5000.0f) * effective_steps_per_mm_x());
  const uint32_t accel_y = (uint32_t)lroundf(clampf(cfg.acceleration_mm_s2, 1.0f, 5000.0f) * effective_steps_per_mm_y());
  if (x1_motor) { x1_motor->setSpeedInHz(speed_x); x1_motor->setAcceleration(accel_x); }
  if (x2_motor) { x2_motor->setSpeedInHz(speed_x); x2_motor->setAcceleration(accel_x); }
  if (y_motor) { y_motor->setSpeedInHz(speed_y); y_motor->setAcceleration(accel_y); }
}

static bool wait_until_stopped_or_estop(uint32_t timeout_ms) {
  const uint32_t start_ms = millis();
  while (millis() - start_ms < timeout_ms) {
    if (read_estop_active()) return false;
    const bool x_running = (x1_motor && x1_motor->isRunning()) || (x2_motor && x2_motor->isRunning());
    const bool y_running = y_motor && y_motor->isRunning();
    if (!x_running && !y_running) return true;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

static bool approach_x_limits(float speed_mm_s) {
  set_calibration_speed(speed_mm_s);
  const int32_t search_steps = mm_to_steps_x(cfg.min_x_mm - cfg.calibration_search_mm);
  bool x1_done = false;
  bool x2_done = false;
  uint32_t x1_first_high = 0;
  uint32_t x2_first_high = 0;
  const uint32_t start_ms = millis();
  const uint32_t timeout_ms = (uint32_t)((cfg.calibration_search_mm / fmaxf(speed_mm_s, 0.1f)) * 1000.0f) + 10000;

  if (x1_motor) x1_motor->moveTo(search_steps);
  if (x2_motor) x2_motor->moveTo(search_steps);

  while (millis() - start_ms < timeout_ms) {
    if (read_estop_active()) { stop_x_axis(); return false; }

    const bool x1_hit = digitalRead(X1_HOME_PIN) == HIGH;
    const bool x2_hit = digitalRead(X2_HOME_PIN) == HIGH;

    if (x1_hit && !x1_done) {
      if (x1_first_high == 0) { x1_first_high = micros(); if (x1_motor) x1_motor->forceStop(); }
      if ((micros() - x1_first_high) >= 3000) x1_done = true;
    } else if (!x1_hit) {
      x1_first_high = 0;
    }

    if (x2_hit && !x2_done) {
      if (x2_first_high == 0) { x2_first_high = micros(); if (x2_motor) x2_motor->forceStop(); }
      if ((micros() - x2_first_high) >= 3000) x2_done = true;
    } else if (!x2_hit) {
      x2_first_high = 0;
    }

    if (x1_done && x2_done) return true;

    if ((!x1_motor || !x1_motor->isRunning()) && !x1_done) return false;
    if ((!x2_motor || !x2_motor->isRunning()) && !x2_done) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  stop_x_axis();
  return false;
}

static bool approach_y_limit(float speed_mm_s) {
  set_calibration_speed(speed_mm_s);
  const int32_t search_steps = mm_to_steps_y(cfg.min_y_mm - cfg.calibration_search_mm);
  bool y_done = false;
  uint32_t y_first_high = 0;
  const uint32_t start_ms = millis();
  const uint32_t timeout_ms = (uint32_t)((cfg.calibration_search_mm / fmaxf(speed_mm_s, 0.1f)) * 1000.0f) + 10000;

  if (y_motor) y_motor->moveTo(search_steps);

  while (millis() - start_ms < timeout_ms) {
    if (read_estop_active()) { stop_y_axis(); return false; }
    const bool y_hit = digitalRead(Y_HOME_PIN) == HIGH;
    if (y_hit && !y_done) {
      if (y_first_high == 0) { y_first_high = micros(); if (y_motor) y_motor->forceStop(); }
      if ((micros() - y_first_high) >= 3000) y_done = true;
    } else if (!y_hit) {
      y_first_high = 0;
    }
    if (y_done) return true;
    if ((!y_motor || !y_motor->isRunning()) && !y_done) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  stop_y_axis();
  return false;
}

static bool backoff_from_limits() {
  set_calibration_speed(cfg.calibration_fast_mm_s);
  const int32_t x_backoff_steps = mm_to_steps_x(cfg.min_x_mm + cfg.calibration_backoff_mm);
  const int32_t y_backoff_steps = mm_to_steps_y(cfg.min_y_mm + cfg.calibration_backoff_mm);

  if (x1_motor) x1_motor->forceStopAndNewPosition(mm_to_steps_x(cfg.min_x_mm));
  if (x2_motor) x2_motor->forceStopAndNewPosition(mm_to_steps_x(cfg.min_x_mm));
  if (y_motor) y_motor->forceStopAndNewPosition(mm_to_steps_y(cfg.min_y_mm));

  if (x1_motor) x1_motor->moveTo(x_backoff_steps);
  if (x2_motor) x2_motor->moveTo(x_backoff_steps);
  if (y_motor) y_motor->moveTo(y_backoff_steps);
  return wait_until_stopped_or_estop(30000);
}

static bool run_calibration_sequence() {
  calibration_active = true;
  config_update_active = true;
  stop_x_axis();
  stop_y_axis();
  apply_motor_config();

  bool ok = true;
  ok = ok && approach_x_limits(cfg.calibration_fast_mm_s);
  ok = ok && approach_y_limit(cfg.calibration_fast_mm_s);
  ok = ok && backoff_from_limits();
  ok = ok && approach_x_limits(cfg.calibration_slow_mm_s);
  ok = ok && approach_y_limit(cfg.calibration_slow_mm_s);

  if (ok) {
    const float calibrated_x = cfg.min_x_mm + cfg.x_calibration_offset_mm;
    const float calibrated_y = cfg.min_y_mm + cfg.y_calibration_offset_mm;
    if (x1_motor) x1_motor->forceStopAndNewPosition(mm_to_steps_x(calibrated_x));
    if (x2_motor) x2_motor->forceStopAndNewPosition(mm_to_steps_x(calibrated_x));
    if (y_motor) y_motor->forceStopAndNewPosition(mm_to_steps_y(calibrated_y));
    portENTER_CRITICAL(&state_mux);
    current_x_mm = calibrated_x;
    current_y_mm = calibrated_y;
    target_x_mm = calibrated_x;
    target_y_mm = calibrated_y;
    active_target_x_mm = calibrated_x;
    active_target_y_mm = calibrated_y;
    pending_motion_command = false;
    x_limit_fault = false;
    y_limit_fault = false;
    portEXIT_CRITICAL(&state_mux);
  }

  apply_motor_config();
  config_update_active = false;
  calibration_active = false;
  return ok;
}

static void motion_task(void *) {
  uint32_t last_feedback = 0;
  for (;;) {
    if (calibration_requested && !calibration_active) {
      calibration_requested = false;
      run_calibration_sequence();
    }

    if (config_update_active) {
      vTaskDelay(pdMS_TO_TICKS(MOTION_PERIOD_MS));
      continue;
    }

    const bool estop_now = read_estop_active();
    portENTER_CRITICAL(&state_mux);
    estop_active = estop_now;
    portEXIT_CRITICAL(&state_mux);
    if (estop_now) {
      emergency_stop_outputs();
      vTaskDelay(pdMS_TO_TICKS(MOTION_PERIOD_MS));
      continue;
    } else if (cfg.enable_motors) {
      if (x1_motor) x1_motor->enableOutputs();
      if (x2_motor) x2_motor->enableOutputs();
      if (y_motor) y_motor->enableOutputs();
    }

    update_current_from_motors();

    const bool x1_home = (digitalRead(X1_HOME_PIN) == HIGH);
    const bool x2_home = (digitalRead(X2_HOME_PIN) == HIGH);
    const bool y_home = (digitalRead(Y_HOME_PIN) == HIGH);

    if (cfg.enable_homing && (x1_home || x2_home)) {
      stop_x_axis();
      portENTER_CRITICAL(&state_mux);
      x_limit_fault = true;
      target_x_mm = current_x_mm;
      active_target_x_mm = current_x_mm;
      portEXIT_CRITICAL(&state_mux);
    }
    if (cfg.enable_homing && y_home) {
      stop_y_axis();
      portENTER_CRITICAL(&state_mux);
      y_limit_fault = true;
      target_y_mm = current_y_mm;
      active_target_y_mm = current_y_mm;
      portEXIT_CRITICAL(&state_mux);
    }

    float tx, ty, cx, cy;
    bool pending;
    portENTER_CRITICAL(&state_mux);
    tx = target_x_mm;
    ty = target_y_mm;
    cx = current_x_mm;
    cy = current_y_mm;
    pending = pending_motion_command;
    pending_motion_command = false;
    portEXIT_CRITICAL(&state_mux);

    if (x_limit_fault && !moving_toward_x_limit(tx)) {
      x_limit_fault = false;
    }
    if (y_limit_fault && !moving_toward_y_limit(ty)) {
      y_limit_fault = false;
    }

    if (pending) {
      if (x_limit_fault && moving_toward_x_limit(tx)) {
        tx = cx;
      }
      if (y_limit_fault && moving_toward_y_limit(ty)) {
        ty = cy;
      }
      start_direct_move(tx, ty);
    }

    if (millis() - last_feedback >= FEEDBACK_PERIOD_MS) {
      update_current_from_motors();
      last_feedback = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(MOTION_PERIOD_MS));
  }
}

static bool init_motors() {
  load_config_from_nvs();
  pinMode(X1_HOME_PIN, INPUT_PULLUP);
  pinMode(X2_HOME_PIN, INPUT_PULLUP);
  pinMode(Y_HOME_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  engine.init();
  x1_motor = engine.stepperConnectToPin(X1_STEP_PIN);
  x2_motor = engine.stepperConnectToPin(X2_STEP_PIN);
  y_motor = engine.stepperConnectToPin(Y_STEP_PIN);
  if (!x1_motor || !x2_motor || !y_motor) return false;

  x1_motor->setDirectionPin(X1_DIR_PIN, cfg.invert_x1_dir);
  x2_motor->setDirectionPin(X2_DIR_PIN, cfg.invert_x2_dir);
  y_motor->setDirectionPin(Y_DIR_PIN, cfg.invert_y_dir);
  apply_motor_config();
  return true;
}

static void add_parameters() {
  rclc_add_parameter(&param_server, "steps_per_mm_x", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "steps_per_mm_y", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "full_steps_per_mm_x", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "full_steps_per_mm_y", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "microsteps_x", RCLC_PARAMETER_INT);
  rclc_add_parameter(&param_server, "microsteps_y", RCLC_PARAMETER_INT);
  rclc_add_parameter(&param_server, "max_x_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "max_y_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "min_x_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "min_y_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "speed_mm_s", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "acceleration_mm_s2", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "calibration_fast_mm_s", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "calibration_slow_mm_s", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "calibration_backoff_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "calibration_search_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "x_calibration_offset_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "y_calibration_offset_mm", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&param_server, "invert_x1_dir", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "invert_x2_dir", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "invert_y_dir", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "enable_homing", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "enable_estop_monitor", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "enable_motors", RCLC_PARAMETER_BOOL);
  rclc_add_parameter(&param_server, "reset_saved_config", RCLC_PARAMETER_BOOL);

  rclc_parameter_set_double(&param_server, "steps_per_mm_x", effective_steps_per_mm_x());
  rclc_parameter_set_double(&param_server, "steps_per_mm_y", effective_steps_per_mm_y());
  rclc_parameter_set_double(&param_server, "full_steps_per_mm_x", cfg.full_steps_per_mm_x);
  rclc_parameter_set_double(&param_server, "full_steps_per_mm_y", cfg.full_steps_per_mm_y);
  rclc_parameter_set_int(&param_server, "microsteps_x", cfg.microsteps_x);
  rclc_parameter_set_int(&param_server, "microsteps_y", cfg.microsteps_y);
  rclc_parameter_set_double(&param_server, "max_x_mm", cfg.max_x_mm);
  rclc_parameter_set_double(&param_server, "max_y_mm", cfg.max_y_mm);
  rclc_parameter_set_double(&param_server, "min_x_mm", cfg.min_x_mm);
  rclc_parameter_set_double(&param_server, "min_y_mm", cfg.min_y_mm);
  rclc_parameter_set_double(&param_server, "speed_mm_s", cfg.speed_mm_s);
  rclc_parameter_set_double(&param_server, "acceleration_mm_s2", cfg.acceleration_mm_s2);
  rclc_parameter_set_double(&param_server, "calibration_fast_mm_s", cfg.calibration_fast_mm_s);
  rclc_parameter_set_double(&param_server, "calibration_slow_mm_s", cfg.calibration_slow_mm_s);
  rclc_parameter_set_double(&param_server, "calibration_backoff_mm", cfg.calibration_backoff_mm);
  rclc_parameter_set_double(&param_server, "calibration_search_mm", cfg.calibration_search_mm);
  rclc_parameter_set_double(&param_server, "x_calibration_offset_mm", cfg.x_calibration_offset_mm);
  rclc_parameter_set_double(&param_server, "y_calibration_offset_mm", cfg.y_calibration_offset_mm);
  rclc_parameter_set_bool(&param_server, "invert_x1_dir", cfg.invert_x1_dir);
  rclc_parameter_set_bool(&param_server, "invert_x2_dir", cfg.invert_x2_dir);
  rclc_parameter_set_bool(&param_server, "invert_y_dir", cfg.invert_y_dir);
  rclc_parameter_set_bool(&param_server, "enable_homing", cfg.enable_homing);
  rclc_parameter_set_bool(&param_server, "enable_estop_monitor", cfg.enable_estop_monitor);
  rclc_parameter_set_bool(&param_server, "enable_motors", cfg.enable_motors);
  rclc_parameter_set_bool(&param_server, "reset_saved_config", false);
}

static bool init_ros() {
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "gantry_controller", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&position_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "current_position") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&target_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "target_position") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&x_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "x_position") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&y_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "y_position") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&speed_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "speed") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&accel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "acceleration") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&calibrate_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "calibrate") != RCL_RET_OK) return false;

  const rclc_parameter_options_t options = {
    .notify_changed_over_dds = true,
    .max_params = 32,
    .allow_undeclared_parameters = false,
    .low_mem_mode = false
  };
  if (rclc_parameter_server_init_with_option(&param_server, &node, &options) != RCL_RET_OK) return false;
  add_parameters();
  if (rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 6, &allocator) != RCL_RET_OK) return false;
  rclc_executor_add_subscription(&executor, &target_sub, &target_msg, &target_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &x_sub, &x_msg, &x_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &y_sub, &y_msg, &y_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &speed_sub, &speed_msg, &speed_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &accel_sub, &accel_msg, &accel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &calibrate_sub, &calibrate_msg, &calibrate_callback, ON_NEW_DATA);
  rclc_executor_add_parameter_server(&executor, &param_server, param_callback);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  if (!init_motors()) {
    Serial.println("Motor init failed");
    while (true) delay(1000);
  }
  while (!init_ros()) {
    Serial.println("micro-ROS init failed, retrying...");
    delay(2000);
  }
  xTaskCreatePinnedToCore(motion_task, "motion", 8192, NULL, 2, NULL, 1);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ROS_SPIN_PERIOD_MS));

  static uint32_t last_pub = 0;
  if (millis() - last_pub >= FEEDBACK_PERIOD_MS) {
    update_current_from_motors();
    portENTER_CRITICAL(&state_mux);
    current_position_msg.x = current_x_mm;
    current_position_msg.y = current_y_mm;
    current_position_msg.z = estop_active ? 1.0 : 0.0;
    portEXIT_CRITICAL(&state_mux);
    rcl_publish(&position_pub, &current_position_msg, NULL);
    last_pub = millis();
  }
}
