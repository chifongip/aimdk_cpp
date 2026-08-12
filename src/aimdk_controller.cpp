#include "aimdk_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace {
std::atomic<int> g_controller_count{0};
std::mutex g_rclcpp_mutex;

void init_rclcpp_once() {
  std::lock_guard<std::mutex> lock(g_rclcpp_mutex);
  if (std::getenv("ROS_LOG_DIR") == nullptr) {
    std::filesystem::create_directories("/tmp/robojudo_ros_logs");
    setenv("ROS_LOG_DIR", "/tmp/robojudo_ros_logs", 0);
  }
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  ++g_controller_count;
}

void shutdown_rclcpp_if_last() {
  std::lock_guard<std::mutex> lock(g_rclcpp_mutex);
  if (--g_controller_count == 0 && rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void validate_gains(const std::vector<double>& stiffness,
                    const std::vector<double>& damping,
                    size_t expected_size) {
  if (stiffness.size() != expected_size || damping.size() != expected_size) {
    throw std::invalid_argument("stiffness and damping must match joint_names length");
  }
  for (double value : stiffness) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("stiffness must contain only finite, non-negative values");
    }
  }
  for (double value : damping) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("damping must contain only finite, non-negative values");
    }
  }
}
}  // namespace

AimdkController::AimdkController(const AimdkConfig& cfg)
    : cfg_(cfg),
      state_(cfg.joint_names.size()),
      stiffness_(cfg.stiffness),
      damping_(cfg.damping),
      latest_positions_(cfg.joint_names.size(), 0.0) {
  if (!std::isfinite(cfg_.control_dt) || !std::isfinite(cfg_.publish_dt) ||
      !std::isfinite(cfg_.command_timeout) || !std::isfinite(cfg_.state_timeout) ||
      !std::isfinite(cfg_.odometry_timeout) ||
      !std::isfinite(cfg_.shutdown_publish_duration) || !std::isfinite(cfg_.shutdown_damping) ||
      cfg_.control_dt <= 0.0 || cfg_.publish_dt <= 0.0 || cfg_.command_timeout <= 0.0 ||
      cfg_.state_timeout <= 0.0 || cfg_.odometry_timeout <= 0.0 ||
      cfg_.shutdown_publish_duration < 0.0 || cfg_.shutdown_damping < 0.0) {
    throw std::invalid_argument("AimDK timing values must be positive and damping values must be non-negative");
  }
  validate_gains(cfg_.stiffness, cfg_.damping, cfg_.joint_names.size());
  if (cfg_.enable_odometry && cfg_.odometry_topic.empty()) {
    throw std::invalid_argument("odometry_topic must not be empty when odometry is enabled");
  }

  std::set<std::string> unique_joint_names(cfg_.joint_names.begin(), cfg_.joint_names.end());
  if (unique_joint_names.size() != cfg_.joint_names.size()) {
    throw std::invalid_argument("joint_names must not contain duplicates");
  }

  for (size_t i = 0; i < cfg_.joint_names.size(); ++i) {
    joint_index_[cfg_.joint_names[i]] = i;
    command_joint_names_.insert(cfg_.joint_names[i]);
  }

  std::set<std::string> grouped_joint_names;
  const std::vector<const std::vector<std::string>*> groups = {
      &cfg_.leg_joint_names, &cfg_.waist_joint_names, &cfg_.arm_joint_names, &cfg_.head_joint_names};
  for (const auto* group : groups) {
    for (const auto& name : *group) {
      if (joint_index_.find(name) == joint_index_.end()) {
        throw std::invalid_argument("joint group contains unknown name: " + name);
      }
      if (!grouped_joint_names.insert(name).second) {
        throw std::invalid_argument("joint appears in multiple groups: " + name);
      }
    }
  }
  if (grouped_joint_names != unique_joint_names) {
    throw std::invalid_argument("joint groups must contain every configured joint exactly once");
  }

  init_rclcpp_once();
  rclcpp_registered_ = true;
  try {
    node_ = rclcpp::Node::make_shared(cfg_.node_name);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto qos = rclcpp::SensorDataQoS();

    leg_sub_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
        cfg_.leg_state_topic, qos,
        [this](const aimdk_msgs::msg::JointStateArray::SharedPtr msg) { joint_callback(msg); });
    waist_sub_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
        cfg_.waist_state_topic, qos,
        [this](const aimdk_msgs::msg::JointStateArray::SharedPtr msg) { joint_callback(msg); });
    arm_sub_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
        cfg_.arm_state_topic, qos,
        [this](const aimdk_msgs::msg::JointStateArray::SharedPtr msg) { joint_callback(msg); });
    head_sub_ = node_->create_subscription<aimdk_msgs::msg::JointStateArray>(
        cfg_.head_state_topic, qos,
        [this](const aimdk_msgs::msg::JointStateArray::SharedPtr msg) { joint_callback(msg); });
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        cfg_.base_imu_topic, qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });
    if (cfg_.enable_odometry) {
      // The X2 odometry publisher is a live sensor stream and uses volatile
      // durability. Requesting transient-local here would make DDS reject the
      // endpoint match, leaving odometry_received_ false indefinitely.
      auto odometry_qos = rclcpp::SensorDataQoS();
      odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          cfg_.odometry_topic, odometry_qos,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odometry_callback(msg); });
    }

    if (cfg_.act) {
      leg_pub_ = node_->create_publisher<aimdk_msgs::msg::JointCommandArray>(cfg_.leg_command_topic, qos);
      waist_pub_ = node_->create_publisher<aimdk_msgs::msg::JointCommandArray>(cfg_.waist_command_topic, qos);
      arm_pub_ = node_->create_publisher<aimdk_msgs::msg::JointCommandArray>(cfg_.arm_command_topic, qos);
      head_pub_ = node_->create_publisher<aimdk_msgs::msg::JointCommandArray>(cfg_.head_command_topic, qos);
    }

    executor_->add_node(node_);
    running_ = true;
    spin_thread_ = std::thread([this]() {
      while (running_ && rclcpp::ok()) {
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    if (cfg_.act) {
      publish_running_ = true;
      publish_thread_ = std::thread(&AimdkController::publish_loop, this);
    }
  } catch (...) {
    publish_running_ = false;
    running_ = false;
    if (publish_thread_.joinable()) publish_thread_.join();
    if (spin_thread_.joinable()) spin_thread_.join();
    if (node_ && executor_) executor_->remove_node(node_);
    shutdown_rclcpp_if_last();
    rclcpp_registered_ = false;
    throw;
  }
}

AimdkController::~AimdkController() {
  shutdown();
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (node_ && executor_) {
    executor_->remove_node(node_);
  }
  if (rclcpp_registered_) {
    shutdown_rclcpp_if_last();
    rclcpp_registered_ = false;
  }
}

bool AimdkController::self_check(double timeout_sec) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (state_is_fresh(cfg_.state_timeout)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!cfg_.act) {
    return true;
  }
  return state_is_fresh(cfg_.state_timeout);
}

bool AimdkController::state_is_fresh(double timeout_sec) {
  if (!cfg_.act) {
    return true;
  }
  return get_state_freshness_report(timeout_sec).required_streams_fresh;
}

StateFreshnessReport AimdkController::get_state_freshness_report(double timeout_sec) {
  if (timeout_sec <= 0.0) {
    throw std::invalid_argument("state freshness timeout must be positive");
  }

  StateFreshnessReport report;
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = std::chrono::steady_clock::now();
  report.imu_received = imu_received_;
  if (imu_received_) {
    report.imu_age_sec = std::chrono::duration<double>(now - imu_update_time_).count();
    if (*report.imu_age_sec > timeout_sec) {
      report.reasons.push_back("imu_stale");
    }
  } else {
    report.reasons.push_back("imu_missing");
  }

  for (const auto& name : cfg_.joint_names) {
    const auto it = joint_update_times_.find(name);
    if (it == joint_update_times_.end()) {
      report.missing_joint_names.push_back(name);
      continue;
    }
    const double age_sec = std::chrono::duration<double>(now - it->second).count();
    report.joint_age_sec[name] = age_sec;
    if (age_sec > timeout_sec) {
      report.stale_joint_names.push_back(name);
    }
  }
  if (!report.missing_joint_names.empty()) report.reasons.push_back("joints_missing");
  if (!report.stale_joint_names.empty()) report.reasons.push_back("joints_stale");

  report.odometry_required = cfg_.enable_odometry;
  report.odometry_received = odometry_received_;
  report.odometry_valid = state_.odometry_state.valid;
  report.odometry_degenerate = state_.odometry_state.degenerate;
  if (odometry_received_) {
    report.odometry_age_sec = std::chrono::duration<double>(now - odometry_update_time_).count();
  }
  report.last_odometry_rejection_reason = last_odometry_rejection_reason_;
  if (!last_odometry_rejection_reason_.empty()) {
    report.last_odometry_rejection_age_sec =
        std::chrono::duration<double>(now - last_odometry_rejection_time_).count();
  }
  if (cfg_.enable_odometry) {
    if (!odometry_received_) {
      report.reasons.push_back("odometry_missing");
    } else if (!state_.odometry_state.valid) {
      report.reasons.push_back("odometry_invalid");
    } else if (*report.odometry_age_sec > cfg_.odometry_timeout) {
      report.reasons.push_back("odometry_stale");
    }
  }

  report.required_streams_fresh = report.reasons.empty();
  return report;
}

RobotState AimdkController::get_robot_state() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

void AimdkController::step(const std::vector<double>& positions) {
  if (!cfg_.act) {
    return;
  }
  if (positions.size() != cfg_.joint_names.size()) {
    throw std::invalid_argument("positions must match joint_names length");
  }
  for (double position : positions) {
    if (!std::isfinite(position)) {
      throw std::invalid_argument("positions must contain only finite values");
    }
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (watchdog_tripped_) {
    throw std::runtime_error("AimDK position watchdog is latched; re-arm position control before sending targets");
  }
  latest_positions_ = positions;
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
  command_mode_ = AimdkCommandMode::POSITION;
}

void AimdkController::arm_position_control() {
  if (!cfg_.act) {
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  watchdog_tripped_ = false;
  command_received_ = false;
  command_mode_ = AimdkCommandMode::IDLE;
}

void AimdkController::set_passive() {
  if (!cfg_.act) {
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_mode_ = AimdkCommandMode::PASSIVE;
  command_received_ = true;
}

void AimdkController::set_damping(double damping) {
  if (damping < 0.0 || !std::isfinite(damping)) {
    throw std::invalid_argument("damping must be finite and non-negative");
  }
  if (!cfg_.act) {
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  mode_damping_ = damping;
  command_mode_ = AimdkCommandMode::DAMPING;
  command_received_ = true;
}

void AimdkController::set_gains(const std::vector<double>& stiffness, const std::vector<double>& damping) {
  validate_gains(stiffness, damping, cfg_.joint_names.size());
  std::lock_guard<std::mutex> lock(command_mutex_);
  stiffness_ = stiffness;
  damping_ = damping;
}

void AimdkController::set_control_joint_names(const std::vector<std::string>& joint_names) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_joint_names_.clear();
  for (const auto& name : joint_names) {
    if (joint_index_.find(name) == joint_index_.end()) {
      throw std::invalid_argument("control joint name is not in joint_names: " + name);
    }
    command_joint_names_.insert(name);
  }
}

void AimdkController::shutdown() {
  if (shutdown_started_.exchange(true)) {
    return;
  }
  publish_running_ = false;
  if (publish_thread_.joinable() && publish_thread_.get_id() != std::this_thread::get_id()) {
    publish_thread_.join();
  }

  if (cfg_.act && rclcpp::ok()) {
    command_mode_ = AimdkCommandMode::DAMPING;
    mode_damping_ = cfg_.shutdown_damping;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(cfg_.shutdown_publish_duration);
    while (std::chrono::steady_clock::now() < deadline) {
      publish_damping_commands();
      std::this_thread::sleep_for(std::chrono::duration<double>(cfg_.publish_dt));
    }
  }
  running_ = false;
}

void AimdkController::joint_callback(const aimdk_msgs::msg::JointStateArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto update_time = std::chrono::steady_clock::now();
  for (const auto& joint : msg->joints) {
    auto it = joint_index_.find(joint.name);
    if (it == joint_index_.end()) {
      continue;
    }
    const size_t idx = it->second;
    state_.motor_state.q[idx] = static_cast<float>(joint.position);
    state_.motor_state.dq[idx] = static_cast<float>(joint.velocity);
    state_.motor_state.tau_est[idx] = static_cast<float>(joint.effort);
    received_joint_names_.insert(joint.name);
    joint_update_times_[joint.name] = update_time;
  }
  ++state_.tick;
}

void AimdkController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  imu_received_ = true;
  imu_update_time_ = std::chrono::steady_clock::now();
  state_.imu_state.quaternion = {
      static_cast<float>(msg->orientation.x),
      static_cast<float>(msg->orientation.y),
      static_cast<float>(msg->orientation.z),
      static_cast<float>(msg->orientation.w),
  };
  state_.imu_state.gyroscope = {
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z),
  };
  state_.imu_state.accelerometer = {
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z),
  };
}

void AimdkController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& position = msg->pose.pose.position;
  const auto& orientation = msg->pose.pose.orientation;
  const auto& linear = msg->twist.twist.linear;
  const auto& angular = msg->twist.twist.angular;
  const double quaternion_norm =
      std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
  const bool finite =
      std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
      std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
      std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
      std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z) &&
      std::isfinite(angular.x) && std::isfinite(angular.y) && std::isfinite(angular.z);
  if (!finite || !std::isfinite(quaternion_norm) || quaternion_norm < 1e-6) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_odometry_rejection_reason_ = "invalid_values_or_quaternion";
      last_odometry_rejection_time_ = std::chrono::steady_clock::now();
    }
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Ignoring invalid AimDK odometry sample from %s", cfg_.odometry_topic.c_str());
    return;
  }
  if ((!cfg_.odometry_parent_frame.empty() && msg->header.frame_id != cfg_.odometry_parent_frame) ||
      (!cfg_.odometry_child_frame.empty() && msg->child_frame_id != cfg_.odometry_child_frame)) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_odometry_rejection_reason_ = "frame_mismatch";
      last_odometry_rejection_time_ = std::chrono::steady_clock::now();
    }
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Ignoring odometry with frames %s -> %s; expected %s -> %s",
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
        cfg_.odometry_parent_frame.c_str(), cfg_.odometry_child_frame.c_str());
    return;
  }
  const bool degenerate = msg->pose.covariance[0] >= 0.5;

  std::lock_guard<std::mutex> lock(state_mutex_);
  odometry_received_ = true;
  odometry_update_time_ = std::chrono::steady_clock::now();
  state_.odometry_state.valid = !degenerate;
  state_.odometry_state.degenerate = degenerate;
  ++state_.odometry_state.sequence;
  state_.odometry_state.stamp_sec = msg->header.stamp.sec;
  state_.odometry_state.stamp_nanosec = msg->header.stamp.nanosec;
  state_.odometry_state.frame_id = msg->header.frame_id;
  state_.odometry_state.child_frame_id = msg->child_frame_id;
  state_.odometry_state.position = {
      static_cast<float>(position.x),
      static_cast<float>(position.y),
      static_cast<float>(position.z),
  };
  state_.odometry_state.quaternion = {
      static_cast<float>(orientation.x / quaternion_norm),
      static_cast<float>(orientation.y / quaternion_norm),
      static_cast<float>(orientation.z / quaternion_norm),
      static_cast<float>(orientation.w / quaternion_norm),
  };
  state_.odometry_state.linear_velocity = {
      static_cast<float>(linear.x),
      static_cast<float>(linear.y),
      static_cast<float>(linear.z),
  };
  state_.odometry_state.angular_velocity = {
      static_cast<float>(angular.x),
      static_cast<float>(angular.y),
      static_cast<float>(angular.z),
  };
  state_.odometry_state.pose_covariance.assign(
      msg->pose.covariance.begin(), msg->pose.covariance.end());
}

void AimdkController::publish_loop() {
  auto next_publish = std::chrono::steady_clock::now();
  while (publish_running_ && rclcpp::ok()) {
    next_publish += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cfg_.publish_dt));

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_received_) {
        const bool position_timed_out =
            command_mode_ == AimdkCommandMode::POSITION &&
            std::chrono::steady_clock::now() - last_command_time_ >
                std::chrono::duration<double>(cfg_.command_timeout);
        if (position_timed_out) {
          watchdog_tripped_ = true;
          command_mode_ = AimdkCommandMode::DAMPING;
          mode_damping_ = cfg_.shutdown_damping;
          publish_damping_commands();
        } else if (command_mode_ == AimdkCommandMode::PASSIVE) {
          publish_passive_commands();
        } else if (command_mode_ == AimdkCommandMode::DAMPING) {
          publish_damping_commands();
        } else if (command_mode_ == AimdkCommandMode::POSITION) {
          publish_group(cfg_.leg_joint_names, latest_positions_, stiffness_, damping_, true, leg_pub_);
          publish_group(cfg_.waist_joint_names, latest_positions_, stiffness_, damping_, true, waist_pub_);
          publish_group(cfg_.arm_joint_names, latest_positions_, stiffness_, damping_, true, arm_pub_);
          publish_group(cfg_.head_joint_names, latest_positions_, stiffness_, damping_, true, head_pub_);
        }
      }
    }
    std::this_thread::sleep_until(next_publish);
  }
}

void AimdkController::publish_passive_commands() {
  const std::vector<double> zeros(cfg_.joint_names.size(), 0.0);
  publish_group(cfg_.leg_joint_names, zeros, zeros, zeros, false, leg_pub_);
  publish_group(cfg_.waist_joint_names, zeros, zeros, zeros, false, waist_pub_);
  publish_group(cfg_.arm_joint_names, zeros, zeros, zeros, false, arm_pub_);
  publish_group(cfg_.head_joint_names, zeros, zeros, zeros, false, head_pub_);
}

void AimdkController::publish_damping_commands() {
  const std::vector<double> positions(cfg_.joint_names.size(), 0.0);
  const std::vector<double> stiffness(cfg_.joint_names.size(), 0.0);
  const double damping_value = command_mode_ == AimdkCommandMode::DAMPING ? mode_damping_ : cfg_.shutdown_damping;
  const std::vector<double> damping(cfg_.joint_names.size(), damping_value);
  publish_group(cfg_.leg_joint_names, positions, stiffness, damping, false, leg_pub_);
  publish_group(cfg_.waist_joint_names, positions, stiffness, damping, false, waist_pub_);
  publish_group(cfg_.arm_joint_names, positions, stiffness, damping, false, arm_pub_);
  publish_group(cfg_.head_joint_names, positions, stiffness, damping, false, head_pub_);
}

void AimdkController::publish_group(
    const std::vector<std::string>& names,
    const std::vector<double>& positions,
    const std::vector<double>& stiffness,
    const std::vector<double>& damping,
    bool controlled_only,
    const rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr& pub) {
  if (names.empty() || !pub) {
    return;
  }

  aimdk_msgs::msg::JointCommandArray msg;
  msg.joints.reserve(names.size());
  for (const auto& name : names) {
    auto it = joint_index_.find(name);
    if (it == joint_index_.end()) {
      continue;
    }
    if (controlled_only && command_joint_names_.find(name) == command_joint_names_.end()) {
      continue;
    }
    const size_t idx = it->second;
    aimdk_msgs::msg::JointCommand cmd;
    cmd.name = name;
    cmd.position = positions[idx];
    cmd.velocity = 0.0;
    cmd.effort = 0.0;
    cmd.stiffness = stiffness[idx];
    cmd.damping = damping[idx];
    msg.joints.push_back(cmd);
  }
  if (!msg.joints.empty()) {
    pub->publish(msg);
  }
}
