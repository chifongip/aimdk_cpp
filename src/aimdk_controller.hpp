#pragma once

#include <aimdk_msgs/msg/joint_command_array.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

struct AimdkConfig {
  bool act{true};
  std::string node_name{"robojudo_aimdk_cpp"};
  double control_dt{0.02};
  double publish_dt{0.002};
  double command_timeout{0.1};
  double shutdown_damping{5.0};
  double shutdown_publish_duration{0.2};
  double state_timeout{0.1};
  double odometry_timeout{0.1};
  double telemetry_window_sec{1.0};
  std::string base_imu_topic{"/aima/hal/imu/torso/state"};
  bool enable_odometry{false};
  std::string odometry_topic{"/aima/mc/leg_odometry"};
  std::string odometry_parent_frame{};
  std::string odometry_child_frame{};

  std::string leg_state_topic{"/aima/hal/joint/leg/state"};
  std::string waist_state_topic{"/aima/hal/joint/waist/state"};
  std::string arm_state_topic{"/aima/hal/joint/arm/state"};
  std::string head_state_topic{"/aima/hal/joint/head/state"};

  std::string leg_command_topic{"/aima/hal/joint/leg/command"};
  std::string waist_command_topic{"/aima/hal/joint/waist/command"};
  std::string arm_command_topic{"/aima/hal/joint/arm/command"};
  std::string head_command_topic{"/aima/hal/joint/head/command"};

  std::vector<std::string> joint_names;
  std::vector<std::string> leg_joint_names;
  std::vector<std::string> waist_joint_names;
  std::vector<std::string> arm_joint_names;
  std::vector<std::string> head_joint_names;

  std::vector<double> stiffness;
  std::vector<double> damping;
};

struct MotorState {
  std::vector<float> q;
  std::vector<float> dq;
  std::vector<float> tau_est;

  explicit MotorState(size_t num_motors = 0)
      : q(num_motors, 0.0F), dq(num_motors, 0.0F), tau_est(num_motors, 0.0F) {}
};

struct ImuState {
  std::vector<float> quaternion{0.0F, 0.0F, 0.0F, 1.0F};
  std::vector<float> gyroscope{0.0F, 0.0F, 0.0F};
  std::vector<float> accelerometer{0.0F, 0.0F, 0.0F};
};

struct OdometryState {
  bool valid{false};
  bool degenerate{false};
  uint64_t sequence{0};
  int64_t stamp_sec{0};
  uint32_t stamp_nanosec{0};
  std::string frame_id;
  std::string child_frame_id;
  std::vector<float> position{0.0F, 0.0F, 0.0F};
  std::vector<float> quaternion{0.0F, 0.0F, 0.0F, 1.0F};
  std::vector<float> linear_velocity{0.0F, 0.0F, 0.0F};
  std::vector<float> angular_velocity{0.0F, 0.0F, 0.0F};
  std::vector<double> pose_covariance = std::vector<double>(36, 0.0);
};

struct RobotState {
  uint64_t tick{0};
  MotorState motor_state;
  ImuState imu_state;
  OdometryState odometry_state;

  explicit RobotState(size_t num_motors = 0) : motor_state(num_motors) {}
};

struct StateStreamTelemetry {
  std::string topic;
  uint64_t received_count{0};
  std::optional<double> last_receive_age_sec;
  std::optional<double> receive_rate_hz;
  std::optional<double> last_inter_arrival_sec;
  std::optional<double> max_inter_arrival_sec;
  uint64_t sequence_gap_count{0};
  uint64_t sequence_nonmonotonic_count{0};
  std::optional<uint32_t> last_sequence;
  std::optional<int64_t> last_header_stamp_sec;
  std::optional<uint32_t> last_header_stamp_nanosec;
  std::optional<int64_t> last_measurement_stamp_sec;
  std::optional<uint32_t> last_measurement_stamp_nanosec;
  std::vector<std::string> last_joint_names;
};

struct StateFreshnessReport {
  bool required_streams_fresh{false};
  bool imu_received{false};
  std::optional<double> imu_age_sec;
  std::vector<std::string> missing_joint_names;
  std::vector<std::string> stale_joint_names;
  std::map<std::string, double> joint_age_sec;
  bool odometry_required{false};
  bool odometry_received{false};
  bool odometry_valid{false};
  bool odometry_degenerate{false};
  std::optional<double> odometry_age_sec;
  std::string last_odometry_rejection_reason;
  std::optional<double> last_odometry_rejection_age_sec;
  std::map<std::string, StateStreamTelemetry> stream_telemetry;
  std::vector<std::string> reasons;
};

enum class AimdkCommandMode { IDLE, POSITION, PASSIVE, DAMPING };

class AimdkController {
 public:
  explicit AimdkController(const AimdkConfig& cfg);
  ~AimdkController();

  bool self_check(double timeout_sec = 2.0);
  bool state_is_fresh(double timeout_sec);
  StateFreshnessReport get_state_freshness_report(double timeout_sec);
  RobotState get_robot_state();
  void step(const std::vector<double>& positions);
  void arm_position_control();
  void set_passive();
  void set_damping(double damping);
  void set_gains(const std::vector<double>& stiffness, const std::vector<double>& damping);
  void set_control_joint_names(const std::vector<std::string>& joint_names);
  void shutdown();

 private:
  AimdkConfig cfg_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr joint_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;
  std::thread spin_thread_;
  std::thread publish_thread_;
  std::atomic<bool> publish_running_{false};
  std::atomic<bool> shutdown_started_{false};
  bool rclcpp_registered_{false};

  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr leg_sub_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr waist_sub_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr arm_sub_;
  rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr head_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr leg_pub_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr waist_pub_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr head_pub_;

  std::mutex joint_state_mutex_;
  std::mutex imu_state_mutex_;
  std::mutex odometry_state_mutex_;
  std::mutex command_mutex_;
  std::map<std::string, size_t> joint_index_;
  std::set<std::string> command_joint_names_;
  std::set<std::string> received_joint_names_;
  std::map<std::string, std::chrono::steady_clock::time_point> joint_update_times_;
  bool imu_received_{false};
  std::chrono::steady_clock::time_point imu_update_time_{};
  bool odometry_received_{false};
  std::chrono::steady_clock::time_point odometry_update_time_{};
  std::string last_odometry_rejection_reason_;
  std::chrono::steady_clock::time_point last_odometry_rejection_time_{};
  RobotState state_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> latest_positions_;
  std::chrono::steady_clock::time_point last_command_time_{};
  bool command_received_{false};
  AimdkCommandMode command_mode_{AimdkCommandMode::IDLE};
  double mode_damping_{5.0};
  bool watchdog_tripped_{false};

  struct StreamTelemetryState {
    std::string topic;
    uint64_t received_count{0};
    bool received{false};
    std::chrono::steady_clock::time_point last_receive_time{};
    std::deque<std::chrono::steady_clock::time_point> recent_receive_times;
    std::optional<double> last_inter_arrival_sec;
    std::optional<double> max_inter_arrival_sec;
    uint64_t sequence_gap_count{0};
    uint64_t sequence_nonmonotonic_count{0};
    std::optional<uint32_t> last_sequence;
    std::optional<int64_t> last_header_stamp_sec;
    std::optional<uint32_t> last_header_stamp_nanosec;
    std::optional<int64_t> last_measurement_stamp_sec;
    std::optional<uint32_t> last_measurement_stamp_nanosec;
    std::vector<std::string> last_joint_names;
  };

  struct StreamMessageMetadata {
    std::optional<uint32_t> sequence;
    std::optional<int64_t> header_stamp_sec;
    std::optional<uint32_t> header_stamp_nanosec;
    std::optional<int64_t> measurement_stamp_sec;
    std::optional<uint32_t> measurement_stamp_nanosec;
    std::vector<std::string> joint_names;
  };

  std::map<std::string, StreamTelemetryState> stream_telemetry_;

  void joint_callback(const aimdk_msgs::msg::JointStateArray::SharedPtr msg, const std::string& stream_name);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void record_stream_telemetry(const std::string& stream_name,
                               std::chrono::steady_clock::time_point receive_time,
                               StreamMessageMetadata metadata);
  void append_stream_telemetry(StateFreshnessReport& report,
                               std::chrono::steady_clock::time_point now,
                               std::initializer_list<const char*> stream_names);
  void publish_loop();
  void publish_passive_commands();
  void publish_damping_commands();
  void publish_group(const std::vector<std::string>& names,
                     const std::vector<double>& positions,
                     const std::vector<double>& stiffness,
                     const std::vector<double>& damping,
                     bool controlled_only,
                     const rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr& pub);
};
