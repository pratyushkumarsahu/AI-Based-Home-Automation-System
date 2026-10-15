/**
 * @file patrol_navigator.cpp
 * @brief Autonomous Waypoint Navigator with Reactive LiDAR Obstacle Avoidance
 *
 * Implements R1 and R2:
 * 1. Looping autonomous patrol through all home rooms (living room, dining room, hallway,
 *    kitchen, bedroom 1, bedroom 2, entrance).
 * 2. Real-time LiDAR scanning with reactive avoidance (stop / slow / turn).
 * 3. Recovery behavior when stuck (reverse, rotate, retry) and skip waypoint after N retries.
 * 4. Zone determination based on current robot odometry (/odom) against zone polygons.
 * 5. Publishes /obstacle_status and dispatches obstacle events when path is blocked.
 * 6. Supports GAS_RESPONSE navigation toward gas leak sources.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "smart_home_interfaces/msg/obstacle_status.hpp"

using namespace std::chrono_literals;

struct Waypoint {
  std::string name;
  double x;
  double y;
  double yaw;
  std::string zone;
};

struct ZoneBounds {
  std::string name;
  std::string type;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

enum class NavState {
  PATROL,
  AVOID_OBSTACLE,
  RECOVERY_BACKUP,
  RECOVERY_TURN,
  PAUSED,
  INVESTIGATING,
  GAS_RESPONSE
};

class PatrolNavigator : public rclcpp::Node {
public:
  PatrolNavigator() : Node("patrol_navigator"), current_state_(NavState::PATROL) {
    // Parameters
    this->declare_parameter<double>("tolerance", 0.45);
    this->declare_parameter<double>("linear_speed", 0.35);
    this->declare_parameter<double>("angular_speed", 0.60);
    this->declare_parameter<double>("safety_distance", 0.70);
    this->declare_parameter<double>("stop_distance", 0.45);
    this->declare_parameter<double>("stuck_timeout_sec", 2.5);
    this->declare_parameter<int>("max_stuck_retries", 3);
    this->declare_parameter<double>("obstacle_persistence_sec", 3.5);
    this->declare_parameter<double>("gas_target_x", -4.5);
    this->declare_parameter<double>("gas_target_y", -2.5);

    tolerance_ = this->get_parameter("tolerance").as_double();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    stop_distance_ = this->get_parameter("stop_distance").as_double();
    stuck_timeout_sec_ = this->get_parameter("stuck_timeout_sec").as_double();
    max_stuck_retries_ = this->get_parameter("max_stuck_retries").as_int();
    obstacle_persistence_sec_ = this->get_parameter("obstacle_persistence_sec").as_double();
    gas_target_x_ = this->get_parameter("gas_target_x").as_double();
    gas_target_y_ = this->get_parameter("gas_target_y").as_double();

    init_zones();
    init_waypoints();

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/navigation/status", 10);
    zone_pub_ = this->create_publisher<std_msgs::msg::String>("/current_zone", 10);
    obstacle_status_pub_ = this->create_publisher<smart_home_interfaces::msg::ObstacleStatus>(
      "/obstacle_status", 10);
    obstacle_event_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/decision/obstacle_event", 10);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PatrolNavigator::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&PatrolNavigator::scan_callback, this, std::placeholders::_1));

    nav_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/decision/nav_cmd", 10, std::bind(&PatrolNavigator::nav_cmd_callback, this, std::placeholders::_1));

    // Control loop timer (10 Hz)
    control_timer_ = this->create_wall_timer(
      100ms, std::bind(&PatrolNavigator::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Autonomous Patrol Navigator initialized.");
    RCLCPP_INFO(this->get_logger(), "Patrol route covers %zu waypoints across %zu rooms.",
                waypoints_.size(), zones_.size());
  }

private:
  void init_zones() {
    zones_ = {
      {"entrance",    "normal",     -2.7,  2.7, -6.7, -4.8},
      {"dining_room", "normal",     -2.7,  2.7, -4.8, -2.0},
      {"hallway",     "normal",     -2.7,  2.7, -2.0,  3.5},
      {"living_room", "normal",      2.7,  8.0, -6.7, -0.5},
      {"kitchen",     "normal",     -8.0, -2.7, -6.7, -0.5},
      {"bedroom1",    "restricted", -8.0, -2.7,  0.5,  6.7},
      {"bedroom2",    "normal",      2.7,  8.0,  0.5,  6.7}
    };
  }

  void init_waypoints() {
    waypoints_ = {
      // 1. Hallway Start
      {"hallway_center",        0.0,   0.0,  0.0,   "hallway"},

      // 2. Living Room Visit
      {"living_approach",       1.2,  -1.0,  0.0,   "hallway"},
      {"living_doorway",        3.0,  -1.0, -1.57,  "living_room"},
      {"living_room",           4.5,  -2.5, -1.57,  "living_room"},
      {"living_exit",           3.0,  -1.0,  3.14,  "living_room"},
      {"living_hall_return",    1.2,  -1.0, -1.57,  "hallway"},

      // 3. Entrance Visit (via dining room corridor)
      {"corridor_south",        1.5,  -3.2, -1.57,  "dining_room"},
      {"entrance_approach",     1.4,  -5.8,  3.14,  "entrance"},
      {"entrance_foyer",        0.0,  -5.8,  3.14,  "entrance"},
      {"entrance_exit",        -1.4,  -5.8,  1.57,  "entrance"},
      {"corridor_north",       -1.5,  -3.2,  1.57,  "dining_room"},

      // 4. Kitchen Visit
      {"kitchen_approach",     -1.2,  -1.0,  3.14,  "hallway"},
      {"kitchen_doorway",      -3.0,  -1.0, -1.57,  "kitchen"},
      {"kitchen",              -4.5,  -2.5, -1.57,  "kitchen"},
      {"kitchen_exit",         -3.0,  -1.0,  0.0,   "kitchen"},
      {"kitchen_hall_return",  -1.2,  -1.0,  1.57,  "hallway"},

      // 5. Bedroom 1 Visit
      {"bedroom1_approach",    -1.0,   1.35, 3.14,  "hallway"},
      {"bedroom1_doorway",     -3.0,   1.35, 1.57,  "bedroom1"},
      {"bedroom1",             -4.5,   2.5,  0.0,   "bedroom1"},
      {"bedroom1_exit",        -3.0,   1.35, 0.0,   "bedroom1"},
      {"bedroom1_hall_return", -1.0,   1.35, -1.57, "hallway"},

      // 6. Bedroom 2 Visit (Doorway at X=+2.7, Y=1.35)
      {"bedroom2_approach",     1.0,   1.35, 0.0,   "hallway"},
      {"bedroom2_doorway",      3.0,   1.35, 1.57,  "bedroom2"},
      {"bedroom2",              4.5,   2.5,  0.0,   "bedroom2"},
      {"bedroom2_exit",         3.0,   1.35, 3.14,  "bedroom2"},
      {"bedroom2_hall_return",  1.0,   1.35, -1.57, "hallway"},

      // 7. Loop Return
      {"hallway_mid",           0.0,   0.5, -1.57,  "hallway"}
    };
    current_wp_idx_ = 0;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_yaw_);

    if (!has_odom_) {
      has_odom_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Initial Robot Odometry received: (%.2f, %.2f, yaw=%.2f rad)",
                  robot_x_, robot_y_, robot_yaw_);
    }

    update_current_zone();
  }

  void update_current_zone() {
    std::string detected_zone = "hallway";
    for (const auto & z : zones_) {
      if (robot_x_ >= z.x_min && robot_x_ <= z.x_max &&
          robot_y_ >= z.y_min && robot_y_ <= z.y_max) {
        detected_zone = z.name;
        break;
      }
    }

    if (detected_zone != current_zone_name_) {
      current_zone_name_ = detected_zone;
      RCLCPP_INFO(this->get_logger(), "Robot entered zone: [%s] at (%.2f, %.2f)",
                  current_zone_name_.c_str(), robot_x_, robot_y_);

      auto msg = std_msgs::msg::String();
      msg.data = current_zone_name_;
      zone_pub_->publish(msg);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    min_front_dist_ = 10.0;
    min_left_dist_  = 10.0;
    min_right_dist_ = 10.0;

    const size_t num_readings = msg->ranges.size();
    for (size_t i = 0; i < num_readings; ++i) {
      double r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }

      double angle = msg->angle_min + i * msg->angle_increment;
      while (angle > M_PI) angle -= 2.0 * M_PI;
      while (angle < -M_PI) angle += 2.0 * M_PI;

      // Front sector: [-22 deg, +22 deg]
      if (angle >= -0.38 && angle <= 0.38) {
        if (r < min_front_dist_) min_front_dist_ = r;
      }
      // Left sector: [+22 deg, +80 deg]
      else if (angle > 0.38 && angle <= 1.40) {
        if (r < min_left_dist_) min_left_dist_ = r;
      }
      // Right sector: [-80 deg, -22 deg]
      else if (angle >= -1.40 && angle < -0.38) {
        if (r < min_right_dist_) min_right_dist_ = r;
      }
    }

    has_scan_ = true;
  }

  void nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "PAUSE") {
      current_state_ = NavState::PAUSED;
      stop_robot();
      RCLCPP_INFO(this->get_logger(), "Navigation PAUSED by supervisor.");
    } else if (msg->data == "RESUME") {
      current_state_ = NavState::PATROL;
      RCLCPP_INFO(this->get_logger(), "Navigation RESUMED.");
    } else if (msg->data == "INVESTIGATE") {
      current_state_ = NavState::INVESTIGATING;
      investigate_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Investigating target position (360 sweep).");
    } else if (msg->data == "GAS_RESPONSE") {
      current_state_ = NavState::GAS_RESPONSE;
      RCLCPP_INFO(this->get_logger(), "Engaging GAS_RESPONSE gradient navigation toward (%.2f, %.2f).",
                  gas_target_x_, gas_target_y_);
    }
  }

  void control_loop() {
    if (!has_odom_) {
      return;
    }

    publish_status();

    if (current_state_ == NavState::PAUSED) {
      stop_robot();
      return;
    }

    if (current_state_ == NavState::INVESTIGATING) {
      handle_investigate();
      return;
    }

    if (current_state_ == NavState::GAS_RESPONSE) {
      handle_gas_response();
      return;
    }

    if (current_state_ == NavState::RECOVERY_BACKUP) {
      handle_recovery_backup();
      return;
    }

    if (current_state_ == NavState::RECOVERY_TURN) {
      handle_recovery_turn();
      return;
    }

    // Obstacle status evaluation
    smart_home_interfaces::msg::ObstacleStatus obs_msg;
    obs_msg.header.stamp = this->now();
    obs_msg.header.frame_id = "laser_link";
    obs_msg.min_distance = static_cast<float>(min_front_dist_);

    if (has_scan_ && min_front_dist_ < stop_distance_) {
      obs_msg.is_blocked = true;
      if (!is_blocked_) {
        is_blocked_ = true;
        blocked_start_time_ = this->now();
        RCLCPP_WARN(this->get_logger(),
                    "Path BLOCKED at (%.2f, %.2f) heading to [%s]! Obstacle at %.2f m. Stopping.",
                    robot_x_, robot_y_, waypoints_[current_wp_idx_].name.c_str(), min_front_dist_);
      }

      double blocked_duration = (this->now() - blocked_start_time_).seconds();
      obs_msg.blocked_duration_sec = static_cast<float>(blocked_duration);
      obs_msg.description = "Obstacle detected on route within stop distance";
      obstacle_status_pub_->publish(obs_msg);

      // Raise unknown object event if obstacle persists beyond threshold
      if (blocked_duration > obstacle_persistence_sec_ && !persisted_event_sent_) {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown object on route persisted for %.1f s > %.1f s threshold! Raising event.",
                    blocked_duration, obstacle_persistence_sec_);
        auto event_msg = std_msgs::msg::String();
        event_msg.data = "UNKNOWN_OBJECT_ON_ROUTE";
        obstacle_event_pub_->publish(event_msg);
        persisted_event_sent_ = true;
      }

      // Check if blocked for longer than stuck timeout
      if (blocked_duration > stuck_timeout_sec_) {
        stuck_retry_count_++;
        RCLCPP_WARN(this->get_logger(),
                    "Robot stuck for %.1f s (Retry %d of %d).",
                    blocked_duration, stuck_retry_count_, max_stuck_retries_);

        if (stuck_retry_count_ >= max_stuck_retries_) {
          RCLCPP_WARN(this->get_logger(),
                      "Waypoint [%s] exceeded %d failed recovery retries! Skipping to next waypoint.",
                      waypoints_[current_wp_idx_].name.c_str(), max_stuck_retries_);
          current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
          stuck_retry_count_ = 0;
          is_blocked_ = false;
          persisted_event_sent_ = false;
        } else {
          initiate_recovery();
        }
        return;
      }

      // In-place turn away from obstacle
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.0;
      twist.angular.z = (min_left_dist_ >= min_right_dist_) ? angular_speed_ : -angular_speed_;
      cmd_vel_pub_->publish(twist);
      return;
    } else {
      obs_msg.is_blocked = false;
      obs_msg.blocked_duration_sec = 0.0f;
      obs_msg.description = "Path clear";
      obstacle_status_pub_->publish(obs_msg);

      if (is_blocked_ && min_front_dist_ >= safety_distance_) {
        is_blocked_ = false;
        persisted_event_sent_ = false;
        stuck_retry_count_ = 0;
      }
    }

    // Moderate avoidance if obstacle within safety margin
    if (has_scan_ && min_front_dist_ < safety_distance_) {
      const auto & target_wp = waypoints_[current_wp_idx_];
      double dx = target_wp.x - robot_x_;
      double dy = target_wp.y - robot_y_;
      double target_angle = std::atan2(dy, dx);
      double angle_error = target_angle - robot_yaw_;
      while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
      while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

      double steer_bias = (min_left_dist_ >= min_right_dist_) ? 0.35 : -0.35;
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.15;
      twist.angular.z = std::clamp(1.0 * angle_error + steer_bias, -angular_speed_, angular_speed_);
      cmd_vel_pub_->publish(twist);
      return;
    }

    // Standard waypoint navigation
    navigate_to_current_waypoint();
  }

  void navigate_to_current_waypoint() {
    if (waypoints_.empty()) return;

    const auto & target_wp = waypoints_[current_wp_idx_];
    double dx = target_wp.x - robot_x_;
    double dy = target_wp.y - robot_y_;
    double dist = std::hypot(dx, dy);

    if (dist < tolerance_) {
      RCLCPP_INFO(this->get_logger(),
                  "Waypoint [%s] REACHED at (%.2f, %.2f)! Advancing to next room.",
                  target_wp.name.c_str(), robot_x_, robot_y_);
      current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
      stuck_retry_count_ = 0;
      return;
    }

    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - robot_yaw_;

    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    geometry_msgs::msg::Twist twist;
    if (std::abs(angle_error) > 0.45) {
      twist.linear.x = 0.0;
      twist.angular.z = std::clamp(1.5 * angle_error, -angular_speed_, angular_speed_);
    } else {
      twist.linear.x = std::clamp(0.6 * dist, 0.12, linear_speed_);
      twist.angular.z = std::clamp(1.2 * angle_error, -angular_speed_, angular_speed_);
    }

    cmd_vel_pub_->publish(twist);
  }

  void handle_gas_response() {
    double dx = gas_target_x_ - robot_x_;
    double dy = gas_target_y_ - robot_y_;
    double dist = std::hypot(dx, dy);

    if (dist < 0.6) {
      // Reached gas source location, stay in position and keep monitoring
      stop_robot();
      return;
    }

    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - robot_yaw_;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    geometry_msgs::msg::Twist twist;
    if (std::abs(angle_error) > 0.45) {
      twist.linear.x = 0.0;
      twist.angular.z = std::clamp(1.5 * angle_error, -angular_speed_, angular_speed_);
    } else {
      twist.linear.x = std::clamp(0.5 * dist, 0.12, linear_speed_);
      twist.angular.z = std::clamp(1.0 * angle_error, -angular_speed_, angular_speed_);
    }
    cmd_vel_pub_->publish(twist);
  }

  void initiate_recovery() {
    current_state_ = NavState::RECOVERY_BACKUP;
    recovery_start_time_ = this->now();
    recovery_turn_dir_ = (min_left_dist_ >= min_right_dist_) ? 1.0 : -1.0;
  }

  void handle_recovery_backup() {
    double elapsed = (this->now() - recovery_start_time_).seconds();
    if (elapsed < 2.0) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = -0.15;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
    } else {
      current_state_ = NavState::RECOVERY_TURN;
      recovery_start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Recovery: Reversing done. Rotating to clear space.");
    }
  }

  void handle_recovery_turn() {
    double elapsed = (this->now() - recovery_start_time_).seconds();
    if (elapsed < 2.5) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.0;
      twist.angular.z = recovery_turn_dir_ * angular_speed_;
      cmd_vel_pub_->publish(twist);
    } else {
      current_state_ = NavState::PATROL;
      is_blocked_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery complete! Resuming patrol.");
    }
  }

  void handle_investigate() {
    double elapsed = (this->now() - investigate_start_time_).seconds();
    if (elapsed < 4.0) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.0;
      twist.angular.z = 0.35;
      cmd_vel_pub_->publish(twist);
    } else {
      current_state_ = NavState::PATROL;
      RCLCPP_INFO(this->get_logger(), "Investigation sweep complete. Returning to patrol.");
    }
  }

  void stop_robot() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);
  }

  void publish_status() {
    auto msg = std_msgs::msg::String();
    std::string state_str = "PATROL";
    switch (current_state_) {
      case NavState::PATROL: state_str = "PATROL"; break;
      case NavState::AVOID_OBSTACLE: state_str = "AVOIDING"; break;
      case NavState::RECOVERY_BACKUP: state_str = "RECOVERY_BACKUP"; break;
      case NavState::RECOVERY_TURN: state_str = "RECOVERY_TURN"; break;
      case NavState::PAUSED: state_str = "PAUSED"; break;
      case NavState::INVESTIGATING: state_str = "INVESTIGATING"; break;
      case NavState::GAS_RESPONSE: state_str = "GAS_RESPONSE"; break;
    }
    msg.data = state_str + " | Target: " + waypoints_[current_wp_idx_].name +
               " | Zone: " + current_zone_name_;
    status_pub_->publish(msg);
  }

  // Members
  NavState current_state_;
  std::vector<ZoneBounds> zones_;
  std::vector<Waypoint> waypoints_;
  size_t current_wp_idx_{0};
  std::string current_zone_name_{"unknown"};

  double tolerance_{0.45};
  double linear_speed_{0.35};
  double angular_speed_{0.60};
  double safety_distance_{0.70};
  double stop_distance_{0.45};
  double stuck_timeout_sec_{2.5};
  int max_stuck_retries_{3};
  int stuck_retry_count_{0};
  double obstacle_persistence_sec_{3.5};
  bool persisted_event_sent_{false};

  double gas_target_x_{-4.5};
  double gas_target_y_{-2.5};

  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  bool has_odom_{false};

  double min_front_dist_{10.0};
  double min_left_dist_{10.0};
  double min_right_dist_{10.0};
  bool has_scan_{false};

  bool is_blocked_{false};
  rclcpp::Time blocked_start_time_;
  rclcpp::Time recovery_start_time_;
  double recovery_turn_dir_{1.0};
  rclcpp::Time investigate_start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zone_pub_;
  rclcpp::Publisher<smart_home_interfaces::msg::ObstacleStatus>::SharedPtr obstacle_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_event_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_cmd_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
