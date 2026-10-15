/**
 * @file decision_node.cpp
 * @brief C++ State Machine Decision Maker implementing Workflow Flowchart
 *
 * Implements R6 and Workflow:
 * 1. States: PATROL, INVESTIGATE, ALERT, AVOID_OBSTACLE, GAS_RESPONSE, RETURN_TO_PATROL, IDLE.
 * 2. Strict Priority: GAS DANGER > outsider intrusion > obstacle > family/normal.
 * 3. Log every state transition.
 * 4. Configurable alert cooldown (30s per rule/zone) with severity escalation override.
 * 5. Publishes real-time telemetry on /status (smart_home_interfaces/msg/SystemStatus).
 */

#include <chrono>
#include <ctime>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "smart_home_interfaces/msg/detection.hpp"
#include "smart_home_interfaces/msg/detection_array.hpp"
#include "smart_home_interfaces/msg/alert.hpp"
#include "smart_home_interfaces/msg/gas_level.hpp"
#include "smart_home_interfaces/msg/system_status.hpp"
#include "smart_home_decision/rule_engine.hpp"

using namespace std::chrono_literals;

struct AlertRecord {
  rclcpp::Time timestamp;
  std::string severity;
};

class DecisionMakerNode : public rclcpp::Node {
public:
  DecisionMakerNode() : Node("decision_maker_node"), state_(smart_home::DecisionState::PATROL) {
    // Parameters
    this->declare_parameter<std::string>("home_mode", "away");
    this->declare_parameter<bool>("restricted_hours_enabled", true);
    this->declare_parameter<int>("restricted_start_hour", 22);
    this->declare_parameter<int>("restricted_end_hour", 6);
    this->declare_parameter<double>("loitering_threshold_sec", 4.0);
    this->declare_parameter<double>("obstacle_safety_dist", 0.50);
    this->declare_parameter<double>("blocked_timeout_sec", 3.0);
    this->declare_parameter<std::string>("restricted_zone", "bedroom1");
    this->declare_parameter<double>("gas_warning_ppm", 50.0);
    this->declare_parameter<double>("gas_danger_ppm", 100.0);
    this->declare_parameter<double>("alert_cooldown_sec", 30.0);

    // Configure Rule Engine
    smart_home::RuleEngineConfig cfg;
    cfg.home_mode = this->get_parameter("home_mode").as_string();
    cfg.restricted_hours_enabled = this->get_parameter("restricted_hours_enabled").as_bool();
    cfg.restricted_start_hour = this->get_parameter("restricted_start_hour").as_int();
    cfg.restricted_end_hour = this->get_parameter("restricted_end_hour").as_int();
    cfg.loitering_threshold_sec = this->get_parameter("loitering_threshold_sec").as_double();
    cfg.obstacle_safety_dist = this->get_parameter("obstacle_safety_dist").as_double();
    cfg.blocked_timeout_sec = this->get_parameter("blocked_timeout_sec").as_double();
    cfg.gas_warning_ppm = this->get_parameter("gas_warning_ppm").as_double();
    cfg.gas_danger_ppm = this->get_parameter("gas_danger_ppm").as_double();
    cfg.restricted_zones = {this->get_parameter("restricted_zone").as_string()};
    rule_engine_.set_config(cfg);

    cooldown_sec_ = this->get_parameter("alert_cooldown_sec").as_double();

    // Publishers
    alert_pub_ = this->create_publisher<smart_home_interfaces::msg::Alert>("/alerts", 10);
    nav_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/decision/nav_cmd", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/decision/state", 10);
    status_pub_ = this->create_publisher<smart_home_interfaces::msg::SystemStatus>("/status", 10);

    // Subscribers
    detections_sub_ = this->create_subscription<smart_home_interfaces::msg::DetectionArray>(
      "/detections", 10,
      std::bind(&DecisionMakerNode::detections_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&DecisionMakerNode::scan_callback, this, std::placeholders::_1));

    zone_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/current_zone", 10,
      std::bind(&DecisionMakerNode::zone_callback, this, std::placeholders::_1));

    gas_sub_ = this->create_subscription<smart_home_interfaces::msg::GasLevel>(
      "/gas_level", 10,
      std::bind(&DecisionMakerNode::gas_callback, this, std::placeholders::_1));

    obs_event_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/decision/obstacle_event", 10,
      std::bind(&DecisionMakerNode::obs_event_callback, this, std::placeholders::_1));

    start_time_ = this->now();
    state_entry_time_ = this->now();

    // Decision loop timer (5 Hz)
    timer_ = this->create_wall_timer(
      200ms, std::bind(&DecisionMakerNode::decision_loop, this));

    // Telemetry and normal logging timer (1 Hz)
    log_timer_ = this->create_wall_timer(
      1000ms, std::bind(&DecisionMakerNode::publish_telemetry, this));

    RCLCPP_INFO(this->get_logger(), "Decision Maker State Machine initialized.");
    RCLCPP_INFO(this->get_logger(), "Mode: [%s] | Restricted Zone: [%s] | Cooldown: %.1fs",
                cfg.home_mode.c_str(), this->get_parameter("restricted_zone").as_string().c_str(), cooldown_sec_);
  }

private:
  void zone_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data != current_zone_) {
      RCLCPP_INFO(this->get_logger(), "Zone changed: [%s] -> [%s]",
                  current_zone_.c_str(), msg->data.c_str());
      current_zone_ = msg->data;
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    min_front_dist_ = 10.0;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      double angle = msg->angle_min + i * msg->angle_increment;
      if (angle >= -0.38 && angle <= 0.38) {
        if (r < min_front_dist_) min_front_dist_ = r;
      }
    }
  }

  void gas_callback(const smart_home_interfaces::msg::GasLevel::SharedPtr msg) {
    latest_gas_ppm_ = msg->ppm;
    latest_gas_zone_ = msg->zone;
    has_gas_data_ = true;
  }

  void obs_event_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "UNKNOWN_OBJECT_ON_ROUTE") {
      obstacle_event_pending_ = true;
    }
  }

  void detections_callback(const smart_home_interfaces::msg::DetectionArray::SharedPtr msg) {
    latest_detections_ = *msg;
    has_detections_ = true;
    bool sees_person = false;
    for (const auto & d : msg->detections) {
      if (d.class_name == "person") {
        sees_person = true;
        break;
      }
    }
    if (sees_person && !person_seen_) {
      person_seen_ = true;
      person_seen_start_ = this->now();
    } else if (!sees_person && person_seen_) {
      person_seen_ = false;
    }
  }

  int get_current_hour() const {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm * tm_struct = std::localtime(&t);
    return tm_struct ? tm_struct->tm_hour : 12;
  }

  void transition_to_state(smart_home::DecisionState next_state, const std::string & reason = "") {
    if (state_ == next_state) return;
    std::string old_str = smart_home::to_string(state_);
    std::string next_str = smart_home::to_string(next_state);
    state_ = next_state;
    state_entry_time_ = this->now();

    if (reason.empty()) {
      RCLCPP_INFO(this->get_logger(), "[TRANSITION] [%s] -> [%s]", old_str.c_str(), next_str.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "[TRANSITION] [%s] -> [%s] (Reason: %s)",
                  old_str.c_str(), next_str.c_str(), reason.c_str());
    }

    auto msg = std_msgs::msg::String();
    msg.data = next_str;
    state_pub_->publish(msg);
  }

  void send_nav_cmd(const std::string & cmd) {
    auto msg = std_msgs::msg::String();
    msg.data = cmd;
    nav_cmd_pub_->publish(msg);
  }

  void decision_loop() {
    switch (state_) {
      case smart_home::DecisionState::PATROL:
        evaluate_patrol_state();
        break;

      case smart_home::DecisionState::INVESTIGATE:
        handle_investigate_state();
        break;

      case smart_home::DecisionState::ALERT:
        handle_alert_state();
        break;

      case smart_home::DecisionState::AVOID_OBSTACLE:
        handle_avoid_obstacle_state();
        break;

      case smart_home::DecisionState::GAS_RESPONSE:
        handle_gas_response_state();
        break;

      case smart_home::DecisionState::RETURN_TO_PATROL:
        transition_to_state(smart_home::DecisionState::PATROL, "Patrol resumed after event completion");
        send_nav_cmd("RESUME");
        break;

      case smart_home::DecisionState::IDLE:
        break;
    }
  }

  void evaluate_patrol_state() {
    int current_hour = get_current_hour();
    double loitering_time = person_seen_ ? (this->now() - person_seen_start_).seconds() : 0.0;

    // =========================================================================
    // PRIORITY 1: GAS DANGER / HAZARD (Highest Priority)
    // =========================================================================
    if (has_gas_data_) {
      auto gas_res = rule_engine_.evaluate_gas_leak(latest_gas_ppm_, latest_gas_zone_);
      if (gas_res.triggered) {
        if (gas_res.severity == "CRITICAL") {
          // Gas danger exceeded -> Engage robot response to hazard source
          if (trigger_alert(gas_res.rule_name, gas_res.severity, gas_res.description, 0.99f, "", latest_gas_ppm_)) {
            send_nav_cmd("GAS_RESPONSE");
            transition_to_state(smart_home::DecisionState::GAS_RESPONSE, "CRITICAL Gas Leak gradient dispatch");
            return;
          }
        } else if (gas_res.severity == "MEDIUM") {
          if (trigger_alert(gas_res.rule_name, gas_res.severity, gas_res.description, 0.90f, "", latest_gas_ppm_)) {
            send_nav_cmd("PAUSE");
            transition_to_state(smart_home::DecisionState::ALERT, "Gas Concentration Warning elevation");
            return;
          }
        }
      }
    }

    // =========================================================================
    // PRIORITY 2: PERSON SECURITY (Outsider Intrusion vs Family)
    // =========================================================================
    if (has_detections_) {
      for (const auto & det : latest_detections_.detections) {
        if (det.class_name == "person") {
          // Evaluate person using identity marker classification
          auto sec_res = rule_engine_.evaluate_person_security(det.identity, det.is_family, current_zone_, current_hour);
          if (sec_res.triggered) {
            if (trigger_alert(sec_res.rule_name, sec_res.severity, sec_res.description, det.confidence, det.identity, 0.0f)) {
              send_nav_cmd("PAUSE");
              transition_to_state(smart_home::DecisionState::ALERT, sec_res.rule_name);
              return;
            }
          }

          // Rule 3: Loitering (only for outsiders or unverified persons; authorized family in allowed zones do not trigger alerts)
          if (!det.is_family) {
            auto res3 = rule_engine_.evaluate_loitering(det.class_name, loitering_time, current_zone_);
            if (res3.triggered) {
              if (trigger_alert(res3.rule_name, res3.severity, res3.description, det.confidence, det.identity, 0.0f)) {
                send_nav_cmd("INVESTIGATE");
                transition_to_state(smart_home::DecisionState::INVESTIGATE, "Person loitering detected");
                return;
              }
            }
          }
        }

        // Rule 4: Inanimate unknown obstacle detection
        if (det.class_name == "obstacle" || det.class_name == "backpack" || det.class_name == "suitcase") {
          auto res4 = rule_engine_.evaluate_unknown_object(det.class_name, current_zone_);
          if (res4.triggered) {
            if (trigger_alert(res4.rule_name, res4.severity, res4.description, det.confidence, "obstacle", 0.0f)) {
              transition_to_state(smart_home::DecisionState::AVOID_OBSTACLE, "Unknown obstacle detected");
              return;
            }
          }
        }
      }
    }

    // =========================================================================
    // PRIORITY 3: OBSTACLE AVOIDANCE (Persistent blockage / corridor box)
    // =========================================================================
    if (obstacle_event_pending_) {
      obstacle_event_pending_ = false;
      if (trigger_alert("Rule_Unknown_Obstacle_Persistent", "WARNING",
                        "Unknown object on patrol route persisted beyond threshold in " + current_zone_,
                        0.90f, "unknown_object", 0.0f)) {
        transition_to_state(smart_home::DecisionState::AVOID_OBSTACLE, "Persistent obstacle event from navigator");
        return;
      }
    }

    if (min_front_dist_ < 0.45) {
      if (!path_blocked_) {
        path_blocked_ = true;
        blocked_start_ = this->now();
      } else {
        double blocked_time = (this->now() - blocked_start_).seconds();
        auto res = rule_engine_.evaluate_blocked_path(min_front_dist_, blocked_time);
        if (res.triggered) {
          if (trigger_alert(res.rule_name, res.severity, res.description, 0.90f, "path_blocked", 0.0f)) {
            transition_to_state(smart_home::DecisionState::AVOID_OBSTACLE, "Path blocked duration threshold exceeded");
            return;
          }
        }
      }
    } else {
      path_blocked_ = false;
    }
  }

  void handle_investigate_state() {
    double elapsed = (this->now() - state_entry_time_).seconds();
    if (elapsed > 4.0) {
      RCLCPP_INFO(this->get_logger(), "[INVESTIGATE] Area investigation complete. Returning to patrol.");
      transition_to_state(smart_home::DecisionState::RETURN_TO_PATROL, "Investigation completed");
    }
  }

  void handle_alert_state() {
    double elapsed = (this->now() - state_entry_time_).seconds();
    if (elapsed > 3.0) {
      RCLCPP_INFO(this->get_logger(), "[ALERT] Alert processing finished. Transitioning to investigate.");
      send_nav_cmd("INVESTIGATE");
      transition_to_state(smart_home::DecisionState::INVESTIGATE, "Alert period completed, investigating scene");
    }
  }

  void handle_avoid_obstacle_state() {
    double elapsed = (this->now() - state_entry_time_).seconds();
    if (min_front_dist_ >= 0.70 || elapsed > 3.0) {
      RCLCPP_INFO(this->get_logger(), "[AVOID_OBSTACLE] Path clear / avoidance maneuver executed. Resuming patrol.");
      transition_to_state(smart_home::DecisionState::RETURN_TO_PATROL, "Path clear after avoidance");
    }
  }

  void handle_gas_response_state() {
    // Check if gas concentration has cleared / fallen below warning threshold
    if (has_gas_data_ && latest_gas_ppm_ < rule_engine_.get_config().gas_warning_ppm) {
      RCLCPP_INFO(this->get_logger(),
                  "🟢 [GAS CLEARANCE] Gas level fell to safe level (%.1f ppm < %.1f ppm threshold). Clearing alarm.",
                  latest_gas_ppm_, rule_engine_.get_config().gas_warning_ppm);
      transition_to_state(smart_home::DecisionState::RETURN_TO_PATROL, "Gas concentration returned to safe level");
    }
  }

  bool trigger_alert(const std::string & rule_name, const std::string & severity,
                     const std::string & description, float confidence,
                     const std::string & identity = "", float gas_ppm = 0.0f) {
    // Alert Cooldown with Severity Escalation Override
    auto key = std::make_pair(rule_name, current_zone_);
    auto it = alert_history_.find(key);
    if (it != alert_history_.end()) {
      double elapsed = (this->now() - it->second.timestamp).seconds();
      if (elapsed < cooldown_sec_) {
        // Cooldown active. Check severity escalation:
        if (smart_home::severity_rank(severity) <= smart_home::severity_rank(it->second.severity)) {
          // Lower or equal severity -> suppress during cooldown
          return false;
        }
        RCLCPP_INFO(this->get_logger(),
                    "[ESCALATION] Severity escalated from %s to %s for [%s in %s]! Overriding cooldown.",
                    it->second.severity.c_str(), severity.c_str(), rule_name.c_str(), current_zone_.c_str());
      }
    }

    // Record alert in history
    alert_history_[key] = {this->now(), severity};
    last_alert_str_ = "[" + severity + "] " + rule_name + " in " + current_zone_;

    alert_count_++;
    smart_home_interfaces::msg::Alert alert_msg;
    alert_msg.header.stamp = this->now();
    alert_msg.header.frame_id = "patrol_robot";
    alert_msg.alert_id = "ALT-" + std::to_string(alert_count_);
    alert_msg.alert_type = rule_name;
    alert_msg.severity = severity;
    alert_msg.zone = current_zone_;
    alert_msg.description = description;
    alert_msg.confidence = confidence;
    alert_msg.timestamp = this->now();
    alert_msg.identity = identity.empty() ? (severity == "CRITICAL" ? "OUTSIDER" : "Unknown") : identity;
    alert_msg.gas_ppm = gas_ppm;

    RCLCPP_WARN(this->get_logger(), "🚨 ALERT TRIGGERED: [%s] [%s] in %s: %s",
                alert_msg.alert_id.c_str(), severity.c_str(), current_zone_.c_str(), description.c_str());

    alert_pub_->publish(alert_msg);
    return true;
  }

  void publish_telemetry() {
    // 1. Publish /status telemetry message
    smart_home_interfaces::msg::SystemStatus status_msg;
    status_msg.header.stamp = this->now();
    status_msg.header.frame_id = "patrol_robot";
    status_msg.state = smart_home::to_string(state_);
    status_msg.zone = current_zone_;
    status_msg.uptime_sec = static_cast<float>((this->now() - start_time_).seconds());
    status_msg.last_alert = last_alert_str_;
    status_msg.gas_ppm = static_cast<float>(latest_gas_ppm_);
    status_pub_->publish(status_msg);

    // 2. Periodic normal condition log when in PATROL state
    if (state_ == smart_home::DecisionState::PATROL) {
      RCLCPP_INFO(this->get_logger(),
                  "🟢 [NORMAL MONITORING] Area: %s | Status: CLEAR | Decision: Continue monitoring.",
                  current_zone_.c_str());
    }
  }

  // Members
  smart_home::DecisionState state_;
  smart_home::RuleEngine rule_engine_;
  std::string current_zone_{"hallway"};
  double cooldown_sec_{30.0};
  int alert_count_{0};
  std::string last_alert_str_{"None"};

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;
  std::map<std::pair<std::string, std::string>, AlertRecord> alert_history_;

  double min_front_dist_{10.0};
  bool path_blocked_{false};
  rclcpp::Time blocked_start_;

  double latest_gas_ppm_{5.0};
  std::string latest_gas_zone_{"kitchen"};
  bool has_gas_data_{false};

  bool obstacle_event_pending_{false};

  bool person_seen_{false};
  rclcpp::Time person_seen_start_;
  bool has_detections_{false};
  smart_home_interfaces::msg::DetectionArray latest_detections_;

  rclcpp::Publisher<smart_home_interfaces::msg::Alert>::SharedPtr alert_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<smart_home_interfaces::msg::SystemStatus>::SharedPtr status_pub_;

  rclcpp::Subscription<smart_home_interfaces::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zone_sub_;
  rclcpp::Subscription<smart_home_interfaces::msg::GasLevel>::SharedPtr gas_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obs_event_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DecisionMakerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
