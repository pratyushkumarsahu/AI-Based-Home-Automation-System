#ifndef SMART_HOME_DECISION__RULE_ENGINE_HPP_
#define SMART_HOME_DECISION__RULE_ENGINE_HPP_

#include <string>
#include <vector>
#include <unordered_set>
#include <chrono>

namespace smart_home {

enum class DecisionState {
  PATROL,
  INVESTIGATE,
  ALERT,
  AVOID_OBSTACLE,
  GAS_RESPONSE,
  RETURN_TO_PATROL,
  IDLE
};

inline std::string to_string(DecisionState state) {
  switch (state) {
    case DecisionState::PATROL: return "PATROL";
    case DecisionState::INVESTIGATE: return "INVESTIGATE";
    case DecisionState::ALERT: return "ALERT";
    case DecisionState::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
    case DecisionState::GAS_RESPONSE: return "GAS_RESPONSE";
    case DecisionState::RETURN_TO_PATROL: return "RETURN_TO_PATROL";
    case DecisionState::IDLE: return "IDLE";
    default: return "UNKNOWN";
  }
}

inline int severity_rank(const std::string & sev) {
  if (sev == "CRITICAL") return 4;
  if (sev == "HIGH")     return 3;
  if (sev == "MEDIUM")   return 2;
  if (sev == "WARNING")  return 1;
  return 0; // INFO / normal
}

struct RuleEvaluation {
  bool triggered{false};
  std::string rule_name;
  std::string severity;   // "INFO", "WARNING", "MEDIUM", "HIGH", "CRITICAL"
  std::string description;
  std::string action;     // "MONITOR", "INVESTIGATE", "ALERT", "AVOID", "GAS_RESPONSE"
};

struct RuleEngineConfig {
  std::string home_mode{"away"}; // "home" or "away"
  bool restricted_hours_enabled{true};
  int restricted_start_hour{22};
  int restricted_end_hour{6};
  double loitering_threshold_sec{4.0};
  double obstacle_safety_dist{0.50};
  double blocked_timeout_sec{3.0};
  double gas_warning_ppm{50.0};
  double gas_danger_ppm{100.0};
  std::unordered_set<std::string> restricted_zones{"bedroom1"};
};

class RuleEngine {
public:
  explicit RuleEngine(const RuleEngineConfig & config = RuleEngineConfig())
  : config_(config) {}

  void set_config(const RuleEngineConfig & config) {
    config_ = config;
  }

  const RuleEngineConfig & get_config() const {
    return config_;
  }

  // Gas Leak Hazard Rule (Highest Priority: GAS DANGER > outsider > obstacle > normal)
  RuleEvaluation evaluate_gas_leak(double ppm, const std::string & zone) const {
    RuleEvaluation result;
    if (ppm >= config_.gas_danger_ppm) {
      result.triggered = true;
      result.rule_name = "Rule_Gas_Danger";
      result.severity = "CRITICAL";
      result.description = "CRITICAL Gas Leak hazard detected in " + zone + " (" +
                            std::to_string(static_cast<int>(ppm)) + " ppm > " +
                            std::to_string(static_cast<int>(config_.gas_danger_ppm)) + " ppm threshold)!";
      result.action = "GAS_RESPONSE";
    } else if (ppm >= config_.gas_warning_ppm) {
      result.triggered = true;
      result.rule_name = "Rule_Gas_Warning";
      result.severity = "MEDIUM";
      result.description = "Gas concentration warning elevated in " + zone + " (" +
                            std::to_string(static_cast<int>(ppm)) + " ppm)";
      result.action = "ALERT";
    }
    return result;
  }

  // Family Member vs Outsider Security Rule (R3)
  RuleEvaluation evaluate_person_security(const std::string & identity, bool is_family,
                                          const std::string & zone, int current_hour) const {
    RuleEvaluation result;

    bool in_restricted = (config_.restricted_zones.find(zone) != config_.restricted_zones.end());

    if (!is_family) {
      // Outsider detected
      if (in_restricted) {
        result.triggered = true;
        result.rule_name = "Rule_1_Restricted_Zone";
        result.severity = "CRITICAL";
        result.description = "Unauthorized person detected inside restricted zone: " + zone;
        result.action = "ALERT";
        return result;
      } else {
        result.triggered = true;
        result.rule_name = "Rule_Outsider_Intrusion";
        result.severity = "HIGH";
        result.description = "Unidentified outsider detected in zone: " + zone;
        result.action = "ALERT";
        return result;
      }
    }

    // Family Member checks:
    bool is_curfew_hour = false;
    if (config_.restricted_hours_enabled) {
      if (config_.restricted_start_hour > config_.restricted_end_hour) {
        is_curfew_hour = (current_hour >= config_.restricted_start_hour || current_hour < config_.restricted_end_hour);
      } else {
        is_curfew_hour = (current_hour >= config_.restricted_start_hour && current_hour < config_.restricted_end_hour);
      }
    }

    if (in_restricted && is_curfew_hour) {
      // Family member in restricted zone during curfew hours -> Alert with HIGH severity
      result.triggered = true;
      result.rule_name = "Rule_Family_Curfew_Restricted";
      result.severity = "HIGH";
      result.description = "Family member (" + identity + ") detected in restricted zone (" +
                           zone + ") during curfew hours (" + std::to_string(current_hour) + ":00)";
      result.action = "ALERT";
      return result;
    }

    // Otherwise, authorized family member in allowed zone -> Normal condition, NO alert
    result.triggered = false;
    result.rule_name = "Rule_Family_Authorized";
    result.severity = "INFO";
    result.description = "Authorized family member (" + identity + ") in " + zone + " (Normal condition)";
    result.action = "MONITOR";
    return result;
  }

  // Rule 1: Backward-compatibility for person in restricted zone
  RuleEvaluation evaluate_restricted_zone(const std::string & class_name, const std::string & zone) const {
    RuleEvaluation result;
    result.rule_name = "Rule_1_Restricted_Zone";
    if (class_name == "person" && config_.restricted_zones.find(zone) != config_.restricted_zones.end()) {
      result.triggered = true;
      result.severity = "CRITICAL";
      result.description = "Unauthorized person detected inside restricted zone: " + zone;
      result.action = "ALERT";
    }
    return result;
  }

  // Rule 2: Backward-compatibility for person during restricted hours or in Away mode
  RuleEvaluation evaluate_time_and_mode(const std::string & class_name, int current_hour) const {
    RuleEvaluation result;
    result.rule_name = "Rule_2_Restricted_Hours_Or_Away";
    if (class_name == "person") {
      bool is_away = (config_.home_mode == "away");
      bool is_restricted_hour = false;
      if (config_.restricted_hours_enabled) {
        if (config_.restricted_start_hour > config_.restricted_end_hour) {
          is_restricted_hour = (current_hour >= config_.restricted_start_hour || current_hour < config_.restricted_end_hour);
        } else {
          is_restricted_hour = (current_hour >= config_.restricted_start_hour && current_hour < config_.restricted_end_hour);
        }
      }

      if (is_away || is_restricted_hour) {
        result.triggered = true;
        result.severity = "HIGH";
        result.description = is_away ? "Person detected while home mode is set to AWAY"
                                     : "Person detected during curfew / restricted hours";
        result.action = "ALERT";
      }
    }
    return result;
  }

  // Rule 3: Person loitering longer than threshold
  RuleEvaluation evaluate_loitering(const std::string & class_name, double duration_sec, const std::string & zone) const {
    RuleEvaluation result;
    result.rule_name = "Rule_3_Loitering";
    if (class_name == "person" && duration_sec >= config_.loitering_threshold_sec) {
      result.triggered = true;
      result.severity = "WARNING";
      result.description = "Person loitering in zone '" + zone + "' for " + std::to_string(static_cast<int>(duration_sec)) + "s";
      result.action = "INVESTIGATE";
    }
    return result;
  }

  // Rule 4: Unknown object appearing on patrol route
  RuleEvaluation evaluate_unknown_object(const std::string & class_name, const std::string & zone) const {
    RuleEvaluation result;
    result.rule_name = "Rule_4_Unknown_Object";
    if (class_name == "obstacle" || class_name == "backpack" || class_name == "suitcase") {
      result.triggered = true;
      result.severity = "WARNING";
      result.description = "Unknown object (" + class_name + ") detected in patrol corridor in zone: " + zone;
      result.action = "AVOID";
    }
    return result;
  }

  // Rule 5: Path blocked too long
  RuleEvaluation evaluate_blocked_path(double front_dist, double blocked_duration_sec) const {
    RuleEvaluation result;
    result.rule_name = "Rule_5_Path_Blocked";
    if (front_dist < config_.obstacle_safety_dist && blocked_duration_sec >= config_.blocked_timeout_sec) {
      result.triggered = true;
      result.severity = "MEDIUM";
      result.description = "Patrol route blocked for " + std::to_string(static_cast<int>(blocked_duration_sec)) + "s";
      result.action = "AVOID";
    }
    return result;
  }

private:
  RuleEngineConfig config_;
};

} // namespace smart_home

#endif // SMART_HOME_DECISION__RULE_ENGINE_HPP_
