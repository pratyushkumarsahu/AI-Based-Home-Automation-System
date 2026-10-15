/**
 * @file test_decision.cpp
 * @brief Unit tests for Smart Home Rule Engine and Decision Logic
 */

#include <gtest/gtest.h>
#include "smart_home_decision/rule_engine.hpp"

class RuleEngineTest : public ::testing::Test {
protected:
  void SetUp() override {
    smart_home::RuleEngineConfig cfg;
    cfg.home_mode = "away";
    cfg.restricted_hours_enabled = true;
    cfg.restricted_start_hour = 22;
    cfg.restricted_end_hour = 6;
    cfg.loitering_threshold_sec = 4.0;
    cfg.obstacle_safety_dist = 0.50;
    cfg.blocked_timeout_sec = 3.0;
    cfg.restricted_zones = {"bedroom1"};
    rule_engine.set_config(cfg);
  }

  smart_home::RuleEngine rule_engine;
};

// 1. Test Rule 1: Restricted Zone
TEST_F(RuleEngineTest, Rule1RestrictedZoneTriggered) {
  auto res = rule_engine.evaluate_restricted_zone("person", "bedroom1");
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "CRITICAL");
  EXPECT_EQ(res.action, "ALERT");
}

TEST_F(RuleEngineTest, Rule1AllowedZoneNotTriggered) {
  auto res = rule_engine.evaluate_restricted_zone("person", "living_room");
  EXPECT_FALSE(res.triggered);
}

TEST_F(RuleEngineTest, Rule1NonPersonNotTriggered) {
  auto res = rule_engine.evaluate_restricted_zone("chair", "bedroom1");
  EXPECT_FALSE(res.triggered);
}

// 2. Test Rule 2: Away Mode / Curfew
TEST_F(RuleEngineTest, Rule2AwayModeTriggered) {
  auto res = rule_engine.evaluate_time_and_mode("person", 14); // 2 PM, away mode
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "HIGH");
}

TEST_F(RuleEngineTest, Rule2HomeModeDuringDayNotTriggered) {
  auto cfg = rule_engine.get_config();
  cfg.home_mode = "home";
  rule_engine.set_config(cfg);

  auto res = rule_engine.evaluate_time_and_mode("person", 14); // 2 PM, home mode
  EXPECT_FALSE(res.triggered);
}

TEST_F(RuleEngineTest, Rule2HomeModeCurfewTriggered) {
  auto cfg = rule_engine.get_config();
  cfg.home_mode = "home";
  rule_engine.set_config(cfg);

  auto res = rule_engine.evaluate_time_and_mode("person", 23); // 11 PM curfew
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "HIGH");
}

// 3. Test Rule 3: Loitering
TEST_F(RuleEngineTest, Rule3LoiteringExceeded) {
  auto res = rule_engine.evaluate_loitering("person", 5.5, "hallway");
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "WARNING");
  EXPECT_EQ(res.action, "INVESTIGATE");
}

TEST_F(RuleEngineTest, Rule3LoiteringUnderThreshold) {
  auto res = rule_engine.evaluate_loitering("person", 1.5, "hallway");
  EXPECT_FALSE(res.triggered);
}

// 4. Test Rule 4: Unknown Object
TEST_F(RuleEngineTest, Rule4UnknownObstacle) {
  auto res = rule_engine.evaluate_unknown_object("obstacle", "hallway");
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "WARNING");
  EXPECT_EQ(res.action, "AVOID");
}

// 5. Test Rule 5: Path Blocked
TEST_F(RuleEngineTest, Rule5BlockedPathExceeded) {
  auto res = rule_engine.evaluate_blocked_path(0.35, 4.0);
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "MEDIUM");
}

TEST_F(RuleEngineTest, Rule5ClearPathNotTriggered) {
  auto res = rule_engine.evaluate_blocked_path(1.5, 4.0);
  EXPECT_FALSE(res.triggered);
}

// 6. Test State Conversions
TEST(DecisionStateTest, StateToString) {
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::PATROL), "PATROL");
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::INVESTIGATE), "INVESTIGATE");
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::ALERT), "ALERT");
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::AVOID_OBSTACLE), "AVOID_OBSTACLE");
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::GAS_RESPONSE), "GAS_RESPONSE");
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::RETURN_TO_PATROL), "RETURN_TO_PATROL");
  EXPECT_EQ(smart_home::to_string(smart_home::DecisionState::IDLE), "IDLE");
}

// 7. Test Gas Leak Evaluation
TEST_F(RuleEngineTest, GasSafeLevelNotTriggered) {
  auto res = rule_engine.evaluate_gas_leak(15.0, "kitchen");
  EXPECT_FALSE(res.triggered);
}

TEST_F(RuleEngineTest, GasWarningThresholdTriggered) {
  auto res = rule_engine.evaluate_gas_leak(65.0, "kitchen");
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "MEDIUM");
  EXPECT_EQ(res.action, "ALERT");
}

TEST_F(RuleEngineTest, GasDangerThresholdTriggered) {
  auto res = rule_engine.evaluate_gas_leak(120.0, "kitchen");
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "CRITICAL");
  EXPECT_EQ(res.action, "GAS_RESPONSE");
}

// 8. Test Family vs Outsider Classification
TEST_F(RuleEngineTest, AuthorizedFamilyInAllowedZoneNotTriggered) {
  auto res = rule_engine.evaluate_person_security("Pratyush", true, "living_room", 14);
  EXPECT_FALSE(res.triggered);
  EXPECT_EQ(res.severity, "INFO");
}

TEST_F(RuleEngineTest, OutsiderAnywhereTriggeredHigh) {
  auto res = rule_engine.evaluate_person_security("OUTSIDER", false, "living_room", 14);
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "HIGH");
  EXPECT_EQ(res.action, "ALERT");
}

TEST_F(RuleEngineTest, FamilyInRestrictedCurfewTriggeredHigh) {
  auto res = rule_engine.evaluate_person_security("Rohit", true, "bedroom1", 23);
  EXPECT_TRUE(res.triggered);
  EXPECT_EQ(res.severity, "HIGH");
  EXPECT_EQ(res.action, "ALERT");
}

// 9. Test Severity Ranking
TEST(SeverityTest, SeverityRankingComparison) {
  EXPECT_GT(smart_home::severity_rank("CRITICAL"), smart_home::severity_rank("HIGH"));
  EXPECT_GT(smart_home::severity_rank("HIGH"), smart_home::severity_rank("MEDIUM"));
  EXPECT_GT(smart_home::severity_rank("MEDIUM"), smart_home::severity_rank("WARNING"));
  EXPECT_GT(smart_home::severity_rank("WARNING"), smart_home::severity_rank("INFO"));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
