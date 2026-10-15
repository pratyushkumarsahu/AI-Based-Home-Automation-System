/**
 * @file person_motion_node.cpp
 * @brief Kinematic walking motion controller for family members and outsider in Gazebo
 *
 * Implements P2:
 * 1. Simulates realistic person locomotion along room paths.
 * 2. Pratyush walks in living room, Arpita walks in dining room/south corridor,
 *    Rohit walks in bedroom 2/north corridor, Outsider enters/patrols hallway and bedroom 1.
 * 3. Orients models in their direction of movement (yaw) and updates poses via Gazebo set_pose service.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>

using namespace std::chrono_literals;

struct Point2D {
  double x;
  double y;
};

struct Walker {
  std::string name;
  std::vector<Point2D> waypoints;
  size_t current_wp_idx{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double speed{0.18}; // m/s
  bool active{true};
};

class PersonMotionNode : public rclcpp::Node {
public:
  PersonMotionNode() : Node("person_motion_node") {
    this->declare_parameter<std::string>("scenario", "full");
    scenario_ = this->get_parameter("scenario").as_string();

    init_walkers();

    // 10 Hz control loop
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PersonMotionNode::update_motion, this));

    RCLCPP_INFO(this->get_logger(),
                "Person Motion Controller initialized for scenario [%s] with %zu active walkers.",
                scenario_.c_str(), walkers_.size());
  }

private:
  void init_walkers() {
    walkers_.clear();

    // 1. Pratyush: Living Room route
    Walker pratyush;
    pratyush.name = "family_pratyush";
    pratyush.waypoints = {
      {2.5, -0.5},
      {3.5, -0.8},
      {3.0, -1.5},
      {2.2, -0.5},
      {2.5, -0.5}
    };
    pratyush.x = pratyush.waypoints[0].x;
    pratyush.y = pratyush.waypoints[0].y;
    pratyush.speed = 0.16;
    pratyush.active = (scenario_ == "family" || scenario_ == "full");
    if (pratyush.active) walkers_.push_back(pratyush);

    // 2. Arpita: Dining Room and Corridor route
    Walker arpita;
    arpita.name = "family_arpita";
    arpita.waypoints = {
      {1.5, -0.7},
      {2.2, -0.7},
      {2.0, -1.4},
      {1.2, -1.4},
      {1.5, -0.7}
    };
    arpita.x = arpita.waypoints[0].x;
    arpita.y = arpita.waypoints[0].y;
    arpita.speed = 0.18;
    arpita.active = (scenario_ == "family" || scenario_ == "full");
    if (arpita.active) walkers_.push_back(arpita);

    // 3. Rohit: Bedroom 2 and Corridor North route
    Walker rohit;
    rohit.name = "family_rohit";
    rohit.waypoints = {
      {1.8, 0.8},
      {2.4, 1.2},
      {3.2, 1.5},
      {2.0, 1.0},
      {1.8, 0.8}
    };
    rohit.x = rohit.waypoints[0].x;
    rohit.y = rohit.waypoints[0].y;
    rohit.speed = 0.18;
    rohit.active = (scenario_ == "family" || scenario_ == "full");
    if (rohit.active) walkers_.push_back(rohit);

    // 4. Outsider / Intruder:
    Walker intruder;
    intruder.name = "intruder";
    if (scenario_ == "intruder") {
      intruder.waypoints = {
        {2.2, -0.8},
        {3.2, -1.8},
        {2.0, -1.2},
        {2.2, -0.8}
      };
      intruder.active = true;
    } else if (scenario_ == "full") {
      intruder.waypoints = {
        {2.0, 0.0},
        {1.4, 0.2},
        {1.8, -0.3},
        {2.2, 0.0},
        {2.0, 0.0}
      };
      intruder.active = true;
    } else {
      intruder.active = false;
    }
    if (intruder.active) {
      intruder.x = intruder.waypoints[0].x;
      intruder.y = intruder.waypoints[0].y;
      intruder.speed = 0.18;
      walkers_.push_back(intruder);
    }
  }

  void update_motion() {
    const double dt = 0.10; // 100ms
    for (auto & w : walkers_) {
      if (!w.active || w.waypoints.empty()) continue;

      const auto & target = w.waypoints[w.current_wp_idx];
      double dx = target.x - w.x;
      double dy = target.y - w.y;
      double dist = std::hypot(dx, dy);

      if (dist < 0.15) {
        // Advance to next waypoint
        w.current_wp_idx = (w.current_wp_idx + 1) % w.waypoints.size();
        continue;
      }

      double yaw = std::atan2(dy, dx);
      double step = std::min(dist, w.speed * dt);
      w.x += step * std::cos(yaw);
      w.y += step * std::sin(yaw);

      // Send pose update to Gazebo
      gz::msgs::Pose req;
      req.set_name(w.name);
      req.mutable_position()->set_x(w.x);
      req.mutable_position()->set_y(w.y);
      req.mutable_position()->set_z(0.0);

      // Orientation quaternion from yaw
      double qz = std::sin(yaw * 0.5);
      double qw = std::cos(yaw * 0.5);
      req.mutable_orientation()->set_x(0.0);
      req.mutable_orientation()->set_y(0.0);
      req.mutable_orientation()->set_z(qz);
      req.mutable_orientation()->set_w(qw);

      gz::msgs::Boolean rep;
      bool result = false;
      gz_node_.Request("/world/smart_home_world/set_pose", req, 20, rep, result);
    }
  }

  std::string scenario_;
  std::vector<Walker> walkers_;
  rclcpp::TimerBase::SharedPtr timer_;
  gz::transport::Node gz_node_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersonMotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
