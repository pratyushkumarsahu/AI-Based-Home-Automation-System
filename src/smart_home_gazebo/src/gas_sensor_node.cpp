/**
 * @file gas_sensor_node.cpp
 * @brief Realistic Simulated Gas Sensor Node with Visual Plume Emitter & RViz Markers
 *
 * Implements R4 and P3:
 * 1. Simulates gas dispersion from an active source (kitchen stove at -5.5, -2.5) in the house.
 * 2. Computes concentration (ppm) based on robot distance to source:
 *    ppm = peak_ppm / (1.0 + falloff_k * dist^2) + noise.
 * 3. Can be activated/deactivated via topic /gas_leak/trigger (std_msgs/Bool).
 * 4. Publishes /gas_level (smart_home_interfaces/msg/GasLevel) and /gas_level/ppm (std_msgs/Float32).
 * 5. Controls visual Gazebo Harmonic ParticleEmitter (smoke_emitter) on kitchen stove.
 * 6. Controls fallback visual cloud model (gas_cloud) in Gazebo world via set_pose service.
 * 7. Publishes translucent RViz markers (sphere with color gradient green->yellow->red + text label).
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "smart_home_interfaces/msg/gas_level.hpp"

#include <gz/transport/Node.hh>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>

using namespace std::chrono_literals;

class GasSensorNode : public rclcpp::Node {
public:
  GasSensorNode() : Node("gas_sensor_node"), rng_(std::random_device{}()) {
    // Parameters
    this->declare_parameter<double>("source_x", -4.5);
    this->declare_parameter<double>("source_y", -2.5);
    this->declare_parameter<double>("source_z", 0.8);
    this->declare_parameter<double>("peak_ppm", 400.0);
    this->declare_parameter<double>("falloff_k", 0.12);
    this->declare_parameter<double>("noise_sigma", 1.0);
    this->declare_parameter<bool>("leak_active", false);

    source_x_ = this->get_parameter("source_x").as_double();
    source_y_ = this->get_parameter("source_y").as_double();
    peak_ppm_ = this->get_parameter("peak_ppm").as_double();
    falloff_k_ = this->get_parameter("falloff_k").as_double();
    noise_sigma_ = this->get_parameter("noise_sigma").as_double();
    leak_active_ = this->get_parameter("leak_active").as_bool();

    // Publishers
    gas_pub_ = this->create_publisher<smart_home_interfaces::msg::GasLevel>("/gas_level", 10);
    gas_ppm_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gas_level/ppm", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gas_leak_marker", 10);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GasSensorNode::odom_callback, this, std::placeholders::_1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/gas_leak/trigger", 10, std::bind(&GasSensorNode::trigger_callback, this, std::placeholders::_1));

    // Gazebo Transport: Particle Emitter Command Publisher
    gz_emitter_pub_ = gz_node_.Advertise<gz::msgs::ParticleEmitter>(
      "/model/gas_stove/link/stove_body/particle_emitter/smoke_emitter/cmd");

    // Initialize visual state
    update_visual_plume(leak_active_);

    // Simulation timer (4 Hz)
    timer_ = this->create_wall_timer(250ms, std::bind(&GasSensorNode::publish_gas_level, this));

    RCLCPP_INFO(this->get_logger(),
                "Gas Sensor Node initialized. Source at (%.2f, %.2f) [Kitchen]. Leak Active: %s",
                source_x_, source_y_, leak_active_ ? "YES" : "NO");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    has_odom_ = true;
  }

  void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (leak_active_ != msg->data) {
      leak_active_ = msg->data;
      RCLCPP_INFO(this->get_logger(), "Gas Leak Trigger state changed: %s",
                  leak_active_ ? "ACTIVATED" : "DEACTIVATED");
      update_visual_plume(leak_active_);
    }
  }

  void update_visual_plume(bool active) {
    // 1. Control Particle Emitter
    gz::msgs::ParticleEmitter emitter_msg;
    emitter_msg.mutable_emitting()->set_data(active);
    if (active) {
      emitter_msg.mutable_rate()->set_data(25);
    } else {
      emitter_msg.mutable_rate()->set_data(0);
    }
    gz_emitter_pub_.Publish(emitter_msg);

    // 2. Control Fallback Visual Cloud Model (gas_cloud)
    gz::msgs::Pose req;
    req.set_name("gas_cloud");
    req.mutable_position()->set_x(-5.5);
    req.mutable_position()->set_y(-2.5);
    req.mutable_position()->set_z(active ? 1.15 : -10.0);
    req.mutable_orientation()->set_x(0.0);
    req.mutable_orientation()->set_y(0.0);
    req.mutable_orientation()->set_z(0.0);
    req.mutable_orientation()->set_w(1.0);

    gz::msgs::Boolean rep;
    bool result = false;
    gz_node_.Request("/world/smart_home_world/set_pose", req, 50, rep, result);

    RCLCPP_INFO(this->get_logger(), "Visual gas plume in Gazebo: %s (Emitter + Gas Cloud)",
                active ? "ACTIVE (Stove Plume Visible)" : "OFF (Cleared)");
  }

  void publish_gas_level() {
    std::normal_distribution<double> noise_dist(0.0, noise_sigma_);
    double noise = noise_dist(rng_);

    double current_ppm = 5.0 + std::abs(noise); // Safe ambient level

    if (leak_active_) {
      double dist = std::hypot(robot_x_ - source_x_, robot_y_ - source_y_);
      double plume = peak_ppm_ / (1.0 + falloff_k_ * dist * dist);
      current_ppm = std::max(5.0, plume + noise);
    }

    // Determine current zone of gas source
    std::string zone = "kitchen";

    // 1. Publish custom GasLevel message
    smart_home_interfaces::msg::GasLevel msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gas_sensor_link";
    msg.ppm = static_cast<float>(current_ppm);
    msg.zone = zone;
    msg.leak_active = leak_active_;
    gas_pub_->publish(msg);

    // 2. Also publish standard Float32
    std_msgs::msg::Float32 f_msg;
    f_msg.data = static_cast<float>(current_ppm);
    gas_ppm_pub_->publish(f_msg);

    // 3. Publish RViz Visualization Markers
    publish_rviz_markers(current_ppm);

    // Periodically reaffirm visual plume state in sim
    if (leak_active_) {
      gz::msgs::ParticleEmitter emitter_msg;
      emitter_msg.mutable_emitting()->set_data(true);
      gz_emitter_pub_.Publish(emitter_msg);
    }
  }

  void publish_rviz_markers(double current_ppm) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Marker 1: Translucent Sphere at Gas Source (Kitchen Stove)
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = "odom";
    sphere_marker.header.stamp = this->now();
    sphere_marker.ns = "gas_hazard";
    sphere_marker.id = 1;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose.position.x = -5.5;
    sphere_marker.pose.position.y = -2.5;
    sphere_marker.pose.position.z = 1.15;
    sphere_marker.pose.orientation.w = 1.0;

    double sphere_scale = 0.6 + std::min(1.2, current_ppm / 200.0);
    sphere_marker.scale.x = sphere_scale;
    sphere_marker.scale.y = sphere_scale;
    sphere_marker.scale.z = sphere_scale;

    // Color gradient based on concentration
    if (current_ppm < 50.0) {
      // Safe (<50 ppm): Green translucent
      sphere_marker.color.r = 0.1f;
      sphere_marker.color.g = 0.85f;
      sphere_marker.color.b = 0.1f;
      sphere_marker.color.a = 0.35f;
    } else if (current_ppm < 100.0) {
      // Warning (50-100 ppm): Yellow/Amber translucent
      sphere_marker.color.r = 0.95f;
      sphere_marker.color.g = 0.80f;
      sphere_marker.color.b = 0.1f;
      sphere_marker.color.a = 0.55f;
    } else {
      // Critical Danger (>=100 ppm): Red translucent
      sphere_marker.color.r = 0.95f;
      sphere_marker.color.g = 0.1f;
      sphere_marker.color.b = 0.1f;
      sphere_marker.color.a = 0.70f;
    }
    marker_array.markers.push_back(sphere_marker);

    // Marker 2: Text Label Showing PPM and Hazard Level
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "odom";
    text_marker.header.stamp = this->now();
    text_marker.ns = "gas_hazard";
    text_marker.id = 2;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = -5.5;
    text_marker.pose.position.y = -2.5;
    text_marker.pose.position.z = 1.85;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.28; // Text height
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 0.95f;

    std::string level_str = (current_ppm >= 100.0) ? "CRITICAL DANGER" :
                            ((current_ppm >= 50.0) ? "WARNING" : "NORMAL");
    char text_buf[128];
    std::snprintf(text_buf, sizeof(text_buf), "Gas: %.1f PPM [%s]", current_ppm, level_str.c_str());
    text_marker.text = text_buf;
    marker_array.markers.push_back(text_marker);

    marker_pub_->publish(marker_array);
  }

  // Members
  double source_x_{-4.5};
  double source_y_{-2.5};
  double peak_ppm_{400.0};
  double falloff_k_{0.12};
  double noise_sigma_{1.0};
  bool leak_active_{false};

  double robot_x_{0.0};
  double robot_y_{0.0};
  bool has_odom_{false};

  std::mt19937 rng_;

  rclcpp::Publisher<smart_home_interfaces::msg::GasLevel>::SharedPtr gas_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gas_ppm_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_emitter_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GasSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
