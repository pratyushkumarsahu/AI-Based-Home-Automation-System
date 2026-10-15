/**
 * @file alert_manager_node.cpp
 * @brief Alert Manager Node with Snapshot Saving, CSV/JSON Logging, Local Notifications, and RViz Markers
 *
 * Implements R7:
 * 1. Subscribes to /alerts and prints formatted, high-visibility console alerts.
 * 2. Saves timestamped snapshot images (.jpg) with bounding boxes and identity labels.
 * 3. Appends structured records to both CSV and JSON log files (including identity and ppm).
 * 4. Writes local notification records to logs/notifications.txt and rings terminal bell.
 * 5. Publishes 3D RViz markers for alerts and home zones on /alert_markers.
 */

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "smart_home_interfaces/msg/alert.hpp"

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

namespace fs = std::filesystem;

class AlertManagerNode : public rclcpp::Node {
public:
  AlertManagerNode() : Node("alert_manager_node") {
    // Parameters
    this->declare_parameter<std::string>("snapshots_dir", "");
    this->declare_parameter<std::string>("logs_dir", "");
    this->declare_parameter<bool>("terminal_bell", true);

    snapshots_dir_ = this->get_parameter("snapshots_dir").as_string();
    logs_dir_ = this->get_parameter("logs_dir").as_string();
    terminal_bell_ = this->get_parameter("terminal_bell").as_bool();

    if (snapshots_dir_.empty()) {
      snapshots_dir_ = fs::current_path() / "snapshots";
    }
    if (logs_dir_.empty()) {
      logs_dir_ = fs::current_path() / "logs";
    }

    fs::create_directories(snapshots_dir_);
    fs::create_directories(logs_dir_);

    csv_file_path_ = logs_dir_ / "security_alerts.csv";
    json_file_path_ = logs_dir_ / "security_alerts.json";
    notifications_file_path_ = logs_dir_ / "notifications.txt";

    init_log_files();

    // Publishers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/alert_markers", 10);
    zone_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/zone_markers", 10);

    // Subscribers
    alert_sub_ = this->create_subscription<smart_home_interfaces::msg::Alert>(
      "/alerts", 10,
      std::bind(&AlertManagerNode::alert_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/annotated_image", 10,
      std::bind(&AlertManagerNode::image_callback, this, std::placeholders::_1));

    // Zone marker timer (publish static room boundaries every 5s)
    zone_timer_ = this->create_wall_timer(
      5000ms, std::bind(&AlertManagerNode::publish_zone_markers, this));

    RCLCPP_INFO(this->get_logger(), "Alert Manager initialized.");
    RCLCPP_INFO(this->get_logger(), "Snapshots saved to: [%s]", snapshots_dir_.c_str());
    RCLCPP_INFO(this->get_logger(), "Security logs saved to: [%s]", logs_dir_.c_str());
  }

private:
  void init_log_files() {
    if (!fs::exists(csv_file_path_)) {
      std::ofstream csv(csv_file_path_);
      csv << "timestamp,alert_id,alert_type,severity,zone,identity,ppm,confidence,description,snapshot_path\n";
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      latest_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      has_frame_ = true;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to buffer camera frame: %s", e.what());
    }
  }

  std::string get_iso_timestamp() const {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time));
    return std::string(buf);
  }

  void alert_callback(const smart_home_interfaces::msg::Alert::SharedPtr msg) {
    std::string timestamp_str = get_iso_timestamp();
    int64_t epoch_sec = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    // Generate snapshot file path
    std::string snap_filename = "alert_" + msg->alert_id + "_" + std::to_string(epoch_sec) + ".jpg";
    fs::path snap_path = snapshots_dir_ / snap_filename;

    // Save snapshot image
    save_snapshot(snap_path, msg, timestamp_str);

    // Append to CSV log
    append_to_csv(timestamp_str, msg, snap_path.string());

    // Append to JSON log
    append_to_json(timestamp_str, msg, snap_path.string());

    // Write local notification and optional terminal bell
    write_local_notification(timestamp_str, msg);

    // Publish RViz visualization markers
    publish_rviz_marker(msg);

    // Print formatted console alert
    print_console_alert(timestamp_str, msg, snap_path.string());
  }

  void save_snapshot(const fs::path & path, const smart_home_interfaces::msg::Alert::SharedPtr & msg,
                     const std::string & timestamp_str) {
    cv::Mat image_to_save;
    if (has_frame_ && !latest_frame_.empty()) {
      image_to_save = latest_frame_.clone();
    } else {
      image_to_save = cv::Mat(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
    }

    // Prominent alert banner at top
    cv::Scalar banner_color = (msg->severity == "CRITICAL") ? cv::Scalar(0, 0, 180) :
                             (msg->severity == "HIGH")     ? cv::Scalar(0, 50, 200) :
                             (msg->severity == "MEDIUM")   ? cv::Scalar(0, 140, 220) :
                                                             cv::Scalar(0, 180, 180);
    cv::rectangle(image_to_save, cv::Point(0, 0), cv::Point(640, 65), banner_color, cv::FILLED);

    std::string line1 = "ALERT: " + msg->alert_id + " [" + msg->severity + "] - " + msg->alert_type;
    std::string line2 = "Zone: " + msg->zone + " | Identity: " + (msg->identity.empty() ? "N/A" : msg->identity);
    if (msg->gas_ppm > 0.0f) {
      line2 += " | Gas: " + std::to_string(static_cast<int>(msg->gas_ppm)) + " ppm";
    }
    line2 += " | " + timestamp_str;

    cv::putText(image_to_save, line1, cv::Point(15, 26), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);
    cv::putText(image_to_save, line2, cv::Point(15, 52), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 255, 255), 1);

    cv::imwrite(path.string(), image_to_save);
  }

  void append_to_csv(const std::string & timestamp_str,
                     const smart_home_interfaces::msg::Alert::SharedPtr & msg,
                     const std::string & snap_path) {
    std::ofstream csv(csv_file_path_, std::ios::app);
    csv << "\"" << timestamp_str << "\","
        << "\"" << msg->alert_id << "\","
        << "\"" << msg->alert_type << "\","
        << "\"" << msg->severity << "\","
        << "\"" << msg->zone << "\","
        << "\"" << msg->identity << "\","
        << msg->gas_ppm << ","
        << msg->confidence << ","
        << "\"" << msg->description << "\","
        << "\"" << snap_path << "\"\n";
  }

  void append_to_json(const std::string & timestamp_str,
                      const smart_home_interfaces::msg::Alert::SharedPtr & msg,
                      const std::string & snap_path) {
    std::ofstream json(json_file_path_, std::ios::app);
    json << "{\n"
         << "  \"timestamp\": \"" << timestamp_str << "\",\n"
         << "  \"alert_id\": \"" << msg->alert_id << "\",\n"
         << "  \"alert_type\": \"" << msg->alert_type << "\",\n"
         << "  \"severity\": \"" << msg->severity << "\",\n"
         << "  \"zone\": \"" << msg->zone << "\",\n"
         << "  \"identity\": \"" << msg->identity << "\",\n"
         << "  \"gas_ppm\": " << msg->gas_ppm << ",\n"
         << "  \"confidence\": " << msg->confidence << ",\n"
         << "  \"description\": \"" << msg->description << "\",\n"
         << "  \"snapshot_path\": \"" << snap_path << "\"\n"
         << "}\n";
  }

  void write_local_notification(const std::string & timestamp_str,
                                const smart_home_interfaces::msg::Alert::SharedPtr & msg) {
    std::ofstream notif(notifications_file_path_, std::ios::app);
    notif << "[" << timestamp_str << "] [" << msg->severity << "] "
          << msg->alert_id << " in " << msg->zone << " (" << msg->alert_type << "): "
          << msg->description;
    if (!msg->identity.empty()) {
      notif << " | Identity: " << msg->identity;
    }
    if (msg->gas_ppm > 0.0f) {
      notif << " | Gas: " << msg->gas_ppm << " ppm";
    }
    notif << "\n";

    if (terminal_bell_) {
      std::cout << "\a" << std::flush;
    }
  }

  void publish_rviz_marker(const smart_home_interfaces::msg::Alert::SharedPtr & msg) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "security_alerts";
    marker.id = marker_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.5;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    if (msg->severity == "CRITICAL") {
      marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f;
    } else if (msg->severity == "HIGH") {
      marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 0.0f;
    } else {
      marker.color.r = 1.0f; marker.color.g = 0.8f; marker.color.b = 0.0f;
    }
    marker.color.a = 0.85f;
    markers.markers.push_back(marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "base_link";
    text_marker.header.stamp = this->now();
    text_marker.ns = "security_alerts_text";
    text_marker.id = marker_id_++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 1.0;
    text_marker.pose.position.y = 0.0;
    text_marker.pose.position.z = 0.85;
    text_marker.scale.z = 0.25;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;
    text_marker.text = "[" + msg->severity + "] " + msg->alert_type + " (" + msg->zone + ")";
    markers.markers.push_back(text_marker);

    marker_pub_->publish(markers);
  }

  void publish_zone_markers() {
    visualization_msgs::msg::MarkerArray markers;
    struct ZoneDef { std::string name; double cx, cy, sx, sy; float r, g, b; };
    const std::vector<ZoneDef> defs = {
      {"entrance",    0.0, -5.75, 5.4, 1.9, 0.4f, 0.7f, 0.4f},
      {"dining_room", 0.0, -3.40, 5.4, 2.8, 0.5f, 0.6f, 0.7f},
      {"hallway",     0.0,  0.75, 5.4, 5.5, 0.7f, 0.7f, 0.5f},
      {"living_room", 5.35,-3.60, 5.3, 6.2, 0.4f, 0.6f, 0.8f},
      {"kitchen",    -5.35,-3.60, 5.3, 6.2, 0.8f, 0.6f, 0.3f},
      {"bedroom1",   -5.35, 3.60, 5.3, 6.2, 0.9f, 0.2f, 0.2f},
      {"bedroom2",    5.35, 3.60, 5.3, 6.2, 0.3f, 0.7f, 0.9f}
    };

    int zid = 0;
    for (const auto & zd : defs) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "smart_home_world";
      m.header.stamp = this->now();
      m.ns = "room_zones";
      m.id = zid++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = zd.cx;
      m.pose.position.y = zd.cy;
      m.pose.position.z = 0.02;
      m.scale.x = zd.sx;
      m.scale.y = zd.sy;
      m.scale.z = 0.04;
      m.color.r = zd.r;
      m.color.g = zd.g;
      m.color.b = zd.b;
      m.color.a = 0.25f;
      markers.markers.push_back(m);
    }
    zone_marker_pub_->publish(markers);
  }

  void print_console_alert(const std::string & timestamp_str,
                           const smart_home_interfaces::msg::Alert::SharedPtr & msg,
                           const std::string & snap_path) {
    std::cout << "\n\033[1;31m======================================================================\033[0m\n";
    std::cout << "\033[1;31m🚨 [SECURITY ALERT] ID: " << msg->alert_id
              << " | Severity: " << msg->severity << "\033[0m\n";
    std::cout << "📍 \033[1;33mLocation/Zone:\033[0m " << msg->zone << " (" << msg->alert_type << ")\n";
    std::cout << "👤 \033[1;35mIdentity:\033[0m      " << (msg->identity.empty() ? "N/A" : msg->identity) << "\n";
    if (msg->gas_ppm > 0.0f) {
      std::cout << "💨 \033[1;31mGas Level:\033[0m     " << msg->gas_ppm << " ppm\n";
    }
    std::cout << "📝 \033[1;37mDescription:\033[0m   " << msg->description << "\n";
    std::cout << "📊 \033[1;36mConfidence:\033[0m    " << static_cast<int>(msg->confidence * 100)
              << "% | Time: " << timestamp_str << "\n";
    std::cout << "📷 \033[1;32mSnapshot:\033[0m      " << snap_path << "\n";
    std::cout << "\033[1;31m======================================================================\033[0m\n" << std::endl;
  }

  // Members
  fs::path snapshots_dir_;
  fs::path logs_dir_;
  fs::path csv_file_path_;
  fs::path json_file_path_;
  fs::path notifications_file_path_;
  bool terminal_bell_{true};

  cv::Mat latest_frame_;
  bool has_frame_{false};
  int marker_id_{0};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_marker_pub_;
  rclcpp::Subscription<smart_home_interfaces::msg::Alert>::SharedPtr alert_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr zone_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AlertManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
