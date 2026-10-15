/**
 * @file detection_node.cpp
 * @brief AI Object & Person Detection Node with OpenCV DNN, Family Recognition, and Adaptive Fallback
 *
 * Implements Stage 4 and R3:
 * 1. Pretrained deep neural network object detection (OpenCV DNN / ONNX).
 * 2. Automatic fallback detector (color and contour segmentation) if model weights are absent.
 * 3. Family vs Outsider recognition via visible identity markers (colored badges mapped to names).
 * 4. Populates custom Detection and DetectionArray messages with class, confidence, bbox, zone,
 *    identity, and is_family.
 * 5. Generates annotated image stream for visualization and alert snapshot generation.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "smart_home_interfaces/msg/detection.hpp"
#include "smart_home_interfaces/msg/detection_array.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Standard COCO class names (80 classes)
static const std::vector<std::string> COCO_CLASSES = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
  "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
  "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
  "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
  "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
  "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
  "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

struct PersonIdentity {
  std::string name;
  bool is_family;
};

class ObjectDetectionNode : public rclcpp::Node {
public:
  ObjectDetectionNode() : Node("object_detection_node") {
    // Parameters
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<double>("confidence_threshold", 0.35);
    this->declare_parameter<double>("nms_threshold", 0.45);
    this->declare_parameter<int>("input_width", 640);
    this->declare_parameter<int>("input_height", 640);

    model_path_ = this->get_parameter("model_path").as_string();
    conf_thresh_ = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
    nms_thresh_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
    input_width_ = this->get_parameter("input_width").as_int();
    input_height_ = this->get_parameter("input_height").as_int();

    // Publishers
    detections_pub_ = this->create_publisher<smart_home_interfaces::msg::DetectionArray>(
      "/detections", 10);
    annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/annotated_image", 10);

    // Subscribers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&ObjectDetectionNode::image_callback, this, std::placeholders::_1));

    zone_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/current_zone", 10,
      std::bind(&ObjectDetectionNode::zone_callback, this, std::placeholders::_1));

    // Initialize AI DNN model
    init_model();

    // Startup log per R8: clearly say detector mode and active sensors
    std::string mode_str = use_fallback_ ? "FALLBACK DETECTOR (Color/Contour)" :
      ("YOLO ONNX (" + (model_path_.empty() ? "yolov5n.onnx" : model_path_) + ")");
    RCLCPP_INFO(this->get_logger(), "=================================================");
    RCLCPP_INFO(this->get_logger(), "Detector Mode: [%s]", mode_str.c_str());
    RCLCPP_INFO(this->get_logger(), "Active Sensors: RGB Camera (/camera/image_raw), LiDAR (/scan), Odometry (/odom), IMU (/imu), Gas Sensor (/gas_level)");
    RCLCPP_INFO(this->get_logger(), "=================================================");
  }

private:
  void init_model() {
    use_fallback_ = true;

    if (model_path_.empty()) {
      try {
        std::string share_dir = ament_index_cpp::get_package_share_directory("smart_home_detection");
        std::string cand = share_dir + "/models/yolov5n.onnx";
        std::ifstream f(cand.c_str());
        if (f.good()) {
          model_path_ = cand;
        }
      } catch (...) {}
    }

    if (model_path_.empty()) {
      const std::vector<std::string> candidates = {
        "src/smart_home_detection/models/yolov5n.onnx",
        "/home/vboxuser/home_monitor_ws/src/smart_home_detection/models/yolov5n.onnx",
        "models/yolov5n.onnx",
        "/home/vboxuser/home_monitor_ws/models/yolov5n.onnx"
      };
      for (const auto & cand : candidates) {
        std::ifstream f(cand.c_str());
        if (f.good()) {
          model_path_ = cand;
          break;
        }
      }
    }

    if (model_path_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "No YOLO ONNX model found in search paths. Engaging Fallback Detector.");
      return;
    }

    std::ifstream f(model_path_.c_str());
    if (!f.good()) {
      RCLCPP_WARN(this->get_logger(),
                  "Model file not found at [%s]. Engaging Fallback Detector.",
                  model_path_.c_str());
      return;
    }

    try {
      net_ = cv::dnn::readNetFromONNX(model_path_);
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      use_fallback_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Successfully loaded YOLO ONNX model from [%s]! Primary detector ACTIVE.",
                  model_path_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize DNN model: %s. Reverting to Fallback Detector.",
                   e.what());
      use_fallback_ = true;
    }
  }

  void zone_callback(const std_msgs::msg::String::SharedPtr msg) {
    current_zone_ = msg->data;
  }

  PersonIdentity classify_person(const cv::Mat & frame, const cv::Rect & bbox) {
    // Extract torso / badge ROI
    int cx = std::max(0, bbox.x);
    int cy = std::max(0, bbox.y);
    int cw = std::min(frame.cols - cx, bbox.width);
    int ch = std::min(frame.rows - cy, bbox.height);

    if (cw < 4 || ch < 4) {
      return {"OUTSIDER", false};
    }

    cv::Rect roi_rect;
    if (static_cast<double>(ch) / cw > 1.6) {
      // Full body detection (from DNN YOLO or tall silhouette)
      int chest_y = cy + static_cast<int>(ch * 0.10);
      int chest_h = static_cast<int>(ch * 0.45);
      int chest_x = cx + static_cast<int>(cw * 0.05);
      int chest_w = static_cast<int>(cw * 0.90);
      chest_x = std::clamp(chest_x, 0, frame.cols - 1);
      chest_y = std::clamp(chest_y, 0, frame.rows - 1);
      chest_w = std::clamp(chest_w, 1, frame.cols - chest_x);
      chest_h = std::clamp(chest_h, 1, frame.rows - chest_y);
      roi_rect = cv::Rect(chest_x, chest_y, chest_w, chest_h);
    } else {
      // Badge / upper body contour: use full bounding box
      roi_rect = cv::Rect(cx, cy, cw, ch);
    }

    cv::Mat roi = frame(roi_rect);
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // 1. Green badge: ID 1 = Pratyush (Target H=68)
    cv::Mat mask_green;
    cv::inRange(hsv, cv::Scalar(42, 50, 40), cv::Scalar(78, 255, 255), mask_green);
    int green_pixels = cv::countNonZero(mask_green);

    // 2. Cyan badge: ID 3 = Rohit (Target H=90)
    cv::Mat mask_cyan;
    cv::inRange(hsv, cv::Scalar(80, 50, 40), cv::Scalar(99, 255, 255), mask_cyan);
    int cyan_pixels = cv::countNonZero(mask_cyan);

    // 3. Blue badge: ID 2 = Arpita (Target H=111)
    cv::Mat mask_blue;
    cv::inRange(hsv, cv::Scalar(101, 50, 40), cv::Scalar(135, 255, 255), mask_blue);
    int blue_pixels = cv::countNonZero(mask_blue);

    int total_pixels = roi_rect.width * roi_rect.height;
    int min_thresh = std::max(4, static_cast<int>(total_pixels * 0.003));

    if (green_pixels >= min_thresh && green_pixels > blue_pixels && green_pixels > cyan_pixels) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                  "IDENTIFICATION: [Pratyush] verified (FAMILY) with Green Badge (pixels: %d >= %d) in zone [%s]",
                  green_pixels, min_thresh, current_zone_.c_str());
      return {"Pratyush", true};
    } else if (blue_pixels >= min_thresh && blue_pixels > green_pixels && blue_pixels > cyan_pixels) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                  "IDENTIFICATION: [Arpita] verified (FAMILY) with Blue Badge (pixels: %d >= %d) in zone [%s]",
                  blue_pixels, min_thresh, current_zone_.c_str());
      return {"Arpita", true};
    } else if (cyan_pixels >= min_thresh && cyan_pixels > green_pixels && cyan_pixels > blue_pixels) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                  "IDENTIFICATION: [Rohit] verified (FAMILY) with Cyan Badge (pixels: %d >= %d) in zone [%s]",
                  cyan_pixels, min_thresh, current_zone_.c_str());
      return {"Rohit", true};
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "IDENTIFICATION: [OUTSIDER] detected (NO AUTHORIZED BADGE) in zone [%s]! Security Alert triggered.",
                current_zone_.c_str());
    return {"OUTSIDER", false};
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat & frame = cv_ptr->image;
    smart_home_interfaces::msg::DetectionArray detection_array;
    detection_array.header = msg->header;

    if (!use_fallback_) {
      run_dnn_detection(frame, detection_array);
      if (detection_array.detections.empty()) {
        // Fallback secondary stage if YOLO didn't pick up the simple Gazebo model
        run_fallback_detection(frame, detection_array);
      }
    } else {
      run_fallback_detection(frame, detection_array);
    }

    // Publish detections
    detections_pub_->publish(detection_array);

    // Annotate and publish visualization image
    draw_annotations(frame, detection_array);
    sensor_msgs::msg::Image::SharedPtr annotated_msg =
      cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    annotated_image_pub_->publish(*annotated_msg);
  }

  void run_dnn_detection(const cv::Mat & frame, smart_home_interfaces::msg::DetectionArray & det_array) {
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, cv::Size(input_width_, input_height_),
                           cv::Scalar(), true, false);
    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    if (outputs.empty()) return;

    cv::Mat & out = outputs[0];
    float x_factor = static_cast<float>(frame.cols) / input_width_;
    float y_factor = static_cast<float>(frame.rows) / input_height_;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    // YOLOv5 format: [1, 25200, 85]
    if (out.dims == 3 && out.size[2] == 85) {
      float * data = reinterpret_cast<float*>(out.data);
      const int rows = out.size[1];
      for (int i = 0; i < rows; ++i) {
        float obj_conf = data[4];
        if (obj_conf > conf_thresh_) {
          const float * class_scores = data + 5;
          cv::Mat scores(1, 80, CV_32F, const_cast<float*>(class_scores));
          cv::Point class_id_point;
          double max_class_score;
          cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
          float final_conf = obj_conf * static_cast<float>(max_class_score);
          if (final_conf > conf_thresh_) {
            float cx = data[0] * x_factor;
            float cy = data[1] * y_factor;
            float w = data[2] * x_factor;
            float h = data[3] * y_factor;

            int left = std::max(0, static_cast<int>(cx - 0.5f * w));
            int top = std::max(0, static_cast<int>(cy - 0.5f * h));
            int width = std::min(frame.cols - left, static_cast<int>(w));
            int height = std::min(frame.rows - top, static_cast<int>(h));

            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(final_conf);
            class_ids.push_back(class_id_point.x);
          }
        }
        data += 85;
      }
    }
    // YOLOv8 format: [1, 84, 8400]
    else if (out.dims == 3 && out.size[1] == 84) {
      int dimensions = out.size[1];
      int rows = out.size[2];
      cv::Mat reshaped(dimensions, rows, CV_32F, out.ptr<float>());
      cv::Mat transposed;
      cv::transpose(reshaped, transposed);
      for (int i = 0; i < transposed.rows; ++i) {
        float * row_ptr = transposed.ptr<float>(i);
        float * classes_scores = row_ptr + 4;
        cv::Mat scores(1, 80, CV_32F, classes_scores);
        cv::Point class_id_point;
        double max_class_score;
        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
        if (max_class_score > conf_thresh_) {
          float cx = row_ptr[0] * x_factor;
          float cy = row_ptr[1] * y_factor;
          float w = row_ptr[2] * x_factor;
          float h = row_ptr[3] * y_factor;

          int left = std::max(0, static_cast<int>(cx - 0.5f * w));
          int top = std::max(0, static_cast<int>(cy - 0.5f * h));
          int width = std::min(frame.cols - left, static_cast<int>(w));
          int height = std::min(frame.rows - top, static_cast<int>(h));

          boxes.push_back(cv::Rect(left, top, width, height));
          confidences.push_back(static_cast<float>(max_class_score));
          class_ids.push_back(class_id_point.x);
        }
      }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_thresh_, nms_thresh_, indices);

    for (int idx : indices) {
      smart_home_interfaces::msg::Detection det;
      int cid = class_ids[idx];
      det.class_name = (cid < static_cast<int>(COCO_CLASSES.size())) ? COCO_CLASSES[cid] : "object";
      det.confidence = confidences[idx];
      det.xmin = boxes[idx].x;
      det.ymin = boxes[idx].y;
      det.xmax = boxes[idx].x + boxes[idx].width;
      det.ymax = boxes[idx].y + boxes[idx].height;
      det.timestamp = this->now();
      det.zone = current_zone_;

      if (det.class_name == "person") {
        PersonIdentity pid = classify_person(frame, boxes[idx]);
        det.identity = pid.name;
        det.is_family = pid.is_family;
      } else {
        det.identity = det.class_name;
        det.is_family = false;
      }

      det_array.detections.push_back(det);
    }
  }

  void run_fallback_detection(const cv::Mat & frame, smart_home_interfaces::msg::DetectionArray & det_array) {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Red clothing (Intruder jacket: H=0, S=210, V=216, rejects wood trim at H=15)
    cv::Mat mask_red1, mask_red2, mask_red;
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(7, 255, 255), mask_red1);
    cv::inRange(hsv, cv::Scalar(173, 120, 70), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    // Green clothing / badge (Pratyush: H=68, S=255)
    cv::Mat mask_green;
    cv::inRange(hsv, cv::Scalar(42, 50, 40), cv::Scalar(78, 255, 255), mask_green);

    // Cyan clothing / badge (Rohit: H=90, S=224)
    cv::Mat mask_cyan;
    cv::inRange(hsv, cv::Scalar(80, 50, 40), cv::Scalar(99, 255, 255), mask_cyan);

    // Blue clothing / badge (Arpita: H=111, S=216)
    cv::Mat mask_blue;
    cv::inRange(hsv, cv::Scalar(101, 50, 40), cv::Scalar(135, 255, 255), mask_blue);

    // Combine person masks (Red jacket for intruder, Green/Cyan/Blue badge for family)
    cv::Mat person_mask;
    cv::bitwise_or(mask_red, mask_green, person_mask);
    cv::bitwise_or(person_mask, mask_cyan, person_mask);
    cv::bitwise_or(person_mask, mask_blue, person_mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(person_mask, person_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(person_mask, person_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(person_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & cnt : contours) {
      double area = cv::contourArea(cnt);
      if (area >= 60.0) {
        cv::Rect bbox = cv::boundingRect(cnt);
        double aspect_ratio = static_cast<double>(bbox.height) / static_cast<double>(bbox.width);

        if (aspect_ratio >= 0.4 && aspect_ratio <= 4.0 && area >= 60.0) {
          PersonIdentity pid = classify_person(frame, bbox);

          // If bbox was just the badge, expand it to human proportions for detection visualization
          cv::Rect det_box = bbox;
          if (aspect_ratio < 1.4) {
            int exp_top = static_cast<int>(bbox.height * 0.8);
            int exp_bot = static_cast<int>(bbox.height * 2.2);
            int exp_side = static_cast<int>(bbox.width * 0.4);
            det_box.y = std::max(0, bbox.y - exp_top);
            det_box.height = std::min(frame.rows - det_box.y, bbox.height + exp_top + exp_bot);
            det_box.x = std::max(0, bbox.x - exp_side);
            det_box.width = std::min(frame.cols - det_box.x, bbox.width + 2 * exp_side);
          }

          smart_home_interfaces::msg::Detection det;
          det.class_name = "person";
          det.confidence = std::min(0.95f, static_cast<float>(0.75f + (area / 10000.0f) * 0.2f));
          det.xmin = det_box.x;
          det.ymin = det_box.y;
          det.xmax = det_box.x + det_box.width;
          det.ymax = det_box.y + det_box.height;
          det.timestamp = this->now();
          det.zone = current_zone_;
          det.identity = pid.name;
          det.is_family = pid.is_family;

          det_array.detections.push_back(det);
        }
      }
    }

    // Hallway obstacle box segmentation (orange/brown box)
    cv::Mat mask_orange;
    cv::inRange(hsv, cv::Scalar(10, 120, 100), cv::Scalar(25, 255, 255), mask_orange);
    cv::morphologyEx(mask_orange, mask_orange, cv::MORPH_OPEN, kernel);
    std::vector<std::vector<cv::Point>> box_contours;
    cv::findContours(mask_orange, box_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & bcnt : box_contours) {
      double area = cv::contourArea(bcnt);
      if (area > 2000.0) {
        cv::Rect bbox = cv::boundingRect(bcnt);
        double aspect_ratio = static_cast<double>(bbox.height) / static_cast<double>(bbox.width);
        if (aspect_ratio >= 0.7 && aspect_ratio <= 1.4) {
          smart_home_interfaces::msg::Detection det;
          det.class_name = "obstacle";
          det.confidence = 0.88f;
          det.xmin = bbox.x;
          det.ymin = bbox.y;
          det.xmax = bbox.x + bbox.width;
          det.ymax = bbox.y + bbox.height;
          det.timestamp = this->now();
          det.zone = current_zone_;
          det.identity = "obstacle_box";
          det.is_family = false;
          det_array.detections.push_back(det);
        }
      }
    }
  }

  void draw_annotations(cv::Mat & frame, const smart_home_interfaces::msg::DetectionArray & det_array) {
    std::string banner = "Zone: " + current_zone_ + " | Mode: " + (use_fallback_ ? "Fallback Detector" : "DNN YOLO");
    cv::rectangle(frame, cv::Point(10, 10), cv::Point(420, 35), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(frame, banner, cv::Point(15, 28), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);

    for (const auto & det : det_array.detections) {
      cv::Rect rect(det.xmin, det.ymin, det.xmax - det.xmin, det.ymax - det.ymin);
      cv::Scalar color;
      std::string label;

      if (det.class_name == "person") {
        if (det.is_family) {
          color = cv::Scalar(0, 200, 0); // Green for family
          label = "FAMILY: " + det.identity + " (" + std::to_string(static_cast<int>(det.confidence * 100)) + "%)";
        } else {
          color = cv::Scalar(0, 0, 255); // Red for outsider
          label = "OUTSIDER (" + std::to_string(static_cast<int>(det.confidence * 100)) + "%)";
        }
      } else {
        color = cv::Scalar(0, 165, 255); // Orange for obstacle
        label = det.class_name + " (" + std::to_string(static_cast<int>(det.confidence * 100)) + "%)";
      }

      cv::rectangle(frame, rect, color, 2);
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      cv::rectangle(frame, cv::Point(rect.x, rect.y - text_size.height - 4),
                    cv::Point(rect.x + text_size.width, rect.y), color, cv::FILLED);
      cv::putText(frame, label, cv::Point(rect.x, rect.y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(255, 255, 255), 1);
    }
  }

  // Members
  std::string model_path_;
  float conf_thresh_{0.35f};
  float nms_thresh_{0.45f};
  int input_width_{640};
  int input_height_{640};

  bool use_fallback_{true};
  cv::dnn::Net net_;
  std::string current_zone_{"hallway"};

  rclcpp::Publisher<smart_home_interfaces::msg::DetectionArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zone_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
