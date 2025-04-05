// gps_navigation_node.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <optional>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/coordinates.hpp"
#include "custom_msg/msg/gps_data.hpp"
#include "custom_msg/msg/two_int.hpp"

using namespace std::chrono_literals;

class GPSNavigationNode : public rclcpp::Node {
public:
  GPSNavigationNode() : Node("navigation_node") {
    // Create subscriptions
    waypoint_sub_ = this->create_subscription<custom_msg::msg::Coordinates>(
      "coordinates", 10, std::bind(&GPSNavigationNode::waypoint_callback, this, std::placeholders::_1));
    
    kalman_sub_ = this->create_subscription<custom_msg::msg::GpsData>(
      "kalman/data", 2, std::bind(&GPSNavigationNode::kalman_callback, this, std::placeholders::_1));
    
    dr_sub_ = this->create_subscription<custom_msg::msg::GpsData>(
      "deadReckoning/pose", 2, std::bind(&GPSNavigationNode::dr_callback, this, std::placeholders::_1));
    
    dr_vel_sub_ = this->create_subscription<custom_msg::msg::Coordinates>(
      "deadReckoning/vel", 2, std::bind(&GPSNavigationNode::dr_vel_callback, this, std::placeholders::_1));
    
    // Create publisher
    pwm_pub_ = this->create_publisher<custom_msg::msg::TwoInt>("PWM", 5);

    // PID and state initialization
    Kp_ = 0.5;
    Ki_ = 0.1;
    Kd_ = 7.0;
    Kd_line_ = 1.1;
    integral_ = 0.0;
    previous_error_ = 0.0;

    deltaT_ = 0.05;
    dir_ = 1;
    integral_min_ = -50;
    integral_max_ = 50;
    usingGPS_ = true;

    pwmr_old_ = 0;
    pwml_old_ = 0;

    // Start processing thread
    running_ = true;
    processor_thread_ = std::thread(&GPSNavigationNode::run_processing_loop, this);

    RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
  }

  ~GPSNavigationNode() {
    running_ = false;
    if (processor_thread_.joinable()) {
      processor_thread_.join();
    }
  }

private:
  // Waypoint and position tracking
  std::vector<std::tuple<float, float, int>> waypoint_buffer_;
  std::optional<std::pair<float, float>> current_target_;
  std::pair<float, float> prev_waypoint_ = {0.0f, 0.0f};
  std::pair<float, float> prev_waypoint_holder_ = {0.0f, 0.0f};

  // Position tracking
  float currentX_ = 0.0f, currentY_ = 0.0f, currentTheta_ = 0.0f;
  float DR_x_ = 0.0f, DR_y_ = 0.0f, DR_theta_ = 0.0f;
  float kalman_x_ = 0.0f, kalman_y_ = 0.0f, kalman_theta_ = 0.0f;

  // Painting state
  int shouldBePainting_ = 0;
  int isPainting_ = 0;

  // PID controller variables
  float integral_, previous_error_;
  float Kp_, Ki_, Kd_, Kd_line_;
  float deltaT_;
  float integral_min_, integral_max_;
  int dir_;
  bool usingGPS_;

  // PWM tracking
  float pwmr_old_, pwml_old_;

  // Thread management
  std::mutex mutex_;
  bool running_;
  std::thread processor_thread_;

  // ROS subscriptions and publishers
  rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr kalman_sub_;
  rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr dr_sub_;
  rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr dr_vel_sub_;
  rclcpp::Publisher<custom_msg::msg::TwoInt>::SharedPtr pwm_pub_;

  // Callback functions
  void waypoint_callback(const custom_msg::msg::Coordinates::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoint_buffer_.emplace_back(msg->x, msg->y, msg->toggle);
    RCLCPP_INFO(this->get_logger(), "Received waypoint: x=%.2f, y=%.2f, toggle=%d", 
                msg->x, msg->y, msg->toggle);
  }

  void dr_callback(const custom_msg::msg::GpsData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    DR_x_ = msg->x;
    DR_y_ = msg->y;
    DR_theta_ = msg->angle;
    
    if (!usingGPS_) {
      currentX_ = DR_x_;
      currentY_ = DR_y_;
      currentTheta_ = DR_theta_;
    }
  }

  void kalman_callback(const custom_msg::msg::GpsData::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    kalman_x_ = msg->x;
    kalman_y_ = msg->y;
    kalman_theta_ = msg->angle;
    
    if (usingGPS_) {
      currentX_ = kalman_x_;
      currentY_ = kalman_y_;
      currentTheta_ = kalman_theta_;
    }
  }

  void dr_vel_callback(const custom_msg::msg::Coordinates::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    isPainting_ = msg->toggle;
  }

  // Calculate distance between two points
  float calculate_distance(float x1, float y1, float x2, float y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

  // Calculate angle to target
  float calculate_angle_to_target(float x1, float y1, float x2, float y2) {
    return std::atan2(y2 - y1, x2 - x1);
  }

  // Normalize angle to [-π, π]
  float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
  }

  // Process waypoints and navigate
  void run_processing_loop() {
    rclcpp::Rate rate(1.0 / deltaT_);
    
    while (rclcpp::ok() && running_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Get new waypoint if needed
        if (!current_target_ && !waypoint_buffer_.empty()) {
          prev_waypoint_ = prev_waypoint_holder_;
          auto [x, y, t] = waypoint_buffer_.front();
          waypoint_buffer_.erase(waypoint_buffer_.begin());
          current_target_ = std::make_pair(x, y);
          shouldBePainting_ = t;
          
          RCLCPP_INFO(this->get_logger(), "New target: x=%.2f, y=%.2f, paint=%d", 
                      x, y, shouldBePainting_);
        }
        
        // If we have a target, navigate to it
        if (current_target_) {
          float target_x = current_target_->first;
          float target_y = current_target_->second;
          
          // Calculate distance to target
          float distance = calculate_distance(currentX_, currentY_, target_x, target_y);
          
          // Calculate angle to target
          float target_angle = calculate_angle_to_target(currentX_, currentY_, target_x, target_y);
          
          // Calculate motor speeds
          int base_speed = 20; // Adjust as needed

          float error = normalize_angle(target_angle - currentTheta_);
          integral_ += error * deltaT_;
          
          // Apply integral limits
          if (integral_ > integral_max_) integral_ = integral_max_;
          if (integral_ < integral_min_) integral_ = integral_min_;
          
          float derivative = (error - previous_error_) / deltaT_;
          previous_error_ = error;
          
          float output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;

          int pwmr = base_speed - static_cast<int>(steering);
          int pwml = base_speed + static_cast<int>(steering);
          
          // Limit PWM values
          pwmr = std::max(-255, std::min(255, pwmr));
          pwml = std::max(-255, std::min(255, pwml));
          
          // Apply PWM smoothing
          pwmr = 0.8 * pwmr + 0.2 * pwmr_old_;
          pwml = 0.8 * pwml + 0.2 * pwml_old_;
          pwmr_old_ = pwmr;
          pwml_old_ = pwml;
          
          // Publish PWM command
          auto pwm_msg = custom_msg::msg::TwoInt();
          pwm_msg.r = pwmr * dir_;
          pwm_msg.l = pwml * dir_;
          pwm_pub_->publish(pwm_msg);
          
          // Check if we've reached the waypoint
          if (distance < 5) { // Threshold distance
            prev_waypoint_holder_ = *current_target_;
            current_target_ = std::nullopt;
            RCLCPP_INFO(this->get_logger(), "\n\n\n\n --------------------------------------\n Hit waypoint \n --------------------------------------\n\n\n\n");
          }
        }
      }
      
      rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPSNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}