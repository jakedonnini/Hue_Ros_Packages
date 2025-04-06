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
    
    dr_vel_sub_ = this->create_subscription<custom_msg::msg::Coordinates>(
      "deadReckoning/vel", 2, std::bind(&GPSNavigationNode::dr_vel_callback, this, std::placeholders::_1));
    
    // Create publisher
    pwm_pub_ = this->create_publisher<custom_msg::msg::TwoInt>("PWM", 5);

    // PID and state initialization
    Kp_ = 0.5;
    Ki_ = 0.2;
    Kd_ = 20.0;
    Kd_line_ = 0.0;
    integral_ = 0.0;
    previous_error_ = 0.0;

    largeTurn = false;

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
  bool largeTurn;

  // PWM tracking
  float pwmr_old_, pwml_old_;

  // Thread management
  std::mutex mutex_;
  bool running_;
  std::thread processor_thread_;

  // ROS subscriptions and publishers
  rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr kalman_sub_;
  rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr dr_vel_sub_;
  rclcpp::Publisher<custom_msg::msg::TwoInt>::SharedPtr pwm_pub_;

  // Callback functions
  void waypoint_callback(const custom_msg::msg::Coordinates::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoint_buffer_.emplace_back(msg->x, msg->y, msg->toggle);
    RCLCPP_INFO(this->get_logger(), "Received waypoint: x=%.2f, y=%.2f, toggle=%d", 
                msg->x, msg->y, msg->toggle);
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

  // Calculate distance between perpendicular point and line
  float calculate_distance_to_line(float x1, float y1, float x2, float y2) {
    float distPoints = calculate_distance(x1, y1, x2, y2);
    float distToLine = ((x2 - x1) * (y1 - y2) - (x1 - x2) * (y1 - y2)) / distPoints;
    return distToLine;
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

  float constrain(float value, float min, float max) {
    return std::max(min, std::min(max, value));
  }

  // Process waypoints and navigate
  void run_processing_loop() {
    rclcpp::Rate rate(1.0 / deltaT_); // maybe make this every time we get kalman data
    
    while (rclcpp::ok() && running_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Get new waypoint if needed

        if (current_target_ == std::nullopt && !waypoint_buffer_.empty()) {
          prev_waypoint_ = prev_waypoint_holder_;
          auto [x, y, t] = waypoint_buffer_.front();
          waypoint_buffer_.erase(waypoint_buffer_.begin());
          current_target_ = std::make_pair(x, y);
          shouldBePainting_ = t;
          
          RCLCPP_INFO(this->get_logger(), "New target: x=%.2f, y=%.2f, paint=%d", 
                      x, y, shouldBePainting_);
        }
        
        // If we have a target, navigate to it
        float target_x = current_target_->first;
        float target_y = current_target_->second;
        
        // Calculate distance to target
        float distance = calculate_distance(currentX_, currentY_, target_x, target_y);
        
        // Calculate angle to target
        float target_angle = calculate_angle_to_target(currentX_, currentY_, target_x, target_y);
        
        // distance to line
        float line_distance = calculate_distance_to_line(currentX_, currentY_, target_x, target_y);

        RCLCPP_INFO_STREAM(this->get_logger(), 
          "target null: " << (current_target_ == std::nullopt) 
          << ", bufferEmpty: " << !waypoint_buffer_.empty()
          << ", distance: " << distance
          << ", line_distance: " << line_distance);

        // Check if we've reached the waypoint
        if (current_target_ != std::nullopt && distance < 5) { // Threshold distance
          prev_waypoint_holder_ = *current_target_;
          current_target_ = std::nullopt;
          RCLCPP_INFO(this->get_logger(), "\n\n\n\n --------------------------------------\n Hit waypoint (%f, %f) \n --------------------------------------\n\n\n\n", target_x, target_y);
        }

        // Calculate motor speeds
        int pwmAvg = 20; // Adjust as needed

        float P_term = Kp_ * line_distance;

        float thetaError = normalize_angle(target_angle - currentTheta_);
        integral_ += thetaError * deltaT_;
        // Apply integral limits
        if (integral_ > integral_max_) integral_ = integral_max_;
        if (integral_ < integral_min_) integral_ = integral_min_;
        float I_term = Ki_ * integral_;
        
        float D_term = Kd_line_ * (line_distance - previous_error_) / deltaT_;
        previous_error_ = line_distance;
        
        // PD for line centering
        float pwmDel = P_term + D_term;

        // PI for the angle
        float pwmDelTheta = Kd_ * thetaError + I_term;
        pwmDel = constrain(pwmDel, -20, 20);

        // If the angle is within this threshold then move forward or turn
        float largeTurnThreshold = ((M_PI / 180) / 2) * 45; // 45 deg converted to rad, /2 for abs value
        float fineThreshold = ((M_PI / 180) / 2) * 5; // 5 deg converted to rad, /2 for abs value

        if (std::abs(thetaError) > largeTurnThreshold) { // greater than 45 deg
          largeTurn = true; // we have found a big turn
        } 
        
        if (largeTurn && (std::abs(thetaError) > fineThreshold)) {
          pwmAvg = 0;
          // don't worry about line at 0 point
          pwmDel = 0;
        } else {
          largeTurn = false;
          integral_ = 0;
        }

        if (waypoint_buffer_.empty() && current_target_ == std::nullopt) {
          pwmAvg = 0;
          pwmDel = 0;
          pwmDelTheta = 0;
        }

        // slow down close to point but not to 0
        // float constrainedDist = constrain(distance/10, 0.1, 1); // at 40cm away we start to slow down (twice the overshoot)
        // float speed = pwmAvg*constrainedDist;

        RCLCPP_INFO(this->get_logger(), "PWM: AVG: %d, Del: %f, DelT: %f, I: %f", pwmAvg, pwmDel, pwmDelTheta, I_term);        

        int pwmr = static_cast<int>(pwmAvg + pwmDel + pwmDelTheta);
        int pwml = static_cast<int>(pwmAvg - pwmDel - pwmDelTheta);
        
        // Limit PWM values
        pwmr = std::max(-100, std::min(100, pwmr));
        pwml = std::max(-100, std::min(100, pwml));

        // remove dead band
        if (pwmr > 0) {
          pwmr += 39;
        } 
        if (pwmr < 0) {
          pwmr -= 39;
        }

        if (pwml > 0) {
          pwml += 39;
        } 
        if (pwml < 0) {
          pwml -= 39;
        }
        
        // Publish PWM command
        auto pwm_msg = custom_msg::msg::TwoInt();
        pwm_msg.r = pwmr * dir_;
        pwm_msg.l = pwml * dir_;

        int avgSpeed = (pwmr + pwml) / 2;

        // Check if we are not up to speed (than 1/4 of nominal speed then stop painting)
        bool notUpToSpeed = avgSpeed <= ((pwmAvg+39) * 0.25) && shouldBePainting_ == 1;

        bool paintingIncorrect = isPainting_ != shouldBePainting_;

        if (notUpToSpeed){
          pwm_msg.toggle = 0;
        }  else if (shouldBePainting_ == 1) {
          // if we are painting and we are up to speed then turn on
          pwm_msg.toggle = 1;
        } else {
          // if we are not painting then turn off
          pwm_msg.toggle = 0;
        }

        if (isPainting_ && !current_target_ && shouldBePainting_ == 0) {
          // if we are painting and we are not moving then turn off
          pwm_msg.toggle = 0;
          pwm_pub_->publish(pwm_msg);
        } 

        bool sureOff = (pwml == 0 && pwmr == 0);

        if (pwmr != pwmr_old_ || pwml != pwml_old_ || paintingIncorrect || sureOff) {
          // Publish the PWM command
          pwm_pub_->publish(pwm_msg);
          pwmr_old_ = pwmr;
          pwml_old_ = pwml;
        }

        RCLCPP_INFO(this->get_logger(), "PWM: r: %d, l: %d, Waypoints: %d, %d Current Pos: %d, %d TE: %f D: %f", pwmr, pwml, static_cast<int>(target_x), static_cast<int>(target_y), static_cast<int>(currentX_), static_cast<int>(currentY_), thetaError, distance);        
        
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