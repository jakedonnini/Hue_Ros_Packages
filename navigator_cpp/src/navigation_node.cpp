#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <fstream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/coordinates.hpp"
#include "custom_msg/msg/two_int.hpp"
#include "custom_msg/msg/gps_data.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class GPSSubscriberPublisher : public rclcpp::Node
{
public:
  GPSSubscriberPublisher()
  : Node("navigation_node"), running_(true)
  {
    // Kalman Filter Matrices
    dt_ = 0.05;  // time step
    x_ = Eigen::Vector3d::Zero();  // initial state [x, y, theta]
    
    // Define initial state covariance matrix
    P_ = Eigen::Matrix3d::Identity();
    
    // Define state transition matrix
    F_ = Eigen::Matrix3d::Identity();
    
    // Control matrix - will be updated in getEncoderPose
    B_ = Eigen::MatrixXd(3, 2);
    B_ << dt_ * cos(x_(2)), 0,
          dt_ * sin(x_(2)), 0,
          0, dt_;
    
    // Process noise covariance
    Q_ = Eigen::Matrix3d::Zero();
    Q_(0, 0) = 0.4;
    Q_(1, 1) = 0.4;
    Q_(2, 2) = 0.3;
    
    // Measurement noise covariance (GPS noise)
    R_ = Eigen::Matrix2d::Zero();
    R_(0, 0) = 0.11;
    R_(1, 1) = 0.15;
    
    // Observation matrix
    H_ = Eigen::MatrixXd(2, 3);
    H_ << 1, 0, 0,
          0, 1, 0;

    // Read transformation matrix from file
    std::string save_path = "/home/hue/ros2_ws/src/navigator/navigator/transformation_matrix.txt";
    std::tie(Rot_Matrix_, startingAngle_) = readTransformationMatrix(save_path);

    // Subscribe to GPS data
    gps_subscription_ = this->create_subscription<custom_msg::msg::GpsData>(
      "gps/data", 10, std::bind(&GPSSubscriberPublisher::gpsCallback, this, _1));

    // Subscribe to encoder data
    encoder_subscription_ = this->create_subscription<custom_msg::msg::TwoInt>(
      "encoder", 10, std::bind(&GPSSubscriberPublisher::encoderCallback, this, _1));

    // Subscribe to waypoint data
    waypoint_subscription_ = this->create_subscription<custom_msg::msg::Coordinates>(
      "coordinates", 10, std::bind(&GPSSubscriberPublisher::waypointCallback, this, _1));

    // Create publisher for PWM values
    pwm_publisher_ = this->create_publisher<custom_msg::msg::TwoInt>("PWM", 10);

    // Initialize variables
    waypointBuffer_ = {};
    currentTWayPoint_ = std::nullopt;
    pantingToggle_ = 0;
    shouldBePainting_ = false;
    isPainting_ = 0;
    sentToggle_ = false;
    prevWaypoint_ = std::make_pair(0.0, 0.0);
    prevWaypointHolder_ = std::make_pair(0.0, 0.0);
    largeTurn_ = false;
    firstWayPointSent_ = false;

    // Initial GPS values
    latitude_ = 0.0;
    longitude_ = 0.0;
    gpsTheta_ = startingAngle_;

    // Initial PWM values
    pwmr_value_ = 0;
    pwml_value_ = 0;
    dir_ = -1; // set to -1 to invert the forward direction

    // PID constants
    Kp_ = 35.0;
    Ki_ = 0.2;
    Kd_ = 0.1;

    // PID terms
    integral_ = 0.0;
    integral_min_ = -100.0;
    integral_max_ = 100.0;
    previous_error_ = 0.0;

    // Encoder variables
    encoder_left_ = 0;
    encoder_right_ = 0;
    encoder_data_updated_ = false;

    currentX_ = 0.0;
    currentY_ = 0.0;

    // For checking, not used in actual calcs
    encoderX_ = 0.0;
    encoderY_ = 0.0;
    encoderTheta_ = startingAngle_;
    x_(2) = startingAngle_;

    // Constants (change if drive train changes)
    wheelR_ = 9.708;
    wheelL_ = 64.77;
    encoderTicks_ = 8192.0 / 2.0; // only counts half the ticks of encoder
    errorScaler_ = 1.0;

    // Save old values to only send when it changes
    pwmr_value_old_ = 0;
    pwml_value_old_ = 0;
    destickAccum_ = 0;
    pwmAvgAccum_ = 0;

    // GPS
    origin_lat_ = std::nullopt;
    origin_lon_ = std::nullopt;
    new_gps_data_ = false;
    x_gps_cm_ = 0.0;
    y_gps_cm_ = 0.0;

    // Conversion factor for GPS to meters
    lat_to_cm_ = 111139.0 * 100.0; // 100 for cm
    lon_to_cm_ = 111139.0 * 100.0; // Will be updated once origin_lat is set

    // Start threads
    publisher_thread_ = std::thread(&GPSSubscriberPublisher::runPublishLoop, this);
    processor_thread_ = std::thread(&GPSSubscriberPublisher::runProcessingLoop, this);
    logging_thread_ = std::thread(&GPSSubscriberPublisher::logPositions, this);
  }

  ~GPSSubscriberPublisher()
  {
    stopThreads();
  }

private:
  // Subscribers
  rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr gps_subscription_;
  rclcpp::Subscription<custom_msg::msg::TwoInt>::SharedPtr encoder_subscription_;
  rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr waypoint_subscription_;
  
  // Publisher
  rclcpp::Publisher<custom_msg::msg::TwoInt>::SharedPtr pwm_publisher_;
  
  // Kalman filter variables
  double dt_;
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
  Eigen::Matrix3d F_;
  Eigen::MatrixXd B_;
  Eigen::Matrix3d Q_;
  Eigen::Matrix2d R_;
  Eigen::MatrixXd H_;
  
  // Transformation matrix and starting angle
  Eigen::Matrix2d Rot_Matrix_;
  double startingAngle_;
  
  // Waypoint variables
  std::vector<std::tuple<double, double, int>> waypointBuffer_;
  std::optional<std::pair<double, double>> currentTWayPoint_;
  int pantingToggle_;
  bool shouldBePainting_;
  int isPainting_;
  bool sentToggle_;
  std::pair<double, double> prevWaypoint_;
  std::pair<double, double> prevWaypointHolder_;
  bool largeTurn_;
  bool firstWayPointSent_ = false;
  
  // GPS variables
  double latitude_;
  double longitude_;
  double gpsTheta_;
  std::optional<double> origin_lat_;
  std::optional<double> origin_lon_;
  bool new_gps_data_;
  double x_gps_cm_;
  double y_gps_cm_;
  double lat_to_cm_;
  double lon_to_cm_;
  
  // PWM and control variables
  int pwmr_value_;
  int pwml_value_;
  int dir_;
  double Kp_;
  double Ki_;
  double Kd_;
  double integral_;
  double integral_min_;
  double integral_max_;
  double previous_error_;
  int pwmr_value_old_;
  int pwml_value_old_;
  int destickAccum_;
  int pwmAvgAccum_;
  
  // Encoder variables
  int encoder_left_;
  int encoder_right_;
  bool encoder_data_updated_;
  double encoderX_;
  double encoderY_;
  double encoderTheta_;
  double wheelR_;
  double wheelL_;
  double encoderTicks_;
  double errorScaler_;
  
  // Position variables
  double currentX_;
  double currentY_;
  
  // Threading
  std::thread publisher_thread_;
  std::thread processor_thread_;
  std::thread logging_thread_;
  std::mutex lock_;
  bool running_;

  void gpsCallback(const custom_msg::msg::GpsData::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(lock_);
    latitude_ = msg->x;
    longitude_ = msg->y;
    gpsTheta_ = msg->angle;
    new_gps_data_ = true;
  }

  void encoderCallback(const custom_msg::msg::TwoInt::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(lock_);
    encoder_left_ = msg->l;
    encoder_right_ = msg->r;
    isPainting_ = msg->toggle;
    encoder_data_updated_ = true;
  }

  void waypointCallback(const custom_msg::msg::Coordinates::SharedPtr msg)
  {
    std::lock_guard<std::mutex> guard(lock_);
    waypointBuffer_.push_back(std::make_tuple(msg->x, msg->y, msg->toggle));
  }

  void runPublishLoop()
  {
    while (running_) {
      adjustPwmValues();
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)));
    }
  }

  void runProcessingLoop()
  {
    while (running_) {
      // Process encoder data if updated
      {
        std::lock_guard<std::mutex> guard(lock_);
        if (encoder_data_updated_) {
          getEncoderPose();
          encoder_data_updated_ = false;
        }
      }

      // Process GPS data if updated
      {
        std::lock_guard<std::mutex> guard(lock_);
        if (new_gps_data_) {
          updateKalmanWithGps();
          new_gps_data_ = false;
        }
      }

      // Check for waypoints to process
      {
        std::lock_guard<std::mutex> guard(lock_);
        if (!currentTWayPoint_.has_value() && !waypointBuffer_.empty()) {
          auto [x, y, t] = waypointBuffer_.front();
          waypointBuffer_.erase(waypointBuffer_.begin());
          currentTWayPoint_ = std::make_pair(x, y);
          
          // Keep track of spraying state
          if (t == 1) {
            // Toggle every time t is 1
            shouldBePainting_ = !shouldBePainting_;
          }
          sentToggle_ = false;
          pantingToggle_ = t;

          if (!firstWayPointSent_) {
            // When the first waypoint is sent reset the origin so we always start at 0 0
            origin_lat_ = latitude_;
            origin_lon_ = longitude_;
            lon_to_cm_ = 111139.0 * 100.0 * cos(M_PI * origin_lat_.value() / 180.0);
            firstWayPointSent_ = true;
          }
        }
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 500)));
    }
  }

  void getEncoderPose()
  {
    double vL = (6.2832 * wheelR_ * encoder_left_ * errorScaler_ * dir_) / (encoderTicks_ * dt_);
    double vR = (6.2832 * wheelR_ * encoder_right_ * errorScaler_ * dir_) / (encoderTicks_ * dt_);
    double V = 0.5 * (vR + vL);
    double dV = (vR - vL) / wheelL_;

    // Compute the encoder pos without the GPS for reference
    encoderX_ += dt_ * V * cos(encoderTheta_);
    encoderY_ += dt_ * V * sin(encoderTheta_);
    encoderTheta_ += dt_ * dV;

    // Apply rotation matrix to align encoder position with GPS
    Eigen::Vector2d pos(encoderX_, encoderY_);
    Eigen::Vector2d rotated_pos = Rot_Matrix_ * pos;
    encoderX_ = rotated_pos(0);
    encoderY_ = rotated_pos(1);

    // Create extended 3x3 rotation matrix (includes theta)
    Eigen::Matrix3d Rot_Extended = Eigen::Matrix3d::Identity();
    Rot_Extended.block<2, 2>(0, 0) = Rot_Matrix_;

    // Update B Control matrix
    B_(0, 0) = dt_ * cos(x_(2));
    B_(1, 0) = dt_ * sin(x_(2));
    B_(2, 0) = 0;
    B_(0, 1) = 0;
    B_(1, 1) = 0;
    B_(2, 1) = dt_;

    Eigen::Vector2d u(V, dV);
    Eigen::Vector3d stateUpdate = B_ * u;

    // Multiply the u vector and B matrix by the rotation to get into the GPS frame
    Eigen::Vector3d correctedStateUpdate = Rot_Extended * stateUpdate;

    // Predict Step
    x_ = F_ * x_ + correctedStateUpdate;
    P_ = F_ * P_ * F_.transpose() + Q_;
  }

  void updateKalmanWithGps()
  {
    if (!origin_lat_.has_value() || !origin_lon_.has_value()) {
      // Set the origin once we start getting data
      origin_lat_ = latitude_;
      origin_lon_ = longitude_;
      lon_to_cm_ = 111139.0 * 100.0 * cos(M_PI * origin_lat_.value() / 180.0);
    }

    // Convert GPS data to cm relative to origin
    double delta_lat = latitude_ - origin_lat_.value();
    double delta_lon = longitude_ - origin_lon_.value();
    x_gps_cm_ = delta_lon * lon_to_cm_;
    y_gps_cm_ = delta_lat * lat_to_cm_;

    // Measurement update
    Eigen::Vector2d z(x_gps_cm_, y_gps_cm_);
    Eigen::Vector2d y = z - H_ * x_;
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix3d::Identity() - K * H_) * P_;
  }

  std::tuple<double, double, double> getPosError()
  {
    if (!currentTWayPoint_.has_value()) {
      return std::make_tuple(0.0, 0.0, 0.0);
    }

    auto [waypointX, waypointY] = currentTWayPoint_.value();

    // Current positions based on GPS and encoder data
    currentX_ = x_(0);
    currentY_ = x_(1);
    double currentTheta = x_(2);

    double dist2Go = sqrt(pow(currentX_ - waypointX, 2) + pow(currentY_ - waypointY, 2));
    if (dist2Go < 5) { // Threshold saying we hit the point
      prevWaypointHolder_ = std::make_pair(waypointX, waypointY);
      largeTurn_ = false;
      currentTWayPoint_ = std::nullopt;
    }

    double desiredQ = atan2(waypointY - currentY_, waypointX - currentX_);
    double thetaError = desiredQ - currentTheta;

    if (thetaError > M_PI) {
      thetaError -= 2 * M_PI;
    } else if (thetaError < -M_PI) {
      thetaError += 2 * M_PI;
    }

    // Find the distance to the nearest point on the line between waypoints
    double distPoints = sqrt(pow(waypointX - prevWaypoint_.first, 2) + pow(waypointY - prevWaypoint_.second, 2));
    double distToLine = ((waypointX - prevWaypoint_.first) * (prevWaypoint_.second - currentY_) - 
                         (prevWaypoint_.first - currentX_) * (waypointY - prevWaypoint_.second)) / distPoints;

    return std::make_tuple(dist2Go, thetaError, distToLine);
  }

  double constrain(double val, double min_val, double max_val)
  {
    return std::max(min_val, std::min(val, max_val));
  }

  void adjustPwmValues()
  {
    auto [dist, thetaError, distToLine] = getPosError();

    // For painting 40 seems to be pretty good for paint spread
    double pwmAvg = 40; // normally 60 with dead zone

    // PID calculations
    // Proportional term (P)
    double P_term = Kp_ * distToLine;
    
    // Integral term (I)
    integral_ += distToLine;
    integral_ = std::max(integral_min_, std::min(integral_, integral_max_)); // Clamping
    double I_term = Ki_ * integral_;
    
    // Derivative term (D)
    double D_term = Kd_ * (distToLine - previous_error_);
    
    // PID output
    double pid_output = P_term + I_term; // + D_term
    
    // Update the previous error
    previous_error_ = distToLine;

    // Adjust PWM values based on the PID output
    double pwmDel = pid_output;
    double pwmDelTheta = Kd_ * thetaError;

    // If the angle is within this threshold then move forward
    // otherwise stop and turn
    double threshold = 0.20;

    if (std::abs(thetaError) > 0.2618) {
      largeTurn_ = true;
    }

    if (largeTurn_ && std::abs(thetaError) > 0.03) {
      pwmAvg = 0;
    } else {
      largeTurn_ = false;
    }

    if (waypointBuffer_.empty() && !currentTWayPoint_.has_value()) {
      pwmAvg = 0;
      pwmDel = 0;
      pwmDelTheta = 0;
    }

    pwmr_value_ = pwmAvg + pwmDel + pwmDelTheta;
    pwml_value_ = pwmAvg - pwmDel - pwmDelTheta;

    int max_pwm = 128;
    int min_pwm = -128;
    
    pwmr_value_ = constrain(pwmr_value_, min_pwm, max_pwm);
    pwml_value_ = constrain(pwml_value_, min_pwm, max_pwm);

    // Anti-windup: Reduce integral accumulation if PWM is saturated
    if (pwmr_value_ == max_pwm || pwmr_value_ == min_pwm) {
      integral_ -= 0.1 * (pwmr_value_ - (pwmAvg + pwmDel));
    }
    
    if (pwml_value_ == max_pwm || pwml_value_ == min_pwm) {
      integral_ -= 0.1 * (pwml_value_ - (pwmAvg - pwmDel));
    }

    // Remove dead zone between 39 and -39 for L and R
    if (pwmr_value_ > 0) {
      pwmr_value_ += 39;
    }
    if (pwmr_value_ < 0) {
      pwmr_value_ -= 39;
    }

    if (pwml_value_ > 0) {
      pwml_value_ += 39;
    }
    if (pwml_value_ < 0) {
      pwml_value_ -= 39;
    }

    RCLCPP_INFO(this->get_logger(), 
      "PID: Theta error: %.2f PID: %.2f P: %.2f I: %.2f D: %.2f PWM_del %.2f",
      thetaError, pid_output, P_term, I_term, D_term, pwmDel);

    auto pwm_msg = std::make_unique<custom_msg::msg::TwoInt>();
    pwm_msg->r = static_cast<int>(pwmr_value_) * dir_; // dir will flip the direction of movement
    pwm_msg->l = static_cast<int>(pwml_value_) * dir_;

    // If speed is below a threshold then we should stop painting to avoid pooling
    double avgSpeed = (pwmr_value_ + pwml_value_) / 2;

    // If less than 3/4 of nominal speed then stop painting
    // when speed is reached the next block of code should turn sprayer back on
    bool notUpToSpeed = avgSpeed <= (pwmAvg * 0.75) && shouldBePainting_;
    
    // Only send the toggle commands once
    bool paintingIncorrect = static_cast<int>(shouldBePainting_) != isPainting_;

    if (paintingIncorrect || notUpToSpeed) {
      pwm_msg->toggle = 1;
    } else {
      pwm_msg->toggle = 0;
    }

    // If no waypoints make sure the sprayer is off
    if (isPainting_ && !currentTWayPoint_.has_value() && !shouldBePainting_) {
      pwm_msg->toggle = 1;
      pwm_publisher_->publish(*pwm_msg);
    }

    // If wheel still spinning send off again
    bool sureOff = (pwml_value_ == 0 && pwmr_value_ == 0) && (encoder_left_ != 0 || encoder_right_ != 0);

    // Publish the PWM values
    // Only send if new values
    if ((pwmr_value_old_ != pwmr_value_) || (pwml_value_old_ != pwml_value_) || sureOff || paintingIncorrect) {
      pwm_publisher_->publish(*pwm_msg);
      pwmr_value_old_ = pwmr_value_;
      pwml_value_old_ = pwml_value_;
    }

    // For Kalman filter testing
    RCLCPP_INFO(this->get_logger(),
      "GPS: %.2f, %.2f, [ENCODER] X: %.2f Y: %.2f Q: %.2f, Current Pos: %.2f, %.2f Theta error: %.2f dist2go %.2f Waypoint: (%s), PWM R: %d L: %d, PWM AVG %d",
      x_gps_cm_, y_gps_cm_, encoderX_, encoderY_, encoderTheta_, currentX_, currentY_, thetaError, dist,
      currentTWayPoint_.has_value() ? 
        (std::to_string(currentTWayPoint_.value().first) + ", " + std::to_string(currentTWayPoint_.value().second)).c_str() : 
        "null",
      static_cast<int>(pwmr_value_), static_cast<int>(pwml_value_), static_cast<int>(avgSpeed));
  }

  std::pair<Eigen::Matrix2d, double> readTransformationMatrix(const std::string& file_path)
  {
    try {
      std::ifstream file(file_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", file_path.c_str());
        return {Eigen::Matrix2d::Identity(), 0.0};
      }

      std::string line;
      std::vector<std::string> matrix_lines;
      double theta = 0.0;

      while (std::getline(file, line)) {
        if (line.find("[") != std::string::npos || line.find("]") != std::string::npos) {
          matrix_lines.push_back(line);
        }
        if (line.find("Final Encoder Angle") != std::string::npos) {
          std::string angle_str = line.substr(line.find(":") + 1);
          theta = std::stod(angle_str);
        }
      }

      if (matrix_lines.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid matrix format in file");
        return {Eigen::Matrix2d::Identity(), 0.0};
      }

      Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
      
      // Parse first row
      std::string row1 = matrix_lines[0];
      row1 = row1.substr(row1.find("[") + 1);
      row1 = row1.substr(0, row1.find("]"));
      std::istringstream iss1(row1);
      iss1 >> R(0, 0) >> R(0, 1);

      // Parse second row
      std::string row2 = matrix_lines[1];
      row2 = row2.substr(row2.find("[") + 1);
      row2 = row2.substr(0, row2.find("]"));
      std::istringstream iss2(row2);
      iss2 >> R(1, 0) >> R(1, 1);

      return {R, theta};
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error reading transformation matrix file: %s", e.what());
      return {Eigen::Matrix2d::Identity(), 0.0};
    }
  }

  void logPositions()
  {
    try {
      std::ofstream file("/home/hue/ros2_ws/position_log_teleop.txt");
      if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open position log file");
        return;
      }

      file << "Time,GPS_X,GPS_Y,Encoder_X,Encoder_Y,Kalman_X,Kalman_Y,Theta\n";
      
      while (running_) {
        double gps_x, gps_y, encoder_x, encoder_y, kalman_x, kalman_y, theta;
        
        {
          std::lock_guard<std::mutex> guard(lock_);
          gps_x = x_gps_cm_;
          gps_y = y_gps_cm_;
          encoder_x = encoderX_;
          encoder_y = encoderY_;
          kalman_x = x_(0);
          kalman_y = x_(1);
          theta = encoderTheta_;
        }
        
        auto now = std::chrono::system_clock::now();
        auto now_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
        auto timestamp = now_seconds.time_since_epoch().count();
        
        file << timestamp << "," << gps_x << "," << gps_y << "," 
             << encoder_x << "," << encoder_y << "," 
             << kalman_x << "," << kalman_y << "," << theta << "\n";
        
        file.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)));
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write log: %s", e.what());
    }
  }

  void stopThreads()
  {
    running_ = false;
    if (publisher_thread_.joinable()) publisher_thread_.join();
    if (processor_thread_.joinable()) processor_thread_.join();
    if (logging_thread_.joinable()) logging_thread_.join();

    // Ensure the motors turn off on close
    auto pwm_msg = std::make_unique<custom_msg::msg::TwoInt>();
    pwm_msg->r = 0;
    pwm_msg->l = 0;
    pwm_publisher_->publish(*pwm_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GPSSubscriberPublisher>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}