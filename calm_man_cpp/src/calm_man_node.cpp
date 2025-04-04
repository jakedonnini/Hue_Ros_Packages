#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/coordinates.hpp"
#include "custom_msg/msg/gps_data.hpp"
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>

using std::placeholders::_1;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Vector2d;

class KalmanFilter : public rclcpp::Node {
public:
    KalmanFilter() : Node("navigation_node"), running_(true), dt_(0.05) {
        // Initialize Kalman filter matrices
        x_ = Vector3d::Zero();
        last_x_ = x_;
        P_ = Matrix3d::Identity();
        F_ = Matrix3d::Identity();
        B_ = Matrix3d::Zero();
        Q_ = (Matrix3d() << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.2).finished();
        R_ = (Matrix3d() << 0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2).finished();
        H_ = Matrix3d::Identity();

        // Subscribers
        gps_subscription_ = this->create_subscription<custom_msg::msg::GpsData>(
            "gps/data", 5, std::bind(&KalmanFilter::gps_callback, this, _1));
        dead_reck_subscription_ = this->create_subscription<custom_msg::msg::Coordinates>(
            "deadReckoning/vel", 5, std::bind(&KalmanFilter::deadReck_callback, this, _1));
        DR_subscription_ = this->create_subscription<custom_msg::msg::GpsData>(
            "deadReckoning/pose", 5, std::bind(&KalmanFilter::deadReck_callback_pose, this, _1));

        // Publishers
        kalman_publisher_ = this->create_publisher<custom_msg::msg::GpsData>("kalman/data", 10);
        rotation_publisher_ = this->create_publisher<custom_msg::msg::GpsData>("deadReckoning/rotation", 10);

        // Start processing and publishing threads
        processing_thread_ = std::thread(&KalmanFilter::run_processing_loop, this);
        publishing_thread_ = std::thread(&KalmanFilter::run_publishing_loop, this);
    }

    ~KalmanFilter() {
        stop_threads();
    }

private:
    // ROS 2 variables
    rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr gps_subscription_;
    rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr dead_reck_subscription_;
    rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr DR_subscription_;
    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr kalman_publisher_;
    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr rotation_publisher_;
    
    // Kalman filter matrices
    Vector3d x_, last_x_;
    Matrix3d P_, F_, B_, Q_, R_, H_;

    // Motion and control variables
    double dt_;
    double V_, dV_;
    double DR_x_, DR_y_, DR_angle_;
    double gps_x_, gps_y_, gps_theta_;
    bool new_gps_data_ = false;
    bool encoder_data_updated_ = false;
    bool running_;
    std::mutex mutex_;
    
    std::thread processing_thread_, publishing_thread_;

    void gps_callback(const custom_msg::msg::GpsData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        gps_x_ = msg->x;
        gps_y_ = msg->y;
        gps_theta_ = msg->angle;
        new_gps_data_ = true;
    }

    void deadReck_callback(const custom_msg::msg::Coordinates::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        V_ = msg->x;
        dV_ = msg->y;
        encoder_data_updated_ = true;
    }

    void deadReck_callback_pose(const custom_msg::msg::GpsData::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        DR_x_ = msg->x;
        DR_y_ = msg->y;
        DR_angle_ = msg->angle;
    }

    void run_processing_loop() {
        while (running_) {
            if (encoder_data_updated_) {
                std::lock_guard<std::mutex> lock(mutex_);
                update_kalman_with_DR();
                encoder_data_updated_ = false;
            }
            if (new_gps_data_) {
                std::lock_guard<std::mutex> lock(mutex_);
                update_kalman_with_gps();
                new_gps_data_ = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 500)));
        }
    }

    void run_publishing_loop() {
        while (running_) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto kal_msg = custom_msg::msg::GpsData();
            kal_msg.x = x_[0];
            kal_msg.y = x_[1];
            kal_msg.angle = x_[2];
            kalman_publisher_->publish(kal_msg);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)));
        }
    }

    void update_kalman_with_DR() {
        B_ << dt_ * cos(x_[2]), 0,
              dt_ * sin(x_[2]), 0,
              0, dt_;
        Vector2d u(V_, dV_);
        x_ = F_ * x_ + B_ * u;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update_kalman_with_gps() {
        Vector3d z(gps_x_, gps_y_, gps_theta_);
        Vector3d y = z - H_ * x_;
        Matrix3d S = H_ * P_ * H_.transpose() + R_;
        Matrix3d K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Matrix3d::Identity() - K * H_) * P_;
    }

    void stop_threads() {
        running_ = false;
        if (processing_thread_.joinable()) processing_thread_.join();
        if (publishing_thread_.joinable()) publishing_thread_.join();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
