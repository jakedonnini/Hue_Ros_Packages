#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <mutex>
#include "custom_msg/msg/gps_data.hpp"
#include "custom_msg/msg/coordinates.hpp"

using namespace std::chrono_literals;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix;
using std::placeholders::_1;

class KalmanFilter : public rclcpp::Node {
public:
    KalmanFilter() : Node("navigation_node"), dt_(0.05), running_(true) {
        x_ = Vector3d::Zero();
        last_x_ = x_;
        P_ = Matrix3d::Identity();
        F_ = Matrix3d::Identity();
        B_ = Matrix<double, 3, 2>::Zero();

        Q_ << 0.1, 0, 0,
              0, 0.1, 0,
              0, 0, 0.2;

        R_ << 0.2, 0, 0,
              0, 0.2, 0,
              0, 0, 0.2;

        H_ = Matrix3d::Identity();

        gps_subscription_ = this->create_subscription<custom_msg::msg::GpsData>(
            "gps/data", 5, std::bind(&KalmanFilter::gps_callback, this, _1));
        dead_reck_subscription_ = this->create_subscription<custom_msg::msg::Coordinates>(
            "deadReckoning/vel", 5, std::bind(&KalmanFilter::deadReck_callback, this, _1));
        DR_subscription_ = this->create_subscription<custom_msg::msg::GpsData>(
            "deadReckoning/pose", 5, std::bind(&KalmanFilter::deadReck_callback_pose, this, _1));

        kalman_publisher_ = this->create_publisher<custom_msg::msg::GpsData>("kalman/data", 10);
        rotation_publisher_ = this->create_publisher<custom_msg::msg::GpsData>("deadReckoning/rotation", 10);

        processing_thread_ = std::thread(&KalmanFilter::run_processing_loop, this);
        publishing_thread_ = std::thread(&KalmanFilter::run_publishing_loop, this);
    }

    ~KalmanFilter() {
        stop_threads();
    }

private:
    rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr gps_subscription_;
    rclcpp::Subscription<custom_msg::msg::Coordinates>::SharedPtr dead_reck_subscription_;
    rclcpp::Subscription<custom_msg::msg::GpsData>::SharedPtr DR_subscription_;
    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr kalman_publisher_;
    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr rotation_publisher_;

    Vector3d x_, last_x_;
    Matrix3d P_, F_, Q_, R_, H_;
    Matrix<double, 3, 2> B_;

    double dt_;
    bool running_;
    double V_, dV_;
    double DR_x_, DR_y_, DR_angle_;
    double gps_x_, gps_y_, gps_theta_;
    bool new_gps_data_ = false;
    bool encoder_data_updated_ = false;
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
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (encoder_data_updated_) {
                    update_kalman_with_DR();
                    encoder_data_updated_ = false;
                }
                if (new_gps_data_) {
                    update_kalman_with_gps();
                    new_gps_data_ = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt_ * 500)));
        }
    }

    void run_publishing_loop() {
        while (running_) {
            std::lock_guard<std::mutex> lock(mutex_);
            custom_msg::msg::GpsData kal_msg;
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
        Eigen::Vector2d u(V_, dV_);
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilter>());
    rclcpp::shutdown();
    return 0;
}
