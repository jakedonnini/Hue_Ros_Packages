#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/gps_data.hpp"
#include "custom_msg/msg/coordinates.hpp"
#include "custom_msg/msg/two_int.hpp"
#include <cmath>

class DeadReckoning : public rclcpp::Node {
public:
    DeadReckoning() : Node("dead_reckoning"), encoderX(0.0), encoderY(0.0), encoderTheta(0.0),
                       wheelR(9.708), wheelL(64.77), encoderTicks(8192.0 / 2), errorScaler(1), dt(0.05), dir(-1) {
        
        encoder_subscription = this->create_subscription<custom_msg::msg::TwoInt>(
            "encoder", 10, std::bind(&DeadReckoning::encoder_callback, this, std::placeholders::_1));
        
        vel_publisher = this->create_publisher<custom_msg::msg::Coordinates>("deadReckoning/vel", 10);
        pose_publisher = this->create_publisher<custom_msg::msg::GpsData>("deadReckoning/pose", 10);
    }

private:
    void encoder_callback(const custom_msg::msg::TwoInt::SharedPtr msg) {
        double vL = (6.2832 * wheelR * msg->l * errorScaler * dir) / (encoderTicks * dt);
        double vR = (6.2832 * wheelR * msg->r * errorScaler * dir) / (encoderTicks * dt);
        double V = 0.5 * (vR + vL);
        double dV = (vR - vL) / wheelL;

        // Compute the estimated position
        encoderX += dt * V * std::cos(encoderTheta);
        encoderY += dt * V * std::sin(encoderTheta);
        encoderTheta += dt * dV;

        // Publish velocity
        auto coord_msg = custom_msg::msg::Coordinates();
        coord_msg.x = V;
        coord_msg.y = dV;
        coord_msg.toggle = msg->toggle;
        vel_publisher->publish(coord_msg);

        // Publish pose
        auto pose_msg = custom_msg::msg::GpsData();
        pose_msg.x = encoderX;
        pose_msg.y = encoderY;
        pose_msg.angle = encoderTheta;
        pose_publisher->publish(pose_msg);
    }

    rclcpp::Subscription<custom_msg::msg::TwoInt>::SharedPtr encoder_subscription;
    rclcpp::Publisher<custom_msg::msg::Coordinates>::SharedPtr vel_publisher;
    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr pose_publisher;

    // Robot kinematics variables
    double encoderX, encoderY, encoderTheta;
    double wheelR, wheelL;
    double encoderTicks, errorScaler, dt;
    int dir;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeadReckoning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
