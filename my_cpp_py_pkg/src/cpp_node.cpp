#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/two_int.hpp"
#include <libserialport.h>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>

class ArduinoSerialNode : public rclcpp::Node {
public:
    ArduinoSerialNode() : Node("arduino_serial_node"), running_(true) {
        declare_parameter<std::string>("port", "/dev/ttyRobot1");
        declare_parameter<int>("baudrate", 115200);

        std::string port = get_parameter("port").as_string();
        int baudrate = get_parameter("baudrate").as_int();

        if (sp_get_port_by_name(port.c_str(), &serial_port_) != SP_OK ||
            sp_open(serial_port_, SP_MODE_READ_WRITE) != SP_OK ||
            sp_set_baudrate(serial_port_, baudrate) != SP_OK) {
            RCLCPP_ERROR(get_logger(), "Failed to open serial port");
            return;
        }
        sp_set_bits(serial_port_, 8);
        sp_set_parity(serial_port_, SP_PARITY_NONE);
        sp_set_stopbits(serial_port_, 1);
        sp_set_flowcontrol(serial_port_, SP_FLOWCONTROL_NONE);

        encoder_pub_ = create_publisher<custom_msg::msg::TwoInt>("encoder", 5);
        pwm_sub_ = create_subscription<custom_msg::msg::TwoInt>("PWM", 5,
            std::bind(&ArduinoSerialNode::pwm_callback, this, std::placeholders::_1));

        read_thread_ = std::thread(&ArduinoSerialNode::read_encoder_values, this);
    }

    ~ArduinoSerialNode() {
        running_ = false;
        if (read_thread_.joinable()) read_thread_.join();
        if (serial_port_) {
            sp_close(serial_port_);
            sp_free_port(serial_port_);
        }
    }

private:
    void read_encoder_values() {
        while (running_ && rclcpp::ok()) {
            char buffer[128];
            std::lock_guard<std::mutex> lock(serial_mutex_);
            int bytes_read = sp_blocking_read_next(serial_port_, buffer, sizeof(buffer) - 1, 100);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::istringstream iss(buffer);
                int left_enc, right_enc, toggleState;
                if (iss >> left_enc >> right_enc >> toggleState) {
                    auto msg = custom_msg::msg::TwoInt();
                    msg.l = left_enc;
                    msg.r = right_enc;
                    msg.toggle = toggleState;
                    encoder_pub_->publish(msg);
                } else {
                    RCLCPP_WARN(get_logger(), "Invalid data received: %s", buffer);
                }
            }
        }
    }

    void pwm_callback(const custom_msg::msg::TwoInt::SharedPtr msg) {
        pwml_value_ = msg->l;
        pwmr_value_ = msg->r;
        isSpraying_ = msg->toggle;
        send_pwm_to_arduino();
    }

    void send_pwm_to_arduino() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        std::ostringstream oss;
        oss << pwml_value_ << " " << pwmr_value_ << " " << isSpraying_ << "\n";
        sp_blocking_write(serial_port_, oss.str().c_str(), oss.str().length(), 100);
    }

    sp_port *serial_port_ = nullptr;
    rclcpp::Publisher<custom_msg::msg::TwoInt>::SharedPtr encoder_pub_;
    rclcpp::Subscription<custom_msg::msg::TwoInt>::SharedPtr pwm_sub_;
    std::thread read_thread_;
    std::mutex serial_mutex_;
    bool running_;
    int pwml_value_ = 0, pwmr_value_ = 0, isSpraying_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
