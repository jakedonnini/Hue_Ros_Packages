/*
#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/two_int.hpp"
#include <serial/serial.h>
#include <thread>
#include <mutex>

class ArduinoSerialNode : public rclcpp::Node {
public:
    ArduinoSerialNode() : Node("arduino_serial_node"), running_(true), pwml_value_(0), pwmr_value_(0), isSpraying_(0) {
        // Initialize serial connection
        try {
            ser_.setPort("/dev/ttyRobot1");
            ser_.setBaudrate(460800);
            serial::Timeout to = serial::Timeout::simpleTimeout(10);
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port");
            rclcpp::shutdown();
        }

        // Publishers and Subscribers
        encoder_pub_ = this->create_publisher<custom_msg::msg::TwoInt>("encoder", 5);
        pwm_sub_ = this->create_subscription<custom_msg::msg::TwoInt>(
            "PWM", 5, std::bind(&ArduinoSerialNode::pwm_callback, this, std::placeholders::_1));

        // Start encoder reading thread
        read_thread_ = std::thread(&ArduinoSerialNode::read_encoder_values, this);
    }

    ~ArduinoSerialNode() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (ser_.isOpen()) {
            ser_.close();
        }
    }

private:
    serial::Serial ser_;
    std::mutex ser_mutex_;
    std::atomic<bool> running_;
    std::thread read_thread_;
    rclcpp::Publisher<custom_msg::msg::TwoInt>::SharedPtr encoder_pub_;
    rclcpp::Subscription<custom_msg::msg::TwoInt>::SharedPtr pwm_sub_;
    int pwml_value_, pwmr_value_, isSpraying_;

    void read_encoder_values() {
        while (running_ && rclcpp::ok()) {
            try {
                std::lock_guard<std::mutex> lock(ser_mutex_);
                std::string data = ser_.readline();
                if (!data.empty()) {
                    int left_enc, right_enc, toggleState;
                    if (sscanf(data.c_str(), "%d %d %d", &left_enc, &right_enc, &toggleState) == 3) {
                        auto enc_msg = custom_msg::msg::TwoInt();
                        enc_msg.l = left_enc;
                        enc_msg.r = right_enc;
                        enc_msg.toggle = toggleState;
                        encoder_pub_->publish(enc_msg);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Invalid data received: %s", data.c_str());
                    }
                }
            } catch (const serial::SerialException &e) {
                RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
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
        try {
            std::lock_guard<std::mutex> lock(ser_mutex_);
            std::string cmd = std::to_string(pwml_value_) + " " + std::to_string(pwmr_value_) + " " + std::to_string(isSpraying_) + "\n";
            ser_.write(cmd);
        } catch (const serial::SerialException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send PWM: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/
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
        declare_parameter<int>("baudrate", 460800);

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
