#include <rclcpp/rclcpp.hpp>
#include "custom_msg/msg/two_int.hpp"
#include <libserialport.h>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>

class ArduinoSerialNode : public rclcpp::Node {
public:
    ArduinoSerialNode() : Node("arduino_serial_node"), running_(true) {
        encoder_pub_ = this->create_publisher<custom_msg::msg::TwoInt>("encoder", 5);
        pwm_sub_ = this->create_subscription<custom_msg::msg::TwoInt>(
            "PWM", 5, std::bind(&ArduinoSerialNode::pwm_callback, this, std::placeholders::_1));
        idk()
        if (open_serial("/dev/ttyRobot1", 460800)) {
            read_thread_ = std::thread(&ArduinoSerialNode::read_encoder_values, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }
    }

    ~ArduinoSerialNode() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        close_serial();
    }

private:
    rclcpp::Publisher<custom_msg::msg::TwoInt>::SharedPtr encoder_pub_;
    rclcpp::Subscription<custom_msg::msg::TwoInt>::SharedPtr pwm_sub_;
    std::thread read_thread_;
    std::mutex serial_mutex_;
    bool running_;
    struct sp_port *port_ = nullptr;

    bool open_serial(const std::string &device, int baudrate) {
        // if (sp_get_port_by_name(device.c_str(), &port_) != SP_OK || sp_open(port_, SP_MODE_READ_WRITE) != SP_OK) {
        //     return false;
        // }
        // sp_set_baudrate(port_, baudrate);
        // sp_set_bits(port_, 8);
        // sp_set_parity(port_, SP_PARITY_NONE);
        // sp_set_stopbits(port_, 1);
        // sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);
        // return true;
        struct sp_port **ports;
        if (sp_list_ports(&ports) == SP_OK) {
            for (int i = 0; ports[i] != NULL; i++) {
                char *port_name = sp_get_port_name(ports[i]);
                RCLCPP_INFO(this->get_logger(), "Detected port: %s", port_name);
            }
            sp_free_port_list(ports);
        } else {
            RCLCPP_INFO(this->get_logger(), "ERORR LISTING PORTS");
        }
        if (sp_get_port_by_name(device.c_str(), &port_) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find serial port: %s", device.c_str());
            return false;
        }
        if (sp_open(port_, SP_MODE_READ_WRITE) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", device.c_str());
            return false;
        }
        if (sp_set_baudrate(port_, baudrate) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate");
            return false;
        }
        sp_set_bits(port_, 8);
        sp_set_parity(port_, SP_PARITY_NONE);
        sp_set_stopbits(port_, 1);
        sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);
        return true;
    }

    void close_serial() {
        if (port_) {
            sp_close(port_);
            sp_free_port(port_);
        }
    }

    void idk() {
        struct sp_port **ports;
    
        // Get the list of available ports
        if (sp_list_ports(&ports) != SP_OK) {
            std::cerr << "Failed to list serial ports." << std::endl;
            return 1;
        }

        // Iterate through the ports and print their names
        std::cout << "Available Serial Ports:" << std::endl;
        for (int i = 0; ports[i] != nullptr; i++) {
            std::cout << " - " << sp_get_port_name(ports[i]) << std::endl;
        }

        // Free the port list
        sp_free_port_list(ports);
        
        return 0;
    }

    void read_encoder_values() {
        char buffer[128];
        while (running_ && rclcpp::ok()) {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            int bytes_read = sp_nonblocking_read(port_, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::istringstream iss(buffer);
                int left_enc, right_enc, toggleState;
                if (iss >> left_enc >> right_enc >> toggleState) {
                    auto enc_msg = custom_msg::msg::TwoInt();
                    enc_msg.l = left_enc;
                    enc_msg.r = right_enc;
                    enc_msg.toggle = toggleState;
                    encoder_pub_->publish(enc_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid data received: %s", buffer);
                }
            }
        }
    }

    void pwm_callback(const custom_msg::msg::TwoInt::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        std::string data = std::to_string(msg->l) + " " + std::to_string(msg->r) + " " + std::to_string(msg->toggle) + "\n";
        sp_blocking_write(port_, data.c_str(), data.length(), 100);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
