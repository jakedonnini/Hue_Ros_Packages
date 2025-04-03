#include <rclcpp/rclcpp.hpp>
#include "custom_msg/msg/coordinates.hpp"
#include <std_msgs/msg/float64.hpp>
#include <libserialport.h>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>

class GPSPublisher : public rclcpp::Node {
public:
    GPSPublisher() : Node("gps_publisher"), running_(true) {
        coords_publisher_ = this->create_publisher<custom_msg::msg::Coordinates>("gps2", 10);
        
        // symlink in and resolve to port incase it changes
        const char *port_symlink = "/dev/ttyRobot2";
        char real_path[50];
        ssize_t len = readlink(port_symlink, real_path, sizeof(real_path) - 1);
        std::string final_path;

        if (len != -1) {
            real_path[len] = '\0';  // Null-terminate
            final_path = "/dev/" + std::string(real_path);
        } else {
            final_path = port_symlink;  // Use original path if not a symlink
        }
        
        if (open_serial(final_path, 9600)) {
            read_thread_ = std::thread(&GPSPublisher::read_gps_data, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }
    }

    ~GPSPublisher() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        close_serial();
    }

private:
    rclcpp::Publisher<custom_msg::msg::Coordinates>::SharedPtr coords_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_publisher_;
    std::thread read_thread_;
    std::mutex serial_mutex_;
    bool running_;
    struct sp_port *port_ = nullptr;

    bool open_serial(const std::string &device, int baudrate) {
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

    void read_gps_data() {
        char buffer[128];
        while (running_ && rclcpp::ok()) {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            int bytes_read = sp_nonblocking_read(port_, buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                process_nmea_data(std::string(buffer));
            }
        }
    }

    void process_nmea_data(const std::string &nmea_sentence) {
        if (nmea_sentence.rfind("$GPGGA", 0) == 0 || nmea_sentence.rfind("$GNGGA", 0) == 0) {
            publish_gps_data(nmea_sentence);
        } else if (nmea_sentence.rfind("$GPVTG", 0) == 0) {
            publish_heading(nmea_sentence);
        }
    }

    void publish_gps_data(const std::string &nmea_sentence) {
        auto gps_msg = custom_msg::msg::Coordinates();
        // Parse latitude and longitude here...
        coords_publisher_->publish(gps_msg);
    }

    void publish_heading(const std::string &nmea_sentence) {
        auto heading_msg = std_msgs::msg::Float64();
        // Parse heading here...
        heading_publisher_->publish(heading_msg);
    }

    std::string resolve_symlink(const std::string &symlink_path) {
        char real_path[50];
        ssize_t len = readlink(symlink_path.c_str(), real_path, sizeof(real_path) - 1);
        if (len != -1) {
            real_path[len] = '\0';
            return std::string(real_path);
        } else {
            return symlink_path;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
