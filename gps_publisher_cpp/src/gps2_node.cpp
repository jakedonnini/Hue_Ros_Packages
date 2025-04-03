#include <rclcpp/rclcpp.hpp>
#include "custom_msg/msg/coordinates.hpp"
#include <std_msgs/msg/float64.hpp>
#include <libserialport.h>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>

class GPSPublisher2 : public rclcpp::Node {
public:
    GPSPublisher2() : Node("gps_publisher"), running_(true) {
        coords_publisher_ = this->create_publisher<custom_msg::msg::Coordinates>("gps2", 10);
        
        // symlink in and resolve to port incase it changes
        const char *port_symlink = "/dev/ttyRobot3";
        char real_path[50];
        ssize_t len = readlink(port_symlink, real_path, sizeof(real_path) - 1);
        std::string final_path;

        if (len != -1) {
            real_path[len] = '\0';  // Null-terminate
            final_path = "/dev/" + std::string(real_path);
        } else {
            final_path = port_symlink;  // Use original path if not a symlink
        }

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
        
        if (open_serial(final_path, 9600)) {
            read_thread_ = std::thread(&GPSPublisher2::read_gps_data, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }
    }

    ~GPSPublisher2() {
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
                std::string nmea_sentence(buffer);
                // RCLCPP_INFO(this->get_logger(), "Raw data: %s", buffer);
                if (nmea_sentence.rfind("$G", 0) == 0) { // Only process GPS messages
                    process_nmea_data(nmea_sentence);
                }
            }
        }
    }

    void process_nmea_data(const std::string &nmea_sentence) {
        if (nmea_sentence.rfind("$GPGGA", 0) == 0 || nmea_sentence.rfind("$GNGGA", 0) == 0) {
            parse_gga(nmea_sentence);
        } else if (nmea_sentence.rfind("$GPVTG", 0) == 0) {
            parse_vtg(nmea_sentence);
        }
    }

    void parse_gga(const std::string &nmea_sentence) {
        std::vector<std::string> parts = split(nmea_sentence, ',');
        
        if (parts.size() < 15) {
            RCLCPP_WARN(this->get_logger(), "Incomplete GGA data");
            return;
        }

        double latitude = convert_to_decimal(parts[2], parts[3]);
        double longitude = convert_to_decimal(parts[4], parts[5]);

        if (latitude != 0.0 && longitude != 0.0) {
            publish_gps_data(latitude, longitude);
        }
    }

    void parse_vtg(const std::string &nmea_sentence) {
        std::vector<std::string> parts = split(nmea_sentence, ',');

        if (parts.size() < 9) {
            RCLCPP_WARN(this->get_logger(), "Incomplete VTG data");
            return;
        }

        try {
            double heading = std::stod(parts[1]); // True heading in degrees
            publish_heading(heading);
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Invalid heading data");
        }
    }

    double convert_to_decimal(const std::string &value, const std::string &direction) {
        if (value.empty() || direction.empty()) return 0.0;

        try {
            double degrees = std::stod(value.substr(0, 2));
            double minutes = std::stod(value.substr(2));
            double decimal = degrees + (minutes / 60.0);

            if (direction == "S" || direction == "W") {
                decimal = -decimal;
            }
            return decimal;
        } catch (const std::exception &e) {
            return 0.0;
        }
    }

    void publish_gps_data(double latitude, double longitude) {
        auto gps_msg = custom_msg::msg::Coordinates();
        gps_msg.x = latitude;
        gps_msg.y = longitude;
        coords_publisher_->publish(gps_msg);
    }

    void publish_heading(double heading) {
        auto heading_msg = std_msgs::msg::Float64();
        heading_msg.data = heading;
        heading_publisher_->publish(heading_msg);
    }

    std::vector<std::string> split(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delimiter)) {
            tokens.push_back(item);
        }
        return tokens;
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
    auto node = std::make_shared<GPSPublisher2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
