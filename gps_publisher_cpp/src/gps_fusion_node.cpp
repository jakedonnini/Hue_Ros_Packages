#include <rclcpp/rclcpp.hpp>
#include "custom_msg/msg/coordinates.hpp"
#include "custom_msg/msg/gps_data.hpp"
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class GPSFusionNode : public rclcpp::Node {
public:
    GPSFusionNode() : Node("gps_fusion_node"), origin_lat_(0.0), origin_lon_(0.0), x_mid_zero_(0.0), y_mid_zero_(0.0) {
        using namespace message_filters;

        gps_sub1_.subscribe(this, "gps");
        gps_sub2_.subscribe(this, "gps2");

        sync_ = std::make_shared<Synchronizer<sync_policies::ApproximateTime<custom_msg::msg::Coordinates, custom_msg::msg::Coordinates>>>(10);
        sync_->connectInput(gps_sub1_, gps_sub2_);
        sync_->registerCallback(std::bind(&GPSFusionNode::gps_callback, this, std::placeholders::_1, std::placeholders::_2));

        gps_fusion_publisher_ = this->create_publisher<custom_msg::msg::GpsData>("gps/data", 10);
        gps1_cm_publisher_ = this->create_publisher<custom_msg::msg::Coordinates>("gps1_cm", 10);
        gps2_cm_publisher_ = this->create_publisher<custom_msg::msg::Coordinates>("gps2_cm", 10);
    }

private:
    message_filters::Subscriber<custom_msg::msg::Coordinates> gps_sub1_, gps_sub2_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<custom_msg::msg::Coordinates, custom_msg::msg::Coordinates>>> sync_;
    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr gps_fusion_publisher_;
    rclcpp::Publisher<custom_msg::msg::Coordinates>::SharedPtr gps1_cm_publisher_, gps2_cm_publisher_;
    
    double origin_lat_, origin_lon_, lon_to_cm_;
    double x_mid_zero_, y_mid_zero_;
    const double lat_to_cm_ = 111139.0 * 100;

    void gps_callback(const custom_msg::msg::Coordinates::SharedPtr msg1, const custom_msg::msg::Coordinates::SharedPtr msg2) {
        double lat1 = msg1->x, lon1 = msg1->y;
        double lat2 = msg2->x, lon2 = msg2->y;

        if (origin_lat_ == 0.0 && origin_lon_ == 0.0) {
            origin_lat_ = lat1;
            origin_lon_ = lon1;
            lon_to_cm_ = lat_to_cm_ * std::cos(origin_lat_ * M_PI / 180.0);
        }

        double x1 = lat_to_cm_ * (lat1 - origin_lat_);
        double y1 = lon_to_cm_ * (lon1 - origin_lon_);
        double x2 = lat_to_cm_ * (lat2 - origin_lat_);
        double y2 = lon_to_cm_ * (lon2 - origin_lon_);

        double mid_x = (x1 + x2) / 2.0;
        double mid_y = (y1 + y2) / 2.0;

        if (x_mid_zero_ == 0.0 && y_mid_zero_ == 0.0) {
            x_mid_zero_ = mid_x;
            y_mid_zero_ = mid_y;
        }

        double delta_x = x1 - x2;
        double delta_y = y1 - y2;
        double angle = std::atan2(delta_y, delta_x) + M_PI;

        mid_x -= x_mid_zero_;
        mid_y -= y_mid_zero_;
        x1 -= x_mid_zero_;
        x2 -= x_mid_zero_;
        y1 -= y_mid_zero_;
        y2 -= y_mid_zero_;

        auto gps_data = custom_msg::msg::GpsData();
        gps_data.x = mid_x;
        gps_data.y = mid_y;
        gps_data.angle = angle;

        auto gps_coord1 = custom_msg::msg::Coordinates();
        gps_coord1.x = x1;
        gps_coord1.y = y1;

        auto gps_coord2 = custom_msg::msg::Coordinates();
        gps_coord2.x = x2;
        gps_coord2.y = y2;

        gps_fusion_publisher_->publish(gps_data);
        gps1_cm_publisher_->publish(gps_coord1);
        gps2_cm_publisher_->publish(gps_coord2);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
