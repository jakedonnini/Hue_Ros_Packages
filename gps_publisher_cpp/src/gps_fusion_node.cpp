#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/coordinates.hpp"
#include "custom_msg/msg/gps_data.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

class GPSFusionNode : public rclcpp::Node {
public:
    GPSFusionNode() : Node("gps_fusion_node"),
                      origin_lat_(0.0), origin_lon_(0.0),
                      lat_to_cm_(111139.0 * 100), lon_to_cm_(0.0),
                      x_mid_zero_(0.0), y_mid_zero_(0.0),
                      origin_set_(false), zero_mid_set_(false) {

        gps_sub1_.subscribe(this, "gps");
        gps_sub2_.subscribe(this, "gps2");

        // sync_.reset(new Sync(MySyncPolicy(10), gps_sub1_, gps_sub2_));
        // sync_->registerCallback(std::bind(&GPSFusionNode::gps_callback, this, _1, _2));

        gps_fusion_pub_ = this->create_publisher<custom_msg::msg::GpsData>("gps/data", 10);
        gps1_cm_pub_ = this->create_publisher<custom_msg::msg::Coordinates>("gps1_cm", 10);
        gps2_cm_pub_ = this->create_publisher<custom_msg::msg::Coordinates>("gps2_cm", 10);
    }

private:
    void gps_callback(const custom_msg::msg::Coordinates::ConstSharedPtr msg1,
                      const custom_msg::msg::Coordinates::ConstSharedPtr msg2) {

        double lat1 = msg1->x;
        double lon1 = msg1->y;
        double lat2 = msg2->x;
        double lon2 = msg2->y;

        if (!origin_set_) {
            origin_lat_ = lat1;
            origin_lon_ = lon1;
            lon_to_cm_ = lat_to_cm_ * std::cos(origin_lat_ * M_PI / 180.0);
            origin_set_ = true;
        }

        double x1 = lat_to_cm_ * (lat1 - origin_lat_);
        double y1 = lon_to_cm_ * (lon1 - origin_lon_);
        double x2 = lat_to_cm_ * (lat2 - origin_lat_);
        double y2 = lon_to_cm_ * (lon2 - origin_lon_);

        double mid_x = (x1 + x2) / 2.0;
        double mid_y = (y1 + y2) / 2.0;

        if (!zero_mid_set_) {
            x_mid_zero_ = mid_x;
            y_mid_zero_ = mid_y;
            zero_mid_set_ = true;
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
        gps_fusion_pub_->publish(gps_data);

        auto gps_coord1 = custom_msg::msg::Coordinates();
        gps_coord1.x = x1;
        gps_coord1.y = y1;
        gps1_cm_pub_->publish(gps_coord1);

        auto gps_coord2 = custom_msg::msg::Coordinates();
        gps_coord2.x = x2;
        gps_coord2.y = y2;
        gps2_cm_pub_->publish(gps_coord2);
    }

    // typedef message_filters::sync_policies::ApproximateTime<
    //     custom_msg::msg::Coordinates,
    //     custom_msg::msg::Coordinates> MySyncPolicy;

    // typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // std::shared_ptr<Sync> sync_;

    message_filters::Subscriber<custom_msg::msg::Coordinates> gps_sub1_;
    message_filters::Subscriber<custom_msg::msg::Coordinates> gps_sub2_;

    rclcpp::Publisher<custom_msg::msg::GpsData>::SharedPtr gps_fusion_pub_;
    rclcpp::Publisher<custom_msg::msg::Coordinates>::SharedPtr gps1_cm_pub_;
    rclcpp::Publisher<custom_msg::msg::Coordinates>::SharedPtr gps2_cm_pub_;

    double origin_lat_;
    double origin_lon_;
    double lat_to_cm_;
    double lon_to_cm_;
    double x_mid_zero_;
    double y_mid_zero_;

    bool origin_set_;
    bool zero_mid_set_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSFusionNode>());
    rclcpp::shutdown();
    return 0;
}
