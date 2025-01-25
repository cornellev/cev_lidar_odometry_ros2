#include <cmath>
#include <cstddef>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_icp/icp/icp.h"
#include "cev_icp/icp/driver.h"
#include "rclcpp/qos.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

class LidarOdometry : public rclcpp::Node {
public:
    LidarOdometry(): Node("lidar_odometry"), driver_(driver_init()) {
        this->declare_parameter("odom_frame", "odom");

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarOdometry::scan_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    }

private:
    icp::ICPDriver driver_init() {
        auto method = this->declare_parameter("icp_method", "trimmed");

        icp::ICP::Config config;
        auto icp_opt = icp::ICP::from_method(method, config);  // use defaults for now
        if (!icp_opt) {
            throw std::runtime_error("Invalid ICP method: " + method);
        }

        icp::ICPDriver driver(std::move(icp_opt.value()));

        driver.set_transform_tolerance(1 * M_PI / 180, 0.01);  // 1 degree, 1 cm
        driver.set_max_iterations(100);

        return driver;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        auto points = get_points(scan);
        if (prev_points_.empty()) {
            prev_points_ = points;
            return;
        }

        auto result = driver_.converge(prev_points_, points, icp::RBTransform());
        auto new_rot = result.transform.rotation * current_transform_.rotation;
        auto new_trans = result.transform.rotation * current_transform_.translation
                         + result.transform.translation;
        current_transform_ = icp::RBTransform(new_trans, new_rot);

        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = scan->header.stamp;
        odom.header.frame_id = this->get_parameter("odom_frame").as_string();
        odom.child_frame_id = scan->header.frame_id;

        odom.pose.pose.position = tf2::toMsg(Eigen::Vector3d(current_transform_.translation.x(),
            current_transform_.translation.y(), 0));

        Eigen::Rotation2Dd rot(current_transform_.rotation);
        Eigen::AngleAxisd angle_axis(rot.angle(), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat(angle_axis);
        odom.pose.pose.orientation = tf2::toMsg(quat);

        pub_->publish(odom);
    }

    std::vector<icp::Vector> get_points(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        std::vector<icp::Vector> points(scan->ranges.size());
        for (std::size_t i = 0; i < scan->ranges.size(); i++) {
            auto range = scan->ranges[i];
            if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }

            auto angle = scan->angle_min + i * scan->angle_increment;
            auto x = range * std::cos(angle);
            auto y = range * std::sin(angle);
            points.emplace_back(x, y);
        }

        return points;
    }

    icp::ICPDriver driver_;
    icp::RBTransform current_transform_;
    std::vector<icp::Vector> prev_points_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char* argv[]) {
    icp::ICP::register_builtin_methods();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometry>());
    rclcpp::shutdown();
    return 0;
}
