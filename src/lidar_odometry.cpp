#include <cmath>
#include <cstddef>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_icp/icp/icp.h"
#include "cev_icp/icp/driver.h"
#include "rclcpp/qos.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LidarOdometry : public rclcpp::Node {
public:
    LidarOdometry(): Node("lidar_odometry"), driver_(driver_init()) {
        this->declare_parameter("odom_frame", "odom");

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarOdometry::scan_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1));
        prev_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("prev_scan",
            rclcpp::QoS(1));
        transformed_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "transformed_scan", rclcpp::QoS(1));

        tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    icp::ICPDriver driver_init() {
        auto method = this->declare_parameter("icp_method", "feature_aware");

        // TODO: load from params
        icp::ICP::Config config;
        config.set("overlap_rate", 0.7);
        auto icp_opt = icp::ICP::from_method(method, config);
        if (!icp_opt) {
            throw std::runtime_error("Invalid ICP method: " + method);
        }

        icp::ICPDriver driver(std::move(icp_opt.value()));

        // driver.set_transform_tolerance(1 * M_PI / 180, 0.01);  // 1 degree, 1 cm
        driver.set_max_iterations(10);

        return driver;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        auto points = get_points(scan);
        if (prev_points_.empty()) {
            prev_points_ = points;
            prev_scan_ = scan;
            return;
        }

        auto result = driver_.converge(prev_points_, points, icp::RBTransform());

        Eigen::Rotation2Dd result_rot(result.transform.rotation);

        if (result.transform.translation.norm() > 0.05
            || std::abs(result_rot.angle()) > 2 * M_PI / 180) {
            current_transform_ = current_transform_.and_then(result.transform);
            prev_points_ = points;
            prev_scan_ = scan;
        }

        std::cout << result.transform.rotation << std::endl;

        RCLCPP_INFO(this->get_logger(), "dx: %f, dy: %f, x: %f, y: %f, iter: %zu, angle: %f",
            result.transform.translation.x(), result.transform.translation.y(),
            current_transform_.translation.x(), current_transform_.translation.y(),
            result.iteration_count, result_rot.angle());

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

        geometry_msgs::msg::TransformStamped t;
        t.header = odom.header;
        t.child_frame_id = odom.child_frame_id;
        t.transform.translation.x = odom.pose.pose.position.x;
        t.transform.translation.y = odom.pose.pose.position.y;
        t.transform.translation.z = odom.pose.pose.position.z;
        t.transform.rotation = odom.pose.pose.orientation;

        tf_->sendTransform(t);

        prev_scan_->header.stamp = scan->header.stamp;
        prev_scan_pub_->publish(*prev_scan_);

        // Create a PointCloud2 message
        std::vector<icp::Vector> transformed_points(points.size());
        for (std::size_t i = 0; i < points.size(); i++) {
            transformed_points[i] = result.transform.apply_to(prev_points_[i]);
        }

        auto cloud = create_pointcloud(transformed_points, scan->header);
        transformed_scan_pub_->publish(cloud);

        pub_->publish(odom);
    }

    std::vector<icp::Vector> get_points(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        std::vector<icp::Vector> points;
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

    sensor_msgs::msg::PointCloud2 create_pointcloud(const std::vector<icp::Vector>& points,
        const std_msgs::msg::Header& header) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 1;        // Unorganized point cloud
        cloud.width = points.size();
        cloud.is_dense = false;  // Allow NaN/inf values
        cloud.is_bigendian = false;

        // Define the fields (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
            sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);

        modifier.resize(points.size());  // Allocate memory

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto& point: points) {
            *iter_x = static_cast<float>(point.x());
            *iter_y = static_cast<float>(point.y());
            *iter_z = 0.0f;  // Assume lidar is 2D
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        return cloud;
    }

    icp::ICPDriver driver_;
    icp::RBTransform current_transform_;
    std::vector<icp::Vector> prev_points_;
    sensor_msgs::msg::LaserScan::SharedPtr prev_scan_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr prev_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_scan_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometry>());
    rclcpp::shutdown();
    return 0;
}