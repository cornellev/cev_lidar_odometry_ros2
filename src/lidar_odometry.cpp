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
#include <chrono>

class LidarOdometry : public rclcpp::Node {
public:
    LidarOdometry(): Node("lidar_odometry"), driver(driver_init()) {
        this->declare_parameter("odom_frame", "odom");

        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
            rclcpp::SensorDataQoS(),
            std::bind(&LidarOdometry::scan_callback, this, std::placeholders::_1));

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1));

        // TODO: make false
        if (this->declare_parameter("show_debug_scans", true)) {
            DebugPublishers publishers;

            publishers.base_scan =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("base_scan", rclcpp::QoS(1));
            publishers.transformed_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "transformed_scan", rclcpp::QoS(1));
            publishers.current_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "current_scan", rclcpp::QoS(1));

            debug_publishers = publishers;
        }

        // TODO: make false
        if (this->declare_parameter("publish_tf", true)) {
            tf = tf2_ros::TransformBroadcaster(this);
        }

        this->declare_parameter("rebase_translation_min_cm", 0.1);
        this->declare_parameter("rebase_angle_min_rad", 5 * M_PI / 180);
        this->declare_parameter("rebase_time_min_ms", 1000);
    }

private:
    struct DebugPublishers {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_scan;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_scan;
    };

    struct BaseScanInfo {
        sensor_msgs::msg::LaserScan::SharedPtr base_scan;
        std::vector<icp::Vector> points;
    };

    icp::ICPDriver driver_init() {
        auto method = this->declare_parameter("icp_method", "trimmed");

        // TODO: load from params
        icp::ICP::Config config;
        config.set("overlap_rate", 0.7);
        auto icp_opt = icp::ICP::from_method(method, config);
        if (!icp_opt) {
            throw std::runtime_error("Invalid ICP method: " + method);
        }

        icp::ICPDriver driver(std::move(icp_opt.value()));

        // TODO: parameterize
        // driver.set_transform_tolerance(1 * M_PI / 180, 0.01);  // 1 degree, 1 cm
        driver.set_max_iterations(30);

        return driver;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        auto points = get_points(scan);

        if (!base_info.has_value()) {
            BaseScanInfo info;
            info.base_scan = scan;
            info.points = points;

            base_info = info;
            return;
        }

        // TODO: maybe add an inverse function so we don't have to swap the order
        auto result = driver.converge(points, base_info->points, icp::RBTransform());

        Eigen::Rotation2Dd result_rot(result.transform.rotation);

        RCLCPP_DEBUG(this->get_logger(), "dx: %f, dy: %f, dtheta: %f",
            result.transform.translation.x(), result.transform.translation.y(),
            result_rot.smallestAngle());
        Eigen::Rotation2Dd final_rot(current_transform.rotation);
        RCLCPP_DEBUG(this->get_logger(), "x: %f, y: %f, theta: %f",
            current_transform.translation.x(), current_transform.translation.y(),
            final_rot.smallestAngle());

        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = scan->header.stamp;
        odom.header.frame_id = this->get_parameter("odom_frame").as_string();
        odom.child_frame_id = scan->header.frame_id;

        odom.pose.pose.position = tf2::toMsg(Eigen::Vector3d(current_transform.translation.x(),
            current_transform.translation.y(), 0));

        Eigen::Rotation2Dd rot(current_transform.rotation);
        Eigen::AngleAxisd angle_axis(rot.smallestAngle(), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat(angle_axis);
        odom.pose.pose.orientation = tf2::toMsg(quat);

        odom_pub->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header = odom.header;
        t.child_frame_id = odom.child_frame_id;
        t.transform.translation.x = odom.pose.pose.position.x;
        t.transform.translation.y = odom.pose.pose.position.y;
        t.transform.translation.z = odom.pose.pose.position.z;
        t.transform.rotation = odom.pose.pose.orientation;

        if (tf.has_value()) {
            tf->sendTransform(t);
        }

        if (debug_publishers.has_value()) {
            debug_publishers->base_scan->publish(create_pointcloud(base_info->points,
                base_info->base_scan->header));

            std::vector<icp::Vector> transformed_points(points.size());
            for (std::size_t i = 0; i < points.size(); i++) {
                transformed_points[i] = result.transform.apply_to(base_info->points[i]);
            }

            auto cloud = create_pointcloud(transformed_points, scan->header);
            debug_publishers->transformed_scan->publish(cloud);

            auto current_cloud = create_pointcloud(points, scan->header);
            debug_publishers->current_scan->publish(current_cloud);
        }

        // update base scan
        rclcpp::Time last_scan_time(base_info->base_scan->header.stamp);
        auto rebase_ms =
            std::chrono::milliseconds(this->get_parameter("rebase_time_min_ms").as_int());

        bool conditions[3] = {
            result.transform.translation.norm()
                > this->get_parameter("rebase_translation_min_cm").as_double(),
            std::abs(result_rot.smallestAngle())
                > this->get_parameter("rebase_angle_min_rad").as_double(),
            this->get_clock()->now() - last_scan_time > rclcpp::Duration(rebase_ms),
        };

        if (conditions[0] || conditions[1] || conditions[2]) {
            current_transform = current_transform.and_then(result.transform);

            base_info->base_scan = scan;
            base_info->points = points;

            RCLCPP_DEBUG(this->get_logger(),
                "Rebasing scan. Reasons: translation: %d, rotation: %d, time: %d", conditions[0],
                conditions[1], conditions[2]);
        }
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

    icp::ICPDriver driver;
    icp::RBTransform current_transform;
    std::optional<BaseScanInfo> base_info;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    std::optional<DebugPublishers> debug_publishers;

    std::optional<tf2_ros::TransformBroadcaster> tf;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometry>());
    rclcpp::shutdown();
    return 0;
}