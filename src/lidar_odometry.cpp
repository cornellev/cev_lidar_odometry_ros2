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
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <chrono>

class LidarOdometry : public rclcpp::Node {
public:
    LidarOdometry(): Node("lidar_odometry"), driver(driver_init()) {
        this->declare_parameter("odom_frame", "odom");

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1));

        scan_sub.subscribe(this, "scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        using std::placeholders::_1;
        using std::placeholders::_2;

        if (this->declare_parameter("use_odom_guess", false)) {
            guess_sub.subscribe(this, "odom_guess", rclcpp::SensorDataQoS().get_rmw_qos_profile());
            sync = std::make_shared<message_filters::Synchronizer<sync_policy>>(sync_policy(30),
                scan_sub, guess_sub);
            sync->setAgePenalty(0.5);
            sync->registerCallback(std::bind(&LidarOdometry::scan_guess_callback, this, _1, _2));
        } else {
            scan_sub.registerCallback(std::bind(&LidarOdometry::scan_only_callback, this, _1));
        }

        if (this->declare_parameter("show_debug_scans", false)) {
            DebugPublishers publishers;

            publishers.base_scan =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("base_scan", rclcpp::QoS(1));
            publishers.transformed_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "transformed_scan", rclcpp::QoS(1));
            publishers.current_scan = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "current_scan", rclcpp::QoS(1));

            debug_publishers = publishers;
        }

        if (this->declare_parameter("publish_tf", false)) {
            tf = tf2_ros::TransformBroadcaster(this);
        }

        this->declare_parameter("rebase_translation_min_m", 0.05);        // 5 cm
        this->declare_parameter("rebase_angle_min_rad", 2 * M_PI / 180);  // 2 deg
        this->declare_parameter("rebase_time_min_ms", 1000);
    }

private:
    using sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan,
        nav_msgs::msg::Odometry>;

    struct DebugPublishers {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_scan;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_scan;
    };

    struct BaseScanInfo {
        sensor_msgs::msg::LaserScan::SharedPtr base_scan;
        std::vector<icp::Vector> points;
        std::optional<nav_msgs::msg::Odometry> previous_guess;
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
        driver.set_transform_tolerance(0.1 * M_PI / 180, 0.005);  // 0.1 degree, 5 mm
        driver.set_max_iterations(10);

        return driver;
    }

    void scan_only_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        update_odometry(scan, {});
    }

    void scan_guess_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const nav_msgs::msg::Odometry::SharedPtr guess) {
        update_odometry(scan, *guess);
    }

    void update_odometry(const sensor_msgs::msg::LaserScan::SharedPtr scan,
        std::optional<nav_msgs::msg::Odometry> guess) {
        auto points = get_points(scan);

        if (!base_info.has_value()) {
            BaseScanInfo info;
            info.base_scan = scan;
            info.points = points;
            info.previous_guess = guess;

            base_info = info;
            return;
        }

        icp::RBTransform init;
        if (guess && base_info->previous_guess) {
            Eigen::Quaterniond guess_q;
            Eigen::Vector3d guess_t;
            tf2::fromMsg(guess->pose.pose.orientation, guess_q);
            tf2::fromMsg(guess->pose.pose.position, guess_t);

            Eigen::Quaterniond prev_q;
            Eigen::Vector3d prev_t;
            tf2::fromMsg(base_info->previous_guess->pose.pose.orientation, prev_q);
            tf2::fromMsg(base_info->previous_guess->pose.pose.position, prev_t);

            Eigen::Matrix3d guess_mat = guess_q.toRotationMatrix();
            Eigen::Matrix3d prev_mat = prev_q.toRotationMatrix();

            // we only care about planar rotation
            Eigen::Matrix2d guess_2d = guess_mat.topLeftCorner<2, 2>();
            Eigen::Matrix2d prev_2d = prev_mat.topLeftCorner<2, 2>();

            init.rotation = guess_2d * prev_2d.transpose();
            init.translation = guess_t - init.rotation * prev_t;
        }

        auto result = driver.converge(points, base_info->points, init);

        Eigen::Rotation2Dd result_rot(result.transform.rotation);
        RCLCPP_DEBUG(this->get_logger(), "dx: %f, dy: %f, dtheta: %f, iterations: %zu",
            result.transform.translation.x(), result.transform.translation.y(), result_rot.angle(),
            result.iteration_count);

        // publish debug scans (important that this happens before we update base scans)
        if (debug_publishers.has_value()) {
            auto point_transform = result.transform.inverse();

            debug_publishers->base_scan->publish(create_pointcloud(base_info->points,
                base_info->base_scan->header));

            std::vector<icp::Vector> transformed_points(points.size());
            for (std::size_t i = 0; i < points.size(); i++) {
                transformed_points[i] = point_transform.apply_to(base_info->points[i]);
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
                > this->get_parameter("rebase_translation_min_m").as_double(),
            std::abs(result_rot.smallestAngle())
                > this->get_parameter("rebase_angle_min_rad").as_double(),
            this->get_clock()->now() - last_scan_time > rclcpp::Duration(rebase_ms),
        };

        if (conditions[0] || conditions[1] || conditions[2]) {
            current_transform = current_transform.and_then(result.transform);

            base_info->base_scan = scan;
            base_info->points = points;
            base_info->previous_guess = guess;

            RCLCPP_DEBUG(this->get_logger(),
                "Rebasing scan. Reasons: translation: %d, rotation: %d, time: %d", conditions[0],
                conditions[1], conditions[2]);
        }

        Eigen::Rotation2Dd final_rot(current_transform.rotation);
        RCLCPP_DEBUG(this->get_logger(), "x: %f, y: %f, theta: %f",
            current_transform.translation.x(), current_transform.translation.y(),
            final_rot.smallestAngle());

        // publish odometry and tf
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

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> guess_sub;
    std::shared_ptr<message_filters::Synchronizer<sync_policy>> sync;
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