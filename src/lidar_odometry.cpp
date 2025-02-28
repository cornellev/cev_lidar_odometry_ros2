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
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "message_filters/time_synchronizer.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <chrono>

class LidarOdometry : public rclcpp::Node {
public:
    LidarOdometry(): Node("lidar_odometry"), driver(driver_init()) {
        odom_frame = declare_parameter("odom_frame", "odom");

        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1));

        auto sensor_qos = rclcpp::SensorDataQoS();
        scan_sub.subscribe(this, "scan", sensor_qos.get_rmw_qos_profile());

        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listen = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        using std::placeholders::_1;
        using std::placeholders::_2;

        if (declare_parameter("use_odom_guess", false)) {
            guess_sub.subscribe(this, "odom_guess", sensor_qos.get_rmw_qos_profile());
            sync = std::make_shared<message_filters::Synchronizer<sync_policy>>(sync_policy(30),
                scan_sub, guess_sub);
            sync->setAgePenalty(0.0);
            sync->registerCallback(std::bind(&LidarOdometry::scan_guess_callback, this, _1, _2));

            transform_is_initialized = false;

            RCLCPP_INFO(get_logger(), "Using odometry guess");
        } else {
            scan_sub.registerCallback(std::bind(&LidarOdometry::scan_only_callback, this, _1));

            transform_is_initialized = true;

            RCLCPP_INFO(get_logger(), "Not using odometry guess");
        }

        if (declare_parameter("show_debug_scans", false)) {
            DebugPublishers publishers;

            publishers.base_scan = create_publisher<sensor_msgs::msg::PointCloud2>("base_scan",
                rclcpp::QoS(1));
            publishers.transformed_scan =
                create_publisher<sensor_msgs::msg::PointCloud2>("transformed_scan", rclcpp::QoS(1));
            publishers.current_scan =
                create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", rclcpp::QoS(1));

            debug_publishers = publishers;
            RCLCPP_INFO(get_logger(), "Publishing debug messages");
        } else {
            RCLCPP_INFO(get_logger(), "Not publishing debug scans");
        }

        if (declare_parameter("publish_tf", false)) {
            tf_broadcast = tf2_ros::TransformBroadcaster(this);
            RCLCPP_INFO(get_logger(), "Publishing TF");
        } else {
            RCLCPP_INFO(get_logger(), "Not publishing TF");
        }

        declare_parameter("rebase_translation_min_m", 0.05);  // 5 cm
        declare_parameter("rebase_angle_min_deg", 2.0);       // 2 deg
        declare_parameter("rebase_time_min_ms", 1000);
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
        sensor_msgs::msg::LaserScan::ConstSharedPtr base_scan;
        std::vector<icp::Vector> points;
        std::optional<nav_msgs::msg::Odometry> odom_guess;
    };

    icp::ICPDriver driver_init() {
        auto method = declare_parameter("icp_method", "trimmed");

        // TODO: load from params
        icp::ICP::Config config;
        config.set("overlap_rate", 0.7);
        auto icp_opt = icp::ICP::from_method(method, config);
        if (!icp_opt) {
            throw std::runtime_error("Invalid ICP method: " + method);
        }

        icp::ICPDriver driver(std::move(icp_opt.value()));

        // TODO: parameterize
        driver.set_transform_tolerance(0.01 * M_PI / 180, 0.005);  // 0.01 degree, 5 mm
        driver.set_max_iterations(0);

        return driver;
    }

    void scan_only_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
        update_odometry(scan, {});
    }

    void scan_guess_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
        const nav_msgs::msg::Odometry::ConstSharedPtr& guess) {
        update_odometry(scan, *guess);
    }

    void update_odometry(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
        std::optional<nav_msgs::msg::Odometry> odom_guess) {
        auto points = get_points(scan);

        // return if we don't have a previous scan yet
        if (!base_info) {
            BaseScanInfo info;
            info.base_scan = scan;
            info.points = points;
            info.odom_guess = odom_guess;
            base_info = info;
            return;
        }

        // wait until we can initialize the current odom
        if (!transform_is_initialized) {
            assert(odom_guess.has_value());

            auto transformed_guess = transform_odom(*odom_guess, scan->header.frame_id);
            if (transformed_guess) {
                Eigen::Isometry3d odom_eigen;
                tf2::fromMsg(transformed_guess->pose.pose, odom_eigen);
                current_transform.rotation = odom_eigen.rotation().topLeftCorner<2, 2>();
                current_transform.translation = odom_eigen.translation().head<2>();
                transform_is_initialized = true;
                RCLCPP_INFO(get_logger(), "Initialized odometry from external odometry.");
            } else {
                return;
            }
        }

        // get initial guess if available from odometry
        icp::RBTransform initial_match_guess;
        if (odom_guess) {
            assert(base_info->odom_guess.has_value());
            assert(odom_guess->header.frame_id == odom_frame);
            assert(base_info->odom_guess->header.frame_id == odom_frame);

            // TODO: for now i'm just transforming both, we may consider storing the transformed
            // version?
            auto last_laser_frame_odom = transform_odom(*base_info->odom_guess,
                scan->header.frame_id);
            auto current_laser_frame_odom = transform_odom(*odom_guess, scan->header.frame_id);

            if (last_laser_frame_odom && current_laser_frame_odom) {
                // T_c(p) gives a point in the current laser frame in the odom frame
                Eigen::Isometry3d current_guess_eigen;
                tf2::fromMsg(current_laser_frame_odom->pose.pose, current_guess_eigen);

                // T_p(p) gives a point in the past laser frame in the odom frame
                Eigen::Isometry3d previous_guess_eigen;
                tf2::fromMsg(last_laser_frame_odom->pose.pose, previous_guess_eigen);

                // Want: T_d(p) that gives a point in the past laser frame in the current laser
                // frame. Actually, we will want T_d^-1 for the initial guess.
                //
                // T_d(T_p(p)) = T_c(p)
                // T_d = T_c * T_p^-1
                Eigen::Isometry3d diff = current_guess_eigen * previous_guess_eigen.inverse();

                initial_match_guess.rotation = diff.inverse().rotation().topLeftCorner<2, 2>();
                initial_match_guess.translation = diff.inverse().translation().head<2>();
            }
        }

        // compute transform using initial guess (if available, otherwise identity)
        auto result = driver.converge(base_info->points, points, initial_match_guess);
        auto car_transform = result.transform.inverse();

        Eigen::Rotation2Dd result_rot(car_transform.rotation);
        RCLCPP_DEBUG(get_logger(), "dx: %f, dy: %f, dtheta: %f, iterations: %zu",
            car_transform.translation.x(), car_transform.translation.y(), result_rot.angle(),
            result.iteration_count);

        // publish debug scans (important that this happens before we update base scans)
        if (debug_publishers.has_value()) {
            auto point_transform = car_transform.inverse();

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

        // update base scan if any condition is satisfied
        rclcpp::Time last_scan_time(base_info->base_scan->header.stamp);
        auto rebase_ms = std::chrono::milliseconds(get_parameter("rebase_time_min_ms").as_int());
        double rebase_angle_rad = get_parameter("rebase_angle_min_deg").as_double() * M_PI / 180;
        double rebase_dist_m = get_parameter("rebase_translation_min_m").as_double();

        bool conditions[3] = {
            car_transform.translation.norm() > rebase_dist_m,
            std::abs(result_rot.smallestAngle()) > rebase_angle_rad,
            get_clock()->now() - last_scan_time > rclcpp::Duration(rebase_ms),
        };

        if (conditions[0] || conditions[1] || conditions[2]) {
            current_transform = current_transform.and_then(car_transform);

            base_info->base_scan = scan;
            base_info->points = points;
            base_info->odom_guess = odom_guess;

            RCLCPP_DEBUG(get_logger(),
                "Rebasing scan. Reasons: translation: %d, rotation: %d, time: %d", conditions[0],
                conditions[1], conditions[2]);
        }

        // TODO: incorporate the transform we just got even if we don't rebase
        Eigen::Rotation2Dd final_rot(current_transform.rotation);
        RCLCPP_DEBUG(get_logger(), "x: %f, y: %f, theta: %f", current_transform.translation.x(),
            current_transform.translation.y(), final_rot.smallestAngle());

        // publish odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = scan->header.stamp;
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = scan->header.frame_id;

        odom.pose.pose.position = tf2::toMsg(Eigen::Vector3d(current_transform.translation.x(),
            current_transform.translation.y(), 0));

        Eigen::AngleAxisd angle_axis(final_rot.smallestAngle(), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond quat(angle_axis);
        odom.pose.pose.orientation = tf2::toMsg(quat);

        // TEMP
        // auto transformed_guess = transform_odom(*odom_guess, scan->header.frame_id);
        // odom = transformed_guess.value();

        odom_pub->publish(odom);

        // publish tf
        if (tf_broadcast.has_value()) {
            geometry_msgs::msg::TransformStamped tf_stamped;
            tf_stamped.header = odom.header;
            tf_stamped.child_frame_id = odom.child_frame_id;
            tf_stamped.transform.translation.x = odom.pose.pose.position.x;
            tf_stamped.transform.translation.y = odom.pose.pose.position.y;
            tf_stamped.transform.translation.z = odom.pose.pose.position.z;
            tf_stamped.transform.rotation = odom.pose.pose.orientation;

            tf_broadcast->sendTransform(tf_stamped);
        }
    }

    std::vector<icp::Vector> get_points(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
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

    std::optional<nav_msgs::msg::Odometry> transform_odom(const nav_msgs::msg::Odometry& odom,
        const std::string& target_frame) {
        // auto odom_child_to_target_tf = tf_buffer->lookupTransform(target_frame,
        // odom.child_frame_id,
        //     odom.header.stamp);
        // TODO: use above, doing this because clocks are not synced between computer and robot

        geometry_msgs::msg::TransformStamped odom_child_to_target_tf;
        try {
            odom_child_to_target_tf = tf_buffer->lookupTransform(target_frame, odom.child_frame_id,
                tf2::TimePointZero);
            // TODO: should we catch other tf exceptions as well? which ones do we need to catch
        } catch (const tf2::ConnectivityException& _) {
            // data is likely not in buffer yet, lookup failed
            return {};
        }

        Eigen::Isometry3d odom_child_to_target = tf2::transformToEigen(odom_child_to_target_tf);
        Eigen::Isometry3d odom_to_odom_child;
        tf2::fromMsg(odom.pose.pose, odom_to_odom_child);
        odom_to_odom_child = odom_to_odom_child.inverse();

        RCLCPP_DEBUG(get_logger(), "%f %f %f %f -- %f %f %f %f",
            odom_child_to_target.translation().x(), odom_child_to_target.translation().y(),
            odom_child_to_target.translation().z(),
            odom_child_to_target.rotation().eulerAngles(0, 1, 2)[2],
            odom_to_odom_child.translation().x(), odom_to_odom_child.translation().y(),
            odom_to_odom_child.translation().z(),
            odom_to_odom_child.rotation().eulerAngles(0, 1, 2)[2]);

        Eigen::Isometry3d odom_to_target = odom_child_to_target * odom_to_odom_child;
        RCLCPP_DEBUG(get_logger(), "%f %f %f %f", odom_to_target.translation().x(),
            odom_to_target.translation().y(), odom_to_target.translation().z(),
            odom_to_target.rotation().eulerAngles(0, 1, 2)[2]);

        geometry_msgs::msg::Pose target_pose_odom_frame = tf2::toMsg(odom_to_target.inverse());

        Eigen::Vector3d lin_vel;
        Eigen::Vector3d ang_vel;
        tf2::fromMsg(odom.twist.twist.linear, lin_vel);
        tf2::fromMsg(odom.twist.twist.angular, ang_vel);

        // i don't care about the covariance lol
        nav_msgs::msg::Odometry result;
        result.header = odom.header;
        result.child_frame_id = target_frame;
        result.pose.pose = target_pose_odom_frame;
        tf2::toMsg(odom_child_to_target.rotation() * lin_vel, result.twist.twist.linear);
        tf2::toMsg(odom_child_to_target.rotation() * ang_vel, result.twist.twist.angular);

        return result;
    }

    std::string odom_frame;

    icp::ICPDriver driver;
    bool transform_is_initialized;
    icp::RBTransform current_transform;
    std::optional<BaseScanInfo> base_info;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> guess_sub;
    std::shared_ptr<message_filters::Synchronizer<sync_policy>> sync;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    std::optional<DebugPublishers> debug_publishers;

    std::optional<tf2_ros::TransformBroadcaster> tf_broadcast;

    std::shared_ptr<tf2_ros::TransformListener> tf_listen;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarOdometry>());
    rclcpp::shutdown();
    return 0;
}