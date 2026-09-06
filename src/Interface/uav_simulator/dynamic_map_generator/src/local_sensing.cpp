#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
bool has_map = false;
bool has_odom = false;

// 地图参数
double resolution, x_size, y_size, z_size;
Eigen::Vector3d local_range;
double window_forward_size = 12.0;
double window_lateral_size = 10.0;
double window_rear_margin = 0.30;
Eigen::Vector2d intent_direction(1.0, 0.0);

// Depth-camera occlusion model: the local map must only contain geometry the
// robot could actually perceive. Points whose line of sight from the current
// pose passes through a nearer surface (angular bins, nearest-range shadow
// map) are dropped, so occluded geometry (e.g. a wall's far side or a second
// obstacle row behind a wall) never reaches the ESDF/field.
bool occlusion_culling_enabled = true;
double occlusion_bin_deg = 0.5;
double occlusion_range_margin = 0.20;

// 当前UAV位置
Eigen::Vector3d current_position;

// 发布器
ros::Publisher local_map_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    has_odom = true;
}

void intentCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const Eigen::Vector2d direction(msg->twist.linear.x, msg->twist.linear.y);
    if (!direction.allFinite()) {
        ROS_WARN_THROTTLE(1.0, "[local_sensing] Ignoring non-finite human intent.");
        return;
    }

    const double norm = direction.norm();
    if (norm > 1e-6) {
        intent_direction = direction / norm;
    }
}

void mockMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // The static map publisher is latched in the normal benchmark, while the
    // paper's dynamic-update cases publish replacement clouds on the same
    // topic. Refresh the cloud on every message so an update is visible to the
    // intent-aligned local-map window instead of being ignored after startup.
    pcl::PointCloud<pcl::PointXYZ>::Ptr next_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *next_cloud);
    full_cloud = next_cloud;
    kdtree.setInputCloud(full_cloud);
    const bool first = !has_map;
    has_map = true;
    ROS_INFO_THROTTLE(1.0, "[local_sensing] Mock map %s with %lu points.",
                      first ? "received" : "updated", full_cloud->points.size());
}

void pubLocalMap() {
    if (!has_map || !has_odom) return;

    pcl::PointCloud<pcl::PointXYZ> localMap;
    const Eigen::Vector2d e_j = intent_direction;
    const Eigen::Vector2d n_j(-e_j.y(), e_j.x());
    const double longitudinal_min = -window_rear_margin;
    const double longitudinal_max = window_forward_size - window_rear_margin;
    const double lateral_half_size = 0.5 * window_lateral_size;
    const double vertical_half_size = std::max(0.0, local_range.z());

    // Query the circumscribed sphere first, then retain the intent-aligned box.
    const double longitudinal_center =
        0.5 * (longitudinal_min + longitudinal_max);
    const double longitudinal_half_size = 0.5 * window_forward_size;
    const Eigen::Vector2d search_center_xy =
        current_position.head<2>() + longitudinal_center * e_j;
    pcl::PointXYZ center(search_center_xy.x(), search_center_xy.y(), current_position.z());
    const double sensing_radius = std::sqrt(
        longitudinal_half_size * longitudinal_half_size +
        lateral_half_size * lateral_half_size +
        vertical_half_size * vertical_half_size);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(center, sensing_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        localMap.points.reserve(pointIdxRadiusSearch.size());
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            const pcl::PointXYZ& point = full_cloud->points[pointIdxRadiusSearch[i]];
            const Eigen::Vector2d relative_xy(
                point.x - current_position.x(), point.y - current_position.y());
            const double longitudinal = relative_xy.dot(e_j);
            const double lateral = relative_xy.dot(n_j);
            const double vertical = point.z - current_position.z();
            if (longitudinal >= longitudinal_min && longitudinal <= longitudinal_max &&
                std::abs(lateral) <= lateral_half_size &&
                std::abs(vertical) <= vertical_half_size) {
                localMap.points.push_back(point);
            }
        }
    }

    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    // Occlusion culling: angular shadow test from the current pose. Process
    // the window points by ascending range; a point is dropped when a nearer
    // point in the same angular bin (by more than the range margin) already
    // occludes it. This removes back faces of thick geometry and everything
    // hidden behind the first visible surface along each ray.
    const size_t raw_points = localMap.points.size();
    if (occlusion_culling_enabled && raw_points > 1) {
        const int n_az = std::max(1, static_cast<int>(std::ceil(360.0 / occlusion_bin_deg)));
        const int n_el = std::max(1, static_cast<int>(std::ceil(180.0 / occlusion_bin_deg)));
        std::vector<float> min_range(static_cast<size_t>(n_az) * n_el, 1e9f);
        struct Cand { float x, y, z; float r2; };
        std::vector<Cand> cands;
        cands.reserve(raw_points);
        for (const auto& p : localMap.points) {
            const double dx = p.x - current_position.x();
            const double dy = p.y - current_position.y();
            const double dz = p.z - current_position.z();
            cands.push_back({p.x, p.y, p.z,
                             static_cast<float>(dx * dx + dy * dy + dz * dz)});
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b) { return a.r2 < b.r2; });
        pcl::PointCloud<pcl::PointXYZ> visible;
        visible.points.reserve(cands.size());
        for (const auto& c : cands) {
            const double dx = c.x - current_position.x();
            const double dy = c.y - current_position.y();
            const double dz = c.z - current_position.z();
            const double r = std::sqrt(static_cast<double>(c.r2));
            if (r < 1e-9) { visible.points.emplace_back(c.x, c.y, c.z); continue; }
            const double az = std::atan2(dy, dx);              // [-pi, pi]
            const double el = std::asin(std::max(-1.0, std::min(1.0, dz / r)));
            const int ia = std::max(0, std::min(n_az - 1,
                static_cast<int>(std::floor((az + M_PI) / (2.0 * M_PI) * n_az))));
            const int ie = std::max(0, std::min(n_el - 1,
                static_cast<int>(std::floor((el + M_PI / 2.0) / M_PI * n_el))));
            const size_t bin = static_cast<size_t>(ia) * n_el + static_cast<size_t>(ie);
            if (r <= min_range[bin] + occlusion_range_margin) {
                if (r < min_range[bin]) min_range[bin] = static_cast<float>(r);
                visible.points.emplace_back(c.x, c.y, c.z);
            }
        }
        localMap = visible;
    }

    sensor_msgs::PointCloud2 localMapMsg;
    pcl::toROSMsg(localMap, localMapMsg);
    localMapMsg.header.frame_id = "world";
    localMapMsg.header.stamp = ros::Time::now();

    local_map_pub.publish(localMapMsg);
    ROS_INFO_THROTTLE(1.0,
        "[local_sensing] e_J=(%.3f, %.3f), window=%.2fm x %.2fm, rear=%.2fm, points=%zu"
        " (raw=%zu, occl=%s)",
        e_j.x(), e_j.y(), window_forward_size, window_lateral_size,
        window_rear_margin, localMap.points.size(), raw_points,
        occlusion_culling_enabled ? "on" : "off");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_sensing_node");
    ros::NodeHandle nh("~");

    // 获取地图参数
    nh.param("sdf_map/resolution", resolution, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);
    nh.param("sdf_map/local_update_range_x", local_range(0), -1.0);
    nh.param("sdf_map/local_update_range_y", local_range(1), -1.0);
    nh.param("sdf_map/local_update_range_z", local_range(2), -1.0);
    nh.param("fluid_window_forward_size", window_forward_size, 12.0);
    nh.param("fluid_window_lateral_size", window_lateral_size, 10.0);
    nh.param("fluid_window_rear_margin", window_rear_margin, 0.30);
    nh.param("occlusion_culling", occlusion_culling_enabled, true);
    nh.param("occlusion_bin_deg", occlusion_bin_deg, 0.5);
    nh.param("occlusion_range_margin", occlusion_range_margin, 0.20);
    std::string local_map_topic;
    nh.param<std::string>("local_map_topic", local_map_topic, "/sim/local_map");
    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "/sim/odom");
    std::string human_intent_topic;
    nh.param<std::string>("human_intent_topic", human_intent_topic, "/human_intent");

    window_forward_size = std::max(0.1, window_forward_size);
    window_lateral_size = std::max(0.1, window_lateral_size);
    window_rear_margin = std::max(0.0, std::min(window_forward_size, window_rear_margin));

    // 订阅 odom、/mock_map 和世界坐标系的人类意图速度
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1, odomCallback);
    ros::Subscriber map_sub = nh.subscribe("/mock_map", 1, mockMapCallback);
    ros::Subscriber intent_sub = nh.subscribe(human_intent_topic, 10, intentCallback);

    // 发布 /sim/local_map（主题名可由 local_map_topic 参数覆盖，供多 planner
    // 命名空间并行消融使用）
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(local_map_topic, 1);

    ROS_INFO("[local_sensing] Node initialized: %.2fm x %.2fm intent-aligned window "
             "(rear margin %.2fm, vertical half-range %.2fm), intent topic %s.",
             window_forward_size, window_lateral_size, window_rear_margin,
             local_range.z(), human_intent_topic.c_str());

    ros::Rate rate(10.0);

    while (ros::ok()) {
        ros::spinOnce();
        pubLocalMap();
        rate.sleep();
    }

    return 0;
}
