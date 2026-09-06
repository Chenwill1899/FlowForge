#include <array>
#include <cmath>
#include <string>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

class CarModelVisualization {
 public:
  explicit CarModelVisualization(ros::NodeHandle& nh) : nh_(nh), root_nh_() {
    nh_.param("viz/odom_topic", odom_topic_, std::string("/sim/odom"));
    nh_.param("viz/frame_id", frame_id_, std::string("world"));
    nh_.param("viz/marker_topic", marker_topic_, std::string("/gvf_car/robot_model"));
    nh_.param("viz/single_marker_topic", single_marker_topic_,
              std::string("/gvf_car/robot_marker"));
    // Scout 2 dimensions (urdf/scout2/scout2.urdf): base 0.925x0.38x0.21,
    // wheelbase 0.498, track 0.583, wheel r 0.165 / w 0.117.
    root_nh_.param("gvf_car/vehicle_length", body_length_, 0.925);
    root_nh_.param("gvf_car/vehicle_width", body_width_, 0.38);
    root_nh_.param("gvf_car/vehicle_clearance", vehicle_clearance_, 0.06);
    root_nh_.param("gvf_car/collision_slice_z", collision_slice_z_, 0.10);
    nh_.param("viz/body_height", body_height_, 0.21);
    nh_.param("viz/wheel_radius", wheel_radius_, 0.165);
    nh_.param("viz/wheel_width", wheel_width_, 0.1165);
    body_center_z_ = collision_slice_z_ + 0.5 * body_height_;
    nh_.param("viz/body_center_z", body_center_z_, body_center_z_);
    nh_.param("viz/show_collision_footprint", show_collision_footprint_, true);
    nh_.param("viz/body_color_r", body_color_r_, 0.15);
    nh_.param("viz/body_color_g", body_color_g_, 0.55);
    nh_.param("viz/body_color_b", body_color_b_, 0.95);
    nh_.param("viz/body_alpha", body_alpha_, 0.95);
    nh_.param("viz/wheel_color_r", wheel_color_r_, 0.10);
    nh_.param("viz/wheel_color_g", wheel_color_g_, 0.10);
    nh_.param("viz/wheel_color_b", wheel_color_b_, 0.10);
    nh_.param("viz/wheel_alpha", wheel_alpha_, 1.0);
    nh_.param("viz/model_scale", model_scale_, 0.001);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    single_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(single_marker_topic_, 1);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &CarModelVisualization::odomCallback, this);
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(makeBodyMarker(*msg));
    if (show_collision_footprint_) {
      markers.markers.push_back(makeCollisionFootprintMarker(*msg, 6));
    }

    // Scout 2: wheel centers at +-wheelbase/2 forward, +-track/2 lateral.
    const std::array<std::pair<double, double>, 4> wheel_offsets = {{
        {0.249, 0.29153},
        {0.249, -0.29153},
        {-0.249, 0.29153},
        {-0.249, -0.29153},
    }};

    for (size_t i = 0; i < wheel_offsets.size(); ++i) {
      markers.markers.push_back(makeWheelMarker(*msg, static_cast<int>(i + 1), wheel_offsets[i].first,
                                                wheel_offsets[i].second));
    }

    markers.markers.push_back(makeHeadingMarker(*msg, 5));
    marker_pub_.publish(markers);
    single_marker_pub_.publish(makeCompositeModelMarker(*msg));
  }

  // The UAV RViz layouts use rviz/Marker for the robot mesh, whereas the native car
  // layout uses rviz/MarkerArray. Publish the real Scout 2 model (from robot_simulation's
  // scout_simulator, param/scout2.STL) as one MESH_RESOURCE marker as well, so the layouts
  // can be shared without a message-type mismatch. The STL is in millimetres and exported in
  // a different orientation, so scale by 1e-3 and apply the same roll+90deg / yaw+180deg
  // correction the reference simulator uses.
  visualization_msgs::Marker makeCompositeModelMarker(const nav_msgs::Odometry& odom) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = odom.header.stamp;
    marker.ns = "car_model_compound";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = odom.pose.pose;
    marker.mesh_resource = "package://diff_drive_gvf_sim/meshes/scout2.STL";
    marker.mesh_use_embedded_materials = true;
    marker.scale.x = marker.scale.y = marker.scale.z = model_scale_;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    marker.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(roll + M_PI / 2.0, pitch, yaw + M_PI);
    return marker;
  }

  visualization_msgs::Marker makeBodyMarker(const nav_msgs::Odometry& odom) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = odom.header.stamp;
    marker.ns = "car_model";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = odom.pose.pose.position.x;
    marker.pose.position.y = odom.pose.pose.position.y;
    marker.pose.position.z = body_center_z_;
    marker.pose.orientation = odom.pose.pose.orientation;
    marker.scale.x = body_length_;
    marker.scale.y = body_width_;
    marker.scale.z = body_height_;
    marker.color.r = body_color_r_;
    marker.color.g = body_color_g_;
    marker.color.b = body_color_b_;
    marker.color.a = body_alpha_;
    return marker;
  }

  visualization_msgs::Marker makeWheelMarker(const nav_msgs::Odometry& odom, int id, double x_off,
                                             double y_off) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = odom.header.stamp;
    marker.ns = "car_model";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    const double qw = odom.pose.pose.orientation.w;
    const double qx = odom.pose.pose.orientation.x;
    const double qy = odom.pose.pose.orientation.y;
    const double qz = odom.pose.pose.orientation.z;
    const double yaw =
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    marker.pose.position.x = odom.pose.pose.position.x + c * x_off - s * y_off;
    marker.pose.position.y = odom.pose.pose.position.y + s * x_off + c * y_off;
    marker.pose.position.z = wheel_radius_;
    // Cylinder axis along the axle (y): rotate the z-axis cylinder by roll=pi/2.
    tf::Quaternion q_yaw, q_roll;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q_yaw);
    q_roll = tf::createQuaternionFromRPY(M_PI / 2.0, 0.0, 0.0);
    tf::quaternionTFToMsg(q_yaw * q_roll, marker.pose.orientation);
    marker.scale.x = 2.0 * wheel_radius_;
    marker.scale.y = 2.0 * wheel_radius_;
    marker.scale.z = wheel_width_;
    marker.color.r = wheel_color_r_;
    marker.color.g = wheel_color_g_;
    marker.color.b = wheel_color_b_;
    marker.color.a = wheel_alpha_;
    return marker;
  }

  visualization_msgs::Marker makeHeadingMarker(const nav_msgs::Odometry& odom, int id) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = odom.header.stamp;
    marker.ns = "car_model";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.14;
    marker.scale.z = 0.16;
    marker.color.r = 1.0;
    marker.color.g = 0.35;
    marker.color.b = 0.10;
    marker.color.a = 1.0;

    geometry_msgs::Point start;
    start.x = odom.pose.pose.position.x;
    start.y = odom.pose.pose.position.y;
    start.z = body_center_z_ + 0.02;

    const double qw = odom.pose.pose.orientation.w;
    const double qx = odom.pose.pose.orientation.x;
    const double qy = odom.pose.pose.orientation.y;
    const double qz = odom.pose.pose.orientation.z;
    const double yaw =
        std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    geometry_msgs::Point end = start;
    end.x += 0.55 * body_length_ * std::cos(yaw);
    end.y += 0.55 * body_length_ * std::sin(yaw);

    marker.points.push_back(start);
    marker.points.push_back(end);
    return marker;
  }

  visualization_msgs::Marker makeCollisionFootprintMarker(const nav_msgs::Odometry& odom, int id) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = odom.header.stamp;
    marker.ns = "car_model";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = odom.pose.pose.position;
    marker.pose.position.z = collision_slice_z_ + 0.03;
    marker.pose.orientation = odom.pose.pose.orientation;
    marker.scale.x = 0.035;
    marker.color.r = 1.0;
    marker.color.g = 0.65;
    marker.color.b = 0.15;
    marker.color.a = 0.9;

    const double half_l = 0.5 * body_length_ + vehicle_clearance_;
    const double half_w = 0.5 * body_width_ + vehicle_clearance_;

    geometry_msgs::Point p;
    p.z = 0.0;

    p.x = half_l;
    p.y = half_w;
    marker.points.push_back(p);
    p.x = half_l;
    p.y = -half_w;
    marker.points.push_back(p);
    p.x = -half_l;
    p.y = -half_w;
    marker.points.push_back(p);
    p.x = -half_l;
    p.y = half_w;
    marker.points.push_back(p);
    p.x = half_l;
    p.y = half_w;
    marker.points.push_back(p);
    return marker;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle root_nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher single_marker_pub_;

  std::string odom_topic_ = "/sim/odom";
  std::string frame_id_ = "world";
  std::string marker_topic_ = "/gvf_car/robot_model";
  std::string single_marker_topic_ = "/gvf_car/robot_marker";
  double body_length_ = 0.80;
  double body_width_ = 0.50;
  double vehicle_clearance_ = 0.06;
  double collision_slice_z_ = 0.10;
  double body_height_ = 0.18;
  double wheel_radius_ = 0.09;
  double wheel_width_ = 0.05;
  double body_center_z_ = 0.12;
  bool show_collision_footprint_ = true;
  double body_color_r_ = 0.15;
  double body_color_g_ = 0.55;
  double body_color_b_ = 0.95;
  double body_alpha_ = 0.95;
  double wheel_color_r_ = 0.10;
  double wheel_color_g_ = 0.10;
  double wheel_color_b_ = 0.10;
  double wheel_alpha_ = 1.0;
  double model_scale_ = 0.001;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "car_model_visualization");
  ros::NodeHandle nh("~");
  CarModelVisualization viz(nh);
  ros::spin();
  return 0;
}
