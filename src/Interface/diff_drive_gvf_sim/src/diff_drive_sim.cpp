#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <diff_drive_gvf_sim/diff_drive_command_limiter.h>

class DiffDriveSimulator {
 public:
  explicit DiffDriveSimulator(ros::NodeHandle& nh) : nh_(nh) {
    nh_.param("sim/odom_topic", odom_topic_, std::string("/sim/odom"));
    nh_.param("sim/cmd_topic", cmd_topic_, std::string("/cmd_vel"));
    nh_.param("sim/world_frame", world_frame_, std::string("world"));
    nh_.param("sim/base_frame", base_frame_, std::string("base_link"));
    nh_.param("sim/update_rate", update_rate_, 50.0);
    nh_.param("sim/path_topic", path_topic_, std::string("/gvf_car/executed_path"));
    nh_.param("sim/path_max_poses", path_max_poses_, 2000);
    // Path grows/republishes on every odom tick otherwise; at update_rate=50Hz that means
    // re-serializing and redrawing a several-thousand-point rviz path 50x/sec once the
    // history fills up. Throttle independently of the odom rate, matching the drone side's
    // odom_visualization (10Hz).
    nh_.param("sim/path_publish_rate", path_publish_rate_, 10.0);
    nh_.param("sim/max_v", command_limit_.max_v, 0.8);
    nh_.param("sim/min_w", command_limit_.min_w, -0.5235);
    nh_.param("sim/max_w", command_limit_.max_w, 0.5235);
    nh_.param("sim/init_x", x_, 0.0);
    nh_.param("sim/init_y", y_, 0.0);
    nh_.param("sim/init_yaw", yaw_, 0.0);

    cmd_sub_ = nh_.subscribe(cmd_topic_, 10, &DiffDriveSimulator::cmdCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    path_msg_.header.frame_id = world_frame_;
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, update_rate_)),
                             &DiffDriveSimulator::timerCallback, this);
  }

 private:
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_ = FLAG_Race::limitDiffDriveCommand(*msg, command_limit_);
  }

  void timerCallback(const ros::TimerEvent& event) {
    const double dt = (event.current_real - event.last_real).toSec();
    if (dt <= 1e-6) return;

    x_ += last_cmd_.linear.x * std::cos(yaw_) * dt;
    y_ += last_cmd_.linear.x * std::sin(yaw_) * dt;
    yaw_ += last_cmd_.angular.z * dt;
    while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
    travelled_ += last_cmd_.linear.x * dt;

    // Wheel joint states (Scout 2, radius 0.165 m): robot_state_publisher only
    // publishes the continuous wheel transforms when /joint_states supplies
    // them; the wheels also rotate with the travelled distance.
    sensor_msgs::JointState js;
    js.header.stamp = event.current_real;
    js.name = {"front_right_wheel_joint", "front_left_wheel_joint",
               "rear_left_wheel_joint", "rear_right_wheel_joint"};
    const double wheel_angle = travelled_ / 0.165;
    js.position = {wheel_angle, wheel_angle, wheel_angle, wheel_angle};
    joint_state_pub_.publish(js);

    nav_msgs::Odometry odom;
    odom.header.stamp = event.current_real;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
    // World-frame linear velocity, matching this workspace's odom contract
    // (so3_quadrotor_simulator publishes world-frame twist, and the GVF
    // planner + benchmark recorder read it that way). Publishing the raw
    // body-frame command here made every downstream consumer see the car
    // driving permanently along +x.
    odom.twist.twist.linear.x = last_cmd_.linear.x * std::cos(yaw_);
    odom.twist.twist.linear.y = last_cmd_.linear.x * std::sin(yaw_);
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular = last_cmd_.angular;
    odom_pub_.publish(odom);

    const double since_last_path = (event.current_real - last_path_publish_time_).toSec();
    if (path_publish_rate_ <= 0.0 || since_last_path >= 1.0 / path_publish_rate_) {
      last_path_publish_time_ = event.current_real;
      geometry_msgs::PoseStamped pose;
      pose.header = odom.header;
      pose.pose = odom.pose.pose;
      path_msg_.header.stamp = odom.header.stamp;
      path_msg_.poses.push_back(pose);
      if (path_max_poses_ > 0 && static_cast<int>(path_msg_.poses.size()) > path_max_poses_) {
        path_msg_.poses.erase(path_msg_.poses.begin(),
                              path_msg_.poses.begin() + (path_msg_.poses.size() - path_max_poses_));
      }
      path_pub_.publish(path_msg_);
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(tf_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher joint_state_pub_;
  ros::Timer timer_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::string odom_topic_ = "/sim/odom";
  std::string cmd_topic_ = "/cmd_vel";
  std::string path_topic_ = "/gvf_car/executed_path";
  std::string world_frame_ = "world";
  std::string base_frame_ = "base_link";
  double update_rate_ = 50.0;
  int path_max_poses_ = 2000;
  double path_publish_rate_ = 10.0;
  ros::Time last_path_publish_time_;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double travelled_ = 0.0;
  FLAG_Race::DiffDriveCommandLimit command_limit_;
  geometry_msgs::Twist last_cmd_;
  nav_msgs::Path path_msg_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_drive_sim");
  ros::NodeHandle nh("~");
  DiffDriveSimulator sim(nh);
  ros::spin();
  return 0;
}
