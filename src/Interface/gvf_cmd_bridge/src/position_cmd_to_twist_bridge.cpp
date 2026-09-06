#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace {

double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

}  // namespace

// Converts the drone-oriented quadrotor_msgs/PositionCommand output of the (unmodified)
// GVF planner into geometry_msgs/Twist, so ground-robot simulators can be driven without
// touching the planner. "holonomic" rotates the commanded world-frame velocity into the
// body frame (B2-style planar platform); "diff_drive" projects it onto the current heading
// and steers toward the commanded direction (forward-only, non-holonomic platform).
class PositionCommandToTwistBridge {
 public:
  explicit PositionCommandToTwistBridge(ros::NodeHandle& nh) {
    nh.param<std::string>("bridge/odom_topic", odom_topic_, std::string("/sim/odom"));
    nh.param<std::string>("bridge/cmd_in_topic", cmd_in_topic_, std::string("/position_cmd"));
    nh.param<std::string>("bridge/cmd_out_topic", cmd_out_topic_, std::string("/cmd_vel"));
    nh.param<std::string>("bridge/mode", mode_, std::string("holonomic"));
    nh.param("bridge/max_vx", max_vx_, 0.6);
    nh.param("bridge/max_vy", max_vy_, 0.4);
    nh.param("bridge/max_w", max_w_, 1.2);
    nh.param("bridge/yaw_kp", yaw_kp_, 2.0);
    nh.param("bridge/heading_deadband", heading_deadband_, 0.05);

    odom_sub_ = nh.subscribe(odom_topic_, 10, &PositionCommandToTwistBridge::odomCallback, this);
    cmd_sub_ = nh.subscribe(cmd_in_topic_, 10, &PositionCommandToTwistBridge::cmdCallback, this);
    twist_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_out_topic_, 10);
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    has_yaw_ = true;
  }

  void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    if (!has_yaw_) return;

    const double vx_world = msg->velocity.x;
    const double vy_world = msg->velocity.y;
    geometry_msgs::Twist twist;

    if (mode_ == "diff_drive") {
      const double speed = std::hypot(vx_world, vy_world);
      double heading_error = 0.0;
      double v_cmd = 0.0;
      if (speed > heading_deadband_) {
        const double desired_yaw = std::atan2(vy_world, vx_world);
        heading_error = wrapAngle(desired_yaw - yaw_);
        v_cmd = speed * std::max(0.0, std::cos(heading_error));
      }
      twist.linear.x = clamp(v_cmd, 0.0, max_vx_);
      twist.angular.z = clamp(yaw_kp_ * heading_error + msg->yaw_dot, -max_w_, max_w_);
    } else {
      const double c = std::cos(yaw_);
      const double s = std::sin(yaw_);
      const double vx_body = c * vx_world + s * vy_world;
      const double vy_body = -s * vx_world + c * vy_world;
      twist.linear.x = clamp(vx_body, -max_vx_, max_vx_);
      twist.linear.y = clamp(vy_body, -max_vy_, max_vy_);
      twist.angular.z = clamp(msg->yaw_dot, -max_w_, max_w_);
    }

    twist_pub_.publish(twist);
  }

  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher twist_pub_;

  std::string odom_topic_;
  std::string cmd_in_topic_;
  std::string cmd_out_topic_;
  std::string mode_;
  double max_vx_ = 0.6;
  double max_vy_ = 0.4;
  double max_w_ = 1.2;
  double yaw_kp_ = 2.0;
  double heading_deadband_ = 0.05;
  double yaw_ = 0.0;
  bool has_yaw_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "position_cmd_to_twist_bridge");
  ros::NodeHandle nh("~");
  PositionCommandToTwistBridge bridge(nh);
  ros::spin();
  return 0;
}
