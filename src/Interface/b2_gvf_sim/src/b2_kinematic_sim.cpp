#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class B2KinematicSim {
 public:
  explicit B2KinematicSim(ros::NodeHandle& nh) {
    nh.param("sim/init_x", x_, 0.0);
    nh.param("sim/init_y", y_, 0.0);
    nh.param("sim/init_yaw", yaw_, 0.0);
    nh.param("sim/init_z", z_, 0.5);
    nh.param("sim/rate", rate_hz_, 50.0);
    nh.param("sim/cmd_timeout", cmd_timeout_, 0.3);
    nh.param<std::string>("sim/odom_topic", odom_topic_, "/sim/odom");
    nh.param<std::string>("sim/cmd_topic", cmd_topic_, "/cmd_vel");
    nh.param<std::string>("sim/frame_id", frame_id_, "world");
    nh.param<std::string>("sim/child_frame_id", child_frame_id_, "b2_base");

    odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    cmd_sub_ = nh.subscribe(cmd_topic_, 10, &B2KinematicSim::cmdCallback, this);
    timer_ = nh.createTimer(ros::Duration(1.0 / std::max(1.0, rate_hz_)),
                            &B2KinematicSim::timerCallback, this);
    last_update_ = ros::Time::now();
    last_cmd_time_ = ros::Time(0);
  }

 private:
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
  }

  static double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void timerCallback(const ros::TimerEvent& event) {
    ros::Time now = event.current_real.isZero() ? ros::Time::now() : event.current_real;
    double dt = last_update_.isZero() ? 1.0 / std::max(1.0, rate_hz_) : (now - last_update_).toSec();
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / std::max(1.0, rate_hz_);
    last_update_ = now;

    geometry_msgs::Twist active_cmd = cmd_;
    if (last_cmd_time_.isZero() || (now - last_cmd_time_).toSec() > cmd_timeout_) {
      active_cmd = geometry_msgs::Twist();
    }

    const double vx = active_cmd.linear.x;
    const double vy = active_cmd.linear.y;
    const double wz = active_cmd.angular.z;
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);

    x_ += (c * vx - s * vy) * dt;
    y_ += (s * vx + c * vy) * dt;
    yaw_ = wrapAngle(yaw_ + wz * dt);

    publish(now, active_cmd);
  }

  void publish(const ros::Time& stamp, const geometry_msgs::Twist& active_cmd) {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
    odom.twist.twist = active_cmd;
    odom_pub_.publish(odom);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.child_frame_id = child_frame_id_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = z_;
    tf::quaternionTFToMsg(q, tf_msg.transform.rotation);
    tf_broadcaster_.sendTransform(tf_msg);
  }

  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;
  ros::Timer timer_;
  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::Twist cmd_;
  ros::Time last_update_;
  ros::Time last_cmd_time_;
  std::string odom_topic_;
  std::string cmd_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.5;
  double yaw_ = 0.0;
  double rate_hz_ = 50.0;
  double cmd_timeout_ = 0.3;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "b2_kinematic_sim");
  ros::NodeHandle nh("~");
  B2KinematicSim sim(nh);
  ros::spin();
  return 0;
}
