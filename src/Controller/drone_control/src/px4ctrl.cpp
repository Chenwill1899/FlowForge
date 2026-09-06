#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <controller_msgs/cmd.h>
#include <deque>
#include <tuple>
#include <algorithm>
#include <mavros_msgs/ExtendedState.h>
#include <quadrotor_msgs/GoalSet.h>
#include <controller_msgs/DroneState.h>
#include <cmath>
#define PI 3.14159265358979
#define VELOCITY2D_CONTROL 0b011111000111 // Velocity control mask for PX4 (VX/VY/VZ/YAW)
// Note: Set mask bits to 1 for unused axes, 0 for used axes. This mask enables VX/VY/VZ/YAW control
class PX4CTRL
{
	public:
	// Member functions
		PX4CTRL();
		void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg);
		void state_cb(const mavros_msgs::State::ConstPtr& msg);
		void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
		void target_cb(const quadrotor_msgs::GoalSet::ConstPtr& msg); // Read waypoints from RVIZ
		double uav_to_goal(double x, double y); // Current distance between UAV and goal point
		void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg); // ego callback function, read position and velocity of ahead point (tracking point)
		void cmd_cb(const controller_msgs::cmd::ConstPtr& msg);
		double plan_yaw_rate(); // Calculate yaw rate for heading control
		geometry_msgs::Point vel_command(double x, double y, double z); // Body-frame velocity control with feedforward+PD
		void info_state(const ros::TimerEvent &e); // Status monitoring
		void main_state(const ros::TimerEvent &e); // Main control logic
		void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);
		void initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); // 处理 /autoloc/initial_pose topic
		bool arm();
		bool disarm();
		bool offboard();
		bool check_rc_auto();
		void publish_drone_state(); // 发布无人机状态的新函数
	//ros
		ros::NodeHandle nh;
		ros::Timer status, zhuantai;
		ros::Subscriber state_sub, rc_sub, twist_sub, target_sub, odom_sub, cmd_sub, extended_state_sub, initial_pose_sub;
		ros::Publisher pubMarkerPointer, local_pos_pub, drone_state_pub;
	//msgs
		quadrotor_msgs::PositionCommand ego;
		visualization_msgs::Marker trackpoint;
		tf::StampedTransform ts; // Used to publish the coordinate system of the UAV's current position
		tf::TransformBroadcaster tfBroadcasterPointer; // Broadcast coordinate axis
		unsigned short velocity_mask = VELOCITY2D_CONTROL;
		mavros_msgs::PositionTarget current_goal;
		mavros_msgs::RCIn rc;
		int rc_channel7_value;
		nav_msgs::Odometry position_msg;
		geometry_msgs::PoseStamped target_pos;
		mavros_msgs::State current_state;
		double position_x, position_y, position_z, now_x, now_y, currvx, currvy, current_yaw, targetpos_x, targetpos_y;
		double ego_pos_x, ego_pos_y, ego_pos_z,feedforward_x, feedforward_y, feedforward_z, ego_yaw; //EGO planner information has position
		bool goal_receive;
		bool get_now_pos;
		bool reach_goal;
		geometry_msgs::Point vel;
		double des_vx, des_vy, des_vz, feed_gain;
	// Parameter tuning section
		double stop_dist;      // Deceleration distance to target
		double max_vel;        // Maximum velocity limit
		double max_beishu;     // Velocity limit multiplier
		double px, dx;         // X-axis PD gains
		double py, dy;         // Y-axis PD gains
		double pz;             // Z-axis P gain
		double yaw_rate_gain;  // Yaw rate gain
		double max_yaw_rate;   // Maximum yaw rate (rad/s)
		double hover_height_;  // Hover altitude
		bool is_stopped;
		double stop_pos_x, stop_pos_y, stop_pos_z;
		ros::Time stop_time;
		std::deque<std::tuple<ros::Time, double, double, double>> position_history;  // Position history queue
		bool is_landed;        // Whether on ground
		bool is_in_air;        // Whether airborne
		bool takeoff_triggered;  // Takeoff trigger flag
		bool landing_triggered; // Landing trigger flag
		bool return_triggered; // Return trigger flag
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		bool is_rc_auto;
		bool reached_hover_height;  // 是否达到悬停高度
		bool is_returned; // 是否返航
		bool is_reached; // 是否到达目标点
		ros::Time last_state_pub_time; // 上次发布状态消息的时间
		ros::Time last_goal_update_time; // 上次目标点更新的时间
		double init_x_, init_y_, init_z_; // 添加原点坐标参数
		int drone_id_; // 添加无人机ID参数
		double init_yaw_; // 初始偏航角
	bool use_autoloc_init_; // 是否使用 autoloc 初始化
	bool autoloc_received_; // 是否收到 autoloc 数据
	ros::Time autoloc_timeout_; // autoloc 超时时间
	ros::Time near_goal_time_; // 距离目标点小于0.5米时的时间
	bool near_goal_recorded_; // 是否已经记录了距离小于0.5米的时间
};

PX4CTRL::PX4CTRL()
{
	goal_receive = false;
	get_now_pos = false;
	bool reach_goal = false;
	// 初始化位置变量
	now_x = 0.0;
	now_y = 0.0;
	status = nh.createTimer(ros::Duration(0.02), &PX4CTRL::main_state, this);
	zhuantai = nh.createTimer(ros::Duration(1), &PX4CTRL::info_state, this);
	state_sub = nh.subscribe("/mavros/state", 10, &PX4CTRL::state_cb, this); // Read topic of flight control status
	rc_sub=nh.subscribe("/mavros/rc/in",10, &PX4CTRL::rc_cb, this); // Read topic of remote controller channel
	// Subscribe to the topic of the EGO planner's planning command
	twist_sub = nh.subscribe("/position_cmd", 10, &PX4CTRL::twist_cb, this);
	//twist_sub = nh.subscribe("/position_cmd", 10, &PX4CTRL::twist_cb, this);
	target_sub = nh.subscribe("/goal_with_id", 10, &PX4CTRL::target_cb, this);
	odom_sub=nh.subscribe("/mavros/local_position/odom", 10, &PX4CTRL::odom_cb, this);
	local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    pubMarkerPointer = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
    ros::param::param("~/hover_height", hover_height_, 1.5);  // Default 1.5m
    ros::param::param("~/stop_dist", stop_dist, 1.0);
    ros::param::param("~/max_vel", max_vel, 1.75);
    ros::param::param("~/max_beishu", max_beishu, 1.5);
    ros::param::param("~/px", px, 1.0);
    ros::param::param("~/dx", dx, 0.25);
    ros::param::param("~/py", py, 1.0);
    ros::param::param("~/dy", dy, 0.25);
    ros::param::param("~/pz", pz, 1.5);
    ros::param::param("~/yaw_rate_gain", yaw_rate_gain, 1.25);
    ros::param::param("~/max_yaw_rate", max_yaw_rate, 100.0);
    cmd_sub = nh.subscribe("/control", 10, &PX4CTRL::cmd_cb, this);
    extended_state_sub = nh.subscribe("/mavros/extended_state", 10, &PX4CTRL::extended_state_cb, this);
    is_stopped = false;
    is_landed = true;
    is_in_air = false;
    takeoff_triggered = false;
    landing_triggered = false;
    return_triggered = false;
    is_returned = false;
	is_reached = false;
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    is_rc_auto = false;
    
    // 初始化DroneState消息的发布器
    drone_state_pub = nh.advertise<controller_msgs::DroneState>("/drone_state", 10);
    
    // 初始化新的变量
    reached_hover_height = false;
    last_state_pub_time = ros::Time::now();
    last_goal_update_time = ros::Time::now();
    
    // 获取初始位置参数
    ros::param::param("~/init_x", init_x_, 0.0);
    ros::param::param("~/init_y", init_y_, 0.0);
    ros::param::param("~/init_z", init_z_, 0.0);
    ros::param::param("~/drone_id", drone_id_, 0);
    ros::param::param("~/use_autoloc_init", use_autoloc_init_, false);
    
    // 获取初始四元数参数并计算偏航角
    double init_qx, init_qy, init_qz, init_qw;
    ros::param::param("~/init_qx", init_qx, 0.0);
    ros::param::param("~/init_qy", init_qy, 0.0);
    ros::param::param("~/init_qz", init_qz, 0.0);
    ros::param::param("~/init_qw", init_qw, 1.0);
    
    // 从四元数计算初始偏航角（只计算一次）
    init_yaw_ = atan2(2.0 * (init_qw * init_qz + init_qx * init_qy), 
                      1.0 - 2.0 * (init_qy * init_qy + init_qz * init_qz));
    
    // 初始化 autoloc 相关变量
    autoloc_received_ = false;
    autoloc_timeout_ = ros::Time::now() + ros::Duration(5.0); // 5秒超时
    near_goal_time_ = ros::Time(0); // 初始化为0
    near_goal_recorded_ = false; // 初始化为false
    
    // 如果启用 autoloc 初始化，订阅 topic
    if (use_autoloc_init_) {
        initial_pose_sub = nh.subscribe("/autoloc/initial_pose", 1, &PX4CTRL::initial_pose_cb, this);
        ROS_INFO("Waiting for /autoloc/initial_pose topic for initialization...");
    } else {
        ROS_INFO("Using manual init parameters: x=%.2f, y=%.2f, z=%.2f", init_x_, init_y_, init_z_);
    }
}

void PX4CTRL::rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
    rc = *msg;
    if(rc.channels.size() > 6){
        rc_channel7_value = rc.channels[6];
        // Update safety flag in real-time
        is_rc_auto = (rc_channel7_value > 1400) && (rc_channel7_value < 1600);
    }else{
        ROS_WARN_THROTTLE(1, "RC channels data incomplete!");
        is_rc_auto = false; 
    }
}

void PX4CTRL::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

//read vehicle odometry
void PX4CTRL::odom_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_pos";
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tfBroadcasterPointer.sendTransform(ts);
    if(!get_now_pos)
    {
        // 只有当里程计数据合理时才更新起飞位置
        if(fabs(msg->pose.pose.position.x) > 0.01 || fabs(msg->pose.pose.position.y) > 0.01)
        {
            now_x = msg->pose.pose.position.x;
            now_y = msg->pose.pose.position.y;
            get_now_pos = true;
            ROS_INFO("Update takeoff position: now_x=%.3f, now_y=%.3f", now_x, now_y);
        }
    }
    position_x = position_msg.pose.pose.position.x;
    position_y = position_msg.pose.pose.position.y;
    position_z = position_msg.pose.pose.position.z;
    currvx = position_msg.twist.twist.linear.x;
    currvy = position_msg.twist.twist.linear.y;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat); // Convert the quaternion from mavros/local_position/pose to tf2::Quaternion quat
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    
    // Maintain position history for 0.3 seconds
    position_history.emplace_back(ros::Time::now(), position_x, position_y, position_z);
    
    // Remove old data beyond 0.3 seconds
    while (!position_history.empty() && 
          (ros::Time::now() - std::get<0>(position_history.front())).toSec() > 0.3) {
        position_history.pop_front();
    }
}

void PX4CTRL::target_cb(const quadrotor_msgs::GoalSet::ConstPtr& msg)
{
    // 只在目标位置变化且drone_id匹配时更新标志
    if(msg->drone_id == drone_id_ && (msg->goal[0] != targetpos_x || msg->goal[1] != targetpos_y)) {
        targetpos_x = msg->goal[0];
        targetpos_y = msg->goal[1];
        goal_receive = true;
        is_reached = false;
        last_goal_update_time = ros::Time::now(); // 记录目标点更新时间
    }
    // 否则保持原有状态（不强制设为true）
}

double PX4CTRL::uav_to_goal(double x, double y)
{
	double dist = sqrt(pow(x - position_x, 2) + pow(y - position_y, 2));
	return dist;
}

void PX4CTRL::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego callback function
{
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
    // Speed can be used as feedforward, according to needs, here the ego speed is converted to the body system
    feedforward_x = ego.velocity.x * cos(current_yaw) + ego.velocity.y * sin(current_yaw);
	feedforward_y = -ego.velocity.x * sin(current_yaw) + ego.velocity.y * cos(current_yaw);
	feedforward_z = ego.velocity.z;
    ego_yaw = ego.yaw;
    // Tracking point visualization
    trackpoint.header.frame_id = "world";
    trackpoint.ns = "track_drone";
    trackpoint.id = 0;
    trackpoint.type = visualization_msgs::Marker::SPHERE;
    trackpoint.action = visualization_msgs::Marker::ADD;
    trackpoint.scale.x = 0.3;
    trackpoint.scale.y = 0.3;
    trackpoint.scale.z = 0.3;
    trackpoint.color.a = 1.0;
    trackpoint.color.r = 0.0;
    trackpoint.color.g = 1.0;
    trackpoint.color.b = 0.0;
    trackpoint.pose.position.x = ego_pos_x;
    trackpoint.pose.position.y = ego_pos_y;
    trackpoint.pose.position.z = ego_pos_z;
    pubMarkerPointer.publish(trackpoint);
}

double PX4CTRL::plan_yaw_rate()
{
	double the; // Polar coordinate system angle
    the = ego_yaw - current_yaw;
    // Limit the range of the angle
    if (the > PI)
        the -= 2 * PI;
    else if (the < -PI)
        the += 2 * PI;
    if(the*180/PI > max_yaw_rate)
        the = max_yaw_rate*PI/180;
    else if(the*180/PI < -max_yaw_rate)
        the =-max_yaw_rate*PI/180;
    return the;
}

geometry_msgs::Point PX4CTRL::vel_command(double x, double y, double z)
{
    vel.x = (x - position_x) * cos(current_yaw) + (y - position_y) * sin(current_yaw);
    vel.y = -(x - position_x)*sin(current_yaw) + (y - position_y)*cos(current_yaw);
    vel.z = z - position_z;
    return vel;
}

void PX4CTRL::info_state(const ros::TimerEvent &e)
{
	if(!goal_receive)
	{
		std::cout<<"wait for goal"<<std::endl;
	}
	if(goal_receive && !is_reached)
	{
		std::cout<<"trrige_and_go_to_goal"<<std::endl;
	}
	if(is_reached && goal_receive)
	{
		std::cout<<"reach_goal"<<std::endl;
	}
	if(is_rc_auto)
	{
		std::cout<<"Auto: Takeoff or Land by Rviz"<<std::endl;
	}else{
		std::cout<<"Manual: Takeoff or Land by RC"<<std::endl;
	}
}

void PX4CTRL::main_state(const ros::TimerEvent &e)
{
    // 检查 autoloc 超时，如果启用但超时未收到数据，则回退到手动参数
    if (use_autoloc_init_ && !autoloc_received_ && ros::Time::now() > autoloc_timeout_) {
        ROS_WARN("Autoloc initialization timeout! Falling back to manual parameters.");
        use_autoloc_init_ = false; // 禁用 autoloc，避免重复警告
        initial_pose_sub.shutdown(); // 停止订阅
    }
	if(!goal_receive && (takeoff_triggered || !is_rc_auto))
	{
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal.header.stamp = ros::Time::now();
		current_goal.type_mask = velocity_mask;
		// 计算世界坐标系下的位置误差
		double world_error_x = (now_x - position_x) * 1;
		double world_error_y = (now_y - position_y) * 1;
		
		// 转换到机体坐标系
		current_goal.velocity.x = world_error_x * cos(current_yaw) + world_error_y * sin(current_yaw);
		current_goal.velocity.y = -world_error_x * sin(current_yaw) + world_error_y * cos(current_yaw);
		current_goal.velocity.z = (hover_height_ - position_z) * 1;
		current_goal.yaw_rate = 0;

		if(is_rc_auto){
			if(!current_state.armed){
				arm();
			}else if(current_state.mode != "OFFBOARD"){
				offboard();
			}
		}
    	reached_hover_height = (fabs(hover_height_ - position_z) < 0.1);
		
	} else{
		reached_hover_height = false;
	}

	//if goal_receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
	if(goal_receive && is_in_air)// Trigger after trajectory tracking
	{
		// 注释掉这行，避免在轨迹跟踪时重置起飞位置
		// get_now_pos = false;
		if(uav_to_goal(targetpos_x, targetpos_y) < (stop_dist + uav_to_goal(ego_pos_x, ego_pos_y)))
		{
			feed_gain = 0;
			reach_goal = true;
		}
		else
		{
			feed_gain = 0.8;
			reach_goal = false;
		}

		if(is_stopped) 
		{
		ego_pos_x = stop_pos_x;
		ego_pos_y = stop_pos_y; 
		ego_pos_z = stop_pos_z;
		feedforward_x = 0;
		feedforward_y = 0;
		feedforward_z = 0;
		}
		
		if(uav_to_goal(targetpos_x, targetpos_y)<0.5){
			if(return_triggered){
				// 当距离小于0.5米时，记录时间（只记录一次）
				if(!near_goal_recorded_){
					near_goal_time_ = ros::Time::now();
					near_goal_recorded_ = true;
				}
				// 等待2秒后再执行赋值
				ros::Time current_time = ros::Time::now();
				if(near_goal_time_.toSec() > 0 && 
				   (current_time - near_goal_time_).toSec() >= 2.0){
					ego_yaw = init_yaw_;
					is_returned = true;
				}
			}
			is_reached = true;
		} else {
			is_returned = false;
			is_reached = false;
			near_goal_recorded_ = false; // 重置标志
			near_goal_time_ = ros::Time(0); // 重置时间
		}

		des_vx = feed_gain * feedforward_x + px * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).x - dx * currvx;
		des_vy = feed_gain * feedforward_y + py * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).y - dy * currvy;
		des_vz = current_goal.velocity.z =  pz * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).z;
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal.header.stamp = ros::Time::now();
		current_goal.type_mask = velocity_mask;
		current_goal.yaw_rate = yaw_rate_gain * plan_yaw_rate();
		//============================x==================//
		if(des_vx > max_vel*max_beishu)
		{
			current_goal.velocity.x = max_vel;
		}
		if(des_vx < -max_vel*max_beishu)
		{
			current_goal.velocity.x = -max_vel;
		}
		if(des_vx > -max_vel*max_beishu && des_vx < max_vel*max_beishu)
		{
			current_goal.velocity.x = des_vx;
		}
		//=================y===================================//
		if(des_vy > max_vel*max_beishu)
		{
			current_goal.velocity.y = max_vel;
		}
		if(des_vy < -max_vel*max_beishu)
		{
			current_goal.velocity.y = -max_vel;
		}
		if(des_vy > -max_vel*max_beishu && des_vy < max_vel*max_beishu)
		{
			current_goal.velocity.y = des_vy;
		}
		//===============================z=======================//
		if(des_vz > max_vel)
		{
			current_goal.velocity.z = max_vel;
		}
		if(des_vz < -max_vel)
		{
			current_goal.velocity.z = -max_vel;
		}
		if(des_vz > -max_vel && des_vz < max_vel)
		{
			current_goal.velocity.z = des_vz;
		}
	}else{
		reach_goal = false;
		is_reached = false;
	}

    if(landing_triggered && is_in_air)
    {
		current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
		current_goal.header.stamp = ros::Time::now();
		current_goal.type_mask = velocity_mask;
		
		// 如果处于返航状态，使用初始点作为降落点
		if(is_returned) {
		    current_goal.velocity.x = (init_x_ - position_x) * 1;
		    current_goal.velocity.y = (init_y_ - position_y) * 1;
		} else {
		    current_goal.velocity.x = (now_x - position_x) * 1;
		    current_goal.velocity.y = (now_y - position_y) * 1;
		}
		
		current_goal.velocity.z = -1.0;
		current_goal.yaw_rate = 0;
    }

    
    // 在main_state函数末尾调用发布状态函数
    publish_drone_state();
    
    local_pos_pub.publish(current_goal);
}

void PX4CTRL::cmd_cb(const controller_msgs::cmd::ConstPtr& msg)
{
    if(msg->cmd == 5)
    {
        is_stopped = true;
        
        // Find position 0.2 seconds ago
        const ros::Time target_time = ros::Time::now() - ros::Duration(0.2);
        auto it = position_history.rbegin();
        
        for(; it != position_history.rend(); ++it) {
            if(std::get<0>(*it) <= target_time) {
                std::tie(std::ignore, stop_pos_x, stop_pos_y, stop_pos_z) = *it;
                break;
            }
        }
        
        // If no old enough data found, use the oldest position
        if(it == position_history.rend() && !position_history.empty()) {
            auto& oldest = position_history.front();
            std::tie(std::ignore, stop_pos_x, stop_pos_y, stop_pos_z) = oldest;
        }
    } 
    else
    {
        is_stopped = false;
    }

	if(msg->cmd == 1)
    {
        takeoff_triggered = true;
    }
    else{
		takeoff_triggered = false;
	}

	if(msg->cmd == 2)
    {
        landing_triggered = true;
    }else{
		landing_triggered = false;
	}

	if(msg->cmd == 3)
    {
        return_triggered = true;
    }else{
		return_triggered = false;
	}
}

void PX4CTRL::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    switch(msg->landed_state)
    {
    case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
        is_landed = true;
        is_in_air = false;
        goal_receive = false;  // 新增着陆时重置目标接收标志
        break;
    case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
        is_landed = false;
        is_in_air = true;
        break;
    default:
        is_landed = true;
        is_in_air = false;
        break;
    }
}

bool PX4CTRL::arm()
{
    if(!is_rc_auto){  // Directly use flag
        return false;
    }
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if(arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
        return true;
    }else{
        ROS_ERROR("Arming failed");
        return false;
    }
}

bool PX4CTRL::disarm()
{
    if(!is_rc_auto){
        return false;
    }
    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    
    if(arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle disarmed");
        return true;
    }else{
        ROS_ERROR("Disarming failed");
        return false;
    }
}

bool PX4CTRL::offboard()
{
    if(!is_rc_auto){
        return false;
    }
    
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    
    if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
        return true;
    }else{
        ROS_ERROR("Failed to set Offboard");
        return false;
    }
}

void PX4CTRL::publish_drone_state()
{
    static ros::Time last_returned_time;
    static bool last_is_returned = false;
    static ros::Time last_goal_update_time = ros::Time::now();
    
    // 检查距离上次发布是否已经过了足够的时间（例如0.1秒，即10Hz）
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_state_pub_time).toSec() < 0.1) {
        return; // 还没到发布时间，直接返回
    }
    
    // 更新上次发布时间
    last_state_pub_time = current_time;
    
    controller_msgs::DroneState state_msg;
    state_msg.header.stamp = current_time;
    
    // 将当前状态赋值给消息字段
    state_msg.landed = is_landed;
    state_msg.tookoff = takeoff_triggered && is_in_air && reached_hover_height;
    
    // 只有当目标点没有最近变化时（例如1秒内）才设置reached为true
    if (is_reached && (current_time - last_goal_update_time).toSec() > 1.0) {
        state_msg.reached = true;
    } else {
        state_msg.reached = false;
    }
    
    // 检查is_returned的持续状态
    if (is_returned && !last_is_returned) {
        // is_returned刚变为true，记录时间
        last_returned_time = current_time;
    } else if (is_returned && last_is_returned) {
        // is_returned持续为true，检查是否已经持续3秒
        ros::Duration duration = current_time - last_returned_time;
        state_msg.returned = return_triggered && (duration.toSec() >= 3.0);
    } else {
        // is_returned为false
        state_msg.returned = false;
    }
    
    // 更新上一次的is_returned状态
    last_is_returned = is_returned;
    
    // 发布消息
    drone_state_pub.publish(state_msg);
}

// 处理 /autoloc/initial_pose topic 的回调函数
void PX4CTRL::initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!use_autoloc_init_ || autoloc_received_) {
        return; // 如果未启用或已接收，直接返回
    }
    
    // 提取位置信息 (init_z 始终使用人工赋值)
    init_x_ = msg->pose.position.x;
    init_y_ = msg->pose.position.y;
    // init_z_ 保持人工设置的值，不从 autoloc 更新
    
    // 提取姿态信息并计算偏航角
    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;
    
    init_yaw_ = atan2(2.0 * (qw * qz + qx * qy), 
                      1.0 - 2.0 * (qy * qy + qz * qz));
    
    // 悬停高度保持人工设置的值，不从 autoloc 更新
    // hover_height_ = init_z_; // 注释掉，保持原有设置
    
    // 标记已接收到 autoloc 数据
    autoloc_received_ = true;
    
    ROS_INFO("Received autoloc initial pose: x=%.3f, y=%.3f (z=%.3f kept manual), yaw=%.3f", 
             init_x_, init_y_, init_z_, init_yaw_ * 180.0 / PI);
    
    // 取消订阅，避免重复更新
    initial_pose_sub.shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "px4ctrl_node");
	setlocale(LC_ALL,""); 
	PX4CTRL px4ctrl;
	ros::spin();
	return 0;
}
