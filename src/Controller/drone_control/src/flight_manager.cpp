#include <ros/ros.h>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <controller_msgs/cmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

class FlightManager {
public:
    FlightManager() : nh_("~"), autoloc_received_(false) {
        
        nh_.param<int>("drone_id", drone_id_, 0);
        nh_.param<double>("init_x", init_x_, 0.0);
        nh_.param<double>("init_y", init_y_, 0.0);
        nh_.param<double>("init_z", init_z_, 0.0);
        nh_.param<bool>("use_autoloc_init", use_autoloc_init_, false);

        
        cmd_sub_ = nh_.subscribe("/control", 10, &FlightManager::cmdCallback, this);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &FlightManager::odomCallback, this);
        
        
        goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        
        // 初始化 autoloc 相关变量
        autoloc_timeout_ = ros::Time::now() + ros::Duration(5.0); // 5秒超时
        
        // 如果启用 autoloc 初始化，订阅 topic
        if (use_autoloc_init_) {
            initial_pose_sub_ = nh_.subscribe("/autoloc/initial_pose", 1, &FlightManager::initialPoseCallback, this);
            ROS_INFO("FlightManager: Waiting for /autoloc/initial_pose topic for initialization...");
        } else {
            ROS_INFO("FlightManager: Using manual init parameters: x=%.2f, y=%.2f, z=%.2f", init_x_, init_y_, init_z_);
        }
    }

private:
    void cmdCallback(const controller_msgs::cmd::ConstPtr& msg) {
        // 检查 autoloc 超时
        checkAutolocTimeout();
        
        int new_cmd_ = msg->cmd;
        if(new_cmd_ != current_cmd_) {
            previous_cmd_ = current_cmd_;
            current_cmd_ = new_cmd_;
            handleCommand();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
    }
    
    void initialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!use_autoloc_init_ || autoloc_received_) {
            return; // 如果未启用或已接收，直接返回
        }
        
        // 提取位置信息 (init_z 始终使用人工赋值)
        init_x_ = msg->pose.position.x;
        init_y_ = msg->pose.position.y;
        // init_z_ 保持人工设置的值，不从 autoloc 更新
        
        // 标记已接收到 autoloc 数据
        autoloc_received_ = true;
        
        ROS_INFO("FlightManager: Received autoloc initial pose: x=%.3f, y=%.3f (z=%.3f kept manual)", 
                 init_x_, init_y_, init_z_);
        
        // 取消订阅，避免重复更新
        initial_pose_sub_.shutdown();
    }
    
    void checkAutolocTimeout() {
        // 检查 autoloc 超时，如果启用但超时未收到数据，则回退到手动参数
        if (use_autoloc_init_ && !autoloc_received_ && ros::Time::now() > autoloc_timeout_) {
            ROS_WARN("FlightManager: Autoloc initialization timeout! Falling back to manual parameters.");
            use_autoloc_init_ = false; // 禁用 autoloc，避免重复警告
            initial_pose_sub_.shutdown(); // 停止订阅
        }
    }

    void handleCommand() {
        quadrotor_msgs::GoalSet goal_msg;
        goal_msg.drone_id = drone_id_;

        switch(current_cmd_) {
            case 3:  // Return
                goal_msg.goal = {
                    static_cast<float>(init_x_),
                    static_cast<float>(init_y_),
                    static_cast<float>(init_z_)
                };
                goal_pub_.publish(goal_msg);
                ROS_INFO("Return to initial position: (%.2f, %.2f, %.2f)", 
                        init_x_, init_y_, init_z_);
                break;
                
            case 4:  // Continue
                if(previous_cmd_ == 3) {
                    goal_msg.goal = {
                        static_cast<float>(current_pose_.position.x),
                        static_cast<float>(current_pose_.position.y),
                        static_cast<float>(current_pose_.position.z)
                    };
                    goal_pub_.publish(goal_msg);
                    ROS_INFO("Continue to current position: (%.2f, %.2f, %.2f)",
                            goal_msg.goal[0], goal_msg.goal[1], goal_msg.goal[2]);
                }
                break;

            default:
                break;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Publisher goal_pub_;
    
    int drone_id_;
    double init_x_, init_y_, init_z_;
    geometry_msgs::Pose current_pose_;
    int current_cmd_ = 0;
    int previous_cmd_ = 0;
    
    bool use_autoloc_init_; // 是否使用 autoloc 初始化
    bool autoloc_received_; // 是否收到 autoloc 数据
    ros::Time autoloc_timeout_; // autoloc 超时时间
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "flight_manager");
    FlightManager manager;
    ros::spin();
    return 0;
}
