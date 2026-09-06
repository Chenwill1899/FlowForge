#ifndef  _GVF_MANAGER_H
#define  _GVF_MANAGER_H   

//standard
#include <string>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <math.h>
#include <numeric>
#include <memory>
#include <mutex>
#include <vector>
#include <deque>
#include <limits>
#include <Eigen/Dense>
//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <ros/topic_manager.h>

//自定义
#include <fluid/cbf_projection.h>
#include "common_msgs/common_msgs.h"
#include <plan_env/sdf_map.h>
#include "fluid/gvf.h"

using namespace std;

#define PI acos(-1)

namespace FLAG_Race
{

class gvf_manager
{
    public:
        std::string cloud_topic_, odom_topic_, cmd_topic_;
        double current_yaw_ = 0.0;  // 由 odom 估计的机体朝向

        // 差速车车体轮廓碰撞检测：在流体路径中不使用（仅旧的 A*+B 样条
        // replan 的 checkCollision 使用），故不再保留相关配置。

        double cmd_vel_max_ = 1.25;          // m/s，GVF 输出速度限幅（诊断）

        Eigen::Vector3d odom_;

        // 由 odom 位置差分估计的真实速度
        struct OdomPosSample { ros::Time t; Eigen::Vector3d p; };
        std::deque<OdomPosSample> odom_pos_history_;
        Eigen::Vector3d odom_vel_est_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d odom_vel_lpf_ = Eigen::Vector3d::Zero();
        bool odom_vel_initialized_ = false;
        double odom_vel_est_window_ = 0.3;
        double odom_vel_lpf_hz_ = 2.0;
        Eigen::Vector3d last_odom_pos_ = Eigen::Vector3d::Zero();
        ros::Time last_odom_time_;
        bool has_last_odom_ = false;

        struct gvfManager {
            std::string index;
            std::shared_ptr<SDFMap> sdf_map_;
            std::shared_ptr<gvf>  gvf_;
            ros::Time curr_time;  // 当前时间定时器
            ros::Time last_time;  // 上一次时间定时器
            bool is_initialized = false;
            Eigen::Vector3d odom;
        };
        std::vector<gvfManager> swarmParticlesManager;

    public:
        //ROS
        ros::Publisher  cmd_pub;
        ros::Timer      cmd_timer;
        ros::Subscriber human_intent_sub_;
        ros::Subscriber odom_sub;
        ros::Publisher  field_diagnostics_pub_;
        ros::Publisher  reanchor_event_pub_;
        int reanchor_count_ = 0;

        // ==================== 人机意图 / 流体引导输入 ====================
        bool human_input_enable_ = false;
        std::string human_intent_topic_ = "/human_intent";
        double human_intent_timeout_ = 0.5;
        double human_intent_max_speed_ = 1.0;
        Eigen::Vector3d human_intent_velocity_ = Eigen::Vector3d::Zero();
        ros::Time human_intent_last_time_;
        bool human_intent_received_ = false;
        // 专用于慢速流体路径的 odometry 镜像（与 odom_ 分离，避免与其他
        // 回调竞争）。
        Eigen::Vector3d human_fluid_odom_ = Eigen::Vector3d::Zero();
        double human_fluid_yaw_ = 0.0;
        bool human_fluid_have_odom_ = false;

        // --- Darcy-fluid guidance anchor：仅在激活或航向显著变化时 latch ---
        bool fluid_anchor_active_ = false;
        Eigen::Vector2d fluid_anchor_pos_ = Eigen::Vector2d::Zero();
        double fluid_anchor_heading_ = 0.0;
        double fluid_heading_latch_deg_ = 15.0;
        double fluid_intent_speed_deadband_ = 0.05;
        // 安全监督移除最小余量处完全向内流体指令后的切线逃逸。
        double fluid_safety_escape_speed_ratio_ = 0.25;
        bool fluid_safety_escape_side_latched_ = false;
        double fluid_safety_escape_tangent_sign_ = 1.0;

        // 3D 势流引导开关（gvf/fluid_solver_3d）。
        bool fluid_solver_3d_ = false;
        double fluid_3d_w_max_ = 1.2;   // |velocity.z| 指令上限
        // 3-D 人机流体指令 slew 限幅。
        double fluid_3d_cmd_accel_xy_max_ = 4.0;  // m/s^2
        double fluid_3d_cmd_accel_z_max_ = 2.5;   // m/s^2
        Eigen::Vector3d fluid_3d_cmd_filtered_ = Eigen::Vector3d::Zero();
        bool fluid_3d_cmd_filter_initialized_ = false;
        ros::Time fluid_3d_cmd_filter_last_time_;

        // ==================== 安全监督 + CBF ====================
        double safety_min_clearance_ = 0.30;
        double safety_brake_decel_ = 1.5;
        bool   safety_braking_active_ = false;
        double safety_last_clearance_ = -1.0;

        double cbf_gain_ = 1.5;
        int cbf_projection_count_ = 0;
        double safety_esdf_max_age_ = 0.5;
        bool   safety_esdf_h_valid_ = false;
        double safety_esdf_h_ = 0.0;
        Eigen::Vector3d safety_esdf_grad_ = Eigen::Vector3d::Zero();
        ros::Time safety_esdf_h_stamp_;

        bool fetchSafetyDistance(const Eigen::Vector3d& pos,
                                 double& h, Eigen::Vector3d& grad);
        void applyCbfFinal(const Eigen::Vector3d& pos, Eigen::Vector3d& v_cmd,
                           double horizontal_cap);
        void resetFluid3DCommandFilter();
        void limitFluid3DCommandRate(const ros::Time& now,
                                      Eigen::Vector3d& v_cmd,
                                      bool command_valid);
        void checkSafetySupervisor(const Eigen::Vector3d& pos,
                                   const Eigen::Vector3d& intent_velocity,
                                   Eigen::Vector3d& v_cmd);

    private:
        // formation_planning 使用 AsyncSpinner。50 Hz 定时器可能在 Darcy
        // 求解尚未结束时派发下一个流体指令 tick；该互斥串行化完整流体
        // 指令/状态路径。输入使用独立的短持有互斥，慢求解不会占用
        // spinner 线程派发 odom/摇杆释放。
        std::mutex human_fluid_cmd_mutex_;
        std::mutex human_fluid_input_mutex_;

    public:
        gvf_manager(){};  
        gvf_manager(ros::NodeHandle& nh); 
        ~gvf_manager();
        void initCallback(ros::NodeHandle &nh);
        void InitGvf(ros::NodeHandle &nh);
        void humanIntentCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void cmdCallback(const ros::TimerEvent& event);
        void updateFluidAnchor(const ros::Time& now,
                               const Eigen::Vector2d& pos_xy,
                               const Eigen::Vector3d& intent_velocity,
                               const ros::Time& intent_last_time,
                               bool intent_received);
        void publishFieldDiagnostics(const ros::Time& stamp,
                                     const Eigen::Vector3d& pos,
                                     const Eigen::Vector3d& intent_velocity);
};

}  // namespace FLAG_Race

#endif
