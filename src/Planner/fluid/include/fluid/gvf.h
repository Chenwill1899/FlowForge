#ifndef  _GVF_H
#define  _GVF_H

// gvf is the ROS adapter for the GVF-Nav fluid guidance algorithm.  All
// algorithm state and guidance laws live in fluid/fluid_guidance.h
// (fluid::FluidGuidance, a ROS-free module); this class owns the ROS wiring:
//
//   * init() reads the gvf/fluid_* params and builds the algorithm config;
//   * setSdfMap() wraps a plan_env SDFMap's getDistance into the module's
//     DistanceQuery callback;
//   * calcFluidGuidance2D/3D delegate to the module and then render its
//     visualization snapshot (FluidVis2D/3D) as RViz markers.
//
// Everything legacy (path reparametrization, lifted-GVF path tracking, the
// local occupancy/ESDF buffer, A*+B-spline coupling) has been removed.

#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// ros
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <plan_env/sdf_map.h>
#include <fluid/fluid_guidance.h>
#include <fluid/fluid_solver_3d.h>

using namespace std;

namespace FLAG_Race
{
class gvf
{
    public:
        ros::Publisher vector_field_pub_;
        ros::Timer quiver_anim_timer_;
        bool fluid3d_enabled_ = false;         // gvf/fluid_solver_3d, mirrors module config
        bool fluid_human_input_mode_ = false;  // gvf/human_input_enable
        std::string frame_id_ = "world";

        fluid::FluidGuidance fluid_;

        // The fluid algorithm needs no map buffer of its own: it reads the
        // shared SDFMap via the injected DistanceQuery.
        void setSdfMap(const std::shared_ptr<SDFMap>& m);

        // ===== 2D/3D fluid guidance entry points (delegate to fluid_) =====
        Eigen::Vector2d calcFluidGuidance2D(const Eigen::Vector2d& pos_xy,
                                            const Eigen::Vector2d& anchor_xy,
                                            double heading_rad,
                                            double v_J);
        Eigen::Vector3d calcFluidGuidance3D(const Eigen::Vector3d& pos,
                                            const Eigen::Vector2d& anchor_xy,
                                            double heading_rad,
                                            double v_J);
        void resetFluidEscapeState() { fluid_.reset(); }

        // Diagnostics passthrough to the algorithm module.
        void getFluidDiagnostics(Eigen::Vector3d& field_direction,
                                 double& potential,
                                 bool& field_valid,
                                 bool& potential_valid) const {
            fluid_.getDiagnostics(field_direction, potential,
                                  field_valid, potential_valid);
        }
        void setFluidDiagnosticDirection(const Eigen::Vector3d& direction) {
            fluid_.setDiagnosticDirection(direction);
        }
        void getStreamlineDiagnostics(const Eigen::Vector3d& pos,
                                      Eigen::Vector3d& projection,
                                      Eigen::Vector3d& e_perp,
                                      double& eta_xy,
                                      double& eta_3d,
                                      bool& valid,
                                      int& track_mode,
                                      bool& end_cap,
                                      uint64_t& version,
                                      double& update_disp,
                                      double& streamline_clearance_min,
                                      double& projection_clearance,
                                      double& d_v) const {
            fluid_.getStreamlineDiagnostics(
                pos, projection, e_perp, eta_xy, eta_3d, valid, track_mode,
                end_cap, version, update_disp, streamline_clearance_min,
                projection_clearance, d_v);
        }

        // ===== Fluid visualization (renders the module's snapshot) =====
        void publishFluidQuiver(const Eigen::Vector2d& origin,
                                const Eigen::Vector2d& e_J,
                                const Eigen::Vector2d& n_J,
                                double h, int n_forward, int n_lateral,
                                const Eigen::MatrixXd& Ux, const Eigen::MatrixXd& Uy,
                                const std::vector<std::vector<bool>>& solid,
                                const Eigen::Vector2d& display_bias,
                                const std::string& ns = "fluid_quiver",
                                bool prior_style = false,
                                const Eigen::Vector2d& mask_origin = Eigen::Vector2d::Zero(),
                                double mask_forward_len = -1.0,
                                double mask_lateral_len = -1.0,
                                double draw_z = -1.0);
        void publishFluidStreamlines2D(const Eigen::Vector2d& origin,
                                       const Eigen::Vector2d& e_J,
                                       const Eigen::Vector2d& n_J,
                                       double h, int n_forward, int n_lateral,
                                       const Eigen::MatrixXd& Ux,
                                       const Eigen::MatrixXd& Uy,
                                       const std::vector<std::vector<bool>>& solid,
                                       const std::string& ns,
                                       bool coarse_style = false,
                                       double draw_z = -1.0,
                                       const Eigen::Vector2d& mask_origin = Eigen::Vector2d::Zero(),
                                       double mask_forward_len = -1.0,
                                       double mask_lateral_len = -1.0);
        // 3D-mode animation frame (built from the module's FluidVis3D snapshot).
        struct QuiverFrame3D {
            bool valid = false;
            fluid3d::Grid3D g;
            int slice_m = 0;
            double draw_z = 1.0;
            Eigen::MatrixXd Ux_h, Uy_h;               // fused horizontal slice
            std::vector<std::vector<bool>> solid_h;
            int col_j = 0;
            Eigen::MatrixXd Us_v, Uz_v;               // (i, m) vertical plane, raw field
            std::vector<std::vector<bool>> solid_v;
            std::vector<std::vector<Eigen::Vector3d>> streamlines;
            bool coarse_valid = false;
            fluid3d::Grid3D g_coarse;
            Eigen::MatrixXd Ux_hc, Uy_hc;
            std::vector<std::vector<bool>> solid_hc;
        };
        QuiverFrame3D quiver3d_frame_;
        void buildQuiverFrame3D(const fluid3d::Grid3D& g,
                                const std::vector<double>& D,
                                const std::vector<uint8_t>& solid,
                                const fluid3d::PotentialField3D& field,
                                const fluid3d::Grid3D& g_coarse,
                                const std::vector<uint8_t>& solid_coarse,
                                const fluid3d::PotentialField3D& field_coarse,
                                const Eigen::Vector3d& pos,
                                const Eigen::Vector2d& anchor_xy,
                                double heading_rad, double v_cap);
        void renderQuiverFrame3D();
        void quiverAnimCallback(const ros::TimerEvent& /*event*/);
        // Renders the module's 2D snapshot (fine fused field + coarse context
        // + streamlines) into RViz markers.
        void renderFluidVis2D();

    public:
        gvf() = default;
        ~gvf() = default;
        void init(ros::NodeHandle& nh, const std::string& particle,
                  const std::string& odom, const std::string& cloud);
};

}  // namespace FLAG_Race

#endif
