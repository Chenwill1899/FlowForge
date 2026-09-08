#ifndef _FLUID_GUIDANCE_H
#define _FLUID_GUIDANCE_H

// Self-contained 2D/3D fluid guidance algorithm (the GVF-Nav original
// contribution).  Owns ALL cross-tick guidance state and the guidance laws:
//
//   2D  -- harmonic stream-function field (Darcy flow) around inflated
//          obstacles, nested coarse+fine solve, stream-function-error
//          control (chi*), per-component bypass side latch, near-wall
//          escape net, ray-homing regression.
//   3D  -- pressure-projection potential flow (Neumann obstacle walls),
//          nested coarse+fine solve, stagnation crossflow latch, local
//          section-return transverse coordinates and pseudoinverse feedback,
//          altitude homing, vertical-lane preview, escape net. Nearest-segment
//          projection selects the next interval seed and supplies legacy diagnostics.
//
// The numerical kernels live in fluid_solver_2d / fluid_solver_3d (separate
// translation units, gtest-able).  This module is the guidance POLICY that
// sits on top of them: window construction, solve throttling, field caching,
// stateful latch/streamline layers and command composition.
//
// Deliberately ROS-free and map-free:
//   * obstacle distance is injected as a DistanceQuery callback (a live
//     SDFMap, a synthetic field, a fixed grid -- anything returning a
//     clearance scalar at a world point);
//   * time is injected as double seconds (ros::Time has no place in the
//     algorithm and makes unit-testing time-dependent logic impossible);
//   * visualization is emitted as a pure-data snapshot (FluidVis2D/3D) that
//     a ROS adapter renders -- no publishers, no message types here.

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <fluid/fluid_solver_2d.h>
#include <fluid/fluid_solver_3d.h>
#include <fluid/flow_coordinates_3d.h>

namespace FLAG_Race {
namespace fluid {

// Returns the ESDF clearance (m) at a world point.  Callers must define the
// finite/corrupt-value contract; this module treats non-finite reads as 0.
using DistanceQuery = std::function<double(const Eigen::Vector3d&)>;

struct FluidGuidanceConfig
{
    // ---- shared ----
    double cruise_z = 1.0;
    double window_forward_size = 12.0;
    double window_lateral_size = 10.0;
    double window_rear_margin = 0.30;
    double grid_resolution = 0.15;
    double coarse_grid_resolution = 0.30;
    double d_s = 0.3;        // solid threshold
    double d_drag = 0.6;     // full-speed clearance (speed ramp)
    double d_turn = 0.9;     // full-speed clearance (escape gate)
    double d_look = 2.5;     // ray-convergence gate threshold
    double delta_theta_max_rad = 0.6;
    double conv_gate_margin = 1.0;
    double conv_length = 1.5;
    bool use_regression = true;
    double bypass_offset = 1.0;
    double default_bias_sign = 1.0;
    double side_deadband = 0.05;
    double speed_floor = 0.15;
    double escape_tangent_ratio = 0.50;
    double escape_outward_ratio = 0.15;
    double escape_stall_lo_ratio = 0.15;
    double escape_stall_hi_ratio = 0.50;
    double escape_solid_weight = 0.60;
    double resolve_period = 0.1;
    double cg_tolerance = 1e-5;
    int cg_max_iterations = 200;
    double cg_residual_acceptance = 10.0;
    double fine_forward_size = 6.0;
    double fine_lateral_size = 6.0;
    bool use_coarse = true;
    bool use_fine = true;
    double psi_error_gain = 1.0;
    double side_match_centroid_dist = 1.0;
    double side_latch_ttl = 2.0;
    double side_match_lateral_tol = 0.35;
    double side_match_forward_tol = 0.35;

    // ---- 3D ----
    bool solver_3d = false;
    double fine_resolution = 0.20;
    double grid_resolution_z = 0.15;
    double coarse_resolution_z = 0.30;
    double z_min = 0.2;
    double z_max = 2.6;
    double floor_band = 0.30;
    double ceil_band = 0.30;
    double crossflow_ratio = 0.5;
    double stall_speed_ratio = 0.35;
    double stall_w_veto = 0.25;
    double stall_release_ratio = 0.6;
    double stall_clearance = 1.2;
    double stall_probe = 1.2;
    double stall_latch_time = 0.4;
    double stall_release_time = 0.8;
    double alt_gain = 0.8;
    double alt_rate_max = 0.4;
    double alt_gate_lo = 0.9;
    double alt_gate_hi = 1.5;
    double vertical_preview_len = 3.0;
    double vertical_preview_clearance = 0.45;
    double vertical_preview_rate = 0.6;
    double k_n = 1.0;
    double k_n_z_ratio = 0.0;
    double streamline_fwd_len = 4.0;
    double streamline_bwd_len = 1.5;
    double streamline_ds = 0.1;
    double streamline_max_disp = 1.5;
    double streamline_degrade_cooldown = 0.5;
    bool freeze_streamline = false;
    bool homing_in_track = true;
    double d_v_max = 1.0;
    double crossflow_tau = 0.5;
    double crossflow_rate_max = 1.5;
};

// Pure-data 2D visualization snapshot (the fused command field + coarse
// prior + window coverage).  A ROS adapter renders it; this struct never
// touches a message type.
struct FluidVis2D
{
    bool valid = false;
    bool coarse_style = false;          // coarse-only ablation rendering
    // active (fine unless coarse-only) level
    Eigen::Vector2d origin{0.0, 0.0};
    Eigen::Vector2d e_J{1.0, 0.0};
    Eigen::Vector2d n_J{0.0, 1.0};
    double h = 0.15;
    int n_forward = 0, n_lateral = 0;
    Eigen::MatrixXd Ux, Uy;             // fused command field (fuseLiftedGvfFluid per cell)
    std::vector<std::vector<bool>> solid;
    // coarse prior layer
    bool coarse_valid = false;
    Eigen::Vector2d coarse_origin{0.0, 0.0};
    double coarse_h = 0.0;
    int coarse_n_forward = 0, coarse_n_lateral = 0;
    Eigen::MatrixXd coarse_Ux, coarse_Uy;
    std::vector<std::vector<bool>> coarse_solid;
    // window coverage (RViz crop)
    bool coverage_valid = false;
    Eigen::Vector2d center_xy{0.0, 0.0};
    Eigen::Vector2d vis_e_J{1.0, 0.0};
    Eigen::Vector2d vis_n_J{0.0, 1.0};
    double vis_z = 1.0;
};

// Pure-data 3D visualization snapshot: the solved fine + coarse fields and
// the built quiver frame (horizontal fused slice, vertical raw plane, blue
// streamlines).  A ROS adapter renders it.
struct FluidVis3D
{
    bool valid = false;
    // fine solve results (what the controller samples)
    fluid3d::Grid3D grid;
    std::vector<double> D;
    std::vector<uint8_t> solid;
    fluid3d::PotentialField3D field;
    // coarse solve results (context layer)
    bool coarse_valid = false;
    fluid3d::Grid3D grid_coarse;
    std::vector<uint8_t> solid_coarse;
    fluid3d::PotentialField3D field_coarse;
    // inputs the quiver frame was built with
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector2d anchor_xy = Eigen::Vector2d::Zero();
    double heading_rad = 0.0;
    double v_cap = 0.0;
};

// The 2D/3D fluid guidance policy.  One instance per planner; step*() is
// called at the command rate, and internally throttles the expensive solve
// to config.resolve_period (the guidance laws themselves are cheap).
class FluidGuidance
{
public:
    FluidGuidance() = default;

    void setConfig(const FluidGuidanceConfig& cfg);
    void setDistanceQuery(DistanceQuery q) { distance_ = std::move(q); }

    const FluidGuidanceConfig& config() const { return c_; }

    // Fusion laws exposed read-only for the visualization adapter (builds the
    // quiver/streamline field exactly the way the controller samples it).
    Eigen::Vector2d fuse2D(const Eigen::Vector2d& field_uv, bool field_valid,
                           double psi_here, bool psi_valid, double heading_rad,
                           double phi, double v_cap, double D_local) const {
        return fuseLiftedGvfFluid(field_uv, field_valid, psi_here, psi_valid,
                                  heading_rad, phi, v_cap, D_local);
    }
    Eigen::Vector3d fuse3D(const Eigen::Vector3d& field_uvw, bool field_valid,
                           double heading_rad, double phi, double v_cap,
                           double D_local) const {
        return fuseLiftedGvfFluid3D(field_uvw, field_valid, heading_rad, phi,
                                    v_cap, D_local);
    }

    // 2D guidance: plane velocity from the harmonic stream-function field.
    Eigen::Vector2d calcGuidance2D(const Eigen::Vector2d& pos_xy,
                                   const Eigen::Vector2d& anchor_xy,
                                   double heading_rad, double v_J,
                                   double now_sec, bool build_viz);
    // 3D guidance: full velocity from the potential-flow field + streamline
    // tracking.
    Eigen::Vector3d calcGuidance3D(const Eigen::Vector3d& pos,
                                   const Eigen::Vector2d& anchor_xy,
                                   double heading_rad, double v_J,
                                   double now_sec, bool build_viz);

    void reset();

    // ---- diagnostics (read by the ROS telemetry layer) ----
    void getDiagnostics(Eigen::Vector3d& field_direction, double& potential,
                        bool& field_valid, bool& potential_valid) const;
    void setDiagnosticDirection(const Eigen::Vector3d& direction);
    void getStreamlineDiagnostics(const Eigen::Vector3d& pos,
                                  Eigen::Vector3d& projection,
                                  Eigen::Vector3d& e_perp,
                                  double& eta_xy, double& eta_3d, bool& valid,
                                  int& track_mode, bool& end_cap,
                                  uint64_t& version, double& update_disp,
                                  double& streamline_clearance_min,
                                  double& projection_clearance,
                                  double& d_v) const;

    struct FlowErrorDiagnostics3D {
        fluid3d::CoordinateResult3D coordinate;
        fluid3d::FlowControlResult3D control;
        double control_compute_ms = 0.0;
        Eigen::Vector3d anchor = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        Eigen::Vector3d query = anchor;
        uint64_t field_version = 0;
        double gradient_relative_error = std::numeric_limits<double>::quiet_NaN();
        double compute_ms = 0.0;
    };
    const FlowErrorDiagnostics3D& flowErrorDiagnostics3D() const { return flow_error_diag_; }

    // ---- visualization snapshots ----
    const FluidVis2D& vis2D() const { return vis2d_; }
    const FluidVis3D& vis3D() const { return vis3d_; }

    // Streaming state left live for the 3D streamline marker rendering:
    // the last built reference polyline.
    struct Streamline3DView
    {
        std::vector<Eigen::Vector3d> points;
        std::vector<double> arc;
        std::vector<Eigen::Vector3d> tangents;
        bool valid = false;
        bool end_cap = false;
        uint64_t version = 0;
    };
    const Streamline3DView& streamline() const { return streamline_view_; }

private:
    // ---- config ----
    FluidGuidanceConfig c_;
    DistanceQuery distance_;
    double now_sec_ = 0.0;

    // ---- 2D state ----
    bool fluid_field_valid_ = false;
    Eigen::Vector2d fluid_field_origin_{0.0, 0.0};
    Eigen::Vector2d fluid_field_e_J_{1.0, 0.0};
    Eigen::Vector2d fluid_field_n_J_{0.0, 1.0};
    double fluid_field_h_ = 0.15;
    int fluid_field_n_forward_ = 0, fluid_field_n_lateral_ = 0;
    Eigen::MatrixXd fluid_psi_, fluid_Ux_field_, fluid_Uy_field_;
    Eigen::Vector2d fluid_cached_t_field_{1.0, 0.0};
    bool fluid_cached_t_field_valid_ = false;
    double fluid_psi_error_gain_ = 1.0;
    bool fluid_psi_anchor_ref_valid_ = false;
    double fluid_psi_anchor_ref_ = 0.0;
    double fluid_field_solve_speed_ = 0.0;
    Eigen::Vector2d fluid_cached_clearance_grad_{0.0, 0.0};
    double fluid_cached_clearance_ = 0.0;
    bool fluid_cached_clearance_grad_valid_ = false;
    double fluid_last_solve_time_ = 0.0;

    struct FluidSideLatchEntry
    {
        Eigen::Vector2d centroid;
        double side = 0.0;
        double eta_min = 0.0, eta_max = 0.0;
        double s_min = 0.0, s_max = 0.0;
        double stamp = 0.0;
    };
    std::vector<FluidSideLatchEntry> fluid_side_latch_;
    std::vector<double> computeLatchedSides(const fluid2d::SolidComponents& comps);
    void commitSideLatch(const fluid2d::SolidComponents& comps,
                         const std::vector<double>& chosen_sides);

    // ---- 3D state ----
    fluid3d::Grid3D fluid3d_field_grid_;
    fluid3d::FlowCoordinates3D flow_coordinates_;
    std::shared_ptr<const fluid3d::CoordinateGrid3D> flow_coordinate_grid_;
    FlowErrorDiagnostics3D flow_error_diag_;
    uint64_t flow_field_version_ = 0;
    std::vector<double> fluid3d_Ux_, fluid3d_Uy_, fluid3d_Uz_;
    std::vector<double> fluid3d_phi_;
    bool fluid3d_field_valid_ = false;
    Eigen::Vector3d fluid3d_cached_t_field_{1.0, 0.0, 0.0};
    bool fluid3d_cached_t_field_valid_ = false;
    Eigen::Vector3d fluid3d_cached_clearance_grad_{0.0, 0.0, 0.0};
    double fluid3d_cached_clearance_ = 0.0;
    bool fluid3d_cached_clearance_grad_valid_ = false;
    bool fluid3d_cross_latched_ = false;
    double fluid3d_cross_side_ = 1.0;
    double fluid3d_stall_accum_ = 0.0;
    double fluid3d_release_accum_ = 0.0;
    double fluid3d_last_tick_time_ = 0.0;
    double fluid3d_cross_beta_ = 0.0;
    struct CompExtent { double eta_min, eta_max, s_min, z_max; };
    std::vector<CompExtent> fluid3d_coarse_extents_;
    std::vector<double> fluid3d_D_;
    std::vector<uint8_t> fluid3d_solid_;
    bool fluid3d_field_data_valid_ = false;
    double fluid3d_field_solve_speed_ = 0.0;
    int fluid3d_track_mode_ = 2;
    double fluid3d_d_v_observed_ = 0.0;
    double fluid3d_d_v_current_ = std::numeric_limits<double>::quiet_NaN();
    double fluid3d_streamline_update_disp_ = 0.0;
    double fluid3d_streamline_clearance_min_ = std::numeric_limits<double>::quiet_NaN();
    double fluid3d_streamline_degrade_until_ = 0.0;
    uint64_t fluid3d_streamline_version_ = 0;

    Streamline3DView streamline_view_;
    struct StreamlineProj3D
    {
        double s = 0.0;
        Eigen::Vector3d p{0.0, 0.0, 0.0};
        Eigen::Vector3d t{1.0, 0.0, 0.0};
        bool end_cap = false;
    };
    void updateStreamline3D(const Eigen::Vector3d& pos, const fluid3d::Grid3D& g,
                            double v_cap);
    StreamlineProj3D projectToStreamline3D(const Eigen::Vector3d& x) const;
    Eigen::Vector3d calcStreamlineGuidance3D(const Eigen::Vector3d& pos,
                                             double v_cap, double D_local,
                                             double w_robot,
                                             double heading_rad);

    // ---- diagnostics ----
    Eigen::Vector3d fluid_diag_field_direction_{0.0, 0.0, 0.0};
    double fluid_diag_potential_ = 0.0;
    bool fluid_diag_field_valid_ = false;
    bool fluid_diag_potential_valid_ = false;

    // ---- visualization snapshots ----
    FluidVis2D vis2d_;
    FluidVis3D vis3d_;

    // ---- guidance laws ----
    Eigen::Vector2d fuseLiftedGvfFluid(const Eigen::Vector2d& field_uv,
                                       bool field_valid, double psi_here,
                                       bool psi_valid, double heading_rad,
                                       double phi, double v_cap,
                                       double D_local) const;
    Eigen::Vector3d fuseLiftedGvfFluid3D(const Eigen::Vector3d& field_uvw,
                                         bool field_valid, double heading_rad,
                                         double phi, double v_cap,
                                         double D_local) const;
};

}  // namespace fluid
}  // namespace FLAG_Race

#endif  // _FLUID_GUIDANCE_H
