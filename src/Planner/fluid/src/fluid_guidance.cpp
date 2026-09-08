#include "fluid/fluid_guidance.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <queue>

namespace FLAG_Race {
namespace fluid {

using namespace fluid2d;

namespace {

// One solved 2D grid level (the FluidLevel counterpart of fluid_solver_2d.h,
// extended with the timing breakdown the live path logs).
struct FluidLevel2D
{
    Eigen::Vector2d origin{0.0, 0.0};
    double h = 0.0, rear_margin = 0.0;
    int n_forward = 0, n_lateral = 0;
    Eigen::MatrixXd D;
    std::vector<std::vector<bool>> solid;
    fluid2d::SolidComponents comps;
    fluid2d::FluidHarmonicField field;
    double ms_esdf = 0.0, ms_mask = 0.0, ms_label = 0.0, ms_field = 0.0;
    double ms_total() const { return ms_esdf + ms_mask + ms_label + ms_field; }
};

// One solved 3D grid level.
struct FluidLevel3D
{
    fluid3d::Grid3D g;
    double rear_margin = 0.0;
    int m_lo = 0, m_hi = -1;
    std::vector<double> D;
    std::vector<uint8_t> solid;
    fluid3d::SolidComponents3D comps;
    fluid3d::PotentialField3D field;
    double ms_esdf = 0.0, ms_mask = 0.0, ms_label = 0.0, ms_field = 0.0;
    double ms_total() const { return ms_esdf + ms_mask + ms_label + ms_field; }
};

double wallMs(const std::chrono::steady_clock::time_point& a,
              const std::chrono::steady_clock::time_point& b)
{
    return 1e3 * std::chrono::duration<double>(b - a).count();
}

}  // namespace

void FluidGuidance::setConfig(const FluidGuidanceConfig& cfg)
{
    c_ = cfg;
    // Sanity clamps mirroring gvf::init.
    c_.grid_resolution = std::max(1e-3, c_.grid_resolution);
    c_.window_forward_size = std::max(3.0 * c_.grid_resolution, c_.window_forward_size);
    c_.window_lateral_size = std::max(3.0 * c_.grid_resolution, c_.window_lateral_size);
    c_.window_rear_margin = std::max(1.5 * c_.grid_resolution, c_.window_rear_margin);
    c_.window_rear_margin = std::min(
        c_.window_forward_size - 1.5 * c_.grid_resolution, c_.window_rear_margin);
    c_.delta_theta_max_rad = std::max(0.0, c_.delta_theta_max_rad);
    c_.conv_gate_margin = std::max(1e-3, c_.conv_gate_margin);
    c_.conv_length = std::max(1e-3, c_.conv_length);
    c_.bypass_offset = std::max(0.0, c_.bypass_offset);
    c_.default_bias_sign = c_.default_bias_sign < 0.0 ? -1.0 : 1.0;
    c_.side_deadband = std::max(0.0, c_.side_deadband);
    c_.speed_floor = std::max(0.0, std::min(1.0, c_.speed_floor));
    c_.escape_tangent_ratio = std::max(0.0, std::min(1.0, c_.escape_tangent_ratio));
    c_.escape_outward_ratio = std::max(0.0, std::min(1.0, c_.escape_outward_ratio));
    c_.escape_solid_weight = std::max(0.0, std::min(1.0, c_.escape_solid_weight));
    if (!c_.use_coarse && !c_.use_fine) c_.use_fine = true;

    c_.fine_resolution = std::max(1e-3, c_.fine_resolution);
    c_.grid_resolution_z = std::max(1e-3, c_.grid_resolution_z);
    c_.coarse_resolution_z = std::max(1e-3, c_.coarse_resolution_z);
    if (c_.grid_resolution_z > 2.0 * c_.fine_resolution)
        c_.grid_resolution_z = 2.0 * c_.fine_resolution;
    if (c_.coarse_resolution_z > 2.0 * c_.coarse_grid_resolution)
        c_.coarse_resolution_z = 2.0 * c_.coarse_grid_resolution;
    c_.floor_band = std::max(0.0, c_.floor_band);
    c_.ceil_band = std::max(0.0, c_.ceil_band);
    c_.z_max = std::max(c_.z_min + c_.floor_band + c_.ceil_band +
                            3.0 * c_.grid_resolution_z, c_.z_max);
    c_.crossflow_ratio = std::max(0.0, std::min(2.0, c_.crossflow_ratio));
    c_.stall_speed_ratio = std::max(0.0, std::min(1.0, c_.stall_speed_ratio));
    c_.stall_w_veto = std::max(0.0, c_.stall_w_veto);
    c_.stall_release_ratio = std::max(
        c_.stall_speed_ratio + 0.05, std::min(1.0, c_.stall_release_ratio));
    c_.stall_clearance = std::max(0.0, c_.stall_clearance);
    c_.stall_probe = std::max(0.1, c_.stall_probe);
    c_.stall_latch_time = std::max(0.0, c_.stall_latch_time);
    c_.stall_release_time = std::max(0.0, c_.stall_release_time);
    c_.alt_gain = std::max(0.0, c_.alt_gain);
    c_.alt_rate_max = std::max(0.0, c_.alt_rate_max);
    c_.alt_gate_lo = std::max(0.0, c_.alt_gate_lo);
    c_.alt_gate_hi = std::max(c_.alt_gate_lo + 1e-3, c_.alt_gate_hi);
    c_.vertical_preview_len = std::max(0.0, c_.vertical_preview_len);
    c_.vertical_preview_clearance = std::max(0.0, c_.vertical_preview_clearance);
    c_.vertical_preview_rate = std::max(0.0, c_.vertical_preview_rate);
    c_.psi_error_gain = std::max(0.0, c_.psi_error_gain);
    c_.cg_residual_acceptance = std::max(1.0, c_.cg_residual_acceptance);
    c_.side_match_centroid_dist = std::max(1e-3, c_.side_match_centroid_dist);
    c_.side_latch_ttl = std::max(0.1, c_.side_latch_ttl);
    c_.side_match_lateral_tol = std::max(0.0, std::min(1.0, c_.side_match_lateral_tol));
    c_.side_match_forward_tol = std::max(0.0, std::min(1.0, c_.side_match_forward_tol));
    c_.k_n = std::max(0.0, c_.k_n);
    c_.k_n_z_ratio = std::max(0.0, std::min(1.0, c_.k_n_z_ratio));
    c_.streamline_fwd_len = std::max(0.5, c_.streamline_fwd_len);
    c_.streamline_bwd_len = std::max(0.0, c_.streamline_bwd_len);
    c_.streamline_ds = std::max(1e-3, c_.streamline_ds);
    c_.streamline_max_disp = std::max(1e-3, c_.streamline_max_disp);
    c_.streamline_degrade_cooldown = std::max(0.0, c_.streamline_degrade_cooldown);
    c_.d_v_max = std::max(0.0, c_.d_v_max);
    c_.crossflow_tau = std::max(1e-3, c_.crossflow_tau);
    c_.crossflow_rate_max = std::max(0.0, c_.crossflow_rate_max);
}

void FluidGuidance::reset()
{
    fluid_field_valid_ = false;
    fluid_cached_t_field_valid_ = false;
    fluid_cached_clearance_grad_.setZero();
    fluid_cached_clearance_ = 0.0;
    fluid_cached_clearance_grad_valid_ = false;
    fluid_last_solve_time_ = 0.0;
    fluid3d_field_valid_ = false;
    fluid3d_phi_.clear();
    fluid3d_cached_t_field_valid_ = false;
    fluid3d_cached_clearance_grad_.setZero();
    fluid3d_cached_clearance_ = 0.0;
    fluid3d_cached_clearance_grad_valid_ = false;
    flow_coordinates_ = fluid3d::FlowCoordinates3D{};
    flow_coordinate_grid_.reset();
    flow_error_diag_ = FlowErrorDiagnostics3D{};
    flow_field_version_ = 0;
    fluid3d_cross_latched_ = false;
    fluid3d_cross_beta_ = 0.0;
    fluid3d_stall_accum_ = 0.0;
    fluid3d_release_accum_ = 0.0;
    fluid3d_last_tick_time_ = 0.0;
    fluid3d_coarse_extents_.clear();
    fluid_psi_anchor_ref_valid_ = false;
    streamline_view_.valid = false;
    fluid3d_field_data_valid_ = false;
    fluid3d_track_mode_ = 2;
    fluid3d_streamline_version_ = 0;
    fluid3d_streamline_update_disp_ = 0.0;
    fluid3d_streamline_clearance_min_ = std::numeric_limits<double>::quiet_NaN();
    fluid3d_d_v_current_ = std::numeric_limits<double>::quiet_NaN();
    fluid_diag_field_direction_.setZero();
    fluid_diag_potential_ = 0.0;
    fluid_diag_field_valid_ = false;
    fluid_diag_potential_valid_ = false;
    vis2d_ = FluidVis2D{};
    vis3d_ = FluidVis3D{};
}

void FluidGuidance::getDiagnostics(Eigen::Vector3d& field_direction,
                                   double& potential, bool& field_valid,
                                   bool& potential_valid) const
{
    field_direction = fluid_diag_field_direction_;
    potential = fluid_diag_potential_;
    field_valid = fluid_diag_field_valid_;
    potential_valid = fluid_diag_potential_valid_;
}

void FluidGuidance::setDiagnosticDirection(const Eigen::Vector3d& direction)
{
    if (direction.allFinite() && direction.squaredNorm() > 1e-12) {
        fluid_diag_field_direction_ = direction.normalized();
        fluid_diag_field_valid_ = true;
    }
}

void FluidGuidance::getStreamlineDiagnostics(
    const Eigen::Vector3d& pos, Eigen::Vector3d& projection,
    Eigen::Vector3d& e_perp, double& eta_xy, double& eta_3d, bool& valid,
    int& track_mode, bool& end_cap, uint64_t& version, double& update_disp,
    double& streamline_clearance_min, double& projection_clearance,
    double& d_v) const
{
    valid = streamline_view_.valid && streamline_view_.points.size() >= 2;
    track_mode = fluid3d_track_mode_;
    end_cap = valid && streamline_view_.end_cap;
    version = fluid3d_streamline_version_;
    update_disp = fluid3d_streamline_update_disp_;
    streamline_clearance_min = fluid3d_streamline_clearance_min_;
    projection.setConstant(std::numeric_limits<double>::quiet_NaN());
    e_perp.setConstant(std::numeric_limits<double>::quiet_NaN());
    eta_xy = eta_3d = std::numeric_limits<double>::quiet_NaN();
    projection_clearance = std::numeric_limits<double>::quiet_NaN();
    d_v = fluid3d_d_v_current_;
    if (valid) {
        const StreamlineProj3D proj = projectToStreamline3D(pos);
        projection = proj.p;
        e_perp = pos - proj.p;
        eta_xy = e_perp.head<2>().norm();
        eta_3d = e_perp.norm();
        end_cap = proj.end_cap;
        if (distance_) projection_clearance = distance_(proj.p);
    }
}

// ------------------------------------------------------------------ 2D law

Eigen::Vector2d FluidGuidance::fuseLiftedGvfFluid(const Eigen::Vector2d& field_uv,
                                                  bool field_valid,
                                                  double psi_here,
                                                  bool psi_valid,
                                                  double heading_rad,
                                                  double phi, double v_cap,
                                                  double D_local) const
{
    // Primary law (paper eq. 29, normalized): stream-function-error control
    // on the cached harmonic field.  chi = psi / v_s, chi* anchored, k_psi in
    // s^-1; the correction is exactly normal to the flow, so the tangential
    // speed law is untouched and d/dt e_chi = -k_psi*e_chi (exact decay).
    const double v_t = v_cap * std::max(c_.speed_floor,
        fluid2d::fluidSpeedRatio(D_local, c_.d_s, c_.d_drag));
    auto applyRayHoming = [&](const Eigen::Vector2d& v) -> Eigen::Vector2d {
        const double speed = v.norm();
        if (!c_.use_regression || speed <= 1e-9) return v;
        const double theta_ray_target =
            heading_rad - std::atan(phi / c_.conv_length);
        const double theta_field = std::atan2(v.y(), v.x());
        double delta = theta_ray_target - theta_field;
        while (delta > M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        const double clear_gate = 1.0 - fluid2d::fluidSmoothstepW(
            D_local, c_.d_look, c_.d_look + c_.conv_gate_margin);
        delta = std::max(-c_.delta_theta_max_rad,
                         std::min(c_.delta_theta_max_rad, delta)) * clear_gate;
        const double theta_cmd = theta_field + delta;
        return speed * Eigen::Vector2d(std::cos(theta_cmd), std::sin(theta_cmd));
    };
    if (c_.use_regression && field_valid && psi_valid && fluid_psi_anchor_ref_valid_ &&
        fluid_field_solve_speed_ > 1e-9) {
        const fluid2d::StreamFunctionLawResult r = fluid2d::streamFunctionErrorLaw(
            field_uv, fluid_field_solve_speed_, psi_here / fluid_field_solve_speed_,
            fluid_psi_anchor_ref_, v_t, c_.psi_error_gain);
        if (!r.degraded) return applyRayHoming(r.v);
    }

    // Degenerate branch (no field / psi invalid / chi* not anchored): legacy
    // angle-space law, degraded-mode fallback only.
    double theta_field;
    if (field_valid && field_uv.squaredNorm() > 1e-12) {
        theta_field = std::atan2(field_uv.y(), field_uv.x());
    } else {
        theta_field = heading_rad;
    }
    return applyRayHoming(v_t * Eigen::Vector2d(
        std::cos(theta_field), std::sin(theta_field)));
}

Eigen::Vector2d FluidGuidance::calcGuidance2D(const Eigen::Vector2d& pos_xy,
                                              const Eigen::Vector2d& anchor_xy,
                                              double heading_rad, double v_J,
                                              double now_sec, bool build_viz)
{
    now_sec_ = now_sec;
    if (!distance_) return Eigen::Vector2d::Zero();

    const double v_cap = std::max(0.0, v_J);
    const Eigen::Vector2d e_J(std::cos(heading_rad), std::sin(heading_rad));
    const Eigen::Vector2d n_J(-std::sin(heading_rad), std::cos(heading_rad));
    vis2d_.coverage_valid = true;
    vis2d_.center_xy = pos_xy;
    vis2d_.vis_e_J = e_J;
    vis2d_.vis_n_J = n_J;
    vis2d_.vis_z = c_.cruise_z;

    double D_here = distance_(Eigen::Vector3d(pos_xy.x(), pos_xy.y(), c_.cruise_z));
    // Defensive: a corrupted ESDF read must never read as "clear space".
    // Fall back to the last known-good sampled clearance instead of either
    // extreme (huge bogus value == "go fast"; forced zero == permanent stall).
    if (!std::isfinite(D_here) || std::abs(D_here) > 1e6) {
        D_here = fluid_cached_clearance_grad_valid_ ? fluid_cached_clearance_ : 0.0;
    }

    const bool due = (now_sec - fluid_last_solve_time_) >= c_.resolve_period;

    if (due && v_cap > 1e-9) {
        const auto solve_started = std::chrono::steady_clock::now();
        auto build_level = [&](double forward_size, double lateral_size, double hh,
                               bool centered, double rear_margin_override,
                               const FluidCoarsePrior* prior,
                               const std::function<std::vector<double>(
                                   const fluid2d::SolidComponents&)>& side_fn) {
            FluidLevel2D lv;
            lv.h = hh;
            lv.n_forward = std::max(
                3, static_cast<int>(std::round(forward_size / hh)));
            lv.n_lateral = std::max(
                3, static_cast<int>(std::round(lateral_size / hh)));
            lv.rear_margin = rear_margin_override >= 0.0
                ? std::max(1.5 * hh, std::min((lv.n_forward - 1.5) * hh, rear_margin_override))
                : (centered
                    ? 0.5 * lv.n_forward * hh
                    : std::max(1.5 * hh, std::min((lv.n_forward - 1.5) * hh, c_.window_rear_margin)));
            lv.origin = pos_xy - lv.rear_margin * e_J - 0.5 * lv.n_lateral * hh * n_J;

            const auto t0 = std::chrono::steady_clock::now();
            lv.D.resize(lv.n_forward, lv.n_lateral);
            for (int i = 0; i < lv.n_forward; ++i) {
                for (int j = 0; j < lv.n_lateral; ++j) {
                    const Eigen::Vector2d world =
                        fluid2d::fluidCellWorld(lv.origin, e_J, n_J, hh, i, j);
                    double sample = distance_(
                        Eigen::Vector3d(world.x(), world.y(), c_.cruise_z));
                    if (!std::isfinite(sample) || std::abs(sample) > 1e6) sample = 0.0;
                    lv.D(i, j) = sample;
                }
            }

            const auto t1 = std::chrono::steady_clock::now();
            lv.solid.assign(lv.n_forward, std::vector<bool>(lv.n_lateral, false));
            for (int i = 0; i < lv.n_forward; ++i) {
                for (int j = 0; j < lv.n_lateral; ++j) {
                    lv.solid[i][j] = lv.D(i, j) <= c_.d_s;
                }
            }
            // Fold unreachable free pockets into solid so every surviving free
            // cell reaches the Dirichlet ring (well-posed/SPD solve).
            std::vector<std::vector<bool>> reachable(
                lv.n_forward, std::vector<bool>(lv.n_lateral, false));
            std::queue<std::pair<int, int>> frontier;
            auto push_if_free = [&](int i, int j) {
                if (!lv.solid[i][j] && !reachable[i][j]) {
                    reachable[i][j] = true;
                    frontier.emplace(i, j);
                }
            };
            for (int i = 0; i < lv.n_forward; ++i) {
                push_if_free(i, 0);
                push_if_free(i, lv.n_lateral - 1);
            }
            for (int j = 1; j + 1 < lv.n_lateral; ++j) {
                push_if_free(0, j);
                push_if_free(lv.n_forward - 1, j);
            }
            constexpr int di[4] = {1, -1, 0, 0};
            constexpr int dj[4] = {0, 0, 1, -1};
            while (!frontier.empty()) {
                const auto cell = frontier.front();
                frontier.pop();
                for (int d = 0; d < 4; ++d) {
                    const int ni = cell.first + di[d];
                    const int nj = cell.second + dj[d];
                    if (ni >= 0 && ni < lv.n_forward && nj >= 0 && nj < lv.n_lateral) {
                        push_if_free(ni, nj);
                    }
                }
            }
            for (int i = 0; i < lv.n_forward; ++i) {
                for (int j = 0; j < lv.n_lateral; ++j) {
                    if (!lv.solid[i][j] && !reachable[i][j]) lv.solid[i][j] = true;
                }
            }

            const auto t2 = std::chrono::steady_clock::now();
            lv.comps = labelSolidComponents(lv.solid, lv.origin, e_J, n_J, hh,
                                            anchor_xy, lv.n_forward, lv.n_lateral);
            const auto t3 = std::chrono::steady_clock::now();
            lv.field = solveHarmonicStreamField(
                lv.solid, lv.comps, lv.origin, e_J, n_J, hh,
                lv.n_forward, lv.n_lateral, anchor_xy, v_cap, side_fn(lv.comps),
                c_.bypass_offset, c_.default_bias_sign, c_.side_deadband,
                c_.cg_tolerance, c_.cg_max_iterations,
                c_.cg_residual_acceptance, prior);
            const auto t4 = std::chrono::steady_clock::now();

            lv.ms_esdf  = wallMs(t0, t1);
            lv.ms_mask  = wallMs(t1, t2);
            lv.ms_label = wallMs(t2, t3);
            lv.ms_field = wallMs(t3, t4);
            return lv;
        };

        FluidLevel2D coarse;
        if (c_.use_coarse) {
            coarse = build_level(c_.window_forward_size, c_.window_lateral_size,
                                 c_.coarse_grid_resolution,
                                 /*centered=*/false, /*rear_margin_override=*/-1.0,
                                 nullptr,
                [&](const fluid2d::SolidComponents& comps) {
                    return computeLatchedSides(comps);
                });
        }
        FluidCoarsePrior prior;
        if (coarse.field.valid) {
            prior.psi = &coarse.field.psi;
            prior.origin = coarse.origin;
            prior.e_J = e_J;
            prior.n_J = n_J;
            prior.h = coarse.h;
            prior.n_forward = coarse.n_forward;
            prior.n_lateral = coarse.n_lateral;
            commitSideLatch(coarse.comps, coarse.field.chosen_sides);
        }
        FluidLevel2D fine;
        if (c_.use_fine) {
            fine = build_level(c_.fine_forward_size, c_.fine_lateral_size,
                               c_.grid_resolution,
                               /*centered=*/true, /*rear_margin_override=*/-1.0,
                               coarse.field.valid ? &prior : nullptr,
                [&](const fluid2d::SolidComponents& comps) {
                    std::vector<double> sides(comps.num_components, 0.0);
                    if (!coarse.field.valid) return sides;
                    for (int c = 0; c < comps.num_components; ++c) {
                        int best = -1;
                        double best_d = 1e18;
                        for (int cc = 0; cc < coarse.comps.num_components; ++cc) {
                            const double d = (comps.centroid_world[c] -
                                              coarse.comps.centroid_world[cc]).norm();
                            if (d < best_d) { best_d = d; best = cc; }
                        }
                        if (best >= 0 && best_d <= c_.side_match_centroid_dist &&
                            best < static_cast<int>(coarse.field.chosen_sides.size()) &&
                            std::abs(coarse.field.chosen_sides[best]) == 1.0) {
                            sides[c] = coarse.field.chosen_sides[best];
                        }
                    }
                    return sides;
                });
        }

        const FluidLevel2D& active = fine.field.valid ? fine : coarse;
        const double h = active.h;
        const int n_forward = active.n_forward;
        const int n_lateral = active.n_lateral;
        const double rear_margin = active.rear_margin;
        const Eigen::Vector2d origin = active.origin;
        const Eigen::MatrixXd& D = active.D;
        const std::vector<std::vector<bool>>& solid = active.solid;
        const fluid2d::FluidHarmonicField& field = active.field;

        if (field.valid) {
            // chi* continuity across the rebuild (chi*_new = chi_new(x) - (chi_old(x) - chi*_old)).
            if (fluid_psi_anchor_ref_valid_ && fluid_field_valid_ &&
                fluid_field_solve_speed_ > 1e-9) {
                double chi_old_here = 0.0, chi_new_here = 0.0;
                const bool old_ok = bilinearSamplePsi(
                    fluid_psi_, fluid_field_origin_, fluid_field_e_J_,
                    fluid_field_n_J_, fluid_field_h_, fluid_field_n_forward_,
                    fluid_field_n_lateral_, pos_xy, chi_old_here);
                const bool new_ok = bilinearSamplePsi(
                    field.psi, origin, e_J, n_J, h, n_forward, n_lateral,
                    pos_xy, chi_new_here);
                if (old_ok && new_ok) {
                    fluid_psi_anchor_ref_ = fluid2d::chiStarContinuityUpdate(
                        chi_old_here / fluid_field_solve_speed_,
                        chi_new_here / v_cap, fluid_psi_anchor_ref_);
                }
            } else {
                double chi_anchor = 0.0;
                if (bilinearSamplePsi(field.psi, origin, e_J, n_J, h,
                                      n_forward, n_lateral, anchor_xy,
                                      chi_anchor)) {
                    fluid_psi_anchor_ref_ = chi_anchor / v_cap;
                    fluid_psi_anchor_ref_valid_ = true;
                } else {
                    fluid_psi_anchor_ref_valid_ = false;
                }
            }
            fluid_psi_ = field.psi;
            fluid_Ux_field_ = field.Ux_query;
            fluid_Uy_field_ = field.Uy_query;
            fluid_field_origin_ = origin;
            fluid_field_e_J_ = e_J;
            fluid_field_n_J_ = n_J;
            fluid_field_h_ = h;
            fluid_field_n_forward_ = n_forward;
            fluid_field_n_lateral_ = n_lateral;
            fluid_field_solve_speed_ = v_cap;
            fluid_field_valid_ = true;
        }

        // Refresh the cached center-cell clearance + outward normal.
        {
            const int ci = std::max(1, std::min(
                n_forward - 2, static_cast<int>(std::round(rear_margin / h - 0.5))));
            const int cj = std::max(1, std::min(
                n_lateral - 2, static_cast<int>(std::round(0.5 * n_lateral - 0.5))));
            const double grad_s = (D(ci + 1, cj) - D(ci - 1, cj)) / (2.0 * h);
            const double grad_eta = (D(ci, cj + 1) - D(ci, cj - 1)) / (2.0 * h);
            const Eigen::Vector2d center_grad = grad_s * e_J + grad_eta * n_J;
            const double center_grad_norm = center_grad.norm();
            const double center_clearance = D(ci, cj);
            if (center_grad.allFinite() && center_grad_norm > 1e-6 &&
                center_grad_norm < 50.0 && std::isfinite(center_clearance) &&
                std::abs(center_clearance) < 100.0) {
                fluid_cached_clearance_ = center_clearance;
                fluid_cached_clearance_grad_ = center_grad;
                fluid_cached_clearance_grad_valid_ = true;
            }
        }
        fluid_last_solve_time_ = now_sec;

        // Build the visualization snapshot from the just-solved field (the
        // fused command field per cell, matching what the controller samples).
        if (build_viz) {
            Eigen::MatrixXd Ux_cmd, Uy_cmd;
            if (!c_.use_fine) {
                // Coarse-only ablation: draw the FUSED command field on the
                // full-reach grid.
                Ux_cmd.resize(n_forward, n_lateral);
                Uy_cmd.resize(n_forward, n_lateral);
                for (int qi = 0; qi < n_forward; ++qi) {
                    for (int qj = 0; qj < n_lateral; ++qj) {
                        if (solid[qi][qj]) {
                            Ux_cmd(qi, qj) = 0.0;
                            Uy_cmd(qi, qj) = 0.0;
                            continue;
                        }
                        const Eigen::Vector2d cell_pos = fluid2d::fluidCellWorld(
                            origin, e_J, n_J, h, qi, qj);
                        const double phi_cell = (cell_pos - anchor_xy).dot(n_J);
                        const Eigen::Vector2d v_cell = fuseLiftedGvfFluid(
                            Eigen::Vector2d(field.Ux(qi, qj), field.Uy(qi, qj)),
                            true, field.psi(qi, qj), true,
                            heading_rad, phi_cell, v_cap, D(qi, qj));
                        Ux_cmd(qi, qj) = v_cell.x();
                        Uy_cmd(qi, qj) = v_cell.y();
                    }
                }
                vis2d_.valid = true;
                vis2d_.coarse_style = true;
                vis2d_.origin = origin;
                vis2d_.e_J = e_J;
                vis2d_.n_J = n_J;
                vis2d_.h = h;
                vis2d_.n_forward = n_forward;
                vis2d_.n_lateral = n_lateral;
                vis2d_.Ux = Ux_cmd;
                vis2d_.Uy = Uy_cmd;
                vis2d_.solid = solid;
                vis2d_.coarse_valid = false;
            } else {
                Ux_cmd.resize(n_forward, n_lateral);
                Uy_cmd.resize(n_forward, n_lateral);
                for (int qi = 0; qi < n_forward; ++qi) {
                    for (int qj = 0; qj < n_lateral; ++qj) {
                        if (solid[qi][qj]) {
                            Ux_cmd(qi, qj) = 0.0;
                            Uy_cmd(qi, qj) = 0.0;
                            continue;
                        }
                        const Eigen::Vector2d field_uv(fluid_Ux_field_(qi, qj),
                                                       fluid_Uy_field_(qi, qj));
                        const Eigen::Vector2d cell_pos = fluid2d::fluidCellWorld(
                            origin, e_J, n_J, h, qi, qj);
                        const double phi_cell = (cell_pos - anchor_xy).dot(n_J);
                        const Eigen::Vector2d v_cell = fuseLiftedGvfFluid(
                            field_uv, true, fluid_psi_(qi, qj), true,
                            heading_rad, phi_cell, v_cap, D(qi, qj));
                        Ux_cmd(qi, qj) = v_cell.x();
                        Uy_cmd(qi, qj) = v_cell.y();
                    }
                }
                vis2d_.valid = true;
                vis2d_.coarse_style = false;
                vis2d_.origin = origin;
                vis2d_.e_J = e_J;
                vis2d_.n_J = n_J;
                vis2d_.h = h;
                vis2d_.n_forward = n_forward;
                vis2d_.n_lateral = n_lateral;
                vis2d_.Ux = Ux_cmd;
                vis2d_.Uy = Uy_cmd;
                vis2d_.solid = solid;
                vis2d_.coarse_valid = coarse.field.valid;
                if (coarse.field.valid) {
                    vis2d_.coarse_origin = coarse.origin;
                    vis2d_.coarse_h = coarse.h;
                    vis2d_.coarse_n_forward = coarse.n_forward;
                    vis2d_.coarse_n_lateral = coarse.n_lateral;
                    vis2d_.coarse_Ux = coarse.field.Ux;
                    vis2d_.coarse_Uy = coarse.field.Uy;
                    vis2d_.coarse_solid = coarse.solid;
                }
            }
        }
    } else if (due) {
        // Joystick released: u = 0 analytically; hold the last field.
        fluid_last_solve_time_ = now_sec;
    }

    // Per-tick command: bilinearly sample the cached field at the live pos.
    Eigen::Vector2d field_uv = Eigen::Vector2d::Zero();
    bool field_sample_valid = false;
    if (fluid_field_valid_) {
        field_sample_valid = bilinearSampleField(
            fluid_Ux_field_, fluid_Uy_field_, fluid_field_origin_,
            fluid_field_e_J_, fluid_field_n_J_, fluid_field_h_, fluid_field_n_forward_,
            fluid_field_n_lateral_, pos_xy, field_uv);
    }
    if (field_sample_valid && field_uv.squaredNorm() > 1e-12) {
        fluid_cached_t_field_ = field_uv.normalized();
        fluid_cached_t_field_valid_ = true;
    }
    double psi_here = 0.0;
    const bool psi_sample_valid = fluid_field_valid_ && bilinearSamplePsi(
        fluid_psi_, fluid_field_origin_, fluid_field_e_J_, fluid_field_n_J_,
        fluid_field_h_, fluid_field_n_forward_, fluid_field_n_lateral_,
        pos_xy, psi_here);

    const double phi_here = (pos_xy - anchor_xy).dot(n_J);
    Eigen::Vector2d u_raw = fuseLiftedGvfFluid(
        field_uv, field_sample_valid, psi_here, psi_sample_valid,
        heading_rad, phi_here, v_cap, D_here);

    // Near-wall escape net: keep a useful tangential/outward command at the
    // solid boundary instead of a dead stop.
    const double D_grad = fluid_cached_clearance_;
    const double grad_consistency_tol = std::max(0.50, 2.0 * c_.grid_resolution);
    const bool gradient_valid = fluid_cached_clearance_grad_valid_ &&
                                std::abs(D_grad - D_here) <= grad_consistency_tol;
    const double grad_norm = fluid_cached_clearance_grad_.norm();
    const double raw_speed = u_raw.norm();
    const double stall_lo = std::max(0.0, c_.escape_stall_lo_ratio) * v_cap;
    const double stall_hi = std::max(stall_lo + 1e-6,
                                     c_.escape_stall_hi_ratio * v_cap);
    const double stall_w = fluid2d::fluidSmoothstepW(raw_speed, stall_lo, stall_hi);
    const double near_wall_w =
        fluid2d::fluidSmoothstepW(D_here, c_.d_s, c_.d_drag);
    const bool in_open_near_band = D_here > c_.d_s && D_here < c_.d_turn;
    const double escape_w = D_here <= c_.d_s
        ? std::max(0.0, c_.escape_solid_weight)
        : (in_open_near_band ? stall_w * near_wall_w : 0.0);
    bool escape_active = false;
    if (v_cap > 1e-9 && escape_w > 1e-6 && gradient_valid) {
        const Eigen::Vector2d outward = fluid_cached_clearance_grad_ / grad_norm;
        const Eigen::Vector2d tangent_ccw(-outward.y(), outward.x());
        const Eigen::Vector2d align_ref =
            fluid_cached_t_field_valid_ ? fluid_cached_t_field_ : e_J;
        double tangent_alignment = tangent_ccw.dot(align_ref);
        if (std::abs(tangent_alignment) <= 1e-4) {
            tangent_alignment = tangent_ccw.dot(e_J);
        }
        const Eigen::Vector2d tangent =
            (tangent_alignment < 0.0 ? -1.0 : 1.0) * tangent_ccw;

        const double inward = u_raw.dot(outward);
        if (inward < 0.0) {
            u_raw -= inward * outward;
        }

        const double tangent_speed = u_raw.dot(tangent);
        if (tangent_speed >= -1e-3) {
            const double tangent_target = c_.escape_tangent_ratio * v_cap;
            const double tangent_supplement = escape_w * std::max(
                0.0, tangent_target - std::max(0.0, tangent_speed));
            u_raw += tangent_supplement * tangent;
        }

        const double outward_speed = u_raw.dot(outward);
        const double outward_target =
            c_.escape_outward_ratio * v_cap * near_wall_w;
        const double outward_supplement = escape_w * std::max(
            0.0, outward_target - std::max(0.0, outward_speed));
        u_raw += outward_supplement * outward;
        escape_active = true;
    }

    if (!u_raw.allFinite()) {
        u_raw.setZero();
        reset();
    }

    if (v_cap > 1e-9) {
        const double speed = u_raw.norm();
        if (speed > v_cap) u_raw *= v_cap / speed;
    } else {
        u_raw.setZero();
    }

    if (D_here <= c_.d_s && !gradient_valid) {
        u_raw.setZero();
    }
    setDiagnosticDirection(Eigen::Vector3d(u_raw.x(), u_raw.y(), 0.0));
    return u_raw;
}

// ------------------------------------------------------------------ 2D side

std::vector<double> FluidGuidance::computeLatchedSides(
    const fluid2d::SolidComponents& comps)
{
    std::vector<double> sides(comps.num_components, 0.0);
    fluid_side_latch_.erase(
        std::remove_if(fluid_side_latch_.begin(), fluid_side_latch_.end(),
                       [&](const FluidSideLatchEntry& e) {
                           return (now_sec_ - e.stamp) > c_.side_latch_ttl;
                       }),
        fluid_side_latch_.end());
    if (comps.num_components == 0 || fluid_side_latch_.empty()) return sides;

    const size_t latch_n = fluid_side_latch_.size();
    std::vector<bool> used(latch_n, false);
    auto overlap_ok = [&](const FluidSideLatchEntry& e, int c) {
        const double lat_lo = std::max(e.eta_min, comps.eta_min[c]);
        const double lat_hi = std::min(e.eta_max, comps.eta_max[c]);
        const double lat_len = std::max(1e-6, std::min(e.eta_max - e.eta_min,
                                                       comps.eta_max[c] - comps.eta_min[c]));
        if ((lat_hi - lat_lo) / lat_len < c_.side_match_lateral_tol) return false;
        const double s_lo = std::max(e.s_min, comps.s_min[c]);
        const double s_hi = std::min(e.s_max, comps.s_max[c]);
        const double s_len = std::max(1e-6, std::min(e.s_max - e.s_min,
                                                     comps.s_max[c] - comps.s_min[c]));
        if ((s_hi - s_lo) / s_len < c_.side_match_forward_tol) return false;
        return true;
    };
    for (int c = 0; c < comps.num_components; ++c) {
        int best = -1;
        double best_d = 1e18;
        for (size_t e = 0; e < latch_n; ++e) {
            if (used[e]) continue;
            const FluidSideLatchEntry& en = fluid_side_latch_[e];
            const double d = (comps.centroid_world[c] - en.centroid).norm();
            if (d > c_.side_match_centroid_dist || d >= best_d) continue;
            if (!overlap_ok(en, c)) continue;
            best_d = d;
            best = static_cast<int>(e);
        }
        if (best >= 0) {
            used[best] = true;
            sides[c] = fluid_side_latch_[best].side;
        }
    }
    return sides;
}

void FluidGuidance::commitSideLatch(const fluid2d::SolidComponents& comps,
                                    const std::vector<double>& chosen_sides)
{
    if (comps.num_components == 0) return;
    const size_t latch_n = fluid_side_latch_.size();
    std::vector<bool> used(latch_n, false);
    auto overlap_ok = [&](const FluidSideLatchEntry& e, int c) {
        const double lat_lo = std::max(e.eta_min, comps.eta_min[c]);
        const double lat_hi = std::min(e.eta_max, comps.eta_max[c]);
        const double lat_len = std::max(1e-6, std::min(e.eta_max - e.eta_min,
                                                       comps.eta_max[c] - comps.eta_min[c]));
        if ((lat_hi - lat_lo) / lat_len < c_.side_match_lateral_tol) return false;
        const double s_lo = std::max(e.s_min, comps.s_min[c]);
        const double s_hi = std::min(e.s_max, comps.s_max[c]);
        const double s_len = std::max(1e-6, std::min(e.s_max - e.s_min,
                                                     comps.s_max[c] - comps.s_min[c]));
        if ((s_hi - s_lo) / s_len < c_.side_match_forward_tol) return false;
        return true;
    };
    for (int c = 0; c < comps.num_components; ++c) {
        const double side = (c < static_cast<int>(chosen_sides.size()) &&
                             std::abs(chosen_sides[c]) == 1.0)
                                ? chosen_sides[c]
                                : 0.0;
        int best = -1;
        double best_d = 1e18;
        for (size_t e = 0; e < latch_n; ++e) {
            if (used[e]) continue;
            const FluidSideLatchEntry& en = fluid_side_latch_[e];
            const double d = (comps.centroid_world[c] - en.centroid).norm();
            if (d > c_.side_match_centroid_dist || d >= best_d) continue;
            if (!overlap_ok(en, c)) continue;
            best_d = d;
            best = static_cast<int>(e);
        }
        FluidSideLatchEntry entry;
        entry.centroid = comps.centroid_world[c];
        entry.side = side;
        entry.eta_min = comps.eta_min[c];
        entry.eta_max = comps.eta_max[c];
        entry.s_min = comps.s_min[c];
        entry.s_max = comps.s_max[c];
        entry.stamp = now_sec_;
        if (best >= 0) {
            used[best] = true;
            fluid_side_latch_[best] = entry;
        } else if (side != 0.0) {
            fluid_side_latch_.push_back(entry);
        }
    }
}

// ------------------------------------------------------------------ 3D law

Eigen::Vector3d FluidGuidance::fuseLiftedGvfFluid3D(const Eigen::Vector3d& field_uvw,
                                                    bool field_valid,
                                                    double heading_rad,
                                                    double phi, double v_cap,
                                                    double D_local) const
{
    // Horizontal angle law = fuseLiftedGvfFluid verbatim; w passes through.
    const Eigen::Vector2d uh = field_uvw.head<2>();
    const bool uh_valid = field_valid && uh.squaredNorm() > 1e-12;
    const double theta_field =
        uh_valid ? std::atan2(uh.y(), uh.x()) : heading_rad;

    const double theta_ray_target =
        heading_rad - std::atan(phi / c_.conv_length);
    double delta = theta_ray_target - theta_field;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    const double conv_gate = 1.0 - fluid2d::fluidSmoothstepW(
        D_local, c_.d_look, c_.d_look + c_.conv_gate_margin);
    delta = std::max(-c_.delta_theta_max_rad,
                     std::min(c_.delta_theta_max_rad, delta)) * conv_gate;
    const double theta_cmd = theta_field + delta;

    const double uh_norm = uh_valid ? uh.norm() : 0.0;
    Eigen::Vector3d dir(std::cos(theta_cmd) * uh_norm,
                        std::sin(theta_cmd) * uh_norm,
                        field_valid ? field_uvw.z() : 0.0);
    if (dir.squaredNorm() < 1e-12) {
        dir = Eigen::Vector3d(std::cos(heading_rad), std::sin(heading_rad), 0.0);
    }
    const double speed = v_cap * std::max(c_.speed_floor,
        fluid2d::fluidSpeedRatio(D_local, c_.d_s, c_.d_drag));
    return speed * dir.normalized();
}

Eigen::Vector3d FluidGuidance::calcGuidance3D(const Eigen::Vector3d& pos,
                                              const Eigen::Vector2d& anchor_xy,
                                              double heading_rad, double v_J,
                                              double now_sec, bool build_viz)
{
    now_sec_ = now_sec;
    if (!distance_) return Eigen::Vector3d::Zero();

    const double v_cap = std::max(0.0, v_J);
    const Eigen::Vector2d pos_xy = pos.head<2>();
    const Eigen::Vector2d e_J(std::cos(heading_rad), std::sin(heading_rad));
    const Eigen::Vector2d n_J(-std::sin(heading_rad), std::cos(heading_rad));
    vis3d_.pos = pos;
    vis3d_.anchor_xy = anchor_xy;
    vis3d_.heading_rad = heading_rad;
    vis3d_.v_cap = v_cap;

    double D_here = distance_(pos);
    if (!std::isfinite(D_here) || std::abs(D_here) > 1e6) {
        D_here = fluid3d_cached_clearance_grad_valid_ ? fluid3d_cached_clearance_ : 0.0;
    }

    const double tick_dt = fluid3d_last_tick_time_ <= 0.0
        ? 0.0
        : std::max(0.0, std::min(0.1, now_sec - fluid3d_last_tick_time_));
    fluid3d_last_tick_time_ = now_sec;
    const bool due = (now_sec - fluid_last_solve_time_) >= c_.resolve_period;

    if (due && v_cap > 1e-9) {
        const auto solve_started = std::chrono::steady_clock::now();
        const double ustar_lat = fluid3d_cross_beta_ * v_cap;

        auto build_level3d = [&](double forward_size, double lateral_size,
                                 double hh, double hz, bool centered,
                                 const fluid3d::CoarsePrior3D* prior) {
            FluidLevel3D lv;
            lv.g.e_J = e_J;
            lv.g.n_J = n_J;
            lv.g.h = hh;
            lv.g.h_z = hz;
            lv.g.z_lo = c_.z_min;
            lv.g.n_fwd = std::max(3, static_cast<int>(std::round(forward_size / hh)));
            lv.g.n_lat = std::max(3, static_cast<int>(std::round(lateral_size / hh)));
            lv.g.n_z = std::max(3, static_cast<int>(std::round(
                (c_.z_max - c_.z_min) / hz)));
            lv.rear_margin = centered
                ? 0.5 * lv.g.n_fwd * hh
                : std::max(1.5 * hh, std::min((lv.g.n_fwd - 1.5) * hh, c_.window_rear_margin));
            lv.g.origin_xy = pos_xy - lv.rear_margin * e_J - 0.5 * lv.g.n_lat * hh * n_J;

            const double z_floor_top = c_.z_min + c_.floor_band;
            const double z_ceil_bot = lv.g.z_lo + lv.g.n_z * hz - c_.ceil_band;
            std::vector<uint8_t> band_solid(lv.g.n_z, 0);
            lv.m_lo = lv.g.n_z;
            lv.m_hi = -1;
            for (int m = 0; m < lv.g.n_z; ++m) {
                const double zc = lv.g.z_lo + (m + 0.5) * hz;
                band_solid[m] = (zc <= z_floor_top || zc >= z_ceil_bot) ? 1 : 0;
                if (!band_solid[m]) {
                    lv.m_lo = std::min(lv.m_lo, m);
                    lv.m_hi = std::max(lv.m_hi, m);
                }
            }

            const auto t0 = std::chrono::steady_clock::now();
            lv.D.assign(lv.g.size(), 0.0);
#pragma omp parallel for collapse(2) schedule(static) num_threads(4)
            for (int i = 0; i < lv.g.n_fwd; ++i) {
                for (int j = 0; j < lv.g.n_lat; ++j) {
                    for (int m = 0; m < lv.g.n_z; ++m) {
                        double sample = distance_(lv.g.cellWorld(i, j, m));
                        if (!std::isfinite(sample) || std::abs(sample) > 1e6) {
                            sample = 0.0;
                        }
                        lv.D[lv.g.idx(i, j, m)] = sample;
                    }
                }
            }

            const auto t1 = std::chrono::steady_clock::now();
            lv.solid.assign(lv.g.size(), 0);
            for (int i = 0; i < lv.g.n_fwd; ++i) {
                for (int j = 0; j < lv.g.n_lat; ++j) {
                    for (int m = 0; m < lv.g.n_z; ++m) {
                        const int k = lv.g.idx(i, j, m);
                        lv.solid[k] =
                            (band_solid[m] || lv.D[k] <= c_.d_s) ? 1 : 0;
                    }
                }
            }
            fluid3d::foldUnreachablePockets3D(lv.solid, lv.g);

            const auto t2 = std::chrono::steady_clock::now();
            lv.comps = fluid3d::labelSolidComponents3D(
                lv.solid, lv.g, anchor_xy, pos_xy, lv.m_lo, lv.m_hi);
            const auto t3 = std::chrono::steady_clock::now();
            lv.field = fluid3d::solvePotentialFlow3D(
                lv.solid, lv.g, v_cap, ustar_lat, prior,
                c_.cg_tolerance, c_.cg_max_iterations,
                c_.cg_residual_acceptance);
            const auto t4 = std::chrono::steady_clock::now();

            lv.ms_esdf  = wallMs(t0, t1);
            lv.ms_mask  = wallMs(t1, t2);
            lv.ms_label = wallMs(t2, t3);
            lv.ms_field = wallMs(t3, t4);
            return lv;
        };

        const FluidLevel3D coarse = build_level3d(
            c_.window_forward_size, c_.window_lateral_size,
            c_.coarse_grid_resolution, c_.coarse_resolution_z,
            /*centered=*/false, nullptr);
        fluid3d::CoarsePrior3D prior;
        if (coarse.field.valid) {
            prior.phi = &coarse.field.phi;
            prior.grid = coarse.g;
        }
        const FluidLevel3D fine = build_level3d(
            c_.fine_forward_size, c_.fine_lateral_size,
            c_.fine_resolution, c_.grid_resolution_z,
            /*centered=*/true, coarse.field.valid ? &prior : nullptr);

        if (fine.field.valid) {
            fluid3d_field_grid_ = fine.g;
            fluid3d_Ux_ = fine.field.Ux;
            fluid3d_Uy_ = fine.field.Uy;
            fluid3d_Uz_ = fine.field.Uz;
            fluid3d_phi_ = fine.field.phi;
            fluid3d_field_valid_ = true;
            fluid3d_D_ = fine.D;
            fluid3d_solid_ = fine.solid;
            fluid3d_field_data_valid_ = true;
            fluid3d_field_solve_speed_ = v_cap;
            updateStreamline3D(pos, fine.g, v_cap);
            // Freeze the existing numerical field and the selected seed for
            // section-return coordinates. No change to the coarse/fine solve.
            auto snapshot = std::make_shared<fluid3d::CoordinateGrid3D>();
            snapshot->grid = fine.g;
            snapshot->p = fine.field.phi;
            snapshot->ux = fine.field.Ux;
            snapshot->uy = fine.field.Uy;
            snapshot->uz = fine.field.Uz;
            snapshot->solid = fine.solid;
            snapshot->far_velocity = Eigen::Vector3d(
                v_cap * e_J.x() + ustar_lat * n_J.x(),
                v_cap * e_J.y() + ustar_lat * n_J.y(), 0.0);
            snapshot->speed_scale = v_cap;
            flow_coordinate_grid_ = snapshot;
            ++flow_field_version_;
            flow_coordinates_ = fluid3d::FlowCoordinates3D{};
            if (streamline_view_.valid) {
                const auto seed = std::min_element(streamline_view_.arc.begin(),
                    streamline_view_.arc.end(), [](double a, double b) {
                        return std::abs(a) < std::abs(b);
                    });
                const std::size_t index = std::distance(streamline_view_.arc.begin(), seed);
                flow_coordinates_.reset([snapshot](const Eigen::Vector3d& x) {
                    return snapshot->sample(x);
                }, streamline_view_.points[index]);
            }
        }

        fluid3d_coarse_extents_.clear();
        for (int c = 0; c < coarse.comps.num_components; ++c) {
            fluid3d_coarse_extents_.push_back({coarse.comps.eta_min[c],
                                               coarse.comps.eta_max[c],
                                               coarse.comps.s_min[c],
                                               coarse.comps.z_max[c]});
        }

        {
            const fluid3d::Grid3D& g = fine.g;
            const int ci = std::max(1, std::min(g.n_fwd - 2,
                static_cast<int>(std::round(fine.rear_margin / g.h - 0.5))));
            const int cj = std::max(1, std::min(g.n_lat - 2,
                static_cast<int>(std::round(0.5 * g.n_lat - 0.5))));
            const int cm = std::max(1, std::min(g.n_z - 2,
                static_cast<int>(std::round((pos.z() - g.z_lo) / g.h_z - 0.5))));
            const double grad_s =
                (fine.D[g.idx(ci + 1, cj, cm)] - fine.D[g.idx(ci - 1, cj, cm)]) /
                (2.0 * g.h);
            const double grad_eta =
                (fine.D[g.idx(ci, cj + 1, cm)] - fine.D[g.idx(ci, cj - 1, cm)]) /
                (2.0 * g.h);
            const double grad_z =
                (fine.D[g.idx(ci, cj, cm + 1)] - fine.D[g.idx(ci, cj, cm - 1)]) /
                (2.0 * g.h_z);
            const Eigen::Vector3d center_grad(
                grad_s * e_J.x() + grad_eta * n_J.x(),
                grad_s * e_J.y() + grad_eta * n_J.y(),
                grad_z);
            const double center_grad_norm = center_grad.norm();
            const double center_clearance = fine.D[g.idx(ci, cj, cm)];
            if (center_grad.allFinite() && center_grad_norm > 1e-6 &&
                center_grad_norm < 50.0 && std::isfinite(center_clearance) &&
                std::abs(center_clearance) < 100.0) {
                fluid3d_cached_clearance_ = center_clearance;
                fluid3d_cached_clearance_grad_ = center_grad;
                fluid3d_cached_clearance_grad_valid_ = true;
            }
        }
        fluid_last_solve_time_ = now_sec;

        // Visualization snapshot: expose the solved fine/coarse fields so the
        // ROS adapter can build the quiver frame (subscriber-gated upstream).
        if (build_viz) {
            vis3d_.valid = true;
            vis3d_.grid = fine.g;
            vis3d_.D = fine.D;
            vis3d_.solid = fine.solid;
            vis3d_.field = fine.field;
            vis3d_.coarse_valid = coarse.field.valid;
            if (coarse.field.valid) {
                vis3d_.grid_coarse = coarse.g;
                vis3d_.solid_coarse = coarse.solid;
                vis3d_.field_coarse = coarse.field;
            }
        }
    } else if (due) {
        fluid_last_solve_time_ = now_sec;
    }

    // Per-tick command: trilinearly sample the cached field at the live 3D pos.
    Eigen::Vector3d field_uvw = Eigen::Vector3d::Zero();
    bool field_sample_valid = false;
    if (fluid3d_field_valid_) {
        field_sample_valid = fluid3d::trilinearSampleField3D(
            fluid3d_Ux_, fluid3d_Uy_, fluid3d_Uz_, fluid3d_field_grid_,
            pos, field_uvw);
    }
    if (field_sample_valid && field_uvw.squaredNorm() > 1e-12) {
        fluid_diag_field_direction_ = field_uvw.normalized();
        fluid_diag_field_valid_ = true;
    } else {
        fluid_diag_field_direction_.setZero();
        fluid_diag_field_valid_ = false;
    }
    fluid_diag_potential_valid_ = false;
    if (fluid3d_field_data_valid_ && !fluid3d_phi_.empty()) {
        const Eigen::Vector2d rel = pos_xy - fluid3d_field_grid_.origin_xy;
        const double gi = rel.dot(fluid3d_field_grid_.e_J) /
                          fluid3d_field_grid_.h - 0.5;
        const double gj = rel.dot(fluid3d_field_grid_.n_J) /
                          fluid3d_field_grid_.h - 0.5;
        const double gm = (pos.z() - fluid3d_field_grid_.z_lo) /
                          fluid3d_field_grid_.h_z - 0.5;
        const int i0 = static_cast<int>(std::floor(gi));
        const int j0 = static_cast<int>(std::floor(gj));
        const int m0 = static_cast<int>(std::floor(gm));
        if (i0 >= 0 && j0 >= 0 && m0 >= 0 &&
            i0 + 1 < fluid3d_field_grid_.n_fwd &&
            j0 + 1 < fluid3d_field_grid_.n_lat &&
            m0 + 1 < fluid3d_field_grid_.n_z) {
            const double tx = gi - i0, ty = gj - j0, tz = gm - m0;
            auto at = [&](int i, int j, int m) {
                return fluid3d_phi_[fluid3d_field_grid_.idx(i, j, m)];
            };
            auto lerp3 = [&](int i, int j) {
                const double a = at(i, j, m0) * (1.0 - tz) + at(i, j, m0 + 1) * tz;
                const double b = at(i, j + 1, m0) * (1.0 - tz) + at(i, j + 1, m0 + 1) * tz;
                return a * (1.0 - ty) + b * ty;
            };
            const double a = lerp3(i0, j0);
            const double b = lerp3(i0 + 1, j0);
            fluid_diag_potential_ = a * (1.0 - tx) + b * tx;
            fluid_diag_potential_valid_ = std::isfinite(fluid_diag_potential_);
        }
    }
    if (field_sample_valid && field_uvw.squaredNorm() > 1e-12) {
        fluid3d_cached_t_field_ = field_uvw.normalized();
        fluid3d_cached_t_field_valid_ = true;
    }

    const auto coordinate_started = std::chrono::steady_clock::now();
    flow_error_diag_ = FlowErrorDiagnostics3D{};
    flow_error_diag_.query = pos;
    flow_error_diag_.field_version = flow_field_version_;
    if (flow_coordinate_grid_) {
        if (flow_coordinates_.status() != fluid3d::CoordinateStatus3D::kNoField)
            flow_error_diag_.anchor = flow_coordinates_.anchor();
        flow_error_diag_.coordinate = flow_coordinates_.evaluate(pos);
        const auto sample = flow_coordinate_grid_->sample(pos);
        if (sample.status == fluid3d::CoordinateStatus3D::kOk)
            flow_error_diag_.gradient_relative_error =
                (sample.velocity - sample.potential_gradient).norm() /
                std::max(1e-12, sample.velocity.norm());
    }
    flow_error_diag_.compute_ms = wallMs(coordinate_started, std::chrono::steady_clock::now());

    const double phi_here = (pos_xy - anchor_xy).dot(n_J);
    const bool streamline_ok = streamline_view_.valid &&
                               fluid3d_field_data_valid_ &&
                               fluid3d_field_valid_;
    Eigen::Vector3d v_nominal = Eigen::Vector3d::Zero();
    Eigen::Vector3d u_raw;
    v_nominal = calcStreamlineGuidance3D(pos, v_cap, D_here,
                                       field_sample_valid ? field_uvw.z() : 0.0,
                                       heading_rad);
    u_raw = v_nominal;
    if (streamline_ok) {
        const double theta_cmd = std::atan2(u_raw.y(), u_raw.x());
        const double theta_target = heading_rad - std::atan(phi_here / c_.conv_length);
        double delta = theta_target - theta_cmd;
        while (delta > M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        const double conv_gate = c_.freeze_streamline ? 0.0 :
            (1.0 - fluid2d::fluidSmoothstepW(
                D_here, c_.d_look, c_.d_look + c_.conv_gate_margin));
        delta = std::max(-c_.delta_theta_max_rad,
                         std::min(c_.delta_theta_max_rad, delta)) * conv_gate;
        const double theta_final = theta_cmd + delta;
        const double h_norm = u_raw.head<2>().norm();
        u_raw.x() = h_norm * std::cos(theta_final);
        u_raw.y() = h_norm * std::sin(theta_final);
        fluid3d_track_mode_ = 0;
    } else {
        // A missing chart is not replaced by a different nominal controller.
        // Existing downstream navigation/safety processing remains unchanged.
        fluid3d_track_mode_ = 2;
    }
    setDiagnosticDirection(v_nominal);

    // Near-wall escape net, 3D form.
    const double D_grad = fluid3d_cached_clearance_;
    const double grad_consistency_tol =
        std::max(0.50, 2.0 * c_.fine_resolution);
    const bool gradient_valid = fluid3d_cached_clearance_grad_valid_ &&
                                std::abs(D_grad - D_here) <= grad_consistency_tol;
    const double grad_norm = fluid3d_cached_clearance_grad_.norm();
    const double raw_speed = u_raw.norm();
    const double stall_lo = std::max(0.0, c_.escape_stall_lo_ratio) * v_cap;
    const double stall_hi = std::max(stall_lo + 1e-6,
                                     c_.escape_stall_hi_ratio * v_cap);
    const double stall_w = fluid2d::fluidSmoothstepW(raw_speed, stall_lo, stall_hi);
    const double near_wall_w =
        fluid2d::fluidSmoothstepW(D_here, c_.d_s, c_.d_drag);
    const bool in_open_near_band = D_here > c_.d_s && D_here < c_.d_turn;
    const double escape_w = D_here <= c_.d_s
        ? std::max(0.0, c_.escape_solid_weight)
        : (in_open_near_band ? stall_w * near_wall_w : 0.0);
    bool escape_active = false;
    if (v_cap > 1e-9 && escape_w > 1e-6 && gradient_valid) {
        const Eigen::Vector3d outward = fluid3d_cached_clearance_grad_ / grad_norm;
        const Eigen::Vector3d align_ref = fluid3d_cached_t_field_valid_
            ? fluid3d_cached_t_field_
            : Eigen::Vector3d(e_J.x(), e_J.y(), 0.0);
        Eigen::Vector3d tangent = align_ref - align_ref.dot(outward) * outward;
        if (tangent.squaredNorm() < 1e-12) {
            const Eigen::Vector3d hdir(e_J.x(), e_J.y(), 0.0);
            tangent = hdir - hdir.dot(outward) * outward;
        }
        const bool tangent_ok = tangent.squaredNorm() > 1e-12;
        if (tangent_ok) tangent.normalize();

        const double inward = u_raw.dot(outward);
        if (inward < 0.0) {
            u_raw -= inward * outward;
        }
        if (tangent_ok) {
            const double tangent_speed = u_raw.dot(tangent);
            if (tangent_speed >= -1e-3) {
                const double tangent_target = c_.escape_tangent_ratio * v_cap;
                const double tangent_supplement = escape_w * std::max(
                    0.0, tangent_target - std::max(0.0, tangent_speed));
                u_raw += tangent_supplement * tangent;
            }
        }
        const double outward_speed = u_raw.dot(outward);
        const double outward_target =
            c_.escape_outward_ratio * v_cap * near_wall_w;
        const double outward_supplement = escape_w * std::max(
            0.0, outward_target - std::max(0.0, outward_speed));
        u_raw += outward_supplement * outward;
        escape_active = true;
    }
    if (escape_active && fluid3d_track_mode_ == 0) fluid3d_track_mode_ = 1;

    if (!u_raw.allFinite()) {
        u_raw.setZero();
        reset();
    }

    const bool hard_stop = D_here <= c_.d_s && !gradient_valid;

    // Altitude homing, gated on the clearance AT CRUISE altitude.
    bool homing_applied = false;
    if (v_cap > 1e-9 && !hard_stop && c_.alt_gain > 0.0) {
        double D_cruise = distance_(
            Eigen::Vector3d(pos.x(), pos.y(), c_.cruise_z));
        if (!std::isfinite(D_cruise) || std::abs(D_cruise) > 1e6) {
            D_cruise = 0.0;
        }
        const double gate = 1.0 - fluid2d::fluidSmoothstepW(
            D_cruise, c_.alt_gate_lo, c_.alt_gate_hi);
        homing_applied = gate > 0.01;
        if (pos.z() < c_.cruise_z && u_raw.z() < 0.0) {
            u_raw.z() *= (1.0 - gate);
        }
        const double w_home = std::max(-c_.alt_rate_max,
            std::min(c_.alt_rate_max,
                     c_.alt_gain * (c_.cruise_z - pos.z())));
        u_raw.z() += gate * w_home;
    }

    // Vertical-lane preview.
    if (v_cap > 1e-9 && c_.vertical_preview_len > 1e-6 &&
        c_.vertical_preview_rate > 1e-6) {
        const double z_lo = c_.z_min + c_.floor_band + c_.grid_resolution_z;
        const double z_hi = c_.z_max - c_.ceil_band - c_.grid_resolution_z;
        if (z_hi > z_lo) {
            const double z_up = z_hi;
            const double z_down = z_lo;
            double current_clearance = std::numeric_limits<double>::infinity();
            double up_clearance = current_clearance;
            double down_clearance = current_clearance;
            auto clearance_at = [&](const Eigen::Vector3d& point) {
                const double value = distance_(point);
                return std::isfinite(value) && std::abs(value) <= 1e6 ? value : 0.0;
            };
            const int samples = std::max(1, static_cast<int>(std::ceil(
                c_.vertical_preview_len / std::max(0.1, c_.fine_resolution))));
            for (int i = 1; i <= samples; ++i) {
                const Eigen::Vector2d xy = pos_xy +
                    (c_.vertical_preview_len * i / samples) * e_J;
                current_clearance = std::min(current_clearance,
                    clearance_at(Eigen::Vector3d(xy.x(), xy.y(), pos.z())));
                up_clearance = std::min(up_clearance,
                    clearance_at(Eigen::Vector3d(xy.x(), xy.y(), z_up)));
                down_clearance = std::min(down_clearance,
                    clearance_at(Eigen::Vector3d(xy.x(), xy.y(), z_down)));
            }
            const double clearance = c_.vertical_preview_clearance;
            const bool up_open = up_clearance >= clearance;
            const bool down_open = down_clearance >= clearance;
            if (current_clearance < clearance && (up_open || down_open)) {
                // ponytail: endpoint-lane probe; use a trajectory feasibility
                // check if slanted blockers make this approximation insufficient.
                const bool choose_up = up_open && (!down_open ||
                    z_up - pos.z() <= pos.z() - z_down);
                u_raw.z() += choose_up ? c_.vertical_preview_rate
                                       : -c_.vertical_preview_rate;
            }
        }
    }

    // The joystick cap is a PLANAR authority limit; z is capped separately by
    // the manager's actuator clamp before publish.
    if (v_cap > 1e-9) {
        const double horizontal_speed = u_raw.head<2>().norm();
        if (horizontal_speed > v_cap) {
            u_raw.head<2>() *= v_cap / horizontal_speed;
        }
    } else {
        u_raw.setZero();
    }

    if (hard_stop) {
        u_raw.setZero();
        fluid3d_track_mode_ = 2;
    }

    // Record the SAFETY-layer modification d_v = v_final - v_nominal.
    {
        const Eigen::Vector3d d_v = u_raw - v_nominal;
        const double d_norm = d_v.norm();
        fluid3d_d_v_observed_ = std::max(fluid3d_d_v_observed_, d_norm);
        fluid3d_d_v_current_ = d_norm;
        if (fluid3d_track_mode_ == 0 && d_norm > 0.05 && !hard_stop) {
            fluid3d_track_mode_ = 1;
        }
    }

    // Crossflow latch update, last: it consumes this tick's field sample and
    // only influences the NEXT solve's u*.
    if (v_cap > 1e-9) {
        if (!fluid3d_cross_latched_) {
            const bool stalled = field_sample_valid &&
                field_uvw.norm() < c_.stall_speed_ratio * v_cap &&
                std::abs(field_uvw.z()) < c_.stall_w_veto;
            bool blocked = false;
            if (stalled) {
                const double z_band_lo = c_.z_min + c_.floor_band +
                                         0.5 * c_.grid_resolution_z;
                const double z_band_hi = c_.z_max - c_.ceil_band -
                                         0.5 * c_.grid_resolution_z;
                const double z_probe = z_band_hi > z_band_lo
                    ? std::max(z_band_lo, std::min(z_band_hi, pos.z()))
                    : pos.z();
                double D_ahead = 1e9;
                for (double frac : {0.6, 1.0}) {
                    const Eigen::Vector2d p_xy =
                        pos_xy + frac * c_.stall_probe * e_J;
                    double d = distance_(
                        Eigen::Vector3d(p_xy.x(), p_xy.y(), z_probe));
                    if (!std::isfinite(d) || std::abs(d) > 1e6) d = 0.0;
                    D_ahead = std::min(D_ahead, d);
                }
                blocked = D_ahead < c_.stall_clearance;
            }
            fluid3d_stall_accum_ = (stalled && blocked)
                ? fluid3d_stall_accum_ + tick_dt : 0.0;
            if (fluid3d_stall_accum_ >= c_.stall_latch_time) {
                const double eta_robot = (pos_xy - anchor_xy).dot(n_J);
                double side = c_.default_bias_sign;
                double best_s = 1e18;
                for (const auto& c : fluid3d_coarse_extents_) {
                    if (c.s_min < 0.0 || c.s_min > c_.stall_probe + 1.0) continue;
                    if (eta_robot < c.eta_min - c_.side_deadband ||
                        eta_robot > c.eta_max + c_.side_deadband) continue;
                    if (c.s_min >= best_s) continue;
                    best_s = c.s_min;
                    const double cost_plus = std::max(0.0, c.eta_max - eta_robot);
                    const double cost_minus = std::max(0.0, eta_robot - c.eta_min);
                    if (std::abs(cost_plus - cost_minus) < c_.side_deadband) {
                        side = c_.default_bias_sign;
                    } else {
                        side = (cost_plus <= cost_minus) ? 1.0 : -1.0;
                    }
                }
                fluid3d_cross_side_ = side;
                fluid3d_cross_latched_ = true;
                fluid3d_release_accum_ = 0.0;
            }
        } else {
            const bool recovered = field_sample_valid &&
                field_uvw.norm() >= c_.stall_release_ratio * v_cap;
            fluid3d_release_accum_ = recovered
                ? fluid3d_release_accum_ + tick_dt : 0.0;
            if (fluid3d_release_accum_ >= c_.stall_release_time) {
                fluid3d_cross_latched_ = false;
                fluid3d_stall_accum_ = 0.0;
            }
        }
        const double beta_des = (fluid3d_cross_latched_ ? fluid3d_cross_side_ : 0.0) *
                                c_.crossflow_ratio;
        const double beta_err = beta_des - fluid3d_cross_beta_;
        const double beta_rate = std::max(-c_.crossflow_rate_max,
            std::min(c_.crossflow_rate_max,
                     beta_err / std::max(1e-6, c_.crossflow_tau)));
        fluid3d_cross_beta_ += beta_rate * tick_dt;
    }

    return u_raw;
}

// ---- 3D local reference streamline ----

void FluidGuidance::updateStreamline3D(const Eigen::Vector3d& pos,
                                       const fluid3d::Grid3D& g, double v_cap)
{
    Streamline3DView next;
    if (!fluid3d_field_valid_ || !fluid3d_field_data_valid_) {
        streamline_view_.valid = false;
        return;
    }
    if (c_.freeze_streamline && streamline_view_.valid &&
        streamline_view_.points.size() >= 2) {
        return;
    }
    if (now_sec_ < fluid3d_streamline_degrade_until_) {
        streamline_view_.valid = false;
        return;
    }
    const double ds = std::max(1e-3, c_.streamline_ds);
    const double u_min = 0.02 * std::max(0.1, fluid3d_field_solve_speed_);
    Eigen::Vector3d seed = pos;
    if (streamline_view_.valid && streamline_view_.points.size() >= 2) {
        const StreamlineProj3D proj = projectToStreamline3D(pos);
        seed = proj.p;
    }

    auto solid_at = [&](const Eigen::Vector3d& p) {
        const Eigen::Vector2d rel = p.head<2>() - g.origin_xy;
        const int i = std::max(0, std::min(g.n_fwd - 1,
            static_cast<int>(std::floor(rel.dot(g.e_J) / g.h))));
        const int j = std::max(0, std::min(g.n_lat - 1,
            static_cast<int>(std::floor(rel.dot(g.n_J) / g.h))));
        const int m = std::max(0, std::min(g.n_z - 1,
            static_cast<int>(std::floor((p.z() - g.z_lo) / g.h_z))));
        return fluid3d_solid_[g.idx(i, j, m)] != 0;
    };
    auto sample_u = [&](const Eigen::Vector3d& p, Eigen::Vector3d& u) {
        return fluid3d::trilinearSampleField3D(
            fluid3d_Ux_, fluid3d_Uy_, fluid3d_Uz_, g, p, u);
    };
    auto step = [&](const Eigen::Vector3d& p, double dir, Eigen::Vector3d& p_next) {
        Eigen::Vector3d u, u_mid;
        if (!sample_u(p, u) || solid_at(p)) return false;
        const double un = u.norm();
        if (un < u_min) return false;
        const Eigen::Vector3d p_mid = p + 0.5 * ds * dir * (u / un);
        if (solid_at(p_mid)) return false;
        if (!sample_u(p_mid, u_mid)) return false;
        const double unm = u_mid.norm();
        if (unm < u_min) return false;
        p_next = p + ds * dir * (u_mid / unm);
        return !solid_at(p_next);
    };

    std::vector<Eigen::Vector3d> pts, tans;
    pts.push_back(seed);
    {
        Eigen::Vector3d u0;
        Eigen::Vector3d t0 = (sample_u(seed, u0) && u0.norm() > u_min)
                                 ? u0.normalized()
                                 : Eigen::Vector3d::UnitX();
        tans.push_back(t0);
    }
    {
        Eigen::Vector3d p = seed;
        double s = 0.0;
        while (s < c_.streamline_bwd_len) {
            Eigen::Vector3d p_next;
            if (!step(p, -1.0, p_next)) break;
            pts.insert(pts.begin(), p_next);
            tans.insert(tans.begin(), p_next - p);
            p = p_next;
            s += ds;
        }
    }
    {
        Eigen::Vector3d p = seed;
        double s = 0.0;
        while (s < c_.streamline_fwd_len) {
            Eigen::Vector3d p_next;
            if (!step(p, 1.0, p_next)) break;
            pts.push_back(p_next);
            tans.push_back(p_next - p);
            p = p_next;
            s += ds;
        }
    }
    for (size_t k = 0; k < tans.size(); ++k) {
        if (tans[k].norm() > 1e-6) tans[k].normalize();
    }
    if (pts.size() < 2) {
        streamline_view_.valid = false;
        return;
    }

    double max_disp = 0.0;
    if (streamline_view_.valid && streamline_view_.points.size() >= 2) {
        const Streamline3DView& old = streamline_view_;
        for (const Eigen::Vector3d& p : pts) {
            double best = 1e18;
            for (size_t k = 0; k + 1 < old.points.size(); ++k) {
                const Eigen::Vector3d d = old.points[k + 1] - old.points[k];
                const double L2 = d.squaredNorm();
                const double s = (L2 > 1e-12)
                    ? std::min(1.0, std::max(0.0, (p - old.points[k]).dot(d) / L2))
                    : 0.0;
                best = std::min(best, (p - (old.points[k] + s * d)).squaredNorm());
            }
            max_disp = std::max(max_disp, std::sqrt(best));
        }
        if (max_disp > c_.streamline_max_disp) {
            streamline_view_.valid = false;
            fluid3d_streamline_degrade_until_ =
                now_sec_ + c_.streamline_degrade_cooldown;
            return;
        }
    }

    next.arc.resize(pts.size(), 0.0);
    for (size_t k = 1; k < pts.size(); ++k) {
        const Eigen::Vector3d d = pts[k] - pts[k - 1];
        next.arc[k] = next.arc[k - 1] + (k > 0 && pts[k] == pts[k - 1] ? 0.0 : d.norm());
    }
    const size_t seed_k = std::distance(pts.begin(), std::find(pts.begin(), pts.end(), seed));
    for (double& a : next.arc) a -= next.arc[seed_k];
    next.points = pts;
    next.tangents = tans;
    next.valid = true;
    next.end_cap = false;
    streamline_view_ = next;
    fluid3d_streamline_update_disp_ = max_disp;
    ++fluid3d_streamline_version_;
    fluid3d_streamline_clearance_min_ = std::numeric_limits<double>::quiet_NaN();
    if (distance_) {
        double min_clearance = std::numeric_limits<double>::infinity();
        for (const Eigen::Vector3d& p : streamline_view_.points) {
            min_clearance = std::min(min_clearance, distance_(p));
        }
        if (std::isfinite(min_clearance)) fluid3d_streamline_clearance_min_ = min_clearance;
    }
}

FluidGuidance::StreamlineProj3D FluidGuidance::projectToStreamline3D(
    const Eigen::Vector3d& x) const
{
    StreamlineProj3D out;
    const Streamline3DView& sl = streamline_view_;
    if (!sl.valid || sl.points.size() < 2) return out;
    double best_dist2 = 1e18;
    int best_k = 0;
    double best_s = 0.0;
    Eigen::Vector3d best_p = sl.points.front();
    Eigen::Vector3d best_t = sl.tangents.empty() ? Eigen::Vector3d::UnitX()
                                                 : sl.tangents.front();
    for (size_t k = 0; k + 1 < sl.points.size(); ++k) {
        const Eigen::Vector3d d = sl.points[k + 1] - sl.points[k];
        const double L2 = d.squaredNorm();
        const double s = (L2 > 1e-12)
            ? std::min(1.0, std::max(0.0, (x - sl.points[k]).dot(d) / L2))
            : 0.0;
        const Eigen::Vector3d cand = sl.points[k] + s * d;
        const double dist2 = (x - cand).squaredNorm();
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            best_k = static_cast<int>(k);
            best_s = s;
            best_p = cand;
            if (!sl.tangents.empty()) {
                best_t = sl.tangents[k] * (1.0 - s) + sl.tangents[k + 1] * s;
                if (best_t.norm() > 1e-9) best_t.normalize();
            }
        }
    }
    out.s = best_s;
    out.p = best_p;
    out.t = best_t;
    out.end_cap = (best_k == 0) || (best_k == static_cast<int>(sl.points.size()) - 2);
    return out;
}

Eigen::Vector3d FluidGuidance::calcStreamlineGuidance3D(const Eigen::Vector3d& pos,
                                                        double v_cap,
                                                        double D_local,
                                                        double w_robot,
                                                        double heading_rad)
{
    (void)w_robot;
    (void)heading_rad;
    const auto started = std::chrono::steady_clock::now();
    const double v_t = v_cap * std::max(c_.speed_floor,
        fluid2d::fluidSpeedRatio(D_local, c_.d_s, c_.d_drag));
    flow_error_diag_.control = flow_coordinates_.nominalVelocity(
        pos, v_t, c_.k_n * Eigen::Matrix2d::Identity());
    flow_error_diag_.control_compute_ms = wallMs(started, std::chrono::steady_clock::now());
    return flow_error_diag_.control.velocity;
}

}  // namespace fluid
}  // namespace FLAG_Race
