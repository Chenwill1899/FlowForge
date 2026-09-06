#include "fluid/gvf.h"

namespace FLAG_Race
{
void gvf::setSdfMap(const std::shared_ptr<SDFMap>& m)
{
    fluid_.setDistanceQuery([m](const Eigen::Vector3d& p) -> double {
        return m ? m->getDistance(p) : 0.0;
    });
}

void gvf::init(ros::NodeHandle& nh, const std::string& particle,
               const std::string& odom, const std::string& cloud)
{
    // frame + mode flags
    nh.param("sdf_map/frame_id", frame_id_, std::string("world"));
    nh.param("gvf/human_input_enable", fluid_human_input_mode_, false);

    fluid::FluidGuidanceConfig cfg;

    // ---- shared / 2D ----
    nh.param("gvf/fluid_cruise_z", cfg.cruise_z, 1.0);
    nh.param("gvf/fluid_window_forward_size", cfg.window_forward_size, 12.0);
    nh.param("gvf/fluid_window_lateral_size", cfg.window_lateral_size, 10.0);
    nh.param("gvf/fluid_window_rear_margin", cfg.window_rear_margin, 0.30);
    nh.param("gvf/fluid_grid_resolution", cfg.grid_resolution, 0.15);
    nh.param("gvf/fluid_d_s", cfg.d_s, 0.3);
    nh.param("gvf/fluid_d_drag", cfg.d_drag, 0.6);
    nh.param("gvf/fluid_d_turn", cfg.d_turn, 0.9);
    nh.param("gvf/fluid_d_look", cfg.d_look, 2.5);
    nh.param("gvf/fluid_delta_theta_max_rad", cfg.delta_theta_max_rad, 0.6);
    nh.param("gvf/fluid_use_regression", cfg.use_regression, true);
    nh.param("gvf/fluid_conv_gate_margin", cfg.conv_gate_margin, 1.0);
    nh.param("gvf/fluid_conv_length", cfg.conv_length, 1.5);
    nh.param("gvf/fluid_bypass_offset", cfg.bypass_offset, 1.0);
    nh.param("gvf/fluid_default_bias_sign", cfg.default_bias_sign, 1.0);
    nh.param("gvf/fluid_side_deadband", cfg.side_deadband, 0.05);
    nh.param("gvf/fluid_speed_floor", cfg.speed_floor, 0.15);
    nh.param("gvf/fluid_escape_tangent_ratio", cfg.escape_tangent_ratio, 0.50);
    nh.param("gvf/fluid_escape_outward_ratio", cfg.escape_outward_ratio, 0.15);
    nh.param("gvf/fluid_escape_stall_lo_ratio", cfg.escape_stall_lo_ratio, 0.15);
    nh.param("gvf/fluid_escape_stall_hi_ratio", cfg.escape_stall_hi_ratio, 0.50);
    nh.param("gvf/fluid_escape_solid_weight", cfg.escape_solid_weight, 0.60);
    nh.param("gvf/fluid_resolve_period", cfg.resolve_period, 0.1);
    nh.param("gvf/fluid_cg_tolerance", cfg.cg_tolerance, 1e-5);
    nh.param("gvf/fluid_cg_max_iterations", cfg.cg_max_iterations, 200);
    nh.param("gvf/fluid_coarse_grid_resolution", cfg.coarse_grid_resolution, 0.30);
    nh.param("gvf/fluid_fine_forward_size", cfg.fine_forward_size, 6.0);
    nh.param("gvf/fluid_fine_lateral_size", cfg.fine_lateral_size, 6.0);
    nh.param("gvf/fluid_use_coarse", cfg.use_coarse, true);
    nh.param("gvf/fluid_use_fine", cfg.use_fine, true);
    nh.param("gvf/fluid_psi_error_gain", cfg.psi_error_gain, 1.0);
    nh.param("gvf/fluid_cg_residual_acceptance", cfg.cg_residual_acceptance, 10.0);
    nh.param("gvf/fluid_side_match_centroid_dist", cfg.side_match_centroid_dist, 1.0);
    nh.param("gvf/fluid_side_latch_ttl", cfg.side_latch_ttl, 2.0);
    nh.param("gvf/fluid_side_match_lateral_tol", cfg.side_match_lateral_tol, 0.35);
    nh.param("gvf/fluid_side_match_forward_tol", cfg.side_match_forward_tol, 0.35);

    // ---- 3D ----
    nh.param("gvf/fluid_solver_3d", fluid3d_enabled_, false);
    nh.param("gvf/fluid_solver_3d", cfg.solver_3d, false);
    nh.param("gvf/fluid_3d_fine_resolution", cfg.fine_resolution, 0.20);
    nh.param("gvf/fluid_3d_grid_resolution_z", cfg.grid_resolution_z, 0.15);
    nh.param("gvf/fluid_3d_coarse_resolution_z", cfg.coarse_resolution_z, 0.30);
    nh.param("gvf/fluid_3d_z_min", cfg.z_min, 0.2);
    nh.param("gvf/fluid_3d_z_max", cfg.z_max, 2.6);
    nh.param("gvf/fluid_3d_floor_band", cfg.floor_band, 0.30);
    nh.param("gvf/fluid_3d_ceil_band", cfg.ceil_band, 0.30);
    nh.param("gvf/fluid_3d_crossflow_ratio", cfg.crossflow_ratio, 0.5);
    nh.param("gvf/fluid_3d_stall_speed_ratio", cfg.stall_speed_ratio, 0.35);
    nh.param("gvf/fluid_3d_stall_w_veto", cfg.stall_w_veto, 0.25);
    nh.param("gvf/fluid_3d_stall_release_ratio", cfg.stall_release_ratio, 0.6);
    nh.param("gvf/fluid_3d_stall_clearance", cfg.stall_clearance, 1.2);
    nh.param("gvf/fluid_3d_stall_probe", cfg.stall_probe, 1.2);
    nh.param("gvf/fluid_3d_stall_latch_time", cfg.stall_latch_time, 0.4);
    nh.param("gvf/fluid_3d_stall_release_time", cfg.stall_release_time, 0.8);
    nh.param("gvf/fluid_3d_alt_gain", cfg.alt_gain, 0.8);
    nh.param("gvf/fluid_3d_alt_rate_max", cfg.alt_rate_max, 0.4);
    nh.param("gvf/fluid_3d_alt_gate_lo", cfg.alt_gate_lo, 0.9);
    nh.param("gvf/fluid_3d_alt_gate_hi", cfg.alt_gate_hi, 1.5);
    nh.param("gvf/fluid_3d_vertical_preview_len", cfg.vertical_preview_len, 3.0);
    nh.param("gvf/fluid_3d_vertical_preview_clearance", cfg.vertical_preview_clearance, 0.45);
    nh.param("gvf/fluid_3d_vertical_preview_rate", cfg.vertical_preview_rate, 0.6);
    nh.param("gvf/fluid_3d_k_n", cfg.k_n, 1.0);
    nh.param("gvf/fluid_3d_k_n_z_ratio", cfg.k_n_z_ratio, 0.0);
    nh.param("gvf/fluid_3d_streamline_fwd_len", cfg.streamline_fwd_len, 4.0);
    nh.param("gvf/fluid_3d_streamline_bwd_len", cfg.streamline_bwd_len, 1.5);
    nh.param("gvf/fluid_3d_streamline_ds", cfg.streamline_ds, 0.1);
    nh.param("gvf/fluid_3d_streamline_max_disp", cfg.streamline_max_disp, 1.5);
    nh.param("gvf/fluid_3d_streamline_degrade_cooldown", cfg.streamline_degrade_cooldown, 0.5);
    nh.param("gvf/fluid_3d_freeze_streamline", cfg.freeze_streamline, false);
    nh.param("gvf/fluid_3d_homing_in_track", cfg.homing_in_track, true);
    nh.param("gvf/fluid_3d_d_v_max", cfg.d_v_max, 1.0);
    nh.param("gvf/fluid_3d_crossflow_tau", cfg.crossflow_tau, 0.5);
    nh.param("gvf/fluid_3d_crossflow_rate_max", cfg.crossflow_rate_max, 1.5);

    fluid_.setConfig(cfg);

    vector_field_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        particle + "/gvf/vector_field", 10);
    // 30 Hz animation-only repaint of the cached 3D quiver frame.
    quiver_anim_timer_ = nh.createTimer(ros::Duration(1.0 / 30.0),
                                        &gvf::quiverAnimCallback, this);
}

Eigen::Vector2d gvf::calcFluidGuidance2D(const Eigen::Vector2d& pos_xy,
                                         const Eigen::Vector2d& anchor_xy,
                                         double heading_rad,
                                         double v_J)
{
    const double now_sec = ros::Time::now().toSec();
    const bool build_viz = vector_field_pub_.getNumSubscribers() > 0;
    const Eigen::Vector2d cmd = fluid_.calcGuidance2D(
        pos_xy, anchor_xy, heading_rad, v_J, now_sec, build_viz);
    if (build_viz) renderFluidVis2D();
    return cmd;
}

Eigen::Vector3d gvf::calcFluidGuidance3D(const Eigen::Vector3d& pos,
                                         const Eigen::Vector2d& anchor_xy,
                                         double heading_rad,
                                         double v_J)
{
    const double now_sec = ros::Time::now().toSec();
    const bool build_viz = vector_field_pub_.getNumSubscribers() > 0;
    const Eigen::Vector3d cmd = fluid_.calcGuidance3D(
        pos, anchor_xy, heading_rad, v_J, now_sec, build_viz);
    if (build_viz) {
        const fluid::FluidVis3D& s = fluid_.vis3D();
        if (s.valid) {
            buildQuiverFrame3D(s.grid, s.D, s.solid, s.field,
                               s.grid_coarse, s.solid_coarse, s.field_coarse,
                               s.pos, s.anchor_xy, s.heading_rad, s.v_cap);
            renderQuiverFrame3D();
        }
    }
    return cmd;
}

void gvf::renderFluidVis2D()
{
    const fluid::FluidVis2D& s = fluid_.vis2D();
    if (!s.valid) return;
    if (s.coarse_style) {
        // Coarse-only ablation: fused command field on the full-reach grid.
        publishFluidQuiver(s.origin, s.e_J, s.n_J, s.h, s.n_forward, s.n_lateral,
                           s.Ux, s.Uy, s.solid, Eigen::Vector2d::Zero(),
                           "fluid_quiver_coarse", /*prior_style=*/true);
        publishFluidStreamlines2D(s.origin, s.e_J, s.n_J, s.h,
                                  s.n_forward, s.n_lateral,
                                  s.Ux, s.Uy, s.solid,
                                  "fluid_quiver_coarse", /*coarse_style=*/true);
        return;
    }
    // Fine level: fused command field (foreground).
    publishFluidQuiver(s.origin, s.e_J, s.n_J, s.h, s.n_forward, s.n_lateral,
                       s.Ux, s.Uy, s.solid, Eigen::Vector2d::Zero());
    // Coarse context, cut out the fine window.
    if (s.coarse_valid) {
        publishFluidQuiver(s.coarse_origin, s.e_J, s.n_J, s.coarse_h,
                           s.coarse_n_forward, s.coarse_n_lateral,
                           s.coarse_Ux, s.coarse_Uy, s.coarse_solid,
                           Eigen::Vector2d::Zero(),
                           "fluid_quiver_coarse", /*prior_style=*/true,
                           s.origin, s.n_forward * s.h, s.n_lateral * s.h);
        publishFluidStreamlines2D(
            s.coarse_origin, s.e_J, s.n_J, s.coarse_h,
            s.coarse_n_forward, s.coarse_n_lateral,
            s.coarse_Ux, s.coarse_Uy, s.coarse_solid,
            "fluid_quiver_coarse", /*coarse_style=*/true, -1.0,
            s.origin, s.n_forward * s.h, s.n_lateral * s.h);
    }
    // Fine streamlines from the fused command field.
    publishFluidStreamlines2D(s.origin, s.e_J, s.n_J, s.h,
                              s.n_forward, s.n_lateral,
                              s.Ux, s.Uy, s.solid,
                              "fluid_quiver", /*coarse_style=*/false);
}

void gvf::publishFluidQuiver(const Eigen::Vector2d& origin,
                             const Eigen::Vector2d& e_J,
                             const Eigen::Vector2d& n_J,
                             double h, int n_forward, int n_lateral,
                             const Eigen::MatrixXd& Ux, const Eigen::MatrixXd& Uy,
                             const std::vector<std::vector<bool>>& solid,
                             const Eigen::Vector2d& display_bias,
                             const std::string& ns,
                             bool prior_style,
                             const Eigen::Vector2d& mask_origin,
                             double mask_forward_len,
                             double mask_lateral_len,
                             double draw_z)
{
    const bool has_mask = mask_forward_len > 0.0 && mask_lateral_len > 0.0;
    const double z_draw = draw_z < 0.0 ? fluid_.config().cruise_z : draw_z;
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker lines;
    lines.header.frame_id = frame_id_;
    lines.header.stamp = ros::Time::now();
    lines.ns = ns;
    lines.id = 0;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = prior_style ? 0.012 : 0.02;
    lines.color.a = 0.8;

    const int skip_forward = std::max(1, n_forward / 25) * (prior_style ? 2 : 1);
    const int skip_lateral = std::max(1, n_lateral / 25) * (prior_style ? 2 : 1);
    const double arrow_scale = 0.2;
    const double max_shaft = 0.7 * h * std::min(skip_forward, skip_lateral);

    for (int i = 0; i < n_forward; i += skip_forward) {
        for (int j = 0; j < n_lateral; j += skip_lateral) {
            if (solid[i][j]) continue;
            const Eigen::Vector2d world = fluid2d::fluidCellWorld(origin, e_J, n_J, h, i, j);
            if (has_mask) {
                const Eigen::Vector2d rel = world - mask_origin;
                const double s = rel.dot(e_J);
                const double t = rel.dot(n_J);
                if (s >= 0.0 && s <= mask_forward_len && t >= 0.0 && t <= mask_lateral_len)
                    continue;
            }
            const double ux = Ux(i, j) + display_bias.x();
            const double uy = Uy(i, j) + display_bias.y();
            const double speed = std::hypot(ux, uy);
            const double shaft = std::min(speed * arrow_scale, max_shaft);
            const double kx = speed > 1e-6 ? ux / speed * shaft : 0.0;
            const double ky = speed > 1e-6 ? uy / speed * shaft : 0.0;

            geometry_msgs::Point p0, p1;
            p0.x = world.x(); p0.y = world.y(); p0.z = z_draw;
            p1.x = world.x() + kx;
            p1.y = world.y() + ky;
            p1.z = z_draw;

            std_msgs::ColorRGBA color;
            if (prior_style) {
                color.r = 0.25; color.g = 0.45; color.b = 0.95; color.a = 0.45;
            } else {
                color.r = std::min(1.0, speed / 2.0);
                color.g = std::max(0.0, 1.0 - speed / 2.0);
                color.b = 0.2;
                color.a = 0.85;
            }

            double lat_w = 1.0;
            if (!prior_style) {
                const double half_w = 0.5 * n_lateral * h;
                const double d_mid = (world - origin).dot(n_J) - half_w;
                const double sigma = 0.45 * half_w;
                lat_w = 0.35 + 0.65 * std::exp(-0.5 * (d_mid / sigma) * (d_mid / sigma));
            }

            if (prior_style) {
                std_msgs::ColorRGBA tip = color;
                std_msgs::ColorRGBA tail = color;
                tail.a = color.a * 0.18;
                lines.points.push_back(p0);
                lines.points.push_back(p1);
                lines.colors.push_back(tail);
                lines.colors.push_back(tip);
            } else {
                const double wavelength = 1.0;
                const double wave_speed = 2.5;
                const double s_axis = (world - origin).dot(e_J);
                const double phase = s_axis / wavelength -
                    wave_speed / wavelength * lines.header.stamp.toSec();
                const double c_pos = phase - std::floor(phase);

                auto pulse_alpha = [&](double u) {
                    double d = u - c_pos;
                    d -= std::round(d);
                    const double g = std::exp(-0.5 * (d / 0.14) * (d / 0.14));
                    return static_cast<float>(color.a * lat_w * (0.10 + 0.90 * g));
                };
                const int n_sub = 6;
                for (int q = 0; q < n_sub; ++q) {
                    const double u0 = static_cast<double>(q) / n_sub;
                    const double u1 = static_cast<double>(q + 1) / n_sub;
                    geometry_msgs::Point a = p0, b = p0;
                    a.x = p0.x + (p1.x - p0.x) * u0;
                    a.y = p0.y + (p1.y - p0.y) * u0;
                    b.x = p0.x + (p1.x - p0.x) * u1;
                    b.y = p0.y + (p1.y - p0.y) * u1;
                    std_msgs::ColorRGBA ca = color, cb = color;
                    ca.a = pulse_alpha(u0);
                    cb.a = pulse_alpha(u1);
                    lines.points.push_back(a);
                    lines.points.push_back(b);
                    lines.colors.push_back(ca);
                    lines.colors.push_back(cb);
                }
            }

            const double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
            if (seg_len > 1e-3) {
                const double dx = (p1.x - p0.x) / seg_len;
                const double dy = (p1.y - p0.y) / seg_len;
                const double head = std::min(0.35 * seg_len, prior_style ? 0.06 : 0.10);
                const double c = 0.866, s = 0.5;
                geometry_msgs::Point bl = p1, br = p1;
                bl.x -= head * (c * dx - s * dy);
                bl.y -= head * (c * dy + s * dx);
                br.x -= head * (c * dx + s * dy);
                br.y -= head * (c * dy - s * dx);
                std_msgs::ColorRGBA head_col = color;
                head_col.a = static_cast<float>(color.a * lat_w * (prior_style ? 1.0 : 0.85));
                lines.points.push_back(p1); lines.points.push_back(bl);
                lines.points.push_back(p1); lines.points.push_back(br);
                for (int k = 0; k < 4; ++k) lines.colors.push_back(head_col);
            }
        }
    }

    marker_array.markers.push_back(lines);
    vector_field_pub_.publish(marker_array);
}

void gvf::publishFluidStreamlines2D(const Eigen::Vector2d& origin,
                                    const Eigen::Vector2d& e_J,
                                    const Eigen::Vector2d& n_J,
                                    double h, int n_forward, int n_lateral,
                                    const Eigen::MatrixXd& Ux,
                                    const Eigen::MatrixXd& Uy,
                                    const std::vector<std::vector<bool>>& solid,
                                    const std::string& ns,
                                    bool coarse_style,
                                    double draw_z,
                                    const Eigen::Vector2d& mask_origin,
                                    double mask_forward_len,
                                    double mask_lateral_len)
{
    const double z = draw_z < 0.0 ? fluid_.config().cruise_z : draw_z;
    const bool has_mask = mask_forward_len > 0.0 && mask_lateral_len > 0.0;

    visualization_msgs::Marker stream;
    stream.header.frame_id = frame_id_;
    stream.header.stamp = ros::Time::now();
    stream.ns = ns + "_streamlines";
    stream.id = 1;
    stream.type = visualization_msgs::Marker::LINE_LIST;
    stream.action = visualization_msgs::Marker::ADD;
    stream.pose.orientation.w = 1.0;
    stream.scale.x = coarse_style ? 0.02 : 0.035;

    const double t_now = stream.header.stamp.toSec();
    const double dash_wavelength = 0.8;
    const double dash_speed = 2.5;

    auto sample_uv = [&](const Eigen::Vector2d& w, Eigen::Vector2d& uv) {
        const Eigen::Vector2d rel = w - origin;
        const double gi = rel.dot(e_J) / h - 0.5;
        const double gj = rel.dot(n_J) / h - 0.5;
        if (gi < 0.0 || gj < 0.0 ||
            gi > n_forward - 1.001 || gj > n_lateral - 1.001) return false;
        const int i0 = static_cast<int>(gi), j0 = static_cast<int>(gj);
        if (solid[i0][j0] || solid[i0 + 1][j0] ||
            solid[i0][j0 + 1] || solid[i0 + 1][j0 + 1]) return false;
        const double fi = gi - i0, fj = gj - j0;
        uv.x() = (1 - fi) * (1 - fj) * Ux(i0, j0) + fi * (1 - fj) * Ux(i0 + 1, j0) +
                 (1 - fi) * fj * Ux(i0, j0 + 1) + fi * fj * Ux(i0 + 1, j0 + 1);
        uv.y() = (1 - fi) * (1 - fj) * Uy(i0, j0) + fi * (1 - fj) * Uy(i0 + 1, j0) +
                 (1 - fi) * fj * Uy(i0, j0 + 1) + fi * fj * Uy(i0 + 1, j0 + 1);
        return true;
    };
    auto dash_alpha = [&](double arc) {
        double ph = arc / dash_wavelength - dash_speed * t_now / dash_wavelength;
        ph -= std::floor(ph);
        const double d = ph - 0.5;
        const double g = std::exp(-0.5 * (d / 0.18) * (d / 0.18));
        return static_cast<float>(coarse_style ? 0.22 + 0.42 * g
                                               : 0.32 + 0.52 * g);
    };

    const int n_seeds = coarse_style ? 18 : 14;
    const int max_steps = 3 * n_forward;
    const double step = 0.5 * h;
    for (int k = 0; k < n_seeds; ++k) {
        const double t_lat = (k + 0.5) * (n_lateral * h) / n_seeds;
        Eigen::Vector2d pos = origin + 0.6 * h * e_J + t_lat * n_J;
        Eigen::Vector2d uv;
        if (!sample_uv(pos, uv)) continue;
        double arc = 0.0;
        for (int st = 0; st < max_steps; ++st) {
            const double spd = uv.norm();
            if (spd < 1e-4) break;
            Eigen::Vector2d dir = uv / spd;
            Eigen::Vector2d uv_mid;
            if (sample_uv(pos + 0.5 * step * dir, uv_mid) && uv_mid.norm() > 1e-4)
                dir = uv_mid.normalized();
            const Eigen::Vector2d nxt = pos + step * dir;
            if (!sample_uv(nxt, uv)) break;
            bool masked = false;
            if (has_mask) {
                const Eigen::Vector2d rel = nxt - mask_origin;
                const double s = rel.dot(e_J);
                const double t = rel.dot(n_J);
                if (s >= 0.0 && s <= mask_forward_len &&
                    t >= 0.0 && t <= mask_lateral_len) masked = true;
            }
            if (!masked) {
                geometry_msgs::Point a, b;
                a.x = pos.x(); a.y = pos.y(); a.z = z - 0.03;
                b.x = nxt.x(); b.y = nxt.y(); b.z = z - 0.03;
                std_msgs::ColorRGBA ca;
                if (coarse_style) {
                    ca.r = 0.95; ca.g = 0.62; ca.b = 0.10;
                } else {
                    ca.r = 1.0; ca.g = 0.55; ca.b = 0.05;
                }
                std_msgs::ColorRGBA cb = ca;
                ca.a = dash_alpha(arc);
                cb.a = dash_alpha(arc + step);
                stream.points.push_back(a);
                stream.points.push_back(b);
                stream.colors.push_back(ca);
                stream.colors.push_back(cb);
            }
            pos = nxt;
            arc += step;
        }
    }
    if (!stream.points.empty()) {
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(stream);
        vector_field_pub_.publish(arr);
    }
}

void gvf::buildQuiverFrame3D(const fluid3d::Grid3D& g,
                             const std::vector<double>& D,
                             const std::vector<uint8_t>& solid,
                             const fluid3d::PotentialField3D& field,
                             const fluid3d::Grid3D& g_coarse,
                             const std::vector<uint8_t>& solid_coarse,
                             const fluid3d::PotentialField3D& field_coarse,
                             const Eigen::Vector3d& pos,
                             const Eigen::Vector2d& anchor_xy,
                             double heading_rad, double v_cap)
{
    QuiverFrame3D& f = quiver3d_frame_;
    f.g = g;

    // ---- Horizontal slice at the drone's altitude, fused through the same
    // angle law as the command (RViz shows what the robot is commanded with).
    f.slice_m = std::max(1, std::min(g.n_z - 2,
        static_cast<int>(std::round((pos.z() - g.z_lo) / g.h_z - 0.5))));
    f.draw_z = g.z_lo + (f.slice_m + 0.5) * g.h_z;
    f.Ux_h.resize(g.n_fwd, g.n_lat);
    f.Uy_h.resize(g.n_fwd, g.n_lat);
    f.solid_h.assign(g.n_fwd, std::vector<bool>(g.n_lat, false));
    for (int i = 0; i < g.n_fwd; ++i) {
        for (int j = 0; j < g.n_lat; ++j) {
            const int k = g.idx(i, j, f.slice_m);
            if (solid[k]) {
                f.solid_h[i][j] = true;
                f.Ux_h(i, j) = 0.0;
                f.Uy_h(i, j) = 0.0;
                continue;
            }
            const Eigen::Vector3d cell = g.cellWorld(i, j, f.slice_m);
            const double phi_cell =
                (cell.head<2>() - anchor_xy).dot(g.n_J);
            const Eigen::Vector3d v_cell = fluid_.fuse3D(
                Eigen::Vector3d(field.Ux[k], field.Uy[k], field.Uz[k]),
                true, heading_rad, phi_cell, v_cap, D[k]);
            f.Ux_h(i, j) = v_cell.x();
            f.Uy_h(i, j) = v_cell.y();
        }
    }

    // ---- Vertical (e_J, z) plane through the robot column: raw field's
    // forward/vertical components (where a climb-over arc is visible).
    f.col_j = std::max(1, std::min(g.n_lat - 2,
        static_cast<int>(std::round(0.5 * g.n_lat - 0.5))));
    f.Us_v.resize(g.n_fwd, g.n_z);
    f.Uz_v.resize(g.n_fwd, g.n_z);
    f.solid_v.assign(g.n_fwd, std::vector<bool>(g.n_z, false));
    for (int i = 0; i < g.n_fwd; ++i) {
        for (int m = 0; m < g.n_z; ++m) {
            const int k = g.idx(i, f.col_j, m);
            if (solid[k]) {
                f.solid_v[i][m] = true;
                f.Us_v(i, m) = 0.0;
                f.Uz_v(i, m) = 0.0;
                continue;
            }
            f.Us_v(i, m) = field.Ux[k] * g.e_J.x() + field.Uy[k] * g.e_J.y();
            f.Uz_v(i, m) = field.Uz[k];
        }
    }

    // ---- Blue streamlines on the robot's current horizontal slice.
    f.streamlines.clear();
    auto sample3 = [&](const Eigen::Vector3d& w, Eigen::Vector3d& uvw) {
        const Eigen::Vector2d rel = w.head<2>() - g.origin_xy;
        const double gi = rel.dot(g.e_J) / g.h - 0.5;
        const double gj = rel.dot(g.n_J) / g.h - 0.5;
        const double gm = (w.z() - g.z_lo) / g.h_z - 0.5;
        if (gi < 0.0 || gj < 0.0 || gm < 0.0 ||
            gi > g.n_fwd - 1.001 || gj > g.n_lat - 1.001 || gm > g.n_z - 1.001) {
            return false;
        }
        const int i0 = static_cast<int>(gi);
        const int j0 = static_cast<int>(gj);
        const int m0 = static_cast<int>(gm);
        for (int di = 0; di < 2; ++di)
            for (int dj = 0; dj < 2; ++dj)
                for (int dm = 0; dm < 2; ++dm)
                    if (solid[g.idx(i0 + di, j0 + dj, m0 + dm)]) return false;
        Eigen::Vector3d raw;
        if (!fluid3d::trilinearSampleField3D(field.Ux, field.Uy, field.Uz,
                                             g, w, raw)) {
            return false;
        }
        const double phi = (w.head<2>() - anchor_xy).dot(g.n_J);
        const double D_local = D[g.idx(
            std::min(g.n_fwd - 1, static_cast<int>(std::lround(gi))),
            std::min(g.n_lat - 1, static_cast<int>(std::lround(gj))),
            std::min(g.n_z - 1, static_cast<int>(std::lround(gm))))];
        uvw = fluid_.fuse3D(raw, true, heading_rad, phi, v_cap, D_local);
        return true;
    };
    const int max_steps = 3 * g.n_fwd;
    const double step = 0.5 * g.h;
    const double lat_extent = g.n_lat * g.h;
    const double streamline_z = f.draw_z;
    for (int ks = 1; ks <= 5; ++ks) {
        const Eigen::Vector2d seed_xy = g.origin_xy + 0.6 * g.h * g.e_J +
            (ks / 6.0) * lat_extent * g.n_J;
        Eigen::Vector2d p_xy = seed_xy;
        Eigen::Vector3d p(p_xy.x(), p_xy.y(), streamline_z);
        Eigen::Vector3d uvw;
        if (!sample3(p, uvw)) continue;
        std::vector<Eigen::Vector3d> line;
        line.emplace_back(p_xy.x(), p_xy.y(), streamline_z);
        for (int st = 0; st < max_steps; ++st) {
            const Eigen::Vector2d uv_xy = uvw.head<2>();
            const double spd = uv_xy.norm();
            if (spd < 1e-4) break;
            Eigen::Vector2d dir = uv_xy / spd;

            Eigen::Vector3d uvw_mid;
            const Eigen::Vector3d mid(
                p_xy.x() + 0.5 * step * dir.x(),
                p_xy.y() + 0.5 * step * dir.y(),
                streamline_z);
            if (sample3(mid, uvw_mid) && uvw_mid.head<2>().norm() > 1e-4) {
                dir = uvw_mid.head<2>().normalized();
            }

            const Eigen::Vector2d nxt_xy = p_xy + step * dir;
            const Eigen::Vector3d nxt(nxt_xy.x(), nxt_xy.y(), streamline_z);
            if (!sample3(nxt, uvw)) break;
            line.emplace_back(nxt_xy.x(), nxt_xy.y(), streamline_z);
            p_xy = nxt_xy;
        }
        if (line.size() >= 3) f.streamlines.push_back(std::move(line));
    }

    // ---- Coarse-context slice at the same altitude: raw prior field.
    f.coarse_valid = false;
    if (field_coarse.valid) {
        const fluid3d::Grid3D& gc = g_coarse;
        const int mc = std::max(0, std::min(gc.n_z - 1,
            static_cast<int>(std::round((f.draw_z - gc.z_lo) / gc.h_z - 0.5))));
        f.g_coarse = gc;
        f.Ux_hc.resize(gc.n_fwd, gc.n_lat);
        f.Uy_hc.resize(gc.n_fwd, gc.n_lat);
        f.solid_hc.assign(gc.n_fwd, std::vector<bool>(gc.n_lat, false));
        for (int i = 0; i < gc.n_fwd; ++i) {
            for (int j = 0; j < gc.n_lat; ++j) {
                const int k = gc.idx(i, j, mc);
                if (solid_coarse[k]) {
                    f.solid_hc[i][j] = true;
                    f.Ux_hc(i, j) = 0.0;
                    f.Uy_hc(i, j) = 0.0;
                } else {
                    f.Ux_hc(i, j) = field_coarse.Ux[k];
                    f.Uy_hc(i, j) = field_coarse.Uy[k];
                }
            }
        }
        f.coarse_valid = true;
    }

    f.valid = true;
}

void gvf::renderQuiverFrame3D()
{
    const QuiverFrame3D& f = quiver3d_frame_;
    if (!f.valid) return;

    // Layer 1: fine field at the robot's current height.
    publishFluidQuiver(f.g.origin_xy, f.g.e_J, f.g.n_J, f.g.h,
                       f.g.n_fwd, f.g.n_lat, f.Ux_h, f.Uy_h, f.solid_h,
                       Eigen::Vector2d::Zero(), "fluid_quiver",
                       /*prior_style=*/false, Eigen::Vector2d::Zero(),
                       -1.0, -1.0, f.draw_z);

    // Layer 1b: coarse-context slice, muted, fine window cut out.
    if (f.coarse_valid) {
        publishFluidQuiver(f.g_coarse.origin_xy, f.g_coarse.e_J, f.g_coarse.n_J,
                           f.g_coarse.h, f.g_coarse.n_fwd, f.g_coarse.n_lat,
                           f.Ux_hc, f.Uy_hc, f.solid_hc,
                           Eigen::Vector2d::Zero(), "fluid_quiver_coarse",
                           /*prior_style=*/true, f.g.origin_xy,
                           f.g.n_fwd * f.g.h, f.g.n_lat * f.g.h, f.draw_z);
    }

    visualization_msgs::MarkerArray marker_array;
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker remove_vertical;
    remove_vertical.header.frame_id = frame_id_;
    remove_vertical.header.stamp = stamp;
    remove_vertical.ns = "fluid_quiver_vert";
    remove_vertical.id = 0;
    remove_vertical.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(remove_vertical);

    // Layer 2: cached 3D streamline polylines, deep-gold like the 2D fine ones.
    visualization_msgs::Marker stream;
    stream.header.frame_id = frame_id_;
    stream.header.stamp = stamp;
    stream.ns = "fluid_streamlines_3d";
    stream.id = 2;
    stream.type = visualization_msgs::Marker::LINE_LIST;
    stream.action = visualization_msgs::Marker::ADD;
    stream.pose.orientation.w = 1.0;
    stream.scale.x = 0.035;
    const double t_now = stamp.toSec();
    const double dash_wavelength = 0.8;
    const double dash_speed = 2.5;
    auto dash_alpha = [&](double arc) {
        double ph = arc / dash_wavelength - dash_speed * t_now / dash_wavelength;
        ph -= std::floor(ph);
        const double d = ph - 0.5;
        const double gau = std::exp(-0.5 * (d / 0.18) * (d / 0.18));
        return static_cast<float>(0.12 + 0.45 * gau);
    };
    for (const auto& line : f.streamlines) {
        double arc = 0.0;
        for (size_t idx = 0; idx + 1 < line.size(); ++idx) {
            const double seg = (line[idx + 1] - line[idx]).norm();
            geometry_msgs::Point a, b;
            a.x = line[idx].x(); a.y = line[idx].y(); a.z = line[idx].z();
            b.x = line[idx + 1].x(); b.y = line[idx + 1].y(); b.z = line[idx + 1].z();
            std_msgs::ColorRGBA ca;
            ca.r = 1.0; ca.g = 0.55; ca.b = 0.05;
            std_msgs::ColorRGBA cb = ca;
            ca.a = dash_alpha(arc);
            cb.a = dash_alpha(arc + seg);
            stream.points.push_back(a);
            stream.points.push_back(b);
            stream.colors.push_back(ca);
            stream.colors.push_back(cb);
            arc += seg;
        }
    }
    marker_array.markers.push_back(stream);

    vector_field_pub_.publish(marker_array);
}

void gvf::quiverAnimCallback(const ros::TimerEvent& /*event*/)
{
    if (vector_field_pub_.getNumSubscribers() == 0) return;
    if (fluid3d_enabled_) {
        renderQuiverFrame3D();
        return;
    }
    // 2D quiver is republished by calcFluidGuidance2D every solve; the 30 Hz
    // animator only repaints the 3D cached frame.
}

}  // namespace FLAG_Race
