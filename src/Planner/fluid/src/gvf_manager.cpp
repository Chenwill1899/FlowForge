#include "fluid/gvf_manager.h"

namespace FLAG_Race
{
    gvf_manager::gvf_manager(ros::NodeHandle &nh)
    {
        nh.param<std::string>("gvf/cloud_topic", cloud_topic_, "click_map");
        nh.param<std::string>("gvf/odom_topic", odom_topic_, "odom");
        nh.param<std::string>("gvf/cmd_topic", cmd_topic_, "/drone_1_planning/pos_cmd");

        nh.param("gvf/human_input_enable", human_input_enable_, false);
        nh.param<std::string>("gvf/human_intent_topic", human_intent_topic_, "/human_intent");
        nh.param("gvf/human_intent_timeout", human_intent_timeout_, 0.5);
        nh.param("gvf/human_intent_max_speed", human_intent_max_speed_, 1.0);

        double fluid_solid_clearance = 0.30;
        nh.param("gvf/fluid_d_s", fluid_solid_clearance, 0.30);
        nh.param("gvf/safety_min_clearance", safety_min_clearance_, 0.30);
        fluid_solid_clearance = std::max(0.0, fluid_solid_clearance);
        if (safety_min_clearance_ < fluid_solid_clearance) {
            ROS_WARN("[GVF][SAFETY] safety_min_clearance %.3f is inside fluid_d_s %.3f; "
                     "raising it to the Darcy solid boundary",
                     safety_min_clearance_, fluid_solid_clearance);
            safety_min_clearance_ = fluid_solid_clearance;
        }
        nh.param("gvf/safety_brake_decel", safety_brake_decel_, 1.5);
        nh.param("gvf/fluid_safety_escape_speed_ratio", fluid_safety_escape_speed_ratio_, 0.25);
        nh.param("gvf/cbf_gain", cbf_gain_, 1.5);
        nh.param("gvf/safety_esdf_max_age", safety_esdf_max_age_, 0.5);
        cbf_gain_ = std::max(0.0, cbf_gain_);
        safety_esdf_max_age_ = std::max(0.05, safety_esdf_max_age_);

        nh.param("gvf/fluid_solver_3d", fluid_solver_3d_, false);
        nh.param("gvf/fluid_3d_w_max", fluid_3d_w_max_, 1.2);
        fluid_3d_w_max_ = std::max(0.0, fluid_3d_w_max_);
        nh.param("gvf/fluid_3d_cmd_accel_xy_max", fluid_3d_cmd_accel_xy_max_, 4.0);
        nh.param("gvf/fluid_3d_cmd_accel_z_max", fluid_3d_cmd_accel_z_max_, 2.5);
        fluid_3d_cmd_accel_xy_max_ = std::max(0.0, fluid_3d_cmd_accel_xy_max_);
        fluid_3d_cmd_accel_z_max_ = std::max(0.0, fluid_3d_cmd_accel_z_max_);
        fluid_3d_cmd_filter_last_time_ = ros::Time(0);

        nh.param("gvf/cmd/vel_max", cmd_vel_max_, 1.5);
        nh.param("gvf/odom_vel_est_window", odom_vel_est_window_, 0.3);
        nh.param("gvf/odom_vel_lpf_hz", odom_vel_lpf_hz_, 2.0);

        initCallback(nh);
        InitGvf(nh);
    }
    gvf_manager::~gvf_manager() {}

    void gvf_manager::initCallback(ros::NodeHandle &nh)
    {
        if (human_input_enable_)
        {
            human_intent_sub_ = nh.subscribe(
                human_intent_topic_, 10, &gvf_manager::humanIntentCallback, this);
            ROS_INFO("[GVF] human input enabled: %s, timeout=%.2fs, max_speed=%.2fm/s",
                     human_intent_topic_.c_str(), human_intent_timeout_, human_intent_max_speed_);
        }
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &gvf_manager::odomCallback, this);
        cmd_pub    = nh.advertise<quadrotor_msgs::PositionCommand>(cmd_topic_, 10);
        cmd_timer  = nh.createTimer(ros::Duration(0.02), &gvf_manager::cmdCallback, this);  // 50Hz

        field_diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
            "/paper/gvf_field_diagnostics", 10);
        reanchor_event_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
            "/paper/gvf_reanchor_events", 10);
    }

    void gvf_manager::humanIntentCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        Eigen::Vector3d velocity(
            msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z);

        if (!velocity.allFinite())
        {
            ROS_WARN_THROTTLE(1.0, "[GVF] ignoring non-finite human intent");
            velocity.setZero();
        }
        else
        {
            const double max_speed = std::max(0.0, human_intent_max_speed_);
            const double speed = velocity.head<2>().norm();
            if (max_speed > 1e-6 && speed > max_speed)
            {
                velocity.head<2>() *= max_speed / speed;
            }
            // The current human_input_sim interface controls planar motion only.
            velocity.z() = 0.0;
        }

        const ros::Time now = ros::Time::now();
        Eigen::Vector3d stop_pos = Eigen::Vector3d::Zero();
        double stop_yaw = 0.0;
        bool publish_deadman_stop = false;
        {
            std::lock_guard<std::mutex> input_guard(human_fluid_input_mutex_);
            human_intent_velocity_ = velocity;
            human_intent_last_time_ = now;
            human_intent_received_ = true;
            publish_deadman_stop = human_input_enable_ && human_fluid_have_odom_ &&
                velocity.head<2>().norm() < fluid_intent_speed_deadband_;
            stop_pos = human_fluid_odom_;
            stop_yaw = human_fluid_yaw_;
        }

        // Intent release stops immediately in BOTH modes.  The 3-D slew limiter
        // alone would ramp the last command to zero over ~0.4 s, which violates
        // the paper's post-release stop criterion.  Reset the 3-D command filter
        // together with the deadman zero so the next push re-ramps from rest.
        if (publish_deadman_stop && cmd_pub) {
            if (fluid_solver_3d_) {
                resetFluid3DCommandFilter();
            }
            quadrotor_msgs::PositionCommand stop_cmd;
            stop_cmd.header.stamp = now;
            stop_cmd.header.frame_id = "world";
            stop_cmd.position.x = stop_pos.x();
            stop_cmd.position.y = stop_pos.y();
            stop_cmd.position.z = stop_pos.z();
            stop_cmd.velocity.x = 0.0;
            stop_cmd.velocity.y = 0.0;
            stop_cmd.velocity.z = 0.0;
            stop_cmd.acceleration.x = 0.0;
            stop_cmd.acceleration.y = 0.0;
            stop_cmd.acceleration.z = 0.0;
            stop_cmd.yaw = stop_yaw;
            stop_cmd.yaw_dot = 0.0;
            cmd_pub.publish(stop_cmd);
        }
    }

    void gvf_manager::resetFluid3DCommandFilter()
    {
        fluid_3d_cmd_filtered_.setZero();
        fluid_3d_cmd_filter_initialized_ = false;
        fluid_3d_cmd_filter_last_time_ = ros::Time(0);
    }

    void gvf_manager::limitFluid3DCommandRate(const ros::Time& now,
                                               Eigen::Vector3d& v_cmd,
                                               bool command_valid)
    {
        if (!fluid_solver_3d_) return;

        if (!v_cmd.allFinite()) v_cmd.setZero();
        if (!fluid_3d_cmd_filter_initialized_) {
            // Start from rest instead of copying the first field sample.  This
            // avoids a startup/reattachment impulse while retaining the normal
            // command authority after a few control ticks.
            fluid_3d_cmd_filtered_.setZero();
            fluid_3d_cmd_filter_last_time_ = now;
            fluid_3d_cmd_filter_initialized_ = true;
        }

        double dt = (now - fluid_3d_cmd_filter_last_time_).toSec();
        if (!std::isfinite(dt) || dt <= 0.0) dt = 0.02;
        // A slow fluid solve may skip timer ticks.  Do not turn that scheduling
        // gap into an unbounded velocity jump on the next publish.
        dt = std::max(0.002, std::min(0.05, dt));
        fluid_3d_cmd_filter_last_time_ = now;

        Eigen::Vector3d target = command_valid ? v_cmd : Eigen::Vector3d::Zero();
        if (!target.allFinite()) target.setZero();

        Eigen::Vector2d dxy = target.head<2>() - fluid_3d_cmd_filtered_.head<2>();
        const double max_dxy = std::max(0.0, fluid_3d_cmd_accel_xy_max_) * dt;
        const double dxy_norm = dxy.norm();
        if (max_dxy <= 0.0) {
            dxy.setZero();
        } else if (dxy_norm > max_dxy && dxy_norm > 1e-12) {
            dxy *= max_dxy / dxy_norm;
        }

        double dz = target.z() - fluid_3d_cmd_filtered_.z();
        const double max_dz = std::max(0.0, fluid_3d_cmd_accel_z_max_) * dt;
        if (max_dz <= 0.0) dz = 0.0;
        else dz = std::max(-max_dz, std::min(max_dz, dz));

        fluid_3d_cmd_filtered_.head<2>() += dxy;
        fluid_3d_cmd_filtered_.z() += dz;
        v_cmd = fluid_3d_cmd_filtered_;
    }

    void gvf_manager::updateFluidAnchor(const ros::Time& now,
                                        const Eigen::Vector2d& pos_xy,
                                        const Eigen::Vector3d& intent_velocity,
                                        const ros::Time& intent_last_time,
                                        bool intent_received)
    {
        const bool intent_fresh = intent_received &&
            (now - intent_last_time).toSec() <= human_intent_timeout_;
        const double speed = intent_velocity.head<2>().norm();

        if (!intent_fresh || speed < fluid_intent_speed_deadband_) {
            // Joystick released/stale: drop the latch so the next push re-anchors fresh
            // instead of restoring toward a stale line.
            if (fluid_anchor_active_ && !swarmParticlesManager.empty() &&
                swarmParticlesManager[0].gvf_) {
                swarmParticlesManager[0].gvf_->resetFluidEscapeState();
            }
            fluid_safety_escape_side_latched_ = false;
            fluid_safety_escape_tangent_sign_ = 1.0;
            fluid_anchor_active_ = false;
            return;
        }

        const double heading = std::atan2(intent_velocity.y(), intent_velocity.x());

        bool should_anchor = !fluid_anchor_active_;
        if (fluid_anchor_active_) {
            double dh = heading - fluid_anchor_heading_;
            while (dh > M_PI) dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            if (std::abs(dh) > fluid_heading_latch_deg_ * M_PI / 180.0) should_anchor = true;
        }
        if (should_anchor) {
            if (!swarmParticlesManager.empty() && swarmParticlesManager[0].gvf_) {
                swarmParticlesManager[0].gvf_->resetFluidEscapeState();
            }
            fluid_safety_escape_side_latched_ = false;
            fluid_safety_escape_tangent_sign_ = 1.0;
            fluid_anchor_pos_ = pos_xy;
            fluid_anchor_heading_ = heading;
            fluid_anchor_active_ = true;
            ++reanchor_count_;
            std_msgs::Float64MultiArray event;
            event.data = {
                now.toSec(), static_cast<double>(reanchor_count_),
                pos_xy.x(), pos_xy.y(), 0.0, heading
            };
            reanchor_event_pub_.publish(event);
        }
    }

    void gvf_manager::publishFieldDiagnostics(const ros::Time& stamp,
                                              const Eigen::Vector3d& pos,
                                              const Eigen::Vector3d& intent_velocity)
    {
        if (swarmParticlesManager.empty() || !swarmParticlesManager[0].gvf_) return;
        Eigen::Vector3d direction = Eigen::Vector3d::Zero();
        double potential = 0.0;
        bool field_valid = false, potential_valid = false;
        swarmParticlesManager[0].gvf_->getFluidDiagnostics(
            direction, potential, field_valid, potential_valid);
        // checkSafetySupervisor/applyCbfFinal already queried the ESDF in this
        // tick. Reuse that diagnostic value rather than issuing another map query
        // from the telemetry path.
        const double clearance = safety_last_clearance_;
        Eigen::Vector3d streamline_projection, e_perp;
        double eta_xy = std::numeric_limits<double>::quiet_NaN();
        double eta_3d = std::numeric_limits<double>::quiet_NaN();
        bool streamline_valid = false;
        int streamline_mode = 2;
        bool streamline_end_cap = false;
        uint64_t streamline_version = 0;
        double streamline_update_disp = std::numeric_limits<double>::quiet_NaN();
        double streamline_clearance_min = std::numeric_limits<double>::quiet_NaN();
        double projection_clearance = std::numeric_limits<double>::quiet_NaN();
        double d_v = std::numeric_limits<double>::quiet_NaN();
        swarmParticlesManager[0].gvf_->getStreamlineDiagnostics(
            pos, streamline_projection, e_perp, eta_xy, eta_3d, streamline_valid,
            streamline_mode, streamline_end_cap, streamline_version,
            streamline_update_disp, streamline_clearance_min, projection_clearance, d_v);
        std_msgs::Float64MultiArray msg;
        msg.data = {
            stamp.toSec(), pos.x(), pos.y(), pos.z(),
            direction.x(), direction.y(), direction.z(), potential,
            field_valid ? 1.0 : 0.0, potential_valid ? 1.0 : 0.0,
            clearance, intent_velocity.x(), intent_velocity.y(), intent_velocity.z(),
            fluid_anchor_pos_.x(), fluid_anchor_pos_.y(), 0.0,
            fluid_anchor_heading_, static_cast<double>(reanchor_count_),
            std::isfinite(eta_3d) ? eta_3d : -1.0,
            streamline_valid ? 1.0 : 0.0,
            static_cast<double>(streamline_mode),
            streamline_end_cap ? 1.0 : 0.0,
            static_cast<double>(streamline_version), streamline_update_disp,
            streamline_projection.x(), streamline_projection.y(), streamline_projection.z(),
            e_perp.x(), e_perp.y(), e_perp.z(), eta_xy, eta_3d,
            streamline_clearance_min, projection_clearance, d_v
        };
        const auto& flow = swarmParticlesManager[0].gvf_->fluid_.flowErrorDiagnostics3D();
        const auto& error = flow.coordinate;
        msg.data.insert(msg.data.end(), {
            error.valid() ? 1.0 : 0.0, static_cast<double>(error.status),
            static_cast<double>(flow.field_version),
            flow.anchor.x(), flow.anchor.y(), flow.anchor.z(),
            flow.query.x(), flow.query.y(), flow.query.z(),
            error.eta.x(), error.eta.y(), error.eta.norm(),
            error.section_residual, error.arc_length, static_cast<double>(error.steps),
            flow.gradient_relative_error, flow.compute_ms
        });
        const auto& control = flow.control;
        const Eigen::Vector3d delta = flow_final_velocity_ - control.velocity;
        msg.data.insert(msg.data.end(), {
            control.valid() ? 1.0 : 0.0, static_cast<double>(control.status),
            control.minimum_sigma, control.condition, control.fd_difference,
            control.ju_residual, control.inverse_residual, control.decay_residual,
            control.velocity.x(), control.velocity.y(), control.velocity.z(),
            flow_final_velocity_.x(), flow_final_velocity_.y(), flow_final_velocity_.z(),
            delta.x(), delta.y(), delta.z(), flow.control_compute_ms
        });
        field_diagnostics_pub_.publish(msg);
    }

    bool gvf_manager::fetchSafetyDistance(const Eigen::Vector3d& pos,
                                          double& h, Eigen::Vector3d& grad)
    {
        auto& pm = swarmParticlesManager[0];
        if (!pm.sdf_map_) return false;

        double clearance = 0.0;
        Eigen::Vector3d g = Eigen::Vector3d::Zero();
        clearance = pm.sdf_map_->getDistWithGradTrilinear(pos, g);
        // Plausibility bound: the map's distance buffers are initialized to the
        // 10000.0 "unknown" sentinel before the first ESDF update, the EDT can
        // leave ~5000 m artifacts outside the local window (observed live: the
        // 3D benchmark's ESDF simply does not cover the robot's start cell --
        // with garbage gradients ~1e5, the derivative of the sentinel field --
        // while 1 m ahead the data is real), and a corrupt read can return
        // ~1e162. Only |clearance| <= 100 m (the codebase's own plausibility
        // convention, cf. the 2D cache refresh) is VERIFIED data; anything
        // beyond that -- whatever its gradient looks like -- is an unverified
        // region the CBF must never trust.
        if (std::isfinite(clearance) && std::abs(clearance) <= 100.0) {
            // Cache the last verified (h, grad) pair so a later corrupt read can
            // still use the most recent trustworthy geometry for
            // safety_esdf_max_age_ seconds.
            safety_esdf_h_ = clearance;
            safety_esdf_grad_ = g;
            safety_esdf_h_stamp_ = ros::Time::now();
            safety_esdf_h_valid_ = true;
            safety_last_clearance_ = clearance;
        } else if (safety_esdf_h_valid_ &&
                   (ros::Time::now() - safety_esdf_h_stamp_).toSec() <=
                       safety_esdf_max_age_) {
            // Unverified/impossible read: defer to the last verified pair within
            // its age limit. This is NOT "trusting the upstream command": the
            // cached geometry is real, recent, and already validated.
            ROS_WARN_THROTTLE(1.0,
                              "[GVF][SAFETY] unverified ESDF read, using last verified (h,grad) aged %.2fs",
                              (ros::Time::now() - safety_esdf_h_stamp_).toSec());
            clearance = safety_esdf_h_;
            g = safety_esdf_grad_;
        } else {
            // No verified distance at all (never verified, or the cached pair is
            // stale): engineering degrade. The CBF theorem-grade guarantee only
            // extends to regions with verified (h, grad); elsewhere the command
            // passes through unprojected (the fluid D-ramp and escape net stay
            // active), which is documented as a degrade, not as a safety claim
            // (paper VIII domain caveat).
            return false;
        }
        h = clearance - safety_min_clearance_;
        grad = g;
        return true;
    }

    void gvf_manager::applyCbfFinal(const Eigen::Vector3d& pos, Eigen::Vector3d& v_cmd,
                                    double horizontal_cap)
    {
        // In 3-D the actuator limits are deliberately separable: the horizontal
        // command lives in a disk and the vertical command in an interval.  Do
        // this before the map/CBF early returns as well, so a disabled/degraded
        // CBF cannot re-introduce an over-limit command through the safety
        // supervisor.
        const double xy_cap = std::isfinite(horizontal_cap)
            ? std::max(0.0, horizontal_cap) : 0.0;
        const double z_cap = std::max(0.0, fluid_3d_w_max_);
        auto clamp3dActuatorLimits = [&](Eigen::Vector3d& v) {
            if (!v.allFinite()) {
                v.setZero();
                return;
            }
            const double xy_speed = v.head<2>().norm();
            if (xy_speed > xy_cap && xy_speed > 1e-12) {
                v.head<2>() *= xy_cap / xy_speed;
            }
            v.z() = std::max(-z_cap, std::min(z_cap, v.z()));
        };
        if (fluid_solver_3d_) clamp3dActuatorLimits(v_cmd);

        auto& pm = swarmParticlesManager[0];
        if (!pm.sdf_map_) return;
        // cbf_gain <= 0 disables the projection entirely (A/B switch; the legacy
        // braking in checkSafetySupervisor then remains the only safety gate).
        if (cbf_gain_ <= 0.0) return;

        double h = 0.0;
        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        if (!fetchSafetyDistance(pos, h, grad)) {
            // Engineering degrade (paper VIII domain caveat): no verified
            // (h, grad) in this region -- the CBF guarantee does not extend
            // here, so the command passes through unprojected. Logged so the
            // degrade is observable, never silent.
            ROS_WARN_THROTTLE(1.0,
                              "[GVF][SAFETY][CBF] no verified (h,grad): CBF degraded, command passes");
            return;
        }
        const double gnorm = grad.norm();
        if (gnorm < 1e-6) {
            // Flat/degenerate gradient: cannot project. Keep the command only
            // while clear of the boundary; inside it, stop.
            if (h <= 0.0) {
                v_cmd.setZero();
                safety_braking_active_ = true;
            }
            return;
        }

        // Periodic safety telemetry (benchmark/experiment visibility): the
        // verified clearance and the cumulative CBF projection count.
        ROS_INFO_THROTTLE(1.0,
                          "[GVF][SAFETY] clearance=%.3f cbf_projections=%d",
                          h, cbf_projection_count_);

        // Closed-form unit-weight CBF projection (paper VIII, eq. 77).  In 2-D
        // this remains the original unconstrained half-space projection.  In 3-D
        // the projection is bounded by the horizontal disk and vertical interval:
        // move along the CBF gradient, clamping each independent actuator set.
        bool projected = false;
        if (!fluid_solver_3d_) {
            projected = projectCbf(v_cmd, grad, h, cbf_gain_);
        } else {
            clamp3dActuatorLimits(v_cmd);
            const double rhs = -cbf_gain_ * h;
            const double lhs = grad.dot(v_cmd);
            if (lhs < rhs) {
                projected = true;
                const Eigen::Vector2d grad_xy = grad.head<2>();
                const double grad_xy_norm = grad_xy.norm();
                const double grad_z = grad.z();
                const double max_lhs = grad_xy_norm * xy_cap +
                                       std::abs(grad_z) * z_cap;

                if (rhs > max_lhs + 1e-9) {
                    // The CBF half-space and actuator limits have no intersection
                    // at this state.  Keep the command inside both actuator
                    // limits and choose the maximum-clearance direction.
                    if (grad_xy_norm > 1e-9 && xy_cap > 0.0) {
                        v_cmd.head<2>() = xy_cap * grad_xy / grad_xy_norm;
                    } else {
                        v_cmd.head<2>().setZero();
                    }
                    if (std::abs(grad_z) > 1e-9 && z_cap > 0.0) {
                        v_cmd.z() = grad_z > 0.0 ? z_cap : -z_cap;
                    } else {
                        v_cmd.z() = 0.0;
                    }
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[GVF][SAFETY][CBF] bounded 3D caps infeasible: "
                        "rhs=%.3f max=%.3f (xy_cap=%.3f z_cap=%.3f)",
                        rhs, max_lhs, xy_cap, z_cap);
                } else {
                    const Eigen::Vector3d base = v_cmd;
                    auto boundedGradientStep = [&](double lambda) {
                        Eigen::Vector3d trial = base;
                        Eigen::Vector2d xy = base.head<2>() + lambda * grad_xy;
                        const double xy_norm = xy.norm();
                        if (xy_norm > xy_cap && xy_norm > 1e-12) {
                            xy *= xy_cap / xy_norm;
                        }
                        trial.head<2>() = xy;
                        trial.z() = std::max(
                            -z_cap, std::min(z_cap, base.z() + lambda * grad_z));
                        return trial;
                    };

                    double lo = 0.0;
                    double hi = 1.0;
                    while (grad.dot(boundedGradientStep(hi)) < rhs && hi < 1e12) {
                        hi *= 2.0;
                    }
                    for (int iter = 0; iter < 40; ++iter) {
                        const double mid = 0.5 * (lo + hi);
                        if (grad.dot(boundedGradientStep(mid)) >= rhs) {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    v_cmd = boundedGradientStep(hi);
                }
            }
            clamp3dActuatorLimits(v_cmd);
        }
        if (projected) {
            ++cbf_projection_count_;
            safety_braking_active_ = true;
            ROS_INFO_THROTTLE(0.25,
                              "[GVF][SAFETY][CBF] projected h=%.3f |v|=%.3f",
                              h, v_cmd.norm());
        }
        // Keep the independent caps as the final actuator guard.
        if (fluid_solver_3d_) clamp3dActuatorLimits(v_cmd);
        if (!cbfSatisfied(v_cmd, grad, h, cbf_gain_)) {
            ROS_WARN_THROTTLE(1.0,
                              "[GVF][SAFETY][CBF] constraint NOT satisfied post-projection");
        }
        if (!v_cmd.allFinite()) v_cmd.setZero();
    }

    void gvf_manager::checkSafetySupervisor(const Eigen::Vector3d& pos,
                                            const Eigen::Vector3d& intent_velocity,
                                            Eigen::Vector3d& v_cmd)
    {
        auto& pm = swarmParticlesManager[0];
        safety_braking_active_ = false;
        if (!pm.sdf_map_) return;

        double h = 0.0;
        Eigen::Vector3d grad = Eigen::Vector3d::Zero();
        if (!fetchSafetyDistance(pos, h, grad)) {
            // Engineering degrade: no verified distance in this region (map
            // unknown). The supervisor's braking/escape cannot act without a
            // gradient; the command passes through and the fluid guidance
            // remains the active protection. Not a theorem-grade guarantee.
            ROS_WARN_THROTTLE(1.0,
                              "[GVF][SAFETY] no verified distance: supervisor degraded, command passes");
            return;
        }
        const double clearance = h + safety_min_clearance_;

        // The ESDF gradient points toward increasing clearance, i.e. away from the nearest
        // obstacle. We only ever suppress the component of v_cmd that heads *into* the
        // obstacle; tangential and outward motion is always preserved. Otherwise a robot
        // that brakes to a stop at the clearance limit gets its escape command (pointing
        // back out) zeroed too, and stays frozen forever regardless of human input.
        Eigen::Vector3d n = grad;
        const double gnorm = n.norm();
        if (gnorm < 1e-6) {
            // No usable gradient (e.g. outside the map): fall back to a plain hard stop,
            // but only when actually below the hard clearance limit.
            if (clearance <= safety_min_clearance_) {
                v_cmd.setZero();
                safety_braking_active_ = true;
            }
            return;
        }
        n /= gnorm;

        auto addTangentialFluidEscape = [&]() {
            if (!human_input_enable_ || !fluid_anchor_active_) return;
            const Eigen::Vector2d intent_xy = intent_velocity.head<2>();
            const double intent_speed = intent_xy.norm();
            if (intent_speed < fluid_intent_speed_deadband_) return;

            Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
            if (fluid_solver_3d_) {
                // 3D escape: project the current desire (command, then intent,
                // then the anchor heading) onto the plane perpendicular to the
                // wall normal.
                Eigen::Vector3d ref = v_cmd;
                if (ref.norm() < 1e-4) {
                    ref = Eigen::Vector3d(intent_xy.x(), intent_xy.y(), 0.0);
                }
                if (ref.norm() < 1e-4) {
                    ref = Eigen::Vector3d(std::cos(fluid_anchor_heading_),
                                          std::sin(fluid_anchor_heading_), 0.0);
                }
                Eigen::Vector3d proj = ref - ref.dot(n) * n;
                if (proj.norm() < 1e-6) {
                    const Eigen::Vector3d hdir(std::cos(fluid_anchor_heading_),
                                               std::sin(fluid_anchor_heading_), 0.0);
                    proj = hdir - hdir.dot(n) * n;
                    if (proj.norm() < 1e-6) return;
                }
                tangent = proj.normalized();
            } else {
                // `n` is a normalized 3-D ESDF gradient.  Its horizontal projection is
                // not generally unit length, so normalize the wall tangent explicitly.
                Eigen::Vector2d tangent_ccw_xy(-n.y(), n.x());
                const double tangent_norm = tangent_ccw_xy.norm();
                if (tangent_norm < 1e-6) return;
                tangent_ccw_xy /= tangent_norm;
                const Eigen::Vector3d tangent_ccw(tangent_ccw_xy.x(), tangent_ccw_xy.y(), 0.0);

                // Preserve a healthy tangential component already selected by
                // the projected fluid field before consulting the joystick/default
                // lateral convention.
                if (!fluid_safety_escape_side_latched_) {
                    double alignment = v_cmd.head<2>().dot(tangent_ccw_xy);
                    if (std::abs(alignment) < 1e-4) {
                        alignment = intent_xy.dot(tangent_ccw_xy);
                    }
                    if (std::abs(alignment) < 1e-4) {
                        const Eigen::Vector2d anchor_lateral(-std::sin(fluid_anchor_heading_),
                                                               std::cos(fluid_anchor_heading_));
                        alignment = anchor_lateral.dot(tangent_ccw_xy);
                    }
                    fluid_safety_escape_tangent_sign_ = alignment < 0.0 ? -1.0 : 1.0;
                    fluid_safety_escape_side_latched_ = true;
                }

                tangent = fluid_safety_escape_tangent_sign_ * tangent_ccw;
            }
            const double target_speed = std::max(0.0, fluid_safety_escape_speed_ratio_) *
                                        intent_speed;
            const double tangent_speed = v_cmd.dot(tangent);
            // This is a last-line stall recovery, not a second steering controller.
            // Never reverse an existing safe tangential command, and never reduce a
            // healthy component that already exceeds the requested minimum.
            if (tangent_speed < -1e-3) return;

            double supplement = std::max(0.0, target_speed - std::max(0.0, tangent_speed));
            const double ceiling = std::max(0.0, human_intent_max_speed_);
            if (ceiling > 1e-6 && supplement > 0.0) {
                // Limit only the added tangent instead of scaling the whole command,
                // which would unnecessarily weaken an existing outward component.
                const double speed_sq = v_cmd.squaredNorm();
                if (speed_sq >= ceiling * ceiling) {
                    supplement = 0.0;
                } else {
                    const double disc = tangent_speed * tangent_speed +
                                        ceiling * ceiling - speed_sq;
                    const double max_supplement = -tangent_speed +
                        std::sqrt(std::max(0.0, disc));
                    supplement = std::min(supplement, std::max(0.0, max_supplement));
                }
            }
            v_cmd += supplement * tangent;
            ROS_INFO_THROTTLE(0.25,
                              "[GVF][SAFETY] tangential escape clearance=%.3f target=%.2f add=%.2f",
                              clearance, target_speed, supplement);
        };

        const double v_into = -v_cmd.dot(n);   // > 0 means the command is approaching the obstacle
        if (v_into <= 0.0) {
            // The local fluid grid can return exactly zero for an inflated-solid
            // center cell.  There is then no inward component for the normal
            // projection to remove, but keeping that zero would still deadlock
            // the robot on the clearance boundary.
            if (clearance <= safety_min_clearance_ && v_cmd.norm() < 1e-3) {
                addTangentialFluidEscape();
            }
            return;  // moving away or tangential: never brake it
        }

        const double stopping_dist = (v_into * v_into) / (2.0 * std::max(0.05, safety_brake_decel_));
        double scale = 1.0;
        if (clearance <= safety_min_clearance_) {
            scale = 0.0;
        } else if (clearance < stopping_dist + safety_min_clearance_) {
            scale = std::max(0.0, std::min(1.0,
                (clearance - safety_min_clearance_) / std::max(1e-3, stopping_dist)));
        }
        if (scale < 1.0) {
            // Remove only the (1 - scale) fraction of the inward component; v_into*n points
            // outward, so adding it back cancels the corresponding inward velocity.
            v_cmd += (1.0 - scale) * v_into * n;
            safety_braking_active_ = true;
            if (scale <= 1e-6) addTangentialFluidEscape();
        }
    }

    void gvf_manager::cmdCallback(const ros::TimerEvent& event)
    {
        if (!human_input_enable_) return;

        // Timer callbacks share an eight-thread AsyncSpinner.  Skip an
        // overlapping 50 Hz tick rather than queueing a stale command behind a
        // slower Darcy solve; the last published command remains in effect.
        std::unique_lock<std::mutex> fluid_guard(
            human_fluid_cmd_mutex_, std::try_to_lock);
        if (!fluid_guard.owns_lock())
        {
            ROS_WARN_THROTTLE(1.0,
                              "[GVF][FLUID] skipping overlapping command tick");
            return;
        }

        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d intent_velocity = Eigen::Vector3d::Zero();
        ros::Time intent_last_time;
        bool have_odom = false;
        bool intent_received = false;
        {
            std::lock_guard<std::mutex> input_guard(human_fluid_input_mutex_);
            pos = human_fluid_odom_;
            intent_velocity = human_intent_velocity_;
            intent_last_time = human_intent_last_time_;
            have_odom = human_fluid_have_odom_;
            intent_received = human_intent_received_;
        }

        if (!have_odom)
        {
            ROS_WARN_THROTTLE(1.0, "[GVF] human input waiting for odometry");
            return;
        }
        if (swarmParticlesManager.empty())
        {
            ROS_WARN_THROTTLE(1.0, "[GVF] human input waiting for particle init");
            return;
        }

        const ros::Time now = ros::Time::now();

        // gvf::calcFluidGuidance2D/3D evaluate the fluid guidance law directly
        // (fused Darcy field + escape net + streamline tracking) at the robot's
        // live position, replacing the corridor build and the velocity-matching
        // governor: this fused field is itself both the obstacle-avoidance
        // guidance and the final commanded velocity.
        updateFluidAnchor(now, pos.head<2>(), intent_velocity,
                          intent_last_time, intent_received);

        Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
        auto& pm = swarmParticlesManager[0];
        bool command_computed = false;
        if (fluid_anchor_active_ && pm.gvf_)
        {
            if (fluid_solver_3d_) {
                // Full 3D position in, full 3D velocity out; anchor/heading
                // stay planar (intent is planar by design).
                v_cmd = pm.gvf_->calcFluidGuidance3D(
                    pos, fluid_anchor_pos_, fluid_anchor_heading_,
                    intent_velocity.head<2>().norm());
            } else {
                const Eigen::Vector2d v_xy = pm.gvf_->calcFluidGuidance2D(
                    pos.head<2>(), fluid_anchor_pos_, fluid_anchor_heading_,
                    intent_velocity.head<2>().norm());
                v_cmd.head<2>() = v_xy;
            }
            command_computed = true;
        }

        // A Darcy solve can take longer than one control period. Intent and
        // odometry callbacks remain live on the input mutex while it runs, so
        // take a second snapshot before publishing. A release, timeout, or
        // heading re-anchor invalidates the just-computed field command.
        Eigen::Vector3d latest_pos = pos;
        Eigen::Vector3d latest_intent_velocity = intent_velocity;
        ros::Time latest_intent_last_time = intent_last_time;
        double latest_yaw = 0.0;
        bool latest_have_odom = have_odom;
        bool latest_intent_received = intent_received;
        {
            std::lock_guard<std::mutex> input_guard(human_fluid_input_mutex_);
            latest_pos = human_fluid_odom_;
            latest_intent_velocity = human_intent_velocity_;
            latest_intent_last_time = human_intent_last_time_;
            latest_yaw = human_fluid_yaw_;
            latest_have_odom = human_fluid_have_odom_;
            latest_intent_received = human_intent_received_;
        }

        const ros::Time publish_now = ros::Time::now();
        const double latest_speed = latest_intent_velocity.head<2>().norm();
        const bool latest_intent_fresh = latest_intent_received &&
            (publish_now - latest_intent_last_time).toSec() <= human_intent_timeout_;
        bool command_matches_latest = command_computed && latest_have_odom &&
            latest_intent_fresh && latest_speed >= fluid_intent_speed_deadband_;
        if (command_matches_latest) {
            const double latest_heading = std::atan2(latest_intent_velocity.y(),
                                                     latest_intent_velocity.x());
            double dh = latest_heading - fluid_anchor_heading_;
            while (dh > M_PI) dh -= 2.0 * M_PI;
            while (dh < -M_PI) dh += 2.0 * M_PI;
            if (std::abs(dh) > fluid_heading_latch_deg_ * M_PI / 180.0) {
                command_matches_latest = false;
            }
        }

        pos = latest_pos;
        intent_velocity = latest_intent_velocity;
        updateFluidAnchor(publish_now, pos.head<2>(), intent_velocity,
                          latest_intent_last_time, latest_intent_received);
        if (!command_matches_latest) v_cmd.setZero();

        // Clamp the tracking speed to the *current* human intent speed, not just the
        // static configured ceiling: human_intent_velocity_ is already clamped to
        // human_intent_max_speed_ in humanIntentCallback, so using its live magnitude
        // here also gives proportional speed control from stick deflection, and drops
        // to ~0 within one control tick of release.
        const double v_ceiling = std::min(std::max(0.0, human_intent_max_speed_),
                                           intent_velocity.head<2>().norm());
        if (v_ceiling > 1e-6)
        {
            // Horizontal and vertical are clamped separately: the intent
            // ceiling is a planar-stick quantity, and scaling the whole
            // 3-vector by a 2D-norm ratio would mis-scale a nonzero z.
            const double v_sp = v_cmd.head<2>().norm();
            if (v_sp > v_ceiling) v_cmd.head<2>() *= v_ceiling / v_sp;
            if (fluid_solver_3d_) {
                v_cmd.z() = std::max(-fluid_3d_w_max_,
                                     std::min(fluid_3d_w_max_, v_cmd.z()));
            } else {
                v_cmd.z() = 0.0;   // invariant guard: the 2D solver never writes z
            }
        }
        else
        {
            v_cmd.setZero();
        }

        // In 2-D retain the legacy placement.  The 3-D path applies the
        // supervisor after its nominal command slew limit below, so an urgent
        // outward correction is never rate-limited away.
        if (!fluid_solver_3d_) {
            checkSafetySupervisor(pos, intent_velocity, v_cmd);
        }

        // Serialize the final validity check with humanIntentCallback. If a
        // release wins this lock first, this publish is forced to zero; if this
        // publish wins first, the callback's immediate deadman zero follows it.
        {
            std::lock_guard<std::mutex> input_guard(human_fluid_input_mutex_);
            const ros::Time final_now = ros::Time::now();
            const Eigen::Vector3d final_intent = human_intent_velocity_;
            const double final_speed = final_intent.head<2>().norm();
            const bool final_fresh = human_intent_received_ &&
                (final_now - human_intent_last_time_).toSec() <= human_intent_timeout_;
            bool final_command_valid = human_fluid_have_odom_ && final_fresh &&
                final_speed >= fluid_intent_speed_deadband_;
            if (final_command_valid) {
                const double final_heading = std::atan2(final_intent.y(), final_intent.x());
                double dh = final_heading - fluid_anchor_heading_;
                while (dh > M_PI) dh -= 2.0 * M_PI;
                while (dh < -M_PI) dh += 2.0 * M_PI;
                final_command_valid = std::abs(dh) <=
                    fluid_heading_latch_deg_ * M_PI / 180.0;
            }
            // A deadman/release command is normally zero, but CBF may still
            // need to add an outward recovery velocity.  Give that recovery
            // the configured horizontal actuator authority instead of
            // treating the absent stick input as a zero safety cap.
            const double final_horizontal_cap = final_command_valid
                ? std::min(std::max(0.0, human_intent_max_speed_), final_speed)
                : std::max(0.0, human_intent_max_speed_);
            if (!final_command_valid) {
                v_cmd.setZero();
            } else {
                const double final_ceiling = std::min(
                    std::max(0.0, human_intent_max_speed_), final_speed);
                const double speed = v_cmd.head<2>().norm();
                if (speed > final_ceiling && speed > 1e-9) {
                    v_cmd.head<2>() *= final_ceiling / speed;
                }
                // Same split-clamp rationale as the pre-supervisor clamp.
                if (fluid_solver_3d_) {
                    v_cmd.z() = std::max(-fluid_3d_w_max_,
                                         std::min(fluid_3d_w_max_, v_cmd.z()));
                } else {
                    v_cmd.z() = 0.0;
                }
            }
            pos = human_fluid_odom_;
            latest_yaw = human_fluid_yaw_;

            // Smooth the nominal 3-D fluid command before safety supervision.
            // The safety supervisor below may still add an immediate outward
            // correction when clearance is violated; normal commands receive
            // independent horizontal/vertical acceleration bounds.
            limitFluid3DCommandRate(final_now, v_cmd, final_command_valid);

            if (fluid_solver_3d_) {
                checkSafetySupervisor(pos, final_intent, v_cmd);
            }

            // CBF safety projection: the LAST modification before publish,
            // after every speed/actuator clamp, so nothing downstream can
            // re-break grad^T v >= -k_h*h (paper VIII). The deadman-zero path
            // above is included: at h <= 0 the projection may only add an
            // outward recovery component.
            applyCbfFinal(pos, v_cmd, final_horizontal_cap);
            flow_final_velocity_ = v_cmd;

            quadrotor_msgs::PositionCommand cmd;
            cmd.header.stamp = final_now;
            cmd.header.frame_id = "world";
            // Keep the current altitude and horizontal position as the position
            // reference while using velocity as the fluid control input.
            cmd.position.x = pos.x();
            cmd.position.y = pos.y();
            cmd.position.z = pos.z();
            cmd.velocity.x = v_cmd.x();
            cmd.velocity.y = v_cmd.y();
            // position.z stays the odom passthrough above in both modes: the
            // SO3 controller then runs z as a pure first-order velocity
            // servo (kv.z ~= 2.0, tau ~= 0.5 s), exactly how x/y behave.
            cmd.velocity.z = fluid_solver_3d_ ? v_cmd.z() : 0.0;
            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;
            cmd.yaw = latest_yaw;
            cmd.yaw_dot = 0.0;
            publishFieldDiagnostics(final_now, pos, final_intent);
            cmd_pub.publish(cmd);
        }
    }

    void gvf_manager::InitGvf(ros::NodeHandle &nh)
    {
        try {
            std::string particle_base = "/particle0";
            auto sdf_map_ = std::make_shared<SDFMap>();
            sdf_map_->initMap(nh, particle_base, odom_topic_, cloud_topic_);

            auto gvf_ = std::make_shared<gvf>();
            gvf_->init(nh, particle_base, odom_topic_, cloud_topic_);
            // Darcy-fluid guidance (calcFluidGuidance2D/3D) reuses this same
            // already-built ESDF for obstacle distance instead of maintaining
            // its own (gvf's own occupancy/distance buffers are never fed by a
            // cloud subscription and would just be an empty map).
            gvf_->setSdfMap(sdf_map_);

            gvfManager pm {
                particle_base,
                sdf_map_,
                gvf_,
                ros::Time::now(),     // curr_time
                ros::Time(0),         // last_time
                true,
            };

            swarmParticlesManager.push_back(pm);  // 将实例存入向量

            std::cout << "\033[1;33m" << "-----------------------------------------" << "\033[0m" << std::endl;

        } catch (const std::exception& e) {
            ROS_ERROR("Exception caught while initializing environments for %s", e.what());
        }  
    }

    void gvf_manager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_yaw_ = tf::getYaw(msg->pose.pose.orientation);

        Eigen::Vector3d curr_pos(
            msg->pose.pose.position.x + 0.000001,
            msg->pose.pose.position.y + 0.000001,
            msg->pose.pose.position.z);

        ros::Time curr_time = msg->header.stamp;
        if (curr_time.isZero()) {
            curr_time = ros::Time::now();
        }

        odom_pos_history_.push_back({curr_time, curr_pos});
        const double history_keep_time = std::max(odom_vel_est_window_ + 1.0, 1.0);
        while (odom_pos_history_.size() > 2 &&
               (curr_time - odom_pos_history_.front().t).toSec() > history_keep_time)
        {
            odom_pos_history_.pop_front();
        }

        if (odom_pos_history_.size() >= 2)
        {
            const double target_window = std::max(0.0, odom_vel_est_window_);
            const OdomPosSample* old_sample = &odom_pos_history_.front();
            for (const auto& sample : odom_pos_history_)
            {
                if ((curr_time - sample.t).toSec() >= target_window)
                {
                    old_sample = &sample;
                }
                else
                {
                    break;
                }
            }

            const double dt = (curr_time - old_sample->t).toSec();
            if (dt > 1e-3 && dt < 2.0)
            {
                const Eigen::Vector3d raw_vel = (curr_pos - old_sample->p) / dt;
                if (raw_vel.norm() <= 5.0)
                {
                    odom_vel_est_ = raw_vel;

                    if (!odom_vel_initialized_)
                    {
                        odom_vel_lpf_ = raw_vel;
                        odom_vel_initialized_ = true;
                    }
                    else
                    {
                        const double lpf_hz = std::max(0.0, odom_vel_lpf_hz_);
                        double alpha = 1.0;
                        if (lpf_hz > 1e-6)
                        {
                            const double tau = 1.0 / (2.0 * PI * lpf_hz);
                            alpha = dt / (tau + dt);
                        }
                        odom_vel_lpf_ = odom_vel_lpf_ + alpha * (raw_vel - odom_vel_lpf_);
                    }

                    odom_vel_est_ = odom_vel_lpf_;
                }
            }
        }

        last_odom_pos_ = curr_pos;
        last_odom_time_ = curr_time;
        {
            std::lock_guard<std::mutex> input_guard(human_fluid_input_mutex_);
            human_fluid_have_odom_ = true;
            human_fluid_odom_ = curr_pos;
            human_fluid_yaw_ = current_yaw_;
        }
        has_last_odom_ = true;
        this->odom_ = curr_pos;
        if (!swarmParticlesManager.empty()) {
            swarmParticlesManager[0].odom = curr_pos;
        }
    }

}  // namespace FLAG_Race
