#include "fluid/fluid_solver_2d.h"

#include <chrono>
#include <queue>

namespace FLAG_Race {
namespace fluid2d {

SolidComponents labelSolidComponents(const std::vector<std::vector<bool>>& solid,
                                     const Eigen::Vector2d& origin,
                                     const Eigen::Vector2d& e_J,
                                     const Eigen::Vector2d& n_J,
                                     double h,
                                     const Eigen::Vector2d& anchor_xy,
                                     int n_forward, int n_lateral)
{
    SolidComponents out;
    out.label = Eigen::MatrixXi::Constant(n_forward, n_lateral, -1);
    constexpr int di[4] = {1, -1, 0, 0};
    constexpr int dj[4] = {0, 0, 1, -1};
    for (int i = 0; i < n_forward; ++i) {
        for (int j = 0; j < n_lateral; ++j) {
            if (!solid[i][j] || out.label(i, j) >= 0) continue;
            const int comp = out.num_components++;
            std::queue<std::pair<int, int>> q;
            q.emplace(i, j);
            out.label(i, j) = comp;
            double sum_x = 0.0, sum_y = 0.0;
            int count = 0;
            double eta_min = 1e18, eta_max = -1e18;
            double s_min = 1e18, s_max = -1e18;
            while (!q.empty()) {
                const auto cell = q.front();
                q.pop();
                const Eigen::Vector2d world = fluidCellWorld(
                    origin, e_J, n_J, h, cell.first, cell.second);
                sum_x += world.x();
                sum_y += world.y();
                ++count;
                const double eta = (world - anchor_xy).dot(n_J);
                eta_min = std::min(eta_min, eta);
                eta_max = std::max(eta_max, eta);
                const double s = (world - origin).dot(e_J);
                s_min = std::min(s_min, s);
                s_max = std::max(s_max, s);
                for (int d = 0; d < 4; ++d) {
                    const int ni = cell.first + di[d];
                    const int nj = cell.second + dj[d];
                    if (ni < 0 || ni >= n_forward || nj < 0 || nj >= n_lateral) continue;
                    if (!solid[ni][nj] || out.label(ni, nj) >= 0) continue;
                    out.label(ni, nj) = comp;
                    q.emplace(ni, nj);
                }
            }
            out.centroid_world.emplace_back(sum_x / count, sum_y / count);
            out.eta_min.push_back(eta_min);
            out.eta_max.push_back(eta_max);
            out.s_min.push_back(s_min);
            out.s_max.push_back(s_max);
        }
    }
    return out;
}

FluidHarmonicField solveHarmonicStreamField(
    const std::vector<std::vector<bool>>& solid,
    const SolidComponents& comps,
    const Eigen::Vector2d& origin,
    const Eigen::Vector2d& e_J,
    const Eigen::Vector2d& n_J,
    double h, int n_forward, int n_lateral,
    const Eigen::Vector2d& anchor_xy,
    double v_cap,
    const std::vector<double>& component_side,
    double bypass_offset, double default_bias_sign,
    double side_deadband,
    double cg_tolerance, int cg_max_iterations,
    double residual_acceptance,
    const FluidCoarsePrior* prior)
{
    FluidHarmonicField out;
    const int N = n_forward * n_lateral;
    auto idx = [&](int i, int j) { return i * n_lateral + j; };
    auto psi_inf = [&](double x, double y) {
        return v_cap * ((x - anchor_xy.x()) * n_J.x() + (y - anchor_xy.y()) * n_J.y());
    };
    // Anchor gauge: the reference streamline is fixed at the anchor (which
    // lies on the joystick centerline, so psi_inf(anchor) == 0). Using the
    // robot's live position here would let the implicit circulation Gamma
    // drift as the robot moves laterally between rebuilds -- exactly the
    // closed-loop self-motion of Gamma the paper's anchored reference
    // eliminates.
    const double psi_anchor = psi_inf(anchor_xy.x(), anchor_xy.y());
    const double eta_reference = 0.0;   // side decisions measured from the centerline

    std::vector<double> psi_component(std::max(1, comps.num_components), 0.0);
    out.chosen_sides.assign(comps.num_components, default_bias_sign);
    for (int c = 0; c < comps.num_components; ++c) {
        // A solid component spanning both flow-wise edges is a local boundary
        // whose end is not visible in this window (for example a corridor
        // wall), not a closed obstacle that can be bypassed.  Giving it the
        // generic +/- bypass_offset value compresses or reverses the coarse
        // stream-function gradient and then contaminates the nested fine BC.
        // Match the ambient stream function instead; this is a topological
        // test and does not depend on recognizing a wall or cylinder shape.
        const bool spans_flow_window =
            comps.s_min[c] <= 0.5 * h + 1e-9 &&
            comps.s_max[c] >= (n_forward - 0.5) * h - 1e-9;
        if (spans_flow_window) {
            out.chosen_sides[c] = 0.0;
            psi_component[c] = psi_inf(comps.centroid_world[c].x(),
                                       comps.centroid_world[c].y());
            continue;
        }
        double side;
        if (c < static_cast<int>(component_side.size()) &&
            std::isfinite(component_side[c]) &&
            std::abs(component_side[c]) == 1.0) {
            side = component_side[c];   // latched side (gvf-layer state)
        } else {
            // Auto decision from the component's lateral extent vs the
            // centerline (eta_reference = 0), not vs the robot position.
            const double cost_plus = std::max(0.0, comps.eta_max[c] - eta_reference);
            const double cost_minus = std::max(0.0, eta_reference - comps.eta_min[c]);
            if (std::abs(cost_plus - cost_minus) < side_deadband) {
                side = default_bias_sign;
            } else {
                side = (cost_plus <= cost_minus) ? 1.0 : -1.0;
            }
        }
        out.chosen_sides[c] = side;
        // With a coarse prior, take the value the coarse solve already assigned
        // this obstacle instead of re-deciding its side from what fits in this
        // (much smaller) window. The eta rule above still runs as the fallback
        // for a component the coarse level cannot resolve.
        double psi_from_prior = 0.0;
        if (prior != nullptr &&
            prior->sample(comps.centroid_world[c], psi_from_prior)) {
            psi_component[c] = psi_from_prior;
        } else {
            psi_component[c] = psi_anchor - side * v_cap * bypass_offset;
        }
    }

    std::vector<bool> is_dirichlet(N, false);
    std::vector<double> psi_bc(N, 0.0);
    for (int i = 0; i < n_forward; ++i) {
        for (int j = 0; j < n_lateral; ++j) {
            const int k = idx(i, j);
            if (solid[i][j]) {
                is_dirichlet[k] = true;
                psi_bc[k] = psi_component[comps.label(i, j)];
            } else if (i == 0 || i == n_forward - 1 ||
                       j == 0 || j == n_lateral - 1) {
                is_dirichlet[k] = true;
                const Eigen::Vector2d world = fluidCellWorld(origin, e_J, n_J, h, i, j);
                // The outer ring is where "beyond this window the world is empty
                // and the flow is uniform" gets asserted to the solver. On the
                // top level that is the only thing available; on a nested fine
                // level the coarse solve knows better, so use it.
                double psi_ring = 0.0;
                if (prior == nullptr || !prior->sample(world, psi_ring)) {
                    psi_ring = psi_inf(world.x(), world.y());
                }
                psi_bc[k] = psi_ring;
            }
        }
    }

    // Matrix-free CG over the free cells only.
    const auto t_asm0 = std::chrono::steady_clock::now();
    std::vector<double> x(N, 0.0), b(N, 0.0);
    constexpr int di[4] = {1, -1, 0, 0};
    constexpr int dj[4] = {0, 0, 1, -1};
    for (int i = 0; i < n_forward; ++i) {
        for (int j = 0; j < n_lateral; ++j) {
            const int k = idx(i, j);
            if (is_dirichlet[k]) continue;
            for (int d = 0; d < 4; ++d) {
                const int km = idx(i + di[d], j + dj[d]);
                if (is_dirichlet[km]) b[k] += psi_bc[km];
            }
        }
    }

    // Branch-free stencil: multiply by a plain double mask (1.0 free, 0.0
    // Dirichlet) so the sweep stays contiguous and vectorizable.
    std::vector<double> freemask(N, 0.0);
    for (int k = 0; k < N; ++k) freemask[k] = is_dirichlet[k] ? 0.0 : 1.0;

    // Only the interior is swept: the outer ring is Dirichlet by construction,
    // so its result is identically zero and skipping it also keeps every k+-1 /
    // k+-n_lateral offset in bounds without a per-cell bounds test.
    // Returns v.(A*v), fused into the same pass to avoid a second sweep of N.
    auto applyA = [&](const std::vector<double>& v, std::vector<double>& out_v) {
        double dot = 0.0;
        std::fill(out_v.begin(), out_v.end(), 0.0);
        for (int i = 1; i + 1 < n_forward; ++i) {
            const int row = i * n_lateral;
            for (int j = 1; j + 1 < n_lateral; ++j) {
                const int k = row + j;
                const double acc = freemask[k] *
                    (4.0 * v[k] - v[k + n_lateral] - v[k - n_lateral]
                                - v[k + 1] - v[k - 1]);
                out_v[k] = acc;
                dot += v[k] * acc;
            }
        }
        return dot;
    };

    const auto t_fac0 = std::chrono::steady_clock::now();
    // Initial iterate: prefer the coarse solution over psi_inf, and not only as
    // boundary data (the coarse solve already carries the obstacles'
    // long-wavelength perturbation, which plain CG resolves slowest).
    for (int i = 0; i < n_forward; ++i) {
        for (int j = 0; j < n_lateral; ++j) {
            const int k = idx(i, j);
            if (is_dirichlet[k]) { x[k] = 0.0; continue; }
            const Eigen::Vector2d w = fluidCellWorld(origin, e_J, n_J, h, i, j);
            double psi_guess = 0.0;
            if (prior == nullptr || !prior->sample(w, psi_guess)) {
                psi_guess = psi_inf(w.x(), w.y());
            }
            x[k] = psi_guess;
        }
    }

    const auto t_slv0 = std::chrono::steady_clock::now();
    std::vector<double> r(N, 0.0), pv(N, 0.0), Ap(N, 0.0);
    applyA(x, Ap);
    double rr = 0.0, bnorm2 = 0.0;
    for (int k = 0; k < N; ++k) {
        r[k] = freemask[k] * (b[k] - Ap[k]);
        pv[k] = r[k];
        rr += r[k] * r[k];
        bnorm2 += freemask[k] * b[k] * b[k];
    }
    const double tol2 = cg_tolerance * cg_tolerance * std::max(bnorm2, 1e-30);
    int iters = 0;
    while (rr > tol2 && iters < cg_max_iterations) {
        const double pAp = applyA(pv, Ap);
        if (!(pAp > 0.0) || !std::isfinite(pAp)) break;   // breakdown guard
        const double alpha = rr / pAp;
        double rr_new = 0.0;
        for (int k = 0; k < N; ++k) {
            x[k] += alpha * pv[k];
            r[k] -= alpha * Ap[k];
            rr_new += r[k] * r[k];
        }
        const double beta = rr_new / rr;
        for (int k = 0; k < N; ++k) pv[k] = r[k] + beta * pv[k];
        rr = rr_new;
        ++iters;
    }
    const auto t_grd0 = std::chrono::steady_clock::now();

    Eigen::VectorXd psi_vec(N);
    for (int k = 0; k < N; ++k) psi_vec[k] = is_dirichlet[k] ? psi_bc[k] : x[k];
    if (!psi_vec.allFinite()) return out;
    out.cg_iterations = iters;
    out.cg_error = std::sqrt(rr / std::max(bnorm2, 1e-30));

    // Validity gate: hitting the iteration cap is not success. A solution
    // whose residual is not a bounded multiple of the tolerance (or is
    // non-finite) is invalid and the caller holds the last valid field.
    if (!std::isfinite(out.cg_error) ||
        out.cg_error > residual_acceptance * std::max(1e-30, cg_tolerance)) {
        return out;
    }

    out.ms_assemble = std::chrono::duration<double, std::milli>(t_fac0 - t_asm0).count();
    out.ms_factor   = std::chrono::duration<double, std::milli>(t_slv0 - t_fac0).count();
    out.ms_solve    = std::chrono::duration<double, std::milli>(t_grd0 - t_slv0).count();

    out.psi.resize(n_forward, n_lateral);
    out.Ux.resize(n_forward, n_lateral);
    out.Uy.resize(n_forward, n_lateral);
    out.Ux_query.resize(n_forward, n_lateral);
    out.Uy_query.resize(n_forward, n_lateral);
    for (int i = 0; i < n_forward; ++i) {
        for (int j = 0; j < n_lateral; ++j) out.psi(i, j) = psi_vec[idx(i, j)];
    }

    for (int i = 0; i < n_forward; ++i) {
        for (int j = 0; j < n_lateral; ++j) {
            if (solid[i][j]) {
                out.Ux(i, j) = 0.0;
                out.Uy(i, j) = 0.0;
                continue;
            }
            const int im = std::max(0, i - 1), ip = std::min(n_forward - 1, i + 1);
            const int jm = std::max(0, j - 1), jp = std::min(n_lateral - 1, j + 1);
            const double dpsi_ds =
                (out.psi(ip, j) - out.psi(im, j)) / (std::max(1, ip - im) * h);
            const double dpsi_deta =
                (out.psi(i, jp) - out.psi(i, jm)) / (std::max(1, jp - jm) * h);
            // In the e_J/n_J frame, u=(dpsi/deta, -dpsi/ds).
            const Eigen::Vector2d world_uv = dpsi_deta * e_J - dpsi_ds * n_J;
            out.Ux(i, j) = world_uv.x();
            out.Uy(i, j) = world_uv.y();
        }
    }
    // The query copies start as the raw harmonic field; only the query copies
    // get dilated below. Ux/Uy stay raw and harmonic.
    out.Ux_query = out.Ux;
    out.Uy_query = out.Uy;

    // Dilate the QUERY field into solid cells: assign each solid cell the mean
    // of its non-solid 4-neighbors' u instead of leaving it exactly zero.
    // Without this, a robot whose position samples land in/near a solid cell
    // (D <= fluid_d_s_) gets no direction at all from the primary field. The
    // harmonic field itself is left untouched -- dilation is an interpolation
    // artifact for bilinear sampling, not part of the solution.
    {
        constexpr int di2[4] = {1, -1, 0, 0};
        constexpr int dj2[4] = {0, 0, 1, -1};
        for (int i = 0; i < n_forward; ++i) {
            for (int j = 0; j < n_lateral; ++j) {
                if (!solid[i][j]) continue;
                double sum_x = 0.0, sum_y = 0.0;
                int count = 0;
                for (int d = 0; d < 4; ++d) {
                    const int ni = i + di2[d], nj = j + dj2[d];
                    if (ni < 0 || ni >= n_forward || nj < 0 ||
                        nj >= n_lateral || solid[ni][nj]) continue;
                    sum_x += out.Ux_query(ni, nj);
                    sum_y += out.Uy_query(ni, nj);
                    ++count;
                }
                if (count > 0) {
                    out.Ux_query(i, j) = sum_x / count;
                    out.Uy_query(i, j) = sum_y / count;
                }
            }
        }
    }
    out.ms_grad = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_grd0).count();
    out.valid = true;
    return out;
}

bool bilinearSampleField(const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fy,
                         const Eigen::Vector2d& origin,
                         const Eigen::Vector2d& e_J,
                         const Eigen::Vector2d& n_J,
                         double h, int n_forward, int n_lateral,
                         const Eigen::Vector2d& world_xy, Eigen::Vector2d& out_uv)
{
    const Eigen::Vector2d relative = world_xy - origin;
    const double gx = relative.dot(e_J) / h - 0.5;
    const double gy = relative.dot(n_J) / h - 0.5;
    const int i0 = static_cast<int>(std::floor(gx));
    const int j0 = static_cast<int>(std::floor(gy));
    if (i0 < 0 || j0 < 0 || i0 + 1 >= n_forward || j0 + 1 >= n_lateral) {
        return false;
    }
    const double tx = gx - i0, ty = gy - j0;
    auto lerp2 = [&](const Eigen::MatrixXd& F) {
        const double v0 = F(i0, j0) * (1 - tx) + F(i0 + 1, j0) * tx;
        const double v1 = F(i0, j0 + 1) * (1 - tx) + F(i0 + 1, j0 + 1) * tx;
        return v0 * (1 - ty) + v1 * ty;
    };
    out_uv = Eigen::Vector2d(lerp2(Fx), lerp2(Fy));
    return true;
}

bool bilinearSamplePsi(const Eigen::MatrixXd& P,
                       const Eigen::Vector2d& origin,
                       const Eigen::Vector2d& e_J,
                       const Eigen::Vector2d& n_J,
                       double h, int n_forward, int n_lateral,
                       const Eigen::Vector2d& world_xy, double& out_psi)
{
    const Eigen::Vector2d relative = world_xy - origin;
    const double gx = relative.dot(e_J) / h - 0.5;
    const double gy = relative.dot(n_J) / h - 0.5;
    const int i0 = static_cast<int>(std::floor(gx));
    const int j0 = static_cast<int>(std::floor(gy));
    if (i0 < 0 || j0 < 0 || i0 + 1 >= n_forward || j0 + 1 >= n_lateral) {
        return false;
    }
    const double tx = gx - i0, ty = gy - j0;
    const double v0 = P(i0, j0) * (1 - tx) + P(i0 + 1, j0) * tx;
    const double v1 = P(i0, j0 + 1) * (1 - tx) + P(i0 + 1, j0 + 1) * tx;
    out_psi = v0 * (1 - ty) + v1 * ty;
    return std::isfinite(out_psi);
}

}  // namespace fluid2d
}  // namespace FLAG_Race
