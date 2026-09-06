#include "fluid/fluid_solver_3d.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace FLAG_Race {
namespace fluid3d {

namespace {

using Clock = std::chrono::steady_clock;

// Team-size cap for every parallel region in this TU. At these grid sizes
// (10-45k cells) the per-iteration work is ~30-130 us, so a full 16-thread
// team is dominated by fork/barrier/reduction overhead: measured on the
// production fine grid (53x53x16, 146 CG iterations), 16 threads ran the
// solve at 30 ms vs 19 ms single-threaded, while 4 threads ran it at 7.9 ms
// -- and under benchmark CPU load the wide-team barriers ballooned to
// 190-320 ms per solve. Four threads is the measured sweet spot and also
// keeps the solver polite toward the concurrently-OpenMP'd SDFMap ESDF pass.
inline int solverThreads()
{
#ifdef _OPENMP
    return std::max(1, std::min(4, omp_get_max_threads()));
#else
    return 1;
#endif
}

inline double msSince(const Clock::time_point& t0)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

inline bool onBoundaryShell(const Grid3D& g, int i, int j, int m)
{
    return i == 0 || i == g.n_fwd - 1 || j == 0 || j == g.n_lat - 1 ||
           m == 0 || m == g.n_z - 1;
}

}  // namespace

bool CoarsePrior3D::sample(const Eigen::Vector3d& world, double& out) const
{
    if (phi == nullptr || grid.n_fwd < 2 || grid.n_lat < 2 || grid.n_z < 2) {
        return false;
    }
    const Eigen::Vector2d rel_xy = world.head<2>() - grid.origin_xy;
    const double gx = rel_xy.dot(grid.e_J) / grid.h - 0.5;
    const double gy = rel_xy.dot(grid.n_J) / grid.h - 0.5;
    const double gz = (world.z() - grid.z_lo) / grid.h_z - 0.5;
    // Permit only the half-cell interpolation fringe at a genuinely shared
    // boundary. Clamping a point farther outside silently copies an unrelated
    // coarse edge value into the fine Dirichlet shell.
    constexpr double eps = 1e-9;
    if (gx < -0.5 - eps || gx > grid.n_fwd - 0.5 + eps ||
        gy < -0.5 - eps || gy > grid.n_lat - 0.5 + eps ||
        gz < -0.5 - eps || gz > grid.n_z - 0.5 + eps) {
        return false;
    }
    const int i0 = std::max(0, std::min(grid.n_fwd - 2,
                                        static_cast<int>(std::floor(gx))));
    const int j0 = std::max(0, std::min(grid.n_lat - 2,
                                        static_cast<int>(std::floor(gy))));
    const int m0 = std::max(0, std::min(grid.n_z - 2,
                                        static_cast<int>(std::floor(gz))));
    const double tx = std::max(0.0, std::min(1.0, gx - i0));
    const double ty = std::max(0.0, std::min(1.0, gy - j0));
    const double tz = std::max(0.0, std::min(1.0, gz - m0));
    const std::vector<double>& P = *phi;
    auto at = [&](int i, int j, int m) { return P[grid.idx(i, j, m)]; };
    auto lerp_z = [&](int i, int j) {
        return at(i, j, m0) * (1 - tz) + at(i, j, m0 + 1) * tz;
    };
    const double v00 = lerp_z(i0, j0), v10 = lerp_z(i0 + 1, j0);
    const double v01 = lerp_z(i0, j0 + 1), v11 = lerp_z(i0 + 1, j0 + 1);
    const double v0 = v00 * (1 - tx) + v10 * tx;
    const double v1 = v01 * (1 - tx) + v11 * tx;
    out = v0 * (1 - ty) + v1 * ty;
    return std::isfinite(out);
}

void foldUnreachablePockets3D(std::vector<uint8_t>& solid, const Grid3D& g)
{
    const int N = g.size();
    std::vector<uint8_t> reachable(N, 0);
    std::queue<int> q;
    for (int i = 0; i < g.n_fwd; ++i) {
        for (int j = 0; j < g.n_lat; ++j) {
            for (int m = 0; m < g.n_z; ++m) {
                if (!onBoundaryShell(g, i, j, m)) continue;
                const int k = g.idx(i, j, m);
                if (solid[k] || reachable[k]) continue;
                reachable[k] = 1;
                q.push(k);
            }
        }
    }
    const int s_fwd = g.n_lat * g.n_z, s_lat = g.n_z, s_z = 1;
    while (!q.empty()) {
        const int k = q.front();
        q.pop();
        const int m = k % g.n_z;
        const int j = (k / g.n_z) % g.n_lat;
        const int i = k / (g.n_lat * g.n_z);
        const int off[6] = {s_fwd, -s_fwd, s_lat, -s_lat, s_z, -s_z};
        const bool ok[6] = {i + 1 < g.n_fwd, i > 0, j + 1 < g.n_lat, j > 0,
                            m + 1 < g.n_z, m > 0};
        for (int d = 0; d < 6; ++d) {
            if (!ok[d]) continue;
            const int kn = k + off[d];
            if (solid[kn] || reachable[kn]) continue;
            reachable[kn] = 1;
            q.push(kn);
        }
    }
    for (int k = 0; k < N; ++k) {
        if (!solid[k] && !reachable[k]) solid[k] = 1;
    }
}

SolidComponents3D labelSolidComponents3D(const std::vector<uint8_t>& solid,
                                         const Grid3D& g,
                                         const Eigen::Vector2d& anchor_xy,
                                         const Eigen::Vector2d& pos_xy,
                                         int m_lo, int m_hi)
{
    SolidComponents3D out;
    out.label.assign(g.size(), -1);
    m_lo = std::max(0, m_lo);
    m_hi = std::min(g.n_z - 1, m_hi);
    const int s_fwd = g.n_lat * g.n_z, s_lat = g.n_z, s_z = 1;
    for (int i = 0; i < g.n_fwd; ++i) {
        for (int j = 0; j < g.n_lat; ++j) {
            for (int m = m_lo; m <= m_hi; ++m) {
                const int k0 = g.idx(i, j, m);
                if (!solid[k0] || out.label[k0] >= 0) continue;
                const int comp = out.num_components++;
                double eta_min = 1e18, eta_max = -1e18;
                double s_min = 1e18, z_max = -1e18;
                std::queue<int> q;
                out.label[k0] = comp;
                q.push(k0);
                while (!q.empty()) {
                    const int k = q.front();
                    q.pop();
                    const int cm = k % g.n_z;
                    const int cj = (k / g.n_z) % g.n_lat;
                    const int ci = k / (g.n_lat * g.n_z);
                    const Eigen::Vector3d world = g.cellWorld(ci, cj, cm);
                    const Eigen::Vector2d rel_a = world.head<2>() - anchor_xy;
                    const Eigen::Vector2d rel_p = world.head<2>() - pos_xy;
                    eta_min = std::min(eta_min, rel_a.dot(g.n_J));
                    eta_max = std::max(eta_max, rel_a.dot(g.n_J));
                    s_min = std::min(s_min, rel_p.dot(g.e_J));
                    z_max = std::max(z_max, world.z());
                    const int off[6] = {s_fwd, -s_fwd, s_lat, -s_lat, s_z, -s_z};
                    const bool ok[6] = {ci + 1 < g.n_fwd, ci > 0,
                                        cj + 1 < g.n_lat, cj > 0,
                                        cm + 1 <= m_hi, cm - 1 >= m_lo};
                    for (int d = 0; d < 6; ++d) {
                        if (!ok[d]) continue;
                        const int kn = k + off[d];
                        if (!solid[kn] || out.label[kn] >= 0) continue;
                        out.label[kn] = comp;
                        q.push(kn);
                    }
                }
                out.eta_min.push_back(eta_min);
                out.eta_max.push_back(eta_max);
                out.s_min.push_back(s_min);
                out.z_max.push_back(z_max);
            }
        }
    }
    return out;
}

PoissonSystem3D buildPoissonSystem3D(const std::vector<uint8_t>& solid,
                                     const Grid3D& g,
                                     double ustar_s, double ustar_lat,
                                     const CoarsePrior3D* prior)
{
    PoissonSystem3D sys;
    const int N = g.size();
    sys.gamma = g.gamma();
    sys.ustar_s = ustar_s;
    sys.ustar_lat = ustar_lat;
    sys.cell_type.assign(N, 0);
    sys.freemask.assign(N, 0.0);
    sys.diag.assign(N, 0.0);
    sys.b.assign(N, 0.0);
    sys.phi_bc.assign(N, 0.0);

    for (int i = 0; i < g.n_fwd; ++i) {
        for (int j = 0; j < g.n_lat; ++j) {
            for (int m = 0; m < g.n_z; ++m) {
                const int k = g.idx(i, j, m);
                if (solid[k]) {
                    sys.cell_type[k] = 2;
                } else if (onBoundaryShell(g, i, j, m)) {
                    sys.cell_type[k] = 1;
                    double bc = 0.0;
                    if (prior == nullptr || !prior->sample(g.cellWorld(i, j, m), bc)) {
                        bc = 0.0;   // phi == 0 is the exact obstacle-free far field
                    }
                    sys.phi_bc[k] = bc;
                } else {
                    sys.freemask[k] = 1.0;
                }
            }
        }
    }

    const int s_fwd = g.n_lat * g.n_z, s_lat = g.n_z;
    const double gamma = sys.gamma;
    for (int i = 1; i + 1 < g.n_fwd; ++i) {
        for (int j = 1; j + 1 < g.n_lat; ++j) {
            for (int m = 1; m + 1 < g.n_z; ++m) {
                const int k = g.idx(i, j, m);
                if (sys.cell_type[k] != 0) continue;
                int n_solid_lat = 0, n_solid_z = 0;
                double b = 0.0;
                // Lateral faces carry the u* flux; signs are the outward face
                // normals dotted with u*: a solid +e_J neighbour pushes phi up
                // upstream of a wall so u_s = ustar_s - dphi/ds decays into it.
                const int kp_f = k + s_fwd, km_f = k - s_fwd;
                if (sys.cell_type[kp_f] == 2) { ++n_solid_lat; b += g.h * ustar_s; }
                else if (sys.cell_type[kp_f] == 1) b += sys.phi_bc[kp_f];
                if (sys.cell_type[km_f] == 2) { ++n_solid_lat; b -= g.h * ustar_s; }
                else if (sys.cell_type[km_f] == 1) b += sys.phi_bc[km_f];
                const int kp_l = k + s_lat, km_l = k - s_lat;
                if (sys.cell_type[kp_l] == 2) { ++n_solid_lat; b += g.h * ustar_lat; }
                else if (sys.cell_type[kp_l] == 1) b += sys.phi_bc[kp_l];
                if (sys.cell_type[km_l] == 2) { ++n_solid_lat; b -= g.h * ustar_lat; }
                else if (sys.cell_type[km_l] == 1) b += sys.phi_bc[km_l];
                const int kp_z = k + 1, km_z = k - 1;
                if (sys.cell_type[kp_z] == 2) ++n_solid_z;
                else if (sys.cell_type[kp_z] == 1) b += gamma * sys.phi_bc[kp_z];
                if (sys.cell_type[km_z] == 2) ++n_solid_z;
                else if (sys.cell_type[km_z] == 1) b += gamma * sys.phi_bc[km_z];
                // A free cell walled in on all six faces is unreachable and
                // should have been folded by foldUnreachablePockets3D; demote
                // it defensively rather than leave a singular zero row.
                if (n_solid_lat == 4 && n_solid_z == 2) {
                    sys.cell_type[k] = 2;
                    sys.freemask[k] = 0.0;
                    continue;
                }
                sys.diag[k] = (4.0 - n_solid_lat) + gamma * (2.0 - n_solid_z);
                sys.b[k] = b;
            }
        }
    }
    return sys;
}

double applyPoisson3D(const PoissonSystem3D& sys, const Grid3D& g,
                      const std::vector<double>& v, std::vector<double>& out_v,
                      bool out_prezeroed)
{
    // Branch-free 7-point stencil, same freemask trick as the 2D solver: a
    // plain double mask keeps the sweep contiguous and vectorizable, and all
    // CG vectors hold zero on non-free cells so solid neighbours (coupling
    // dropped from diag) and Dirichlet neighbours (folded into b) both
    // correctly contribute nothing. Only the interior is swept: boundary
    // shells are never unknowns, so every offset stays in bounds.
    const int n_fwd = g.n_fwd, n_lat = g.n_lat, n_z = g.n_z;
    const int s_fwd = n_lat * n_z, s_lat = n_z;
    const double gamma = sys.gamma;
    const double* fm = sys.freemask.data();
    const double* dg = sys.diag.data();
    const double* pv = v.data();
    double* po = out_v.data();
    if (!out_prezeroed) std::fill(out_v.begin(), out_v.end(), 0.0);
    double dot = 0.0;
#pragma omp parallel for collapse(2) schedule(static) reduction(+ : dot) num_threads(solverThreads())
    for (int i = 1; i < n_fwd - 1; ++i) {
        for (int j = 1; j < n_lat - 1; ++j) {
            const int base = (i * n_lat + j) * n_z;
            for (int m = 1; m < n_z - 1; ++m) {
                const int k = base + m;
                const double acc = fm[k] *
                    (dg[k] * pv[k]
                     - pv[k + s_fwd] - pv[k - s_fwd]
                     - pv[k + s_lat] - pv[k - s_lat]
                     - gamma * (pv[k + 1] + pv[k - 1]));
                po[k] = acc;
                dot += pv[k] * acc;
            }
        }
    }
    return dot;
}

PotentialField3D solvePotentialFlow3D(const std::vector<uint8_t>& solid,
                                      const Grid3D& g,
                                      double v_cap, double ustar_lat,
                                      const CoarsePrior3D* prior,
                                      double cg_tolerance, int cg_max_iterations,
                                      double residual_acceptance)
{
    PotentialField3D out;
    if (g.n_fwd < 3 || g.n_lat < 3 || g.n_z < 3) return out;
    const int N = g.size();

    const Clock::time_point t_asm0 = Clock::now();
    PoissonSystem3D sys = buildPoissonSystem3D(solid, g, v_cap, ustar_lat, prior);

    // Initial iterate: the coarse phi already carries every obstacle's
    // long-wavelength perturbation (the frequencies plain CG resolves
    // slowest); without a prior, phi = 0 is exact wherever there are no
    // obstacles. No cross-frame warm start, for the same condition-number
    // reason documented in the 2D solver.
    std::vector<double> x(N, 0.0);
    if (prior != nullptr) {
        for (int i = 1; i + 1 < g.n_fwd; ++i) {
            for (int j = 1; j + 1 < g.n_lat; ++j) {
                for (int m = 1; m + 1 < g.n_z; ++m) {
                    const int k = g.idx(i, j, m);
                    if (sys.cell_type[k] != 0) continue;
                    double guess = 0.0;
                    if (prior->sample(g.cellWorld(i, j, m), guess)) x[k] = guess;
                }
            }
        }
    }
    out.ms_assemble = msSince(t_asm0);

    const Clock::time_point t_slv0 = Clock::now();
    std::vector<double> r(N, 0.0), pv(N, 0.0), Ap(N, 0.0);
    applyPoisson3D(sys, g, x, Ap);
    double rr = 0.0, bnorm2 = 0.0;
#pragma omp parallel for schedule(static) reduction(+ : rr, bnorm2) num_threads(solverThreads())
    for (int k = 0; k < N; ++k) {
        r[k] = sys.freemask[k] * (sys.b[k] - Ap[k]);
        pv[k] = r[k];
        rr += r[k] * r[k];
        bnorm2 += sys.freemask[k] * sys.b[k] * sys.b[k];
    }
    const double tol2 = cg_tolerance * cg_tolerance * std::max(bnorm2, 1e-30);
    int iters = 0;
    double pAp_iter = 0.0, rr_new_iter = 0.0, alpha = 0.0, beta = 0.0;
    bool stop = false, breakdown = false;
#pragma omp parallel num_threads(solverThreads()) shared(rr, iters, pAp_iter, rr_new_iter, alpha, beta, stop, breakdown)
    {
        while (true) {
#pragma omp single
            stop = !(rr > tol2 && iters < cg_max_iterations);
            if (stop) break;

#pragma omp single
            pAp_iter = 0.0;
#pragma omp for collapse(2) schedule(static) reduction(+ : pAp_iter)
            for (int i = 1; i < g.n_fwd - 1; ++i) {
                for (int j = 1; j < g.n_lat - 1; ++j) {
                    const int base = (i * g.n_lat + j) * g.n_z;
                    for (int m = 1; m < g.n_z - 1; ++m) {
                        const int k = base + m;
                        const double acc = sys.freemask[k] *
                            (sys.diag[k] * pv[k]
                             - pv[k + g.n_lat * g.n_z] - pv[k - g.n_lat * g.n_z]
                             - pv[k + g.n_z] - pv[k - g.n_z]
                             - sys.gamma * (pv[k + 1] + pv[k - 1]));
                        Ap[k] = acc;
                        pAp_iter += pv[k] * acc;
                    }
                }
            }
#pragma omp single
            {
                breakdown = !(pAp_iter > 0.0) || !std::isfinite(pAp_iter);
                if (!breakdown) alpha = rr / pAp_iter;
            }
            if (breakdown) break;

#pragma omp single
            rr_new_iter = 0.0;
#pragma omp for schedule(static) reduction(+ : rr_new_iter)
            for (int k = 0; k < N; ++k) {
                x[k] += alpha * pv[k];
                r[k] -= alpha * Ap[k];
                rr_new_iter += r[k] * r[k];
            }
#pragma omp single
            {
                beta = rr > 0.0 ? rr_new_iter / rr : 0.0;
                rr = rr_new_iter;
                ++iters;
            }

#pragma omp for schedule(static)
            for (int k = 0; k < N; ++k) pv[k] = r[k] + beta * pv[k];
            // OpenMP reductions sum in nondeterministic order, so the
            // iteration count can jitter by 1-2 run to run; the consumer is
            // a direction field, which is insensitive at these tolerances.
        }
    }
    out.cg_iterations = iters;
    out.cg_error = std::sqrt(rr / std::max(bnorm2, 1e-30));
    out.ms_solve = msSince(t_slv0);

    // Validity gate: hitting the iteration cap is not success. A solution
    // whose residual is not a bounded multiple of the tolerance (or is
    // non-finite) is invalid and the caller holds the last valid field.
    if (!std::isfinite(out.cg_error) ||
        out.cg_error > residual_acceptance * std::max(1e-30, cg_tolerance)) {
        return out;
    }

    const Clock::time_point t_grd0 = Clock::now();
    out.phi.assign(N, 0.0);
    for (int k = 0; k < N; ++k) {
        if (sys.cell_type[k] == 0) out.phi[k] = x[k];
        else if (sys.cell_type[k] == 1) out.phi[k] = sys.phi_bc[k];
    }
    for (int k = 0; k < N; ++k) {
        if (!std::isfinite(out.phi[k])) return out;   // out.valid stays false
    }

    // Velocity extraction: average the two face-normal velocities per axis
    // (the staggered/MAC velocity brought to cell centers). Where both
    // neighbours are non-solid this is exactly the central difference
    // u = u* - grad(phi); a solid face contributes its enforced zero flux
    // instead, so a dead-end column correctly reads u ~ 0 rather than the
    // free-stream value a naive central difference through the wall gives.
    out.Ux.assign(N, 0.0);
    out.Uy.assign(N, 0.0);
    out.Uz.assign(N, 0.0);
    const int s_fwd = g.n_lat * g.n_z, s_lat = g.n_z;
#pragma omp parallel for collapse(2) schedule(static) num_threads(solverThreads())
    for (int i = 0; i < g.n_fwd; ++i) {
        for (int j = 0; j < g.n_lat; ++j) {
            for (int m = 0; m < g.n_z; ++m) {
                const int k = g.idx(i, j, m);
                if (sys.cell_type[k] == 2) continue;   // dilated below
                auto axis_u = [&](int stride, bool has_m, bool has_p,
                                  double ustar_a, double h_a) {
                    double sum = 0.0;
                    int count = 0;
                    if (has_m) {
                        ++count;
                        if (sys.cell_type[k - stride] != 2) {
                            sum += ustar_a - (out.phi[k] - out.phi[k - stride]) / h_a;
                        }   // solid face: enforced zero flux
                    }
                    if (has_p) {
                        ++count;
                        if (sys.cell_type[k + stride] != 2) {
                            sum += ustar_a - (out.phi[k + stride] - out.phi[k]) / h_a;
                        }
                    }
                    return count > 0 ? sum / count : ustar_a;
                };
                const double u_s = axis_u(s_fwd, i > 0, i + 1 < g.n_fwd,
                                          sys.ustar_s, g.h);
                const double u_e = axis_u(s_lat, j > 0, j + 1 < g.n_lat,
                                          sys.ustar_lat, g.h);
                const double u_z = axis_u(1, m > 0, m + 1 < g.n_z, 0.0, g.h_z);
                out.Ux[k] = u_s * g.e_J.x() + u_e * g.n_J.x();
                out.Uy[k] = u_s * g.e_J.y() + u_e * g.n_J.y();
                out.Uz[k] = u_z;
            }
        }
    }

    // Dilate into solid cells: mean of the non-solid 6-neighbours, so a
    // robot sample landing inside the inflated band still gets a direction
    // (same rationale as the 2D dilation; the quiver skips solid cells, so
    // this only feeds trilinearSampleField3D).
    for (int i = 0; i < g.n_fwd; ++i) {
        for (int j = 0; j < g.n_lat; ++j) {
            for (int m = 0; m < g.n_z; ++m) {
                const int k = g.idx(i, j, m);
                if (sys.cell_type[k] != 2) continue;
                double sx = 0.0, sy = 0.0, sz = 0.0;
                int count = 0;
                const int off[6] = {s_fwd, -s_fwd, s_lat, -s_lat, 1, -1};
                const bool ok[6] = {i + 1 < g.n_fwd, i > 0, j + 1 < g.n_lat,
                                    j > 0, m + 1 < g.n_z, m > 0};
                for (int d = 0; d < 6; ++d) {
                    if (!ok[d]) continue;
                    const int kn = k + off[d];
                    if (sys.cell_type[kn] == 2) continue;
                    sx += out.Ux[kn];
                    sy += out.Uy[kn];
                    sz += out.Uz[kn];
                    ++count;
                }
                if (count > 0) {
                    out.Ux[k] = sx / count;
                    out.Uy[k] = sy / count;
                    out.Uz[k] = sz / count;
                }
            }
        }
    }
    out.ms_grad = msSince(t_grd0);
    out.valid = true;
    return out;
}

bool trilinearSampleField3D(const std::vector<double>& Fx,
                            const std::vector<double>& Fy,
                            const std::vector<double>& Fz,
                            const Grid3D& g,
                            const Eigen::Vector3d& world,
                            Eigen::Vector3d& out_uvw)
{
    const Eigen::Vector2d rel_xy = world.head<2>() - g.origin_xy;
    const double gx = rel_xy.dot(g.e_J) / g.h - 0.5;
    const double gy = rel_xy.dot(g.n_J) / g.h - 0.5;
    const double gz = (world.z() - g.z_lo) / g.h_z - 0.5;
    const int i0 = static_cast<int>(std::floor(gx));
    const int j0 = static_cast<int>(std::floor(gy));
    const int m0 = static_cast<int>(std::floor(gz));
    if (i0 < 0 || j0 < 0 || m0 < 0 ||
        i0 + 1 >= g.n_fwd || j0 + 1 >= g.n_lat || m0 + 1 >= g.n_z) {
        return false;
    }
    const double tx = gx - i0, ty = gy - j0, tz = gz - m0;
    auto lerp3 = [&](const std::vector<double>& F) {
        auto at = [&](int i, int j, int m) { return F[g.idx(i, j, m)]; };
        auto lz = [&](int i, int j) {
            return at(i, j, m0) * (1 - tz) + at(i, j, m0 + 1) * tz;
        };
        const double v0 = lz(i0, j0) * (1 - tx) + lz(i0 + 1, j0) * tx;
        const double v1 = lz(i0, j0 + 1) * (1 - tx) + lz(i0 + 1, j0 + 1) * tx;
        return v0 * (1 - ty) + v1 * ty;
    };
    out_uvw = Eigen::Vector3d(lerp3(Fx), lerp3(Fy), lerp3(Fz));
    return true;
}

}  // namespace fluid3d
}  // namespace FLAG_Race
