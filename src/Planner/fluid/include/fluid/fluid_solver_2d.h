#ifndef _FLUID_SOLVER_2D_H
#define _FLUID_SOLVER_2D_H

// 2D harmonic stream-function guidance solver core.
//
// Solves Laplace's equation for the stream function psi on a local window
// (5-point FD stencil, matrix-free CG): Dirichlet psi = psi_k on each solid
// connected component k, Dirichlet psi = psi_inf on the outer ring (solid BC
// takes priority where a component touches the ring), free interior cells
// satisfy 4*psi(i,j) - sum(neighbors) = 0. Because psi is harmonic it has no
// interior local extrema, so critical points of the reconstructed field
// u = (dpsi/deta)*e_J - (dpsi/ds)*n_J can only be saddles -- no stagnation
// basins by construction.
//
// psi_inf(x,y) = v_cap * ((x,y)-anchor).n_J is the uniform joystick-aligned
// far-field potential; with this codebase's e_J/n_J convention (n_J = e_J
// rotated +90 deg) it reduces exactly to u = v_cap*e_J far from all
// obstacles. Per-component boundary values are anchored to the joystick
// centerline (the anchor lies on it, so psi_inf(anchor) = 0): the implicit
// circulation of each obstacle is fixed at anchor time and does not drift
// with the robot's lateral position.
//
// The solver is stateless: per-component bypass sides are decided by the
// caller (side-latch layer) and passed in as component_side[]; the solver
// only turns them into Dirichlet data. This keeps the continuous side
// decision (latch, centroid association, one-to-one matching) out of the
// pure solver so it can be tested as a function of (mask, BCs, params).
//
// The harmonic field itself is returned raw (Ux/Uy pre-dilation); a separate
// query field (Ux_query/Uy_query) is dilated into solid cells for bilinear
// sampling at robot/visualization positions. Only the query field may carry
// nonzero values inside solid cells -- that is an interpolation artifact,
// never part of the harmonic solution.

#include <vector>

#include <Eigen/Dense>

namespace FLAG_Race {
namespace fluid2d {

// One at D<=d_near, zero at D>=d_far, cubic in between. Used by both the
// near-wall resistance gate and the speed-ramp/convergence-gate laws.
inline double fluidSmoothstepW(double D, double d_near, double d_far)
{
    const double xi = std::min(1.0, std::max(0.0, (D - d_near) / (d_far - d_near)));
    return 1.0 - 3.0 * xi * xi + 2.0 * xi * xi * xi;
}

// 0 at D<=d_s (solid), 1 at D>=d_turn (full joystick speed), smooth ramp
// between. Reuses the two existing clearance knobs instead of adding a new
// one.
inline double fluidSpeedRatio(double D, double d_s, double d_turn)
{
    return 1.0 - fluidSmoothstepW(D, d_s, d_turn);
}

struct SolidComponents
{
    Eigen::MatrixXi label;                          // forward x lateral, -1 = free cell
    std::vector<Eigen::Vector2d> centroid_world;    // per-component centroid (diagnostics only)
    std::vector<double> eta_min, eta_max;           // per-component lateral extent along n_J (relative to anchor)
    std::vector<double> s_min, s_max;               // per-component forward extent along e_J (relative to origin)
    int num_components = 0;
};

inline Eigen::Vector2d fluidCellWorld(const Eigen::Vector2d& origin,
                                      const Eigen::Vector2d& e_J,
                                      const Eigen::Vector2d& n_J,
                                      double h, int i, int j)
{
    return origin + (i + 0.5) * h * e_J + (j + 0.5) * h * n_J;
}

struct FluidHarmonicField
{
    Eigen::MatrixXd psi, Ux, Uy;          // all forward x lateral; Ux/Uy world-frame, RAW harmonic field
    Eigen::MatrixXd Ux_query, Uy_query;   // dilated query copies for bilinear sampling (not harmonic in solid)
    std::vector<double> chosen_sides;     // per-component effective bypass side (+-1), for the side latch
    bool valid = false;
    // Per-stage wall time (ms), for the breakdown logged by calcFluidGuidance2D.
    double ms_assemble = 0.0, ms_factor = 0.0, ms_solve = 0.0, ms_grad = 0.0;
    // CG convergence: iterations sets the solve cost, and error rising toward
    // the tolerance is the early warning that a scene has outgrown maxIterations.
    int cg_iterations = 0;
    double cg_error = 0.0;
};

// A solved coarser level, consumed by a finer one purely as a source of
// Dirichlet data. This is what makes the two levels one field rather than two:
// the fine solve is the exact harmonic continuation of the coarse solve into
// the high-resolution region, so nothing is blended and nothing can cancel.
// It carries the far-field decision as well as the far-field values.
struct FluidCoarsePrior
{
    const Eigen::MatrixXd* psi = nullptr;
    Eigen::Vector2d origin{0.0, 0.0}, e_J{1.0, 0.0}, n_J{0.0, 1.0};
    double h = 0.0;
    int n_forward = 0, n_lateral = 0;

    bool sample(const Eigen::Vector2d& world_xy, double& out) const
    {
        if (psi == nullptr || n_forward < 2 || n_lateral < 2) return false;
        const Eigen::Vector2d relative = world_xy - origin;
        const double gx = relative.dot(e_J) / h - 0.5;
        const double gy = relative.dot(n_J) / h - 0.5;
        // Clamp rather than reject: a fine cell can land just outside the coarse
        // cell-centre lattice at the shared boundary, and the nearest coarse
        // value there is a far better Dirichlet datum than falling back to the
        // obstacle-free psi_inf, which is precisely the information the coarse
        // level exists to correct.
        const int i0 = std::max(0, std::min(n_forward - 2,
                                            static_cast<int>(std::floor(gx))));
        const int j0 = std::max(0, std::min(n_lateral - 2,
                                            static_cast<int>(std::floor(gy))));
        const double tx = std::max(0.0, std::min(1.0, gx - i0));
        const double ty = std::max(0.0, std::min(1.0, gy - j0));
        const Eigen::MatrixXd& P = *psi;
        const double v0 = P(i0, j0) * (1 - tx) + P(i0 + 1, j0) * tx;
        const double v1 = P(i0, j0 + 1) * (1 - tx) + P(i0 + 1, j0 + 1) * tx;
        out = v0 * (1 - ty) + v1 * ty;
        return std::isfinite(out);
    }
};

// One solved grid level: geometry, the ESDF slice it was built from, its solid
// mask/components, and the resulting harmonic field. Two of these are nested
// per solve (see calcFluidGuidance2D).
struct FluidLevel
{
    Eigen::Vector2d origin{0.0, 0.0};
    double h = 0.0, rear_margin = 0.0;
    int n_forward = 0, n_lateral = 0;
    Eigen::MatrixXd D;
    std::vector<std::vector<bool>> solid;
    SolidComponents comps;
    FluidHarmonicField field;
    double ms_esdf = 0.0, ms_mask = 0.0, ms_label = 0.0, ms_field = 0.0;
    double ms_total() const { return ms_esdf + ms_mask + ms_label + ms_field; }
};

// 4-connected flood fill over the (already BFS-reachability-folded) solid
// mask, computing each component's world-space centroid and its lateral extent
// along n_J (eta = (cell-anchor).n_J) and forward extent along e_J
// (s = (cell-origin).e_J). Used by solveHarmonicStreamField to pick which
// side is cheaper to bypass on, and by the gvf-layer side latch to associate
// components across solves.
SolidComponents labelSolidComponents(const std::vector<std::vector<bool>>& solid,
                                     const Eigen::Vector2d& origin,
                                     const Eigen::Vector2d& e_J,
                                     const Eigen::Vector2d& n_J,
                                     double h,
                                     const Eigen::Vector2d& anchor_xy,
                                     int n_forward, int n_lateral);

// Solves the 2D harmonic stream function psi on the local window (see the
// file comment for the BC structure). component_side[c], when nonempty and
// with |value| == 1, fixes the bypass side of component c (side latch layer);
// otherwise the solver falls back to the eta-extent rule measured from the
// anchor's centerline (eta_reference = 0). Closed components use
// psi_k = psi_inf(anchor) - side_k*v_cap*bypass_offset. A component spanning
// both flow-wise window edges is instead treated as an unresolved local
// boundary and receives psi_inf at its centroid, so a corridor wall cannot be
// assigned the circulation intended for a bypassable obstacle. The coarse
// prior (if given) takes precedence for the remaining components.
//
// The solve is invalid (out.valid stays false) if CG fails to converge to
// cg_error <= residual_acceptance * cg_tolerance (or any residual is
// non-finite); callers then hold the last valid field.
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
    const FluidCoarsePrior* prior);

// Bilinear sample of a cached per-cell field at an arbitrary world point,
// using the same cell-center convention as the rotated D grid built in
// calcFluidGuidance2D. i runs along e_J and j runs along n_J.
// Returns false (leaving out_uv untouched) if world_xy needs a full-cell
// margin on any side that the window doesn't have.
bool bilinearSampleField(const Eigen::MatrixXd& Fx, const Eigen::MatrixXd& Fy,
                         const Eigen::Vector2d& origin,
                         const Eigen::Vector2d& e_J,
                         const Eigen::Vector2d& n_J,
                         double h, int n_forward, int n_lateral,
                         const Eigen::Vector2d& world_xy, Eigen::Vector2d& out_uv);

// Scalar bilinear sample (the stream function itself, for the psi-error
// control law and the chi* reference bookkeeping).
bool bilinearSamplePsi(const Eigen::MatrixXd& P,
                       const Eigen::Vector2d& origin,
                       const Eigen::Vector2d& e_J,
                       const Eigen::Vector2d& n_J,
                       double h, int n_forward, int n_lateral,
                       const Eigen::Vector2d& world_xy, double& out_psi);

// ===== Stream-function-error control law (paper eq. 29, normalized) =====
//
// chi = psi / v_s is the v_cap-normalized stream function (v_s = the speed
// the cached field was solved with, NOT the live joystick speed, so the
// normalization is invariant between field rebuilds). With
//
//   grad chi = (-u_y, u_x) / v_s,     e_chi = chi - chi*,
//
// the nominal law is
//
//   v_nom = v_t * u/|u| - k_psi * e_chi * grad_chi / (|grad_chi|^2 + eps),
//   k_psi in s^-1
//
// giving d/dt e_chi = -k_psi * e_chi on the closed loop (exact exponential
// decay, |grad_chi|^2 = |u|^2/v_s^2). Degraded when the field is degenerate
// (|u| ~ 0), in which case the caller falls back to its angle-space law.
struct StreamFunctionLawResult
{
    Eigen::Vector2d v;       // v_nom
    double e_chi = 0.0;      // chi - chi*
    double correction_norm = 0.0;
    bool degraded = true;    // true if the field is too degenerate for the law
};

inline StreamFunctionLawResult streamFunctionErrorLaw(const Eigen::Vector2d& u,
                                                      double v_s,
                                                      double chi, double chi_star,
                                                      double v_t, double k_psi)
{
    StreamFunctionLawResult r;
    r.e_chi = chi - chi_star;
    const double u_norm = u.norm();
    if (!(u_norm > 1e-9) || !(v_s > 1e-9)) {
        r.degraded = true;
        r.v.setZero();
        return r;
    }
    r.v = (v_t / u_norm) * u;                       // v_t * t_hat
    const Eigen::Vector2d grad_chi(-u.y() / v_s, u.x() / v_s);
    const double gn2 = grad_chi.squaredNorm();
    if (gn2 > 1e-4) {
        const Eigen::Vector2d corr = -k_psi * r.e_chi * grad_chi / gn2;
        r.correction_norm = corr.norm();
        r.v += corr;
        r.degraded = false;
    } else {
        r.degraded = true;
    }
    return r;
}

// Keeps the stream-function error continuous across field rebuilds without
// re-sampling the (possibly long-departed) anchor: if the reference level
// was chi*_old while the field was old, then after replacing the field the
// new reference chi*_new = chi_new(x) - (chi_old(x) - chi*_old) makes
// e_chi(x) identical at the swap instant (paper eq. 71-74 time-varying
// reference).
inline double chiStarContinuityUpdate(double chi_old_here, double chi_new_here,
                                      double chi_star_old)
{
    return chi_new_here - (chi_old_here - chi_star_old);
}

}  // namespace fluid2d
}  // namespace FLAG_Race

#endif
