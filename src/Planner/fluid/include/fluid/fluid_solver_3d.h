#ifndef _FLUID_SOLVER_3D_H
#define _FLUID_SOLVER_3D_H

// 3D potential-flow guidance solver core.
//
// The 2D fluid guidance solves a harmonic *stream function* psi (gvf.cpp,
// solveHarmonicStreamField). A scalar stream function does not exist in 3D,
// so this level solves the pressure-projection form instead: given the
// uniform far field u* = v_cap*e_J + ustar_lat*n_J (always horizontal),
// find phi so that
//
//     u = u* - grad(phi)
//
// is discretely divergence-free with zero normal flux through every solid
// face. Obstacles are therefore Neumann walls here, not the per-component
// Dirichlet iso-psi constants of the 2D solver; the 2D bypass_offset side
// lever has no analogue and is replaced at the caller level by a latched
// lateral crossflow folded into u* (which only perturbs the RHS fluxes --
// same solve cost).
//
// Discretization: cell-centered finite volumes in the rotated intent frame
// (i along e_J, spacing h; j along n_J, spacing h; m along world z, spacing
// h_z), flat index k = (i*n_lat + j)*n_z + m -- z innermost, so the 7-point
// stencil touches five contiguous columns and OpenMP collapse(2) over (i,j)
// hands each thread whole columns with no false sharing. With the whole
// system divided by h_z and gamma = (h/h_z)^2, each free cell k satisfies
//
//   diag[k]*phi_k - sum_{lat free nb} phi_nb - gamma*sum_{z free nb} phi_nb = b[k]
//   diag[k] = (4 - n_solid_lat_faces) + gamma*(2 - n_solid_z_faces)
//   b[k]    = h * sum_{lat solid faces} (u* . n_face)     // Neumann flux
//           + sum_{lat Dirichlet nb} phi_bc
//           + gamma * sum_{z Dirichlet nb} phi_bc
//
// (z faces carry no u* flux because u*_z = 0; Dirichlet faces keep their
// diagonal contribution, solid faces drop it.) Boundary shells of the window
// are Dirichlet where free: phi_bc = 0 on a top-level solve (phi == 0 is the
// exact obstacle-free far field) or the trilinear sample of a coarser solve
// (CoarsePrior3D) -- the same nesting contract as the 2D FluidCoarsePrior.
//
// SPD: with Dirichlet data folded into b, A = sum over interior faces of
// c_f (e_k - e_m)(e_k - e_m)^T plus, for each free-Dirichlet face, c_f e_k
// e_k^T, with c_f in {1, gamma} > 0. x^T A x = 0 forces x constant per
// free-connected component and zero on any cell with a Dirichlet face;
// foldUnreachablePockets3D guarantees every surviving free cell reaches a
// Dirichlet shell, so A is SPD and matrix-free CG applies -- the same
// well-posedness argument as the 2D solver, restated for Neumann obstacles.
//
// This lives in its own translation unit (unlike the 2D solver, which is
// file-local in gvf.cpp) so the operator symmetry/SPD properties and the
// flux behavior around obstacles are directly gtest-able.

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace FLAG_Race {
namespace fluid3d {

struct Grid3D
{
    Eigen::Vector2d origin_xy{0.0, 0.0};        // world xy of the (i=0,j=0) cell corner
    Eigen::Vector2d e_J{1.0, 0.0}, n_J{0.0, 1.0};
    double z_lo = 0.0;                          // world z of the m=0 cell bottom
    double h = 0.15, h_z = 0.15;                // xy / z cell size (h_z <= 2h; see gamma)
    int n_fwd = 0, n_lat = 0, n_z = 0;

    int idx(int i, int j, int m) const { return (i * n_lat + j) * n_z + m; }
    int size() const { return n_fwd * n_lat * n_z; }
    double gamma() const { return (h / h_z) * (h / h_z); }
    Eigen::Vector3d cellWorld(int i, int j, int m) const
    {
        const Eigen::Vector2d xy =
            origin_xy + (i + 0.5) * h * e_J + (j + 0.5) * h * n_J;
        return Eigen::Vector3d(xy.x(), xy.y(), z_lo + (m + 0.5) * h_z);
    }
};

// Labels ONLY the obstacle band m in [m_lo, m_hi]. The caller forces solid
// floor/ceiling bands onto the mask before solving; those slabs touch every
// obstacle, so labeling them would merge all obstacles into one component
// and destroy the per-obstacle lateral extents the crossflow latch needs.
struct SolidComponents3D
{
    std::vector<int> label;                     // flat Grid3D layout, -1 = not labeled
    std::vector<double> eta_min, eta_max;       // lateral extent along n_J vs anchor
    std::vector<double> s_min;                  // nearest forward distance vs robot
    std::vector<double> z_max;                  // top height (diagnostics/latch logs)
    int num_components = 0;
};

// Assembled system, exposed (rather than local to the solve) so tests can
// probe operator symmetry and definiteness through applyPoisson3D.
struct PoissonSystem3D
{
    // cell_type: 0 = free unknown, 1 = Dirichlet shell, 2 = solid.
    std::vector<uint8_t> cell_type;
    std::vector<double> freemask;               // 1.0 free, 0.0 otherwise (branch-free kernel)
    std::vector<double> diag;                   // valid where freemask == 1
    std::vector<double> b;
    std::vector<double> phi_bc;                 // nonzero only on Dirichlet cells
    double gamma = 1.0;
    double ustar_s = 0.0, ustar_lat = 0.0;      // u* in the (e_J, n_J) frame
};

struct PotentialField3D
{
    std::vector<double> phi;                    // free: solved; Dirichlet: phi_bc; solid: 0
    std::vector<double> Ux, Uy, Uz;             // world-frame, face-averaged (see .cpp)
    bool valid = false;
    double ms_assemble = 0.0, ms_solve = 0.0, ms_grad = 0.0;
    int cg_iterations = 0;
    double cg_error = 0.0;
};

// A solved coarser level consumed as Dirichlet data + initial iterate by a
// finer one -- the exact continuation contract of the 2D FluidCoarsePrior.
// Sampling accepts the shared half-cell fringe but rejects points genuinely
// outside the coarse domain.
struct CoarsePrior3D
{
    const std::vector<double>* phi = nullptr;
    Grid3D grid;
    bool sample(const Eigen::Vector3d& world, double& out) const;
};

// 6-connected reachability fold seeded from every free boundary-shell cell.
// Free cells that cannot reach the window boundary become solid, which is
// what grounds every remaining unknown to Dirichlet data (SPD argument).
void foldUnreachablePockets3D(std::vector<uint8_t>& solid, const Grid3D& g);

SolidComponents3D labelSolidComponents3D(const std::vector<uint8_t>& solid,
                                         const Grid3D& g,
                                         const Eigen::Vector2d& anchor_xy,
                                         const Eigen::Vector2d& pos_xy,
                                         int m_lo, int m_hi);

PoissonSystem3D buildPoissonSystem3D(const std::vector<uint8_t>& solid,
                                     const Grid3D& g,
                                     double ustar_s, double ustar_lat,
                                     const CoarsePrior3D* prior);

// out_v = A*v over the free interior; returns v . (A*v) fused into the same
// sweep. v must be zero on non-free cells (all CG vectors maintain this).
// Set out_prezeroed when the caller has already zeroed the output and every
// non-boundary entry will be overwritten by the stencil sweep.
double applyPoisson3D(const PoissonSystem3D& sys, const Grid3D& g,
                      const std::vector<double>& v, std::vector<double>& out_v,
                      bool out_prezeroed = false);

// Full pipeline for one level: assemble, CG (matrix-free, warm-started from
// the prior when given), face-averaged velocity extraction, solid dilation.
// The solve is invalid (out.valid stays false) if CG fails to converge to
// cg_error <= residual_acceptance * cg_tolerance (or any value is
// non-finite); callers then hold the last valid field.
PotentialField3D solvePotentialFlow3D(const std::vector<uint8_t>& solid,
                                      const Grid3D& g,
                                      double v_cap, double ustar_lat,
                                      const CoarsePrior3D* prior,
                                      double cg_tolerance, int cg_max_iterations,
                                      double residual_acceptance);

// Trilinear sample of the cached per-cell field, cell-center convention
// (grid coord = projection/h - 0.5 per axis), full-cell margin required on
// every side -- the 3D analogue of bilinearSampleField.
bool trilinearSampleField3D(const std::vector<double>& Fx,
                            const std::vector<double>& Fy,
                            const std::vector<double>& Fz,
                            const Grid3D& g,
                            const Eigen::Vector3d& world,
                            Eigen::Vector3d& out_uvw);

}  // namespace fluid3d
}  // namespace FLAG_Race

#endif
