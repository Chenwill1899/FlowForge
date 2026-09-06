#ifndef _CBF_PROJECTION_H
#define _CBF_PROJECTION_H

// Closed-form Control Barrier Function projection (paper VIII, eq. 77 with
// unit weight matrix and a single active constraint, linear class-K
// alpha(h) = k_h*h):
//
//     safety set:      C = { x : h(x) = d(x) - d_min >= 0 }
//     constraint:      grad(h)^T v >= -k_h * h
//     projection:      v* = argmin 1/2 |v - v_nom|^2  s.t.  grad^T v >= -k_h h
//
// For a single constraint the Euclidean (unit-weight) solution is closed
// form: v* = v_nom + ((rhs - lhs)/|grad|^2) * grad when violated, i.e. the
// minimal-norm correction along the gradient. This keeps the safety set
// forward-invariant for the holonomic model: on the boundary h = 0 any
// commanded inward motion is removed and replaced by exactly the recovery
// rate -k_h*h.
//
// The function is intentionally header-only and dependency-free (Eigen
// only) so the gtest can exercise the exact code path the node uses.

#include <Eigen/Dense>

namespace FLAG_Race {

// Projects v onto the half-space grad^T v >= -k_h*h. Returns true if a
// correction was applied (constraint was violated before projection).
// grad_eps guards the division when the gradient degenerates (flat ESDF,
// outside the map); the caller then owns the degrade policy (hard stop).
template <typename Vec>
inline bool projectCbf(Vec& v, const Vec& grad, double h, double k_h,
                       double grad_eps = 1e-9)
{
    const double lhs = grad.dot(v);
    const double rhs = -k_h * h;
    if (lhs >= rhs) return false;             // constraint already satisfied
    const double gn2 = grad.squaredNorm();
    if (!(gn2 > grad_eps)) return false;      // no usable gradient
    v += ((rhs - lhs) / gn2) * grad;
    return true;
}

// Re-check after projection (and after any downstream clamp): returns true
// if the constraint holds.
template <typename Vec>
inline bool cbfSatisfied(const Vec& v, const Vec& grad, double h, double k_h,
                         double tol = 1e-9)
{
    return grad.dot(v) + k_h * h >= -tol;
}

}  // namespace FLAG_Race

#endif
