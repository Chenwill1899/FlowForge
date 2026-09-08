#include <fluid/flow_coordinates_3d.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

namespace FLAG_Race { namespace fluid3d {
namespace {
constexpr double minimum_speed = 0.02;
constexpr double potential_tolerance = 1e-8;
constexpr double position_tolerance = 1e-7;

CoordinateStatus3D usable(const CoordinateSample3D& s) {
    if (s.status != CoordinateStatus3D::kOk) return s.status;
    if (!std::isfinite(s.potential) || !s.velocity.allFinite() ||
        !s.potential_gradient.allFinite()) return CoordinateStatus3D::kInvalidInput;
    if (s.velocity.norm() < minimum_speed) return CoordinateStatus3D::kWeakField;
    // Unlike the continuum field, interpolated discrete velocity and the
    // derivative of interpolated potential need not coincide. Check actual
    // transversality instead of assuming d(phi)/dl = |u| numerically.
    const double product = s.potential_gradient.norm() * s.velocity.norm();
    if (product < 1e-12 || s.potential_gradient.dot(s.velocity) <= 1e-3 * product)
        return CoordinateStatus3D::kNonTransverse;
    return CoordinateStatus3D::kOk;
}
}

CoordinateSample3D CoordinateGrid3D::sample(const Eigen::Vector3d& x) const {
    CoordinateSample3D s;
    const auto& g = grid;
    const std::size_t count = static_cast<std::size_t>(g.size());
    if (!x.allFinite() || !far_velocity.allFinite() || !std::isfinite(speed_scale) ||
        speed_scale <= 0.0 || g.h <= 0.0 || g.h_z <= 0.0 ||
        p.size() != count || ux.size() != count || uy.size() != count ||
        uz.size() != count || solid.size() != count) return s;
    const Eigen::Vector2d rel = x.head<2>() - g.origin_xy;
    const double c[3] = {rel.dot(g.e_J)/g.h - 0.5, rel.dot(g.n_J)/g.h - 0.5,
                         (x.z()-g.z_lo)/g.h_z - 0.5};
    const int dims[3] = {g.n_fwd, g.n_lat, g.n_z};
    int base[3], owner[3];
    double t[3];
    for (int d=0; d<3; ++d) {
        if (!std::isfinite(c[d]) || c[d] < 0.0 || c[d] >= dims[d]-1) {
            s.status = CoordinateStatus3D::kOutsideGrid; return s;
        }
        base[d] = static_cast<int>(std::floor(c[d]));
        owner[d] = static_cast<int>(std::floor(c[d]+0.5));
        t[d] = c[d]-base[d];
    }
    if (solid[g.idx(owner[0],owner[1],owner[2])]) {
        s.status = CoordinateStatus3D::kSolid; return s;
    }
    double value=0.0;
    Eigen::Vector3d dp=Eigen::Vector3d::Zero(), v=Eigen::Vector3d::Zero();
    for (int i=0;i<2;++i) for (int j=0;j<2;++j) for (int m=0;m<2;++m) {
        const int k=g.idx(base[0]+i,base[1]+j,base[2]+m);
        const double w[3]={i?t[0]:1-t[0], j?t[1]:1-t[1], m?t[2]:1-t[2]};
        const double weight=w[0]*w[1]*w[2];
        value+=weight*p[k];
        v+=weight*Eigen::Vector3d(ux[k],uy[k],uz[k]);
        dp.x()+=(i?1:-1)*w[1]*w[2]*p[k]/g.h;
        dp.y()+=(j?1:-1)*w[0]*w[2]*p[k]/g.h;
        dp.z()+=(m?1:-1)*w[0]*w[1]*p[k]/g.h_z;
    }
    const Eigen::Vector3d gradp(dp.x()*g.e_J.x()+dp.y()*g.n_J.x(),
                               dp.x()*g.e_J.y()+dp.y()*g.n_J.y(), dp.z());
    s.potential=(far_velocity.dot(x)-value)/speed_scale;
    s.potential_gradient=(far_velocity-gradp)/speed_scale;
    s.velocity=v/speed_scale;
    s.status=CoordinateStatus3D::kOk;
    return s;
}

CoordinateStatus3D FlowCoordinates3D::reset(CoordinateSampler3D sampler,
                                            const Eigen::Vector3d& anchor) {
    sampler_=std::move(sampler);
    anchor_=anchor;
    status_=CoordinateStatus3D::kNoField;
    if (!sampler_ || !anchor.allFinite()) return status_;
    const auto s=sampler_(anchor);
    status_=usable(s);
    if (status_ != CoordinateStatus3D::kOk) return status_;
    anchor_potential_=s.potential;
    const Eigen::Vector3d t=s.velocity.normalized();
    Eigen::Index axis;
    t.cwiseAbs().minCoeff(&axis);
    basis_.col(0)=(Eigen::Vector3d::Unit(axis)-t[axis]*t).normalized();
    basis_.col(1)=t.cross(basis_.col(0)).normalized();
    return status_;
}

CoordinateResult3D FlowCoordinates3D::evaluate(const Eigen::Vector3d& x) const {
    CoordinateResult3D result;
    result.status=status_;
    if (status_ != CoordinateStatus3D::kOk) return result;
    if (!x.allFinite()) { result.status=CoordinateStatus3D::kInvalidInput; return result; }
    Eigen::Vector3d y=x;
    auto current=sampler_(y);
    result.status=usable(current);
    if (result.status != CoordinateStatus3D::kOk) return result;
    double remaining=anchor_potential_-current.potential;
    double step_size=0.025;
    int rejected=0;
    auto rk4 = [&](const Eigen::Vector3d& p, double h, Eigen::Vector3d& next,
                   CoordinateStatus3D& failure) {
        auto derivative = [&](const Eigen::Vector3d& z, Eigen::Vector3d& v) {
            const auto sample=sampler_(z);
            failure=usable(sample);
            if (failure!=CoordinateStatus3D::kOk) return false;
            v=sample.velocity.normalized();
            return true;
        };
        Eigen::Vector3d a,b,c,d;
        if (!derivative(p,a) || !derivative(p+0.5*h*a,b) ||
            !derivative(p+0.5*h*b,c) || !derivative(p+h*c,d)) return false;
        next=p+h*(a+2*b+2*c+d)/6.0;
        return next.allFinite();
    };
    while (std::abs(remaining)>potential_tolerance) {
        if (++result.steps>4096 || result.arc_length>6.0) {
            result.status=CoordinateStatus3D::kArcLimit; return result;
        }
        // Near the section, this local estimate avoids a long bisection.
        const double slope=current.potential_gradient.dot(current.velocity.normalized());
        const double h=std::copysign(std::min(step_size,1.1*std::abs(remaining)/slope),remaining);
        CoordinateStatus3D failure=CoordinateStatus3D::kIntegrationFailed;
        Eigen::Vector3d full,half,next;
        bool ok=rk4(y,h,full,failure) && rk4(y,0.5*h,half,failure) &&
                rk4(half,0.5*h,next,failure);
        auto proposed=ok ? sampler_(next) : CoordinateSample3D{};
        if (ok) { failure=usable(proposed); ok=failure==CoordinateStatus3D::kOk; }
        const double error=ok ? (next-full).norm()/15.0 : 1.0;
        if (!ok || error>position_tolerance) {
            step_size*=0.5;
            if (++rejected>30 || step_size<1e-8) {
                result.status=ok?CoordinateStatus3D::kIntegrationFailed:failure; return result;
            }
            continue;
        }
        double next_remaining=anchor_potential_-proposed.potential;
        if (remaining*next_remaining<0.0) {
            double lo=0.0,hi=h;
            bool converged=false;
            for (int it=0;it<40;++it) {
                const double mid=0.5*(lo+hi);
                if (!rk4(y,0.5*mid,half,failure) || !rk4(half,0.5*mid,next,failure)) {
                    result.status=failure; return result;
                }
                proposed=sampler_(next);
                failure=usable(proposed);
                if (failure!=CoordinateStatus3D::kOk) { result.status=failure; return result; }
                next_remaining=anchor_potential_-proposed.potential;
                if (std::abs(next_remaining)<=potential_tolerance) { converged=true; break; }
                if (remaining*next_remaining>0.0) lo=mid; else hi=mid;
            }
            if (!converged) { result.status=CoordinateStatus3D::kIntegrationFailed; return result; }
        }
        if (std::abs(next_remaining)>=std::abs(remaining) && remaining*next_remaining>0) {
            result.status=CoordinateStatus3D::kNonTransverse; return result;
        }
        rejected=0;
        result.arc_length+=(next-y).norm();
        y=next; current=proposed; remaining=next_remaining;
        step_size=std::min(0.025,step_size*1.5);
    }
    if ((y-anchor_).norm()>1.5) { result.status=CoordinateStatus3D::kOutsideChart; return result; }
    result.eta=basis_.transpose()*(y-anchor_);
    result.section_point=y;
    result.section_residual=std::abs(remaining);
    result.status=CoordinateStatus3D::kOk;
    return result;
}

FlowControlResult3D FlowCoordinates3D::nominalVelocity(
    const Eigen::Vector3d& x, double tangential_speed, const Eigen::Matrix2d& gain) const {
    FlowControlResult3D out;
    if (!std::isfinite(tangential_speed) || tangential_speed < 0.0 || !gain.allFinite() ||
        (gain-gain.transpose()).norm() > 1e-12 ||
        gain.selfadjointView<Eigen::Lower>().llt().info() != Eigen::Success) {
        out.status=CoordinateStatus3D::kInvalidInput; return out;
    }
    out.coordinate=evaluate(x);
    out.status=out.coordinate.status;
    if (!out.coordinate.valid()) return out;
    constexpr double h=0.002;
    Eigen::Matrix<double,2,3> coarse,fine;
    for (int axis=0;axis<3;++axis) {
        const Eigen::Vector3d offset=h*Eigen::Vector3d::Unit(axis);
        const auto a=evaluate(x+offset), b=evaluate(x-offset);
        const auto c=evaluate(x+0.5*offset), d=evaluate(x-0.5*offset);
        for (const auto* r : {&a,&b,&c,&d}) {
            if (!r->valid()) { out.status=r->status; return out; }
        }
        coarse.col(axis)=(a.eta-b.eta)/(2*h);
        fine.col(axis)=(c.eta-d.eta)/h;
    }
    out.status=CoordinateStatus3D::kJacobianFailed;
    out.fd_difference=(fine-coarse).norm()/std::max(1e-12,fine.norm());
    if (!fine.allFinite() || out.fd_difference>0.01) return out;
    Eigen::JacobiSVD<Eigen::Matrix<double,2,3>> svd(fine);
    const auto sigma=svd.singularValues();
    out.minimum_sigma=sigma[1]; out.condition=sigma[0]/sigma[1];
    if (out.minimum_sigma<1e-4 || !std::isfinite(out.condition) || out.condition>1e3) return out;
    out.jacobian=fine;
    const Eigen::Matrix2d gram=fine*fine.transpose();
    const Eigen::LDLT<Eigen::Matrix2d> factor(gram);
    if (factor.info()!=Eigen::Success || !factor.isPositive()) return out;
    out.pseudoinverse=fine.transpose()*factor.solve(Eigen::Matrix2d::Identity());
    out.inverse_residual=(fine*out.pseudoinverse-Eigen::Matrix2d::Identity()).norm();
    const auto s=sampler_(x);
    out.ju_residual=(fine*s.velocity).norm();
    if (!out.pseudoinverse.allFinite() || out.inverse_residual>1e-8 ||
        out.ju_residual>1e-3*std::max(1.0,fine.norm())*s.velocity.norm()) return out;
    const Eigen::Vector3d v=tangential_speed*s.velocity.normalized()-
                               out.pseudoinverse*gain*out.coordinate.eta;
    if (!v.allFinite()) return out;
    out.velocity=v;
    out.decay_residual=(fine*v+gain*out.coordinate.eta).norm();
    out.status=CoordinateStatus3D::kOk;
    return out;
}

const char* coordinateStatusName3D(CoordinateStatus3D s) {
    switch(s) {
    case CoordinateStatus3D::kOk:return "ok";
    case CoordinateStatus3D::kNoField:return "no_field";
    case CoordinateStatus3D::kOutsideGrid:return "outside_grid";
    case CoordinateStatus3D::kSolid:return "solid";
    case CoordinateStatus3D::kWeakField:return "weak_field";
    case CoordinateStatus3D::kNonTransverse:return "non_transverse";
    case CoordinateStatus3D::kIntegrationFailed:return "integration_failed";
    case CoordinateStatus3D::kArcLimit:return "arc_limit";
    case CoordinateStatus3D::kOutsideChart:return "outside_chart";
    case CoordinateStatus3D::kInvalidInput:return "invalid_input";
    case CoordinateStatus3D::kJacobianFailed:return "jacobian_failed";
    }
    return "unknown";
}
} }
