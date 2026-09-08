#ifndef FLOW_COORDINATES_3D_H
#define FLOW_COORDINATES_3D_H

#include <functional>
#include <limits>
#include <Eigen/Dense>
#include <fluid/fluid_solver_3d.h>

namespace FLAG_Race { namespace fluid3d {

enum class CoordinateStatus3D {
    kOk = 0, kNoField, kOutsideGrid, kSolid, kWeakField,
    kNonTransverse, kIntegrationFailed, kArcLimit, kOutsideChart,
    kInvalidInput, kJacobianFailed
};
const char* coordinateStatusName3D(CoordinateStatus3D status);

struct CoordinateSample3D {
    double potential = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d potential_gradient = Eigen::Vector3d::Zero();
    CoordinateStatus3D status = CoordinateStatus3D::kNoField;
};
using CoordinateSampler3D = std::function<CoordinateSample3D(const Eigen::Vector3d&)>;

// Independent snapshot of the existing numerical field. p is the solver's
// perturbation potential, NOT the total potential used to define the section.
struct CoordinateGrid3D {
    Grid3D grid;
    std::vector<double> p, ux, uy, uz;
    std::vector<uint8_t> solid;
    Eigen::Vector3d far_velocity = Eigen::Vector3d::Zero();
    double speed_scale = 1.0;
    CoordinateSample3D sample(const Eigen::Vector3d& x) const;
};

struct CoordinateResult3D {
    Eigen::Vector2d eta = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d section_point = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double section_residual = std::numeric_limits<double>::quiet_NaN();
    double arc_length = 0.0;
    int steps = 0;
    CoordinateStatus3D status = CoordinateStatus3D::kNoField;
    bool valid() const { return status == CoordinateStatus3D::kOk; }
};

struct FlowControlResult3D {
    CoordinateResult3D coordinate;
    Eigen::Matrix<double, 2, 3> jacobian = Eigen::Matrix<double, 2, 3>::Zero();
    Eigen::Matrix<double, 3, 2> pseudoinverse = Eigen::Matrix<double, 3, 2>::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double fd_difference = std::numeric_limits<double>::quiet_NaN();
    double minimum_sigma = std::numeric_limits<double>::quiet_NaN();
    double condition = std::numeric_limits<double>::quiet_NaN();
    double ju_residual = std::numeric_limits<double>::quiet_NaN();
    double inverse_residual = std::numeric_limits<double>::quiet_NaN();
    double decay_residual = std::numeric_limits<double>::quiet_NaN();
    CoordinateStatus3D status = CoordinateStatus3D::kNoField;
    bool valid() const { return status == CoordinateStatus3D::kOk; }
};

// Fixed sampler, anchor, section and basis for exactly one field interval.
// No nearest-reference projection occurs in evaluate().
class FlowCoordinates3D {
public:
    CoordinateStatus3D reset(CoordinateSampler3D sampler, const Eigen::Vector3d& anchor);
    CoordinateResult3D evaluate(const Eigen::Vector3d& x) const;
    FlowControlResult3D nominalVelocity(const Eigen::Vector3d& x, double tangential_speed,
                                         const Eigen::Matrix2d& gain) const;
    const Eigen::Vector3d& anchor() const { return anchor_; }
    const Eigen::Matrix<double, 3, 2>& basis() const { return basis_; }
    double anchorPotential() const { return anchor_potential_; }
    CoordinateStatus3D status() const { return status_; }
private:
    CoordinateSampler3D sampler_;
    Eigen::Vector3d anchor_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 2> basis_ = Eigen::Matrix<double, 3, 2>::Zero();
    double anchor_potential_ = 0.0;
    CoordinateStatus3D status_ = CoordinateStatus3D::kNoField;
};

} }
#endif
