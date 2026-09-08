#include <fluid/flow_coordinates_3d.h>
#include <fluid/fluid_guidance.h>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>

using namespace FLAG_Race::fluid3d;
namespace {
CoordinateSample3D sample(double phi, const Eigen::Vector3d& u) {
    CoordinateSample3D s;
    s.potential=phi; s.velocity=u; s.potential_gradient=u;
    s.status=CoordinateStatus3D::kOk; return s;
}
TEST(FlowCoordinates, UniformObliqueAndVertical) {
    for (const Eigen::Vector3d q : {Eigen::Vector3d(1,2,3).normalized(),Eigen::Vector3d(0,0,1)}) {
        FlowCoordinates3D chart;
        const Eigen::Vector3d anchor(0.1,-0.2,0.3);
        ASSERT_EQ(chart.reset([q](const Eigen::Vector3d& x) {return sample(q.dot(x),q);},anchor),CoordinateStatus3D::kOk);
        EXPECT_DOUBLE_EQ(chart.evaluate(anchor).eta.norm(),0.0);
        const Eigen::Vector2d eta(0.15,-0.12);
        for (double along : {-0.4,0.0,0.4}) {
            const auto r=chart.evaluate(anchor+along*q+chart.basis()*eta);
            ASSERT_TRUE(r.valid()) << coordinateStatusName3D(r.status);
            EXPECT_LE((r.eta-eta).norm(),1e-7);
            EXPECT_LE(r.section_residual,1e-8);
        }
    }
}
TEST(FlowCoordinates, CurvedHarmonicReturnUsesSameSection) {
    FlowCoordinates3D chart;
    auto field=[](const Eigen::Vector3d& x) {
        return sample(0.5*(x.x()*x.x()-x.y()*x.y()),Eigen::Vector3d(x.x(),-x.y(),0));
    };
    const Eigen::Vector3d anchor(1,1,0);
    ASSERT_EQ(chart.reset(field,anchor),CoordinateStatus3D::kOk);
    double max_error=0,max_residual=0,max_ju=0;
    for (double x : {0.8,1.2,1.5}) {
        const Eigen::Vector3d query(x,1.21/x,0.12);
        const auto r=chart.evaluate(query);
        ASSERT_TRUE(r.valid()) << coordinateStatusName3D(r.status);
        const Eigen::Vector3d expected_section(1.1,1.1,0.12);
        max_error=std::max(max_error,(r.section_point-expected_section).norm());
        max_residual=std::max(max_residual,r.section_residual);
        EXPECT_LE((r.eta-chart.basis().transpose()*(expected_section-anchor)).norm(),2e-7);
        Eigen::Matrix<double,2,3> j;
        for(int d=0;d<3;++d) {
            const Eigen::Vector3d offset=1e-3*Eigen::Vector3d::Unit(d);
            auto plus=chart.evaluate(query+offset),minus=chart.evaluate(query-offset);
            ASSERT_TRUE(plus.valid() && minus.valid());
            j.col(d)=(plus.eta-minus.eta)/0.002;
        }
        max_ju=std::max(max_ju,(j*field(query).velocity).norm());
    }
    EXPECT_LE(max_error,2e-7); EXPECT_LE(max_ju,2e-5);
    std::cout << "FLOW_ERROR_METRIC return_error="<<max_error
              <<" section_residual="<<max_residual<<" Ju="<<max_ju<<'\n';
}
TEST(FlowCoordinates, ChecksActualTransversalityOfNumericalField) {
    FlowCoordinates3D chart;
    // Numerical velocity need not equal grad(phi). Return along the velocity,
    // not the gradient: y exp(-x) is the first integral of (1,y,0).
    ASSERT_EQ(chart.reset([](const Eigen::Vector3d& x) {
        auto s=sample(x.x(),Eigen::Vector3d(1,x.y(),0));
        s.potential_gradient=Eigen::Vector3d::UnitX(); return s;
    },Eigen::Vector3d::Zero()),CoordinateStatus3D::kOk);
    auto r=chart.evaluate(Eigen::Vector3d(0.4,0.2,0.1));
    ASSERT_TRUE(r.valid());
    EXPECT_NEAR(r.section_point.y(),0.2*std::exp(-0.4),1e-7);
    EXPECT_NEAR(r.section_point.z(),0.1,1e-9);
    EXPECT_EQ(chart.reset([](const Eigen::Vector3d& x) {
        auto s=sample(x.x(),Eigen::Vector3d::UnitY());
        s.potential_gradient=Eigen::Vector3d::UnitX(); return s;
    },Eigen::Vector3d::Zero()),CoordinateStatus3D::kNonTransverse);
}
TEST(FlowCoordinates, FailuresAreNotZeroErrors) {
    FlowCoordinates3D chart;
    EXPECT_FALSE(chart.evaluate(Eigen::Vector3d::Zero()).valid());
    EXPECT_EQ(chart.reset([](const Eigen::Vector3d& x) {return sample(x.x(),Eigen::Vector3d::Zero());},
                          Eigen::Vector3d::Zero()),CoordinateStatus3D::kWeakField);
    EXPECT_EQ(chart.reset([](const Eigen::Vector3d& x) {
        auto s=sample(x.x(),Eigen::Vector3d::UnitX());
        if(x.x()>0.1 && x.x()<0.2) s.status=CoordinateStatus3D::kSolid;
        return s;
    },Eigen::Vector3d::Zero()),CoordinateStatus3D::kOk);
    const auto r=chart.evaluate(Eigen::Vector3d(0.3,0.1,0));
    EXPECT_FALSE(r.valid()); EXPECT_TRUE(r.eta.array().isNaN().all());
}
TEST(FlowCoordinates, GridUsesTotalPotentialAndMatchingWorldFrame) {
    CoordinateGrid3D f;
    f.grid.n_fwd=f.grid.n_lat=f.grid.n_z=8;
    f.grid.h=f.grid.h_z=0.2;
    f.grid.e_J=Eigen::Vector2d(0,-1); f.grid.n_J=Eigen::Vector2d(1,0);
    f.grid.origin_xy=Eigen::Vector2d(1,2);
    f.far_velocity=Eigen::Vector3d(0,-2,0);f.speed_scale=2;
    f.p.resize(f.grid.size());f.ux.assign(f.grid.size(),-0.2);
    f.uy.assign(f.grid.size(),-2);f.uz.assign(f.grid.size(),-0.1);
    f.solid.assign(f.grid.size(),0);
    for(int i=0;i<8;++i)for(int j=0;j<8;++j)for(int k=0;k<8;++k) {
        const auto x=f.grid.cellWorld(i,j,k);
        f.p[f.grid.idx(i,j,k)]=0.2*x.x()+0.1*x.z();
    }
    const Eigen::Vector3d x(1.7,1.1,0.6);
    auto s=f.sample(x);
    ASSERT_EQ(s.status,CoordinateStatus3D::kOk);
    EXPECT_NEAR(s.potential,(-2*x.y()-0.2*x.x()-0.1*x.z())/2,1e-12);
    EXPECT_LE((s.potential_gradient-s.velocity).norm(),1e-12);
    f.solid[f.grid.idx(4,3,2)]=1;
    EXPECT_EQ(f.sample(f.grid.cellWorld(4,3,2)).status,CoordinateStatus3D::kSolid);
}
TEST(FlowCoordinates, FullTwoDimensionalGainAndZeroTangentialSpeed) {
    FlowCoordinates3D chart;
    ASSERT_EQ(chart.reset([](const Eigen::Vector3d& x) {
        return sample(x.x(),Eigen::Vector3d::UnitX());
    },Eigen::Vector3d::Zero()),CoordinateStatus3D::kOk);
    const Eigen::Matrix2d gain=(Eigen::Matrix2d()<<2,0.3,0.3,1).finished();
    const Eigen::Vector3d x(0.2,0.1,-0.15);
    const auto r=chart.nominalVelocity(x,0.0,gain);
    ASSERT_TRUE(r.valid());
    EXPECT_GT(r.velocity.tail<2>().norm(),0.1);
    EXPECT_LE((r.jacobian*r.velocity+gain*r.coordinate.eta).norm(),1e-10);
    EXPECT_LE(r.inverse_residual,1e-12);
    const auto invalid=chart.nominalVelocity(x,0,Eigen::Matrix2d::Zero());
    EXPECT_FALSE(invalid.valid()); EXPECT_TRUE(invalid.velocity.isZero(0));
}
TEST(FlowCoordinates, CurvedFieldControlHasIndependentIdealDecay) {
    FlowCoordinates3D chart;
    ASSERT_EQ(chart.reset([](const Eigen::Vector3d& x) {
        return sample(0.5*(x.x()*x.x()-x.y()*x.y()),Eigen::Vector3d(x.x(),-x.y(),0));
    },Eigen::Vector3d(1,1,0)),CoordinateStatus3D::kOk);
    const Eigen::Matrix2d gain=(Eigen::Matrix2d()<<1.2,0.2,0.2,0.8).finished();
    const Eigen::Vector3d x(1.2,1.05,0.1);
    const auto r=chart.nominalVelocity(x,0.4,gain);
    ASSERT_TRUE(r.valid()) << coordinateStatusName3D(r.status);
    const double dt=1e-4;
    const auto next=chart.evaluate(x+dt*r.velocity);
    ASSERT_TRUE(next.valid());
    const double decay=((next.eta-r.coordinate.eta)/dt+gain*r.coordinate.eta).norm();
    EXPECT_LE(decay,1e-4);
    EXPECT_LE(r.ju_residual,1e-5);
    EXPECT_LE(r.inverse_residual,1e-10);
    std::cout << "FLOW_CONTROL_METRIC Ju="<<r.ju_residual<<" JJdag="<<r.inverse_residual
              <<" ideal_decay_residual="<<decay<<'\n';
}
TEST(FlowCoordinates, GuidanceObservesFrozenAnchorWithoutChangingPlanarInterface) {
    FLAG_Race::fluid::FluidGuidance g;
    FLAG_Race::fluid::FluidGuidanceConfig c;
    c.z_min=-1;c.z_max=1;c.floor_band=0;c.ceil_band=0;
    c.fine_forward_size=c.fine_lateral_size=2.4;
    c.window_forward_size=c.window_lateral_size=3.0;
    c.resolve_period=0.1;
    g.setConfig(c);g.setDistanceQuery([](const Eigen::Vector3d&) {return 10.0;});
    g.calcGuidance3D(Eigen::Vector3d::Zero(),Eigen::Vector2d::Zero(),0,0.3,1.0,false);
    auto first=g.flowErrorDiagnostics3D();
    ASSERT_TRUE(first.coordinate.valid());EXPECT_LE(first.coordinate.eta.norm(),1e-8);
    g.calcGuidance3D(Eigen::Vector3d(0.01,0.02,0.03),Eigen::Vector2d::Zero(),0,0.3,1.02,false);
    auto next=g.flowErrorDiagnostics3D();
    ASSERT_TRUE(next.coordinate.valid());
    EXPECT_EQ(next.field_version,first.field_version);
    EXPECT_EQ((next.anchor-first.anchor).norm(),0.0);
    EXPECT_NEAR(next.coordinate.eta.norm(),std::hypot(0.02,0.03),1e-7);
    EXPECT_TRUE(g.calcGuidance2D(Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero(),0,0.3,2.0,false).allFinite());
}
}
