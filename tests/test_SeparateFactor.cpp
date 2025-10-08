#include "form/feature/factor.hpp"

#include "gtsam/geometry/Pose3.h"
#include "gtsam/inference/Symbol.h"
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>

#include "gtest/gtest.h"

using gtsam::symbol_shorthand::X;

#define EXPECT_MATRICES_EQ(M_actual, M_expected)                                    \
  EXPECT_TRUE(M_actual.isApprox(M_expected, 1e-5)) << "  Actual:\n"                 \
                                                   << M_actual << "\nExpected:\n"   \
                                                   << M_expected

#define EXPECT_ZERO(v) EXPECT_TRUE(v.isZero(1e-4)) << " Actual is not zero:\n" << v

using namespace form;

TEST(Separate, PointPoint) {
  gtsam::Pose3 x0 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3), {1, 2, 3});
  gtsam::Pose3 x1 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.4, 0.5, 0.6), {4, 5, 6});

  // Points to compare
  gtsam::Vector3 p_i(1.0, 2.0, 3.0);
  gtsam::Vector3 p_j(4.0, 5.0, 6.0);

  auto mf =
      SeparateFactor(X(0), X(1),
                     {SeparateConstraint(p_i, p_j), SeparateConstraint(p_i, p_j),
                      SeparateConstraint(p_i, p_j), SeparateConstraint(p_i, p_j)},
                     1.0);
  auto pf = PointFactor(X(0), X(1),
                        {PointConstraint(p_i, p_j), PointConstraint(p_i, p_j),
                         PointConstraint(p_i, p_j), PointConstraint(p_i, p_j)},
                        1.0);

  gtsam::Matrix H1_actual, H2_actual, H1_expected, H2_expected;
  gtsam::Vector actual = mf.evaluateError(x0, x1, H1_actual, H2_actual);
  gtsam::Vector expected = pf.evaluateError(x0, x1, H1_expected, H2_expected);

  EXPECT_MATRICES_EQ(H1_actual, H1_expected);
  EXPECT_MATRICES_EQ(H2_actual, H2_expected);
  EXPECT_MATRICES_EQ(actual, expected);
}

TEST(Separate, PlanePoint) {
  gtsam::Pose3 x0 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3), {1, 2, 3});
  gtsam::Pose3 x1 = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.4, 0.5, 0.6), {4, 5, 6});

  // Points to compare
  gtsam::Vector3 p_i(1.0, 2.0, 3.0);
  gtsam::Vector3 n_i(0.0, 0.0, 1.0);
  gtsam::Vector3 p_j(4.0, 5.0, 6.0);

  auto mf = SeparateFactor(
      X(0), X(1), {SeparateConstraint(p_i, n_i, p_j, gtsam::Vector3::Zero())}, 1.0);
  auto pf = PlanarFactor(X(0), X(1), {PlanarConstraint(p_i, n_i, p_j)}, 1.0);

  gtsam::Matrix H1_actual, H2_actual, H1_expected, H2_expected;
  gtsam::Vector actual = mf.evaluateError(x0, x1, H1_actual, H2_actual);
  gtsam::Vector expected = pf.evaluateError(x0, x1, H1_expected, H2_expected);

  EXPECT_MATRICES_EQ(H1_actual, H1_expected);
  EXPECT_MATRICES_EQ(H2_actual, H2_expected);
  EXPECT_MATRICES_EQ(actual, expected);
}