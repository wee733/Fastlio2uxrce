#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "gravity_alignment.hpp"

TEST(GravityAlignment, TiltedStartHasVerticalGravity)
{
  for (double roll : {-0.7, 0.0, 0.4}) {
    for (double pitch : {-0.5, 0.0, 0.6}) {
      Eigen::Quaterniond body_to_world(
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
      const Eigen::Vector3d body_force =
        body_to_world.conjugate() * Eigen::Vector3d(0.0, 0.0, 9.81);
      const Eigen::Quaterniond aligned = gravity_alignment(body_force);
      EXPECT_NEAR((aligned * body_force - Eigen::Vector3d(0.0, 0.0, 9.81)).norm(), 0.0, 1e-10);
      EXPECT_NEAR(aligned.norm(), 1.0, 1e-10);
    }
  }
}

TEST(GravityAlignment, LevelStartPreservesAxes)
{
  const Eigen::Quaterniond aligned = gravity_alignment(Eigen::Vector3d(0.0, 0.0, 9.81));
  EXPECT_NEAR((aligned * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX()).norm(), 0.0, 1e-10);
}

TEST(GravityAlignment, RejectsInvalidForce)
{
  EXPECT_THROW(gravity_alignment(Eigen::Vector3d::Zero()), std::invalid_argument);
  EXPECT_THROW(gravity_alignment(Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)), std::invalid_argument);
}
