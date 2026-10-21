#ifndef FAST_LIO__GRAVITY_ALIGNMENT_HPP_
#define FAST_LIO__GRAVITY_ALIGNMENT_HPP_

#include <stdexcept>
#include <Eigen/Geometry>

inline Eigen::Quaterniond gravity_alignment(const Eigen::Vector3d & specific_force)
{
  if (!specific_force.allFinite() || specific_force.norm() < 1e-6) {
    throw std::invalid_argument("Invalid initial accelerometer mean");
  }
  return Eigen::Quaterniond::FromTwoVectors(specific_force, Eigen::Vector3d::UnitZ());
}

#endif  // FAST_LIO__GRAVITY_ALIGNMENT_HPP_
