// MIT License
//
// Copyright (c) 2023 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <dummy_odometry/simple_dynamics.hpp>

#include <cmath>

#include <memory>

#include <dummy_odometry/simple_dynamics_parameters.hpp>


namespace dummy_odometry
{
template<typename Scalar>
SimpleDynamics<Scalar>::SimpleDynamics(
  const SimpleDynamicsParameters<Scalar> & param)
: m_command_mutex(),
  m_dest_pose_velocity(),
  m_position(),
  m_quaternion(),
  m_pose_state_space()
{
  m_dest_pose_velocity = m_dest_pose_velocity.Zero();
  m_position = m_position.Zero();
  m_quaternion.x() = 0;
  m_quaternion.y() = 0;
  m_quaternion.z() = 0;
  m_quaternion.w() = 1;
  for (unsigned int i = 0; i < pose_size; ++i) {
    m_pose_state_space[i] = std::make_unique<state_space::MDK<Scalar>>(
      param.mdk_params[i]
    );
  }
}

template<typename Scalar>
void SimpleDynamics<Scalar>::setDestVelocity(const Vector & dest_velocity)
{
  std::lock_guard<std::mutex> lock{m_command_mutex};
  m_dest_pose_velocity.block(0, 0, 3, 1) = dest_velocity;
}


//! Reference from https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
template<typename Scalar>
void SimpleDynamics<Scalar>::setDestAngularVelocity(const Quaternion & dest_angular_velocity)
{
  std::lock_guard<std::mutex> lock{m_command_mutex};
  const Scalar sinr_cosp = 2 * (
    dest_angular_velocity.w() * dest_angular_velocity.x() +
    dest_angular_velocity.y() * dest_angular_velocity.z()
  );
  const Scalar cosr_cosp = 1 - 2 * (
    std::pow(dest_angular_velocity.x(), 2) +
    std::pow(dest_angular_velocity.y(), 2)
  );
  const Scalar sinp = 2 * (
    dest_angular_velocity.w() * dest_angular_velocity.y() -
    dest_angular_velocity.z() * dest_angular_velocity.x()
  );
  const Scalar siny_cosp = 2 * (
    dest_angular_velocity.w() * dest_angular_velocity.z() +
    dest_angular_velocity.x() * dest_angular_velocity.y()
  );
  const Scalar cosy_cosp = 1 - 2 * (
    std::pow(dest_angular_velocity.y(), 2) +
    std::pow(dest_angular_velocity.z(), 2)
  );
  m_dest_pose_velocity(3) = std::atan2(sinr_cosp, cosr_cosp);
  if (std::abs(sinp) >= 1) {
    m_dest_pose_velocity(4) = std::copysign(M_PI / 2, sinp);
  } else {
    m_dest_pose_velocity(4) = std::asin(sinp);
  }
  m_dest_pose_velocity(5) = std::atan2(siny_cosp, cosy_cosp);
}

template<typename Scalar>
void SimpleDynamics<Scalar>::setDeltaTime(const Scalar delta_time)
{
  std::lock_guard<std::mutex> lock{m_command_mutex};
  for (auto && pss : m_pose_state_space) {
    if (not pss) {
      throw std::runtime_error("m_pose_state_space has null");
    }
    pss->setDeltaTime(delta_time);
  }
}

template<typename Scalar>
SimpleDynamics<Scalar>::Vector SimpleDynamics<Scalar>::getPosition()
{
  std::lock_guard<std::mutex> lock{m_command_mutex};
  return m_position;
}

template<typename Scalar>
SimpleDynamics<Scalar>::Quaternion SimpleDynamics<Scalar>::getAngularPosition()
{
  std::lock_guard<std::mutex> lock{m_command_mutex};
  return m_quaternion;
}

template<typename Scalar>
void SimpleDynamics<Scalar>::updateOdometry()
{
  static Pose pose{};
  static Pose past_pose{};
  static Vector diff_position{};
  std::lock_guard<std::mutex> lock{m_command_mutex};
  unsigned int i = 0;
  for (auto && pss : m_pose_state_space) {
    if (not pss) {
      throw std::runtime_error("m_pose_state_space has null");
    }
    pose(i) = pss->updateStateFromVelocity(m_dest_pose_velocity(i));
    i++;
  }
  m_quaternion = Eigen::AngleAxis<Scalar>(pose(5), Vector::UnitZ());
  m_quaternion = m_quaternion * Eigen::AngleAxis<Scalar>(pose(4), Vector::UnitY());
  m_quaternion = m_quaternion * Eigen::AngleAxis<Scalar>(pose(3), Vector::UnitX());
  diff_position = pose.block(0, 0, 3, 1) - past_pose.block(0, 0, 3, 1);
  past_pose = pose;
  m_position += m_quaternion * diff_position;
}

template class SimpleDynamics<float>;
template class SimpleDynamics<double>;
}  // namespace dummy_odometry
