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

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <array>
#include <mutex>

#include "simple_dynamics_parameters.hpp"
#include "state_space/mdk_parameters.hpp"
#include "state_space/mdk.hpp"


namespace dummy_odometry
{
template<typename Scalar>
class SimpleDynamics
{
public:
  using Vector = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using Pose = Eigen::Matrix<Scalar, 6, 1>;

  explicit SimpleDynamics(const SimpleDynamicsParameters<Scalar> &);

  void setDestVelocity(const Vector &);
  void setDestAngularVelocity(const Quaternion &);

  void setDeltaTime(const Scalar);

  Vector getPosition();
  Quaternion getAngularPosition();

  void updateOdometry();

private:
  static constexpr unsigned int pose_size = 6;

  std::mutex m_command_mutex;

  Pose m_dest_pose_velocity;
  Vector m_position;
  Quaternion m_quaternion;

  std::array<std::unique_ptr<state_space::MDK<Scalar>>, pose_size> m_pose_state_space;
};
}  // namespace dummy_odometry
