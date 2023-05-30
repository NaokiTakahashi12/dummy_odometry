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

#include <memory>

#include <Eigen/Dense>

#include "mdk_parameters.hpp"


namespace dummy_odometry::state_space
{
template<typename Scalar>
class MDK
{
public:
  MDK(const MDKParameters<Scalar> &);
  ~MDK();

  void setDeltaTime(const Scalar delta_time);
  Scalar getDeltaTime() const;

  void setFeedthrough(const Scalar D);
  Scalar getFeedthrough() const;

  Scalar updateStateFromVelocity(const Scalar velocity);
  Scalar updateStateFromForce(const Scalar force);

  void resetStateSpace(const MDKParameters<Scalar> &);
  void resetState();

private:
  static constexpr int state_dimention = 2;

  Scalar m_delta_time;
  Scalar m_inertial;

  Eigen::Matrix<Scalar, state_dimention, 1> m_init_state;
  Eigen::Matrix<Scalar, state_dimention, 1> m_state;
  Eigen::Matrix<Scalar, state_dimention, 1> m_state_dot;

  Eigen::Matrix<Scalar, state_dimention, state_dimention> m_A;
  Eigen::Matrix<Scalar, state_dimention, 1> m_B;
  Eigen::Matrix<Scalar, 1, state_dimention> m_C;
  Scalar m_D;
  Eigen::Matrix<Scalar, 1, state_dimention> m_K;
};
}  // namespace dummy_odometry::state_space
