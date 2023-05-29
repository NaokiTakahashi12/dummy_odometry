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

#include <dummy_odometry_node_parameters.hpp>

#include "../simple_dynamics_parameters.hpp"

namespace dummy_odometry
{
template<typename Scalar>
void loadSimpleDynamicsParam(
  const dummy_odometry_node::Params & params,
  SimpleDynamicsParameters<Scalar> & simple_dynamics_params)
{
  for (int i = 0; i < 6; ++i) {
    simple_dynamics_params.mdk_params[i].feedback_gains[0] =
      params.simple_dynamics.feedback_gains.k1;
    simple_dynamics_params.mdk_params[i].feedback_gains[1] =
      params.simple_dynamics.feedback_gains.k2;
  }
  for (int i = 0; i < 3; ++i) {
    simple_dynamics_params.mdk_params[i].inertial =
      params.simple_dynamics.mass;
  }
  for (int i = 3; i < 6; ++i) {
    simple_dynamics_params.mdk_params[i].inertial =
      params.simple_dynamics.inertia;
  }
  {
    simple_dynamics_params.mdk_params[0].spring =
      params.simple_dynamics.spring.x;
    simple_dynamics_params.mdk_params[1].spring =
      params.simple_dynamics.spring.y;
    simple_dynamics_params.mdk_params[2].spring =
      params.simple_dynamics.spring.z;
    simple_dynamics_params.mdk_params[3].spring =
      params.simple_dynamics.spring.roll;
    simple_dynamics_params.mdk_params[4].spring =
      params.simple_dynamics.spring.pitch;
    simple_dynamics_params.mdk_params[5].spring =
      params.simple_dynamics.spring.yaw;
  }
  {
    simple_dynamics_params.mdk_params[0].damper =
      params.simple_dynamics.damper.x;
    simple_dynamics_params.mdk_params[1].damper =
      params.simple_dynamics.damper.y;
    simple_dynamics_params.mdk_params[2].damper =
      params.simple_dynamics.damper.z;
    simple_dynamics_params.mdk_params[3].damper =
      params.simple_dynamics.damper.roll;
    simple_dynamics_params.mdk_params[4].damper =
      params.simple_dynamics.damper.pitch;
    simple_dynamics_params.mdk_params[5].damper =
      params.simple_dynamics.damper.yaw;
  }
}

template<typename Scalar>
SimpleDynamicsParameters<Scalar> loadSimpleDynamicsParam(
  const dummy_odometry_node::Params & params)
{
  SimpleDynamicsParameters<Scalar> simple_dynamics_params;
  loadSimpleDynamicsParam(params, simple_dynamics_params);
  return simple_dynamics_params;
}
}  // namespace dummy_odometry
