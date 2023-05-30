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

#include <dummy_odometry/state_space/mdk.hpp>

#include <stdexcept>


namespace dummy_odometry::state_space
{
template<typename Scalar>
MDK<Scalar>::MDK(const MDKParameters<Scalar> & params)
: m_delta_time(1e-3),
  m_init_state(),
  m_state(),
  m_state_dot(),
  m_A(),
  m_B(),
  m_C(),
  m_D(),
  m_K()
{
  this->resetStateSpace(params);
}

template<typename Scalar>
MDK<Scalar>::~MDK() {}

template<typename Scalar>
void MDK<Scalar>::setDeltaTime(const Scalar delta_time)
{
  if (delta_time <= 0) {
    throw std::runtime_error("Please set delta_time > 0");
  }
  m_delta_time = delta_time;
}

template<typename Scalar>
Scalar MDK<Scalar>::getDeltaTime() const
{
  return m_delta_time;
}

template<typename Scalar>
void MDK<Scalar>::setFeedthrough(const Scalar D)
{
  m_D = D;
}

template<typename Scalar>
Scalar MDK<Scalar>::getFeedthrough() const
{
  return m_D;
}

template<typename Scalar>
Scalar MDK<Scalar>::updateStateFromVelocity(const Scalar velocity)
{
  const Scalar dest_force = (m_inertial * velocity) / m_delta_time;
  return updateStateFromForce(dest_force);
}

template<typename Scalar>
Scalar MDK<Scalar>::updateStateFromForce(const Scalar force)
{
  const Scalar u = force - m_K * m_state;
  m_state_dot = m_A * m_state + m_B * u;
  m_state = m_delta_time * m_state_dot + m_state;
  return m_C * m_state + m_D;
}

template<typename Scalar>
void MDK<Scalar>::resetStateSpace(const MDKParameters<Scalar> & params)
{
  const Scalar reciprocal_inertial = 1 / params.inertial;
  m_inertial = params.inertial;
  m_init_state.fill(0.0);
  m_A << 0, 1, -params.spring * reciprocal_inertial, -params.damper * reciprocal_inertial;
  m_B << 0, reciprocal_inertial;
  m_C << 1, 0;
  m_D = 0;
  m_K(0) = params.feedback_gains[0];
  m_K(1) = params.feedback_gains[1];
  resetState();
}

template<typename Scalar>
void MDK<Scalar>::resetState()
{
  m_state = m_init_state;
}

template class MDK<float>;
template class MDK<double>;
}  // namespace dummy_odometry::state_space
