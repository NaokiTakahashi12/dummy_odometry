#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>


namespace dummy_odometry
{
template<typename Scalar>
void eigenVectorFromRosVector3(
  const geometry_msgs::msg::Vector3 & msg,
  Eigen::Matrix<Scalar, 3, 1> & vec)
{
  vec.x() = static_cast<Scalar>(msg.x);
  vec.y() = static_cast<Scalar>(msg.y);
  vec.z() = static_cast<Scalar>(msg.z);
}

template<typename Scalar>
void eigenVectorFromRosPoint3(
  const geometry_msgs::msg::Point & msg,
  Eigen::Matrix<Scalar, 3, 1> & vec)
{
  vec.x() = static_cast<Scalar>(msg.x);
  vec.y() = static_cast<Scalar>(msg.y);
  vec.z() = static_cast<Scalar>(msg.z);
}

template<typename Scalar>
void rosVector3FromEigenVector3(
  const Eigen::Matrix<Scalar, 3, 1> & vec,
  geometry_msgs::msg::Vector3 & msg)
{
  msg.x = static_cast<double>(vec.x());
  msg.y = static_cast<double>(vec.y());
  msg.z = static_cast<double>(vec.z());
}

template<typename Scalar>
void rosVector3FromEigenPoint3(
  const Eigen::Matrix<Scalar, 3, 1> & vec,
  geometry_msgs::msg::Point & msg)
{
  msg.x = static_cast<double>(vec.x());
  msg.y = static_cast<double>(vec.y());
  msg.z = static_cast<double>(vec.z());
}

template<typename Scalar>
void eigenQuaternionFromRosVector3(
  const geometry_msgs::msg::Vector3 & msg,
  Eigen::Quaternion<Scalar> & q,
  const Scalar scaler = 1)
{
  q = Eigen::AngleAxis<Scalar>(
    scaler * static_cast<Scalar>(msg.z), Eigen::Vector3<Scalar>::UnitZ());
  q = q * Eigen::AngleAxis<Scalar>(
    scaler * static_cast<Scalar>(msg.y), Eigen::Vector3<Scalar>::UnitY());
  q = q * Eigen::AngleAxis<Scalar>(
    scaler * static_cast<Scalar>(msg.x), Eigen::Vector3<Scalar>::UnitX());
}

template<typename Scalar>
void eigenQuaternionFromRosQuaternion(
  const geometry_msgs::msg::Quaternion & msg,
  Eigen::Quaternion<Scalar> & q)
{
  q.x() = static_cast<Scalar>(msg.x);
  q.y() = static_cast<Scalar>(msg.y);
  q.z() = static_cast<Scalar>(msg.z);
  q.w() = static_cast<Scalar>(msg.w);
}

template<typename Scalar>
void rosQuaternionFromEigenQuaternion(
  const Eigen::Quaternion<Scalar> & q,
  geometry_msgs::msg::Quaternion & msg)
{
  msg.x = static_cast<double>(q.x());
  msg.y = static_cast<double>(q.y());
  msg.z = static_cast<double>(q.z());
  msg.w = static_cast<double>(q.w());
}
}  // namespace dummy_odometry
