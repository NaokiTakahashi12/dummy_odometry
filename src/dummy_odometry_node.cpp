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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <functional>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <dummy_odometry/converter/eigen.hpp>
#include <dummy_odometry/converter/state_space.hpp>
#include <dummy_odometry/simple_dynamics.hpp>
#include <dummy_odometry_node_parameters.hpp>

namespace dummy_odometry
{
class DummyOdometryNode : public rclcpp::Node
{
public:
  explicit DummyOdometryNode(const rclcpp::NodeOptions &);
  ~DummyOdometryNode();

private:
  static constexpr char m_this_node_name[] = "dummy_odometry_node";

  float m_delta_time;

  geometry_msgs::msg::TwistStamped::UniquePtr m_command_velocity;
  nav_msgs::msg::Odometry::UniquePtr m_odometry;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_stamped_cmd_vel_subscription;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_subscription;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odometry_publisher;
  rclcpp::TimerBase::SharedPtr m_update_odometry_timer;
  rclcpp::TimerBase::SharedPtr m_publish_odometry_timer;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  std::unique_ptr<dummy_odometry_node::ParamListener> m_param_listener;
  std::unique_ptr<dummy_odometry_node::Params> m_params;

  std::unique_ptr<SimpleDynamics<float>> m_simple_dynamics;

  bool assertCommandVelocity(const geometry_msgs::msg::TwistStamped &);
  bool assertCommandVelocity(const geometry_msgs::msg::Twist &);
  void commandVelocityStampedCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &);
  void commandVelocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr &);

  void updateOdometryCallback();

  void publishTransformCallback(const nav_msgs::msg::Odometry &);
  void publishOdometryCallback();

  std_msgs::msg::Header::UniquePtr generateOdometryHeaderMsg();
};

DummyOdometryNode::DummyOdometryNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node(
    m_this_node_name,
    rclcpp::NodeOptions(node_options).use_intra_process_comms(true)),
  m_command_velocity(nullptr),
  m_odometry(nullptr),
  m_stamped_cmd_vel_subscription(nullptr),
  m_cmd_vel_subscription(nullptr),
  m_odometry_publisher(nullptr),
  m_update_odometry_timer(nullptr),
  m_publish_odometry_timer(nullptr),
  m_tf_buffer(nullptr),
  m_tf_listener(nullptr),
  m_tf_broadcaster(nullptr),
  m_param_listener(nullptr),
  m_params(nullptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Start " << m_this_node_name);

  m_param_listener = std::make_unique<dummy_odometry_node::ParamListener>(
    this->get_node_parameters_interface()
  );
  m_params = std::make_unique<dummy_odometry_node::Params>(
    m_param_listener->get_params()
  );
  m_odometry = std::make_unique<nav_msgs::msg::Odometry>();
  m_odometry->header = *generateOdometryHeaderMsg();
  m_odometry->child_frame_id = m_params->child_frame_id;

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
  try {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg = m_tf_buffer->lookupTransform(
      m_odometry->child_frame_id,
      m_odometry->header.frame_id,
      tf2::TimePointZero);
    m_odometry->pose.pose.position.x = tf_msg.transform.translation.x;
    m_odometry->pose.pose.position.y = tf_msg.transform.translation.y;
    m_odometry->pose.pose.position.z = tf_msg.transform.translation.z;
    m_odometry->pose.pose.orientation.x = tf_msg.transform.rotation.x;
    m_odometry->pose.pose.orientation.y = tf_msg.transform.rotation.y;
    m_odometry->pose.pose.orientation.z = tf_msg.transform.rotation.z;
    m_odometry->pose.pose.orientation.w = tf_msg.transform.rotation.w;
    RCLCPP_INFO(this->get_logger(), "Overwrite initial odometry from tf");
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(this->get_logger(), e.what());
  }

  if (m_params->tf_broadcast) {
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Disable tf_broadcast");
  }
  if (m_params->require_command_timestamp) {
    m_stamped_cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel_stamped",
      rclcpp::QoS(5),
      std::bind(
        &DummyOdometryNode::commandVelocityStampedCallback,
        this,
        std::placeholders::_1
      )
    );
  } else {
    m_cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel",
      rclcpp::QoS(5),
      std::bind(
        &DummyOdometryNode::commandVelocityCallback,
        this,
        std::placeholders::_1
      )
    );
  }
  m_odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom",
    rclcpp::QoS(5)
  );
  m_delta_time = 1 / m_params->update_odometry_frequency;

  SimpleDynamicsParameters<float> simple_dynamics_params;
  loadSimpleDynamicsParam(*m_params, simple_dynamics_params);
  m_simple_dynamics = std::make_unique<SimpleDynamics<float>>(simple_dynamics_params);
  m_simple_dynamics->setDeltaTime(m_delta_time);

  const unsigned int update_odometry_milliseconds = 1e3 / m_params->update_odometry_frequency;
  m_update_odometry_timer = this->create_wall_timer(
    std::chrono::milliseconds(
      update_odometry_milliseconds
    ),
    std::bind(
      &DummyOdometryNode::updateOdometryCallback,
      this
    )
  );
  const unsigned int publish_odometry_millseconds = 1e3 / m_params->publish_odometry_frequency;
  m_publish_odometry_timer = this->create_wall_timer(
    std::chrono::milliseconds(
      publish_odometry_millseconds
    ),
    std::bind(
      &DummyOdometryNode::publishOdometryCallback,
      this
    )
  );
}

DummyOdometryNode::~DummyOdometryNode()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Finish " << m_this_node_name);
}

bool isNaNVector3(const geometry_msgs::msg::Vector3 & msg)
{
  if (std::isnan(msg.x)) {
    return true;
  } else if (std::isnan(msg.y)) {
    return true;
  } else if (std::isnan(msg.z)) {
    return true;
  }
  return false;
}

bool DummyOdometryNode::assertCommandVelocity(const geometry_msgs::msg::TwistStamped & msg)
{
  if (!m_params) {
    throw std::runtime_error("m_params is null");
  }
  if (msg.header.frame_id != m_params->child_frame_id) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Difference command frame_id: " << msg.header.frame_id);
    return true;
  }
  return assertCommandVelocity(msg.twist);
}

bool DummyOdometryNode::assertCommandVelocity(const geometry_msgs::msg::Twist & msg)
{
  if (isNaNVector3(msg.linear) || isNaNVector3(msg.angular)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Detect NaN in command velocity");
    return true;
  }
  return false;
}

void DummyOdometryNode::commandVelocityStampedCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg)
{
  if (assertCommandVelocity(*msg)) {
    RCLCPP_WARN(this->get_logger(), "Update command velocity failed");
    return;
  }
  if (!m_command_velocity) {
    RCLCPP_INFO_STREAM(this->get_logger(), "First recived command velocity with timestamp");
    m_command_velocity = std::make_unique<geometry_msgs::msg::TwistStamped>();
  }
  m_command_velocity->header = msg->header;
  m_command_velocity->twist = msg->twist;
}

void DummyOdometryNode::commandVelocityCallback(
  const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  if (!m_params) {
    throw std::runtime_error("m_params is null");
  }
  if (assertCommandVelocity(*msg)) {
    RCLCPP_WARN(this->get_logger(), "Update command velocity failed");
    return;
  }
  if (!m_command_velocity) {
    RCLCPP_INFO_STREAM(this->get_logger(), "First recived command velocity");
    m_command_velocity = std::make_unique<geometry_msgs::msg::TwistStamped>();
  }
  m_command_velocity->twist = *msg;
  m_command_velocity->header.stamp = this->get_clock()->now();
  m_command_velocity->header.frame_id = m_params->child_frame_id;
}

void DummyOdometryNode::updateOdometryCallback()
{
  if (!m_odometry) {
    throw std::runtime_error("m_odometry is null");
  }
  if (!m_params) {
    throw std::runtime_error("m_params is null");
  }
  if (!m_command_velocity) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Not recieved command velocity. waiting "
        << m_params->wait_command_seconds
        << " [s]"
    );
    std::this_thread::sleep_for(
      std::chrono::milliseconds(
        static_cast<unsigned int>(m_params->wait_command_seconds * 1e3)
      )
    );
    return;
  }

  if (m_command_velocity->header.frame_id != m_params->child_frame_id) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Different command frmae_id: " << m_command_velocity->header.frame_id
    );
    return;
  }
  Eigen::Matrix<float, 3, 1> odom_l{};
  Eigen::Matrix<float, 3, 1> vel_l{};
  Eigen::Quaternion<float> odom_q{};
  Eigen::Quaternion<float> vel_q{};

  eigenVectorFromRosPoint3(m_odometry->pose.pose.position, odom_l);
  eigenQuaternionFromRosQuaternion(m_odometry->pose.pose.orientation, odom_q);
  eigenVectorFromRosVector3(m_command_velocity->twist.linear, vel_l);
  eigenQuaternionFromRosVector3(
    m_command_velocity->twist.angular,
    vel_q,
    m_delta_time
  );
  //! @todo class
  if ("sum" == m_params->simulate_odometry_method) {
    odom_q = odom_q * vel_q;
    odom_l = m_delta_time * (odom_q * vel_l) + odom_l;
  } else if ("simple_dynamics" == m_params->simulate_odometry_method) {
    m_simple_dynamics->setDestAngularVelocity(vel_q);
    m_simple_dynamics->setDestVelocity(vel_l);
    m_simple_dynamics->updateOdometry();
    odom_q = m_simple_dynamics->getAngularPosition();
    odom_l = m_simple_dynamics->getPosition();
  } else {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Unknown simulate_odometry_method: "
        << m_params->simulate_odometry_method
    );
    return;
  }
  rosVector3FromEigenPoint3(odom_l, m_odometry->pose.pose.position);
  rosQuaternionFromEigenQuaternion(odom_q, m_odometry->pose.pose.orientation);
  m_odometry->header.stamp = this->get_clock()->now();
}

void DummyOdometryNode::publishTransformCallback(const nav_msgs::msg::Odometry & odom_msg)
{
  if (!m_params) {
    throw std::runtime_error("m_params is null");
  }
  if (odom_msg.child_frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "odom_msg child_frame_id is empty");
    return;
  }
  if (!m_tf_broadcaster) {
    return;
  }
  auto odom_to_base_transform_msg = std::make_unique<geometry_msgs::msg::TransformStamped>();
  odom_to_base_transform_msg->header = odom_msg.header;
  odom_to_base_transform_msg->child_frame_id = odom_msg.child_frame_id;
  odom_to_base_transform_msg->transform.translation.x = odom_msg.pose.pose.position.x;
  odom_to_base_transform_msg->transform.translation.y = odom_msg.pose.pose.position.y;
  odom_to_base_transform_msg->transform.translation.z = odom_msg.pose.pose.position.z;
  odom_to_base_transform_msg->transform.rotation = odom_msg.pose.pose.orientation;
  m_tf_broadcaster->sendTransform(*odom_to_base_transform_msg);
}

void DummyOdometryNode::publishOdometryCallback()
{
  if (!m_odometry) {
    throw std::runtime_error("m_odometry is null");
  }
  if (!m_odometry_publisher) {
    throw std::runtime_error("m_odometry_publisher is null");
  }
  publishTransformCallback(*m_odometry);
  {
    m_odometry_publisher->publish(*m_odometry);
  }
}

std_msgs::msg::Header::UniquePtr DummyOdometryNode::generateOdometryHeaderMsg()
{
  if (!m_params) {
    throw std::runtime_error("m_params is null");
  }
  auto odom_header_msg = std::make_unique<std_msgs::msg::Header>();
  odom_header_msg->frame_id = m_params->parent_frame_id;
  odom_header_msg->stamp = this->get_clock()->now();
  return odom_header_msg;
}
}  // namespace dummy_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(dummy_odometry::DummyOdometryNode)
