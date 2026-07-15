// Copyright 2013 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gazebo_plugins/gazebo_ros_p3d.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <ignition/math/Rand.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo_plugins
{

class GazeboRosP3DPrivate
{
public:
  /// Configure the private implementation and connect it to Gazebo's update event.
  void Load(
    const gazebo::physics::ModelPtr & model,
    const sdf::ElementPtr & sdf);

private:
  /// Process one simulation update and publish odometry when the configured period has elapsed.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// Read the update-rate parameter, retaining the default when it is omitted.
  void ConfigureUpdateRate(const sdf::ElementPtr & sdf);

  /// Resolve the tracked link from the required body_name parameter.
  bool ConfigureTrackedLink(
    const gazebo::physics::ModelPtr & model,
    const sdf::ElementPtr & sdf);

  /// Create the odometry publisher using the QoS profile supplied by gazebo_ros.
  void ConfigurePublisher();

  /// Read translation and rotation offsets, including the deprecated plural parameter names.
  void ConfigureOffsets(const sdf::ElementPtr & sdf);

  /// Read the local-twist and Gaussian-noise options.
  void ConfigureTwistAndNoise(const sdf::ElementPtr & sdf);

  /// Resolve the optional link used as the pose and velocity reference frame.
  void ConfigureReferenceFrame(
    const gazebo::physics::ModelPtr & model,
    const sdf::ElementPtr & sdf);

  /// Return true when frame_name identifies one of the accepted inertial frames.
  bool UsesWorldReferenceFrame() const;

  /// Populate the pose and twist covariance diagonals from the configured noise variance.
  void FillCovariance(nav_msgs::msg::Odometry & message) const;

  /// Add one independent Gaussian-noise sample to a scalar velocity component.
  double AddGaussianNoise(double value) const;

  /// Link whose state is published.
  gazebo::physics::LinkPtr link_{nullptr};

  /// Optional link that defines the output reference frame.
  gazebo::physics::LinkPtr reference_link_{nullptr};

  /// ROS node owned by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Odometry publisher.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_{nullptr};

  /// Resolved odometry topic name.
  std::string topic_name_{"odom"};

  /// Output frame name, which defaults to the Gazebo world frame.
  std::string frame_name_{"world"};

  /// Constant translation and rotation offsets applied to the reported pose.
  ignition::math::Pose3d offset_{};

  /// Simulation timestamp of the most recently published message.
  gazebo::common::Time last_time_{};

  /// Maximum publishing rate in hertz; zero disables rate limiting.
  double update_rate_{0.0};

  /// Standard deviation of noise added independently to each velocity axis.
  double gaussian_noise_{0.0};

  /// Whether twist values should be obtained in the tracked link's local frame.
  bool local_twist_{false};

  /// Connection that invokes OnUpdate at the start of each simulation iteration.
  gazebo::event::ConnectionPtr update_connection_{nullptr};
};

GazeboRosP3D::GazeboRosP3D()
: impl_(std::make_unique<GazeboRosP3DPrivate>())
{
}

GazeboRosP3D::~GazeboRosP3D() = default;

void GazeboRosP3D::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  impl_->Load(model, sdf);
}

void GazeboRosP3DPrivate::Load(
  const gazebo::physics::ModelPtr & model,
  const sdf::ElementPtr & sdf)
{
  ros_node_ = gazebo_ros::Node::Get(sdf);

  ConfigureUpdateRate(sdf);
  if (!ConfigureTrackedLink(model, sdf)) {
    return;
  }

  ConfigurePublisher();
  ConfigureOffsets(sdf);
  ConfigureTwistAndNoise(sdf);
  ConfigureReferenceFrame(model, sdf);

  last_time_ = model->GetWorld()->SimTime();

  // Register only after configuration succeeds.
  // This prevents callbacks from observing a partially loaded plugin.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosP3DPrivate::OnUpdate, this, std::placeholders::_1));
}

void GazeboRosP3DPrivate::ConfigureUpdateRate(const sdf::ElementPtr & sdf)
{
  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "p3d plugin missing <update_rate>; defaulting to 0.0 (as fast as possible)");
    return;
  }

  update_rate_ = sdf->GetElement("update_rate")->Get<double>();
}

bool GazeboRosP3DPrivate::ConfigureTrackedLink(
  const gazebo::physics::ModelPtr & model,
  const sdf::ElementPtr & sdf)
{
  if (!sdf->HasElement("body_name")) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Missing <body_name>; cannot load the p3d plugin");
    return false;
  }

  const std::string link_name = sdf->GetElement("body_name")->Get<std::string>();
  link_ = model->GetLink(link_name);
  if (!link_) {
    RCLCPP_ERROR(
      ros_node_->get_logger(), "body_name [%s] does not exist", link_name.c_str());
    return false;
  }

  return true;
}

void GazeboRosP3DPrivate::ConfigurePublisher()
{
  const gazebo_ros::QoS & qos = ros_node_->get_qos();
  publisher_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    topic_name_,
    qos.get_publisher_qos(topic_name_, rclcpp::SensorDataQoS().reliable()));

  topic_name_ = publisher_->get_topic_name();
  RCLCPP_DEBUG(
    ros_node_->get_logger(), "Publishing odometry on topic [%s]", topic_name_.c_str());
}

void GazeboRosP3DPrivate::ConfigureOffsets(const sdf::ElementPtr & sdf)
{
  // Read deprecated names first so the preferred singular names take precedence when both exist.
  if (sdf->HasElement("xyz_offsets")) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "<xyz_offsets> is deprecated; use <xyz_offset> instead");
    offset_.Pos() = sdf->GetElement("xyz_offsets")->Get<ignition::math::Vector3d>();
  }

  if (sdf->HasElement("xyz_offset")) {
    offset_.Pos() = sdf->GetElement("xyz_offset")->Get<ignition::math::Vector3d>();
  } else if (!sdf->HasElement("xyz_offsets")) {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Missing <xyz_offset>; defaulting to zero");
  }

  if (sdf->HasElement("rpy_offsets")) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "<rpy_offsets> is deprecated; use <rpy_offset> instead");
    offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offsets")->Get<ignition::math::Vector3d>());
  }

  if (sdf->HasElement("rpy_offset")) {
    offset_.Rot() = ignition::math::Quaterniond(
      sdf->GetElement("rpy_offset")->Get<ignition::math::Vector3d>());
  } else if (!sdf->HasElement("rpy_offsets")) {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Missing <rpy_offset>; defaulting to zero");
  }
}

void GazeboRosP3DPrivate::ConfigureTwistAndNoise(const sdf::ElementPtr & sdf)
{
  if (!sdf->HasElement("local_twist")) {
    RCLCPP_WARN(
      ros_node_->get_logger(), "p3d plugin missing <local_twist>; defaulting to false");
  } else {
    local_twist_ = sdf->GetElement("local_twist")->Get<bool>();
  }

  if (!sdf->HasElement("gaussian_noise")) {
    RCLCPP_DEBUG(
      ros_node_->get_logger(), "Missing <gaussian_noise>; defaulting to 0.0");
  } else {
    gaussian_noise_ = sdf->GetElement("gaussian_noise")->Get<double>();
  }
}

void GazeboRosP3DPrivate::ConfigureReferenceFrame(
  const gazebo::physics::ModelPtr & model,
  const sdf::ElementPtr & sdf)
{
  if (!sdf->HasElement("frame_name")) {
    RCLCPP_DEBUG(ros_node_->get_logger(), "Missing <frame_name>; defaulting to world");
  } else {
    frame_name_ = sdf->GetElement("frame_name")->Get<std::string>();
  }

  if (UsesWorldReferenceFrame()) {
    return;
  }

  reference_link_ = model->GetLink(frame_name_);
  if (!reference_link_) {
    RCLCPP_WARN(
      ros_node_->get_logger(),
      "frame_name [%s] does not identify a link in this model; using world values",
      frame_name_.c_str());
  }
}

bool GazeboRosP3DPrivate::UsesWorldReferenceFrame() const
{
  return frame_name_ == "/world" || frame_name_ == "world" || frame_name_ == "/map" ||
         frame_name_ == "map";
}

void GazeboRosP3DPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  if (!link_) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosP3DPrivate::OnUpdate");
#endif

  const gazebo::common::Time current_time = info.simTime;
  if (current_time < last_time_) {
    // Gazebo can reset simulation time when a world is reset.
    // Restart rate tracking from the new simulation time.
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update-time difference detected");
    last_time_ = current_time;
  }

  const double elapsed_seconds = (current_time - last_time_).Double();
  if (update_rate_ > 0.0 && elapsed_seconds < (1.0 / update_rate_)) {
    return;
  }

  // Avoid message construction and random-number generation when no consumer is connected.
  if (ros_node_->count_subscribers(topic_name_) == 0U) {
    return;
  }

  if (elapsed_seconds == 0.0) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("fill ROS message");
#endif

  nav_msgs::msg::Odometry odometry_message;
  odometry_message.header.frame_id = frame_name_;
  odometry_message.header.stamp =
    gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  odometry_message.child_frame_id = link_->GetName();

  ignition::math::Vector3d linear_velocity;
  ignition::math::Vector3d angular_velocity;
  if (local_twist_) {
    linear_velocity = link_->RelativeLinearVel();
    angular_velocity = link_->RelativeAngularVel();
  } else {
    linear_velocity = link_->WorldLinearVel();
    angular_velocity = link_->WorldAngularVel();
  }

  auto link_pose = link_->WorldPose();
  if (reference_link_) {
    const auto reference_pose = reference_link_->WorldPose();
    const auto reference_linear_velocity = reference_link_->WorldLinearVel();
    const auto reference_angular_velocity = reference_link_->WorldAngularVel();

    // Express the world-space displacement and orientation in the reference link's axes.
    link_pose.Pos() = link_pose.Pos() - reference_pose.Pos();
    link_pose.Pos() = reference_pose.Rot().RotateVectorReverse(link_pose.Pos());
    link_pose.Rot() *= reference_pose.Rot().Inverse();

    // Preserve the plugin's established velocity transformation semantics.
    linear_velocity = reference_pose.Rot().RotateVector(
      linear_velocity - reference_linear_velocity);
    if (local_twist_) {
      angular_velocity = reference_pose.Rot().RotateVector(
        angular_velocity - reference_angular_velocity);
    }
  }

  // Apply user-provided offsets after transforming the pose into the requested reference frame.
  link_pose.Pos() = link_pose.Pos() + offset_.Pos();
  link_pose.Rot() = offset_.Rot() * link_pose.Rot();
  link_pose.Rot().Normalize();

  odometry_message.pose.pose.position =
    gazebo_ros::Convert<geometry_msgs::msg::Point>(link_pose.Pos());
  odometry_message.pose.pose.orientation =
    gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(link_pose.Rot());

  odometry_message.twist.twist.linear.x = AddGaussianNoise(linear_velocity.X());
  odometry_message.twist.twist.linear.y = AddGaussianNoise(linear_velocity.Y());
  odometry_message.twist.twist.linear.z = AddGaussianNoise(linear_velocity.Z());
  odometry_message.twist.twist.angular.x = AddGaussianNoise(angular_velocity.X());
  odometry_message.twist.twist.angular.y = AddGaussianNoise(angular_velocity.Y());
  odometry_message.twist.twist.angular.z = AddGaussianNoise(angular_velocity.Z());

  FillCovariance(odometry_message);

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish");
#endif

  publisher_->publish(odometry_message);

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

  last_time_ = current_time;
}

void GazeboRosP3DPrivate::FillCovariance(nav_msgs::msg::Odometry & message) const
{
  constexpr std::array<std::size_t, 6> diagonal_indices{0U, 7U, 14U, 21U, 28U, 35U};
  const double variance = gaussian_noise_ * gaussian_noise_;

  // The legacy plugin uses the same configured variance for every pose and twist axis.
  for (const std::size_t index : diagonal_indices) {
    message.pose.covariance[index] = variance;
    message.twist.covariance[index] = variance;
  }
}

double GazeboRosP3DPrivate::AddGaussianNoise(double value) const
{
  return value + ignition::math::Rand::DblNormal(0.0, gaussian_noise_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosP3D)

}  // namespace gazebo_plugins
