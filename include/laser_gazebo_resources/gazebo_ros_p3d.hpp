// Copyright 2012 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo_plugins
{

class GazeboRosP3DPrivate;

/**
 * @brief Publishes a model link's pose and twist as a ROS odometry message.
 *
 * The configured link is measured relative to either the Gazebo world or another link in the
 * same model.
 * Optional translation, rotation, and Gaussian-noise parameters can be provided in the plugin's
 * SDF configuration.
 *
 * Example usage:
 * @code{.xml}
 * <plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
 *   <ros>
 *     <!-- Add a namespace. -->
 *     <namespace>/demo</namespace>
 *
 *     <!-- Remap the default odometry topic. -->
 *     <remapping>odom:=p3d_demo</remapping>
 *   </ros>
 *
 *   <!-- Link whose pose and twist will be published. -->
 *   <body_name>box_link</body_name>
 *
 *   <!-- Optional reference link.
 *        Remove this tag to use the world frame. -->
 *   <frame_name>sphere_link</frame_name>
 *
 *   <!-- Publishing rate in hertz.
 *        Zero publishes as fast as possible. -->
 *   <update_rate>1.0</update_rate>
 *
 *   <!-- Constant translation and rotation offsets applied to the reported pose. -->
 *   <xyz_offset>10 10 10</xyz_offset>
 *   <rpy_offset>0.1 0.1 0.1</rpy_offset>
 *
 *   <!-- Report twist in the tracked link's local frame instead of the world frame. -->
 *   <local_twist>false</local_twist>
 *
 *   <!-- Standard deviation of Gaussian noise added to each reported velocity axis. -->
 *   <gaussian_noise>0.01</gaussian_noise>
 * </plugin>
 * @endcode
 */
class GazeboRosP3D : public gazebo::ModelPlugin
{
public:
  /// Create the plugin and its private implementation.
  GazeboRosP3D();

  /// Destroy the plugin after disconnecting its owned resources.
  ~GazeboRosP3D() override;

  /// Load and configure the plugin from the model's SDF description.
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Private implementation that keeps Gazebo and ROS details out of the public header.
  std::unique_ptr<GazeboRosP3DPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_P3D_HPP_
