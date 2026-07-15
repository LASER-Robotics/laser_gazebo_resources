/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 *
 * Modifications: classical quasi-steady blade-element-momentum rotor model
 * with rigid blades and uniform induced velocity across the rotor disk.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef LASER_GAZEBO_RESOURCES__GAZEBO_MOTOR_MODEL_HPP_
#define LASER_GAZEBO_RESOURCES__GAZEBO_MOTOR_MODEL_HPP_

#include <limits>
#include <memory>
#include <string>

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math.hh>

#include "CommandMotorSpeed.pb.h"
#include "Float.pb.h"
#include "MotorSpeed.pb.h"
#include "Wind.pb.h"
#include "common.h"
#include "rotors_model/motor_model.hpp"

namespace turning_direction {
inline constexpr int kCounterClockwise = 1;
inline constexpr int kClockwise = -1;
}  // namespace turning_direction

namespace gazebo {

using CommandMotorSpeedPtr =
  boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed>;
using WindPtr = boost::shared_ptr<const physics_msgs::msgs::Wind>;

class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:
  GazeboMotorModel() = default;
  ~GazeboMotorModel() override;

  void InitializeParams() override;
  void Publish() override;

 protected:
  void UpdateForcesAndMoments() override;
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  struct BemLoads {
    double thrust{0.0};
    double horizontal_force{0.0};
    double drag_torque{0.0};
  };

  void validate_required_bem_parameters(const sdf::ElementPtr & sdf) const;
  void load_required_bem_parameters(const sdf::ElementPtr & sdf);
  void initialize_bem_model();

  void update_bem_forces_and_moments();
  void update_legacy_forces_and_moments();
  void update_motor_failure();
  void on_update(const common::UpdateInfo & info);

  BemLoads compute_blade_element_loads(
    double omega,
    double horizontal_velocity,
    double vertical_velocity,
    double induced_velocity) const;

  double bem_residual(
    double omega,
    double horizontal_velocity,
    double vertical_velocity,
    double induced_velocity) const;

  double solve_induced_velocity(
    double omega,
    double horizontal_velocity,
    double vertical_velocity,
    double initial_guess,
    bool * converged) const;

  double solve_induced_velocity_bisection(
    double omega,
    double horizontal_velocity,
    double vertical_velocity,
    bool * converged) const;

  double command_to_target_omega(double command) const;
  double command_to_desired_static_thrust(double command) const;
  ignition::math::Vector3d relative_air_velocity_rotor_frame() const;
  void apply_reaction_torque(double torque_z_rotor_frame);

  void velocity_callback(const CommandMotorSpeedPtr & motor_speeds);
  void motor_failure_callback(
    const boost::shared_ptr<const msgs::Int> & failure_message);
  void wind_velocity_callback(const WindPtr & wind_message);

  std::string command_sub_topic_{"/gazebo/command/motor_speed"};
  std::string motor_failure_sub_topic_{"/gazebo/motor_failure_num"};
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_{"/motor_speed"};
  std::string wind_sub_topic_{"/world_wind"};
  std::string namespace_;

  int motor_number_{0};
  int turning_direction_{turning_direction::kClockwise};
  int motor_failure_number_{0};

  double max_force_{std::numeric_limits<double>::max()};
  double max_rot_velocity_{838.0};
  double moment_constant_{0.016};
  double motor_constant_{8.54858e-06};
  double ref_motor_rot_vel_{0.0};
  double actual_motor_omega_{0.0};
  double rolling_moment_coefficient_{1.0e-6};
  double rotor_drag_coefficient_{1.0e-4};
  double rotor_velocity_slowdown_sim_{10.0};
  double motor_quadratic_a_{0.1};
  double motor_quadratic_b_{-0.1};
  double time_constant_down_{1.0 / 40.0};
  double time_constant_up_{1.0 / 80.0};

  bool reversible_{false};
  bool use_pid_{false};
  bool motor_failed_{false};
  bool use_bem_{false};

  // BEM command settings. commandMode is mandatory whenever useBem is true.
  std::string command_mode_;
  double command_deadband_{1.0e-3};
  double command_max_{1.0};

  // Required BEM inputs deliberately have no usable physical defaults. They
  // are loaded only after their explicit presence in the SDF is verified.
  double bem_rho_{std::numeric_limits<double>::quiet_NaN()};
  double bem_prop_diameter_inches_{std::numeric_limits<double>::quiet_NaN()};
  double bem_prop_pitch_inches_{std::numeric_limits<double>::quiet_NaN()};
  double bem_prop_chord_m_{std::numeric_limits<double>::quiet_NaN()};
  int bem_prop_nb_{0};
  double bem_cl0_{std::numeric_limits<double>::quiet_NaN()};
  double bem_cd0_{std::numeric_limits<double>::quiet_NaN()};
  double bem_x_root_{std::numeric_limits<double>::quiet_NaN()};
  int bem_radial_stations_{0};
  int bem_azimuth_stations_{0};
  bool bem_correct_pitch_interpolation_{false};

  // Quantities derived from the required BEM parameters during Load().
  double bem_radius_{0.0};
  double bem_area_{0.0};
  double bem_sigma_{0.0};
  double bem_theta_root_{0.0};
  double bem_theta_twist_{0.0};
  double bem_static_thrust_coefficient_{0.0};
  double bem_static_induced_velocity_ratio_{0.0};
  double induced_velocity_{0.0};

  mutable unsigned int bem_solver_failure_count_{0};

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_pub_;
  transport::SubscriberPtr command_sub_;
  transport::SubscriberPtr motor_failure_sub_;
  transport::SubscriberPtr wind_sub_;

  ignition::math::Vector3d wind_velocity_{0.0, 0.0, 0.0};

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  common::PID pid_;
  event::ConnectionPtr update_connection_;

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  std_msgs::msgs::Float turning_velocity_message_;
};

}  // namespace gazebo

#endif  // LASER_GAZEBO_RESOURCES__GAZEBO_MOTOR_MODEL_HPP_
