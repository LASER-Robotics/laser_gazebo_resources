/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 *
 * Modifications: classical quasi-steady blade-element-momentum rotor model
 * with rigid blades and uniform induced velocity across the rotor disk.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "laser_gazebo_resources/gazebo_motor_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>

namespace gazebo {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-8;
constexpr double kMinimumOmegaForBem = 1.0e-3;
constexpr int kNewtonIterations = 8;
constexpr int kBisectionIterations = 45;

// These parameters define the BEM physics or numerical quadrature. They must
// be explicitly provided when useBem is true; no physical defaults are used.
constexpr std::array<const char *, 12> kRequiredBemParameters{
  "commandMode",
  "bemRho",
  "propDiameterInches",
  "propPitchInches",
  "propChordM",
  "propNb",
  "bemA",
  "bemCd0",
  "bemXRoot",
  "bemRadialStations",
  "bemAzimuthStations",
  "bemCorrectPitchInterpolation",
};

bool is_finite_positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

GazeboMotorModel::~GazeboMotorModel() {
  update_connection_.reset();
  command_sub_.reset();
  motor_failure_sub_.reset();
  wind_sub_.reset();
  motor_velocity_pub_.reset();
  rotor_velocity_filter_.reset();
  use_pid_ = false;
}

void GazeboMotorModel::InitializeParams() {
}

void GazeboMotorModel::Publish() {
  const double signed_omega =
    static_cast<double>(turning_direction_) * actual_motor_omega_;
  turning_velocity_message_.set_data(signed_omega);

  // The publisher is intentionally disabled to avoid queue-overflow warnings.
  // motor_velocity_pub_->Publish(turning_velocity_message_);
}

void GazeboMotorModel::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  if (!model || !sdf) {
    gzthrow("[gazebo_motor_model] Invalid model or SDF pointer.");
  }

  model_ = model;
  namespace_.clear();

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_motor_model] robotNamespace was not provided; "
          << "using an empty namespace.\n";
  }

  node_handle_.reset(new transport::Node());
  node_handle_->Init(namespace_);

  if (!sdf->HasElement("jointName")) {
    gzthrow("[gazebo_motor_model] Missing required parameter <jointName>.");
  }
  joint_name_ = sdf->GetElement("jointName")->Get<std::string>();
  joint_ = model_->GetJoint(joint_name_);
  if (!joint_) {
    gzthrow("[gazebo_motor_model] Could not find joint '" << joint_name_ << "'.");
  }

  if (sdf->HasElement("joint_control_pid")) {
    const sdf::ElementPtr pid = sdf->GetElement("joint_control_pid");
    const double p = pid->HasElement("p") ? pid->Get<double>("p") : 0.1;
    const double i = pid->HasElement("i") ? pid->Get<double>("i") : 0.0;
    const double d = pid->HasElement("d") ? pid->Get<double>("d") : 0.0;
    const double i_max =
      pid->HasElement("iMax") ? pid->Get<double>("iMax") : 0.0;
    const double i_min =
      pid->HasElement("iMin") ? pid->Get<double>("iMin") : 0.0;
    const double cmd_max =
      pid->HasElement("cmdMax") ? pid->Get<double>("cmdMax") : 3.0;
    const double cmd_min =
      pid->HasElement("cmdMin") ? pid->Get<double>("cmdMin") : -3.0;
    pid_.Init(p, i, d, i_max, i_min, cmd_max, cmd_min);
    use_pid_ = true;
  }

  if (!sdf->HasElement("linkName")) {
    gzthrow("[gazebo_motor_model] Missing required parameter <linkName>.");
  }
  link_name_ = sdf->GetElement("linkName")->Get<std::string>();
  link_ = model_->GetLink(link_name_);
  if (!link_) {
    gzthrow("[gazebo_motor_model] Could not find link '" << link_name_ << "'.");
  }

  if (!sdf->HasElement("motorNumber")) {
    gzthrow("[gazebo_motor_model] Missing required parameter <motorNumber>.");
  }
  motor_number_ = sdf->GetElement("motorNumber")->Get<int>();

  if (!sdf->HasElement("turningDirection")) {
    gzthrow("[gazebo_motor_model] Missing required parameter <turningDirection>.");
  }
  const std::string direction =
    sdf->GetElement("turningDirection")->Get<std::string>();
  if (direction == "cw") {
    turning_direction_ = turning_direction::kClockwise;
  } else if (direction == "ccw") {
    turning_direction_ = turning_direction::kCounterClockwise;
  } else {
    gzthrow("[gazebo_motor_model] turningDirection must be 'cw' or 'ccw'.");
  }

  if (sdf->HasElement("reversible")) {
    reversible_ = sdf->GetElement("reversible")->Get<bool>();
  }

  getSdfParam<std::string>(
    sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(
    sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);
  getSdfParam<std::string>(
    sdf, "windSubTopic", wind_sub_topic_, wind_sub_topic_);

  getSdfParam<double>(
    sdf, "rotorDragCoefficient", rotor_drag_coefficient_,
    rotor_drag_coefficient_);
  getSdfParam<double>(
    sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
    rolling_moment_coefficient_);
  getSdfParam<double>(
    sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(sdf, "momentConstant", moment_constant_, moment_constant_);
  getSdfParam<double>(
    sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(
    sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(
    sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_,
    rotor_velocity_slowdown_sim_);
  getSdfParam<double>(
    sdf, "motorQuadraticA", motor_quadratic_a_, motor_quadratic_a_);
  getSdfParam<double>(
    sdf, "motorQuadraticB", motor_quadratic_b_, motor_quadratic_b_);

  getSdfParam<bool>(sdf, "useBem", use_bem_, use_bem_);
  getSdfParam<double>(
    sdf, "commandDeadband", command_deadband_, command_deadband_);
  getSdfParam<double>(sdf, "commandMax", command_max_, command_max_);

  if (!is_finite_positive(rotor_velocity_slowdown_sim_)) {
    gzthrow("[gazebo_motor_model] rotorVelocitySlowdownSim must be positive.");
  }
  if (!is_finite_positive(time_constant_up_) ||
    !is_finite_positive(time_constant_down_)) {
    gzthrow("[gazebo_motor_model] Motor time constants must be positive.");
  }
  if (!is_finite_positive(max_rot_velocity_)) {
    gzthrow("[gazebo_motor_model] maxRotVelocity must be positive.");
  }

  // Validate and load the complete BEM configuration before registering any
  // callbacks. Throwing here prevents this plugin instance from starting.
  if (use_bem_) {
    validate_required_bem_parameters(sdf);
    load_required_bem_parameters(sdf);
    initialize_bem_model();
  }

#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif

  rotor_velocity_filter_ = std::make_unique<FirstOrderFilter<double>>(
    time_constant_up_, time_constant_down_, 0.0);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(
      &GazeboMotorModel::on_update, this, boost::placeholders::_1));

  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
    "~/" + model_->GetName() + command_sub_topic_,
    &GazeboMotorModel::velocity_callback, this);

  motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(
    motor_failure_sub_topic_, &GazeboMotorModel::motor_failure_callback, this);

  wind_sub_ = node_handle_->Subscribe(
    "~/" + wind_sub_topic_, &GazeboMotorModel::wind_velocity_callback, this);

  gzmsg << "[gazebo_motor_model] motor=" << motor_number_
        << " model=" << (use_bem_ ? "BEM" : "quadratic")
        << " commandMode=" << (use_bem_ ? command_mode_ : "legacy")
        << " maxOmega=" << max_rot_velocity_ << " rad/s\n";

  if (use_bem_) {
    gzmsg << "[gazebo_motor_model] BEM motor=" << motor_number_
          << " R=" << bem_radius_
          << " A=" << bem_area_
          << " sigma=" << bem_sigma_
          << " kT_static=" << bem_static_thrust_coefficient_
          << " vi/omega=" << bem_static_induced_velocity_ratio_
          << " quadrature=" << bem_radial_stations_
          << "x" << bem_azimuth_stations_ << "\n";
  }
}

void GazeboMotorModel::validate_required_bem_parameters(
  const sdf::ElementPtr & sdf) const {
  std::vector<std::string> missing_parameters;

  for (const char * parameter_name : kRequiredBemParameters) {
    if (!sdf->HasElement(parameter_name)) {
      missing_parameters.emplace_back(parameter_name);
    }
  }

  if (!missing_parameters.empty()) {
    std::ostringstream message;
    message << "[gazebo_motor_model] useBem=true, but required BEM SDF "
            << "parameters are missing:";
    for (const std::string & parameter_name : missing_parameters) {
      message << " <" << parameter_name << ">";
    }
    message << ". The BEM plugin instance will not start.";
    gzthrow(message.str());
  }

  const std::string command_mode =
    sdf->GetElement("commandMode")->Get<std::string>();
  if (command_mode == "throttle") {
    std::vector<std::string> missing_throttle_parameters;
    if (!sdf->HasElement("motorQuadraticA")) {
      missing_throttle_parameters.emplace_back("motorQuadraticA");
    }
    if (!sdf->HasElement("motorQuadraticB")) {
      missing_throttle_parameters.emplace_back("motorQuadraticB");
    }

    if (!missing_throttle_parameters.empty()) {
      std::ostringstream message;
      message << "[gazebo_motor_model] commandMode=throttle requires:";
      for (const std::string & parameter_name : missing_throttle_parameters) {
        message << " <" << parameter_name << ">";
      }
      message << ". The BEM plugin instance will not start.";
      gzthrow(message.str());
    }
  }
}

void GazeboMotorModel::load_required_bem_parameters(const sdf::ElementPtr & sdf) {
  command_mode_ = sdf->GetElement("commandMode")->Get<std::string>();
  bem_rho_ = sdf->GetElement("bemRho")->Get<double>();
  bem_prop_diameter_inches_ =
    sdf->GetElement("propDiameterInches")->Get<double>();
  bem_prop_pitch_inches_ =
    sdf->GetElement("propPitchInches")->Get<double>();
  bem_prop_chord_m_ = sdf->GetElement("propChordM")->Get<double>();
  bem_prop_nb_ = sdf->GetElement("propNb")->Get<int>();
  bem_cl0_ = sdf->GetElement("bemA")->Get<double>();
  bem_cd0_ = sdf->GetElement("bemCd0")->Get<double>();
  bem_x_root_ = sdf->GetElement("bemXRoot")->Get<double>();
  bem_radial_stations_ = sdf->GetElement("bemRadialStations")->Get<int>();
  bem_azimuth_stations_ =
    sdf->GetElement("bemAzimuthStations")->Get<int>();
  bem_correct_pitch_interpolation_ =
    sdf->GetElement("bemCorrectPitchInterpolation")->Get<bool>();
}

void GazeboMotorModel::initialize_bem_model() {
  if (command_mode_ != "throttle" && command_mode_ != "omega" &&
    command_mode_ != "thrust") {
    gzthrow(
      "[gazebo_motor_model] commandMode must be throttle, omega, or thrust.");
  }

  if (!is_finite_positive(bem_rho_) ||
    !is_finite_positive(bem_prop_diameter_inches_) ||
    !is_finite_positive(bem_prop_pitch_inches_) ||
    !is_finite_positive(bem_prop_chord_m_) || bem_prop_nb_ <= 0 ||
    !is_finite_positive(bem_cl0_) || !std::isfinite(bem_cd0_) ||
    bem_cd0_ < 0.0) {
    gzthrow("[gazebo_motor_model] One or more BEM parameters are invalid.");
  }

  if (!std::isfinite(bem_x_root_) || bem_x_root_ < 0.0 ||
    bem_x_root_ >= 1.0) {
    gzthrow("[gazebo_motor_model] bemXRoot must be in [0, 1).");
  }
  if (bem_radial_stations_ < 1 || bem_azimuth_stations_ < 4) {
    gzthrow("[gazebo_motor_model] Invalid BEM quadrature size.");
  }
  if (command_mode_ == "throttle" &&
    (!std::isfinite(motor_quadratic_a_) ||
    std::abs(motor_quadratic_a_) < kEpsilon ||
    !std::isfinite(motor_quadratic_b_))) {
    gzthrow(
      "[gazebo_motor_model] Invalid motor quadratic parameters for throttle "
      "mode.");
  }

  bem_radius_ = 0.5 * bem_prop_diameter_inches_ * 0.0254;
  bem_area_ = kPi * bem_radius_ * bem_radius_;
  bem_sigma_ = static_cast<double>(bem_prop_nb_) * bem_prop_chord_m_ /
    (kPi * bem_radius_);

  const double pitch_m = bem_prop_pitch_inches_ * 0.0254;
  const double theta_tip = std::atan2(pitch_m, 2.0 * kPi * bem_radius_);
  bem_theta_root_ = std::atan2(
    pitch_m, 2.0 * kPi * bem_x_root_ * bem_radius_);
  bem_theta_twist_ = theta_tip - bem_theta_root_;

  const double reference_omega =
    std::clamp(
      0.60 * max_rot_velocity_, kMinimumOmegaForBem, max_rot_velocity_);
  bool converged = false;
  const double reference_induced_velocity = solve_induced_velocity_bisection(
    reference_omega, 0.0, 0.0, &converged);
  if (!converged || !std::isfinite(reference_induced_velocity)) {
    gzthrow("[gazebo_motor_model] Could not initialize static BEM inflow.");
  }

  const BemLoads reference_loads = compute_blade_element_loads(
    reference_omega, 0.0, 0.0, reference_induced_velocity);
  bem_static_thrust_coefficient_ =
    reference_loads.thrust / (reference_omega * reference_omega);
  bem_static_induced_velocity_ratio_ =
    reference_induced_velocity / reference_omega;
  induced_velocity_ = 0.0;

  if (!is_finite_positive(bem_static_thrust_coefficient_)) {
    gzthrow("[gazebo_motor_model] Invalid static BEM thrust coefficient.");
  }
}

void GazeboMotorModel::on_update(const common::UpdateInfo & info) {
  sampling_time_ = info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = info.simTime.Double();

  if (!std::isfinite(sampling_time_) || sampling_time_ <= 0.0) {
    return;
  }

  update_motor_failure();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::velocity_callback(
  const CommandMotorSpeedPtr & motor_speeds) {
  if (!motor_speeds || motor_speeds->motor_speed_size() <= motor_number_) {
    const int message_size = motor_speeds ? motor_speeds->motor_speed_size() : 0;
    gzerr << "[gazebo_motor_model] Invalid motor index " << motor_number_
          << " for message size " << message_size << ".\n";
    return;
  }

  const double command = motor_speeds->motor_speed(motor_number_);

  if (use_bem_ && command_mode_ == "throttle") {
    ref_motor_rot_vel_ = std::clamp(command, 0.0, command_max_);
  } else if (use_bem_ && command_mode_ == "thrust") {
    ref_motor_rot_vel_ = std::max(command, 0.0);
  } else {
    const double lower_bound = reversible_ ? -max_rot_velocity_ : 0.0;
    ref_motor_rot_vel_ = std::clamp(command, lower_bound, max_rot_velocity_);
  }
}

void GazeboMotorModel::motor_failure_callback(
  const boost::shared_ptr<const msgs::Int> & failure_message) {
  if (failure_message) {
    motor_failure_number_ = failure_message->data();
  }
}

void GazeboMotorModel::wind_velocity_callback(const WindPtr & wind_message) {
  if (!wind_message) {
    return;
  }

  wind_velocity_ = ignition::math::Vector3d(
    wind_message->velocity().x(),
    wind_message->velocity().y(),
    wind_message->velocity().z());
}

void GazeboMotorModel::UpdateForcesAndMoments() {
  if (use_bem_) {
    update_bem_forces_and_moments();
  } else {
    update_legacy_forces_and_moments();
  }
}

double GazeboMotorModel::command_to_desired_static_thrust(double command) const {
  if (command <= command_deadband_) {
    return 0.0;
  }

  const double root = (command - motor_quadratic_b_) / motor_quadratic_a_;
  if (!std::isfinite(root) || root <= 0.0) {
    return 0.0;
  }
  return root * root;
}

double GazeboMotorModel::command_to_target_omega(double command) const {
  double target_omega = 0.0;

  if (command_mode_ == "omega") {
    target_omega = std::abs(command);
  } else if (command_mode_ == "thrust") {
    const double desired_thrust = std::max(command, 0.0);
    target_omega = std::sqrt(
      desired_thrust /
      std::max(bem_static_thrust_coefficient_, kEpsilon));
  } else {
    const double desired_thrust = command_to_desired_static_thrust(command);
    target_omega = std::sqrt(
      desired_thrust /
      std::max(bem_static_thrust_coefficient_, kEpsilon));
  }

  return std::clamp(target_omega, 0.0, max_rot_velocity_);
}

ignition::math::Vector3d
GazeboMotorModel::relative_air_velocity_rotor_frame() const {
#if GAZEBO_MAJOR_VERSION >= 9
  const ignition::math::Vector3d velocity_world = link_->WorldLinearVel();
  const ignition::math::Pose3d pose_world = link_->WorldCoGPose();
#else
  const ignition::math::Vector3d velocity_world =
    ignitionFromGazeboMath(link_->GetWorldLinearVel());
  const ignition::math::Pose3d pose_world =
    ignitionFromGazeboMath(link_->GetWorldCoGPose());
#endif

  const ignition::math::Vector3d relative_velocity_world =
    velocity_world - wind_velocity_;
  return pose_world.Rot().Inverse().RotateVector(relative_velocity_world);
}

GazeboMotorModel::BemLoads GazeboMotorModel::compute_blade_element_loads(
  double omega,
  double horizontal_velocity,
  double vertical_velocity,
  double induced_velocity) const {
  BemLoads loads;
  if (omega <= kMinimumOmegaForBem) {
    return loads;
  }

  const double radial_step =
    bem_radius_ * (1.0 - bem_x_root_) /
    static_cast<double>(bem_radial_stations_);
  const double azimuth_step =
    2.0 * kPi / static_cast<double>(bem_azimuth_stations_);
  const double prefactor = bem_rho_ * bem_sigma_ * bem_radius_ / 4.0;
  const double axial_velocity = vertical_velocity - induced_velocity;

  double thrust_sum = 0.0;
  double horizontal_force_sum = 0.0;
  double torque_sum = 0.0;

  for (int radial_index = 0;
    radial_index < bem_radial_stations_; ++radial_index) {
    const double normalized_radius =
      bem_x_root_ +
      (static_cast<double>(radial_index) + 0.5) *
      (1.0 - bem_x_root_) /
      static_cast<double>(bem_radial_stations_);
    const double local_radius = normalized_radius * bem_radius_;

    double local_pitch = 0.0;
    if (bem_correct_pitch_interpolation_) {
      local_pitch = bem_theta_root_ +
        ((normalized_radius - bem_x_root_) / (1.0 - bem_x_root_)) *
        bem_theta_twist_;
    } else {
      local_pitch = bem_theta_root_ +
        normalized_radius * bem_theta_twist_;
    }

    for (int azimuth_index = 0;
      azimuth_index < bem_azimuth_stations_; ++azimuth_index) {
      const double azimuth =
        (static_cast<double>(azimuth_index) + 0.5) * azimuth_step;
      const double azimuth_sine = std::sin(azimuth);

      const double tangential_velocity =
        omega * local_radius + horizontal_velocity * azimuth_sine;
      const double inflow_angle =
        std::atan2(axial_velocity, tangential_velocity);
      const double angle_of_attack = local_pitch + inflow_angle;
      const double speed_squared =
        tangential_velocity * tangential_velocity +
        axial_velocity * axial_velocity;

      const double angle_sine = std::sin(angle_of_attack);
      const double angle_cosine = std::cos(angle_of_attack);
      const double lift_coefficient =
        bem_cl0_ * angle_sine * angle_cosine;
      const double drag_coefficient = bem_cd0_ * angle_sine * angle_sine;

      const double lift = lift_coefficient * speed_squared;
      const double drag = drag_coefficient * speed_squared;
      const double normal_force =
        lift * std::cos(inflow_angle) + drag * std::sin(inflow_angle);
      const double in_plane_force =
        -lift * std::sin(inflow_angle) + drag * std::cos(inflow_angle);

      thrust_sum += normal_force;
      horizontal_force_sum += in_plane_force * azimuth_sine;
      torque_sum += in_plane_force * local_radius;
    }
  }

  const double integration_scale = prefactor * radial_step * azimuth_step;
  loads.thrust = integration_scale * thrust_sum;
  loads.horizontal_force = integration_scale * horizontal_force_sum;
  loads.drag_torque = integration_scale * torque_sum;
  return loads;
}

double GazeboMotorModel::bem_residual(
  double omega,
  double horizontal_velocity,
  double vertical_velocity,
  double induced_velocity) const {
  const BemLoads loads = compute_blade_element_loads(
    omega, horizontal_velocity, vertical_velocity, induced_velocity);
  const double momentum_speed = std::sqrt(
    horizontal_velocity * horizontal_velocity +
    (vertical_velocity - induced_velocity) *
    (vertical_velocity - induced_velocity) +
    kEpsilon * kEpsilon);
  const double momentum_thrust =
    2.0 * bem_rho_ * bem_area_ * induced_velocity * momentum_speed;
  return loads.thrust - momentum_thrust;
}

double GazeboMotorModel::solve_induced_velocity(
  double omega,
  double horizontal_velocity,
  double vertical_velocity,
  double initial_guess,
  bool * converged) const {
  if (converged) {
    *converged = false;
  }
  if (omega <= kMinimumOmegaForBem) {
    if (converged) {
      *converged = true;
    }
    return 0.0;
  }

  const double upper_bound = std::max(
    2.0 * omega * bem_radius_ + std::abs(vertical_velocity) +
    horizontal_velocity + 5.0,
    10.0);

  double induced_velocity = initial_guess;
  if (!std::isfinite(induced_velocity) || induced_velocity <= 0.0) {
    induced_velocity = bem_static_induced_velocity_ratio_ * omega;
  }
  induced_velocity = std::clamp(induced_velocity, 0.0, upper_bound);

  for (int iteration = 0; iteration < kNewtonIterations; ++iteration) {
    const double residual = bem_residual(
      omega, horizontal_velocity, vertical_velocity, induced_velocity);
    const double scale = 1.0 + std::abs(
      compute_blade_element_loads(
        omega, horizontal_velocity, vertical_velocity, induced_velocity)
      .thrust);

    if (std::abs(residual) <= 1.0e-7 * scale) {
      if (converged) {
        *converged = true;
      }
      return induced_velocity;
    }

    const double step_size =
      std::max(1.0e-4, 1.0e-3 * std::max(1.0, induced_velocity));
    const double lower_sample =
      std::clamp(induced_velocity - step_size, 0.0, upper_bound);
    const double upper_sample =
      std::clamp(induced_velocity + step_size, 0.0, upper_bound);
    const double sample_distance = upper_sample - lower_sample;
    if (sample_distance <= kEpsilon) {
      break;
    }

    const double derivative =
      (bem_residual(
        omega, horizontal_velocity, vertical_velocity, upper_sample) -
      bem_residual(
        omega, horizontal_velocity, vertical_velocity, lower_sample)) /
      sample_distance;
    if (!std::isfinite(derivative) || std::abs(derivative) < 1.0e-9) {
      break;
    }

    double newton_step = -residual / derivative;
    const double maximum_step = 0.5 * std::max(1.0, induced_velocity);
    newton_step = std::clamp(newton_step, -maximum_step, maximum_step);

    double candidate = std::clamp(
      induced_velocity + newton_step, 0.0, upper_bound);
    const double candidate_residual = bem_residual(
      omega, horizontal_velocity, vertical_velocity, candidate);
    if (std::abs(candidate_residual) > std::abs(residual)) {
      candidate = std::clamp(
        induced_velocity + 0.5 * newton_step, 0.0, upper_bound);
    }
    induced_velocity = candidate;
  }

  return solve_induced_velocity_bisection(
    omega, horizontal_velocity, vertical_velocity, converged);
}

double GazeboMotorModel::solve_induced_velocity_bisection(
  double omega,
  double horizontal_velocity,
  double vertical_velocity,
  bool * converged) const {
  if (converged) {
    *converged = false;
  }
  if (omega <= kMinimumOmegaForBem) {
    if (converged) {
      *converged = true;
    }
    return 0.0;
  }

  double lower_bound = 0.0;
  double upper_bound = std::max(
    2.0 * omega * bem_radius_ + std::abs(vertical_velocity) +
    horizontal_velocity + 5.0,
    10.0);
  double lower_residual = bem_residual(
    omega, horizontal_velocity, vertical_velocity, lower_bound);
  double upper_residual = bem_residual(
    omega, horizontal_velocity, vertical_velocity, upper_bound);

  for (int expansion = 0;
    expansion < 8 && lower_residual * upper_residual > 0.0;
    ++expansion) {
    upper_bound *= 2.0;
    upper_residual = bem_residual(
      omega, horizontal_velocity, vertical_velocity, upper_bound);
  }

  if (!std::isfinite(lower_residual) || !std::isfinite(upper_residual) ||
    lower_residual * upper_residual > 0.0) {
    return std::clamp(
      bem_static_induced_velocity_ratio_ * omega, 0.0, upper_bound);
  }

  for (int iteration = 0; iteration < kBisectionIterations; ++iteration) {
    const double middle = 0.5 * (lower_bound + upper_bound);
    const double middle_residual = bem_residual(
      omega, horizontal_velocity, vertical_velocity, middle);
    if (lower_residual * middle_residual <= 0.0) {
      upper_bound = middle;
      upper_residual = middle_residual;
    } else {
      lower_bound = middle;
      lower_residual = middle_residual;
    }
  }

  if (converged) {
    *converged = true;
  }
  return 0.5 * (lower_bound + upper_bound);
}

void GazeboMotorModel::update_bem_forces_and_moments() {
  const double target_omega =
    motor_failed_ ? 0.0 : command_to_target_omega(ref_motor_rot_vel_);

  if (motor_failed_) {
    actual_motor_omega_ = 0.0;
  } else {
    actual_motor_omega_ = rotor_velocity_filter_->updateFilter(
      target_omega, sampling_time_);
  }
  actual_motor_omega_ =
    std::clamp(actual_motor_omega_, 0.0, max_rot_velocity_);

  const double signed_joint_velocity =
    static_cast<double>(turning_direction_) * actual_motor_omega_ /
    rotor_velocity_slowdown_sim_;
  joint_->SetVelocity(0, signed_joint_velocity);
  motor_rot_vel_ =
    static_cast<double>(turning_direction_) * actual_motor_omega_;

  if (actual_motor_omega_ <= kMinimumOmegaForBem) {
    induced_velocity_ = 0.0;
    return;
  }

  // The rotor-link velocity already includes the local omega x r contribution.
  const ignition::math::Vector3d velocity_rotor =
    relative_air_velocity_rotor_frame();
  const double velocity_x = velocity_rotor.X();
  const double velocity_y = velocity_rotor.Y();
  const double horizontal_velocity = std::hypot(velocity_x, velocity_y);
  const double vertical_velocity = -velocity_rotor.Z();

  bool converged = false;
  const double solved_induced_velocity = solve_induced_velocity(
    actual_motor_omega_, horizontal_velocity, vertical_velocity,
    induced_velocity_, &converged);

  if (converged && std::isfinite(solved_induced_velocity) &&
    solved_induced_velocity >= 0.0) {
    induced_velocity_ = solved_induced_velocity;
  } else {
    induced_velocity_ = std::max(
      0.0, bem_static_induced_velocity_ratio_ * actual_motor_omega_);
    ++bem_solver_failure_count_;
    if (bem_solver_failure_count_ % 1000U == 1U) {
      gzerr << "[gazebo_motor_model] BEM inflow solver fallback, motor "
            << motor_number_ << ", failures=" << bem_solver_failure_count_
            << ".\n";
    }
  }

  BemLoads loads = compute_blade_element_loads(
    actual_motor_omega_, horizontal_velocity, vertical_velocity,
    induced_velocity_);

  if (!std::isfinite(loads.thrust) ||
    !std::isfinite(loads.horizontal_force) ||
    !std::isfinite(loads.drag_torque)) {
    gzerr << "[gazebo_motor_model] Non-finite BEM load on motor "
          << motor_number_ << ". Force skipped.\n";
    return;
  }

  if (!reversible_) {
    loads.thrust = std::max(0.0, loads.thrust);
  }

  // Apply axial thrust and in-plane BEM force in the rotor-local frame.
  ignition::math::Vector3d force_rotor(0.0, 0.0, loads.thrust);
  if (horizontal_velocity > 1.0e-6) {
    force_rotor.X(
      -loads.horizontal_force * velocity_x / horizontal_velocity);
    force_rotor.Y(
      -loads.horizontal_force * velocity_y / horizontal_velocity);
  }
  link_->AddRelativeForce(force_rotor);

  // Rotor reaction torque is opposite to the configured rotation direction.
  const double reaction_torque_z =
    -static_cast<double>(turning_direction_) * loads.drag_torque;
  apply_reaction_torque(reaction_torque_z);
}

void GazeboMotorModel::apply_reaction_torque(double torque_z_rotor_frame) {
  const physics::Link_V parent_links = link_->GetParentJointsLinks();
  if (parent_links.empty() || !parent_links.front()) {
    gzerr << "[gazebo_motor_model] Rotor link has no parent link.\n";
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 9
  const ignition::math::Pose3d pose_difference =
    link_->WorldCoGPose() - parent_links.front()->WorldCoGPose();
#else
  const ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(
    link_->GetWorldCoGPose() - parent_links.front()->GetWorldCoGPose());
#endif

  const ignition::math::Vector3d torque_rotor(
    0.0, 0.0, torque_z_rotor_frame);
  const ignition::math::Vector3d torque_parent =
    pose_difference.Rot().RotateVector(torque_rotor);
  parent_links.front()->AddRelativeTorque(torque_parent);
}

void GazeboMotorModel::update_legacy_forces_and_moments() {
  const double filtered_command = motor_failed_ ? 0.0 :
    rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

  const double reference_thrust = filtered_command > command_deadband_ ?
    std::pow(
      (filtered_command - motor_quadratic_b_) / motor_quadratic_a_, 2.0) :
    0.0;

  const double target_omega = motor_constant_ > kEpsilon ?
    std::sqrt(std::max(0.0, reference_thrust) / motor_constant_) :
    0.0;

  actual_motor_omega_ =
    std::clamp(target_omega, 0.0, max_rot_velocity_);
  const double signed_joint_velocity =
    static_cast<double>(turning_direction_) * actual_motor_omega_ /
    rotor_velocity_slowdown_sim_;
  joint_->SetVelocity(0, signed_joint_velocity);
  motor_rot_vel_ =
    static_cast<double>(turning_direction_) * actual_motor_omega_;

  double force =
    actual_motor_omega_ * actual_motor_omega_ * motor_constant_;
  if (!reversible_) {
    force = std::abs(force);
  }
  link_->AddRelativeForce(ignition::math::Vector3d(0.0, 0.0, force));

#if GAZEBO_MAJOR_VERSION >= 9
  const ignition::math::Vector3d velocity_world = link_->WorldLinearVel();
  const ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
#else
  const ignition::math::Vector3d velocity_world =
    ignitionFromGazeboMath(link_->GetWorldLinearVel());
  const ignition::math::Vector3d joint_axis =
    ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
#endif

  const ignition::math::Vector3d relative_velocity =
    velocity_world - wind_velocity_;
  const ignition::math::Vector3d velocity_perpendicular =
    relative_velocity - relative_velocity.Dot(joint_axis) * joint_axis;

  const ignition::math::Vector3d air_drag =
    -std::abs(actual_motor_omega_) * rotor_drag_coefficient_ *
    velocity_perpendicular;
  link_->AddForce(air_drag);

  const double reaction_torque_z =
    -static_cast<double>(turning_direction_) * force * moment_constant_;
  apply_reaction_torque(reaction_torque_z);

  const ignition::math::Vector3d rolling_moment =
    -std::abs(actual_motor_omega_) *
    static_cast<double>(turning_direction_) *
    rolling_moment_coefficient_ * velocity_perpendicular;

  const physics::Link_V parent_links = link_->GetParentJointsLinks();
  if (!parent_links.empty() && parent_links.front()) {
    parent_links.front()->AddTorque(rolling_moment);
  }
}

void GazeboMotorModel::update_motor_failure() {
  const bool failed_now =
    motor_number_ == motor_failure_number_ - 1;

  if (failed_now == motor_failed_) {
    return;
  }

  motor_failed_ = failed_now;
  rotor_velocity_filter_ = std::make_unique<FirstOrderFilter<double>>(
    time_constant_up_, time_constant_down_, 0.0);
  actual_motor_omega_ = 0.0;
  induced_velocity_ = 0.0;

  if (motor_failed_) {
    gzerr << "[gazebo_motor_model] Motor " << motor_number_ << " failed.\n";
  } else {
    gzmsg << "[gazebo_motor_model] Motor " << motor_number_
          << " recovered.\n";
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel)

}  // namespace gazebo
