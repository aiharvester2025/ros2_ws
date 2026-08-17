// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

// Keep Gazebo and RViz in one measured kinematic state without depending on
// Gazebo Classic's ROS service bridge.  On this Foxy installation those
// services can be advertised while remaining unresponsive, whereas a
// ModelPlugin receives ROS messages reliably through gazebo_ros::Node's
// executor.  Articulated joints default to bounded kinematic updates: force
// PID controllers made the long boom/rail/arm chain inject reaction impulses
// into otherwise sensor-only simulations.

#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace oil_palm_harvester_description
{

class HarvesterKinematicGazeboPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = std::move(model);
    this->ros_node_ = gazebo_ros::Node::Get(sdf);
    if (sdf && sdf->HasElement("articulation_control_mode")) {
      const std::string requested_mode =
        sdf->Get<std::string>("articulation_control_mode");
      if (requested_mode == "pid") {
        this->use_kinematic_articulation_control_ = false;
      } else if (requested_mode != "kinematic") {
        RCLCPP_WARN(
          this->ros_node_->get_logger(),
          "Unknown articulation_control_mode '%s'; using safe kinematic mode",
          requested_mode.c_str());
      }
    }
    this->base_pose_ = this->model_->WorldPose();
    // The harvester is a commanded sensor-development model, not a free-body
    // contact-physics machine yet.  Disable gravity for every link.  GUI
    // commands are applied at a bounded 20 Hz rate rather than at every
    // physics iteration.
    for (const auto & link : this->model_->GetLinks()) {
      if (link) {
        link->SetGravityMode(false);
        link->SetLinearVel(ignition::math::Vector3d::Zero);
        link->SetAngularVel(ignition::math::Vector3d::Zero);
      }
    }

    this->joint_controller_ = this->model_->GetJointController();
    for (const auto & joint : this->model_->GetJoints()) {
      if (joint && joint->DOF() > 0U) {
        // Gazebo registers loaded model joints with its JointController.  Use
        // the controller's scoped name, rather than the bare ROS joint name,
        // when configuring and commanding that controller.
        const std::string controller_joint_name = joint->GetScopedName();
        this->controller_joint_names_[joint->GetName()] = controller_joint_name;
        if (this->use_kinematic_articulation_control_) {
          const double measured_position = joint->Position(0);
          const double initial_position = std::isfinite(measured_position) ?
            ClampToJointLimits(joint, measured_position) : 0.0;
          KinematicJoint specification;
          specification.joint = joint;
          specification.controller_joint_name = controller_joint_name;
          specification.lower_limit = joint->LowerLimit(0);
          specification.upper_limit = joint->UpperLimit(0);
          specification.maximum_rate = KinematicRateForJoint(joint);
          this->kinematic_joints_[joint->GetName()] = specification;
          this->kinematic_joint_goals_[joint->GetName()] = initial_position;
          this->applied_kinematic_joint_positions_[joint->GetName()] = initial_position;
        } else if (joint->GetName() == kTurretJointName) {
          // The turret carries the complete boom/arm chain.  A persistent
          // force PID on that chain can feed reaction torque back into the
          // model even after a GUI slider stops.  It is instead moved as a
          // bounded kinematic sensor-development joint in OnUpdate().  A
          // JointController is explicitly detached so it cannot contribute
          // any residual position PID force for this one joint.
          this->turret_joint_ = joint;
          this->joint_controller_->RemoveJoint(joint.get());
          const double measured_position = joint->Position(0);
          const double initial_position = std::isfinite(measured_position) ?
            ClampToJointLimits(joint, measured_position) : 0.0;
          this->turret_goal_position_ = initial_position;
          this->turret_applied_target_ = initial_position;
        } else {
          this->joint_controller_->SetPositionPID(
            controller_joint_name,
            // The GUI can make a large step in one message (especially
            // Randomize).  Clamp the effort/torque command so this controller
            // cannot generate an unbounded reaction impulse in the movable
            // harvester model.
            gazebo::common::PID(
              kPositionControllerP, 0.0, kPositionControllerD,
              0.0, 0.0,
              kPositionControllerMaxCommand, -kPositionControllerMaxCommand));
        }
      }
    }

    if (this->use_kinematic_articulation_control_) {
      // Loaded models register their joints with Gazebo's JointController.
      // Reset clears any retained PID targets without removing those joints:
      // Model::SetJointPositions uses that registered set for its one bounded
      // kinematic batch per update interval.
      this->joint_controller_->Reset();
    }

    // Commands and feedback deliberately use separate topics.  Publishing
    // GUI target values directly to robot_state_publisher made RViz lead the
    // PID-controlled Gazebo model and its simulated sensors.
    this->joint_subscription_ = this->ros_node_->create_subscription<
      sensor_msgs::msg::JointState>(
      "/harvester/joint_commands", rclcpp::QoS(10),
      std::bind(&HarvesterKinematicGazeboPlugin::OnJointState, this, std::placeholders::_1));
    this->joint_state_publisher_ = this->ros_node_->create_publisher<
      sensor_msgs::msg::JointState>("/harvester/joint_states", rclcpp::QoS(10));
    this->velocity_subscription_ = this->ros_node_->create_subscription<
      geometry_msgs::msg::Twist>(
      "/harvester/cmd_vel", rclcpp::QoS(10),
      std::bind(&HarvesterKinematicGazeboPlugin::OnVelocity, this, std::placeholders::_1));
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->ros_node_);
    this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&HarvesterKinematicGazeboPlugin::OnUpdate, this, std::placeholders::_1));

    if (this->use_kinematic_articulation_control_) {
      RCLCPP_INFO(
        this->ros_node_->get_logger(),
        "All %zu movable joints use rate-limited kinematic control at %.0f Hz "
        "(turret limited to %.2f rad/s)",
        this->kinematic_joints_.size(), 1.0 / kKinematicJointUpdatePeriod,
        kTurretMaximumTargetRate);
    } else {
      RCLCPP_WARN(
        this->ros_node_->get_logger(),
        "Using legacy PID articulation control; this fallback can be unstable "
        "for large slider steps");
    }
    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "Harvester kinematic bridge ready: joint commands on /harvester/joint_commands, "
      "measured joint states on /harvester/joint_states, and base velocity on /harvester/cmd_vel");
  }

private:
  void OnJointState(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->name.size() != message->position.size()) {
      RCLCPP_WARN(this->ros_node_->get_logger(), "Ignoring malformed JointState message");
      return;
    }
    std::lock_guard<std::mutex> lock(this->mutex_);
    bool changed = false;
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      const double requested_position = message->position[index];
      if (!std::isfinite(requested_position)) {
        continue;
      }

      if (this->use_kinematic_articulation_control_) {
        const auto kinematic_joint = this->kinematic_joints_.find(message->name[index]);
        if (kinematic_joint == this->kinematic_joints_.end()) {
          continue;
        }
        const double bounded_position = ClampToJointLimits(
          kinematic_joint->second, requested_position);
        const auto existing = this->kinematic_joint_goals_.find(message->name[index]);
        if (existing == this->kinematic_joint_goals_.end() ||
          std::abs(existing->second - bounded_position) > kJointPositionTolerance)
        {
          this->kinematic_joint_goals_[message->name[index]] = bounded_position;
        }
        continue;
      }

      const auto controller_joint = this->controller_joint_names_.find(message->name[index]);
      if (controller_joint == this->controller_joint_names_.end()) {
        continue;
      }
      if (message->name[index] == kTurretJointName) {
        const double bounded_position = this->turret_joint_ ?
          ClampToJointLimits(this->turret_joint_, requested_position) : requested_position;
        if (std::abs(this->turret_goal_position_ - bounded_position) >
          kJointPositionTolerance)
        {
          this->turret_goal_position_ = bounded_position;
        }
        continue;
      }

      const auto existing = this->requested_joint_positions_.find(message->name[index]);
      if (existing == this->requested_joint_positions_.end() ||
        std::abs(existing->second - requested_position) > kJointPositionTolerance)
      {
        this->requested_joint_positions_[message->name[index]] = requested_position;
        changed = true;
      }
    }
    if (changed) {
      this->joint_state_dirty_ = true;
    }
  }

  void OnVelocity(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->linear_x_ = message->linear.x;
    this->angular_z_ = message->angular.z;
    this->velocity_message_pending_ = true;
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    std::map<std::string, double> positions;
    double linear_x;
    double angular_z;
    double turret_goal_position;
    bool velocity_message_pending;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->joint_state_dirty_) {
        positions = this->requested_joint_positions_;
        this->joint_state_dirty_ = false;
      }
      linear_x = this->linear_x_;
      angular_z = this->angular_z_;
      turret_goal_position = this->turret_goal_position_;
      velocity_message_pending = this->velocity_message_pending_;
      this->velocity_message_pending_ = false;
    }

    // Follow normal cmd_vel behaviour: a base command must be refreshed.
    // This prevents a terminal publisher that is interrupted from leaving the
    // harvester travelling indefinitely.
    if (velocity_message_pending) {
      this->last_velocity_command_sim_time_ = info.simTime;
      this->has_velocity_command_ = true;
    }
    if (this->has_velocity_command_ &&
      (info.simTime - this->last_velocity_command_sim_time_).Double() >
      kVelocityCommandTimeout)
    {
      linear_x = 0.0;
      angular_z = 0.0;
    }

    double dt = 0.0;
    bool base_pose_changed = false;
    if (this->last_sim_time_ != gazebo::common::Time::Zero) {
      dt = (info.simTime - this->last_sim_time_).Double();
      if (dt > 0.0 && dt < 1.0 &&
        (std::abs(linear_x) > kVelocityTolerance || std::abs(angular_z) > kVelocityTolerance))
      {
        const double yaw = this->base_pose_.Rot().Yaw() + angular_z * dt;
        this->base_pose_.Pos().X(
          this->base_pose_.Pos().X() + linear_x * std::cos(yaw) * dt);
        this->base_pose_.Pos().Y(
          this->base_pose_.Pos().Y() + linear_x * std::sin(yaw) * dt);
        this->base_pose_.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw);
        base_pose_changed = true;
      }
    }
    this->last_sim_time_ = info.simTime;

    if (this->use_kinematic_articulation_control_) {
      this->UpdateKinematicJointTargets(info.simTime);
    } else {
      if (!positions.empty()) {
        // Legacy fallback only.  The default articulation path above avoids
        // persistent force PIDs because they can energize the long boom/arm
        // chain after a slider command has stopped.
        for (const auto & position : positions) {
          const auto controller_joint = this->controller_joint_names_.find(position.first);
          if (controller_joint != this->controller_joint_names_.end() &&
            !this->joint_controller_->SetPositionTarget(
              controller_joint->second, position.second))
          {
            RCLCPP_WARN(
              this->ros_node_->get_logger(),
              "Gazebo position controller rejected joint '%s'",
              position.first.c_str());
          }
        }
      }
      this->UpdateTurretTarget(turret_goal_position, info.simTime);
    }

    // Keep the robot movable through /harvester/cmd_vel, but update its root
    // at a modest rate.  This avoids the ODE SliderJoint warning storm caused
    // by calling SetWorldPose at the 1 kHz physics rate.
    if (base_pose_changed &&
      (this->last_base_pose_sim_time_ == gazebo::common::Time::Zero ||
      (info.simTime - this->last_base_pose_sim_time_).Double() >= kBasePoseUpdatePeriod))
    {
      this->model_->SetWorldPose(this->base_pose_);
      this->model_->ResetPhysicsStates();
      this->last_base_pose_sim_time_ = info.simTime;
    }

    // RViz needs a regular root transform, but publishing at the physics step
    // rate is unnecessary and adds load while Gazebo is rendering meshes.
    if (this->last_transform_sim_time_ == gazebo::common::Time::Zero ||
      (info.simTime - this->last_transform_sim_time_).Double() >= 0.05)
    {
      this->PublishMeasuredJointStates();
      this->PublishBaseTransform();
      this->last_transform_sim_time_ = info.simTime;
    }
  }

  void PublishMeasuredJointStates()
  {
    sensor_msgs::msg::JointState joint_state;
    // The GUI / robot_state_publisher TF chain is wall-clock stamped.  Keep
    // this feedback in that same time domain; Gazebo sensor streams which need
    // latest TF are normalized separately by their existing relays.
    rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    joint_state.header.stamp = wall_clock.now();
    for (const auto & joint : this->model_->GetJoints()) {
      if (!joint || joint->DOF() == 0U) {
        continue;
      }
      const double position = joint->Position(0);
      if (!std::isfinite(position)) {
        RCLCPP_WARN_THROTTLE(
          this->ros_node_->get_logger(), *this->ros_node_->get_clock(), 5000,
          "Skipping non-finite measured position from Gazebo joint '%s'",
          joint->GetName().c_str());
        continue;
      }
      joint_state.name.push_back(joint->GetName());
      joint_state.position.push_back(position);
    }
    this->joint_state_publisher_->publish(joint_state);
  }

  static double ClampToJointLimits(
    const gazebo::physics::JointPtr & joint, const double position)
  {
    if (!joint || !std::isfinite(position)) {
      return position;
    }
    const double lower = joint->LowerLimit(0);
    const double upper = joint->UpperLimit(0);
    if (std::isfinite(lower) && std::isfinite(upper) && lower <= upper) {
      return std::max(lower, std::min(position, upper));
    }
    return position;
  }

  struct KinematicJoint
  {
    gazebo::physics::JointPtr joint;
    std::string controller_joint_name;
    double lower_limit{0.0};
    double upper_limit{0.0};
    double maximum_rate{0.0};
  };

  static double ClampToJointLimits(
    const KinematicJoint & joint, const double position)
  {
    if (!std::isfinite(position)) {
      return position;
    }
    if (std::isfinite(joint.lower_limit) && std::isfinite(joint.upper_limit) &&
      joint.lower_limit <= joint.upper_limit)
    {
      return std::max(joint.lower_limit, std::min(position, joint.upper_limit));
    }
    return position;
  }

  static double KinematicRateForJoint(const gazebo::physics::JointPtr & joint)
  {
    if (!joint) {
      return kFallbackKinematicJointRate;
    }
    const double velocity_limit = joint->GetVelocityLimit(0);
    double rate = (std::isfinite(velocity_limit) && velocity_limit > 0.0) ?
      std::min(velocity_limit, kMaximumKinematicJointRate) :
      kFallbackKinematicJointRate;
    if (joint->GetName() == kTurretJointName) {
      rate = std::min(rate, kTurretMaximumTargetRate);
    }
    return rate;
  }

  void RebaseKinematicJointPositions()
  {
    for (const auto & entry : this->kinematic_joints_) {
      const auto & specification = entry.second;
      if (!specification.joint) {
        continue;
      }
      const double measured_position = specification.joint->Position(0);
      if (std::isfinite(measured_position)) {
        this->applied_kinematic_joint_positions_[entry.first] =
          ClampToJointLimits(specification, measured_position);
      }
    }
  }

  void UpdateKinematicJointTargets(const gazebo::common::Time & sim_time)
  {
    if (this->last_kinematic_joint_update_sim_time_ == gazebo::common::Time::Zero) {
      this->last_kinematic_joint_update_sim_time_ = sim_time;
      return;
    }

    const double elapsed =
      (sim_time - this->last_kinematic_joint_update_sim_time_).Double();
    if (elapsed < 0.0) {
      // Simulation reset/rewind: recover the measured starting positions but
      // retain the latest GUI goals, then ramp back without a discontinuity.
      this->RebaseKinematicJointPositions();
      this->last_kinematic_joint_update_sim_time_ = sim_time;
      return;
    }
    if (elapsed < kKinematicJointUpdatePeriod) {
      return;
    }
    this->last_kinematic_joint_update_sim_time_ = sim_time;

    std::map<std::string, double> goals;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      goals = this->kinematic_joint_goals_;
    }

    // A paused/unpaused world must not cause a large one-tick articulation
    // jump.  The GUI naturally emits frequent incremental positions while a
    // slider is dragged, and this cap also bounds programmatic commands.
    const double bounded_dt = std::min(elapsed, kMaximumKinematicRampInterval);
    std::map<std::string, double> changed_joint_positions;
    std::map<std::string, double> next_applied_positions;
    for (const auto & entry : this->kinematic_joints_) {
      const auto goal = goals.find(entry.first);
      const auto applied = this->applied_kinematic_joint_positions_.find(entry.first);
      if (goal == goals.end() || applied == this->applied_kinematic_joint_positions_.end()) {
        continue;
      }

      const KinematicJoint & specification = entry.second;
      const double bounded_goal = ClampToJointLimits(specification, goal->second);
      const double maximum_step = specification.maximum_rate * bounded_dt;
      const double remaining = bounded_goal - applied->second;
      const double step = std::max(-maximum_step, std::min(remaining, maximum_step));
      const double next_position = ClampToJointLimits(
        specification, applied->second + step);
      if (std::abs(next_position - applied->second) <= kJointPositionTolerance) {
        continue;
      }

      // Use the scoped name expected by Gazebo's pre-registered
      // JointController.  One Model::SetJointPositions call applies the
      // complete changed set exactly once at this 20 Hz boundary.
      changed_joint_positions[specification.controller_joint_name] = next_position;
      next_applied_positions[entry.first] = next_position;
    }

    if (changed_joint_positions.empty()) {
      return;
    }

    this->model_->SetJointPositions(changed_joint_positions);
    for (const auto & applied : next_applied_positions) {
      this->applied_kinematic_joint_positions_[applied.first] = applied.second;
    }
    // Direct Gazebo joint poses can leave residual twist/force on a dynamic
    // child chain.  Clearing it once after the complete batch prevents the
    // rail/lift/platform/boom motion from continuing after its target stops.
    this->model_->ResetPhysicsStates();
  }

  void UpdateTurretTarget(
    const double requested_goal, const gazebo::common::Time & sim_time)
  {
    if (!this->turret_joint_) {
      return;
    }

    if (this->last_turret_pose_update_sim_time_ == gazebo::common::Time::Zero) {
      this->last_turret_pose_update_sim_time_ = sim_time;
      return;
    }
    const double elapsed =
      (sim_time - this->last_turret_pose_update_sim_time_).Double();
    if (elapsed < kTurretKinematicUpdatePeriod) {
      return;
    }
    this->last_turret_pose_update_sim_time_ = sim_time;

    const double bounded_goal = ClampToJointLimits(this->turret_joint_, requested_goal);
    // Do not let a pause/unpause time jump become a large target jump.
    const double bounded_dt = std::min(elapsed, kTurretMaximumRampInterval);
    const double maximum_step = kTurretMaximumTargetRate * bounded_dt;
    const double remaining = bounded_goal - this->turret_applied_target_;
    const double measured_position = this->turret_joint_->Position(0);
    const bool needs_hold = !std::isfinite(measured_position) ||
      std::abs(measured_position - this->turret_applied_target_) > kJointPositionTolerance;
    if ((std::abs(remaining) <= kJointPositionTolerance && !needs_hold) ||
      maximum_step <= 0.0)
    {
      return;
    }

    if (std::abs(remaining) > kJointPositionTolerance) {
      const double step = std::max(-maximum_step, std::min(remaining, maximum_step));
      this->turret_applied_target_ = ClampToJointLimits(
        this->turret_joint_, this->turret_applied_target_ + step);
    }

    // SetPosition is intentionally restricted to this single revolute joint
    // and rate-limited to 20 Hz.  In particular, never use this path for the
    // boom's prismatic joints: repeating it there caused ODE SliderJoint
    // anchor warnings in earlier versions of the simulation.
    if (!this->turret_joint_->SetPosition(0, this->turret_applied_target_, false))
    {
      RCLCPP_WARN_THROTTLE(
        this->ros_node_->get_logger(), *this->ros_node_->get_clock(), 5000,
        "Gazebo rejected kinematic turret target");
      return;
    }
    // SetPosition updates the complete turret child subtree.  Clear any
    // residual velocity/force state so the old dynamic chain cannot continue
    // moving after the commanded pose has been reached.
    this->model_->ResetPhysicsStates();
  }

  void PublishBaseTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    // The GUI and robot_state_publisher use wall-clock stamps.  Sensor data
    // destined for RViz is normalized to this same time domain by the launch
    // relay, so the whole dynamic TF chain remains coherent in Foxy.
    rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
    transform.header.stamp = wall_clock.now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    // Use the measured Gazebo root pose, not the cmd_vel integration target.
    // That keeps RViz, range rays, camera/lidar frames, and Gazebo aligned
    // even during the 20 Hz base-pose update interval.
    const auto base_link = this->model_->GetLink("base_link");
    const ignition::math::Pose3d pose = base_link ?
      base_link->WorldPose() : this->model_->WorldPose();
    transform.transform.translation.x = pose.Pos().X();
    transform.transform.translation.y = pose.Pos().Y();
    transform.transform.translation.z = pose.Pos().Z();
    transform.transform.rotation.x = pose.Rot().X();
    transform.transform.rotation.y = pose.Rot().Y();
    transform.transform.rotation.z = pose.Rot().Z();
    transform.transform.rotation.w = pose.Rot().W();
    this->tf_broadcaster_->sendTransform(transform);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointControllerPtr joint_controller_;
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  gazebo::common::Time last_sim_time_;
  gazebo::common::Time last_transform_sim_time_;
  gazebo::common::Time last_base_pose_sim_time_;
  gazebo::common::Time last_velocity_command_sim_time_;
  gazebo::common::Time last_turret_pose_update_sim_time_;
  gazebo::common::Time last_kinematic_joint_update_sim_time_;
  ignition::math::Pose3d base_pose_;
  gazebo::physics::JointPtr turret_joint_;
  std::map<std::string, double> requested_joint_positions_;
  std::map<std::string, std::string> controller_joint_names_;
  std::map<std::string, KinematicJoint> kinematic_joints_;
  std::map<std::string, double> kinematic_joint_goals_;
  std::map<std::string, double> applied_kinematic_joint_positions_;
  double turret_goal_position_{0.0};
  double turret_applied_target_{0.0};
  bool use_kinematic_articulation_control_{true};
  bool joint_state_dirty_{false};
  bool velocity_message_pending_{false};
  bool has_velocity_command_{false};
  std::mutex mutex_;
  double linear_x_{0.0};
  double angular_z_{0.0};

  static constexpr double kJointPositionTolerance = 1e-6;
  static constexpr double kVelocityTolerance = 1e-9;
  static constexpr double kVelocityCommandTimeout = 0.5;
  static constexpr double kBasePoseUpdatePeriod = 0.05;
  static constexpr double kPositionControllerP = 2500.0;
  static constexpr double kPositionControllerD = 250.0;
  static constexpr double kPositionControllerMaxCommand = 2000.0;
  static constexpr const char * kTurretJointName = "boom_turret_joint";
  // All kinematic articulation targets are updated only at this cadence.
  // This specifically avoids the 1 kHz SliderJoint anchor-warning flood
  // produced by older direct-position loops.
  static constexpr double kKinematicJointUpdatePeriod = 0.05;
  static constexpr double kMaximumKinematicRampInterval = 0.05;
  static constexpr double kMaximumKinematicJointRate = 0.25;
  static constexpr double kFallbackKinematicJointRate = 0.05;
  // Turret-specific cap remains below its URDF velocity limit because it
  // carries the complete boom/arm chain.
  static constexpr double kTurretMaximumTargetRate = 0.05;
  static constexpr double kTurretMaximumRampInterval = 0.05;
  static constexpr double kTurretKinematicUpdatePeriod = 0.05;
};

GZ_REGISTER_MODEL_PLUGIN(HarvesterKinematicGazeboPlugin)

}  // namespace oil_palm_harvester_description
