// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

// Keep Gazebo and RViz in one measured kinematic state without depending on
// Gazebo Classic's ROS service bridge.  On this Foxy installation those
// services can be advertised while remaining unresponsive, whereas a
// ModelPlugin receives ROS messages reliably through gazebo_ros::Node's
// executor.

#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

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
    this->base_pose_ = this->model_->WorldPose();
    // The harvester is a commanded sensor-development model, not a free-body
    // contact-physics machine yet.  Disable gravity for every link.  GUI
    // targets are held by Gazebo's persistent position controller instead of
    // being teleported at every physics iteration.
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
      const auto existing = this->requested_joint_positions_.find(message->name[index]);
      if (existing == this->requested_joint_positions_.end() ||
        std::abs(existing->second - message->position[index]) > kJointPositionTolerance)
      {
        this->requested_joint_positions_[message->name[index]] = message->position[index];
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
    bool velocity_message_pending;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->joint_state_dirty_) {
        positions = this->requested_joint_positions_;
        this->joint_state_dirty_ = false;
      }
      linear_x = this->linear_x_;
      angular_z = this->angular_z_;
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

    bool base_pose_changed = false;
    if (this->last_sim_time_ != gazebo::common::Time::Zero) {
      const double dt = (info.simTime - this->last_sim_time_).Double();
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

    if (!positions.empty()) {
      // Update persistent controller targets.  These commands move the
      // articulated links through the physics engine without a one-step pose
      // teleport or a SliderJoint anchor warning flood.
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
  ignition::math::Pose3d base_pose_;
  std::map<std::string, double> requested_joint_positions_;
  std::map<std::string, std::string> controller_joint_names_;
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
};

GZ_REGISTER_MODEL_PLUGIN(HarvesterKinematicGazeboPlugin)

}  // namespace oil_palm_harvester_description
