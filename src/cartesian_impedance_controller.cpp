// Copyright (c) 2021 Franka Emika GmbH
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

#include <cartesian_impedance_control/cartesian_impedance_controller.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <functional> // Required for std::bind

#include <Eigen/Eigen>

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}

// Here we implement the controller
namespace cartesian_impedance_control {

// Add this using declaration for placeholders
using std::placeholders::_1;

// Add a new member variable for free movement mode
bool free_movement_mode_ = false;
bool policy_control_mode_ = false;

// Add a new member variable for trajectory playback
bool trajectory_playback_mode_ = false;
std::array<double, 7> playback_joint_positions_;
bool playback_positions_received_ = false;

void CartesianImpedanceController::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  F_contact_des = 0.05 * F_contact_target + 0.95 * F_contact_des;
}


void CartesianImpedanceController::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

void CartesianImpedanceController::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] =
         tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}


inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}


CallbackReturn CartesianImpedanceController::on_init() {
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Ensure that the free_movement_moded can be set via launch file or dynamically via ros2 param set
  free_movement_mode_ = get_node()->declare_parameter<bool>("free_movement_mode", false);
  policy_control_mode_ = get_node()->declare_parameter<bool>("policy_control_mode", false);
  trajectory_playback_mode_ = get_node()->declare_parameter<bool>("trajectory_playback_mode", false);

  RCLCPP_INFO(get_node()->get_logger(), "Free movement mode parameter: %s", free_movement_mode_ ? "ON" : "OFF");
  RCLCPP_INFO(get_node()->get_logger(), "Trajectory Playback Mode parameter: %s", trajectory_playback_mode_ ? "ON" : "OFF");
  RCLCPP_INFO(get_node()->get_logger(), "Policy Control Mode parameter: %s", policy_control_mode_ ? "ON" : "OFF");

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));
                                               
  try {
    rclcpp::QoS qos_profile(1); // Depth of the message queue
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&CartesianImpedanceController::topic_callback, this, std::placeholders::_1));
    std::cout << "Successfully subscribed to robot_state_broadcaster" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
  }

  try {
    // Create subscription for policy outputs
    rclcpp::QoS policy_qos_profile(10);  // Buffer size of 10
    policy_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    
    policy_outputs_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/policy_outputs", policy_qos_profile,
      std::bind(&CartesianImpedanceController::policy_outputs_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_node()->get_logger(), "Successfully subscribed to policy outputs");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Exception thrown during policy outputs subscription creation: %s", e.what());
    return CallbackReturn::ERROR;
  }

  try {
    // Subscribe to the /trajectory_playback/joint_positions topic
    rclcpp::QoS qos_profile(10);  // Buffer size of 10
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    trajectory_playback_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/trajectory_playback/joint_positions", qos_profile,
        std::bind(&CartesianImpedanceController::trajectory_playback_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_node()->get_logger(), "Successfully subscribed to /trajectory_playback/joint_positions");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during trajectory playback subscription creation: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Add parameter for policy control mode
  if (free_movement_mode_ && policy_control_mode_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Both free_movement_mode and policy_control_mode are set to true. "
                "Free movement mode will take precedence.");
    policy_control_mode_ = false;  // Disable policy control mode
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  policy_joint_positions_ = {0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741};
  std::cout << "Completed Activation process" << std::endl;
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

std::array<double, 6> CartesianImpedanceController::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

void CartesianImpedanceController::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
}

// This callback is called when the policy outputs are received
// The function is called whenever a new message is published to the /policy_outputs topic
void CartesianImpedanceController::policy_outputs_callback(
    const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg) {
  
  // Check if the message has at least the expected number of elements (7 joint positions)
  if (msg->data.size() < 7) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Invalid policy outputs: Expected at least 7 joint values, got %zu", msg->data.size());
    return;
  }
  
  // Store the joint positions in the policy_joint_positions_ array
  for (size_t i = 0; i < 7; ++i) {
    policy_joint_positions_[i] = msg->data[i];
  }
  
  // Set flag that we've received policy outputs
  policy_outputs_received_ = true;
  
}

void CartesianImpedanceController::trajectory_playback_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Check if the message has exactly 7 joint positions
    if (msg->data.size() != 7) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Invalid trajectory playback message: Expected 7 joint values, got %zu", msg->data.size());
        return;
    }

    // Store the joint positions in the playback_joint_positions_ array
    for (size_t i = 0; i < 7; ++i) {
        playback_joint_positions_[i] = msg->data[i];
    }

    // Set flag that we've received playback positions
    playback_positions_received_ = true;
}

void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    // Retrieve the joint position and velocity values
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

// The update function is called periodically by the controller manager
// Each call of the update function corresponds to a control loop iteration
controller_interface::return_type CartesianImpedanceController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  

  // Get the current value of the free_movement_mode parameter
  get_node()->get_parameter("free_movement_mode", free_movement_mode_);
  // Get the current value of the policy_control_mode parameter
  get_node()->get_parameter("policy_control_mode", policy_control_mode_);
  // Get the current value of the trajectory_playback_mode parameter
  get_node()->get_parameter("trajectory_playback_mode", trajectory_playback_mode_);

  // Get robot state data from franka_robot_model
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  
  // Map the arrays to Eigen matrices
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  
  // Create rotation quaternion from euler angles
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());
  
  // Update joint states using the state interfaces
  updateJointStates();

  // Initialize torque vector
  Eigen::VectorXd tau_d(7);


  // -------------------------------------TRAJECTORY PLAYBACK MODE------------------------------------------------
  if (trajectory_playback_mode_) {
    // Log that we are in trajectory playback mode
    RCLCPP_INFO(get_node()->get_logger(), "Trajectory Playback Mode is ON");
    if (!playback_positions_received_) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                             "Waiting for trajectory playback positions...");
        return controller_interface::return_type::OK;
    }

    // Convert playback_joint_positions_ to Eigen format
    Eigen::Matrix<double, 7, 1> q_desired_raw;
    for (size_t i = 0; i < 7; ++i) {
        q_desired_raw(i) = playback_joint_positions_[i];
    }

    // Initialize filtered targets on first run
    if (!targets_initialized_) {
        filtered_targets_ = q_;  // Start from the current joint positions
        targets_initialized_ = true;
    }

    // Apply clamping and filtering to smooth target transitions
    double max_delta = 0.1;  // [rad] ~1.72 degrees
    double alpha = 0.005;      // Smoothing factor for filtering
    for (size_t i = 0; i < 7; ++i) {
        // Calculate the delta between the raw desired position and the filtered target
        double delta = q_desired_raw(i) - filtered_targets_(i);

        // Clamp the delta to the maximum allowable range
        delta = std::clamp(delta, -max_delta, max_delta);

        // Update the filtered target with the clamped delta and apply smoothing
        filtered_targets_(i) = alpha * (filtered_targets_(i) + delta) + (1.0 - alpha) * filtered_targets_(i);
    }

    // Set the desired joint position (always use the latest filtered_targets_)
    Eigen::Matrix<double, 7, 1> q_desired = filtered_targets_;

    // Compute joint position error
    Eigen::Matrix<double, 7, 1> position_error = q_desired - q_;

    // Define stiffness (kp) and damping (kd) gains
    Eigen::Matrix<double, 7, 1> kp;
    Eigen::Matrix<double, 7, 1> kd;

    kp << 400, 400, 400, 300, 60, 40, 10;  // Joint-specific stiffness
    for (size_t i = 0; i < 7; ++i) {
        kd(i) = 2.5 * std::sqrt(kp(i));  // Critical damping
    }

    // Calculate joint torques using PD control law with gravity compensation
    tau_d = kp.cwiseProduct(position_error) - kd.cwiseProduct(dq_) + coriolis;

    // Debug output for trajectory playback mode
    if (outcounter % 100 == 0) {
        RCLCPP_INFO(get_node()->get_logger(), "==== Trajectory Playback Mode ====");
        RCLCPP_INFO(get_node()->get_logger(), "Raw Desired Positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    q_desired_raw(0), q_desired_raw(1), q_desired_raw(2), q_desired_raw(3),
                    q_desired_raw(4), q_desired_raw(5), q_desired_raw(6));
        RCLCPP_INFO(get_node()->get_logger(), "Filtered Desired Positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    q_desired(0), q_desired(1), q_desired(2), q_desired(3),
                    q_desired(4), q_desired(5), q_desired(6));
        RCLCPP_INFO(get_node()->get_logger(), "Current Positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    q_(0), q_(1), q_(2), q_(3), q_(4), q_(5), q_(6));
        RCLCPP_INFO(get_node()->get_logger(), "Position Error: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    position_error(0), position_error(1), position_error(2), position_error(3),
                    position_error(4), position_error(5), position_error(6));
        RCLCPP_INFO(get_node()->get_logger(), "==========================");
    }
  }


  // --------------------------------------------------FREE MOVEMENT MODE-------------------------------------------------------------------------------
  else if (free_movement_mode_) {
    // Apply gravity compensation torques
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Matrix<double, 7, 1> damping_torque;
    double damping_coefficient = 0.05;  // Default damping coefficient for all joints

    // Reduce damping for all joints
    for (size_t i = 0; i < 7; ++i) {
        damping_torque(i) = -damping_coefficient * dq_(i);  // Default damping for all joints
    }

    // Calculate joint torques using gravity compensation and reduced damping
    tau_d = damping_torque + coriolis;

    // Print debug information
    if (outcounter % 100 == 0) {
        RCLCPP_INFO(get_node()->get_logger(), "==== Free Movement Mode with Damping ====");
        RCLCPP_INFO(get_node()->get_logger(), "Gravity Compensation Torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              coriolis(0), coriolis(1), coriolis(2), coriolis(3), coriolis(4), coriolis(5), coriolis(6));
        RCLCPP_INFO(get_node()->get_logger(), "Damping Torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              damping_torque(0), damping_torque(1), damping_torque(2), damping_torque(3),
              damping_torque(4), damping_torque(5), damping_torque(6));
        RCLCPP_INFO(get_node()->get_logger(), "==========================");
    }
  }

  // --------------------------------------------------POLICY CONTROL MODE-------------------------------------------------------------------------------
  else if (policy_control_mode_) {
    
    // Define stiffness (kp) and damping (kd) gains
    Eigen::Matrix<double, 7, 1> kp;
    Eigen::Matrix<double, 7, 1> kd;

    // Set joint-specific gains with the desired stiffness values
    kp(0) = 400;  // Joint 1
    kp(1) = 400;  // Joint 2
    kp(2) = 400;  // Joint 3
    kp(3) = 300; // Joint 4
    kp(4) = 60.0; // Joint 5
    kp(5) = 40.0; // Joint 6
    kp(6) = 10.0;  // Joint 7

    // Calculate damping using the critically damped formula: kd = 2*sqrt(kp)
    for (size_t i = 0; i < 7; ++i) {
      kd(i) = 2.5 * std::sqrt(kp(i));   // Critical damping for smoother motion
    }

    // Create target position vector from policy outputs with filtering
    Eigen::Matrix<double, 7, 1> q_desired;

    // Initialize filtered targets on first run
    if (!targets_initialized_) {
      filtered_targets_ = q_;  // Start from the default joint positions
      targets_initialized_ = true;
    }

    // Apply filtering to smooth target transitions
    double max_delta = 0.1;  // [rad] ~1.72 degrees
    
    // Calculate the filtered target positions, policy_joint_positions_ is the target from the policy
    for (size_t i = 0; i < 7; ++i) {
      // Calculate the delta between policy output and filtered target, policy_joint_positions_ is the target from the policy and filtered_targets_ is the previous target
      double delta = policy_joint_positions_[i] - filtered_targets_(i);

      // Clamp the delta to the maximum allowable range
      delta = std::clamp(delta, -max_delta, max_delta);

      // Update the filtered target with the clamped delta -> q_desired_t = q_desired_t-1 + alpha*delta
      filtered_targets_(i) = filtered_targets_(i) + filter_factor_ * delta;
    }

    // Set the desired joint position (always use the latest filtered_targets_)
    q_desired = filtered_targets_;

    // Compute joint position error, q_ is the current joint position obtained from the robot state
    Eigen::Matrix<double, 7, 1> position_error = q_desired - q_;

    // Calculate joint torques using PD control law with gravity compensation
    // tau = kp * (q* - q) - kd * q_dot + coriolis
    tau_d = kp.cwiseProduct(position_error) - kd.cwiseProduct(dq_) + coriolis;

    // Debug output for policy control mode
    if (outcounter % 100 == 0) {  // Log every 100 iterations
      RCLCPP_INFO(get_node()->get_logger(), "==== RL Policy Control Mode ====");
      // Log Policy Targets (raw)
      RCLCPP_INFO(get_node()->get_logger(), "Policy Targets (raw): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            policy_joint_positions_[0], policy_joint_positions_[1], policy_joint_positions_[2],
            policy_joint_positions_[3], policy_joint_positions_[4], policy_joint_positions_[5],
            policy_joint_positions_[6]);

      // Log Filtered/Desired Positions
      RCLCPP_INFO(get_node()->get_logger(), "Desired Positions (filtered): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            q_desired(0), q_desired(1), q_desired(2), q_desired(3), q_desired(4), q_desired(5), q_desired(6));

      // Log Current Positions
      RCLCPP_INFO(get_node()->get_logger(), "Current Positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            q_(0), q_(1), q_(2), q_(3), q_(4), q_(5), q_(6));

      // Log Position Error
      RCLCPP_INFO(get_node()->get_logger(), "Position Error: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            position_error(0), position_error(1), position_error(2), position_error(3),
            position_error(4), position_error(5), position_error(6));

      // Log Current Velocities
      RCLCPP_INFO(get_node()->get_logger(), "Current Velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            dq_(0), dq_(1), dq_(2), dq_(3), dq_(4), dq_(5), dq_(6));

       // Log Calculated Torques (before saturation)
      RCLCPP_INFO(get_node()->get_logger(), "Calculated Torques (PD+C): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            tau_d(0), tau_d(1), tau_d(2), tau_d(3), tau_d(4), tau_d(5), tau_d(6));

      RCLCPP_INFO(get_node()->get_logger(), "==========================");
    }
  } 

  // -------------------------------------------ORIGINAL CARTESIAN IMPEDANCE CONTROL MODE---------------------------------------------------------
  else {
    // Calculate cartesian error

    // Position error
    error.head(3) << position - position_d_;

    //Orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    error.tail(3) << -transform.rotation() * error.tail(3);
    
    // Calculate error integral for pose
    I_error += Sm * dt * integrator_weights.cwiseProduct(error);
    // Limit integral error to avoid windup
    for (int i = 0; i < 6; i++){
      I_error(i,0) = std::min(std::max(-max_I(i,0), I_error(i,0)), max_I(i,0)); 
    }

    // Calculate operational space inertia matrix
    Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
    
    // Set Theta for impedance control
    Theta = Lambda;

    // Calculate critically damped damping matrix
    D = D_gain * K.cwiseMax(0.0).cwiseSqrt() * Lambda.cwiseMax(0.0).diagonal().cwiseSqrt().asDiagonal();
    // This creates a block diagonal structure [D_pos, 0; 0, D_rot], where D_pos and D_rot are 3x3 matrices for translational and rotational damping
    D.topRightCorner(3,3).setZero();
    D.bottomLeftCorner(3,3).setZero();
    
    // Calculate impedance force, negative sign: forces should pull towards the desired pose
    F_impedance = -1 * (D * (jacobian * dq_) + K * error);

    // Filter external force and calculate contact force error integral
    F_ext = 0.9 * F_ext + 0.1 * O_F_ext_hat_K_M;
    I_F_error += dt * Sf * (F_contact_des - F_ext);
    F_cmd = Sf * (0.4 * (F_contact_des - F_ext) + 0.9 * I_F_error + 0.9 * F_contact_des);

    // Calculate required torques
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_impedance(7);
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Nullspace component
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                      (nullspace_stiffness_ * config_control * (q_d_nullspace_ - q_) -
                      (2.0 * sqrt(nullspace_stiffness_)) * dq_);

    // Impedance and force control components
    tau_impedance = jacobian.transpose() * Sm * F_impedance + jacobian.transpose() * Sf * F_cmd;
    
    // Combined desired torque with coriolis compensation -> Sent to the robot
    tau_d = tau_impedance + tau_nullspace + coriolis;

    // Debug output for cartesian control mode
    if (outcounter % 100 == 0) {  // Log every 10 iterations
        RCLCPP_INFO(get_node()->get_logger(), "\n==== Cartesian Impedance Control Mode ====");
        
        // Log external forces
        RCLCPP_INFO(get_node()->get_logger(), "External Forces [N]: [Fx: %.3f, Fy: %.3f, Fz: %.3f, Tx: %.3f, Ty: %.3f, Tz: %.3f]",
                    O_F_ext_hat_K[0], O_F_ext_hat_K[1], O_F_ext_hat_K[2],
                    O_F_ext_hat_K[3], O_F_ext_hat_K[4], O_F_ext_hat_K[5]);

        // Log computed torques
        RCLCPP_INFO(get_node()->get_logger(), "Computed Torques [Nm]: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    tau_d(0), tau_d(1), tau_d(2), tau_d(3), tau_d(4), tau_d(5), tau_d(6));

        RCLCPP_INFO(get_node()->get_logger(), "====================================================");
    }
  }
  // -------------------------------------------------------------------------------------------

  // Apply torque rate limiting for smooth control regardless of control mode,
  tau_d = saturateTorqueRate(tau_d, tau_J_d_M);
  tau_J_d_M = tau_d;

  // Send joint torque commands to the robot via the command interfaces, command_interfaces_ are hardware interfaces 
  // Each interface maps to a joint torque command eg. /panda_joint1/effort
  // Calling .set_value() on each command interface sets the desired torque for that joint
  
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  outcounter++;
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
} // namespace cartesian_impedance_control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_impedance_control::CartesianImpedanceController,
                       controller_interface::ControllerInterface)