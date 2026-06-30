#include "lite6_controllers/ctc_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>
#include <algorithm>

namespace lite6_controllers
{

Lite6CTCController::Lite6CTCController() {}

controller_interface::CallbackReturn Lite6CTCController::on_init()
{
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  reference_interfaces_.resize(3 * joint_names_.size(), 0.0);
  
  auto_declare<std::vector<double>>("kp", {800.0, 800.0, 800.0, 800.0, 800.0, 800.0});
  auto_declare<std::vector<double>>("kv", {56.6, 56.6, 56.6, 56.6, 56.6, 56.6});
  
  auto_declare<std::vector<double>>("friction_v_nominal", {1.0, 1.0, 0.5, 0.2, 0.1, 0.05});
  auto_declare<std::vector<double>>("friction_c_nominal", {1.5, 1.5, 1.0, 0.5, 0.2, 0.1});
  auto_declare<std::vector<double>>("armature", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
  
  auto_declare<std::vector<double>>("max_effort", {50.0, 50.0, 32.0, 32.0, 32.0, 20.0}); 
  auto_declare<double>("tanh_slope", 300.0);

  auto_declare<double>("kf_q_pos", 1e-5); 
  auto_declare<double>("kf_q_vel", 1e-2); 
  auto_declare<double>("kf_r_pos", 1e-6); 

  auto_declare<std::vector<double>>("estop_decel", {17.5, 12.5, 20.0, 40.0, 40.0, 50.0});
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Lite6CTCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) config.names.push_back(joint + "/effort");
  return config;
}

controller_interface::InterfaceConfiguration Lite6CTCController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) config.names.push_back(joint + "/position");
  return config;
}

std::vector<hardware_interface::CommandInterface> Lite6CTCController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ref_interfaces;
  size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    ref_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), joint_names_[i] + "/position", &reference_interfaces_[i]));
    ref_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), joint_names_[i] + "/velocity", &reference_interfaces_[n + i]));
    ref_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), joint_names_[i] + "/acceleration", &reference_interfaces_[2 * n + i]));
  }
  return ref_interfaces;
}

bool Lite6CTCController::on_set_chained_mode(bool) { return true; }
controller_interface::return_type Lite6CTCController::update_reference_from_subscribers() { return controller_interface::return_type::OK; }

controller_interface::CallbackReturn Lite6CTCController::on_configure(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();
  
  q_.setZero(n); dq_.setZero(n); 
  estop_q_.setZero(n); estop_dq_.setZero(n); 
  a_d_prev_.setZero(n);
  
  e_.setZero(n); de_.setZero(n); a_d_.setZero(n);
  tau_fric_.setZero(n); tau_cmd_.setZero(n);
  q_target_.setZero(n); dq_target_.setZero(n); ddq_target_.setZero(n);

  kf_.resize(n);
  
  auto kp_v = get_node()->get_parameter("kp").as_double_array();
  auto kv_v = get_node()->get_parameter("kv").as_double_array();
  auto fv_v = get_node()->get_parameter("friction_v_nominal").as_double_array();
  auto fc_v = get_node()->get_parameter("friction_c_nominal").as_double_array();
  auto arm_v = get_node()->get_parameter("armature").as_double_array();
  auto effort_limit_v = get_node()->get_parameter("max_effort").as_double_array();
  auto estop_decel_v = get_node()->get_parameter("estop_decel").as_double_array();

  Kp_ = Eigen::MatrixXd::Zero(n, n);
  Kv_ = Eigen::MatrixXd::Zero(n, n);
  friction_v_.resize(n); friction_c_.resize(n); armature_.resize(n); max_effort_.resize(n); estop_decel_.resize(n);

  for(size_t i=0; i<n; ++i) {
      Kp_(i, i) = kp_v[i]; Kv_(i, i) = kv_v[i]; 
      friction_v_[i] = fv_v[i]; friction_c_[i] = fc_v[i]; armature_[i] = arm_v[i];
      max_effort_[i] = effort_limit_v[i];
      estop_decel_[i] = estop_decel_v[i];
  }
  tanh_slope_ = get_node()->get_parameter("tanh_slope").as_double();

  cmd_buffer_.writeFromNonRT(0);
  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
    "~/system_cmd", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Int32::SharedPtr msg) { cmd_buffer_.writeFromNonRT(msg->data); });

  try {
    std::string urdf_path = ament_index_cpp::get_package_share_directory("lite6_description") + "/urdf/lite6.urdf";
    pinocchio::urdf::buildModel(urdf_path, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_activate(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();
  double q_pos_var = get_node()->get_parameter("kf_q_pos").as_double();
  double q_vel_var = get_node()->get_parameter("kf_q_vel").as_double();
  double r_pos_var = get_node()->get_parameter("kf_r_pos").as_double();

  for (size_t i = 0; i < n; ++i) {
    double initial_pos = state_interfaces_[i].get_value();
    q_[i] = initial_pos; dq_[i] = 0.0; a_d_prev_[i] = 0.0;
    kf_[i].init(initial_pos, q_pos_var, q_vel_var, r_pos_var);
    reference_interfaces_[i] = initial_pos;
    reference_interfaces_[n + i] = 0.0;
    reference_interfaces_[2 * n + i] = 0.0;
  }
  
  system_state_ = SystemState::NORMAL;
  cmd_buffer_.writeFromNonRT(0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_deactivate(const rclcpp_lifecycle::State &) { return controller_interface::CallbackReturn::SUCCESS; }

controller_interface::return_type Lite6CTCController::update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return controller_interface::return_type::OK;
  size_t n = joint_names_.size();

  // 1. Kalman Filter Observation
  for (size_t i = 0; i < n; ++i) {
    double q_measured = state_interfaces_[i].get_value();
    kf_[i].predict(ddq_target_[i], dt); 
    kf_[i].update(q_measured);
    q_[i] = kf_[i].x(0);
    dq_[i] = kf_[i].x(1);
  }

  // 2. State Machine & Kinematic Deceleration Trajectory Generation
  int32_t* cmd_ptr = cmd_buffer_.readFromRT();
  int32_t current_cmd = cmd_ptr ? *cmd_ptr : 0;

  if (current_cmd == 1 && system_state_ != SystemState::ESTOP) {
      system_state_ = SystemState::ESTOP;
      estop_q_ = q_; 
      estop_dq_ = dq_;
  } else if (current_cmd == 2 && system_state_ != SystemState::SYS_ID) {
      system_state_ = SystemState::SYS_ID;
  } else if (current_cmd == 0 && system_state_ != SystemState::NORMAL) {
      Eigen::Map<Eigen::VectorXd> map_q_d(reference_interfaces_.data(), n);
      if ((map_q_d - q_).cwiseAbs().maxCoeff() < 0.05) {
          system_state_ = SystemState::NORMAL;
          for(size_t i=0; i<n; ++i) {
              reference_interfaces_[i] = q_[i];
              reference_interfaces_[n+i] = 0.0;
              reference_interfaces_[2*n+i] = 0.0;
          }
      }
  }

  Eigen::Map<Eigen::VectorXd> map_q_d(reference_interfaces_.data(), n);
  Eigen::Map<Eigen::VectorXd> map_dq_d(reference_interfaces_.data() + n, n);
  Eigen::Map<Eigen::VectorXd> map_ddq_d(reference_interfaces_.data() + 2 * n, n);

  if (system_state_ == SystemState::NORMAL || system_state_ == SystemState::SYS_ID) {
      q_target_ = map_q_d; dq_target_ = map_dq_d; ddq_target_ = map_ddq_d;
  } else {
      for (size_t i = 0; i < n; ++i) {
          if (std::abs(estop_dq_[i]) > 1e-4) {
              double sign = (estop_dq_[i] > 0.0) ? 1.0 : -1.0;
              ddq_target_[i] = -sign * estop_decel_[i];
              
              double next_dq = estop_dq_[i] + ddq_target_[i] * dt;
              
              if ((sign > 0.0 && next_dq <= 0.0) || (sign < 0.0 && next_dq >= 0.0)) {
                  double t_zero = std::abs(estop_dq_[i] / estop_decel_[i]);
                  estop_q_[i] += estop_dq_[i] * t_zero + 0.5 * ddq_target_[i] * t_zero * t_zero;
                  estop_dq_[i] = 0.0;
                  ddq_target_[i] = 0.0;
              } else {
                  estop_q_[i] += estop_dq_[i] * dt + 0.5 * ddq_target_[i] * dt * dt;
                  estop_dq_[i] = next_dq;
              }
          } else {
              estop_dq_[i] = 0.0;
              ddq_target_[i] = 0.0;
          }
      }
      q_target_ = estop_q_;
      dq_target_ = estop_dq_;
  }

  // 3. Error Calculation
  e_ = q_target_ - q_;
  de_ = dq_target_ - dq_;

  // 4. Branching Control Law
  if (system_state_ == SystemState::SYS_ID) {
      a_d_.noalias() = ddq_target_ + (Kp_ * e_) + (Kv_ * de_);
      pinocchio::rnea(pin_model_, pin_data_, q_, dq_, a_d_);
      for(size_t i = 0; i < n; ++i) tau_cmd_[i] = pin_data_.tau[i]; 
  } 
  else {
      a_d_.noalias() = ddq_target_ + (Kp_ * e_) + (Kv_ * de_);
      pinocchio::rnea(pin_model_, pin_data_, q_, dq_, a_d_);
      for(size_t i = 0; i < n; ++i) {
          tau_fric_[i] = friction_v_[i] * dq_[i] + friction_c_[i] * std::tanh(tanh_slope_ * (dq_target_[i] + 5.0 * e_[i]));
          tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]) + tau_fric_[i];
      }
  }

  // 5. Hardware Limits
  for (size_t i = 0; i < n; ++i) {
      if (std::isnan(tau_cmd_[i])) tau_cmd_[i] = 0.0;
      tau_cmd_[i] = std::clamp(tau_cmd_[i], -max_effort_[i], max_effort_[i]);
      command_interfaces_[i].set_value(tau_cmd_[i]);
  }
  a_d_prev_ = a_d_; 

  return controller_interface::return_type::OK;
}

}  // namespace lite6_controllers

PLUGINLIB_EXPORT_CLASS(lite6_controllers::Lite6CTCController, controller_interface::ChainableControllerInterface)