#include "lite6_hardware/lite6_mujoco_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

namespace lite6_hardware
{

hardware_interface::CallbackReturn Lite6MujocoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. Initialize hardware interface state and command vectors
  size_t num_joints = info_.joints.size();
  hw_commands_effort_.resize(num_joints, 0.0);
  hw_states_position_.resize(num_joints, 0.0);
  hw_states_velocity_.resize(num_joints, 0.0);
  hw_states_effort_.resize(num_joints, 0.0);

  // 2. Find and load the MuJoCo model
  std::string pkg_path;
  try {
    pkg_path = ament_index_cpp::get_package_share_directory("lite6_description");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Cannot find lite6_description package!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  std::string xml_path = pkg_path + "/mujoco/scene.xml";
  char error_msg[1000] = "Could not load binary model";
  
  m_model = mj_loadXML(xml_path.c_str(), 0, error_msg, 1000);
  if (!m_model) {
    RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Load model error: %s", error_msg);
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  m_data = mj_makeData(m_model);

  // 3. Mapping joints and motors
  RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), "Mapping joints and motors...");
  for (const auto & joint : info_.joints) {
    // 3.1 Find the joints (joint1 ~ joint6)
    int joint_id = mj_name2id(m_model, mjOBJ_JOINT, joint.name.c_str());
    if (joint_id == -1) {
      RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Joint '%s' not found in MuJoCo XML!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    qpos_indices_.push_back(m_model->jnt_qposadr[joint_id]); // Position data offset in qpos array
    qvel_indices_.push_back(m_model->jnt_dofadr[joint_id]);  // Velocity/force data offset in qvel/qfrc arrays

    // 3.2 Find the corresponding actuators (motor1 ~ motor6)
    std::string motor_name = joint.name;
    size_t pos = motor_name.find("joint");
    if (pos != std::string::npos) {
        motor_name.replace(pos, 5, "motor");
    }

    int act_id = mj_name2id(m_model, mjOBJ_ACTUATOR, motor_name.c_str());
    if (act_id == -1) {
      RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Motor '%s' not found in MuJoCo XML!", motor_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    ctrl_indices_.push_back(act_id);
    
    RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), 
      "Mapped URDF %s -> XML Joint ID: %d, XML Motor ID: %d", 
      joint.name.c_str(), joint_id, act_id);
  }

  RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), "MuJoCo Hardware Interface initialized successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Lite6MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Lite6MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type Lite6MujocoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read data from MuJoCo and assign to ROS 2 state_interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    hw_states_position_[i] = m_data->qpos[qpos_indices_[i]];
    hw_states_velocity_[i] = m_data->qvel[qvel_indices_[i]];
    hw_states_effort_[i] = m_data->qfrc_actuator[qvel_indices_[i]]; 
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Lite6MujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if simulation has started (i.e., any non-zero command has been sent)
  bool sim_started_ = false;
  // If simulation is not started, we don't step the physics engine
  if (!sim_started_) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (std::abs(hw_commands_effort_[i]) > 0.01) {
        sim_started_ = true;
        break;
      }
    }
  }

  if (sim_started_) {
    // Write the effort commands to the MuJoCo actuators
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      m_data->ctrl[ctrl_indices_[i]] = hw_commands_effort_[i];
    }

    // Step the physics engine (10000Hz) 
    for (int step = 0; step < 10; ++step) {
      mj_step(m_model, m_data);
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace lite6_hardware

PLUGINLIB_EXPORT_CLASS(
  lite6_hardware::Lite6MujocoSystem,
  hardware_interface::SystemInterface)