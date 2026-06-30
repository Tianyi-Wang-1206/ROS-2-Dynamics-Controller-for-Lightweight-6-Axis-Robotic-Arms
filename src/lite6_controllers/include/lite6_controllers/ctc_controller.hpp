#ifndef LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_
#define LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace lite6_controllers
{

// 1D Kalman Filter for a single joint state estimation
struct KalmanFilter1D {
  Eigen::Vector2d x; // State vector: [position q, velocity dq]^T
  Eigen::Matrix2d P; // Error covariance matrix
  Eigen::Matrix2d Q; // Process noise covariance
  double R;          // Measurement noise covariance

  void init(double initial_pos, double q_pos_var, double q_vel_var, double r_var) {
    x << initial_pos, 0.0;
    P.setZero(); 
    Q << q_pos_var, 0.0,
         0.0, q_vel_var;
    R = r_var;
  }

  void predict(double accel_cmd, double dt) {
    Eigen::Matrix2d F;
    F << 1.0, dt,
         0.0, 1.0;
    Eigen::Vector2d B;
    B << 0.5 * dt * dt, dt;
    
    x = F * x + B * accel_cmd;
    P = F * P * F.transpose() + Q;
  }

  void update(double meas_pos) {
    Eigen::RowVector2d H(1.0, 0.0);
    double S = P(0, 0) + R;
    Eigen::Vector2d K = P.col(0) / S;
    double y = meas_pos - x(0);
    x = x + K * y;
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    P = (I - K * H) * P;
  }
};

class Lite6CTCController : public controller_interface::ChainableControllerInterface
{
public:
  Lite6CTCController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;

  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_;

  Eigen::VectorXd e_, de_, a_d_, tau_fric_, tau_cmd_;
  Eigen::VectorXd q_target_, dq_target_, ddq_target_;

  Eigen::VectorXd q_, dq_;
  Eigen::VectorXd a_d_prev_; 
  std::vector<KalmanFilter1D> kf_; 

  // PD Control Matrices
  Eigen::MatrixXd Kp_, Kv_;
  
  Eigen::VectorXd friction_v_, friction_c_, armature_;
  Eigen::VectorXd max_effort_;
  double tanh_slope_;

  enum class SystemState { NORMAL, ESTOP, SYS_ID };
  SystemState system_state_ = SystemState::NORMAL;
  
  // Kinematic Deceleration Tracker
  Eigen::VectorXd estop_q_, estop_dq_; 
  Eigen::VectorXd estop_decel_;
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_;
  realtime_tools::RealtimeBuffer<int32_t> cmd_buffer_;
};

}  // namespace lite6_controllers

#endif  // LITE6_CONTROLLERS__CTC_CONTROLLER_HPP_