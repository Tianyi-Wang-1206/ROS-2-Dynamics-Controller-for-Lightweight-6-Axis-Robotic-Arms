#!/usr/bin/env python3
import os
import sys
import math
import threading
import numpy as np

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint, 
    PositionConstraint, OrientationConstraint, BoundingVolume, CollisionObject
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

# Pinocchio for forward kinematics
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory

# PyQt5 imports
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QSlider, QLineEdit, QTabWidget, QGroupBox, 
    QFormLayout, QGridLayout, QMessageBox, QFrame, QStyle, QProgressBar
)
from PyQt5.QtGui import QFont, QDoubleValidator, QPalette, QColor

# System ID Engine Import
try:
    from lite6_hmi.sysid_engine import SysIdEngine
except ImportError as e:
    print(f"[ERROR] Failed to import SysIdEngine: {e}. Check directory structure.")
    raise e


class Lite6StateMachine:
    """State management class corresponding to the C++ controller's internal states."""
    INIT = "INITIALIZING..."
    IDLE = "IDLE (Ready)"
    PLANNING = "PLANNING..."
    EXECUTING = "EXECUTING..."
    ERROR = "HARDWARE E-STOP!"
    RECOVERING = "RECOVERING..."
    SOFT_ERROR = "SOFTWARE FAULT"
    
    SYSID_PRE_ALIGN  = "SYSID: PRE-ALIGNING..."
    SYSID_SEND_CMD   = "SYSID: CONFIGURING HARDWARE..."
    SYSID_RUNNING    = "SYSID: EXCITING TRAJECTORY..."
    SYSID_CALC       = "SYSID: LEAST SQUARES SOLVING..."
    SYSID_POST_ALIGN = "SYSID: ALIGNING JTC TARGETS..."
    SYSID_RESTORE    = "SYSID: RESTORING CLOSED-LOOP..."


class ROS2Worker(QThread):
    """
    Dedicated QThread running rclpy.spin.
    Dispatches hardware state updates to the main GUI thread via PyQt signals.
    """
    telemetry_signal = pyqtSignal(dict)
    state_signal = pyqtSignal(str, str)
    sysid_yaml_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = Node('lite6_industrial_hmi')
        self.state = Lite6StateMachine.INIT
        self.error_msg = ""
        self.state_lock = threading.RLock()
        self.last_state_rx_time = 0.0

        # Publishers
        self.cmd_pub = self.node.create_publisher(Int32, '/ctc_controller/system_cmd', 10)
        self.co_pub = self.node.create_publisher(CollisionObject, '/collision_object', 10)
        self.jtc_pub = self.node.create_publisher(JointTrajectory, '/lite6_arm_controller/joint_trajectory', 10)

        # Joint State variables
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.q_curr = [0.0] * 6
        self.dq_curr = [0.0] * 6
        self.tau_curr = [0.0] * 6
        self.pose_curr = [0.0] * 6

        # Action Client for MoveGroup
        self.move_client = ActionClient(self.node, MoveGroup, 'move_action')
        self.current_goal_handle = None

        # Pinocchio Kinematics Initialization
        try:
            pkg_path = get_package_share_directory('lite6_description')
            urdf_path = os.path.join(pkg_path, 'urdf', 'lite6.urdf')
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.eef_frame_id = self.model.getFrameId("link_eef")
            self.node.get_logger().info("Pinocchio Model initialized in C++ Qt Context.")
        except Exception as e:
            self.node.get_logger().error(f"Pinocchio Kinematics init failed: {e}")

        # SysId Mathematical Engine
        try:
            self.sysid_calc = SysIdEngine()
            self.node.get_logger().info("Fourier System ID Engine initialized successfully.")
        except Exception as e:
            self.node.get_logger().error(f"SysID Engine failed: {e}")
            self.sysid_calc = None

        # Subscribers and Timers
        self.node.create_subscription(JointState, '/joint_states', self.joint_cb, 200)
        self.node.create_timer(1.0, self.publish_ground_plane)
        
        # Delayed system initialization
        self.init_timer = self.node.create_timer(2.0, self.sys_ready)

        # SysId variables
        self.sysid_record_data = []
        self.sysid_start_ros_time = None
        self.sysid_q0_locked = [0.0] * 6

    def run(self):
        """Thread execution entry point."""
        rclpy.spin(self.node)

    def sys_ready(self):
        self.init_timer.cancel()
        with self.state_lock:
            if self.state == Lite6StateMachine.INIT:
                self.set_state(Lite6StateMachine.IDLE)

    def set_state(self, new_state, error_msg=""):
        with self.state_lock:
            self.state = new_state
            self.error_msg = error_msg
            self.state_signal.emit(self.state, self.error_msg)

    def joint_cb(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_sec = self.node.get_clock().now().nanoseconds * 1e-9

        with self.state_lock:
            self.last_state_rx_time = now_sec
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.q_curr[i] = msg.position[idx]
                    self.dq_curr[i] = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
                    self.tau_curr[i] = msg.effort[idx] if len(msg.effort) > idx else 0.0

            # Forward Kinematics via Pinocchio
            if hasattr(self, 'model'):
                q_arr = np.array(self.q_curr)
                pin.forwardKinematics(self.model, self.data, q_arr)
                pin.updateFramePlacements(self.model, self.data)
                se3 = self.data.oMf[self.eef_frame_id]
                rpy_vec = pin.rpy.matrixToRpy(se3.rotation)
                
                self.pose_curr = [
                    se3.translation[0], se3.translation[1], se3.translation[2],
                    rpy_vec[0], rpy_vec[1], rpy_vec[2]
                ]

            # Pack telemetry for delivery to GUI thread
            telemetry_packet = {
                'q': list(self.q_curr),
                'dq': list(self.dq_curr),
                'tau': list(self.tau_curr),
                'pose': list(self.pose_curr),
                'timestamp': self.last_state_rx_time
            }
            self.telemetry_signal.emit(telemetry_packet)

        # Dynamic parameter data recording
        if self.state == Lite6StateMachine.SYSID_RUNNING:
            if self.sysid_start_ros_time is None:
                self.sysid_start_ros_time = current_time
                
            t = current_time - self.sysid_start_ros_time
            if 1.0 < t < (self.sysid_calc.T_total - 1.0):
                with self.state_lock:
                    self.sysid_record_data.append({
                        't': t,
                        'q': np.array(self.q_curr),
                        'dq': np.array(self.dq_curr),
                        'tau_cmd': np.array(self.tau_curr)
                    })
            
            if t >= self.sysid_calc.T_total:
                self.set_state(Lite6StateMachine.SYSID_CALC)
                self.run_sysid_calculation()

    def send_moveit_goal(self, req):
        with self.state_lock:
            if self.state not in [Lite6StateMachine.IDLE, Lite6StateMachine.SOFT_ERROR]:
                self.node.get_logger().warn("HMI Action rejected: State Machine is busy.")
                return
            self.set_state(Lite6StateMachine.PLANNING)
            
        goal = MoveGroup.Goal()
        goal.request = req
        self.move_client.send_goal_async(goal).add_done_callback(self.goal_cb)

    def execute_joint_target(self, target_q, v_scale, a_scale, callback=None):
        req = MotionPlanRequest()
        req.group_name = 'lite6_arm'
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = 'PTP'
        req.num_planning_attempts = 1
        req.allowed_planning_time = 1.0
        req.max_velocity_scaling_factor = v_scale
        req.max_acceleration_scaling_factor = a_scale

        c = Constraints()
        for i in range(6):
            jc = JointConstraint()
            jc.joint_name = self.joint_names[i]
            jc.position = float(target_q[i])
            jc.tolerance_above = 1e-4
            jc.tolerance_below = 1e-4
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        
        if self.state in [Lite6StateMachine.SYSID_PRE_ALIGN, Lite6StateMachine.SYSID_POST_ALIGN, Lite6StateMachine.SYSID_RESTORE]:
            goal = MoveGroup.Goal()
            goal.request = req
            self.move_client.send_goal_async(goal).add_done_callback(
                lambda future: self.sysid_goal_cb(future, callback)
            )
        else:
            self.send_moveit_goal(req)

    def execute_pose_target(self, target_pose, v_scale, a_scale, planner_id):
        req = MotionPlanRequest()
        req.group_name = 'lite6_arm'
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = planner_id
        req.num_planning_attempts = 1
        req.allowed_planning_time = 1.0
        req.max_velocity_scaling_factor = v_scale
        req.max_acceleration_scaling_factor = a_scale
        req.goal_constraints.append(self._build_pose_constraint(target_pose))
        self.send_moveit_goal(req)

    def execute_circular_target(self, target_pose, aux_pose, v_scale, a_scale):
        req = MotionPlanRequest()
        req.group_name = 'lite6_arm'
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = 'CIRC'
        req.num_planning_attempts = 1
        req.allowed_planning_time = 1.0
        req.max_velocity_scaling_factor = v_scale
        req.max_acceleration_scaling_factor = a_scale

        req.goal_constraints.append(self._build_pose_constraint(target_pose))

        pc_aux = PositionConstraint()
        pc_aux.link_name = "link_eef"
        pc_aux.header.frame_id = "link_base"
        s_aux = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[10.0])
        b_aux = BoundingVolume()
        b_aux.primitives.append(s_aux)
        p_aux = Pose()
        p_aux.position.x = float(aux_pose[0])
        p_aux.position.y = float(aux_pose[1])
        p_aux.position.z = float(aux_pose[2])
        b_aux.primitive_poses.append(p_aux)
        pc_aux.constraint_region = b_aux
        pc_aux.weight = 1.0

        req.path_constraints.name = "interim"
        req.path_constraints.position_constraints.append(pc_aux)

        self.send_moveit_goal(req)

    def _build_pose_constraint(self, target_pose):
        c = Constraints()
        pc = PositionConstraint()
        pc.link_name = "link_eef"
        pc.header.frame_id = "link_base"
        
        s = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4])
        b = BoundingVolume()
        b.primitives.append(s)
        p = Pose()
        p.position.x = float(target_pose[0])
        p.position.y = float(target_pose[1])
        p.position.z = float(target_pose[2])
        b.primitive_poses.append(p)
        pc.constraint_region = b
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.link_name = "link_eef"
        oc.header.frame_id = "link_base"
        
        rpy_vec = np.array([float(target_pose[3]), float(target_pose[4]), float(target_pose[5])])
        R_mat = pin.rpy.rpyToMatrix(rpy_vec)
        q = pin.Quaternion(R_mat)
        
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = q.x, q.y, q.z, q.w
        oc.absolute_x_axis_tolerance = 1e-4
        oc.absolute_y_axis_tolerance = 1e-4
        oc.absolute_z_axis_tolerance = 1e-4
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def goal_cb(self, future):
        gh = future.result()
        with self.state_lock:
            if not gh.accepted:
                self.set_state(Lite6StateMachine.SOFT_ERROR, "Plan Rejected by MoveIt!")
                return
            self.set_state(Lite6StateMachine.EXECUTING)
            self.current_goal_handle = gh 
        gh.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.current_goal_handle = None 
        res = future.result().result
        with self.state_lock:
            if res.error_code.val == 1: 
                if self.state in [Lite6StateMachine.EXECUTING, Lite6StateMachine.RECOVERING]:
                    self.set_state(Lite6StateMachine.IDLE)
            elif res.error_code.val == -7:  # PREEMPTED
                pass 
            else:
                if self.state != Lite6StateMachine.ERROR: 
                    self.set_state(Lite6StateMachine.SOFT_ERROR, f"MoveIt Fault: {self.get_moveit_error_str(res.error_code.val)}")

    def sysid_goal_cb(self, future, next_step_callback):
        gh = future.result()
        with self.state_lock:
            if not gh.accepted:
                self.set_state(Lite6StateMachine.SOFT_ERROR, "SysID Alignment Planning Rejected")
                return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(
            lambda fut: self.sysid_result_cb(fut, next_step_callback)
        )

    def sysid_result_cb(self, future, next_step_callback):
        self.current_goal_handle = None
        res = future.result().result
        with self.state_lock:
            if res.error_code.val == 1:
                if next_step_callback is not None:
                    next_step_callback()
            else:
                self.set_state(Lite6StateMachine.SOFT_ERROR, f"SysID alignment movement failed: {res.error_code.val}")

    def publish_ground_plane(self):
        co = CollisionObject()
        co.header.frame_id = "link_base"
        co.id = "ground_plane"
        co.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0, 3.0, 0.1])
        pose = Pose()
        pose.position.z = -0.05
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        self.co_pub.publish(co)

    def trigger_estop(self, msg):
        """Active Hardware E-Stop sequence."""
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return 
            self.set_state(Lite6StateMachine.ERROR, msg)

            hw_cmd = Int32(data=1) 
            self.cmd_pub.publish(hw_cmd)
            
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None

            stop_traj = JointTrajectory()
            stop_traj.joint_names = self.joint_names
            
            pt = JointTrajectoryPoint()
            pt.positions = list(self.q_curr)
            pt.velocities = [0.0] * 6
            pt.accelerations = [0.0] * 6
            
            pt.time_from_start = Duration(sec=0, nanosec=10_000_000) 
            
            stop_traj.points.append(pt)
            self.jtc_pub.publish(stop_traj)

        self.node.get_logger().error(f"E-STOP ENGAGED: {msg}")

    # === SysID Sequencer Methods ===
    def start_sysid_sequence(self, prep_pose, max_limits):
        with self.state_lock:
            if self.state != Lite6StateMachine.IDLE:
                return
            self.state = Lite6StateMachine.SYSID_PRE_ALIGN
            self.sysid_record_data.clear()
            self.sysid_start_ros_time = None
            self.sysid_q0_locked = list(prep_pose)
            self.state_signal.emit(self.state, "")

        self.inject_sysid_parameters(max_limits)
        self.node.get_logger().info("SysID Step 1/5: Aligning to center...")
        self.execute_joint_target(prep_pose, 1.00, 1.00, callback=lambda: self.sysid_step2_configure_hw(prep_pose))

    def inject_sysid_parameters(self, max_limits):
        self.sysid_calc.record_data.clear()
        np.random.seed(42)
        N = self.sysid_calc.N_f
        for i in range(6):
            self.sysid_calc.a[i, 0:N-1] = np.random.uniform(-1.0, 1.0, N-1)
            self.sysid_calc.b[i, 0:N-2] = np.random.uniform(-1.0, 1.0, N-2)
            self.sysid_calc.a[i, N-1] = -np.sum(self.sysid_calc.a[i, 0:N-1])
            
            C1 = -np.sum([(l+1) * self.sysid_calc.b[i, l] for l in range(N-2)])
            C2 = -np.sum([self.sysid_calc.b[i, l] / (l+1) for l in range(N-2)])
            D = (1.0 - 2.0*N) / (N * (N - 1.0))
            self.sysid_calc.b[i, N-2] = (C1 / N - N * C2) / D
            self.sysid_calc.b[i, N-1] = (-C1 / (N - 1.0) + (N - 1.0) * C2) / D

            t_samples = np.linspace(0, self.sysid_calc.T_total, 200)
            max_offset = 0.0
            for t in t_samples:
                q_offset, _, _ = self.sysid_calc.get_fourier_point(i, t, np.zeros(6))
                max_offset = max(max_offset, abs(q_offset))
            
            if max_offset > 0:
                scale_factor = max_limits[i] / max_offset
                self.sysid_calc.a[i, :] *= scale_factor
                self.sysid_calc.b[i, :] *= scale_factor

    def sysid_step2_configure_hw(self, prep_pose):
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return
            self.set_state(Lite6StateMachine.SYSID_SEND_CMD)
        self.node.get_logger().info("SysID Step 2/5: Activating Dynamic excitation in hardware controller...")
        
        self.cmd_pub.publish(Int32(data=2))
        # Delayed call to step 3 using single-shot timers
        self.sysid_timer = self.node.create_timer(1.0, lambda: self.sysid_step3_publish_trajectory(prep_pose))

    def sysid_step3_publish_trajectory(self, prep_pose):
        self.sysid_timer.cancel()
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return
            self.set_state(Lite6StateMachine.SYSID_RUNNING)
        self.node.get_logger().info("SysID Step 3/5: Exciting trajectory...")
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        dt = 0.02
        steps = int(self.sysid_calc.T_total / dt) + 1
        
        for step in range(steps):
            t = step * dt
            pt = JointTrajectoryPoint()
            pt.positions = [0.0]*6
            pt.velocities = [0.0]*6
            pt.accelerations = [0.0]*6
            for i in range(6):
                q, dq, ddq = self.sysid_calc.get_fourier_point(i, t, prep_pose)
                pt.positions[i] = float(q)
                pt.velocities[i] = float(dq)
                pt.accelerations[i] = float(ddq)
            
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(pt)
        
        self.jtc_pub.publish(msg)

    def run_sysid_calculation(self):
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return
        self.node.get_logger().info("SysID Step 4/5: Running mathematical Least-Squares extraction...")
        
        try:
            self.sysid_calc.record_data = self.sysid_record_data
            yaml_results = self.sysid_calc.calculate_least_squares()
        except Exception as e:
            self.node.get_logger().error(f"SysID calculation failed: {e}")
            yaml_results = f"# Calculation Failed: {e}"
        
        self.sysid_yaml_signal.emit(yaml_results)
        
        with self.state_lock:
            self.set_state(Lite6StateMachine.SYSID_POST_ALIGN)
            current_physical_q = list(self.q_curr)
            
        self.node.get_logger().info("Aligning JTC command trajectory targeting current physical position...")
        self.execute_joint_target(current_physical_q, 0.15, 0.15, callback=self.sysid_step5_restore_normal)

    def sysid_step5_restore_normal(self):
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return
            self.set_state(Lite6StateMachine.SYSID_RESTORE)
        self.node.get_logger().info("Step 5/5: Restoring normal closed-loop operation...")
        
        self.cmd_pub.publish(Int32(data=0))
        self.restore_timer = self.node.create_timer(0.5, self.sysid_step5_return_to_prep)

    def sysid_step5_return_to_prep(self):
        self.restore_timer.cancel()
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return
        self.node.get_logger().info("Returning home safely...")
        self.execute_joint_target(self.sysid_q0_locked, 0.15, 0.15, callback=self.sysid_finalize)

    def sysid_finalize(self):
        with self.state_lock:
            if self.state == Lite6StateMachine.ERROR: return
            self.set_state(Lite6StateMachine.IDLE)
        self.node.get_logger().info("Automatic System Identification Complete!")

    def get_moveit_error_str(self, code):
        mapping = {
        # Overall behavior
        1: "Success", 0: "Undefined", 99999: "Failure",

        -1: "Planning Failed", -2: "Invalid Motion Plan", -3: "Motion Plan Invalidated By Environment Change",
        -4: "Control Failed", -5: "Unable To Acquire Sensor Data", -6: "Timed Out", -7: "Preempted",

        # Planning & kinematics request errors
        -10: "Start State In Collision", -11: "Start State Violates Path Constraints", -12: "Goal In Collision",
        -13: "Goal Violates Path Constraints", -14: "Goal Constraints Violated", -15: "Invalid Group Name",
        -16: "Invalid Goal Constraints", -17: "Invalid Robot State", -18: "Invalid Link Name", -19: "Invalid Object Name",
        -26: "Start State Invalid", -27: "Goal State Invalid", -28: "Unrecognized Goal Type",

        # System errors
        -21: "Frame Transform Failure", -22: "Collision Checking Unavailable", -23: "Robot State Stale",
        -24: "Sensor Info Stale", -25: "Communication Failure", -29: "Crash", -30: "Abort",

        # Kinematics errors
        -31: "No IK Solution",
        }
        return mapping.get(code, f"ErrorCode ({code})")


class PyQtHMI(QMainWindow):
    """
    Main Thread PyQt5 HMI Window.
    Processes telemetry and state transitions strictly via Qt Slots.
    """
    def __init__(self, worker):
        super().__init__()
        self.worker = worker
        self.setWindowTitle("UFactory Lite6 - Industrial HMI Interface")
        self.resize(800, 700)

        # Style definition via QSS (Modern Slate Industrial Theme)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f4f6f9;
            }
            QGroupBox {
                font-size: 11pt;
                font-weight: bold;
                border: 2px solid #bdc3c7;
                border-radius: 6px;
                margin-top: 12px;
                background-color: #ffffff;
                padding-top: 18px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
                color: #2c3e50;
            }
            QLabel {
                font-family: "Segoe UI", "Ubuntu";
                font-size: 10pt;
                color: #34495e;
            }
            QLineEdit {
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 5px;
                font-family: "Consolas", "Ubuntu Mono";
                font-size: 10pt;
                background-color: #ffffff;
            }
            QLineEdit:focus {
                border: 1px solid #3498db;
            }
            QLineEdit:disabled {
                background-color: #ecf0f1;
                color: #7f8c8d;
            }
            QSlider::groove:horizontal {
                border: 1px solid #bbb;
                background: #e0e0e0;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::sub-page:horizontal {
                background: #3498db;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #2c3e50;
                border: 1px solid #1a252f;
                width: 20px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 10px;
            }
            QTabWidget::pane {
                border: 1px solid #bdc3c7;
                background-color: #f4f6f9;
            }
            QTabBar::tab {
                background: #e2e6ea;
                border: 1px solid #bdc3c7;
                padding: 8px 12px;
                font-family: "Segoe UI", "Ubuntu";
                font-weight: bold;
                font-size: 7pt;
                color: #2c3e50;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                border-bottom-color: #ffffff;
            }
            QPushButton {
                background-color: #34495e;
                color: #ffffff;
                font-family: "Segoe UI", "Ubuntu";
                font-size: 10pt;
                font-weight: bold;
                border: none;
                border-radius: 5px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #415b76;
            }
            QPushButton:pressed {
                background-color: #2c3e50;
            }
        """)

        # Shared limits
        self.j_limits = [
            [-360.0, 360.0], [-150.0, 150.0], [0.0, 285.0], 
            [-360.0, 360.0], [-93.0, 93.0], [-360.0, 360.0]
        ]
        self.exact_q_target = [0.0] * 6
        self.exact_p_target = [0.0] * 6
        self.exact_aux_target = [0.0] * 6

        # Shared double validator
        self.float_validator = QDoubleValidator(-10000.0, 10000.0, 4, self)
        self.float_validator.setNotation(QDoubleValidator.StandardNotation)

        # Build HMI Layout
        self.setup_ui()

        # Connect ROS worker signals to GUI Main Thread slots
        self.worker.telemetry_signal.connect(self.on_telemetry_updated)
        self.worker.state_signal.connect(self.on_state_updated)
        self.worker.sysid_yaml_signal.connect(self.on_sysid_yaml_updated)

        # Connection timeout watchdog timer
        self.watchdog_timer = QTimer()
        self.watchdog_timer.timeout.connect(self.check_connection_watchdog)
        self.watchdog_timer.start(500)

    def setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        master_layout = QVBoxLayout(main_widget)
        master_layout.setContentsMargins(15, 15, 15, 15)
        master_layout.setSpacing(10)

        # 1. State Machine Banner Frame
        self.state_banner = QLabel("INITIALIZING SYSTEM...")
        self.state_banner.setAlignment(Qt.AlignCenter)
        self.state_banner.setFont(QFont("Segoe UI", 16, QFont.Bold))
        self.state_banner.setFixedHeight(60)
        self.set_banner_color("#2c3e50") # Deep charcoal default
        master_layout.addWidget(self.state_banner)

        # 2. Main Middle Workspace split: Left Telemetry, Right Controls
        workspace_layout = QHBoxLayout()
        master_layout.addLayout(workspace_layout)

        # Col 1: Read-Only Realtime Telemetry
        telemetry_container = QWidget()
        telemetry_container.setFixedWidth(260)
        col1_layout = QVBoxLayout(telemetry_container)
        col1_layout.setContentsMargins(0, 0, 0, 0)
        
        tele_group = QGroupBox("Realtime Telemetry")
        col1_layout.addWidget(tele_group)
        tele_layout = QVBoxLayout(tele_group)
        tele_layout.setSpacing(10)

        # Joint positions
        tele_layout.addWidget(QLabel("Joint Angles (Physical Position)", font=QFont("Segoe UI", 10, QFont.Bold)))
        self.lbl_q_curr = []
        for i in range(6):
            lbl = QLabel(f" J{i+1}:   0.00 °")
            lbl.setFont(QFont("Consolas", 10))
            tele_layout.addWidget(lbl)
            self.lbl_q_curr.append(lbl)

        # Horizontal Divider Line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        tele_layout.addWidget(line)

        # TCP Pose
        tele_layout.addWidget(QLabel("TCP Frame Pose (Base Link)", font=QFont("Segoe UI", 10, QFont.Bold)))
        self.lbl_p_curr = []
        labels = ['X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (°)', 'Pitch (°)', 'Yaw (°)']
        for name in labels:
            lbl = QLabel(f" {name:9s}:   0.00")
            lbl.setFont(QFont("Consolas", 10))
            tele_layout.addWidget(lbl)
            self.lbl_p_curr.append(lbl)
        
        tele_layout.addStretch()
        workspace_layout.addWidget(telemetry_container)

        # Col 2: Action tabs
        self.tabs = QTabWidget()
        workspace_layout.addWidget(self.tabs)

        # Tab A: Manual Control
        tab_manual = QWidget()
        self.tabs.addTab(tab_manual, "Manual Control")
        self.setup_manual_tab(tab_manual)

        # Tab B: System ID
        tab_sysid = QWidget()
        self.tabs.addTab(tab_sysid, "System Identification")
        self.setup_sysid_tab(tab_sysid)

        # 3. Bottom Control Frame: Overrides, Home, Fault Recovery and E-Stop
        bot_frame = QWidget()
        bot_frame.setStyleSheet("background-color: #dee2e6; border-radius: 4px;")
        bot_frame.setFixedHeight(95)
        master_layout.addWidget(bot_frame)
        
        bot_layout = QHBoxLayout(bot_frame)
        bot_layout.setContentsMargins(15, 10, 15, 10)

        # Override adjustments
        overrides_container = QWidget()
        over_layout = QGridLayout(overrides_container)
        over_layout.setContentsMargins(0, 0, 0, 0)
        over_layout.setSpacing(5)

        # Velocity Override
        lbl_v = QLabel("Velocity Scale:")
        lbl_v.setStyleSheet("font-weight: bold; background-color: transparent;")
        over_layout.addWidget(lbl_v, 0, 0)
        
        self.v_slider = QSlider(Qt.Horizontal)
        self.v_slider.setRange(1, 100)
        self.v_slider.setValue(100)
        self.v_slider.setFixedWidth(150)
        self.v_slider.setStyleSheet("background-color: transparent;")
        over_layout.addWidget(self.v_slider, 0, 1)

        self.v_entry = QLineEdit("1.00")
        self.v_entry.setFixedWidth(55)
        self.v_entry.setAlignment(Qt.AlignCenter)
        self.v_entry.setValidator(self.float_validator)
        over_layout.addWidget(self.v_entry, 0, 2)

        # Acceleration Override
        lbl_a = QLabel("Acceleration Scale:")
        lbl_a.setStyleSheet("font-weight: bold; background-color: transparent;")
        over_layout.addWidget(lbl_a, 1, 0)

        self.a_slider = QSlider(Qt.Horizontal)
        self.a_slider.setRange(1, 100)
        self.a_slider.setValue(100)
        self.a_slider.setFixedWidth(150)
        self.a_slider.setStyleSheet("background-color: transparent;")
        over_layout.addWidget(self.a_slider, 1, 1)

        self.a_entry = QLineEdit("1.00")
        self.a_entry.setFixedWidth(55)
        self.a_entry.setAlignment(Qt.AlignCenter)
        self.a_entry.setValidator(self.float_validator)
        over_layout.addWidget(self.a_entry, 1, 2)

        bot_layout.addWidget(overrides_container)
        bot_layout.addStretch()

        # Connect Slider and LineEdit for scaling metrics
        self.v_slider.valueChanged.connect(self.sync_v_slider_to_entry)
        self.v_entry.editingFinished.connect(self.sync_v_entry_to_slider)
        self.a_slider.valueChanged.connect(self.sync_a_slider_to_entry)
        self.a_entry.editingFinished.connect(self.sync_a_entry_to_slider)

        # Utility Buttons
        self.btn_home = QPushButton("⌂ HOME")
        self.btn_home.setFixedSize(110, 45)
        self.btn_home.setStyleSheet("background-color: #2c3e50; font-size: 13px; color: white;")
        self.btn_home.clicked.connect(self.cmd_home)
        bot_layout.addWidget(self.btn_home)

        self.btn_reset_fault = QPushButton("⚠ RESET FAULT")
        self.btn_reset_fault.setFixedSize(130, 45)
        self.btn_reset_fault.setStyleSheet("background-color: #f39c12; font-size: 13px; color: white;")
        self.btn_reset_fault.clicked.connect(self.gui_reset_fault)
        bot_layout.addWidget(self.btn_reset_fault)

        self.btn_estop = QPushButton("STOP (E-STOP)")
        self.btn_estop.setFixedSize(180, 45)
        self.btn_estop.setStyleSheet("background-color: #d9534f; font-size: 15px; font-weight: bold; color: white;")
        self.btn_estop.clicked.connect(lambda: self.worker.trigger_estop("SOFTWARE E-STOP PRESSED"))
        bot_layout.addWidget(self.btn_estop)

    def setup_manual_tab(self, parent):
        tab_layout = QHBoxLayout(parent)
        tab_layout.setContentsMargins(10, 10, 10, 10)
        tab_layout.setSpacing(10)

        # Column 1: Joint Space Control
        j_group = QGroupBox("Joint Space Commands (PTP)")
        tab_layout.addWidget(j_group)
        j_layout = QVBoxLayout(j_group)
        j_layout.setContentsMargins(15, 15, 15, 15)

        self.btn_sync_j = QPushButton("⟲ Sync target with raw state")
        self.btn_sync_j.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; border: 1px solid #bdc3c7;")
        self.btn_sync_j.clicked.connect(self.sync_joints)
        j_layout.addWidget(self.btn_sync_j)

        self.j_sliders = []
        self.j_entries = []

        # Create 6 Joint controls
        for i in range(6):
            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            
            lbl_name = QLabel(f"J{i+1}")
            lbl_name.setFont(QFont("Segoe UI", 9, QFont.Bold))
            lbl_name.setFixedWidth(20)
            row_layout.addWidget(lbl_name)

            lim = self.j_limits[i]
            lbl_low = QLabel(f"{lim[0]:.0f}°")
            lbl_low.setStyleSheet("color: gray;")
            lbl_low.setFixedWidth(35)
            lbl_low.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row_layout.addWidget(lbl_low)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(lim[0] * 100), int(lim[1] * 100))
            slider.setValue(0)
            row_layout.addWidget(slider)
            self.j_sliders.append(slider)

            entry = QLineEdit("0.00")
            entry.setFixedWidth(65)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            row_layout.addWidget(entry)
            self.j_entries.append(entry)

            lbl_high = QLabel(f"{lim[1]:.0f}°")
            lbl_high.setStyleSheet("color: gray;")
            lbl_high.setFixedWidth(35)
            row_layout.addWidget(lbl_high)

            j_layout.addWidget(row_widget)

            # Connect slider and entry dynamically
            self.connect_joint_pair(slider, entry, i)

        self.btn_move_j = QPushButton("Execute Joint Trajectory")
        self.btn_move_j.setStyleSheet("background-color: #2980b9; height: 35px; font-size: 13px;")
        self.btn_move_j.clicked.connect(self.cmd_move_j)
        j_layout.addWidget(self.btn_move_j)

        # Column 2: Cartesian Space Control
        c_group = QGroupBox("Cartesian Space Commands")
        tab_layout.addWidget(c_group)
        c_layout = QVBoxLayout(c_group)
        c_layout.setContentsMargins(15, 15, 15, 15)

        # Pose Coordinates Grid
        pose_widget = QWidget()
        pose_grid = QGridLayout(pose_widget)
        pose_grid.setContentsMargins(0, 0, 0, 0)
        pose_grid.setSpacing(8)

        lbl_target_title = QLabel("Target TCP Coordinate")
        lbl_target_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        pose_grid.addWidget(lbl_target_title, 0, 0, 1, 2)

        self.btn_sync_p = QPushButton("⟲ Sync")
        self.btn_sync_p.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; border: 1px solid #bdc3c7;")
        self.btn_sync_p.setFixedWidth(70)
        self.btn_sync_p.clicked.connect(self.sync_pose)
        pose_grid.addWidget(self.btn_sync_p, 0, 3, 1, 1, Qt.AlignRight)

        self.p_entries = []
        labels = ['X', 'Roll', 'Y', 'Pitch', 'Z', 'Yaw']
        idx_map = [0, 3, 1, 4, 2, 5]
        
        for i, idx in enumerate(idx_map):
            lbl_ax = QLabel(labels[i])
            lbl_ax.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pose_grid.addWidget(lbl_ax, (i // 2) + 1, (i % 2) * 2)
            
            entry = QLineEdit("0.00")
            entry.setFixedWidth(80)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            pose_grid.addWidget(entry, (i // 2) + 1, (i % 2) * 2 + 1)
            self.p_entries.append(entry)

        # Position placeholders
        sorted_entries = [None] * 6
        for i, idx in enumerate(idx_map):
            sorted_entries[idx] = self.p_entries[i]
        self.p_entries = sorted_entries

        c_layout.addWidget(pose_widget)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        c_layout.addWidget(line)

        # Circular Auxiliary Frame Grid
        aux_widget = QWidget()
        aux_grid = QGridLayout(aux_widget)
        aux_grid.setContentsMargins(0, 0, 0, 0)
        aux_grid.setSpacing(8)

        lbl_aux_title = QLabel("Circular Aux Frame")
        lbl_aux_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        aux_grid.addWidget(lbl_aux_title, 0, 0, 1, 2)

        self.btn_sync_aux = QPushButton("⟲ Sync")
        self.btn_sync_aux.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; border: 1px solid #bdc3c7;")
        self.btn_sync_aux.setFixedWidth(70)
        self.btn_sync_aux.clicked.connect(self.sync_aux)
        aux_grid.addWidget(self.btn_sync_aux, 0, 3, 1, 1, Qt.AlignRight)

        self.aux_p_entries = []
        for i, idx in enumerate(idx_map):
            lbl_ax = QLabel(labels[i])
            lbl_ax.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            aux_grid.addWidget(lbl_ax, (i // 2) + 1, (i % 2) * 2)
            
            entry = QLineEdit("0.00")
            entry.setFixedWidth(80)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            entry.setStyleSheet("background-color: #e8f4f8;")
            aux_grid.addWidget(entry, (i // 2) + 1, (i % 2) * 2 + 1)
            self.aux_p_entries.append(entry)

        sorted_aux_entries = [None] * 6
        for i, idx in enumerate(idx_map):
            sorted_aux_entries[idx] = self.aux_p_entries[i]
        self.aux_p_entries = sorted_aux_entries

        c_layout.addWidget(aux_widget)
        c_layout.addStretch()

        # Action Buttons
        self.btn_move_p = QPushButton("Execute Cartesian PTP (MoveP)")
        self.btn_move_p.setStyleSheet("background-color: #5bc0de; height: 30px;")
        self.btn_move_p.clicked.connect(self.cmd_move_p)
        c_layout.addWidget(self.btn_move_p)

        self.btn_move_l = QPushButton("Execute Linear Interpolation (MoveL)")
        self.btn_move_l.setStyleSheet("background-color: #27ae60; height: 30px;")
        self.btn_move_l.clicked.connect(self.cmd_move_l)
        c_layout.addWidget(self.btn_move_l)

        self.btn_move_c = QPushButton("Execute Circular Interpolation (MoveC)")
        self.btn_move_c.setStyleSheet("background-color: #8e44ad; height: 30px;")
        self.btn_move_c.clicked.connect(self.cmd_move_c)
        c_layout.addWidget(self.btn_move_c)

    def setup_sysid_tab(self, parent):
        tab_layout = QVBoxLayout(parent)
        tab_layout.setContentsMargins(10, 10, 10, 10)
        tab_layout.setSpacing(10)

        ctl_group = QGroupBox("Trajectory Alignment and Excitation Bounds")
        tab_layout.addWidget(ctl_group)
        ctl_layout = QVBoxLayout(ctl_group)
        ctl_layout.setSpacing(15)

        # Prep pose sub-group
        prep_widget = QWidget()
        prep_layout = QVBoxLayout(prep_widget)
        prep_layout.setContentsMargins(0, 0, 0, 0)
        prep_layout.addWidget(QLabel("Identification Workspace Center (degrees)", font=QFont("Segoe UI", 11, QFont.Bold)))

        prep_inputs_container = QWidget()
        prep_inputs_layout = QHBoxLayout(prep_inputs_container)
        prep_inputs_layout.setContentsMargins(0, 0, 0, 0)
        prep_inputs_layout.setSpacing(15) # Distance between J1 group and J2 group

        self.sysid_prep_entries = []
        default_prep = [0.0, -30.0, 60.0, 180.0, 0.0, 0.0]
        for i in range(6):
            # Create a tight sub-layout for each Joint label + box
            pair_widget = QWidget()
            pair_layout = QHBoxLayout(pair_widget)
            pair_layout.setContentsMargins(0, 0, 0, 0)
            pair_layout.setSpacing(5)
            
            lbl = QLabel(f"J{i+1}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pair_layout.addWidget(lbl)
            
            entry = QLineEdit(str(default_prep[i]))
            entry.setFixedWidth(65)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            pair_layout.addWidget(entry)
            
            prep_inputs_layout.addWidget(pair_widget)
            self.sysid_prep_entries.append(entry)
            
        prep_inputs_layout.addStretch()
        prep_layout.addWidget(prep_inputs_container)
        ctl_layout.addWidget(prep_widget)

        # Excitation limits sub-group
        limits_widget = QWidget()
        limits_layout = QVBoxLayout(limits_widget)
        limits_layout.setContentsMargins(0, 0, 0, 0)
        limits_layout.addWidget(QLabel("Fourier Excitation Limits (radians from pose center)", font=QFont("Segoe UI", 11, QFont.Bold)))

        lim_inputs_container = QWidget()
        lim_inputs_layout = QHBoxLayout(lim_inputs_container)
        lim_inputs_layout.setContentsMargins(0, 0, 0, 0)
        lim_inputs_layout.setSpacing(15)

        self.sysid_limit_entries = []
        default_limits = [0.4, 0.6, 0.5, 1.0, 1.0, 3.0]
        for i in range(6):
            pair_widget = QWidget()
            pair_layout = QHBoxLayout(pair_widget)
            pair_layout.setContentsMargins(0, 0, 0, 0)
            pair_layout.setSpacing(5)
            
            lbl = QLabel(f"J{i+1}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pair_layout.addWidget(lbl)
            
            entry = QLineEdit(str(default_limits[i]))
            entry.setFixedWidth(65)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            entry.setStyleSheet("background-color: #fff9db;")
            pair_layout.addWidget(entry)
            
            lim_inputs_layout.addWidget(pair_widget)
            self.sysid_limit_entries.append(entry)
            
        lim_inputs_layout.addStretch() # Core Fix
        limits_layout.addWidget(lim_inputs_container)
        ctl_layout.addWidget(limits_widget)

        # Trigger Actions
        self.btn_sys_prep = QPushButton("Move to Alignment Pose")
        self.btn_sys_prep.setStyleSheet("background-color: #bdc3c7; color: #2c3e50; font-size: 11pt;")
        self.btn_sys_prep.clicked.connect(self.cmd_sysid_prep_align)
        ctl_layout.addWidget(self.btn_sys_prep)

        self.btn_run_sysid = QPushButton("Start Automated Identification Routine")
        self.btn_run_sysid.setStyleSheet("background-color: #e74c3c; height: 40px; font-size: 12pt;")
        self.btn_run_sysid.clicked.connect(self.cmd_run_sysid)
        ctl_layout.addWidget(self.btn_run_sysid)

        # Output terminal representation
        out_group = QGroupBox("Identified Matrix Parameters (YAML)")
        tab_layout.addWidget(out_group)
        out_layout = QVBoxLayout(out_group)
        out_layout.setContentsMargins(10, 15, 10, 10)

        from PyQt5.QtWidgets import QTextEdit
        self.txt_output = QTextEdit()
        self.txt_output.setReadOnly(True)
        self.txt_output.setStyleSheet("""
            background-color: #1e272e; 
            color: #2ecc71; 
            font-family: 'Consolas', 'Ubuntu Mono'; 
            font-size: 11pt;
            border: 1px solid #1a252f;
            border-radius: 4px;
            padding: 5px;
        """)
        out_layout.addWidget(self.txt_output)

        self.btn_copy = QPushButton("Copy parameters to clipboard")
        self.btn_copy.setStyleSheet("background-color: #27ae60; height: 35px;")
        self.btn_copy.clicked.connect(self.cmd_copy_yaml)
        out_layout.addWidget(self.btn_copy)

    # === Synchronous slider-entry pairing utility ===
    def connect_joint_pair(self, slider, entry, idx):
        """Map QSlider (integer based) and QLineEdit (float validation) bidirectionally."""
        def slider_moved(val):
            # Block signals to prevent cyclic loop updates
            entry.blockSignals(True)
            entry.setText(f"{val / 100.0:.2f}")
            entry.blockSignals(False)
            
        def entry_edited():
            try:
                val = float(entry.text())
                lim = self.j_limits[idx]
                val = max(lim[0], min(val, lim[1]))
                entry.blockSignals(True)
                entry.setText(f"{val:.2f}")
                entry.blockSignals(False)
                
                slider.blockSignals(True)
                slider.setValue(int(val * 100))
                slider.blockSignals(False)
            except ValueError:
                pass

        slider.valueChanged.connect(slider_moved)
        entry.editingFinished.connect(entry_edited)

    def sync_v_slider_to_entry(self, val):
        self.v_entry.blockSignals(True)
        self.v_entry.setText(f"{val / 100.0:.2f}")
        self.v_entry.blockSignals(False)

    def sync_v_entry_to_slider(self):
        try:
            val = float(self.v_entry.text())
            val = max(0.01, min(val, 1.0))
            self.v_entry.blockSignals(True)
            self.v_entry.setText(f"{val:.2f}")
            self.v_entry.blockSignals(False)
            self.v_slider.blockSignals(True)
            self.v_slider.setValue(int(val * 100))
            self.v_slider.blockSignals(False)
        except ValueError:
            pass

    def sync_a_slider_to_entry(self, val):
        self.a_entry.blockSignals(True)
        self.a_entry.setText(f"{val / 100.0:.2f}")
        self.a_entry.blockSignals(False)

    def sync_a_entry_to_slider(self):
        try:
            val = float(self.a_entry.text())
            val = max(0.01, min(val, 1.0))
            self.a_entry.blockSignals(True)
            self.a_entry.setText(f"{val:.2f}")
            self.a_entry.blockSignals(False)
            self.a_slider.blockSignals(True)
            self.a_slider.setValue(int(val * 100))
            self.a_slider.blockSignals(False)
        except ValueError:
            pass

    # === Banner color transitions ===
    def set_banner_color(self, hex_color):
        self.state_banner.setStyleSheet(f"""
            background-color: {hex_color};
            color: #ffffff;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
        """)

    # === Telemetry Sync and UI refresh slots ===
    def on_telemetry_updated(self, pkt):
        """Processes high-frequency ROS 2 telemetry packet in Qt GUI thread."""
        q_deg = [math.degrees(v) for v in pkt['q']]
        for i in range(6):
            self.lbl_q_curr[i].setText(f" J{i+1}: {q_deg[i]:7.2f} °")

        p = pkt['pose']
        self.lbl_p_curr[0].setText(f" X (mm): {p[0]*1000:7.2f}")
        self.lbl_p_curr[1].setText(f" Y (mm): {p[1]*1000:7.2f}")
        self.lbl_p_curr[2].setText(f" Z (mm): {p[2]*1000:7.2f}")
        self.lbl_p_curr[3].setText(f" Roll (°): {math.degrees(p[3]):7.2f}")
        self.lbl_p_curr[4].setText(f" Pitch(°): {math.degrees(p[4]):7.2f}")
        self.lbl_p_curr[5].setText(f" Yaw  (°): {math.degrees(p[5]):7.2f}")

    def on_state_updated(self, state, error_msg):
        """Processes state transition events safely."""
        if state == Lite6StateMachine.IDLE:
            self.set_banner_color("#27ae60") # Emerald
            self.state_banner.setText("SYSTEM READY")
        elif state == Lite6StateMachine.ERROR:
            self.set_banner_color("#c0392b") # Alizarin red
            self.state_banner.setText(f"SYSTEM FAULT: {error_msg}")
        elif state == Lite6StateMachine.SOFT_ERROR:
            self.set_banner_color("#d35400") # Pumpkin Orange
            self.state_banner.setText(f"WARNING: {error_msg}")
        elif state.startswith("SYSID"):
            self.set_banner_color("#8e44ad") # Amethyst purple
            self.state_banner.setText(state)
        else:
            self.set_banner_color("#2980b9") # Peter river blue
            self.state_banner.setText(state)

    def on_sysid_yaml_updated(self, yaml_string):
        self.txt_output.clear()
        self.txt_output.insertPlainText(yaml_string)

    def check_connection_watchdog(self):
        """Disconnect watchdog: check if heartbeat from subscriber has stalled."""
        with self.worker.state_lock:
            last_rx = self.worker.last_state_rx_time
            state = self.worker.state
        
        now = self.worker.node.get_clock().now().nanoseconds * 1e-9
        if state != Lite6StateMachine.INIT and (now - last_rx) > 0.5:
            if state != Lite6StateMachine.ERROR:
                self.worker.trigger_estop("HARDWARE DISCONNECTED! (Heartbeat Stalled)")

    # === Synchronizers ===
    def sync_joints(self):
        with self.worker.state_lock:
            q_copied = list(self.worker.q_curr)
            
        for i in range(6):
            self.exact_q_target[i] = q_copied[i]
            deg_val = math.degrees(q_copied[i])
            self.j_entries[i].setText(f"{deg_val:.2f}")
            self.j_sliders[i].setValue(int(deg_val * 100))

    def _sync_coordinates_to_entries(self, exact_storage, entries):
        with self.worker.state_lock:
            p = list(self.worker.pose_curr)
            
        for i in range(6): exact_storage[i] = p[i]
        entries[0].setText(f"{p[0]*1000:.2f}")
        entries[1].setText(f"{p[1]*1000:.2f}")
        entries[2].setText(f"{p[2]*1000:.2f}")
        entries[3].setText(f"{math.degrees(p[3]):.2f}")
        entries[4].setText(f"{math.degrees(p[4]):.2f}")
        entries[5].setText(f"{math.degrees(p[5]):.2f}")

    def sync_pose(self):
        self._sync_coordinates_to_entries(self.exact_p_target, self.p_entries)

    def sync_aux(self):
        self._sync_coordinates_to_entries(self.exact_aux_target, self.aux_p_entries)

    # === Command Dispatchers ===
    def cmd_move_j(self):
        try:
            target_rad = [0.0] * 6
            for i in range(6):
                ui_val = float(self.j_entries[i].text())
                ui_str_of_exact = float(f"{math.degrees(self.exact_q_target[i]):.2f}")
                
                if abs(ui_val - ui_str_of_exact) < 1e-6:
                    target_rad[i] = self.exact_q_target[i]
                else:
                    target_rad[i] = math.radians(ui_val)
                    self.exact_q_target[i] = target_rad[i]
            
            self.worker.execute_joint_target(target_rad, self.v_var_val(), self.a_var_val())
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Input format parse exception: {e}")

    def _parse_cartesian_entries(self, entries, exact_storage):
        target = [0.0] * 6
        
        for i in range(3):
            ui_val = float(entries[i].text())
            ui_str_of_exact = float(f"{exact_storage[i] * 1000.0:.2f}")
            
            if abs(ui_val - ui_str_of_exact) < 1e-6:
                target[i] = exact_storage[i]
            else:
                target[i] = ui_val / 1000.0
                exact_storage[i] = target[i]
                
        for i in range(3, 6):
            ui_val = float(entries[i].text())
            ui_str_of_exact = float(f"{math.degrees(exact_storage[i]):.2f}")
            
            if abs(ui_val - ui_str_of_exact) < 1e-6:
                target[i] = exact_storage[i]
            else:
                target[i] = math.radians(ui_val)
                exact_storage[i] = target[i]
                
        return target

    def cmd_move_p(self):
        try:
            target = self._parse_cartesian_entries(self.p_entries, self.exact_p_target)
            self.worker.execute_pose_target(target, self.v_var_val(), self.a_var_val(), 'PTP')
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Invalid input parsed: {e}")

    def cmd_move_l(self):
        try:
            target = self._parse_cartesian_entries(self.p_entries, self.exact_p_target)
            self.worker.execute_pose_target(target, self.v_var_val(), self.a_var_val(), 'LIN')
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Invalid input parsed: {e}")

    def cmd_move_c(self):
        try:
            target = self._parse_cartesian_entries(self.p_entries, self.exact_p_target)
            aux_target = self._parse_cartesian_entries(self.aux_p_entries, self.exact_aux_target)
            self.worker.execute_circular_target(target, aux_target, self.v_var_val(), self.a_var_val())
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Circular geometry definition incorrect: {e}")

    def cmd_home(self):
        home_deg = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
        for i in range(6):
            self.j_entries[i].setText(f"{home_deg[i]:.2f}")
            self.j_sliders[i].setValue(int(home_deg[i] * 100))
        self.cmd_move_j()

    # === Automated SysID Sequencer Calls ===
    def _parse_sysid_inputs(self):
        prep_pose = [math.radians(float(entry.text())) for entry in self.sysid_prep_entries]
        max_limits = [float(entry.text()) for entry in self.sysid_limit_entries]
        return prep_pose, max_limits

    def cmd_sysid_prep_align(self):
        with self.worker.state_lock:
            state = self.worker.state
        if state != Lite6StateMachine.IDLE:
            QMessageBox.warning(self, "Command Blocked", "System must be IDLE.")
            return
        try:
            prep_pose, _ = self._parse_sysid_inputs()
            self.worker.execute_joint_target(prep_pose, 1.00, 1.00)
        except Exception as e:
            QMessageBox.critical(self, "Input parsing fault", f"Error mapping configuration: {e}")

    def cmd_run_sysid(self):
        with self.worker.state_lock:
            state = self.worker.state
            q_curr_copy = list(self.worker.q_curr)
            
        if state != Lite6StateMachine.IDLE:
            QMessageBox.warning(self, "Command Blocked", "Robot system must be IDLE.")
            return
        
        try:
            prep_pose, max_limits = self._parse_sysid_inputs()
        except Exception as e:
            QMessageBox.critical(self, "Input Parse Error", f"Failed to parse inputs: {e}")
            return

        # Check alignment offset to ensure workspace center synchronization
        current_offset = np.abs(np.array(q_curr_copy) - np.array(prep_pose)).max()
        if current_offset > 0.02:
            res = QMessageBox.question(
                self, "SysID Alignment",
                "Robot is not aligned with workspace center.\nPlan and align now?",
                QMessageBox.Yes | QMessageBox.No
            )
            if res == QMessageBox.Yes:
                self.worker.start_sysid_sequence(prep_pose, max_limits)
            return

        self.worker.start_sysid_sequence(prep_pose, max_limits)

    def cmd_copy_yaml(self):
        raw_text = self.txt_output.toPlainText().strip()
        if not raw_text:
            QMessageBox.warning(self, "Clipboard Warning", "Terminal box empty.")
            return
        clipboard = QApplication.clipboard()
        clipboard.setText(raw_text)
        QMessageBox.information(self, "Success", "Identified parameters copied to clipboard.")

    def gui_reset_fault(self):
        with self.worker.state_lock:
            state = self.worker.state
            q_curr_copy = list(self.worker.q_curr)

        if state == Lite6StateMachine.SOFT_ERROR:
            self.worker.set_state(Lite6StateMachine.IDLE)
            return

        if state != Lite6StateMachine.ERROR: 
            return
            
        self.worker.set_state(Lite6StateMachine.RECOVERING)
        
        # Build alignment request
        req = MotionPlanRequest()
        req.group_name = 'lite6_arm'
        req.planner_id = 'PTP'
        req.num_planning_attempts = 1
        req.allowed_planning_time = 1.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        
        c = Constraints()
        for i in range(6):
            jc = JointConstraint()
            jc.joint_name = self.worker.joint_names[i]
            jc.position = float(q_curr_copy[i])
            jc.tolerance_above = 1e-4
            jc.tolerance_below = 1e-4
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        
        goal = MoveGroup.Goal()
        goal.request = req
        self.worker.move_client.send_goal_async(goal).add_done_callback(self.gui_unlock_cb)

    def gui_unlock_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.worker.set_state(Lite6StateMachine.ERROR, "Recovery motion rejected by MoveIt.")
            return
        gh.get_result_async().add_done_callback(self.gui_finalize_recovery)

    def gui_finalize_recovery(self, future):
        res = future.result().result
        if res.error_code.val == 1:
            hw_cmd = Int32(data=0)
            self.worker.cmd_pub.publish(hw_cmd)
            self.sync_joints()
            self.sync_pose()
            self.worker.set_state(Lite6StateMachine.IDLE)
        else:
            self.worker.set_state(Lite6StateMachine.ERROR, f"Recovery motion failed: {res.error_code.val}")

    # === Helper scaling utilities ===
    def v_var_val(self):
        return float(self.v_entry.text())

    def a_var_val(self):
        return float(self.a_entry.text())

    def on_closing(self):
        self.close()


def main(args=None):
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1.0"
    os.environ["QT_SCALE_FACTOR"] = "1.0"    # This factor changes the total size of the GUI window. The fonts inside it will be changed automatically.
    rclpy.init(args=args)

    # 1. Instantiate the ROS 2 executor inside QThread Worker
    worker = ROS2Worker()
    worker.start()

    # Enable High-DPI Scaling for 2K/4K screens
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    try:
        QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    except AttributeError:
        pass # Ignored if using an older version of PyQt5

    # 2. Instantiate and launch the main Qt Application loop
    app = QApplication(sys.argv)
    
    # Establish dynamic color palette settings matching modern OS environments
    app.setStyle('Fusion')
    
    gui = PyQtHMI(worker)
    gui.show()

    exit_code = app.exec_()
    
    # Graceful shutdown of ROS Executor
    worker.node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
