import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

class ShadowTracker(Node):
    """
    A lightweight bridge that continuously mirrors the JTC reference trajectory
    to the shadow robot's joint states.
    """
    def __init__(self):
        super().__init__('shadow_tracker')
        
        self.sub_state = self.create_subscription(
            JointTrajectoryControllerState,
            '/lite6_arm_controller/controller_state',
            self.state_cb, 10)
            
        self.pub_joints = self.create_publisher(JointState, '/shadow/joint_states', 10)

    def state_cb(self, msg):
        # Extract the ideal reference positions
        target_positions = msg.reference.positions
        
        # Fallback to actual feedback if no reference is actively running
        if not target_positions:
            target_positions = msg.feedback.positions
            
        if not target_positions:
            return
            
        out_msg = JointState()
        out_msg.header = msg.header
        out_msg.name = msg.joint_names
        out_msg.position = target_positions
        
        self.pub_joints.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShadowTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()