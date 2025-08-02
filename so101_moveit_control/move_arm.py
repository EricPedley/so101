#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Create action client for your gripper controller
        self.gripper_client = ActionClient(
            self, 
            ExecuteTrajectory, 
            '/execute_trajectory'
        )
        
        self.get_logger().info("Waiting for gripper action server...")
        self.gripper_client.wait_for_server()
        self.get_logger().info("Gripper controller initialized")
    
    def move_gripper(self, position, duration_sec=2):
        """Move gripper to specified position"""
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory.joint_trajectory = JointTrajectory()
        goal_msg.trajectory.joint_trajectory.joint_names = ['gripper']
        goal_msg.trajectory.joint_trajectory.points.append(
            JointTrajectoryPoint(
                positions=[position],
                time_from_start=Duration(sec=duration_sec)
            )
        )
        self.gripper_client.send_goal(goal_msg)
        self.get_logger().info("sent goal")
        
    
    def open_gripper(self):
        return self.move_gripper(0.8)  # Adjust for your gripper
    
    def close_gripper(self):
        return self.move_gripper(0.0)  # Adjust for your gripper
    
    def flex_gripper(self, cycles=3):
        for i in range(cycles):
            self.get_logger().info(f"Flex cycle {i+1}/{cycles}")
            self.open_gripper()
            rclpy.spin_once(self, timeout_sec=3.0)
            self.close_gripper()
            rclpy.spin_once(self, timeout_sec=3.0)

def main():
    rclpy.init()
    controller = GripperController()
    
    try:
        controller.flex_gripper(3)
    except KeyboardInterrupt:
        controller.get_logger().info("Interrupted")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()