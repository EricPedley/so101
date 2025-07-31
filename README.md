# SO-101

This is a toy project to practice my robotics skills. The goal is to, in 24 hours, train a robot arm policy with the SO-101 robot arm in simulation in Isaac Gym, then deploy the policy in Gazebo. The goal will be to have the arm stack two cubes, given only the camera feed and joint angles


# Testing isaac sim control
1. isaac_py /so101_moveit_control/launch/isaac_moveit.py`
2. ros2 run joint_state_publisher_gui joint_state_publisher_gui
3. (new terminal) ros2 topic pub /robot_description std_msgs/msg/String "data: '$(cat /home/miller/code/so101/models/SO101/so101_new_calib.urdf)'" --once

4. ros2 run topic_tools relay /joint_states /joint_command