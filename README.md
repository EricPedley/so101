# SO-101

This is a toy project to practice my robotics skills. The goal is to, in 24 hours, train a robot arm policy with the SO-101 robot arm in simulation in Isaac Gym, then deploy the policy in Gazebo. The goal will be to have the arm stack two cubes, given only the camera feed and joint angles


# Testing isaac sim control
1. isaac_py /so101_moveit_control/launch/isaac_moveit.py`
2. ros2 run joint_state_publisher_gui joint_state_publisher_gui
3. (new terminal) ros2 topic pub /robot_description std_msgs/msg/String "data: '$(cat /home/miller/code/so101/models/SO101/so101_new_calib.urdf)'" --once

4. ros2 run topic_tools relay /joint_states /joint_command

# Ros workspace installation
I so101_moveit_control and models/SO101 symlinked in to my ros workspace. This is what it looks like:
```
~/code/so101/ros_ws
> ls -l src
total 44
drwxrwxr-x 17 miller miller 4096 Jul 30 21:55 build
drwxrwxr-x 16 miller miller 4096 Jul 30 21:55 install
drwxrwxr-x  8 miller miller 4096 Jul 30 21:04 launch_param_builder
drwxrwxr-x  3 miller miller 4096 Jul 30 21:55 log
drwxrwxr-x 18 miller miller 4096 Jul 30 21:21 moveit2
drwxrwxr-x  9 miller miller 4096 Jul 31 12:04 moveit2_tutorials
drwxrwxr-x 10 miller miller 4096 Jul 30 21:04 moveit_resources
drwxrwxr-x 10 miller miller 4096 Jul 30 21:04 moveit_task_constructor
drwxrwxr-x  8 miller miller 4096 Jul 30 21:05 moveit_visual_tools
drwxrwxr-x  9 miller miller 4096 Jul 30 21:04 rosparam_shortcuts
lrwxrwxrwx  1 miller miller   19 Aug  1 21:22 so101_description -> ../../models/SO101/
lrwxrwxrwx  1 miller miller   27 Jul 31 11:41 so101_moveit_control -> ../../so101_moveit_control/
drwxrwxr-x  9 miller miller 4096 Jul 30 21:04 srdfdom
```