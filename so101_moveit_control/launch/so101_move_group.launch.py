from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():
    # Package paths
    moveit_config_pkg = FindPackageShare('so101_moveit_control')
    robot_description_pkg = FindPackageShare('so101_description')
    
    # URDF/xacro file path
    urdf_file = PathJoinSubstitution([
        robot_description_pkg,
        'so101_new_calib.urdf'
    ])
    
    # SRDF file path
    srdf_file = PathJoinSubstitution([
        moveit_config_pkg,
        'so101.srdf'
    ])
    
    # Robot description parameter
    robot_description = {
        'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    }
    
    # Robot description semantic parameter (SRDF)
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(Command(['cat ', srdf_file]), value_type=str)
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml('so101_moveit_control', 'moveit_controllers.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
        
    # Kinematics configuration
    # kinematics_yaml = load_yaml('my_robot_moveit_config', 'config/kinematics.yaml')
    # robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}
    
    planning_pipelines_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                              'default_planner_request_adapters/ResolveConstraintFrames '
                              'default_planner_request_adapters/FixWorkspaceBounds '
                              'default_planner_request_adapters/FixStartStateBounds '
                              'default_planner_request_adapters/FixStartStateCollision '
                              'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    # OMPL planning configuration
    # ompl_planning_yaml = load_yaml('my_robot_moveit_config', 'config/ompl_planning.yaml')
    # planning_pipelines_config['move_group'].update(ompl_planning_yaml)
    
    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            moveit_controllers,
            # trajectory_execution,
            # robot_description_kinematics,
            # robot_description_planning,
            planning_pipelines_config,
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'capabilities': '',
                'disable_capabilities': '',
                'monitor_dynamics': False,
            }
        ],
    )
    
    return LaunchDescription([
        move_group_node
    ])

# Helper function to load YAML files
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None