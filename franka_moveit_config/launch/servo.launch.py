#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    # RViz is required to operate, so no option to launch without it

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'real',
                                     'panda_arm.urdf.xacro')

    # Robot descriptions
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands]
    )
    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                            'srdf',
                                            'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=', load_gripper]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    # Servo
    servo_yaml = load_yaml("franka_moveit_config", "config/sim_panda_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    rviz_file = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz', 
                             'moveit.rviz')

    # Planning Functionality
    ompl_planning_pipeline_config = {
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
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Controller config
    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'panda_ros_controllers.yaml',
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            'db', 
            default_value='False', 
            description='Database flag'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'],
                    'rate': 30,
                    'use_sim_time': True}],
        ),
        # MoveIt Servo and associated containers
        ComposableNodeContainer(
            name="panda_servo",
            namespace="/",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                # Example of launching Servo as a node component
                # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
                # ComposableNode(
                #     package="moveit_servo",
                #     plugin="moveit_servo::ServoServer",
                #     name="servo_server",
                #     parameters=[
                #         servo_params,
                #         moveit_config.robot_description,
                #         moveit_config.robot_description_semantic,
                #     ],
                # ),
                ComposableNode(
                    package="robot_state_publisher",
                    plugin="robot_state_publisher::RobotStatePublisher",
                    name="robot_state_publisher",
                    parameters=[robot_description, {'use_sim_time': True}],
                ),
                ComposableNode(
                    package="tf2_ros",
                    plugin="tf2_ros::StaticTransformBroadcasterNode",
                    name="static_tf2_broadcaster",
                    parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world", "use_sim_time": True}],
                )
            ],
            output="screen",
        ),
        # Launch a standalone Servo node.
        # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {'use_sim_time': True},
            ],
            output="screen",
        ),
        # RosBridge
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join( get_package_share_directory('rosbridge_server'),
                    'launch', 'rosbridge_websocket_launch.xml')
            ),
            launch_arguments={
                'port': '9090',
                # Additional parameters can be added here, for example:
                # 'address': '',
                # 'ssl': 'false'
            }.items()
        ),
        # End effector pose publisher
        Node(
            package='tf2_publisher',
            executable='franka_pub',
            name='tf2listener',
            parameters=[{'use_sim_time': True, 'hz': 10.0}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                robot_description_kinematics,
            ],
        ),
        Node(
            package='franka_control2',
            executable='franka_control2_node',
            parameters=[{'robot_description': robot_description}, moveit_controllers],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager-timeout",
                "300",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panda_arm_controller",
                "--controller-manager-timeout",
                "300",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware}.items(),
            condition=IfCondition(load_gripper)

        ),
    ])
