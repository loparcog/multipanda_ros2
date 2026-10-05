import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def concatenate_ns(ns1, ns2, absolute=False):
    
    if(len(ns1) == 0):
        return ns2
    if(len(ns2) == 0):
        return ns1
    
    # check for /s at the end and start
    if(ns1[0] == '/'):
        ns1 = ns1[1:]
    if(ns1[-1] == '/'):
        ns1 = ns1[:-1]
    if(ns2[0] == '/'):
        ns2 = ns2[1:]
    if(ns2[-1] == '/'):
        ns2 = ns2[:-1]
    if(absolute):
        ns1 = '/' + ns1
    return ns1 + '/' + ns2


def generate_launch_description():

    ###
    # Config params
    arm_id_param = 'arm_id'
    initial_positions_param = 'initial_positions'
    
    arm_id = LaunchConfiguration(arm_id_param)
    initial_positions = LaunchConfiguration(initial_positions_param)
    ###


    # Removed, swapped with other sources for robot description
    #moveit_config = (
    #    MoveItConfigsBuilder("moveit_resources_panda")
    #    .robot_description(file_path="config/panda.urdf.xacro")
    #    .to_moveit_configs()
    #)

    ###
    # XACRO and Sim
    load_gripper = True # We make gripper a fixed variable, mainly because parsing the argument 
                            # within generate_launch_description is a fairly unintuitive process, 
                            # and it's not worth doing just for a single boolean.
        
    if(load_gripper): # mujoco scene file must be manually adjusted since there's no way to pass parameters
        scene_file = 'scene.xml'
    else:
        scene_file = 'scene_ng.xml'

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'sim',
                                         'panda_arm_sim.urdf.xacro')
    xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', scene_file)
    franka_bringup_path = get_package_share_directory('franka_bringup')

    # Robot Descriptions
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' arm_id:=', arm_id,
            ' hand:=', str(load_gripper).lower(),
            ' initial_positions:=', initial_positions])

    robot_description = {'robot_description': robot_description_config}

    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                                'srdf',
                                                'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=', str(load_gripper).lower()]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )
    ###

    # Get parameters for the Servo node
    servo_yaml = load_yaml("franka_moveit_config", "config/sim_panda_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_service_launch = ExecuteProcess(
        cmd=[
            [FindExecutable(name="ros2"),
            " service call ",
            "/servo_node/start_servo ",
            "std_srvs/srv/Trigger"]
        ], shell=True
    )

    # RViz
    # TODO: Review this config and compare with original
    rviz_config_file = (
        get_package_share_directory("franka_moveit_config") + "/rviz/servo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    ###
    # ROS2 sim controls
    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'sim_panda_ros_controllers.yaml',
    )
    ###

    ###
    # Mujoco ros2 server
    mujoco_ros2_node = IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(franka_bringup_path + '/launch/sim/launch_mujoco_ros_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_file,
                'verbose': "true",
                'ns': '',
                'mujoco_plugin_config': ros2_controllers_path
            }.items()
        )

    # Joint State Publisher
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Panda Arm Spawner
    panda_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    jsp_source_list = [concatenate_ns('', 'joint_states', True)]
    if(load_gripper):
        jsp_source_list.append(concatenate_ns('', 'panda_gripper_sim_node/joint_states', True))

    joint_state_publisher = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= "",
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30}],
    )
    ###
    

    # Launch as much as possible in components
    container = ComposableNodeContainer(
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
                parameters=[robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            ),
            # To enable controller use (STRANGE JERK AT START, TEST THOROUGHLY BEFORE USING ON HARDWARE)
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::JoyToServoPub",
            #     name="controller_to_servo_node",
            # ),
            
            # ComposableNode(
            #     package="joy",
            #     plugin="joy::Joy",
            #     name="joy_node",
            # )
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ],
        output="screen",
    )

    # RosBridge
    rosbridge_server = IncludeLaunchDescription(
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
    )

    # End effector pose publisher
    pospub = Node(
        package='tf2_publisher',
        executable='franka_pub',
        name='tf2listener'
    )

    ###
    # ARGS
    arm_id_arg = DeclareLaunchArgument(
        arm_id_param,
        default_value='panda',
        description='The name of the robot. Defaults to panda.')
    
    initial_position_arg = DeclareLaunchArgument(
        initial_positions_param,
        default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
        description='Initial joint positions of the robot. Must be enclosed in quotes, and in pure number.'
                    'Defaults to the "communication_test" pose.')
    ###
    

    return LaunchDescription(
        [   
            arm_id_arg,
            initial_position_arg,
            rviz_node,
            mujoco_ros2_node,
            servo_node,
            container,
            # CUSTOM
            rosbridge_server,
            joint_state_broadcaster_spawner,
            panda_arm_spawner,
            joint_state_publisher,
            servo_service_launch,
            pospub
        ]
        # Add list of controllers
    )
