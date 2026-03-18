from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import math


def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    left_translation = [0.0,0.797/2,0.0] # -0.797/2
    left_rotation = [0.0,0.0,math.pi] # -pi/2
    right_translation = [0.0,-0.797/2,0.0] # 0.797/2
    right_rotation = [0.0,0.0,0.0] # pi/2
    
    left_ur_type = LaunchConfiguration("left_ur_type")
    left_safety_limits = LaunchConfiguration("left_safety_limits")
    left_safety_pos_margin = LaunchConfiguration("left_safety_pos_margin")
    left_safety_k_position = LaunchConfiguration("left_safety_k_position")
    left_kinematics_params_file = LaunchConfiguration("left_kinematics_params_file")
    
    left_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "joint_limits.yaml"])
    left_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "physical_parameters.yaml"])
    left_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "visual_parameters.yaml"])

    right_ur_type = LaunchConfiguration("right_ur_type")
    right_safety_limits = LaunchConfiguration("right_safety_limits")
    right_safety_pos_margin = LaunchConfiguration("right_safety_pos_margin")
    right_safety_k_position = LaunchConfiguration("right_safety_k_position")
    right_kinematics_params_file = LaunchConfiguration("right_kinematics_params_file")
    
    right_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "joint_limits.yaml"])
    right_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "physical_parameters.yaml"])
    right_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "visual_parameters.yaml"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=dual_arm_workcell",
            " ",
            "left_robot_ip:=xxx.yyy.zzz.www",
            " ",
            "left_tf_prefix:=left_",
            " ",
            "left_joint_limit_params:=",
            left_joint_limit_params,
            " ",
            "left_kinematics_params:=",
            left_kinematics_params_file,
            " ",
            "left_physical_params:=",
            left_physical_params,
            " ",
            "left_visual_params:=",
            left_visual_params,
            " ",
            "left_safety_limits:=",
            left_safety_limits,
            " ",
            "left_safety_pos_margin:=",
            left_safety_pos_margin,
            " ",
            "left_safety_k_position:=",
            left_safety_k_position,
            " ",
            "left_name:=",
            "left_ur16e",
            " ",
            "left_script_filename:=",
            "ros_control.urscript",
            " ",
            "left_input_recipe_filename:=",
            "rtde_input_recipe.txt",
            " ",
            "left_output_recipe_filename:=",
            "rtde_output_recipe.txt",
            " ",
            "left_translation_x:=",
            str(left_translation[0]),
            " ",
            "left_translation_y:=",
            str(left_translation[1]),
            " ",
            "left_translation_z:=",
            str(left_translation[2]),
            " ",
            "left_rotation_r:=",
            str(left_rotation[0]),
            " ",
            "left_rotation_p:=",
            str(left_rotation[1]),
            " ",
            "left_rotation_y:=",
            str(left_rotation[2]),
            " ",
            "left_tool0_x:=",
            "0.0",
            " ",
            "left_tool0_y:=",
            "0.0",
            " ",
            "left_tool0_z:=",
            "0.12",
            " ",
            "right_robot_ip:=xxx.yyy.zzz.www",
            " ",
            "right_tf_prefix:=right_",
            " ",
            "right_joint_limit_params:=",
            right_joint_limit_params,
            " ",
            "right_kinematics_params:=",
            right_kinematics_params_file,
            " ",
            "right_physical_params:=",
            right_physical_params,
            " ",
            "right_visual_params:=",
            right_visual_params,
            " ",
            "right_safety_limits:=",
            right_safety_limits,
            " ",
            "right_safety_pos_margin:=",
            right_safety_pos_margin,
            " ",
            "right_safety_k_position:=",
            right_safety_k_position,
            " ",
            "right_name:=",
            "right_ur16e",
            " ",
            "right_script_filename:=",
            "ros_control.urscript",
            " ",
            "right_input_recipe_filename:=",
            "rtde_input_recipe.txt",
            " ",
            "right_output_recipe_filename:=",
            "rtde_output_recipe.txt",
            " ",
            "right_translation_x:=",
            str(right_translation[0]),
            " ",
            "right_translation_y:=",
            str(right_translation[1]),
            " ",
            "right_translation_z:=",
            str(right_translation[2]),
            " ",
            "right_rotation_r:=",
            str(right_rotation[0]),
            " ",
            "right_rotation_p:=",
            str(right_rotation[1]),
            " ",
            "right_rotation_y:=",
            str(right_rotation[2]),
            " ",
            "right_tool0_x:=",
            "0.0",
            " ",
            "right_tool0_y:=",
            "-0.12",
            " ",
            "right_tool0_z:=",
            "0.083",
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # robot_description and param moveit param initialization is finished, now moveit config preparation needs to be done
    # moveit config stuff
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            )
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(value=robot_description_semantic_content,value_type=str)
    }

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "left_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        },
        "right_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        }
    }}

    #### nodes
    co1_deconing_node = Node(
        package="smw_2026",
        executable="co1_deconing",
        name="co1_deconing",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
            },
        ]
    )

    so1_deconing_node = Node(
        package="smw_2026",
        executable="so1_deconing",
        name="so1_deconing",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
            },
        ]
    )

    co4_deconing_node = Node(
        package="smw_2026",
        executable="co4_deconing",
        name="co4_deconing",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
            },
        ]
    )

    left_task_space_cubic_polynomial_trajectory_server = Node(
        package="motion_planning_abstractions",
        executable="task_space_cubic_polynomial_trajectory_server",
        name="left_task_space_cubic_polynomial_trajectory_server",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group" : "left_ur16e",
                "maximum_task_space_velocity" : 1.0,
                "maximum_task_space_acceleration" : 3.0,
                "maximum_joint_space_velocity" : 3.15,
                "maximum_joint_space_acceleration" : 3.14,
                "arm_side" : "left",
                "joint_trajectory_controller" : "left_scaled_joint_trajectory_controller",
                "endeffector_link" : "left_tool0",
            },
            {"use_sim_time":use_sim_time},
        ],
    )
    
    right_task_space_cubic_polynomial_trajectory_server = Node(
        package="motion_planning_abstractions",
        executable="task_space_cubic_polynomial_trajectory_server",
        name="right_task_space_cubic_polynomial_trajectory_server",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group_" : "right_ur16e",
                "maximum_task_space_velocity_" : 1.0,
                "maximum_task_space_acceleration_" : 3.0,
                "maximum_joint_space_velocity_" : 3.15,
                "maximum_joint_space_acceleration_" : 3.14,
                "arm_side" : "right",
                "joint_trajectory_controller_" : "right_scaled_joint_trajectory_controller",
                "endeffector_link_" : "right_tool0",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    # left and right rest
    left_rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_rest_server",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": -0.17128450075258428,
                "shoulder_lift": -0.6185867947391053,
                "elbow": -2.2505955696105957,
                "wrist_1": 0.07810251295056148,
                "wrist_2": -1.5973799864398401,
                "wrist_3": 3.3431692123413086,
                "side":"left",
                "endeffector_link":"left_tool0",
                "side":"left",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    right_rest_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_rest_server",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -2.345877472554342,
                "shoulder_lift": -2.1818658314146937,
                "elbow": 2.570261303578512,
                "wrist_1": -3.2558514080443324,
                "wrist_2": -5.470438305531637,
                "wrist_3": 2.995588779449463,
                "joint_trajectory_controller":"right_scaled_joint_trajectory_controller",
                "endeffector_link":"right_tool0",
                "side":"right",
            },
            {"use_sim_time":use_sim_time},
        ],
    )

    # left and right preaction
    # not calibrated yet
    left_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_preaction_server",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": 0.7448494020060257,
                "shoulder_lift": -1.6898253014238567,
                "elbow": -2.456773372167499,
                "wrist_1": -0.6417658203742237,
                "wrist_2": -1.5154170572868646,
                "wrist_3": 2.4306718044994686,
                "side": "left",
                "endeffector_link": "left_tool0",
                "side": "left",
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    right_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_preaction_server",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -2.3459141890155237,
                "shoulder_lift": -2.1818372211852015,
                "elbow":2.570200506840841,
                "wrist_1": -3.2559219799437464,
                "wrist_2": -5.47044271627535,
                "wrist_3":2.9955930709838867,
                "joint_trajectory_controller": "right_scaled_joint_trajectory_controller",
                "endeffector_link": "right_tool0",
                "side": "right",
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    nodes_to_start = [
        left_task_space_cubic_polynomial_trajectory_server,
        right_task_space_cubic_polynomial_trajectory_server,
        # left_rest_server,
        # right_rest_server,
        left_preaction_server,
        right_preaction_server,
        co1_deconing_node,
        co4_deconing_node,
        so1_deconing_node,
    ]
    
    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []

    # general arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="publish robot description semantic?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="dual_arm_workcell_moveit_config",
            description="dual_arm_workcell_moveit_config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="dual_arm_workcell.srdf",
            description="MoveIt SRDF file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
            description="Launch Servo?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="dual_arm_workcell_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="dual_arm_workcell.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # left robot arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("left_ur_type"),
                    "left_default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual left robot used.",
        )
    )

    # right robot arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("right_ur_type"),
                    "right_default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual right robot used.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])