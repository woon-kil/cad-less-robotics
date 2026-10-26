"""CAD-less 비전 경로와 MoveIt Cartesian 노드를 함께 띄운다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    execute = LaunchConfiguration('execute_trajectory')
    require_transform = LaunchConfiguration('require_transform')
    planning_group = LaunchConfiguration('planning_group')
    target_frame = LaunchConfiguration('target_frame')
    cloud_topic = LaunchConfiguration('cloud_topic')
    camera_source = LaunchConfiguration('camera_source')
    pcd_path = LaunchConfiguration('pcd_path')

    return LaunchDescription([
        DeclareLaunchArgument('execute_trajectory', default_value='false'),
        DeclareLaunchArgument('require_transform', default_value='true'),
        DeclareLaunchArgument('planning_group', default_value='arm_group'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/depth/points'),
        DeclareLaunchArgument('camera_source', default_value='relay'),
        DeclareLaunchArgument('pcd_path', default_value=''),
        DeclareLaunchArgument('start_motion_planner', default_value='true'),
        DeclareLaunchArgument('publish_hand_eye_tf', default_value='false'),
        DeclareLaunchArgument('hand_eye_x', default_value='0'),
        DeclareLaunchArgument('hand_eye_y', default_value='0'),
        DeclareLaunchArgument('hand_eye_z', default_value='0'),
        DeclareLaunchArgument('hand_eye_roll', default_value='0'),
        DeclareLaunchArgument('hand_eye_pitch', default_value='0'),
        DeclareLaunchArgument('hand_eye_yaw', default_value='0'),
        DeclareLaunchArgument('hand_eye_parent', default_value='base_link'),
        DeclareLaunchArgument('hand_eye_child', default_value='camera_link'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hand_eye_tf',
            arguments=[
                '--x', LaunchConfiguration('hand_eye_x'),
                '--y', LaunchConfiguration('hand_eye_y'),
                '--z', LaunchConfiguration('hand_eye_z'),
                '--roll', LaunchConfiguration('hand_eye_roll'),
                '--pitch', LaunchConfiguration('hand_eye_pitch'),
                '--yaw', LaunchConfiguration('hand_eye_yaw'),
                '--frame-id', LaunchConfiguration('hand_eye_parent'),
                '--child-frame-id', LaunchConfiguration('hand_eye_child'),
            ],
            condition=IfCondition(LaunchConfiguration('publish_hand_eye_tf')),
        ),
        Node(
            package='vision_path_generator',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'input_topic': cloud_topic,
                'output_topic': '/vision/camera/points',
                'source': camera_source,
                'pcd_path': pcd_path,
                'frame_id': LaunchConfiguration('hand_eye_child'),
            }],
        ),
        Node(
            package='vision_path_generator',
            executable='path_extractor_node',
            name='path_extractor_node',
            output='screen',
            parameters=[{
                'input_topic': '/vision/camera/points',
                'output_topic': '/vision/waypoints',
                'target_frame': target_frame,
                'require_transform': ParameterValue(require_transform, value_type=bool),
            }],
        ),
        Node(
            package='robot_controller',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_motion_planner')),
            parameters=[{
                'pose_topic': '/vision/waypoints',
                'planning_group': planning_group,
                'planning_frame': target_frame,
                'execute_trajectory': ParameterValue(execute, value_type=bool),
            }],
        ),
    ])
