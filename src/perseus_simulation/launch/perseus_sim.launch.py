from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_ekf = LaunchConfiguration("launch_ekf")
    use_bucket = LaunchConfiguration("use_bucket")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "launch_ekf",
            default_value="false",
            description="If true, launch the EKF filter node",
        ),
        DeclareLaunchArgument(
            "use_bucket",
            default_value="true",
            description="If true, attach the excavation bucket to the rover",
        ),
    ]
    # IMPORTED LAUNCH FILES
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_simulation"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "robot_state_publisher.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
            "use_bucket": use_bucket,
        }.items(),
    )
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "launch_controller_manager": "false",
        }.items(),
    )
    # Delay controller startup until Gazebo and ros2_control have had time to
    # spawn the robot and expose the control interfaces. The 10 s value is a
    # conservative fallback for slower first runs or heavier worlds; if startup
    # sequencing changes, this is the place to tune or replace with an event-
    # driven trigger.
    controllers_launch_delayed = TimerAction(
        period=10.0,
        actions=[controllers_launch],
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "rviz", "view.rviz"]
    )
    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "config", "ekf_sim_config.yaml"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # EKF node - only run if launch_ekf parameter is true
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file, {"use_sim_time": use_sim_time}],
        # Explicit remapping to ensure proper topic connections
        remappings=[
            ("/odometry/filtered", "/odometry/filtered"),  # EKF output
        ],
    )
    # Add delay to EKF to ensure all other nodes are ready
    ekf_delayed = TimerAction(
        period=5.0,
        actions=[ekf_node],
        condition=IfCondition(launch_ekf),
    )
    # Drives the bucket's three rams so they track the linkage. Cosmetic: the
    # rams carry no mass or collision, so the sim is correct without it - they
    # just hold their pose. See scripts/bucket_ram_follower.py for why Gazebo
    # cannot do this itself.
    bucket_ram_follower = Node(
        package="perseus_simulation",
        executable="bucket_ram_follower.py",
        name="bucket_ram_follower",
        parameters=[{"use_sim_time": use_sim_time}],
        output="both",
        condition=IfCondition(use_bucket),
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "twist_mux.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    launch_files = [
        gz_launch,
        rsp_launch,
        controllers_launch_delayed,
        ekf_delayed,
        rosbridge_launch,
        twist_mux_launch,
        bucket_ram_follower,
        # rviz,
    ]

    return LaunchDescription(arguments + launch_files)
