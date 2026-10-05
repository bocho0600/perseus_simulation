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
    PythonExpression,
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
    payload = LaunchConfiguration("payload")
    bucket_controller = LaunchConfiguration("bucket_controller")
    is_bucket = PythonExpression(["'", payload, "' == 'bucket'"])

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
            "payload",
            default_value="bucket",
            description="Which payload to attach to the rover; 'none' for the bare rover",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="If true, run Gazebo without its GUI window",
        ),
        # Same choices and default as perseus-v3's perseus.launch.py.
        DeclareLaunchArgument(
            "bucket_controller",
            default_value="bucket_trajectory_controller",
            choices=[
                "none",
                "bucket_trajectory_controller",
                "bucket_lift_controller",
                "bucket_tilt_controller",
                "bucket_jaw_controller",
            ],
            description=(
                "payload:=bucket only. ros2_control controller that commands the "
                "bucket, spawned into the payloads namespace as on the real robot. "
                "'none' spawns nothing and leaves the bucket joints uncommanded"
            ),
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
            "headless": LaunchConfiguration("headless"),
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
            "payload": payload,
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
    # The bucket shares the drive's controller_manager (gz_ros2_control runs one
    # per model), but its controller is put in the payloads namespace so the
    # action is /payloads/<controller>/follow_joint_trajectory, as on the real
    # robot. Its parameters come from bucket_controllers.yaml via the plugin.
    bucket_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            bucket_controller,
            "--controller-ros-args",
            "-r __ns:=/payloads",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    payload,
                    "' == 'bucket' and '",
                    bucket_controller,
                    "' != 'none'",
                ]
            )
        ),
    )
    controllers_launch_delayed = TimerAction(
        period=10.0,
        actions=[controllers_launch, bucket_controller_spawner],
    )
    # Drives the bucket's rams so they track the linkage. Cosmetic: the rams
    # carry no mass or collision, so the sim is correct without it - they just
    # hold their pose. See perseus_description/scripts/bucket_ram_follower.py for
    # why Gazebo cannot do this itself.
    bucket_ram_follower = Node(
        package="perseus_description",
        executable="bucket_ram_follower.py",
        name="bucket_ram_follower",
        parameters=[{"use_sim_time": use_sim_time}],
        output="both",
        condition=IfCondition(is_bucket),
    )
    # /payloads/joint_states_deg, as on the real robot, where payloads'
    # bucket.launch.py runs this node on the bucket's own /payloads/joint_states.
    # The sim has no /payloads/joint_states, so it reads /joint_states, filtered
    # to the three driven joints. Debug output only; nothing consumes it.
    bucket_joint_states_deg = Node(
        package="perseus_description",
        executable="joint_states_deg.py",
        namespace="payloads",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "joints": [
                    "bucket_lift_joint",
                    "bucket_tilt_joint",
                    "bucket_jaw_joint",
                ],
            }
        ],
        remappings=[("joint_states", "/joint_states")],
        output="both",
        condition=IfCondition(is_bucket),
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
        bucket_joint_states_deg,
        # rviz,
    ]

    return LaunchDescription(arguments + launch_files)
