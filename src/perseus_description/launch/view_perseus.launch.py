from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _joint_state_nodes(context):
    """The slider GUI and, for the bucket, its ram node.

    With payload:=bucket the rams are not free joints: each is a function of the
    lift, tilt and jaw angles. So the slider GUI is rerouted through
    bucket_ram_follower: it publishes /joint_states_raw and reads
    /robot_description_sliders, and the follower republishes /joint_states with
    the ten ram joints exact, plus a description in which they are fixed so they
    get no slider. robot_state_publisher still reads the real description.
    """
    bucket = LaunchConfiguration("payload").perform(context) == "bucket"
    remap = (
        [
            ("joint_states", "joint_states_raw"),
            ("robot_description", "robot_description_sliders"),
        ]
        if bucket
        else []
    )
    actions = [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            remappings=remap,
            output="screen",
        )
    ]
    if bucket:
        actions.append(
            Node(
                package="perseus_description",
                executable="bucket_ram_follower.py",
                name="bucket_ram_follower",
                parameters=[{"viewer_mode": True}],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    hardware_plugin = LaunchConfiguration(
        "hardware_plugin", default="mock_components/GenericSystem"
    )
    can_bus = LaunchConfiguration("can_bus", default="")
    payload = LaunchConfiguration("payload")

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
            "hardware_plugin": hardware_plugin,
            "can_bus": can_bus,
            "payload": payload,
        }.items(),
    )

    # RViz with nixGL support
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_description"), "rviz", "view_perseus.rviz"]
    )
    rviz = ExecuteProcess(
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env={
            "NIXPKGS_ALLOW_UNFREE": "1",
            "QT_QPA_PLATFORM": "xcb",
            "QT_SCREEN_SCALE_FACTORS": "1",
            "ROS_NAMESPACE": "/",
            "RMW_QOS_POLICY_HISTORY": "keep_last",
            "RMW_QOS_POLICY_DEPTH": "100",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "payload",
                default_value="none",
                description=(
                    "Payload attachment to include on the chassis. Set to "
                    "'bucket' to add the bucket: frame mount, lift arms, bucket, "
                    "jaw and rams, with sliders for lift, tilt and jaw"
                ),
            ),
            rsp_launch,
            rviz,
            OpaqueFunction(function=_joint_state_nodes),
        ]
    )
