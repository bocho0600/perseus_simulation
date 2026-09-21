from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _viewer_nodes(context):
    """Everything whose wiring depends on which payload is attached."""
    bucket = LaunchConfiguration("payload").perform(context) == "bucket"
    gui = LaunchConfiguration("gui")

    # With a bucket the slider GUI publishes to a private topic and reads a
    # private copy of the description; bucket_ram_follower (viewer mode) turns
    # those back into /joint_states with the rams filled in exactly, and into a
    # description in which the ram joints are fixed so they get no slider.
    remap = (
        [("joint_states", "joint_states_raw"),
         ("robot_description", "robot_description_sliders")]
        if bucket else []
    )

    actions = [
        # Slider GUI, or its headless stand-in. Without something publishing
        # /joint_states, robot_state_publisher emits only the fixed joints and
        # the tree is left incomplete.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            remappings=remap,
            output="screen",
            condition=IfCondition(gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            remappings=remap,
            output="screen",
            condition=UnlessCondition(gui),
        ),
    ]
    if bucket:
        # Lives in perseus_simulation, which is installed alongside this package
        # in the workspace. It is not declared as a dependency because
        # perseus_simulation already depends on this package.
        actions.append(
            Node(
                package="perseus_simulation",
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
    gui = LaunchConfiguration("gui")
    use_nixgl = LaunchConfiguration("use_nixgl")

    # payload:=bucket is the interface asked for. The perseus-v3 `description`
    # package does NOT have it yet (no payload argument anywhere in its launch or
    # URDF files), so this is where it is defined for now. It maps onto the
    # use_bucket boolean the description here actually takes.
    payload = LaunchConfiguration("payload")
    use_bucket = PythonExpression(["'true' if '", payload, "' == 'bucket' else 'false'"])

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
            "use_bucket": use_bucket,
        }.items(),
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_description"), "rviz", "view_perseus.rviz"]
    )
    # Plain rviz2 by default. This workspace is managed by pixi, whose rviz2 is a
    # conda build; wrapping it in nixGL swaps in nix's GL libraries and it dies
    # with "Invalid parentWindowHandle". use_nixgl:=true restores the old wrapper
    # for machines where rviz2 comes from nix instead.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(PythonExpression(["'", gui, "' == 'true' and '", use_nixgl, "' != 'true'"])),
    )
    rviz_nixgl = ExecuteProcess(
        cmd=["nix", "run", "--impure", "github:nix-community/nixGL", "--",
             "rviz2", "-d", rviz_config],
        output="screen",
        additional_env={
            "NIXPKGS_ALLOW_UNFREE": "1",
            "QT_QPA_PLATFORM": "xcb",
            "QT_SCREEN_SCALE_FACTORS": "1",
            "ROS_NAMESPACE": "/",
            "RMW_QOS_POLICY_HISTORY": "keep_last",
            "RMW_QOS_POLICY_DEPTH": "100",
        },
        condition=IfCondition(PythonExpression(["'", gui, "' == 'true' and '", use_nixgl, "' == 'true'"])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "payload",
                default_value="none",
                description="Payload to attach: 'bucket' or 'none'",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description=(
                    "Launch RViz and the joint state slider GUI. Set false to "
                    "publish the TF tree only, for headless/SSH use"
                ),
            ),
            DeclareLaunchArgument(
                "use_nixgl",
                default_value="false",
                description="Start RViz through nixGL (only for a nix-provided rviz2)",
            ),
            rsp_launch,
            rviz,
            rviz_nixgl,
            OpaqueFunction(function=_viewer_nodes),
        ]
    )
