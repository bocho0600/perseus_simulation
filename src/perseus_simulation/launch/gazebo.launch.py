from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)


def generate_launch_description():
    # ARGUMENTS
    gz_world = LaunchConfiguration("gz_world", default="perseus_arc_world.world")
    headless = LaunchConfiguration("headless")

    # CONFIG + DATA FILES
    gz_bridge_params = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "config", "gz_bridge.yaml"]
    )
    gz_world_path = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "worlds", gz_world]
    )

    arguments = [
        DeclareLaunchArgument(
            "gz_world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perseus_simulation"), "worlds", gz_world]
            ),
            description="The world file from `perseus_simulation` to use",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description=(
                "If true, run only the Gazebo server, with no GUI window. The "
                "lidar and cameras still render, offscreen"
            ),
        ),
        # Default spawn is the centre of the Lunabotics starting zone, facing
        # north into the arena. The world frame matches the guidebook Origin
        # Point (centre of the front wall), so the starting zone is
        # X -2.25..0.00, Y 0.00..2.25 and its centre is (-1.125, 1.125).
        # The guidebook has the robot placed in a randomly selected starting
        # position and direction within that zone, so override these to
        # rehearse other draws.
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="-1.125",
            description="Initial X position of the robot",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="1.125",
            description="Initial Y position of the robot",
        ),
        DeclareLaunchArgument(
            "initial_pose_z",
            default_value="0.35",
            description="Initial Z position of the robot",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="1.5708",
            description="Initial yaw of the robot",
        ),
    ]

    # ENVIRONMENT
    model_path = os.path.join(
        get_package_share_directory("perseus_simulation"), "models"
    )
    # Prepend our models dir to any inherited GZ_SIM_RESOURCE_PATH rather than
    # overwriting it, so meshes provided by other packages (e.g.
    # realsense2_description under the pixi/conda prefix) still resolve.
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        p for p in (model_path, existing_resource_path) if p
    )
    # Our own gz systems (RegolithDig) install to <prefix>/lib.
    plugin_path = os.pathsep.join(
        p
        for p in (
            os.path.join(get_package_prefix("perseus_simulation"), "lib"),
            os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        )
        if p
    )
    set_env = [
        SetEnvironmentVariable("PROJ_IGNORE_CELESTIAL_BODY", "YES"),
        # Ensure the model path is set correctly for Gazebo
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
    ]
    # On a hybrid-graphics laptop GL defaults to the integrated GPU, and this
    # world is rendering-bound (gpu_lidar plus three cameras): on the Intel iGPU
    # it ran at RTF ~0.4, so every sensor published at ~40% of its rate. These
    # ask glvnd for the NVIDIA driver instead (PRIME render offload), for both
    # the GUI (GLX) and --headless-rendering (EGL). Skipped when the NVIDIA
    # driver is absent, where they would leave GL with no vendor at all.
    nvidia_egl_vendor = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(nvidia_egl_vendor):
        set_env += [
            SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
            SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", nvidia_egl_vendor),
        ]

    # IMPORTED LAUNCH FILES
    def gz_launch(context):
        # Perform the world path substitution
        performed_gz_world_path = gz_world_path.perform(context)
        # -s runs the server alone. The gpu_lidar and cameras still need a
        # renderer, which --headless-rendering gives them without a display.
        server_only = (
            "-s --headless-rendering "
            if IfCondition(headless).evaluate(context)
            else ""
        )

        gz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "gz_args": f"{server_only}-r -v 4 {performed_gz_world_path}",
            }.items(),
        )

        return [gz_launch]

    launch_files = [
        OpaqueFunction(function=gz_launch),
    ]

    # NODES
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": gz_bridge_params}],
        output="both",
    )

    # Spawn entity with initial pose parameters
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "perseus",
            "-allow_renaming",
            "true",
            "-x",
            LaunchConfiguration("initial_pose_x"),  # X position
            "-y",
            LaunchConfiguration("initial_pose_y"),  # Y position
            "-z",
            LaunchConfiguration("initial_pose_z"),  # Z position
            "-Y",
            LaunchConfiguration("initial_pose_yaw"),  # Yaw orientation
        ],
        output="both",
    )

    nodes = [
        gz_bridge,
        gz_spawn_entity,
    ]

    return LaunchDescription(set_env + arguments + launch_files + nodes)
