#!/usr/bin/env bash
# Run a command inside the pixi/RoboStack environment with any leaked system
# ROS paths removed.
#
# Why: sourcing a system ROS (e.g. `source /opt/ros/jazzy/setup.bash`, as the
# `a` shell alias does) before `pixi run` injects /opt/ros/* into
# AMENT_PREFIX_PATH, LD_LIBRARY_PATH, PYTHONPATH, etc. Those system libraries
# are ABI-incompatible with the conda/RoboStack build, so loading both into one
# process corrupts the heap ("corrupted size vs. prev_size") and crashes
# Gazebo. We strip only the /opt/ros/* entries; pixi's own conda paths remain.
set -euo pipefail

strip_ros() {
  local var="$1"
  local val="${!var:-}"
  [ -n "$val" ] || return 0
  local out="" entry
  local IFS=':'
  for entry in $val; do
    case "$entry" in
      /opt/ros/* | "") ;;                 # drop system ROS entries and empties
      *) out="${out:+$out:}$entry" ;;
    esac
  done
  if [ -n "$out" ]; then
    export "$var=$out"
  else
    unset "$var"
  fi
}

if [[ "${AMENT_PREFIX_PATH:-}" == *"/opt/ros/"* || "${LD_LIBRARY_PATH:-}" == *"/opt/ros/"* ]]; then
  echo "pixi-run: stripping leaked system ROS (/opt/ros/*) from the environment" >&2
fi

for v in AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH \
         COLCON_PREFIX_PATH LD_LIBRARY_PATH PATH PYTHONPATH \
         PKG_CONFIG_PATH ROS_PACKAGE_PATH; do
  strip_ros "$v"
done

# DDS middleware. perseus-v3 runs rmw_cyclonedds_cpp, so the sim runs it too -
# same middleware in sim and on the robot. cyclonedds_sim.xml keeps traffic on
# loopback and sizes the transport for the point cloud: /livox/lidar is 737 KB
# per message (720 x 32 points x 32 bytes) at 10 Hz, and on stock DDS settings
# a cloud that big is fragmented and half of them are lost - the gz topic
# published 9.8 Hz while ROS subscribers saw ~5.5 Hz with gaps over a second.
#
# The Fast DDS profile is kept and exported alongside it so that setting
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp still gets a transport that can carry the
# cloud, rather than silently dropping back to 5.5 Hz.
#
# All three respect an existing value, so you can override any of them.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_dir="$repo_root/src/perseus_simulation/config"

: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
export RMW_IMPLEMENTATION

if [ -z "${CYCLONEDDS_URI:-}" ] && [ -f "$config_dir/cyclonedds_sim.xml" ]; then
  export CYCLONEDDS_URI="file://$config_dir/cyclonedds_sim.xml"
fi

if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -f "$config_dir/fastdds_large_data.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$config_dir/fastdds_large_data.xml"
fi

exec "$@"
