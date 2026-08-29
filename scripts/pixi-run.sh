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

# Use CycloneDDS, the middleware perseus-v3 runs. This is about the point cloud,
# not just parity. /livox/lidar is 737 KB per message (720 x 32 points x 32
# bytes) at 10 Hz, and measured on this world with the rover spawned:
#
#   Gazebo /scan_3d/points          9.75 Hz   (ground truth, RTF 0.98)
#   ROS /livox/lidar, Fast DDS      6.50 Hz   gaps to 0.52 s
#   ROS /livox/lidar, CycloneDDS    9.80 Hz
#
# Fast DDS is the ROS 2 default, so without this line the sim silently loses a
# third of its clouds. Cyclone needs no tuning to carry them - do not add a
# CYCLONEDDS_URI here. The perseus-v3 dev shell sets none either, so both ends
# run Cyclone's default discovery and find each other; a config that pins
# interfaces or peers breaks that. Verified at 9.79 Hz subscribing from the
# perseus-v3 dev shell.
#
# Respects an existing value so you can still override it.
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"
export RMW_IMPLEMENTATION

exec "$@"
