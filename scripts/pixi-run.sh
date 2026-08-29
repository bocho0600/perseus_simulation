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

exec "$@"
