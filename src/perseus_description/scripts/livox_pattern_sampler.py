#!/usr/bin/env python3
"""Turn Gazebo's dense, regular gpu_lidar scan into a Livox MID-360 style cloud.

gpu_lidar can only render a fixed azimuth/elevation grid, which repeats exactly every
frame. The real MID-360 does not: its beams trace a rosette that shifts each frame and
only fills the field of view over several scans. This node samples the dense grid along
the MID-360's real beam directions, taking the next slice of the pattern each frame.

Pattern: data/mid360_pattern.npy, the 800,000 unit directions of the MID-360 scan pattern
from the UnitySensors package (Field-Robotics-Japan/UnitySensors,
Runtime/ScriptableObjects/ScanPattern/Livox/Mid-360.asset), converted from Unity's frame
(x right, y up, z forward) to ROS (x forward, y left, z up). Each frame consumes
`points_per_scan` consecutive directions and wraps around after size / points_per_scan
frames (40 frames, 4 s, at the defaults).

Output matches livox_ros_driver2's PointCloud2 (xfer_format 0): x, y, z, intensity (f32),
tag, line (u8), timestamp (f64, ns), 32-byte points.

The dense grid is rendered at one instant, so the per-point timestamps are nominal: a
consumer that motion-compensates with them will correct for motion the render never had.
Set spread_timestamps to false to stamp every point with the frame time instead.

Parameters, the first six of which must match the gpu_lidar in sensors.urdf.xacro:
    h_samples, v_samples            grid size (width, height)
    h_min, h_max, v_min, v_max      grid angular limits, rad
    pattern_path                    the .npy above (defaults to the installed copy)
    points_per_scan                 points per output frame
    frame_period                    seconds spanned by one frame's timestamps
    min_range, max_range            m; points outside are dropped
    noise_sigma                     range noise, m
    spread_timestamps               spread point timestamps across frame_period
"""

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

LIVOX_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"],
        "offsets": [0, 4, 8, 12, 16, 17, 24],
        "itemsize": 32,
    }
)
LIVOX_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=24, datatype=PointField.FLOAT64, count=1),
]
_NP_TYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "<i2",
    PointField.UINT16: "<u2",
    PointField.INT32: "<i4",
    PointField.UINT32: "<u4",
    PointField.FLOAT32: "<f4",
    PointField.FLOAT64: "<f8",
}
MID360_LINES = 4  # laser lines; consecutive pattern entries cycle through them
ORIENTATION_MIN_POINTS = 1000


def cloud_to_array(msg):
    """The PointCloud2's points as a structured array, one element per point."""
    fields = [f for f in msg.fields if f.datatype in _NP_TYPES]
    dtype = np.dtype(
        {
            "names": [f.name for f in fields],
            "formats": [_NP_TYPES[f.datatype] for f in fields],
            "offsets": [f.offset for f in fields],
            "itemsize": msg.point_step,
        }
    )
    return np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)


class LivoxPatternSampler(Node):
    def __init__(self):
        super().__init__("livox_pattern_sampler")
        p = self.declare_parameter
        self._h_samples = p("h_samples", 1440).value
        self._v_samples = p("v_samples", 192).value
        self._h_min = p("h_min", -3.14159).value
        self._h_max = p("h_max", 3.14159).value
        self._v_min = p("v_min", -0.13).value
        self._v_max = p("v_max", 0.915).value
        self._n = p("points_per_scan", 20000).value
        self._period = p("frame_period", 0.1).value
        self._min_range = p("min_range", 0.1).value
        self._max_range = p("max_range", 40.0).value
        self._sigma = p("noise_sigma", 0.02).value
        self._spread = p("spread_timestamps", True).value
        path = p("pattern_path", "").value or os.path.join(
            get_package_share_directory("perseus_description"),
            "data",
            "mid360_pattern.npy",
        )

        self._dirs = np.load(path).astype(np.float32)
        self._size = len(self._dirs)
        self._n = min(self._n, self._size)
        self._az = np.arctan2(self._dirs[:, 1], self._dirs[:, 0])
        self._el = np.arcsin(np.clip(self._dirs[:, 2], -1.0, 1.0))
        self._flip_h = False
        self._flip_v = False
        self._oriented = False
        self._frame = 0
        self._rng = np.random.default_rng()
        self._build_indices()

        self._pub = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.create_subscription(
            PointCloud2, "/livox/lidar_dense", self._on_cloud, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"MID-360 pattern: {self._size} directions, {self._n} per frame "
            f"({self._size // self._n} frames per cycle); grid {self._h_samples}x"
            f"{self._v_samples}"
        )

    def _grid_index(self, az, el, flip_h, flip_v):
        col = np.rint((az - self._h_min) / (self._h_max - self._h_min) * (self._h_samples - 1))
        row = np.rint((el - self._v_min) / (self._v_max - self._v_min) * (self._v_samples - 1))
        col = np.clip(col, 0, self._h_samples - 1).astype(np.int32)
        row = np.clip(row, 0, self._v_samples - 1).astype(np.int32)
        if flip_h:
            col = self._h_samples - 1 - col
        if flip_v:
            row = self._v_samples - 1 - row
        return row, col

    def _build_indices(self):
        self._row, self._col = self._grid_index(
            self._az, self._el, self._flip_h, self._flip_v
        )

    def _detect_orientation(self, xyz, finite):
        """Pick the grid orientation that puts the cloud's points on their cells.

        Which end of a gpu_lidar row or column is the minimum angle is not something to
        assume, and a flip would aim every pattern direction at the wrong cell.
        """
        rows, cols = np.nonzero(finite)
        if len(rows) < ORIENTATION_MIN_POINTS:
            return False
        pick = self._rng.choice(len(rows), ORIENTATION_MIN_POINTS, replace=False)
        rows, cols = rows[pick], cols[pick]
        p = xyz[rows, cols]
        az = np.arctan2(p[:, 1], p[:, 0])
        el = np.arcsin(np.clip(p[:, 2] / np.linalg.norm(p, axis=1), -1.0, 1.0))
        best = None
        for flip_h in (False, True):
            for flip_v in (False, True):
                row, col = self._grid_index(az, el, flip_h, flip_v)
                err = np.abs(row - rows).mean() + np.abs(col - cols).mean()
                if best is None or err < best[0]:
                    best = (err, flip_h, flip_v)
        err, self._flip_h, self._flip_v = best
        self._build_indices()
        self.get_logger().info(
            f"Dense grid orientation: flip_h={self._flip_h} flip_v={self._flip_v} "
            f"(mean cell error {err / 2:.2f})"
        )
        return True

    def _on_cloud(self, msg):
        if msg.height <= 1 or msg.width != self._h_samples or msg.height != self._v_samples:
            self.get_logger().error(
                f"Dense cloud is {msg.width}x{msg.height}, expected {self._h_samples}x"
                f"{self._v_samples} (h_samples/v_samples must match sensors.urdf.xacro)",
                throttle_duration_sec=5.0,
            )
            return
        cloud = cloud_to_array(msg).reshape(msg.height, msg.width)
        xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=-1).astype(np.float32)
        finite = np.isfinite(xyz).all(axis=-1)
        if not self._oriented:
            self._oriented = self._detect_orientation(xyz, finite)

        idx = (self._frame * self._n + np.arange(self._n)) % self._size
        self._frame += 1
        row, col = self._row[idx], self._col[idx]
        hit = xyz[row, col]
        ok = finite[row, col]
        r = np.linalg.norm(np.where(ok[:, None], hit, 0.0), axis=1)
        r = r + self._rng.normal(0.0, self._sigma, r.shape).astype(np.float32)
        ok &= (r >= self._min_range) & (r <= self._max_range)

        keep = np.nonzero(ok)[0]
        out = np.zeros(len(keep), dtype=LIVOX_DTYPE)
        pts = self._dirs[idx[keep]] * r[keep, None]
        out["x"], out["y"], out["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        if "intensity" in cloud.dtype.names:
            out["intensity"] = np.clip(cloud["intensity"][row[keep], col[keep]], 0.0, 255.0)
        else:
            out["intensity"] = 100.0
        out["line"] = idx[keep] % MID360_LINES
        t0 = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        out["timestamp"] = t0 + (keep * (self._period * 1e9 / self._n) if self._spread else 0.0)

        result = PointCloud2()
        result.header = msg.header
        result.height = 1
        result.width = len(out)
        result.fields = LIVOX_FIELDS
        result.is_bigendian = False
        result.point_step = LIVOX_DTYPE.itemsize
        result.row_step = result.point_step * result.width
        result.is_dense = True
        result.data = out.tobytes()
        self._pub.publish(result)


def main():
    rclpy.init()
    rclpy.spin(LivoxPatternSampler())


if __name__ == "__main__":
    main()
