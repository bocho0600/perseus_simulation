#!/usr/bin/env python3
"""
Drive the excavation bucket's three 250 mm rams so they follow the linkage.

Why this node exists
--------------------
The rams close two kinematic loops, and nothing in Gazebo Harmonic can express
that on its own:

  * A real closed loop is refused by the physics backend -
    "Asked to create a joint between links [x] as parent and [y] as child, but
    the child link already has a parent joint" (gz-physics-dartsim).
  * URDF ``<mimic>`` does convert into SDF ``<mimic>``, but dartsim does not
    enforce it - the dependent joint sits frozen at zero.
  * And ``<mimic>`` is affine, whereas both the ram angle and its extension are
    nonlinear in the driving joint. The best affine fit still misses the rod-end
    pin by 30 mm mid-travel.

So the relations are solved here in closed form, exactly, and fed to the Gazebo
position controllers declared in ``bucket.urdf.xacro``.

The rams are cosmetic: their mass is already lumped onto the frame, arm and
bucket, and they have no collision geometry and no gravity. If this node is not
running they simply hold their last pose.

Geometry (all measured from the Onshape export, see perseus_description/cad/bucket)
----------------------------------------------------------------------------
Lift, in the frame's x-z plane, with the arm pivot at the origin:

    bracket   B = r * (cos a, sin a),  a = a_ext + q_lift
    ram base  P = (0, -d)
    ram vector V = B - P  ->  angle = atan2(Vz, Vx), extension = |V| - L_ret

Tilt, in the arm's x-z plane: the barrel is bolted to the plate on the arms, so
the anchor is fixed there, and the pin it drives is on the bucket - it has to be
swung into the arm frame by the tilt angle before the same subtraction.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# ── Linkage constants, mirroring bucket.urdf.xacro ──────────────────────────
LIFT_R = 0.59712          # arm pivot -> lift bracket
LIFT_D = 0.24000          # arm pivot above the lift-ram pivot
LIFT_BASE_X = 0.00004     # ram pivot, relative to the arm pivot
LIFT_BASE_Z = -0.24001
RAM_RETRACTED = 0.365     # pin-to-pin, as drawn in CAD
RAM_STROKE = 0.250

# Tilt, solved in the ARM frame: the ram's barrel is bolted to the plate on the
# arms, and its rod reaches forward to a pin on the bucket. So the anchor is
# fixed in the arm and the target swings with the bucket.
TILT_ANCHOR_X = 0.18674   # Actuator_Bracket_4, in the bucket_arm frame
TILT_ANCHOR_Z = 0.01399
TILT_PIVOT_X = 0.70004    # bucket pivot, in the bucket_arm frame
TILT_PIVOT_Z = -0.00001
TILT_PIN_X = 0.04441      # ram pin, in the bucket frame
TILT_PIN_Z = 0.12331

LIFT_A_EXT = math.asin(
    ((RAM_RETRACTED + RAM_STROKE) ** 2 - LIFT_R**2 - LIFT_D**2)
    / (2 * LIFT_R * LIFT_D)
)


def lift_ram(q_lift):
    """Ram angle in the frame and extension, for a lift joint angle."""
    a = LIFT_A_EXT + q_lift
    vx = LIFT_R * math.cos(a) - LIFT_BASE_X
    vz = LIFT_R * math.sin(a) - LIFT_BASE_Z
    return math.atan2(vz, vx), math.hypot(vx, vz) - RAM_RETRACTED


def tilt_ram(q_tilt):
    """Ram angle in the arm frame and extension, for a tilt joint angle."""
    c, s = math.cos(q_tilt), math.sin(q_tilt)
    # the bucket-side pin, swung into the arm frame by the tilt angle
    px = TILT_PIVOT_X + c * TILT_PIN_X - s * TILT_PIN_Z
    pz = TILT_PIVOT_Z + s * TILT_PIN_X + c * TILT_PIN_Z
    vx, vz = px - TILT_ANCHOR_X, pz - TILT_ANCHOR_Z
    return math.atan2(vz, vx), math.hypot(vx, vz) - RAM_RETRACTED


class BucketRamFollower(Node):
    def __init__(self):
        super().__init__("bucket_ram_follower")
        self.declare_parameter("max_rate_hz", 30.0)
        self._min_dt = 1.0 / float(self.get_parameter("max_rate_hz").value)
        self._last = None

        self._pub = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in (
                ("lift_angle", "/bucket/ram_lift/cmd_angle"),
                ("lift_extend", "/bucket/ram_lift/cmd_extend"),
                ("tilt_angle", "/bucket/ram_tilt/cmd_angle"),
                ("tilt_extend", "/bucket/ram_tilt/cmd_extend"),
            )
        }
        self.create_subscription(JointState, "/joint_states", self._on_states, 10)
        self.get_logger().info("bucket_ram_follower ready")

    def _on_states(self, msg):
        # joint_states arrives from two publishers with different joint sets, so
        # take whichever of ours is present and ignore the rest
        try:
            q_lift = msg.position[msg.name.index("bucket_lift_joint")]
            q_tilt = msg.position[msg.name.index("bucket_tilt_joint")]
        except (ValueError, IndexError):
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last is not None and now - self._last < self._min_dt:
            return
        self._last = now

        la, ls = lift_ram(q_lift)
        ta, ts = tilt_ram(q_tilt)
        for key, value in (
            ("lift_angle", la),
            ("lift_extend", min(max(ls, 0.0), RAM_STROKE)),
            ("tilt_angle", ta),
            ("tilt_extend", min(max(ts, 0.0), RAM_STROKE)),
        ):
            self._pub[key].publish(Float64(data=float(value)))


def main():
    rclpy.init()
    node = BucketRamFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
