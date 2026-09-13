# The bucket URDF, and how it plugs into Perseus

Reference for `src/perseus_description/urdf/bucket.urdf.xacro`. For the change
list and the decisions behind it, see [bucket-changelog.md](bucket-changelog.md).

---

## 1. What the machine is

A front-loader attachment that bolts to the front of the chassis. A 410 × 300 mm
gantry of 20×20 V-slot extrusion carries four bearing blocks. Two 40×20 mm arms
pivot on the upper pair; two 250 mm linear actuators pivot on the lower pair,
240 mm below, and push brackets 597 mm out along the arms. A third 250 mm
actuator is bolted to a plate on the arms and reaches forward to tilt the
bucket.

That gives two independent degrees of freedom, and they match the actuator
groups the real robot drives over CAN (`bucket_driver` in the `payloads`
package writes `LIFT_BOTH` and `TILT_BOTH`):

- **lift** — the arms pitch, raising and lowering the bucket fixture
- **tilt** — the bucket pitches about the arm tips

The bucket itself is a **clamshell**: `Bucket_1` contains a FRONT frame, a REAR
frame, a `BucketClipVslotMount` and two 50 mm actuators — the `JAWS_BOTH` group.
The export flattens all of that into one rigid body and so does this
description, so the jaws are a third DOF that is **not** modelled; the floor
collision spans the gap between the halves, i.e. the jaws are simulated closed.

---

## 2. Kinematic structure

```
chassis
 └─ bucket_mount_joint   (fixed)      ← the one tunable: where it bolts on
    └─ bucket_mounting_frame          ← interface link, base_link axes
       └─ bucket_frame_joint (fixed)
          └─ bucket_frame             ← gantry, bearing blocks, rails
             ├─ bucket_lift_joint     (revolute)  ▸ DRIVEN
             │  └─ bucket_arm         ← both tubes, brackets, tilt plate
             │     ├─ bucket_tilt_joint (revolute) ▸ DRIVEN
             │     │  └─ bucket       ← the clamshell scoop
             │     │     └─ (nothing)
             │     └─ ram_tilt_pitch_joint (revolute) ▸ follower
             │        └─ ram_tilt_barrel
             │           └─ ram_tilt_extend_joint (prismatic) ▸ follower
             │              └─ ram_tilt_shaft
             ├─ ram_lift_left_pitch_joint  (revolute) ▸ follower
             │  └─ ram_lift_left_barrel
             │     └─ ram_lift_left_extend_joint (prismatic) ▸ follower
             │        └─ ram_lift_left_shaft
             └─ ram_lift_right_… (same again)
```

`bucket_mounting_frame` is the only name the rest of the robot needs to know.
Its origin is the midpoint of the two arm pivots and its axes match `base_link`:
+x forward, +y left, +z up.

### Driven joints

| Joint | Type | Range (rad) | Effort | Velocity | Zero is |
|---|---|---|---|---|---|
| `bucket_lift_joint` | revolute, axis `0 -1 0` | −1.2455 … 0 | 335.8 N·m | 0.447 rad/s | top of travel (transport) |
| `bucket_tilt_joint` | revolute, axis `0 -1 0` | −0.4904 … +1.600 | 168.9 N·m | 0.888 rad/s | the CAD pose |

Positive lift raises. Positive tilt curls the scoop back to retain a load;
negative dumps.

### Why those exact limits

Both are derived from the actuator geometry, not chosen. A "250 mm Actuator" is
drawn at 0.365 m pin-to-pin with 0.250 m of stroke, so it works between 0.365 m
and 0.615 m.

**Lift.** With `r` the bracket radius (0.59712 m) and `d` the pivot drop
(0.240 m), the ram length is

```
L(a)² = r² + d² + 2·r·d·sin(a)
```

Retracted gives a = −77.61°, extended gives a = −6.24°. So the arms swing
**71.36°**, and at full extension they still sit 6.24° below the frame's own x
axis — they never quite reach horizontal. That 6.24° is baked into the lift
joint's origin `rpy` so joint zero is the top of travel.

**Tilt.** The ram works between a pin 0.13106 m from the bucket pivot and a
bracket 0.51349 m from it. Full extension opens the included angle to 136.34°
(dump); retracting closes it, but the pins go colinear at 0.382 m before the ram
reaches its 0.365 m retracted length, so the geometric stop is a 1.889 rad fold.
Curl is capped at 1.600 rad, short of the bucket folding onto the arms; the dump
limit is the true actuator limit.

> **Note on the exported pose.** The CAD's exported configuration needs a lift
> ram length of 0.640 m — 25 mm beyond full extension. That is not a CAD error:
> the toolkit places each link by its own mate transform and never solves the
> loops, so what you get is the mate zero of every joint at once. It is also why
> the render in `cad/bucket/` shows the actuators detached from their brackets.

---

## 3. Mass and inertia

Onshape reports `"hasMass": false, "massMissingCount": 53` — no part has a
material. So mass is **exact CAD volume × one assumed density**, and that density
is the only genuine assumption in the file:

```xml
<xacro:property name="bucket_density" value="2700.0"/>   <!-- 6061 aluminium -->
```

The frame and rails measure 20×20 mm V-slot at ~70% fill and the parts list
contains a `BucketClipVslotMount`, so extruded aluminium is the right call. For
a steel fabrication set 7850 — every mass and inertia scales by 2.907, and the
file handles that automatically through the `k` scale factor.

| Body | Mass | Contents |
|---|---|---|
| `bucket_frame` | 4.210 kg | gantry + 4 bearing blocks + half of each lift ram |
| `bucket_arm` | 2.860 kg | 2 tubes + 2 brackets + 2 on-arm plates + bracket 4 + ram halves |
| `bucket` | 4.050 kg | the clamshell + half the tilt ram |

Inertia tensors are computed by exact tetrahedron integration over the meshes,
then parallel-axis onto each body's frame — not box approximations. Each
actuator's mass is split in half and lumped at its two pin locations, which is
where it actually loads the linkage.

`bucket_frame` welds into `base_link` during URDF→SDF conversion (it is fixed to
the chassis), so only `bucket_arm` and `bucket` are moving bodies.

---

## 4. Visuals and collision

**Visuals are the CAD meshes** at their exact export transforms, so the model
matches Onshape part for part.

**Collision is fitted boxes**, deliberately:

- the gantry is one slab plus two rails — it welds into the chassis anyway, so a
  672 kB concave mesh collision would cost contact time for nothing;
- the arms are one box per tube;
- **the scoop is four boxes** — floor, rear wall and the two side structures,
  fitted to the measured plate positions. Keeping them separate is what makes
  the scoop concave to the physics engine, so it genuinely carries what it picks
  up. A single bounding box would be a brick, and a concave mesh collision is
  neither fast nor stable in ODE.

---

## 5. The rams

Cosmetic, but exact. Each is a barrel on a revolute joint plus a shaft on a
prismatic joint; the two meshes come from splitting the CAD's fused
`250mm_Actuator_N.stl` at its two disconnected solids.

The tilt ram's **barrel is bolted to the plate on the arms** and its rod reaches
forward to the bucket — even though the export makes `Bucket_1` the *parent* of
`250mm_Actuator_3`. The toolkit roots each mate at whichever body it walked to
first, which is not necessarily the fixed end. Pin positions are identical
either way, so this only matters if you draw the ram: get it wrong and the
barrel telescopes while the rod stays put.

Ram pivots and the brackets they drive are ~10 mm apart laterally in the CAD.
Rod-end bearings absorb that on the real machine; a revolute joint cannot, so
the description moves the skew to the base pin where it is hidden inside the
bearing block. Rod ends then land within **0.02 mm** across the whole workspace.

The rams have token mass (20 g), no collision and gravity disabled, so they
cannot perturb the linkage and they hold their pose if the follower node is not
running. Their real mass is already lumped onto the three load-bearing bodies.

Nothing in Gazebo can make them follow the linkage on its own — closed loops are
refused by dartsim, and `<mimic>` parses but is not enforced — so
`bucket_ram_follower` computes them in closed form. See the changelog for the
evidence.

---

## 6. Mounting

```xml
<xacro:property name="bucket_mount_x" value="0.0591"/>
<xacro:property name="bucket_mount_y" value="0.0"/>
<xacro:property name="bucket_mount_z" value="0.175"/>
```

The frame's two rearward rails run 243 mm back from the gantry at z = 0.018–0.038
in frame coordinates, 390 mm apart. **They bolt to the underside of the
chassis**, so the rail top face sits flush with the chassis floor. The chassis
link origin is the centre of its **front face at the bottom** — the body runs
backwards in −x and upwards in +z from there — so that floor is chassis z = 0:

```
z = 0 − (rail top 0.038 − arm pivot 0.213)                  = 0.175
x = gantry front face 17 mm ahead of the chassis front face = 0.0591
```

Resulting layout:

| | chassis z | base_link z |
|---|---|---|
| rails | −0.020 … 0.000 | 0.330 … 0.350 |
| gantry | −0.095 … +0.205 | 0.255 … 0.555 |
| arm pivot | +0.175 | 0.525 |

So the gantry hangs 95 mm below the chassis floor with 255 mm of ground
clearance against a 150 mm wheel radius, and its top clears the Livox at
base_link 0.700 by 145 mm.

---

## 7. Integration with the main Perseus URDF

Three hooks, nothing more.

**`perseus_description/urdf/perseus.urdf.xacro`** — include and instantiate,
inside the `perseus` macro:

```xml
<xacro:include filename="$(find perseus_description)/urdf/bucket.urdf.xacro"/>
...
<xacro:if value="$(arg use_bucket)">
  <xacro:bucket parent="${prefix}chassis"/>
</xacro:if>
```

`prefix` is inherited automatically via `prefix:=^|''`, the same way `rocker` and
`sensor_mount` work, so it is not passed.

**`perseus/urdf/perseus.urdf.xacro`** — declares the arg. `xacro:arg` has to sit
at file scope, and this is the top-level file every launch resolves through:

```xml
<xacro:arg name="use_bucket" default="false" />
```

**Launch** — `robot_state_publisher.launch.py` accepts `use_bucket` and appends
it to the xacro command; `perseus_sim.launch.py` declares it with
`default_value="true"` and starts `bucket_ram_follower` under an `IfCondition`.

### Turning it on and off

```bash
pixi run sim                      # bucket on (the sim default)
ros2 launch perseus_simulation perseus_sim.launch.py use_bucket:=false
ros2 launch perseus_description view_perseus.launch.py    # bare rover, unchanged
```

Default-off at the description level means every existing launch and the real
robot are untouched.

### Commanding it

Absolute joint angles in radians:

```bash
ros2 topic pub /bucket/lift/cmd_pos std_msgs/msg/Float64 "{data: -0.9}"   # lower
ros2 topic pub /bucket/tilt/cmd_pos std_msgs/msg/Float64 "{data: 0.8}"    # curl
```

A dig-and-carry cycle:

| Step | lift | tilt |
|---|---|---|
| Approach, bucket level at the ground | −0.45 | 0.35 |
| Drive forward to fill | — | — |
| Curl to retain | −0.45 | 1.10 |
| Raise to carry | 0.0 | 1.10 |
| Dump | 0.0 | −0.49 |

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/bucket/lift/cmd_pos` | `std_msgs/Float64` | → sim |
| `/bucket/tilt/cmd_pos` | `std_msgs/Float64` | → sim |
| `/bucket/ram_lift/cmd_angle`, `/cmd_extend` | `std_msgs/Float64` | follower → sim |
| `/bucket/ram_tilt/cmd_angle`, `/cmd_extend` | `std_msgs/Float64` | follower → sim |
| `/joint_states` | `sensor_msgs/JointState` | sim → ROS (all 8 bucket joints) |

All bridged in `perseus_simulation/config/gz_bridge.yaml`.

---

## 8. Editing it

Everything is a property at the top of the file. The ones most likely to need
changing:

| Property | Now | Change it when |
|---|---|---|
| `bucket_density` | 2700 | materials get assigned in Onshape, or it is steel |
| `bucket_mount_{x,y,z}` | 0.0591, 0, 0.395 | you measure how it actually bolts on |
| `tilt_upper` | 1.600 | you know where the bucket really fouls the arms |
| `actuator_force` / `actuator_speed` | 1500 N / 0.1 m/s | you have the real actuator spec (speed is from `bucket_driver.cpp`) |

The linkage dimensions (`arm_length`, `lift_rod_offset`, `lift_pivot_drop`,
`actuator_retracted`, `actuator_stroke`, …) are measured from the CAD — change
them only if the CAD changes, and re-derive the ram constants in
`bucket_ram_follower.py` to match, since they are duplicated there.
