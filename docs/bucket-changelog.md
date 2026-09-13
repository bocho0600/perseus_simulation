# Excavation bucket — what changed and why

Everything added or touched to get the excavation bucket into the Perseus
simulation, and the reasoning behind each decision. For how the bucket itself is
built, see [bucket-urdf.md](bucket-urdf.md).

Nothing here changes the rover when the bucket is off. `use_bucket:=false`
reproduces the original 33-link / 36-joint robot exactly; with the bucket it is
43 links and 46 joints.

---

## Where this started

The starting point was `src/perseus_description/urdf/gen_files/` — an Onshape
export produced by `~/Documents/projects/onshape-urdf/to_urdf_test.py`
(`onshape_robotics_toolkit`), post-processed by a script called
`fix_bucket_urdf.py`. It could not be used as-is, for four independent reasons:

1. **The meshes were not in the workspace.** All 17 links pointed at
   `meshes/bucket/*.stl` and none existed here. They did exist — in the
   toolkit's own `output/meshes/` — they had simply never been copied across.
   This turned out to be the single biggest unlock.
2. **The inertials are unrecoverable.** Onshape's mass-properties endpoint
   answers `"hasMass": false, "massMissingCount": 53` — not one part has a
   material assigned. Worse, the exported centres of mass are in *assembly*
   coordinates, not link coordinates (the 0.7 m arm has its CoM at `x = 0.945`).
3. **The loop closures could never work.** `bucket_loops.urdf.xacro` re-applies
   four removed joints as raw SDF. Two things kill it: the links it references
   hang off *fixed* joints and get welded into their parents during URDF→SDF
   conversion, and — more fundamentally — gz-physics-dartsim refuses closed
   loops outright (verified with a minimal four-bar: *"the child link already
   has a parent joint"*).
4. **The mount pose was `0 0 0`,** marked "MEASURE THESE", which buries the
   attachment inside the chassis.

So the description was rebuilt from the parts of the export that *are*
trustworthy: the mate transforms and the meshes.

---

## Files created

### `src/perseus_description/urdf/bucket.urdf.xacro`

The bucket description. Hand-authored, but every number in it is measured or
derived rather than estimated — see [bucket-urdf.md](bucket-urdf.md).

**How it was built.** The export's joint origins were forward-kinematicked into
a single frame with a throwaway script, which recovers the linkage dimensions
(these are mate transforms, so they are rigid-body facts and are unaffected by
the broken mass properties). The meshes were then measured directly: bounding
boxes for sizes, and exact tetrahedron integration for volume, centroid and
inertia tensor. Volume summed over all meshes came to 4118.4 cm³ against
Onshape's own 4138.7 cm³ for the assembly — a 0.5% tessellation difference,
which is what confirms the mesh integration is sound.

### `src/perseus_description/meshes/bucket/` — 22 STL files

16 are the toolkit's export, copied unchanged.

The other 6 are generated: each `250mm_Actuator_N.stl` is a **fused** barrel and
shaft, which cannot telescope. They turned out to be two *disconnected* solids
in one file, so a union-find over shared vertices splits them cleanly into
`_base` (the half bolted to its parent) and `_slider`. No re-export was needed.

Note the tilt ram's halves are the reverse of the lift rams': its barrel is the
arm-side part. `_base`/`_slider` name the **role**, not the part.

### `src/perseus_simulation/scripts/bucket_ram_follower.py`

Solves the three rams' angles and extensions in closed form from the lift and
tilt joint angles, and publishes them to the Gazebo position controllers.

**Why a node at all.** The rams close two kinematic loops. Every in-engine
option was tried and rejected:

| Option | Result |
|---|---|
| Real closed loop (`<gazebo><joint>`) | gz-physics-dartsim refuses it outright |
| URDF `<mimic>` | Converts into SDF `<mimic>` cleanly, but dartsim does **not** enforce it — verified on a two-bar test model, the dependent joint sat frozen at 0.000 while the driver was at 0.502 rad |
| `<mimic>` even if it worked | It is affine, and both β(q) and s(q) are nonlinear; the best affine fit still misses the rod-end pin by 30 mm mid-travel |

Solving it in a node is exact. Verified by full forward kinematics over an
81-pose sweep of the workspace: **worst-case rod-end miss 0.02 mm**.

The rams are cosmetic — token mass, no collision, gravity disabled — so the
physics is identical whether or not this node runs. Without it they hold their
last pose rather than collapsing.

### `src/perseus_description/cad/bucket/`

The original export, moved out of `urdf/` (which CMake installs wholesale — a
broken xacro should not sit next to the real ones) and kept as the dimensional
source of truth. Its `README.md` records which numbers were salvaged and the
traps in the export.

### `docs/`

This file and [bucket-urdf.md](bucket-urdf.md).

---

## Files modified

All eight changes are integration-only. No existing xacro's geometry, mass or
behaviour was altered.

| File | Change | Why |
|---|---|---|
| `perseus_description/urdf/perseus.urdf.xacro` | `<xacro:include>` for the bucket; instantiate it under `<xacro:if value="$(arg use_bucket)">` | The only hook the bucket needs. Guarded so the bare rover is untouched. |
| `perseus/urdf/perseus.urdf.xacro` | Declare `<xacro:arg name="use_bucket" default="false"/>` | `xacro:arg` must sit at file scope, and this is the top-level file every launch goes through. Defaults off. |
| `perseus/launch/robot_state_publisher.launch.py` | Accept `use_bucket` and pass it to xacro | Threads the argument from launch down into the description. |
| `perseus_simulation/launch/perseus_sim.launch.py` | Declare `use_bucket` (**default `true`**), forward it to RSP, launch `bucket_ram_follower` under `IfCondition` | The simulation is the one place that wants the bucket on by default. |
| `perseus_simulation/config/gz_bridge.yaml` | Six `ROS_TO_GZ` `std_msgs/Float64` entries | Two command the bucket; four drive the rams. |
| `perseus_simulation/CMakeLists.txt` | `install(PROGRAMS scripts/bucket_ram_follower.py ...)` | Makes the node runnable as `ros2 run perseus_simulation bucket_ram_follower.py`. |
| `perseus_simulation/package.xml` | `rclpy`, `sensor_msgs`, `std_msgs` exec deps | The package had no Python node before. |
| `.gitignore` | `__pycache__/` | Byte-compiled output from importing the follower during verification. |

`src/perseus_simulation/models/perseus_arc_world/cubes/model.sdf` is also
modified in the working tree — that is pre-existing local work (a cube resized
0.1 → 1.5 m), unrelated to the bucket, and is deliberately **not** part of this
branch.

---

## Why the bucket is not a ros2_control device

The two bucket joints are driven by Gazebo's own
`gz::sim::systems::JointPositionController`, not by `controller_manager`. Two
reasons:

- **The real robot does not use ros2_control here either.** `bucket_driver` in
  the `payloads` package writes `LIFT_BOTH` and `TILT_BOTH` speeds straight over
  CAN. The joint names in the description match those groups.
- **The controllers are not installed.** The pixi environment has only
  `diff_drive_controller`, `mecanum_drive_controller` and
  `joint_state_broadcaster`. Adding `position_controllers` means a full pixi
  re-solve against a 650 kB lockfile, which was not worth it for a working sim.

If you want it through `controller_manager` later, add
`ros-jazzy-position-controllers` to `pixi.toml`, move the two joints into the
`<ros2_control>` block in `perseus.ros2_control.xacro` under an `is_sim` guard,
and drop the two `JointPositionController` plugins.

`joint_state_broadcaster` only reports joints that are in `<ros2_control>`, i.e.
the four wheels — so the description also carries a
`gz-sim-joint-state-publisher-system` listing the eight bucket and ram joints.
It publishes on the `joint_state` topic the bridge already maps to
`/joint_states`, so `robot_state_publisher` gets everything and nothing fights
over the wheels.

---

## How it was verified

- `check_urdf`: single root, clean tree, both with and without the bucket.
- `gz sdf -p`: plugins and joints survive URDF→SDF conversion.
- Forward kinematics vs. TF: `base_link → bucket` matches hand-computed FK to
  1 mm and 0.1°.
- Ram loop closure: 0.02 mm worst case over an 81-pose sweep.
- Ram tracking in the running sim: within 0.45° and 2 mm of the closed-form
  target.
- **Full excavation cycle**: a 1 kg block dropped into the scoop, curled,
  lowered to the ground, raised — returned to within 1 mm of where it started —
  then dumped on command.
- Driving: 1.03 m with yaw while loaded; bucket joints held to 0.0001 rad, no
  tipping.

---

## Known issues

- **The frame gantry stands around the sensor mast.** With the CAD-derived
  mount the gantry occupies base_link x 0.600–0.617, z 0.473–0.773; the Livox is
  at (0.600, 0, 0.700) — inside the gantry's window, not inside a beam.
  Measured cost: the horizontal LiDAR ring goes from 240 returns (none closer
  than 1 m) to 307, of which 78 are clipped at the 0.1 m sensor minimum and 46
  more hit rover structure. **About 40% of the ring becomes the machine
  itself.** This is physically true of the real rover with this attachment, so
  it is left as-is rather than quietly moved. Either add a crop-box self-filter
  before fast_lio, or correct `bucket_mount_x`/`bucket_mount_z`.
- **Masses depend on one assumed density** (2700 kg/m³, 6061 aluminium). The
  volumes are exact; only the density is a guess. Assigning materials in Onshape
  and re-exporting is the only real fix.
- **The clamshell jaws are not modelled.** `Bucket_1` contains two 50 mm
  actuators — the `JAWS_BOTH` group — flattened into one rigid body by the
  export's `cad.max_depth: 0`. The floor collision spans the gap between the
  halves, i.e. the jaws are simulated permanently closed.
- **Real-time factor is ~0.2–0.4** on this machine. Measured the same with
  `use_bucket:=false`, so the bucket is not the cause.
