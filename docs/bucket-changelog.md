# Excavation bucket — what changed and why

Everything added or touched to get the excavation bucket into the Perseus
simulation, and the reasoning behind each decision. For how the bucket itself is
built, see [bucket-urdf.md](bucket-urdf.md).

Nothing here changes the rover when the bucket is off. `use_bucket:=false`
reproduces the original 33-link / 36-joint robot exactly; with the bucket it is
44 links and 47 joints.

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

### `src/perseus_description/meshes/bucket/` — 37 STL files

12 are the toolkit's export at `max_depth: 0`, copied unchanged: the frame, the
two arm tubes, the four bearing-block plates, the three actuator brackets and the
two on-arm plates. (The fused `Bucket_1.stl` from that export was dropped, see the
jaws section below.)

19 more are the parts inside the bucket subassembly (FRONT and REAR frames, clip
mount, bearing blocks, bearings, and the jaw actuators' barrels and shafts),
copied from a second export at `max_depth: 2`.

The last 6 are generated: the export's `250mm_Actuator_N.stl` is a **fused**
barrel and shaft, which cannot telescope. It turned out to be two *disconnected*
solids in one file, so a union-find over shared vertices splits it cleanly into
`_base` (the half bolted to its parent) and `_slider`. No re-export was needed.
The fused originals are not kept, since nothing loads them; a re-export at any
`max_depth` regenerates them.

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
| `perseus_simulation/config/gz_bridge.yaml` | Seven `ROS_TO_GZ` `std_msgs/Float64` entries | Three command the bucket (lift, tilt, jaw); four drive the rams. |
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

- **~19% of the Livox horizontal ring is the bucket.** Measured: 239 returns
  with the bucket on, of which 45 fall in 0.15–1 m and are the arms and bucket
  ahead of the rover. None are clipped at the sensor minimum. That is physically
  true of the real machine — a front loader's bucket is in its own forward-down
  view — so a crop-box self-filter before fast_lio is the fix, not a model
  change. (An earlier deck-mounted guess at the mount was far worse: 307
  returns of which 78 were clipped at the 0.1 m minimum, ~40% of the ring lost.
  Bolting the rails under the chassis, which is how it actually mounts, removed
  all of the min-range blockage.)
- **Masses depend on one assumed density** (2700 kg/m³, 6061 aluminium). The
  volumes are exact; only the density is a guess. Assigning materials in Onshape
  and re-exporting is the only real fix.
- **Real-time factor is ~0.2–0.4** on this machine. Measured the same with
  `use_bucket:=false`, so the bucket is not the cause.

---

## The jaws (added after the first version)

The first version flattened the whole bucket subassembly into one rigid body,
because the export ran at `cad.max_depth: 0` and there was nothing to hang a
hinge on. The clamshell's encoders sit at the plate apexes, so that needed fixing.

**Re-export.** Same script, `max_depth` changed to 2, run from a scratch directory
(importing the toolkit rewrites `ORT.yaml` in the working directory). Read-only on
Onshape. Depth 1 already separates the FRONT and REAR halves; depth 2 also splits
the two 50 mm jaw actuators into barrel and shaft, which is why it was used. The
raw export is kept in `cad/bucket/depth2/`.

**Checked against the old bucket before trusting it.** Recomposing the 19 new
parts at mate zero reproduces the old fused mesh: volume 1303.6 vs 1297.5 cm³
(0.5%), bounding-box maxima identical, minima within 0.4 mm except 6 mm at the
shaft ends. The hinge is `Revolute_1-2`, axis `0 1 0`, at (+0.0765, +0.3485,
+0.1589) from the tilt pivot.

**What changed in `bucket.urdf.xacro`.** The single `bucket` link became two:
`bucket` (REAR half, 3.389 kg) and `bucket_jaw` (FRONT half plus the two shafts,
0.678 kg), joined by `bucket_jaw_joint`. Masses and inertias were recomputed from
the meshes. The back plate's collision box stays on `bucket`; the floor and side
boxes moved to `bucket_jaw`. Lift, tilt, the mount and the ram follower are
untouched, and ram tracking is unchanged.

**Files.** Added: 19 STLs, `cad/bucket/depth2/`. Removed: `Bucket_1.stl` (no longer
referenced). Modified: `bucket.urdf.xacro`, `gz_bridge.yaml` (one entry).

**Confirmed: the CAD is drawn with the jaw actuators fully extended.** The CAD does
not say which end of the 50 mm stroke its pose is at, so this was checked against
the real machine: the FRONT frame is already tight against the REAR frame and
cannot be pushed closer. The jaws therefore have `-0.8489 … 0`.

**The jaw actuators are animated too**, by the same follower and the same
scheme as the 250 mm rams: a barrel on a pitch joint pinned to the REAR half,
a rod on a prismatic joint, both sides sharing one angle and one extension on
`/bucket/ram_jaw/cmd_angle` and `/cmd_extend`.

An earlier note here claimed the two actuators were skewed 27 mm sideways and
so needed a 3-D follower. That was wrong, and worth recording because the trap
is easy to fall into. The apparent skew came from reading mate-connector
origins as pin positions: the two base connectors are 393 mm apart and the two
rod connectors 447 mm. But a revolute mate's rotation axis *is* its connector's
local z, and here that axis points along the lateral direction — so the
connector origin can sit anywhere along the pin without moving the pin. Drop
that coordinate and both actuators have identical in-plane geometry (base
`y,z = 0.012, -0.032`; rod `-0.19474, -0.02634` in the REAR frame). They are
planar, and pitch alone aims them. The in-plane pin-to-pin length works out at
206.82 mm, which matches the `jaw_ram_len` the model uses to 0.01 mm.

