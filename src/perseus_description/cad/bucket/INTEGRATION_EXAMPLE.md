# Wiring the bucket into the real Perseus description

Written after reading the actual `perseus_description` xacro files, and
validated by expanding the real tree with `use_bucket` both off and on.

---

## What reading your repo changed

| Assumption I made earlier | What Perseus actually does |
|---|---|
| chassis link is `chassis_link` | it is **`${prefix}chassis`** |
| meshes via `package://` | **`file://$(find perseus_description)/meshes/...`** |
| macro takes `prefix:=''` | sub-macros use **`prefix:=^`** (inherit from caller's scope) |
| literal `<inertia>` tensors | repo has **`xacro:box_inertia`** etc. in `inertia_macros.xacro` |
| files named `bucket.xacro` | repo convention is **`<name>.urdf.xacro`** |
| collision = full mesh | collision is a **simplified box**, deliberately |
| `<robot xmlns:xacro="http://www.ros.org/wiki/xacro">` | most files use **`http://ros.org/wiki/xacro`** (both work) |

The generated files now follow all of these.

---

## Do you need both files?

**`bucket.urdf.xacro`** — yes, always. This is the bucket.

**`bucket_loops.urdf.xacro`** — depends on what you're doing:

| What you're doing | Need loops file? |
|---|---|
| RViz / `robot_state_publisher` / TF tree | No |
| Checking mesh placement and joint axes | No |
| Nav2, costmaps, anything using TF only | No |
| **Gazebo physics — the actual excavation sim** | **Yes** |

The loops file contains nothing but a `<gazebo>` block. Every URDF parser
outside Gazebo ignores `<gazebo>` entirely, so **including it always is
harmless** and saves you forgetting it later. That's what I'd do.

The catch: without the loop closures, the four-bar linkage isn't closed. In
RViz that's invisible (you're posing joints by hand). In Gazebo the brackets and
pistons will swing free and the bucket will collapse under gravity. So if the
bucket flops in Gazebo, the first thing to check is whether the loops file got
included and survived SDF conversion.

---

## Wiring it in

Two edits to `perseus.urdf.xacro`.

**1. Add the includes** alongside the existing ones:

```xml
<xacro:include filename="$(find perseus_description)/urdf/bucket.urdf.xacro"/>
<xacro:include filename="$(find perseus_description)/urdf/bucket_loops.urdf.xacro"/>
```

**2. Instantiate it**, near the end of the `perseus` macro (after the chassis
joint exists — order within the macro doesn't strictly matter to xacro, but it
reads better next to `sensor_mount`):

```xml
<xacro:if value="$(arg use_bucket)">
  <xacro:bucket parent="${prefix}chassis"/>
  <xacro:bucket_loops/>
</xacro:if>
```

`prefix` is inherited automatically via `prefix:=^|''`, matching how `rocker`
and `sensor_mount` work, so you don't pass it.

**3. Declare the arg.** `xacro:arg` must sit at file scope, not inside a macro.
`perseus.urdf.xacro` only *defines* the `perseus` macro — something else calls
it. Put this in whichever file does the calling, just inside `<robot>`:

```xml
<xacro:arg name="use_bucket" default="false"/>
```

Defaulting to `false` keeps every existing launch working untouched.

**4. Meshes.** Copy the toolkit's `output/meshes/*.stl` into
`perseus_description/meshes/bucket/`.

---

## Launch

```python
DeclareLaunchArgument("use_bucket", default_value="false",
                      description="Attach the excavation bucket")
```

then append to your existing `Command([...xacro...])`:

```python
" use_bucket:=", LaunchConfiguration("use_bucket"),
```

```bash
ros2 launch perseus_description display.launch.py use_bucket:=true
```

---

## Three mismatches you must resolve manually

**1. Mesh scale.** Every existing Perseus mesh carries `scale="0.1 0.1 0.1"`,
which means your `.dae` files are in some non-metre unit. The bucket `.stl`
files are emitted with **no scale attribute**, because the URDF origins
(`0.945`, `0.7`, `0.365`) read as metres already. Do **not** copy the `0.1`
scale across by reflex. Load it in RViz: if the bucket is the right size next
to the rover, leave it alone.

**2. Collision geometry.** `chassis.urdf.xacro` says outright that the mesh was
replaced with a box to keep collision simple. The bucket currently uses the full
visual mesh for collision on all 19 links — that will be slow in Gazebo and can
cause contact jitter. Worth replacing the structural links (mount plates, square
tubes, actuator bodies) with boxes by hand. Leave the **bucket scoop itself** as
a mesh: its concave shape is the entire point for excavation contact.

**3. `perseus_materials.xacro` is never included.** It defines `black`, `grey`,
`orange` etc., but no file in `perseus.urdf.xacro`'s include list pulls it in,
and `motor_wheel.urdf.xacro` has its `<material name="white"/>` commented out.
The bucket therefore ships with self-contained inline `<material>` blocks with
per-link colours from Onshape. If someone later includes the materials file,
switch the bucket to named references for consistency.

---

## Verify

```bash
xacro urdf/robot.urdf.xacro use_bucket:=true > /tmp/perseus.urdf
check_urdf /tmp/perseus.urdf          # expect one root: base_link
urdf_to_graphiz /tmp/perseus.urdf     # eyeball the tree

# confirm Gazebo kept the loop joints through SDF conversion
gz sdf -p /tmp/perseus.urdf | grep -A6 "_loop"
```

I ran the equivalent checks here. With `use_bucket:=false` you get 18 links /
17 joints; with `true`, 38 links / 37 joints. Single root `base_link`, no
multi-parent links, nothing unreachable, no zero-mass or zero-inertia links, and
all four loop joints present in the `<gazebo>` block.

If `gz sdf -p` does **not** show the `_loop` joints, the SDF converter welded
away one of the links they reference. Fixed joints get collapsed during
conversion unless preserved; the fix is to give the affected link a
`<gazebo><preserveFixedJoint>true</preserveFixedJoint></gazebo>` hint or convert
that fixed joint to a revolute with zero limits.

---

## Still outstanding, in priority order

1. **Assign materials to every part in Onshape.** Every mass and inertia in the
   bucket is a placeholder from `MASS_OVERRIDES_KG` in the script. Excavation
   force modelling is meaningless until these are real.
2. **Re-export with `cad: max_depth: 2`** so the actuators expand into base +
   shaft with a genuine slider mate, instead of the synthetic prismatic joint
   the script inserts. Check the actuator subassembly actually *has* a slider
   or cylindrical mate first.
3. **Verify `ACTUATOR_AXIS`** (currently `0 1 0`). Inferred from joint origins,
   not confirmed. A wrong axis is invisible in XML and breaks everything
   downstream.
4. **Set mate limits in Onshape** so the export stops emitting ±360° on
   every pin.
