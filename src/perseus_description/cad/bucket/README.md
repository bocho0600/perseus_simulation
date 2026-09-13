# Bucket CAD export — reference only

Nothing in here is loaded by the robot description. It is a copy of the Onshape
export and the notes that came with it, kept as the dimensional source of truth.
The description actually used is
[`urdf/bucket.urdf.xacro`](../../urdf/bucket.urdf.xacro).

Regenerate with `~/Documents/projects/onshape-urdf/to_urdf_test.py`, which reads
the live document
`qutrc.onshape.com/documents/6a8e1bd7c7e79e401b258e58` ("Rover_True_Bucket_-_URDF_-_Temp")
through `onshape_robotics_toolkit`. Note that merely *importing* that toolkit
rewrites `ORT.yaml` in the current working directory — run it from its own
directory, or you will truncate the config.

| File | What it is |
|---|---|
| `bucket.urdf.xacro` | the export, post-processed by `fix_bucket_urdf.py` |
| `bucket_loops.urdf.xacro` | the four loop-closing joints, as raw SDF |
| `fix_bucket_urdf.py` | the post-processing script and the config it ran with |
| `CONVERSION_REPORT.md` | what the script had to guess |
| `INTEGRATION_EXAMPLE.md` | the integration its author proposed |
| `bucket-image.png` | render of the assembly |

The 16 STL meshes from `output/meshes/` are **not** duplicated here — they were
copied straight into `perseus_description/meshes/bucket/`, which is where the
description loads them from.

## Why the export is not used directly

**The inertials are not recoverable.** Onshape's own mass-properties endpoint
answers `"hasMass": false, "massMissingCount": 53` — not one of the 53 parts has
a material assigned. That is why every mass came through as 0 or as the
toolkit's default of 1. Worse, the centres of mass are in *assembly*
coordinates rather than link coordinates: the 0.7 m arm has its CoM at
`x = 0.945`. A placeholder mass can be corrected later; a CoM a metre outside
its own link cannot.

**The loop closures would not have survived SDF conversion.** The four joints in
`bucket_loops.urdf.xacro` reference `Actuator_Bracket_*`, which hang off *fixed*
joints and get welded into their parents before the `<gazebo>` block is applied.
`gz sdf -p` drops them and the linkage collapses.

**The exported pose is not an assembly configuration.** The toolkit places each
link by its own mate transform and never solves the loops, so what you get is
the mate zero of every joint at once. Concretely: the lift actuator's pins end
up 0.640 m apart, but the part is drawn at 0.365 m and only has 0.250 m of
stroke. This is why the render shows the actuators detached from their brackets.
It is not a CAD error.

**The mount pose was unknown.** `bucket_mount_{x,y,z}` were all `0.0`, marked
"MEASURE THESE", which would bury the attachment inside the chassis.

## What the CAD does give you, exactly

Two things in the export are trustworthy: the **mate transforms** (rigid-body
facts, unaffected by the mass-property problems) and the **meshes**. Everything
in `bucket.urdf.xacro` is measured from those.

Forward-kinematicking the export into one frame gives the linkage:

| Quantity | Value | Source |
|---|---|---|
| arm pivot → bucket pivot | 0.70004 m | `Revolute_2` |
| arm / bearing-block lateral spacing | 0.200 m | block origins, confirmed by `Revolute_3`'s 0.202 m pose |
| arm pivot above lift-actuator pivot | 0.240 m | upper vs. lower block z |
| lift bracket along the arm | 0.59712 m | `Fastened_2` |
| bucket pivot lateral offset | −0.102 m | the bucket hangs off **one** arm, not both |
| tilt pin on bucket, from bucket pivot | 0.13106 m | `Revolute_9` |
| tilt bracket on arm, from bucket pivot | 0.51349 m | `Fastened_4` |
| actuator pin-to-pin as drawn | 0.365 m | `Revolute_4` pose in the actuator frame |
| actuator stroke | 0.250 m | part name |

and measuring the meshes gives the sizes:

| Part | Measured | Volume |
|---|---|---|
| Frame | 260 × 410 × 300 mm gantry of 20×20 V-slot, two 243 mm rails 390 mm apart at z 0.018–0.038 | 518.1 cm³ |
| Arm tube | 750 mm of 40 × 20 × 2 mm tube (168.5 cm³ is exactly a 2 mm wall) | 168.5 cm³ |
| Bearing block | 110 × 60 × 60 mm | 159.0 cm³ |
| 250 mm actuator | 76 × 382 × 40 mm | 404.9 cm³ |
| Bucket (clamshell) | 236 × 256 × 459 mm | 1297.5 cm³ |

Summed, the meshes come to 4118.4 cm³ against Onshape's 4138.7 cm³ for the
assembly — a 0.5% tessellation difference, which is the cross-check that the
mesh integration is sound.

Two of those numbers pin down the joint travel by themselves. The lift actuator
pivots 0.240 m below the arm pivot and pushes a bracket 0.597 m along it, so its
length is `L² = r² + d² + 2·r·d·sin(a)`; between 0.365 m and 0.615 m that is
71.36° of arm swing, ending 6.24° below the frame's own axis. The tilt actuator
works between 0.131 m and 0.513 m radii, giving 108.2° at the CAD pose and a
geometric fold at 0°.

## Things worth knowing about this CAD

- **Both arms are the same part.** "Left Arm - Square Tube" `<1>` and `<2>` —
  there is no right-hand mirror, so both are offset the same way from their
  pivots rather than symmetrically. The description reproduces that.
- **The bucket is a clamshell.** `Bucket_1` contains "Bucket FRONT Frame",
  "Bucket REAR Frame", a `BucketClipVslotMount` and **two 50 mm actuators** —
  the `JAWS_BOTH` group that `bucket_driver` in the `payloads` package writes
  to. `cad.max_depth: 0` flattens all of it into one rigid body.
- **The bucket sits ~20 mm off the arm centreline.** Its material spans
  y −0.108…0.352 about a pivot pair at y 0 and 0.202.
- **The tilt ram's parent is back to front in the export.** `Revolute_9` makes
  `Bucket_1` the parent of `250mm_Actuator_3`, but on the machine the ram's
  barrel is bolted to the plate on the arms (`Acutator_on_Arm_Mount` →
  `Actuator_Bracket_4`) and its rod reaches forward to the bucket. The toolkit
  roots each mate at whichever body it walked to first, which is not
  necessarily the fixed end. Pin positions are identical either way, so this
  only matters if you draw the ram — get it wrong and the barrel telescopes
  while the rod stays put. `urdf/bucket.urdf.xacro` anchors it to the arm.
- **Ram pivots and the brackets they drive are ~10 mm apart laterally.** The
  rod-end bearings absorb that on the real machine; a revolute joint cannot, so
  the description moves the skew to the base pin where it is hidden inside the
  bearing block.

## To improve the model

1. **Assign a material to every part in Onshape**, then re-export. That fixes
   every mass and inertia at once and is the only thing that will. Until then
   the description multiplies the exact CAD volumes by an assumed 2700 kg/m³.
2. **Re-export with `cad: max_depth: 2`.** The actuators already decompose into
   `Base Actuator v6` + `Shaft v7` with a real slider mate, and the bucket into
   its two clamshell halves. That would give genuine prismatic joints and the
   jaws DOF instead of lumped rigid bodies.
3. **Measure how the frame actually bolts on** and set `bucket_mount_{x,y,z}`.
   The current values put the rails flat on the chassis deck, which is what the
   rail geometry implies but has not been confirmed against the real rover.
4. **Set the mate limits in Onshape** so the export stops emitting ±360° on
   every pin.
