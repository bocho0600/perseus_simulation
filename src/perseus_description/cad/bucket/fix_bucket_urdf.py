#!/usr/bin/env python3
"""
fix_bucket_urdf.py
==================

Post-processes the raw URDF emitted by `onshape-robotics-toolkit` into something
ROS 2 will actually accept, and extracts the closed-loop joints that URDF cannot
represent so they can be re-applied in Gazebo or MuJoCo.

Why this exists
---------------
The toolkit walks the Onshape mate graph and writes one <joint> per mate. Real
linkages (like this bucket's double 4-bar) contain closed loops, so some links
end up with TWO parents. URDF is strictly a tree: one parent per link. The raw
file therefore fails to parse in ROS.

This script:
  1. Detects the spanning tree and identifies the excess (loop-closing) joints.
  2. Removes those joints from the URDF and records them separately.
  3. Repairs invalid inertials (mass=0, all-zero inertia tensors).
  4. Optionally inserts a prismatic DOF into each linear actuator so the
     mechanism can actually move once loops are re-closed.
  5. Sanitises names (leading digits, hyphens) that break ros2_control / SDF.
  6. Rewrites mesh paths to package:// form.
  7. Wraps everything in a xacro:macro with `parent` and `prefix` params, and
     inserts a `bucket_mounting_frame` interface link.
  8. Emits Gazebo <joint> SDF blocks and MuJoCo <equality> blocks for the loops.

Usage
-----
    python3 fix_bucket_urdf.py bucket.urdf -o generated/ \
        --package perseus_description \
        --add-prismatic

Run with --help for all options.
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from xml.dom import minidom

# --------------------------------------------------------------------------
# CONFIGURATION — edit these for your assembly
# --------------------------------------------------------------------------

# Which joint to DROP from the tree for each multi-parent link.
# Rule of thumb: keep the structurally rigid path, drop the actuator rod-end
# pin. That leaves the actuator hanging off the frame and the bracket riding on
# the arm; the loop constraint then re-pins actuator tip -> bracket.
PREFERRED_LOOP_JOINTS = [
    "Revolute_4",    # 250mm_Actuator_1 -> Actuator_Bracket_2  (left lift rod end)
    "Revolute_6",    # 250mm_Actuator_2 -> Actuator_Bracket_1  (right lift rod end)
    "Revolute_8",    # 250mm_Actuator_3 -> Actuator_Bracket_4  (tilt rod end)
    "Revolute_3",    # Bucket_1 -> Left_Arm_2  (parallel arm closure)
]

# Links whose Onshape mass properties failed. Fill in REAL measured/estimated
# masses in kg. These are placeholders — replace them.
MASS_OVERRIDES_KG = {
    "Frame_Repaired_1":             8.000,
    "Main_Bucket_Mount_Plate_v8_1": 0.450,
    "Main_Bucket_Mount_Plate_v8_2": 0.450,
    "Main_Bucket_Mount_Plate_v8_3": 0.450,
    "Main_Bucket_Mount_Plate_v8_4": 0.450,
    "250mm_Actuator_1":             1.200,
    "250mm_Actuator_2":             1.200,
    "250mm_Actuator_3":             1.200,
    "Left_Arm_-_Square_Tube_1":     1.800,
    "Left_Arm_-_Square_Tube_2":     1.800,
    "Actuator_Bracket_1":           0.120,
    "Actuator_Bracket_2":           0.120,
    "Actuator_Bracket_4":           0.120,
    "Acutator_on_Arm_Mount_1":      0.150,
    "Acutator_on_Arm_Mount_3":      0.150,
    "Bucket_1":                     4.500,
}

# Fallback if a link is not in the table above.
DEFAULT_MASS_KG = 0.500

# Nominal half-extent (m) used to synthesise a plausible diagonal inertia when
# the real tensor is missing. Solid-box approximation: I = m/12 * (b^2 + c^2).
DEFAULT_EXTENT_M = 0.10

# Actuators to split with a prismatic joint, and their stroke in metres.
# Key = link name in the URDF, value = (min_travel, max_travel) in metres.
ACTUATOR_STROKE_M = {
    "250mm_Actuator_1": (0.0, 0.250),
    "250mm_Actuator_2": (0.0, 0.250),
    "250mm_Actuator_3": (0.0, 0.250),
}

# Axis along which each actuator extends, in its own link frame.
# VERIFY THIS against the CAD — a wrong axis is the most common silent bug.
ACTUATOR_AXIS = "0 1 0"

# Realistic actuator effort (N) and velocity (m/s) limits.
ACTUATOR_EFFORT_N = 2000.0
ACTUATOR_VELOCITY_MPS = 0.020

# Realistic pin-joint limits. Pin joints in a linkage are constrained by the
# loop geometry, not by hard stops, so a generous range is fine — but +/-2*pi
# (the toolkit default) lets a viewer wind them up absurdly.
PIN_EFFORT_NM = 500.0
PIN_VELOCITY_RADPS = 2.0
PIN_LOWER_RAD = -3.1415927
PIN_UPPER_RAD = 3.1415927


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def sanitise(name: str) -> str:
    """Make a name safe for URDF/SDF/ros2_control consumption."""
    out = name.replace("-", "_").replace(" ", "_")
    while "__" in out:
        out = out.replace("__", "_")
    out = out.strip("_")
    if out and out[0].isdigit():
        out = "a" + out          # leading digits break SDF + topic names
    return out


def box_inertia(mass: float, extent: float) -> dict:
    """Diagonal inertia for a solid cube of side 2*extent."""
    i = mass * (2 * (2 * extent) ** 2) / 12.0
    return {"ixx": f"{i:.8g}", "iyy": f"{i:.8g}", "izz": f"{i:.8g}",
            "ixy": "0", "ixz": "0", "iyz": "0"}


def pretty(elem: ET.Element) -> str:
    raw = ET.tostring(elem, encoding="unicode")
    dom = minidom.parseString(raw)
    txt = dom.toprettyxml(indent="  ")
    txt = txt.replace("XACRO_BOX_INERTIA", "xacro:box_inertia")
    # strip minidom's blank lines and its xml declaration (we add our own)
    lines = [l for l in txt.split("\n") if l.strip() and not l.startswith("<?xml")]
    return "\n".join(lines)


# --------------------------------------------------------------------------
# Core passes
# --------------------------------------------------------------------------

def find_loop_joints(root: ET.Element, preferred: list[str]) -> tuple[list[str], str]:
    """
    Identify which joints must be removed to leave a spanning tree.

    Returns (loop_joint_names, root_link_name).
    """
    joints = root.findall("joint")
    links = [l.get("name") for l in root.findall("link")]

    parents_of = defaultdict(list)
    for j in joints:
        parents_of[j.find("child").get("link")].append(j.get("name"))

    roots = [l for l in links if l not in parents_of]
    if len(roots) != 1:
        raise SystemExit(f"Expected exactly one root link, found {roots}. "
                         "Check `use_user_defined_root` in ORT.yaml.")
    root_link = roots[0]

    loop_joints = []
    for child, jnames in parents_of.items():
        if len(jnames) == 1:
            continue
        # choose which of the competing joints to demote to a loop constraint
        chosen = next((n for n in jnames if n in preferred), None)
        if chosen is None:
            chosen = jnames[0]
            print(f"  ! no preference for '{child}' -> dropping '{chosen}' "
                  f"(candidates: {jnames}). Add one to PREFERRED_LOOP_JOINTS.")
        loop_joints.append(chosen)

    # Sanity: verify the remainder really is a connected tree
    kept = [j for j in joints if j.get("name") not in loop_joints]
    adj = defaultdict(list)
    for j in kept:
        adj[j.find("parent").get("link")].append(j.find("child").get("link"))
    seen, q = {root_link}, deque([root_link])
    while q:
        for c in adj[q.popleft()]:
            if c not in seen:
                seen.add(c)
                q.append(c)
    missing = set(links) - seen
    if missing:
        raise SystemExit(f"Tree is disconnected after removing loops; "
                         f"unreachable links: {sorted(missing)}")

    return loop_joints, root_link


def repair_inertials(root: ET.Element) -> list[str]:
    """Replace mass<=0 and all-zero inertia tensors with usable values."""
    notes = []
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            ET.SubElement(inertial, "mass")
            ET.SubElement(inertial, "inertia")
            notes.append(f"{name}: had no <inertial> at all; created one")

        mass_el = inertial.find("mass")
        inertia_el = inertial.find("inertia")
        if mass_el is None:
            mass_el = ET.SubElement(inertial, "mass")
        if inertia_el is None:
            inertia_el = ET.SubElement(inertial, "inertia")

        try:
            mass = float(mass_el.get("value", "0"))
        except (TypeError, ValueError):
            mass = 0.0

        tensor = {k: float(inertia_el.get(k, "0") or 0)
                  for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        tensor_is_zero = all(abs(v) < 1e-15 for v in tensor.values())
        tensor_is_unit = (abs(tensor["ixx"] - 1) < 1e-9
                          and abs(tensor["iyy"] - 1) < 1e-9
                          and abs(tensor["izz"] - 1) < 1e-9)

        new_mass = mass
        if mass <= 0.0:
            new_mass = MASS_OVERRIDES_KG.get(name, DEFAULT_MASS_KG)
            notes.append(f"{name}: mass was {mass} -> {new_mass} kg (PLACEHOLDER)")
        elif tensor_is_unit and abs(mass - 1.0) < 1e-9:
            new_mass = MASS_OVERRIDES_KG.get(name, DEFAULT_MASS_KG)
            notes.append(f"{name}: toolkit default mass=1/inertia=1 -> "
                         f"{new_mass} kg (PLACEHOLDER)")

        mass_el.set("value", f"{new_mass:.6g}")

        if tensor_is_zero or tensor_is_unit:
            for k, v in box_inertia(new_mass, DEFAULT_EXTENT_M).items():
                inertia_el.set(k, v)
            # marker consumed by emit_macro to swap in the house box_inertia macro
            inertia_el.set("_synth", f"{2*DEFAULT_EXTENT_M:g}")
            notes.append(f"{name}: inertia tensor synthesised from box "
                         f"approximation (PLACEHOLDER)")
        elif mass <= 0.0:
            # Real tensor but bogus mass (the Bucket_1 case). The tensor was
            # computed at some density; without the mass we cannot rescale it
            # correctly, so flag loudly rather than silently guessing.
            notes.append(f"{name}: !! real inertia tensor but mass was 0. "
                         f"Tensor kept as-is and mass forced to {new_mass} kg. "
                         f"These are INCONSISTENT - fix materials in Onshape.")

    return notes


def _mesh_uri(style: str, package: str, subdir: str, base: str) -> str:
    if style == "file":
        # Perseus house style: file://$(find pkg)/meshes/...
        return f"file://$(find {package})/{subdir}/{base}"
    return f"package://{package}/{subdir}/{base}"


def rewrite_meshes(root: ET.Element, package: str, mesh_subdir: str,
                   style: str = "file") -> None:
    for mesh in root.iter("mesh"):
        base = os.path.basename(mesh.get("filename", ""))
        mesh.set("filename", _mesh_uri(style, package, mesh_subdir, base))


def retarget_collisions(root: ET.Element, mesh_subdir: str, package: str,
                        suffix: str, style: str = "file") -> None:
    """Point <collision> meshes at a simplified variant of each mesh."""
    for link in root.findall("link"):
        for coll in link.findall("collision"):
            for mesh in coll.iter("mesh"):
                fn = os.path.basename(mesh.get("filename", ""))
                stem, ext = os.path.splitext(fn)
                mesh.set("filename",
                         _mesh_uri(style, package, mesh_subdir, f"{stem}{suffix}{ext}"))


def retune_limits(root: ET.Element) -> None:
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        lim = j.find("limit")
        if lim is None:
            lim = ET.SubElement(j, "limit")
        lim.set("effort", f"{PIN_EFFORT_NM:g}")
        lim.set("velocity", f"{PIN_VELOCITY_RADPS:g}")
        lim.set("lower", f"{PIN_LOWER_RAD:g}")
        lim.set("upper", f"{PIN_UPPER_RAD:g}")


def split_actuators(root: ET.Element, loop_joint_els: list[ET.Element]) -> list[str]:
    """
    Insert a prismatic DOF into each actuator.

    The toolkit collapsed each actuator subassembly (base + shaft) into ONE
    rigid link, so the cylinder cannot extend. Without this the mechanism is
    frozen solid the moment loop constraints are applied.

    We add a massless `<name>_piston` link as a prismatic child of the actuator
    body, and re-point the loop-closing joint at the piston instead of at the
    body.
    """
    notes = []
    for act_name, (lo, hi) in ACTUATOR_STROKE_M.items():
        if root.find(f"./link[@name='{act_name}']") is None:
            continue
        piston = f"{act_name}_piston"

        link = ET.SubElement(root, "link", {"name": piston})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "0.05"})
        for k, v in box_inertia(0.05, 0.02).items():
            pass
        ET.SubElement(inertial, "inertia", box_inertia(0.05, 0.02))

        j = ET.SubElement(root, "joint",
                          {"name": f"{act_name}_extend", "type": "prismatic"})
        ET.SubElement(j, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(j, "parent", {"link": act_name})
        ET.SubElement(j, "child", {"link": piston})
        ET.SubElement(j, "axis", {"xyz": ACTUATOR_AXIS})
        ET.SubElement(j, "limit", {"effort": f"{ACTUATOR_EFFORT_N:g}",
                                   "velocity": f"{ACTUATOR_VELOCITY_MPS:g}",
                                   "lower": f"{lo:g}", "upper": f"{hi:g}"})

        # Re-point any loop closure that used to attach to the actuator body
        for lj in loop_joint_els:
            if lj.find("parent").get("link") == act_name:
                lj.find("parent").set("link", piston)
                notes.append(f"loop joint '{lj.get('name')}' re-parented "
                             f"{act_name} -> {piston}")

        notes.append(f"{act_name}: added prismatic '{act_name}_extend' "
                     f"({lo}..{hi} m, axis {ACTUATOR_AXIS}) + link '{piston}'")
    return notes


def apply_prefix(root: ET.Element, do_sanitise: bool) -> None:
    """Prefix every link/joint name with ${prefix} and optionally sanitise."""
    def fix(n: str) -> str:
        return "${prefix}" + (sanitise(n) if do_sanitise else n)

    for link in root.findall("link"):
        link.set("name", fix(link.get("name")))
    for j in root.findall("joint"):
        j.set("name", fix(j.get("name")))
        j.find("parent").set("link", fix(j.find("parent").get("link")))
        j.find("child").set("link", fix(j.find("child").get("link")))
    # material names must be globally unique too
    for m in root.iter("material"):
        if m.get("name"):
            m.set("name", fix(m.get("name")))


# --------------------------------------------------------------------------
# Emitters
# --------------------------------------------------------------------------

def _swap_inertia_macros(root: ET.Element) -> None:
    """Replace synthesised literal tensors with the repo's box_inertia macro."""
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        inertia_el = inertial.find("inertia")
        if inertia_el is None or inertia_el.get("_synth") is None:
            continue
        side = inertia_el.get("_synth")
        mass = inertial.find("mass").get("value")
        inertial.remove(inertia_el)
        ET.SubElement(inertial, "XACRO_BOX_INERTIA",
                      {"m": mass, "x": side, "y": side, "z": side})


def emit_macro(root: ET.Element, root_link: str, macro_name: str,
               do_sanitise: bool, prefix_mode: str = "inherit",
               inertia_macros: bool = True, parent_default: str = "chassis",
               package: str = "perseus_description") -> str:
    rl = ("${prefix}" + (sanitise(root_link) if do_sanitise else root_link))

    if inertia_macros:
        _swap_inertia_macros(root)

    body = "\n".join("    " + l for l in pretty(root).split("\n")[1:-1])
    prefix_decl = "prefix:=^|''" if prefix_mode == "inherit" else "prefix:=''"

    return f'''<?xml version="1.0"?>
<!--
  {macro_name}.urdf.xacro  -  AUTOGENERATED by fix_bucket_urdf.py.

  Do not hand-edit: change the Onshape CAD or the script config and regenerate.

  Excavation bucket: a symmetric double four-bar linkage. Two square-tube arms
  pivot on the mount plates and carry the bucket; two 250mm actuators drive the
  lift, a third drives the tilt.

  URDF cannot express closed loops, so four loop-closing joints were removed to
  leave a spanning tree. Include {macro_name}_loops.urdf.xacro as well if you
  need correct physics in Gazebo.

  Usage, inside the perseus macro in perseus.urdf.xacro:

      <xacro:include filename="$(find {package})/urdf/{macro_name}.urdf.xacro"/>
      <xacro:if value="$(arg use_bucket)">
        <xacro:{macro_name} parent="${{prefix}}{parent_default}"/>
      </xacro:if>
-->
<robot name="{macro_name}" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Where the mounting frame sits relative to the chassis. MEASURE THESE. -->
  <xacro:property name="bucket_mount_x" value="0.0"/>
  <xacro:property name="bucket_mount_y" value="0.0"/>
  <xacro:property name="bucket_mount_z" value="0.0"/>

  <xacro:macro name="{macro_name}" params="
      parent:=^|'${{prefix}}{parent_default}'
      {prefix_decl}
      mount_xyz:='${{bucket_mount_x}} ${{bucket_mount_y}} ${{bucket_mount_z}}'
      mount_rpy:='0 0 0'">

    <!-- ================================================================
         Interface link. Everything downstream hangs off this, so the rest
         of the robot only ever needs to know this one name.
         ================================================================ -->
    <link name="${{prefix}}bucket_mounting_frame"/>

    <joint name="${{prefix}}bucket_mount_joint" type="fixed">
      <parent link="${{parent}}"/>
      <child  link="${{prefix}}bucket_mounting_frame"/>
      <origin xyz="${{mount_xyz}}" rpy="${{mount_rpy}}"/>
    </joint>

    <!-- CAD root of the exported assembly, welded to the interface link -->
    <joint name="${{prefix}}bucket_frame_joint" type="fixed">
      <parent link="${{prefix}}bucket_mounting_frame"/>
      <child  link="{rl}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

{body}

  </xacro:macro>
</robot>
'''


def emit_gazebo_loops(loop_els: list[ET.Element], macro_name: str,
                      do_sanitise: bool) -> str:
    def fix(n):
        return "${prefix}" + (sanitise(n) if do_sanitise else n)

    blocks = []
    for j in loop_els:
        o = j.find("origin")
        xyz = o.get("xyz", "0 0 0") if o is not None else "0 0 0"
        rpy = o.get("rpy", "0 0 0") if o is not None else "0 0 0"
        ax = j.find("axis")
        axis = ax.get("xyz", "0 0 1") if ax is not None else "0 0 1"
        blocks.append(f'''      <!-- loop closure: {j.find('parent').get('link')}
                        -> {j.find('child').get('link')} -->
      <joint name="{fix(j.get('name'))}_loop" type="revolute">
        <parent>{fix(j.find('parent').get('link'))}</parent>
        <child>{fix(j.find('child').get('link'))}</child>
        <pose>{xyz} {rpy}</pose>
        <axis>
          <xyz>{axis}</xyz>
          <limit><lower>-3.1416</lower><upper>3.1416</upper></limit>
          <dynamics><damping>0.1</damping></dynamics>
        </axis>
      </joint>''')

    joined = "\n\n".join(blocks)
    return f'''<?xml version="1.0"?>
<!--
  {macro_name}_loops.urdf.xacro  -  AUTOGENERATED.

  URDF is a tree and cannot express the closed loops in this linkage. These
  joints were removed from the tree; Gazebo re-applies them here as raw SDF.
  The <gazebo> element is passed through untouched by the URDF->SDF converter.

  IMPORTANT: loop closures need `preserveFixedJoint` behaviour on the links
  they touch, otherwise the SDF converter may weld those links away before
  these joints are applied. Verify with:
      gz sdf -p robot.urdf | grep -A5 _loop
-->
<robot name="{macro_name}_loops" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="{macro_name}_loops" params="prefix:=^|''">
    <gazebo>
{joined}
    </gazebo>
  </xacro:macro>
</robot>
'''


def emit_mujoco_equality(loop_els: list[ET.Element], do_sanitise: bool) -> str:
    def fix(n):
        return sanitise(n) if do_sanitise else n

    rows = []
    for j in loop_els:
        o = j.find("origin")
        xyz = o.get("xyz", "0 0 0") if o is not None else "0 0 0"
        rows.append(
            f'    <!-- {j.get("name")} -->\n'
            f'    <connect name="{fix(j.get("name"))}_loop"\n'
            f'             body1="{fix(j.find("parent").get("link"))}"\n'
            f'             body2="{fix(j.find("child").get("link"))}"\n'
            f'             anchor="{xyz}"\n'
            f'             solref="0.005 1" solimp="0.95 0.99 0.001"/>')

    body = "\n".join(rows)
    return f'''<!--
  MuJoCo loop closures. Paste this <equality> block into your MJCF, as a direct
  child of <mujoco>, AFTER <worldbody>.

  <connect> pins two bodies together at a point without them being parent/child
  in the kinematic tree - which is exactly what a 4-bar linkage needs and what
  URDF structurally cannot express.

  `anchor` is expressed in body1's local frame. The values below are copied from
  the URDF joint origins; if a loop visibly tears open in sim, that anchor is
  in the wrong frame and needs recomputing.
-->
<equality>
{body}
</equality>
'''


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf", help="raw URDF from onshape-robotics-toolkit")
    ap.add_argument("-o", "--outdir", default="generated")
    ap.add_argument("--package", default="perseus_description",
                    help="ROS package that will host the meshes")
    ap.add_argument("--mesh-subdir", default="meshes/bucket")
    ap.add_argument("--macro-name", default="bucket")
    ap.add_argument("--parent-default", default="chassis",
                    help="Perseus chassis link is named 'chassis', not 'chassis_link'")
    ap.add_argument("--no-sanitise", action="store_true",
                    help="keep original link names (leading digits, hyphens)")
    ap.add_argument("--add-prismatic", action="store_true",
                    help="insert prismatic DOF into each linear actuator")
    ap.add_argument("--collision-suffix", default="",
                    help="e.g. '_collision' to point <collision> at simplified meshes")
    ap.add_argument("--mesh-uri", choices=("package", "file"), default="file",
                    help="mesh URI style. Perseus uses 'file'.")
    ap.add_argument("--prefix-mode", choices=("param", "inherit"), default="inherit",
                    help="'inherit' emits prefix:=^|'' to match Perseus sub-macros")
    ap.add_argument("--inertia-macros", action="store_true", default=True,
                    help="emit xacro:box_inertia instead of literal tensors")
    ap.add_argument("--no-inertia-macros", dest="inertia_macros",
                    action="store_false")
    args = ap.parse_args()

    do_sanitise = not args.no_sanitise
    os.makedirs(args.outdir, exist_ok=True)

    tree = ET.parse(args.urdf)
    root = tree.getroot()
    report = []

    print("1. Finding closed loops...")
    loop_names, root_link = find_loop_joints(root, PREFERRED_LOOP_JOINTS)
    print(f"   root link: {root_link}")
    print(f"   loop-closing joints: {loop_names}")
    report.append(f"Root link: `{root_link}`")
    report.append(f"Loop-closing joints removed from tree: "
                  + ", ".join(f"`{n}`" for n in loop_names))

    loop_els = []
    for n in loop_names:
        el = root.find(f"./joint[@name='{n}']")
        loop_els.append(copy.deepcopy(el))
        root.remove(el)

    print("2. Repairing inertials...")
    notes = repair_inertials(root)
    for n in notes:
        print("   " + n)
    report.append("\n### Inertial repairs\n"
                  + "\n".join(f"- {n}" for n in notes))

    print("3. Retuning joint limits...")
    retune_limits(root)

    prismatic_notes = []
    if args.add_prismatic:
        print("4. Splitting actuators with prismatic DOF...")
        prismatic_notes = split_actuators(root, loop_els)
        for n in prismatic_notes:
            print("   " + n)
        report.append("\n### Actuator prismatic DOFs\n"
                      + "\n".join(f"- {n}" for n in prismatic_notes))
    else:
        report.append("\n### Actuator prismatic DOFs\n- NOT APPLIED. The "
                      "mechanism will be frozen once loops are closed. "
                      "Re-run with --add-prismatic, or better, re-export "
                      "from Onshape with `cad: max_depth: 2`.")

    print("5. Rewriting mesh paths...")
    rewrite_meshes(root, args.package, args.mesh_subdir, args.mesh_uri)
    if args.collision_suffix:
        retarget_collisions(root, args.mesh_subdir, args.package,
                            args.collision_suffix, args.mesh_uri)

    print("6. Applying ${prefix} and sanitising names...")
    apply_prefix(root, do_sanitise)

    # strip the <robot name=...> wrapper attrs; the macro supplies context
    for k in list(root.attrib):
        del root.attrib[k]

    macro_path = os.path.join(args.outdir, f"{args.macro_name}.urdf.xacro")
    loops_path = os.path.join(args.outdir, f"{args.macro_name}_loops.urdf.xacro")
    mjcf_path = os.path.join(args.outdir, f"{args.macro_name}_equality.mjcf.xml")
    report_path = os.path.join(args.outdir, "CONVERSION_REPORT.md")

    with open(macro_path, "w") as f:
        f.write(emit_macro(root, root_link, args.macro_name, do_sanitise,
                           args.prefix_mode, args.inertia_macros,
                           args.parent_default, args.package))
    with open(loops_path, "w") as f:
        f.write(emit_gazebo_loops(loop_els, args.macro_name, do_sanitise))
    with open(mjcf_path, "w") as f:
        f.write(emit_mujoco_equality(loop_els, do_sanitise))
    with open(report_path, "w") as f:
        f.write("# Bucket URDF conversion report\n\n"
                "Autogenerated by `fix_bucket_urdf.py`. Every item marked "
                "PLACEHOLDER is a guess and must be replaced with a real "
                "value before the simulation means anything.\n\n"
                + "\n".join(report) + "\n")

    print(f"\nWrote:\n  {macro_path}\n  {loops_path}\n  {mjcf_path}\n  {report_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
