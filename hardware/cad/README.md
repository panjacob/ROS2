# Arm concept kit

Open `arm_concept_parts.blend` in Blender. Select the **Arm Concept Kit** scene
if Blender opens the original default scene. The six editable mesh objects are
in `01 · URDF links — export these`; labels and origin guides are in the other
collection. The link meshes are laid out along X so each can be inspected and
assembled independently. Units are metres.

| Blender object / STL | Proximal joint at local origin | Next joint on local +Z |
|---|---|---|
| `base_link` | world/base | `joint1` at 0.080 m, axis Z |
| `link1` | `joint1` | `joint2` at 0.050 m, axis Y |
| `link2` | `joint2` | `joint3` at 0.200 m, axis Y |
| `link3` | `joint3` | `joint4` at 0.180 m, axis Y |
| `link4` | `joint4` | `joint5` at 0.060 m, axis Z |
| `link5` | `joint5` | `tool0` at 0.030 m |

To make a zero-pose assembly in Blender, move the origins of the six objects
to Z = 0, 0.080, 0.130, 0.330, 0.510 and 0.570 m respectively, all at X = Y = 0.
The STL files in `exports/` have **local** coordinates around each link's
proximal joint, so the exploded positions are not baked into the exports.

The exports are already copied into `src/arm_description/meshes/` and referenced
by `src/arm_description/urdf/arm_concept.urdf.xacro`. That model uses the same
joint origins as the distances above. If you modify and re-export a link, copy
its new STL to the description package and rebuild the workspace. The present
STLs are concept visuals with overlapping decorative shells; they are not
watertight print parts.

`src/arm_description/urdf/arm_concept.urdf` is the expanded, plain URDF. If you
edit the Xacro source, regenerate it with
`xacro src/arm_description/urdf/arm_concept.urdf.xacro > src/arm_description/urdf/arm_concept.urdf`.

`generate_arm_concept.py` contains the Blender source for the kit. It refuses
to overwrite an existing `.blend` file or scene, protecting later manual edits.
