`base_link.stl` and `link1.stl`–`link5.stl` are the metre-scale visual meshes
from `hardware/cad/arm_concept_parts.blend`. Each mesh's coordinates are local
to its link's proximal joint. `arm_concept.urdf.xacro` references these files.

The meshes are conceptual visuals, not watertight print parts. Keep collision
geometry simple until accurate CAD and inertia data are available.
