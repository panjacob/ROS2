# 5. Mechanical design (3D printed arm)

## 5.1 Requirements to define first

Write these down before designing – every other decision depends on them:

| Parameter | Suggested starting target |
|-----------|---------------------------|
| Reach | 350–450 mm |
| Payload | 200–500 g |
| Repeatability | ±1–2 mm (realistic for printed gearboxes + open-loop steppers) |
| DOF | 5 (base, shoulder, elbow, wrist pitch, wrist roll) + gripper |
| Speed | 30–90 °/s per joint |

## 5.2 Inspiration / proven open-source designs

Study before reinventing (check each project's license before reusing parts):

* **BCN3D Moveo** – 5-DOF, NEMA17, belts; classic stepper arm.
* **Thor (AngelLM)** – 6-DOF, printed, GRBL-based – very close to this setup.
* **PAROL6** – 6-DOF desktop arm with printed parts and steppers.
* **AR4 (Annin Robotics)** – stepper arm with encoders, extensive docs.

## 5.3 Torque budget (do it per joint)

Static torque needed at the shoulder with the arm horizontal:

$$
\tau_{shoulder} = g \left( m_{payload} L + \sum_i m_i \, r_i \right) \cdot SF
$$

where $L$ is total reach, $m_i, r_i$ are mass and centre-of-mass distance of each segment,
and $SF \approx 2$ safety factor. Example: 0.3 kg payload at 0.4 m plus ~0.8 kg of arm at
an average 0.2 m → $\tau \approx 9.81(0.12 + 0.16) \cdot 2 \approx 5.5\,\text{Nm}$.

A NEMA17 gives ~0.4 Nm usable (less at speed), so the shoulder needs **≥ 15:1–20:1**
reduction. That is why the default `config.yaml` uses 20:1 (shoulder), 15:1 (elbow),
5:1 (base, wrist). Update the ratios there once your design is fixed.

Also: use **counterbalancing** (spring or counterweight at the shoulder) – it lowers the
required torque and makes power loss less dramatic.

## 5.4 Gearbox options

| Type | Ratio | Backlash | Printability | Notes |
|------|-------|----------|--------------|-------|
| GT2 belt (1–2 stages) | 3–20 | low | easy | Good for base and wrist; needs tensioning |
| Planetary | 4–10 per stage | medium | easy | Compact, stack 2 stages for shoulder |
| Cycloidal | 10–40 | low | medium | Great for shoulder/elbow; needs pins/bearings |
| Strain wave (harmonic) | 30–100 | very low | hard | Advanced; TPU/printed flexspline wears |
| Worm | 20–60 | medium | easy | Self-locking (arm holds pose unpowered), slow, lossy |

Recommended start: **belts for base & wrist, cycloidal or 2-stage planetary for shoulder & elbow.**

## 5.5 Printing guidelines

* Material: **PETG** or **ASA** for structure (PLA creeps under constant load and heat from
  motors). Gears: PETG/PA-CF; flexible parts: TPU.
* 4–6 perimeters, 30–50 % gyroid infill for load-bearing parts.
* Orient so layer lines are **not** loaded in tension along gear teeth and shafts.
* Design for metal: M3/M4 heat-set inserts, steel shafts/pins, real bearings at every joint.
  Do not let printed plastic be a rotating bearing surface under load.
* Big slewing bearing at the base: thin-section bearing (e.g. 6810) or printed race with 6 mm
  steel/airsoft balls.
* Print tolerance test pieces (bearing fits, insert holes, gear mesh) before full parts.

## 5.6 CAD workflow → URDF

1. Model in Fusion 360 / Onshape / FreeCAD. Use **one body per link** and put each link's
   origin **on its joint axis**.
2. Define a "zero pose" (arm straight up is convenient) – this must equal FluidNC 0° on all
   axes and URDF joint value 0.
3. Export tools:
   * Onshape → `onshape-to-robot`
   * Fusion 360 → `fusion2urdf` (community add-in)
   * FreeCAD → *RobotCAD / CROSS* workbench
   * Or manually export STL per link (meters!) and fill `<mesh>` in
     [arm.urdf.xacro](../src/arm_description/urdf/arm.urdf.xacro).
4. Put meshes in [src/arm_description/meshes](../src/arm_description/meshes) and reference them as
   `package://arm_description/meshes/link2.stl`. Use simplified meshes/primitives for collision.
5. Fill realistic masses/inertias (CAD can compute them) – needed for simulation and
   gravity estimates.
6. Verify: `ros2 launch arm_description display.launch.py` – move sliders, check every joint
   turns around the right axis and direction.

## 5.7 Repository convention

```
hardware/
  cad/          # native CAD sources or links
  step/         # STEP exports per part
  stl/          # print-ready STL, named <link>_<part>_v<N>.stl
  bom.md        # parts list with links/prices
  wiring/       # diagrams (KiCad/draw.io/photos)
```

Use Git LFS for STL/STEP files: `git lfs track "*.stl" "*.step"`.
