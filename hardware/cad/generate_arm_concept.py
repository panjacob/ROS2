"""Run inside Blender to create an exploded, URDF-ready arm concept kit.

Each visible mesh is one URDF link. Coordinates and STL vertices are in metres;
the object's origin is the proximal joint, and the next joint lies on local +Z.
"""

from pathlib import Path
from math import pi, radians

import bpy
from mathutils import Euler


ROOT = Path(__file__).resolve().parent
BLEND_PATH = ROOT / 'arm_concept_parts.blend'
EXPORT_DIR = ROOT / 'exports'
SCENE_NAME = 'Arm Concept Kit'

if BLEND_PATH.exists():
    raise FileExistsError(f'Refusing to replace {BLEND_PATH}')
if SCENE_NAME in bpy.data.scenes:
    raise RuntimeError(f'Scene {SCENE_NAME!r} already exists')

EXPORT_DIR.mkdir(parents=True, exist_ok=True)
scene = bpy.data.scenes.new(SCENE_NAME)
bpy.context.window.scene = scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.length_unit = 'METERS'
scene.render.engine = 'BLENDER_EEVEE'

parts_collection = bpy.data.collections.new('01 · URDF links — export these')
scene.collection.children.link(parts_collection)
guides_collection = bpy.data.collections.new('02 · Labels and joint-axis guides')
scene.collection.children.link(guides_collection)


def material(name, rgb, metallic=0.0, roughness=0.35):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*rgb, 1)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*rgb, 1)
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    return mat


shell = material('PETG · graphite', (0.065, 0.083, 0.105), 0.18)
orange = material('PETG · signal orange', (0.95, 0.28, 0.045), 0.08)
metal = material('Aluminium · machined', (0.45, 0.52, 0.57), 0.82, 0.23)
black = material('Rubber · black', (0.02, 0.025, 0.03), 0.0, 0.7)


def in_collection(obj, collection):
    for old in list(obj.users_collection):
        old.objects.unlink(obj)
    collection.objects.link(obj)


def finish(obj, mat, bevel=0.0):
    obj.data.materials.append(mat)
    if bevel:
        modifier = obj.modifiers.new('Machined edge radius', 'BEVEL')
        modifier.width = bevel
        modifier.segments = 3
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=modifier.name)
        modifier = obj.modifiers.new('Weighted normals', 'WEIGHTED_NORMAL')
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    in_collection(obj, parts_collection)
    return obj


def cube(x, xyz, size, mat, bevel=0.002):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x + xyz[0], xyz[1], xyz[2]))
    obj = bpy.context.object
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, mat, bevel)


def cylinder(x, xyz, radius, depth, mat, axis='Z', bevel=0.0015):
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=radius, depth=depth,
                                        location=(x + xyz[0], xyz[1], xyz[2]))
    obj = bpy.context.object
    if axis == 'Y':
        obj.rotation_euler[0] = pi / 2
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    return finish(obj, mat, bevel)


def ring(x, xyz, major, minor, mat, axis='Z'):
    bpy.ops.mesh.primitive_torus_add(major_segments=64, minor_segments=12,
                                    location=(x + xyz[0], xyz[1], xyz[2]),
                                    major_radius=major, minor_radius=minor)
    obj = bpy.context.object
    if axis == 'Y':
        obj.rotation_euler[0] = pi / 2
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    return finish(obj, mat)


def bolt_circle(x, radius, z, count=8):
    from math import cos, sin, tau
    for index in range(count):
        angle = tau * index / count
        cylinder(x, (radius*cos(angle), radius*sin(angle), z),
                 0.0025, 0.002, metal, bevel=0.0003)


def assemble_link(name, x, length, build):
    before = set(parts_collection.objects)
    build(x)
    added = list(set(parts_collection.objects) - before)
    if not added:
        raise RuntimeError(f'No geometry generated for {name}')
    bpy.ops.object.select_all(action='DESELECT')
    for obj in added:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = added[0]
    bpy.ops.object.join()
    obj = bpy.context.object
    obj.name = name
    obj.data.name = f'{name}_mesh'
    scene.cursor.location = (x, 0, 0)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    obj['urdf_link'] = name
    obj['next_joint_z_m'] = length
    obj['coordinate_note'] = 'local +Z runs from this joint to the next; units: metres'
    return obj


def base(x):
    cylinder(x, (0, 0, 0.037), 0.067, 0.074, shell, bevel=0.004)
    cylinder(x, (0, 0, 0.009), 0.071, 0.016, black, bevel=0.002)
    cylinder(x, (0, 0, 0.071), 0.058, 0.014, metal, bevel=0.002)
    ring(x, (0, 0, 0.079), 0.055, 0.002, orange)
    bolt_circle(x, 0.059, 0.079)
    for side in (-1, 1):
        cube(x, (0, side*0.073, 0.006), (0.085, 0.028, 0.012), shell, 0.002)


def turret(x):
    cylinder(x, (0, 0, 0.017), 0.053, 0.034, orange, bevel=0.003)
    cylinder(x, (0, 0, 0.038), 0.039, 0.014, shell)
    for side in (-1, 1):
        cube(x, (0, side*0.033, 0.041), (0.052, 0.013, 0.037), orange)
    cylinder(x, (0, 0, 0.050), 0.022, 0.078, metal, axis='Y')
    for side in (-1, 1):
        cylinder(x, (0, side*0.043, 0.050), 0.013, 0.003, black, axis='Y')


def arm_beam(x, length, width, mat, accent):
    cylinder(x, (0, 0, 0), width*0.56, width*1.5, metal, axis='Y')
    for side in (-1, 1):
        cylinder(x, (0, side*width*0.76, 0), width*0.32, 0.003,
                 black, axis='Y', bevel=0.0005)
    cube(x, (0, 0, length*0.51), (width*0.82, width*0.83, length*0.88),
         mat, bevel=width*0.10)
    for side in (-1, 1):
        cube(x, (0, side*width*0.43, length*0.51),
             (width*0.62, 0.003, length*0.69), metal, bevel=0.001)
        cube(x, (0, side*width*0.49, length*0.42),
             (width*0.38, 0.0018, length*0.11), accent, bevel=0.0005)
    cylinder(x, (0, 0, length), width*0.46, width*1.28, metal, axis='Y')
    for side in (-1, 1):
        cylinder(x, (0, side*width*0.65, length), width*0.3, 0.002,
                 black, axis='Y', bevel=0.0005)


def wrist_pitch(x):
    cylinder(x, (0, 0, 0), 0.027, 0.063, metal, axis='Y')
    for side in (-1, 1):
        cylinder(x, (0, side*0.033, 0), 0.015, 0.004, black, axis='Y')
    cube(x, (0, 0, 0.031), (0.045, 0.045, 0.052), shell, bevel=0.006)
    cylinder(x, (0, 0, 0.049), 0.025, 0.018, orange)
    ring(x, (0, 0, 0.060), 0.023, 0.0018, metal)


def flange(x):
    cylinder(x, (0, 0, 0.006), 0.022, 0.012, metal)
    cylinder(x, (0, 0, 0.019), 0.028, 0.021, orange)
    cylinder(x, (0, 0, 0.030), 0.032, 0.003, metal, bevel=0.0005)
    ring(x, (0, 0, 0.029), 0.023, 0.0015, black)
    bolt_circle(x, 0.024, 0.033, count=6)


specs = [
    ('base_link', 0.00, 0.080, base, 'joint1 · Z'),
    ('link1', 0.23, 0.050, turret, 'joint2 · Y'),
    ('link2', 0.46, 0.200, lambda x: arm_beam(x, 0.200, 0.055, shell, orange), 'joint3 · Y'),
    ('link3', 0.69, 0.180, lambda x: arm_beam(x, 0.180, 0.045, orange, shell), 'joint4 · Y'),
    ('link4', 0.92, 0.060, wrist_pitch, 'joint5 · Z'),
    ('link5', 1.15, 0.030, flange, 'tool0'),
]

links = []
for name, x, length, builder, next_joint in specs:
    obj = assemble_link(name, x, length, builder)
    obj['next_joint'] = next_joint
    links.append(obj)
    bpy.ops.object.text_add(location=(x-0.065, -0.12, 0))
    label = bpy.context.object
    label.name = f'Label · {name}'
    label.data.body = f'{name}\n{round(length*1000)} mm to {next_joint}'
    label.data.size = 0.014
    label.data.extrude = 0
    in_collection(label, guides_collection)
    bpy.ops.object.empty_add(type='ARROWS', location=(x, 0, 0))
    guide = bpy.context.object
    guide.name = f'Origin · {name}'
    guide.empty_display_size = 0.045
    guide.show_in_front = True
    in_collection(guide, guides_collection)

for obj in links:
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    saved_location = obj.location.copy()
    obj.location = (0, 0, 0)
    bpy.context.view_layer.update()
    bpy.ops.wm.stl_export(filepath=str(EXPORT_DIR / f'{obj.name}.stl'),
                          export_selected_objects=True, apply_modifiers=True,
                          use_scene_unit=False, global_scale=1.0)
    obj.location = saved_location

bpy.ops.object.select_all(action='DESELECT')
bpy.context.view_layer.objects.active = links[0]
links[0].select_set(True)
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        space = area.spaces.active
        space.shading.color_type = 'MATERIAL'
        space.overlay.show_overlays = False
        space.region_3d.view_perspective = 'PERSP'
        space.region_3d.view_location = (0.57, 0, 0.07)
        space.region_3d.view_distance = 2.3
        space.region_3d.view_rotation = Euler((radians(65), 0, radians(35)), 'XYZ').to_quaternion()
        area.tag_redraw()
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND_PATH))
print(f'Created {BLEND_PATH} with {len(links)} link meshes and their STL exports.')
