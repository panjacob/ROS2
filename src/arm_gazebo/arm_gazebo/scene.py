"""One geometry source for Gazebo and MoveIt; no online model downloads."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load_scene(path):
    with open(path, encoding='utf-8') as stream:
        scene = yaml.safe_load(stream)
    for section, key in [('table', 'center'), ('table', 'size'), ('banana', 'position')]:
        values = scene[section][key]
        if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f'{section}.{key} must contain three finite values')
    for section, key in [('table', 'height'), ('banana', 'half_length'),
                         ('banana', 'radius'), ('approach', 'clearance')]:
        if not math.isfinite(scene[section][key]) or scene[section][key] <= 0:
            raise ValueError(f'{section}.{key} must be positive and finite')
    if any(v <= 0 for v in scene['table']['size']):
        raise ValueError('table.size must be positive')
    if scene['table']['height'] <= scene['table']['size'][2]:
        raise ValueError('table.height must exceed the tabletop thickness')
    if not math.isfinite(scene['banana']['bend']):
        raise ValueError('banana.bend must be finite')
    segments = scene['banana']['segments']
    if not isinstance(segments, int) or not 5 <= segments <= 100:
        raise ValueError('banana.segments must be an integer from 5 to 100')
    return scene


def banana_sections(scene):
    banana = scene['banana']
    for i in range(banana['segments']):
        u = i / (banana['segments'] - 1)
        t = 2 * u - 1
        radius = banana['radius'] * (0.25 + 0.75 * math.sqrt(max(0, math.sin(math.pi*u))))
        yield (banana['half_length'] * t, banana['bend'] * (1 - t*t), 0.0), radius


def collision_shapes(scene):
    """(object id, primitive kind, dimensions, world position) tuples."""
    table = scene['table']
    yield 'table', 'box', table['size'], table['center']
    top = table['center'][2] + table['size'][2] / 2
    floor = top - table['height']
    yield 'floor', 'box', [10.0, 10.0, 0.1], [0.0, 0.0, floor - 0.05]
    for x in (-1, 1):
        for y in (-1, 1):
            yield 'table', 'box', [0.045, 0.045, table['height'] - table['size'][2]], [
                table['center'][0] + x * (table['size'][0]/2 - 0.05),
                table['center'][1] + y * (table['size'][1]/2 - 0.05),
                floor + (table['height'] - table['size'][2])/2,
            ]
    for center, radius in banana_sections(scene):
        yield 'banana', 'sphere', [radius], [
            a + b for a, b in zip(scene['banana']['position'], center)]


def approach_target(scene):
    banana = scene['banana']['position']
    highest = max(center[2] + radius for center, radius in banana_sections(scene))
    return [banana[0], banana[1] + scene['banana']['bend'],
            banana[2] + highest + scene['approach']['clearance']]


def _element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def write_world(scene, destination):
    """Build a self-contained SDF world, including the banana's visual geometry."""
    destination = Path(destination)
    sdf = ET.Element('sdf', version='1.9')
    world = _element(sdf, 'world', name='tabletop')
    _add_gui(world)
    physics = _element(world, 'physics', name='physics', type='ignored')
    _element(physics, 'max_step_size', 0.001)
    _element(physics, 'real_time_factor', 1.0)
    _element(world, 'gravity', '0 0 -9.81')
    for filename, name in [
        ('gz-sim-physics-system', 'Physics'),
        ('gz-sim-user-commands-system', 'UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'SceneBroadcaster'),
    ]:
        _element(world, 'plugin', filename=filename, name=f'gz::sim::systems::{name}')
    light = _element(world, 'light', name='sun', type='directional')
    _element(light, 'pose', '0 0 4 0 0 0')
    _element(light, 'diffuse', '0.9 0.9 0.9 1')
    _element(light, 'direction', '-0.5 0.2 -1')
    _element(light, 'cast_shadows', 'true')
    links = {}
    colors = {'table': '0.46 0.29 0.14 1', 'floor': '0.55 0.58 0.62 1',
              'banana': '0.98 0.78 0.05 1'}
    for i, (name, kind, dimensions, position) in enumerate(collision_shapes(scene)):
        if name not in links:
            model = _element(world, 'model', name=name)
            _element(model, 'static', 'true')
            links[name] = _element(model, 'link', name='body')
        link = links[name]
        for role in ['collision', 'visual']:
            shape = _element(link, role, name=f'{role}_{i}')
            _element(shape, 'pose', ' '.join(map(str, [*position, 0, 0, 0])))
            geometry = _element(shape, 'geometry')
            primitive = _element(geometry, kind)
            _element(primitive, 'radius' if kind == 'sphere' else 'size',
                     dimensions[0] if kind == 'sphere' else ' '.join(map(str, dimensions)))
            if role == 'visual':
                material = _element(shape, 'material')
                _element(material, 'ambient', colors[name])
                _element(material, 'diffuse', colors[name])
    # Join the tapered spheres with cylinders: a curved banana without external
    # mesh files, loader-specific materials or resources removed during shutdown.
    sections = list(banana_sections(scene))
    for i, ((start, r1), (end, r2)) in enumerate(zip(sections, sections[1:])):
        visual = _element(links['banana'], 'visual', name=f'banana_segment_{i}')
        center = [origin + (a+b)/2 for origin, a, b in zip(
            scene['banana']['position'], start, end)]
        yaw = math.atan2(end[1]-start[1], end[0]-start[0])
        _element(visual, 'pose', ' '.join(map(str, [*center, 0, math.pi/2, yaw])))
        cylinder = _element(_element(visual, 'geometry'), 'cylinder')
        _element(cylinder, 'radius', min(r1, r2))
        _element(cylinder, 'length', math.dist(start, end))
        material = _element(visual, 'material')
        _element(material, 'ambient', colors['banana'])
        _element(material, 'diffuse', colors['banana'])
    ET.indent(sdf)
    ET.ElementTree(sdf).write(destination, encoding='unicode', xml_declaration=True)
    return destination


def _add_gui(world):
    gui = _element(world, 'gui', fullscreen='false')
    view = _element(gui, 'plugin', filename='MinimalScene', name='3D View')
    settings = _element(view, 'gz-gui')
    _element(settings, 'property', 'docked', type='string', key='state')
    _element(view, 'engine', 'ogre2')
    _element(view, 'scene', 'scene')
    _element(view, 'ambient_light', '0.5 0.5 0.5')
    _element(view, 'background_color', '0.8 0.8 0.8')
    _element(view, 'camera_pose', '1.1 -1.3 0.85 0 0.4 2.18')
    for filename in ['GzSceneManager', 'InteractiveViewControl', 'CameraTracking']:
        plugin = _element(gui, 'plugin', filename=filename, name=filename)
        settings = _element(plugin, 'gz-gui')
        for key, value, kind in [('state', 'floating', 'string'),
                                  ('showTitleBar', 'false', 'bool'),
                                  ('width', '5', 'double'), ('height', '5', 'double')]:
            _element(settings, 'property', value, type=kind, key=key)
    control = _element(gui, 'plugin', filename='WorldControl', name='World control')
    settings = _element(control, 'gz-gui')
    _element(settings, 'property', 'floating', type='string', key='state')
    _element(settings, 'property', 'false', type='bool', key='showTitleBar')
    _element(settings, 'property', '72', type='double', key='height')
    anchors = _element(settings, 'anchors', target='3D View')
    _element(anchors, 'line', own='left', target='left')
    _element(anchors, 'line', own='bottom', target='bottom')
    for key in ['play_pause', 'step', 'use_event']:
        _element(control, key, 'true')
    _element(control, 'start_paused', 'false')
