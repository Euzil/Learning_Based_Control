"""Create a fixed-base scene/config without changing the normal simulator."""
from pathlib import Path
import sys
import copy
import xml.etree.ElementTree as ET
import yaml

workspace = Path(__file__).resolve().parents[1]
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=True)
source = workspace / 'src/robots/pm01_description/configs/pm01.yaml'
config = yaml.safe_load(source.read_text())
scene = (source.parent / config['scene']).resolve()
root = ET.parse(scene)
for include in list(root.getroot().findall('include')):
    included = ET.parse(scene.parent / include.get('file')).getroot()
    root.getroot().remove(include)
    root.getroot().extend(list(included))
for compiler in root.iter('compiler'):
    for attr in ('meshdir', 'assetdir'):
        if compiler.get(attr):
            compiler.set(attr, str((scene.parent / compiler.get(attr)).resolve()))
# Original elbow_yaw supplies wrist twist; append simulated neck pitch and wrist flexion.
# Keep all original joint indices; append extra command entries in runtime YAML.
joints = config['robots'][0]['joints']
actuator = root.getroot().find('actuator')
if actuator is None:
    actuator = ET.SubElement(root.getroot(), 'actuator')

def virtual_joint(body, name, axis, limits, stiffness, damping, torque):
    ET.SubElement(body, 'joint', name=name, type='hinge', pos='0 0 0', axis=axis,
                  range=limits, damping='0.05', armature='0.01', frictionloss='0')
    ET.SubElement(actuator, 'motor', name='sim_'+name+'_motor', joint=name,
                  ctrllimited='true', ctrlrange=f'-{torque} {torque}', gear='1')
    joints['names'].append(name)
    joints['default_position'].append(0.)
    joints['default_stiffness'].append(stiffness)
    joints['default_damping'].append(damping)

head = root.getroot().find(".//body[@name='head_yaw_link']")
# Positive rotation about +Y looks down; both head mesh and camera move.
virtual_joint(head, 'sim_head_pitch_joint', '0 1 0', '-0.7854 0.7854', 15., 1.5, 15.)
config['simulated_head_pitch_extension'] = True
# Muted rounded palms attach at the existing hand reference point. They are
# visualization-only geoms; no extra collision response against the forearm.
for side, thumb_sign in (('l', 1), ('r', -1)):
    parent = root.getroot().find(f".//body[@name='elbow_yaw_{side}_link']")
    palm = ET.SubElement(parent, 'body', name=f'sim_palm_{side}', pos='0.03 0 -0.14')
    ET.SubElement(palm, 'inertial', pos='0 0 -0.05', mass='0.02',
                  diaginertia='0.000024 0.000032 0.000012')
    virtual_joint(palm, f'sim_wrist_flex_{side}_joint', '1 0 0',
                  '-1.0472 1.0472', 8., .8, 4.)
    for name, kind, pos, size, rgba in (
        ('cuff', 'ellipsoid', '0 0 0', '0.027 0.023 0.020', '0.29 0.32 0.35 1'),
        ('palm', 'ellipsoid', '0 0 -0.055', '0.035 0.018 0.055', '0.65 0.68 0.70 1'),
        ('back', 'ellipsoid', '0 -0.015 -0.05', '0.026 0.006 0.038', '0.38 0.43 0.47 1'),
        ('thumb', 'ellipsoid', f'{thumb_sign*.034} 0 -0.035',
         '0.013 0.016 0.028', '0.58 0.62 0.65 1')):
        ET.SubElement(palm, 'geom', name=f'sim_{name}_{side}', type=kind,
                      pos=pos, size=size, rgba=rgba, contype='0', conaffinity='0', group='1')
config['simulated_wrist_flexion_extension'] = True

equality = ET.SubElement(root.getroot(), 'equality')
ET.SubElement(equality, 'weld', name='teleop_fixed_base', body1='base_link')
root.write(out / 'scene.xml')
ik_root = copy.deepcopy(root.getroot())
for tag in ('sensor', 'extension'):
    for element in list(ik_root.findall(tag)):
        ik_root.remove(element)
ET.ElementTree(ik_root).write(out / 'ik.xml')
config['ik_scene'] = str(out / 'ik.xml')
config['scene'] = str(out / 'scene.xml')
joints = config['robots'][0]['joints']
# Lower upper-body stiffness and add damping for smooth pose following.
for i, name in enumerate(joints['names']):
    if name.startswith(('shoulder_', 'elbow_')):
        joints['default_stiffness'][i] = 60.0
        joints['default_damping'][i] = 6.0
    elif name == 'head_yaw_joint':
        joints['default_stiffness'][i] = 15.0
        joints['default_damping'][i] = 1.5
joints['command_topic'] = '/simulation/quest/joint_command'
joints['state_topic'] = '/simulation/quest/joint_state'
(out / 'config.yaml').write_text(yaml.safe_dump(config))
