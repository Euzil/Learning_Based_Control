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
