import os
import sys
import xml.etree.ElementTree as ET

# find urdf file
urdf_file = os.path.join(os.path.dirname(__file__), 'urdf', 'robot.urdf')

if not os.path.exists(urdf_file):
    print(f"FAIL: File not found: {urdf_file}")
    sys.exit(1)

try:
    tree = ET.parse(urdf_file)
    root = tree.getroot()
except Exception as e:
    print(f"FAIL: XML parse error: {e}")
    sys.exit(1)

# get links and joints
links = [link.attrib['name'] for link in root.findall('link')]
joints = root.findall('joint')

print(f"Total links: {len(links)}")
print(f"Total joints: {len(joints)}")

link_set = set(links)
movable_joints = []
all_joints_valid = True
has_tool0 = "tool0" in link_set

for j in joints:
    name = j.attrib.get('name')
    jtype = j.attrib.get('type')
    parent = j.find('parent').attrib.get('link') if j.find('parent') is not None else None
    child = j.find('child').attrib.get('link') if j.find('child') is not None else None

    print(f" - Joint: {name} | Type: {jtype} | Parent: {parent} -> Child: {child}")

    if parent not in link_set or child not in link_set:
        print(f"   ERROR: Joint {name} links missing: parent={parent}, child={child}")
        all_joints_valid = False

    if jtype in ['revolute', 'continuous', 'prismatic']:
        movable_joints.append(name)

print(f"Movable joints count: {len(movable_joints)}")
print(f"Has tool0 frame: {has_tool0}")

# checks
if all_joints_valid and len(movable_joints) == 6 and has_tool0:
    print("PASS: 6-DOF Industrial Arm verification passed!")
    sys.exit(0)
else:
    print("FAIL: Verification failed!")
    sys.exit(1)
