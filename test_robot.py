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
arm_joints = []
gripper_joints = []
all_joints_valid = True
has_tool0 = "tool0" in link_set
has_tool_link = "tool_link" in link_set
has_gripper_base = "gripper_base_link" in link_set

for j in joints:
    name = j.attrib.get('name')
    jtype = j.attrib.get('type')
    parent = j.find('parent').attrib.get('link') if j.find('parent') is not None else None
    child = j.find('child').attrib.get('link') if j.find('child') is not None else None
    axis = j.find('axis').attrib.get('xyz') if j.find('axis') is not None else "fixed"

    print(f" - Joint: {name:<18} | Type: {jtype:<10} | Axis: {axis:<10} | Parent: {parent:<18} -> Child: {child}")

    if parent not in link_set or child not in link_set:
        print(f"   ERROR: Joint {name} links missing: parent={parent}, child={child}")
        all_joints_valid = False

    if jtype in ['revolute', 'continuous']:
        arm_joints.append(name)
    elif jtype == 'prismatic':
        gripper_joints.append(name)

print(f"\nArm revolute joints ({len(arm_joints)}): {arm_joints}")
print(f"Gripper prismatic joints ({len(gripper_joints)}): {gripper_joints}")
print(f"Total movable joints: {len(arm_joints) + len(gripper_joints)}")
print(f"Flange frame (tool0) present: {has_tool0}")
print(f"Gripper base present: {has_gripper_base}")
print(f"TCP frame (tool_link) present: {has_tool_link}")

# checks
if (all_joints_valid and len(arm_joints) == 6 and len(gripper_joints) == 2 
    and has_tool0 and has_gripper_base and has_tool_link):
    print("\nPASS: 6-DOF Industrial Arm + Parallel Gripper verification passed successfully!")
    sys.exit(0)
else:
    print("\nFAIL: Verification failed!")
    sys.exit(1)
