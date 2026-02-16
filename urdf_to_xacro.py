#!/usr/bin/env python3
# https://docs.python.org/3/library/xml.etree.elementtree.html
from unittest import case
import xml.etree.ElementTree as ET

# Register namespace to prevent 'ns0' prefixes in output
ET.register_namespace('xacro', 'http://www.ros.org/wiki/xacro')

# Load the template xacro file
template_file_path = "/Users/nathanlodder/Documents/Academics/ACA-UBC/UBC-Classes/UBC-Courses-25_26/ACA-UBC-ENPH353/ENPH353-Comp/enph353-comp-drone/drone_xacro_template/drone_template.xacro"
tree_template = ET.parse(template_file_path)
root_template = tree_template.getroot()

# import file data from onshape-to-drone call
raw_file_path = "/Users/nathanlodder/Documents/Academics/ACA-UBC/UBC-Classes/UBC-Courses-25_26/ACA-UBC-ENPH353/ENPH353-Comp/enph353-comp-drone/onshape-to-urdf-raw/drone.urdf"
tree_raw = ET.parse(raw_file_path)
root_raw = tree_raw.getroot()

# This safely converts all names and link references to lowercase
for elem in root_raw.iter():
    if 'name' in elem.attrib:
        elem.attrib['name'] = elem.attrib['name'].lower()
    if 'link' in elem.attrib:
        elem.attrib['link'] = elem.attrib['link'].lower()

def get_val(root, path, attrib, default=""):
    el = root.find(path)
    if el is not None:
        return el.get(attrib, default)
    return default

print("--PROCESSING TEMPLATE XACRO FILE--")
# define data extracted from template xacro file
properties = {
    # MOTOR PROPERTIES - LINK
    "l_motor_fl_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_fl']/inertial/origin", "xyz", "0 0 0"),
    "l_motor_fr_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_fr']/inertial/origin", "xyz", "0 0 0"),
    "l_motor_rl_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_rl']/inertial/origin", "xyz", "0 0 0"),
    "l_motor_rr_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/origin", "xyz", "0 0 0"),
    "l_motor_fl_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_fl']/inertial/origin", "rpy", "0 0 0"),
    "l_motor_fr_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_fr']/inertial/origin", "rpy", "0 0 0"),
    "l_motor_rl_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_rl']/inertial/origin", "rpy", "0 0 0"),
    "l_motor_rr_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/origin", "rpy", "0 0 0"),
    "l_motor_mass": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/mass", "value"),
    "l_motor_ixx": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/inertia", "ixx"),
    "l_motor_ixy": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/inertia", "ixy"),
    "l_motor_ixz": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/inertia", "ixz"),
    "l_motor_iyy": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/inertia", "iyy"),
    "l_motor_iyz": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/inertia", "iyz"),
    "l_motor_izz": get_val(root_raw, ".//link[@name='link_motor_rr']/inertial/inertia", "izz"),
    "l_motor_color": get_val(root_raw, ".//link[@name='link_motor_rr']/visual/material/color", "rgba", "0.1 0.1 0.1 1"),
    "l_motor_fl_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_fl']/visual/origin", "xyz", "0 0 0"),
    "l_motor_fr_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_fr']/visual/origin", "xyz", "0 0 0"),
    "l_motor_rl_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_rl']/visual/origin", "xyz", "0 0 0"),
    "l_motor_rr_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_motor_rr']/visual/origin", "xyz", "0 0 0"),
    "l_motor_fl_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_fl']/visual/origin", "rpy", "0 0 0"),
    "l_motor_fr_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_fr']/visual/origin", "rpy", "0 0 0"),
    "l_motor_rl_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_rl']/visual/origin", "rpy", "0 0 0"),
    "l_motor_rr_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_motor_rr']/visual/origin", "rpy", "0 0 0"),
    "l_motor_fl_mesh": get_val(root_raw, ".//link[@name='link_motor_fl']/visual/geometry/mesh", "filename"),
    "l_motor_fr_mesh": get_val(root_raw, ".//link[@name='link_motor_fr']/visual/geometry/mesh", "filename"),
    "l_motor_rl_mesh": get_val(root_raw, ".//link[@name='link_motor_rl']/visual/geometry/mesh", "filename"),
    "l_motor_rr_mesh": get_val(root_raw, ".//link[@name='link_motor_rr']/visual/geometry/mesh", "filename"),

    # MOTOR PROPERTIES - JOINT
    "j_motor_fr_orig_xyz": get_val(root_raw, ".//joint[@name='joint_motor_fr']/origin", "xyz", "0 0 0"),
    "j_motor_fr_orig_rpy": get_val(root_raw, ".//joint[@name='joint_motor_fr']/origin", "rpy", "0 0 0"),
    "j_motor_fl_orig_xyz": get_val(root_raw, ".//joint[@name='joint_motor_fl']/origin", "xyz", "0 0 0"),
    "j_motor_fl_orig_rpy": get_val(root_raw, ".//joint[@name='joint_motor_fl']/origin", "rpy", "0 0 0"),
    "j_motor_rr_orig_xyz": get_val(root_raw, ".//joint[@name='joint_motor_rr']/origin", "xyz", "0 0 0"),
    "j_motor_rr_orig_rpy": get_val(root_raw, ".//joint[@name='joint_motor_rr']/origin", "rpy", "0 0 0"),
    "j_motor_rl_orig_xyz": get_val(root_raw, ".//joint[@name='joint_motor_rl']/origin", "xyz", "0 0 0"),
    "j_motor_rl_orig_rpy": get_val(root_raw, ".//joint[@name='joint_motor_rl']/origin", "rpy", "0 0 0"),
    "j_motor_fr_child_link": get_val(root_raw, ".//joint[@name='joint_motor_fr']/child", "link"),
    "j_motor_fl_child_link": get_val(root_raw, ".//joint[@name='joint_motor_fl']/child", "link"),
    "j_motor_rr_child_link": get_val(root_raw, ".//joint[@name='joint_motor_rr']/child", "link"),
    "j_motor_rl_child_link": get_val(root_raw, ".//joint[@name='joint_motor_rl']/child", "link"),
    "j_motor_parent_link": get_val(root_raw, ".//joint[@name='joint_motor_fr']/parent", "link"),
    "j_motor_joint_axis": get_val(root_raw, ".//joint[@name='joint_motor_fr']/axis", "xyz", "0 0 1"),
    "j_motor_lim_effort": get_val(root_raw, ".//joint[@name='joint_motor_fr']/limit", "effort", "1000"),
    "j_motor_lim_vel": get_val(root_raw, ".//joint[@name='joint_motor_fr']/limit", "velocity", "1000"),

    # ROTOR PROPERTIES - LINK
    "l_rotor_fl_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/origin", "xyz", "0 0 0"),
    "l_rotor_fr_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_fr']/inertial/origin", "xyz", "0 0 0"),
    "l_rotor_rl_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_rl']/inertial/origin", "xyz", "0 0 0"),
    "l_rotor_rr_inert_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_rr']/inertial/origin", "xyz", "0 0 0"),
    "l_rotor_fl_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/origin", "rpy", "0 0 0"),
    "l_rotor_fr_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_fr']/inertial/origin", "rpy", "0 0 0"),
    "l_rotor_rl_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_rl']/inertial/origin", "rpy", "0 0 0"),
    "l_rotor_rr_inert_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_rr']/inertial/origin", "rpy", "0 0 0"),
    "l_rotor_mass": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/mass", "value"),
    "l_rotor_ixx": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/inertia", "ixx"),
    "l_rotor_ixy": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/inertia", "ixy"),
    "l_rotor_ixz": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/inertia", "ixz"),
    "l_rotor_iyy": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/inertia", "iyy"),
    "l_rotor_iyz": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/inertia", "iyz"),
    "l_rotor_izz": get_val(root_raw, ".//link[@name='link_rotor_fl']/inertial/inertia", "izz"),
    "l_rotor_color": get_val(root_raw, ".//link[@name='link_rotor_fl']/visual/material/color", "rgba", "0.1 0.1 0.1 1"),
    "l_rotor_fl_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_fl']/visual/origin", "xyz", "0 0 0"),
    "l_rotor_fr_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_fr']/visual/origin", "xyz", "0 0 0"),
    "l_rotor_rl_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_rl']/visual/origin", "xyz", "0 0 0"),
    "l_rotor_rr_vis_coll_orig_xyz": get_val(root_raw, ".//link[@name='link_rotor_rr']/visual/origin", "xyz", "0 0 0"),
    "l_rotor_fl_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_fl']/visual/origin", "rpy", "0 0 0"),
    "l_rotor_fr_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_fr']/visual/origin", "rpy", "0 0 0"),
    "l_rotor_rl_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_rl']/visual/origin", "rpy", "0 0 0"),
    "l_rotor_rr_vis_coll_orig_rpy": get_val(root_raw, ".//link[@name='link_rotor_rr']/visual/origin", "rpy", "0 0 0"),
    "l_rotor_fl_mesh": get_val(root_raw, ".//link[@name='link_rotor_fl']/visual/geometry/mesh", "filename"),
    "l_rotor_fr_mesh": get_val(root_raw, ".//link[@name='link_rotor_fr']/visual/geometry/mesh", "filename"),
    "l_rotor_rl_mesh": get_val(root_raw, ".//link[@name='link_rotor_rl']/visual/geometry/mesh", "filename"),
    "l_rotor_rr_mesh": get_val(root_raw, ".//link[@name='link_rotor_rr']/visual/geometry/mesh", "filename"),

    # ROTOR PROPERTIES - JOINT
    "j_rotor_fr_orig_xyz": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/origin", "xyz", "0 0 0"),
    "j_rotor_fr_orig_rpy": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/origin", "rpy", "0 0 0"),
    "j_rotor_fl_orig_xyz": get_val(root_raw, ".//joint[@name='joint_rotor_fl']/origin", "xyz", "0 0 0"),
    "j_rotor_fl_orig_rpy": get_val(root_raw, ".//joint[@name='joint_rotor_fl']/origin", "rpy", "0 0 0"),
    "j_rotor_rr_orig_xyz": get_val(root_raw, ".//joint[@name='joint_rotor_rr']/origin", "xyz", "0 0 0"),
    "j_rotor_rr_orig_rpy": get_val(root_raw, ".//joint[@name='joint_rotor_rr']/origin", "rpy", "0 0 0"),
    "j_rotor_rl_orig_xyz": get_val(root_raw, ".//joint[@name='joint_rotor_rl']/origin", "xyz", "0 0 0"),
    "j_rotor_rl_orig_rpy": get_val(root_raw, ".//joint[@name='joint_rotor_rl']/origin", "rpy", "0 0 0"),
    "j_rotor_fr_child_link": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/child", "link"),
    "j_rotor_fl_child_link": get_val(root_raw, ".//joint[@name='joint_rotor_fl']/child", "link"),
    "j_rotor_rr_child_link": get_val(root_raw, ".//joint[@name='joint_rotor_rr']/child", "link"),
    "j_rotor_rl_child_link": get_val(root_raw, ".//joint[@name='joint_rotor_rl']/child", "link"),
    "j_rotor_fr_parent_link": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/parent", "link"),
    "j_rotor_fl_parent_link": get_val(root_raw, ".//joint[@name='joint_rotor_fl']/parent", "link"),
    "j_rotor_rr_parent_link": get_val(root_raw, ".//joint[@name='joint_rotor_rr']/parent", "link"),
    "j_rotor_rl_parent_link": get_val(root_raw, ".//joint[@name='joint_rotor_rl']/parent", "link"),
    "j_rotor_joint_axis": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/axis", "xyz", "0 0 1"),
    "j_rotor_lim_effort": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/limit", "effort", "1000"),
    "j_rotor_lim_vel": get_val(root_raw, ".//joint[@name='joint_rotor_fr']/limit", "velocity", "1000")
}

for prop in properties:
    for elem in root_template.iter():
        if elem.tag == f"{{http://www.ros.org/wiki/xacro}}property" and elem.attrib.get('name') == prop:
            elem.attrib['value'] = properties[prop]

# 1. Identify which components to skip (the ones in macros)
handled_keywords = ['motor', 'rotor']

# 2. Find the insertion point: the first <gazebo> tag in your template
# If no gazebo tag exists, it will just append to the end
insertion_index = len(root_template) 
for i, child in enumerate(root_template):
    if 'gazebo' in child.tag:
        insertion_index = i
        break

print(f"--INSERTING SINGLETONS AT INDEX {insertion_index}--")
newline_above = ET.Comment(" Singleton Links and Joints from URDF ")
newline_above.tail = "\n\n"
root_template.insert(insertion_index, newline_above)

# 3. Insert remaining LINKS and JOINTS from the URDF
# We iterate backwards or use a counter to keep the order correct during insertion
offset = 1
for tag in ['link', 'joint']:
    for elem in root_raw.findall(tag):
        name = elem.get('name', '')
        if not any(kw in name for kw in handled_keywords):
            print(f"Inserting singleton {tag}: {name}")
            elem.tail = "\n\n"
            root_template.insert(insertion_index + offset, elem)
            offset += 1

newline_below = ET.Comment(" End of Extra Links and Joints ")
newline_below.tail = "\n\n"
root_template.insert(insertion_index + offset, newline_below)

# 4. Save the final file
output_path = "/Users/nathanlodder/Documents/Academics/ACA-UBC/UBC-Classes/UBC-Courses-25_26/ACA-UBC-ENPH353/ENPH353-Comp/enph353-comp-drone/drone.urdf.xacro"
tree_template.write(output_path, xml_declaration=True, encoding='utf-8')