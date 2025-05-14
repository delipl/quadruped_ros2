from urdf2mjcf import run
from xacro import process_file

import os
import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
import mujoco
import mediapy as media
import mujoco_viewer
import xml.etree.ElementTree as ET

# Define states for the finite state machine
FSM_HOLD = 0
FSM_SWING1 = 1
FSM_SWING2 = 2
FSM_STOP = 3


def main():

    urdf = process_file(
        "/home/rabin/Documents/quadruped_robot/src/mujoco_ros2_control/mujoco_ros2_control_demos/urdf/quadruped_to_mujoco.urdf"
    )
    with open("quadruped_to_mujoco.urdf", "w") as f:
        f.write(urdf.toxml())

    os.system("sed -i 's/.dae/.stl/g' quadruped_to_mujoco.urdf")

    run(
        urdf_path="quadruped_to_mujoco.urdf",
        mjcf_path="quadruped.xml",
        copy_meshes=True,
    )

    # Run an OS command to modify the MJCF file using sed
    os.system('sed -i \'s/rgba="[^"]*"/rgba="0.5 0.5 0.5 1.0"/g\' quadruped.xml')
    os.system(
        'sed -i \'s/meshdir="meshes"/meshdir="\\/home\\/rabin\\/Documents\\/quadruped_robot\\/src\\/quadruped_robot_description\\/meshes"/g\' quadruped.xml'
    )

    # Parse the MJCF XML file
    tree = ET.parse("quadruped.xml")
    root = tree.getroot()

    # Add the <contact> tag with the <exclude> child
    contact = ET.SubElement(root, "contact")
    leg_prefixes = ["front_left", "front_right", "rear_left", "rear_right"]
    leg_links = [
        # "first_link",
        "second_link",
        "third_link",
        "fourth_link",
        "fifth_link",
        "sixth_link",
    ]
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "weld",
        body1="root",
        body2="world",
        name="weld_to_world",
    )
    for leg_prefix in leg_prefixes:

        # Find the <body> tag with the specified name
        for body in root.findall(f".//body[@name='{leg_prefix}_fifth_link']"):
            # Add the new <body> tag after the found <body>
            ET.SubElement(
                body,
                "body",
                gravcomp="0",
                name=f"{leg_prefix}_chain_close_a_link",
                pos="0.21 0 -0.0115",
                quat="0 0 0 1",
            )
            # Add <equality> constraints for each leg
        ET.SubElement(
            equality,
            "connect",
            anchor="0.0 0 0",
            body1=f"{leg_prefix}_foot_link",
            body2=f"{leg_prefix}_chain_close_a_link",
            active="true",
            name=f"equality_sconstraint_{leg_prefix}",
        )

        # Add a <weld> constraint to the <equality> tag

        for link in leg_links:
            ET.SubElement(
                contact,
                "exclude",
                name=f"exclude_{leg_prefix}_{link}_root",
                body1=f"{leg_prefix}_{link}",
                body2="root",
            )
            for link2 in leg_links:
                if link != link2:
                    ET.SubElement(
                        contact,
                        "exclude",
                        name=f"exclude_{leg_prefix}_{link}_{leg_prefix}_{link2}",
                        body1=f"{leg_prefix}_{link}",
                        body2=f"{leg_prefix}_{link2}",
                    )

    # Write the updated XML back to the file
    tree.write(
        "/home/rabin/Documents/quadruped_robot/src/mujoco_ros2_control/mujoco_ros2_control_demos/mujoco_models/quadruped.xml"
    )


if __name__ == "__main__":
    main()
