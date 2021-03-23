# SR_Driver

---

## Usage

---

## Description
This package is a simple state machine controller for the Soft Robotics MGrip gripper. This package allows you to
control the grip easier via a ROS node. 

You publish a string with your desired state to the "/sr_gripper" node and this will handle the rest of the logic.
The default pressure limits are declared but can be easily modified. Although this was written for the UR5 arm it should
work for the entire UR series(with slight modifications). You will need to modify the pin assignments within the code to
suit your install of the gripper, or install your gripper as we installed ours.

Tested on Ubuntu 16.04 and 20 (results may vary with other distros)
communicating with a UR5 running Version 3.13 of their Polyscope software

---

## Requirements
* ur_rtde python version(https://pypi.org/project/ur-rtde/)
* Python 2.7
* ROS Kinetic


(If you want to regenerate documentation after any modification)
* Sphinx-Common
* Sphinx-Doc

---


