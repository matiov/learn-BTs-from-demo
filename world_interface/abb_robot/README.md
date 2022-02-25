# Overview

This module allows to interface with the ABB robots YuMi and MobileYuMi.  
Here, the robot skills are defined and robot communication is esablished.  
For more information please refer to [RWS](https://github.com/ros-industrial/abb_librws) and [EGM](https://github.com/ros-industrial/abb_libegm).  
Note that the provided code is intended to communicate with an ABB YuMi robot, loaded with specific routines implementing the robot skills (such routines can be provided upon request).
If you intend to use another robot, make sure to modify the interfaces as well as the `launch` and `config` files.  
The exemples available in [this module](./robot_examples) use Behavior Trees as skill representation.

## Contents

* The module [Robot Interface](../world_interface/abb_robot/robot_interface) is a ROS2 python package that defines Demonstrations and robot Actions of the Learning from Demonstration framework and the interface to the ABB robots.
* The ABB modules are ROS2 packages containing the definition of ROS messages and services.
* The [Mobile Base](./robot_behaviors/robot_behaviors/mobile_base_behaviors) and [YuMi](./robot_behaviors/robot_behaviors/yumi_behaviors) modules contain the interfaces to the robot skills and the definitions of the available behaviors.
* The `lfd_gui.py` defines the GUI that is used in the Learning from Demonstration framework. To learn a Behavior Tree from demonstration, please refer to the available [documentation](./robot_interface/doc/marker_LfD.md).

**Note:** to include a python script belonging to pure python module and/or ROS2 python packages, follow this example:
```python
# ROS-related imports
from ros_package.script import function, class
# pure python imports
from python_package.script import function, class
```
Packages are intended as those folders containing the ```setup.py``` script.
