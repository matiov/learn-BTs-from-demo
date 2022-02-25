# Overview

Repository containing different methods to automatically generate a Behavior Tree (BT) for robotic tasks. Mainly Python based.  
In the learning methods, the BT is represented as a string, e.g. `['s(', 'action1', 'action2', ')']` and then the string is converted in a `py_tree` BT.  

Common behaviors and BT methods are defined in the [behaviors](./behaviors) module.  
Application specific behaviors will be defined in the application's dedicated module, together with the extension of the method `get_node_from_string()`, see for example the structure of the [mobile conveyor](./simulation/mobile_conveyor) simulated task.

## Contents

* The [BT-learning](./bt_learning) module contains methods to automatically generate BTs, e.g.: Learning from Demonstration, Genetic Programming, Planners.
* The [Perception Layer](./perception_layer) module contains ROS packages realizing marker recognition.
* The [World Interface](./world_interface) module contains interfaces to the ABB robots and to the camera in case of vision-based applications.

Please see the documentation in every module for more detailed information.

## Available documentation

### Behavior Tree Learning
* Information on the demonstration format for the Learning from Demonstration framework is found [here](./bt_learning/doc/demonstration.md)

### Py Trees
* General information about the `py_trees` library in the [README](./py_trees/README.md)

### World Interface
* General information about the interface with the ABB robots in the [README](./world_interface/abb_robot/README.md)
* Information on the usage of the LfD GUI is found [here](./world_interface/abb_robot/robot_interface/doc/marker_LfD.md)
* Instructions for robot visualization and spawning are in the [robot_bringup](./world_interface/abb_robot/robot_bringup/README.md) package


## DISCLAIMER

The code providing the communication from the proposed framework to the ABB robot is protected by copyright and will not be disclosed. The provided robot interfaces are meant to interact with ABB robots, but can be modified to be compliant with other robot hardware. Said code, as well as the RAPID routines running inside the robot controlle, can be provided upon request. Any case will be treated individually.
