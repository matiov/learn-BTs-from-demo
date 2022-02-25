"""
Definition of Mobile Base Behaviors.

Extends the LfDBehavior class
"""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import pickle
import random
import re
from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists
from behaviors.common_behaviors import Behaviour, RandomSelector, RSequence
from bt_learning.learning_from_demo.demonstration import Demonstration
import bt_learning.learning_from_demo.lfd_behaviors as lfd_bt
import numpy as np
import py_trees as pt
from py_trees.composites import Selector, Sequence
from robot_interface.interface import MobileBaseInterface
import yaml


NUMBER_REGEX = r'[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'

"""
The string representations of the behaviors are:
    - move_to name x y z tolerance frame
    - approach x y z frame OR approach object
    - mobilebase_at x y z tolerance frame
    - reachable x y z frame OR reachable object

where name is the name of the pickle file where demonstrated actions are stored.
"""


class MobileBehaviors(lfd_bt.Behaviors):
    """Defines all executable actions and conditions of the YuMi experiments."""

    def __init__(self, directory_path: str):
        """Directory_path is the path to the directory where planner settings are saved."""
        self.directory_path = directory_path
        self.behavior_list = None

    def get_behavior_list(self) -> BehaviorLists:
        """Return the behavior list."""
        # initialize the dictionary an populate it while parsing the yaml
        condition_nodes = {}
        action_nodes = {}

        file = self.directory_path + '/BT_SETTINGS.yaml'
        if file is not None:
            with open(file) as f:
                bt_settings = yaml.load(f, Loader=yaml.FullLoader)
            try:
                node_name = bt_settings['condition_nodes']
                for _ in range(len(node_name)):
                    condition_nodes[node_name[_]] = []
            except KeyError:
                pass
            try:
                node_name = bt_settings['action_nodes']
                for _ in range(len(node_name)):
                    action_nodes[node_name[_]] = []
            except KeyError:
                pass

        self.behavior_list = BehaviorLists(
            condition_nodes=condition_nodes, action_nodes=action_nodes)

        return self.behavior_list

    def get_node_from_string(
        self,
        string: str,
        world_interface: MobileBaseInterface,
        condition_parameters: Any
    ) -> Tuple[Behaviour or RSequence or RandomSelector or Selector or Sequence, bool]:
        """
        Link a string representation of the skill to the behavior.

        Args
        ----
            string: name of the robot skill as string.
            world_interface: interface to the robot hardware.
            condition_parameters: pre- and post-conditions of the skill.

        Returns
        -------
            node: behavior tree node, eventually inherits from py_trees
            has_children: bool to determine if the node is a control node or a behavior.

        """
        has_children = False
        # Actions
        if string.startswith('move_to'):
            match_str = f'^(move_to\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, string)
            node = MoveTo(
                string,
                self.directory_path, match[1],
                world_interface,
                match.group(2, 3, 4),
                match[5]
            )
        elif string.startswith('approach'):
            parts = string.split()
            if len(parts) == 2:
                # Apporach object
                node = Approach(string, world_interface, target_object=parts[1])
            else:
                # Approach position
                node = Approach(
                    string,
                    world_interface,
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                    parts[4]
                )

        # Conditions
        elif string.startswith('mobilebase_at'):
            match_str = f'^mobilebase_at (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, string)
            node = RobotAt(
                string,
                world_interface,
                float(match[1]),
                float(match[2]),
                float(match[3]),
                float(match[4]),
                match[5]
            )
        elif string.startswith('reachable'):
            parts = string.split()
            if len(parts) == 2:
                node = Reachable(string, world_interface, target_object=parts[1])
            else:
                node = Reachable(
                    string,
                    world_interface,
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                    parts[4]
                )

        else:
            # get control nodes from super class
            node, has_children = super().get_node_from_string(
                string, world_interface, condition_parameters)

        return node, has_children

    def get_actions(
        self,
        demonstrations: Demonstration
    ) -> List[str]:
        """
        Get the combined actions for YuMi and the MobileBase from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            actions: list of the actions in the demonstration.

        """
        # TODO: implement this method correctly
        actions = []

        return actions

    def get_conditions(
        self,
        demonstrations: Demonstration
    ) -> List[str]:
        """
        Get the combined conditions for YuMi and the MobileBase from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            conditions: list of the conditions in the demonstration.

        """
        # TODO: implement this method correctly
        conditions = []

        return conditions

    def compatible(
        self,
        condition1: str,
        condition2: str
    ) -> bool:
        """Return True if the conditions are compatible, False otherwise."""
        parts1 = condition1.split()
        parts2 = condition2.split()
        # The condition type is the first "word"
        type1 = parts1[0]
        type2 = parts2[0]

        # Incompatible conditions of the same type
        if type1 == type2:
            if type1 == 'mobilebase_at' and condition1 != condition2:
                return False

        return True


class MoveTo(lfd_bt.ActionBehavior):
    def __init__(
        self,
        action_string: str,
        directory_path: str,
        name: str,
        world_interface: MobileBaseInterface,
        target: np.ndarray,
        tolerance: float
    ):
        """
        Initialize the move task.

        Args:
        ----
            action_string: name of the action.
            directory_path: path to the directory where the demonstration is stored.
            name: name of the action file.
            world_interface: interface to the robot.
            target: target goal for the robot.
            tolerance: allowed error for the action.

        """
        self.world_interface = world_interface

        with open(directory_path + '/' + name + '.pkl', 'rb') as f:
            self.action_info = pickle.load(f)

        self.target = target
        self.tolerance = tolerance
        self.moving_task = None

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        return []

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        return [
            str(f'robot_at {self.target[0]} {self.target[1]} {self.target[2]}' +
                f' {self.tolerance} {self.action_info.actions[0].frame[0]}')
        ]

    def initialise(self):
        """Initialize the task as a thread."""
        self.moving_task = self.world_interface.move(random.choice(self.action_info.actions))
        self.moving_task.start()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.moving_task.done():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self,  new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        if new_status == pt.common.Status.INVALID and self.moving_task is not None:
            self.moving_task.terminate()

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return 'Move to (%.2g, %.2g, %.2g) in %s' %\
            (
                float(self.target[0]), float(self.target[1]), float(self.target[2]),
                self.action_info.actions[0].frame[0]
            )

    def cost(self) -> int:
        """Define the cost of the action."""
        return 30


class Approach(lfd_bt.ActionBehavior):
    def __init__(
        self,
        action_string: str,
        world_interface: MobileBaseInterface,
        x: float = None,
        y: float = None,
        z: float = None,
        frame: str = None,
        target_object: str = None
    ):
        """
        Approach action for the mobile platform.

        If object is not None, x, y, z, and frame are ignored.
        If object is None, x, y, z, and frame must have values.

        Args:
        ----
            action_string: name of the action.
            world_interface: interface to the robot.
            x: value along X axis of the position to approach.
            y: value along Y axis of the position to approach.
            z: value along Z axis of the position to approach.
            frame: reference frame for the object to approach.
            target_object: name for the object to approach.

        """
        self.world_interface = world_interface

        self.object = target_object
        self.target = None
        self.frame = world_interface.default_frame
        self.moving_task = None
        if self.object is None:
            self.target = np.array([x, y, z])
            self.frame = frame

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        return []

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        if self.object is None:
            return [f'reachable {self.target[0]} {self.target[1]} {self.target[2]} {self.frame}']
        else:
            return [f'reachable {self.object}']

    def initialise(self):
        """Initialize the task as a thread."""
        if self.object is not None:
            self.target = self.world_interface.object_position(self.object, self.frame)
        self.moving_task = self.world_interface.approach(self.target, self.frame)
        self.moving_task.start()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.moving_task.done():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        if new_status == pt.common.Status.INVALID and self.moving_task is not None:
            self.moving_task.terminate()

    def get_display_name(self) -> str:
        """Returnt the action name."""
        if self.object is None:
            return 'Approach (%.2g, %.2g, %.2g) in %s' %\
                (self.target[0], self.target[1], self.target[2], self.frame)
        else:
            return 'Approach ' + self.object

    def cost(self) -> int:
        """Define the cost of the action."""
        return 30


# Conditions.
# Conditions don't need access to the configuration directory

class RobotAt(pt.behaviour.Behaviour):
    """Returns SUCCESS if the object is at a location within a tolerance."""

    def __init__(
        self,
        name: str,
        world_interface: MobileBaseInterface,
        x: float,
        y: float,
        z: float,
        tolerance: float,
        frame: str
    ):
        """
        Condition to determine if the robot is a specific position.

        Args:
        ----
            name: name of the action.
            world_interface: interface to the robot.
            x: value along X axis of the position of the robot.
            y: value along Y axis of the position of the robot.
            z: value along Z axis of the position of the robot.
            tolerance: error in the robot position.
            frame: reference frame for robot.

        """
        super().__init__(name, world_interface)

        self.world_interface = world_interface
        self.position = np.array([x, y, z])
        self.tolerance = tolerance
        self.frame = frame

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        current_position = self.world_interface.robot_position(self.frame)
        if np.linalg.norm(self.position - current_position) <= self.tolerance:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return 'Robot at (%.2g, %.2g, %.2g) in %s?' %\
            (self.position[0], self.position[1], self.position[2], self.frame)


class Reachable(pt.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        world_interface: MobileBaseInterface,
        x: float = None,
        y: float = None,
        z: float = None,
        frame: str = None,
        target_object: str = None
    ):
        """
        Determine if the target pose or object is reachable.

        If object is not None, x, y, z, and frame are ignored.
        If object is None, x, y, z, and frame must have values.

        Args:
        ----
            name: name of the condition.
            world_interface: interface to the robot.
            x: value along X axis of the position to approach.
            y: value along Y axis of the position to approach.
            z: value along Z axis of the position to approach.
            frame: reference frame for the object to approach.
            target_object: name for the object to approach.

        """
        self.world_interface = world_interface
        self.target = None
        self.frame = world_interface.default_frame
        self.object = target_object

        if self.object is None:
            self.target = np.array([x, y, z])
            self.frame = frame

        super().__init__(name, world_interface)

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.object is not None:
            self.target = self.world_interface.object_position(self.object, self.frame)

        if self.world_interface.reachable(self.target, self.frame):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the action name."""
        if self.object is None:
            return 'Reachable (%.2g, %.2g, %.2g) in %s?' %\
                (self.target[0], self.target[1], self.target[2], self.frame)
        else:
            return f'Reachable {self.object}?'
