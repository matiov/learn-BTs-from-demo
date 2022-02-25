"""
Definition of YuMi Behaviors.

Extends the LfDBehavior class.
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
from robot_interface.offline_yumi_itnerface import OfflineInterface
from robot_interface.online_yumi_interface import OnlineYuMiInterface
import yaml


NUMBER_REGEX = r'[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'

"""
The string representations of the behaviors are:
    - pick name object
    - place name object x y z tolerance frame
    - open_gripper
    - close_gripper
    - gripper_state open/closed
    - empty_gripper
    - in_gripper object
    - object_at object x y z tolerance frame
    - object_roughly_at object x y z tolerance frame

where name is the name of the pickle file where demonstrated actions are stored.
"""

# TODO: add support for both Right and Left gripper


class YuMiBehaviors(lfd_bt.Behaviors):
    """Defines all executable actions and conditions of the YuMi experiments."""

    def __init__(self, directory_path: str):
        """Directory_path is the path to the directory where planner settings are saved."""
        self.directory_path = directory_path
        self.behavior_list = None

    def get_behavior_list(self) -> BehaviorLists:
        """Parse the yaml file and returns the behavior list."""
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
        world_interface: OfflineInterface or OnlineYuMiInterface,
        condition_parameters: Any
    ) -> Tuple[Behaviour or RSequence or RandomSelector or Selector or Sequence, bool]:
        """
        Return the Behavior Tree node given its string representation.

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
        if string.startswith('pick'):
            # Pick is parametrized as 'pickN obj' where N is the unique number of
            # the pick action and obj is the object.
            match = re.match('^(pick\\d+) (.+)$', string)
            node = PickBehavior(string, self.directory_path, match[1], world_interface, match[2])
        elif string.startswith('place'):
            # Place is parameterized as 'placeN object x y z' similarily to pick
            match_str = f'^(place\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, string)
            node = PlaceBehavior(
                string,
                self.directory_path,
                match[1],
                world_interface,
                match[2],
                match.group(3, 4, 5),
                match[6]
            )
        elif string.startswith('drop'):
            match_str = f'^(drop\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, string)
            node = DropBehavior(
                string,
                self.directory_path,
                match[1],
                world_interface,
                match[2],
                match.group(3, 4, 5),
                match[6]
            )
        elif string.startswith('close_gripper'):
            node = SetGripper(string, world_interface, 'closed')
        elif string.startswith('open_gripper'):
            node = SetGripper(string, world_interface, 'open')

        # Conditions
        elif string.startswith('in_gripper'):
            node = InGripper(string, world_interface, string[11:])
        elif string.startswith('gripper_state'):
            node = GripperState(string, world_interface, string[14:])
        elif string.startswith('object_at'):
            match_str = f'^object_at (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, string)
            node = ObjectAt(
                string,
                world_interface,
                match[1],
                float(match[2]),
                float(match[3]),
                float(match[4]),
                float(match[5]),
                match[6]
            )
        elif string.startswith('object_roughly_at'):
            match_str = f'^object_roughly_at (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
            match = re.match(match_str, string)
            node = ObjectRoughlyAt(
                string,
                world_interface,
                match[1],
                float(match[2]),
                float(match[3]),
                float(match[4]),
                float(match[5]),
                match[6]
            )

        else:
            # get control node from the super class
            node, has_children = super().get_node_from_string(
                string, world_interface, condition_parameters)

        return node, has_children

    def get_actions(self, demonstrations: Demonstration) -> List[str]:
        """
        Get the combined actions for YuMi and the MobileBase from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            actions: list of the actions in the demonstration.

        """
        actions = ['open_gripper', 'close_gripper']

        # Add approach actions dynamically to match demonstrated actions
        for demo in demonstrations:
            for action in demo:
                name = action.action_string()
                action_string = None
                if name.startswith('pick'):
                    # Approach object
                    target_object = action.actions[0].parameters[0]
                    action_string = f'approach {target_object}'
                elif name.startswith('place'):
                    # Approach position
                    action_string = f'approach {action.targets[0,0]} {action.targets[0,1]}' +\
                        f' {action.targets[0,2]} {action.frames[0]}'

                if action_string is not None and action_string not in actions:
                    actions.append(action_string)

        return actions

    def get_conditions(self, demonstrations: Demonstration) -> List[str]:
        """
        Get the combined conditions for YuMi and the MobileBase from a demonstration.

        Args
        ----
            demonstration: the demonstration to parse.

        Returns
        -------
            conditions: list of the conditions in the demonstration.

        """
        conditions = ['gripper_state open', 'gripper_state closed', 'in_gripper none']

        # Add reachable conditions dynamically to match demonstrated conditions
        for demo in demonstrations:
            for action in demo:
                name = action.action_string()
                condition_string = None
                if name.startswith('pick'):
                    # Reach object
                    target_object = action.actions[0].parameters[0]
                    condition_string = f'reachable {target_object}'
                elif name.startswith('place'):
                    # Approach position
                    condition_string = f'reachable {action.targets[0, 0]}' +\
                        f' {action.targets[0, 1]} {action.targets[0, 2]} {action.frames[0]}'

                if condition_string is not None and condition_string not in conditions:
                    conditions.append(condition_string)

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
            if type1 == 'in_gripper' and parts1[1:] != parts2[1:]:
                return False
            elif type1 == 'gripper_state' and parts1[1:] != parts2[1:]:
                return False
            elif type1 == 'object_at' and parts1[1] == parts2[1] and parts1[2:] != parts2[2:]:
                return False
            elif type1 == 'object_roughly_at' and \
                    parts1[1] == parts2[1] and parts1[2:] != parts2[2:]:
                return False

        # object_at and roughly_at are incompatible
        if (type1 == 'object_roughly_at' and type2 == 'object_at' or
           type1 == 'object_at' and type2 == 'object_roughly_at') and \
           parts1[1] == parts2[1] and parts1[2:] != parts2[2:]:
            return False

        # We cannot hold something and have the gripper open at the same time
        if type1 == 'in_gripper' and \
           condition1 != 'in_gripper none' and condition2 == 'gripper_state open' or \
           type2 == 'in_gripper' and\
           condition2 != 'in_gripper none' and condition1 == 'gripper_state open':
            return False

        return True


class PickBehavior(lfd_bt.ActionBehavior):
    def __init__(
        self,
        action_string: str,
        directory_path: str,
        name: str,
        world_interface: OfflineInterface or OnlineYuMiInterface,
        target_object: str
    ):
        """
        Initialize the pick task.

        Args:
        ----
            action_string: name of the action.
            directory_path: path to the directory where the demonstration is stored.
            name: name of the action file.
            world_interface: interface to the robot.
            target_object: name for the object to pick.

        """
        self.world_interface = world_interface

        with open(directory_path + '/' + name + '.pkl', 'rb') as f:
            # this object contains fields defined in ActionInfo
            self.action_info = pickle.load(f)

        self.object = self.action_info.actions[0].parameters[0]
        self.picking_task = None

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        preconditions = self.action_info.additional_preconditions + ['gripper_state open']
        return preconditions

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        postconditions = ['gripper_state closed', f'in_gripper {self.object}']
        return postconditions

    def initialise(self):
        """Initialize the task as a thread."""
        # take one demonstrated action as sample
        picking_demo = random.choice(self.action_info.actions)
        frame = picking_demo.frame[0]

        # Select orientation from the random sample
        orientation = picking_demo.target_orientation(frame)

        # Select mean position
        # this should be equivalent to self.action_info.equivalent_action.targets[0]
        position = np.zeros((3,))
        for action in self.action_info.actions:
            position += action.target_position(frame)
        position /= len(self.action_info.actions)

        self.picking_task = self.world_interface.pick(position, orientation, frame)
        self.picking_task.start()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.picking_task.done():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        # Clear locks
        if hasattr(self.world_interface, 'state'):
            self.world_interface.state.blocking = False

        if new_status == pt.common.Status.INVALID and self.picking_task is not None:
            self.picking_task.terminate()

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return f'Pick {self.object}'

    def cost(self) -> int:
        """Define the cost of the action."""
        return 20


class PlaceBehavior(lfd_bt.ActionBehavior):

    def __init__(
        self,
        action_string: str,
        directory_path: str,
        name: str,
        world_interface: OfflineInterface or OnlineYuMiInterface,
        target_object: str,
        target: List[float],
        tolerance
    ):
        """
        Initialize the place task.

        Args:
        ----
            action_string: name of the action.
            directory_path: path to the directory where the demonstration is stored.
            name: name of the action file.
            world_interface: interface to the robot.
            target_object: name for the object to pick.
            target: target pose for the action.
            tolerance: error in the robot position.

        """
        self.world_interface = world_interface

        with open(directory_path + '/' + name + '.pkl', 'rb') as f:
            self.action_info = pickle.load(f)

        self.object = self.action_info.actions[0].parameters[0]
        self.target = target
        self.tolerance = tolerance
        self.placing_task = None

        super().__init__(action_string, world_interface)

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        preconditions = self.action_info.equivalent_action.preconditions_with_additional()
        return preconditions

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        postconditions = self.action_info.equivalent_action.postconditions()
        return postconditions

    def initialise(self):
        """Initialize the task as a thread."""
        place_demo = random.choice(self.action_info.actions)
        frame = place_demo.frame[0]

        # Lock postcondition
        if hasattr(self.world_interface, 'state'):
            for condition in self.get_postconditions() + self.get_preconditions():
                if condition.startswith('object_at') or \
                   condition.startswith('object_roughly_at') or \
                   condition.startswith('in_gripper'):
                    self.world_interface.state.blocked_conditions.append(condition)

        self.placing_task = self.world_interface.place(
            place_demo.target_position(frame), place_demo.target_orientation(frame), frame)
        self.placing_task.start()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.placing_task.done():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        if hasattr(self.world_interface, 'state'):
            self.world_interface.state.blocked_conditions.clear()

        if new_status == pt.common.Status.INVALID and self.placing_task is not None:
            self.placing_task.terminate()

    def get_display_name(self) -> str:
        """Returnt the action name."""
        name = 'Place %s at (%.2g, %.2g, %.2g) in %s' %\
            (
                self.object, float(self.target[0]), float(self.target[1]),
                float(self.target[2]), self.action_info.actions[0].frame[0]
            )
        return name

    def cost(self) -> int:
        """Define the cost of the action."""
        return 20


class DropBehavior(PlaceBehavior):

    def get_postconditions(self) -> List[str]:
        """Returnt the postconditions of the action."""
        postconditions = [
            'gripper_state open',
            'in_gripper none',
            f'object_roughly_at {self.object} {self.target[0]} {self.target[1]}' +
            f' {self.target[2]} {self.tolerance} {self.action_info.actions[0].frame[0]}'
        ]
        return postconditions

    def get_display_name(self) -> str:
        """Returnt the action name."""
        name = 'Drop %s at (%.2g, %.2g, %.2g) in %s' %\
            (
                self.object, float(self.target[0]), float(self.target[1]),
                float(self.target[2]), self.action_info.actions[0].frame[0]
            )
        return name


class SetGripper(lfd_bt.ActionBehavior):
    def __init__(
        self,
        action_string: str,
        world_interface: OfflineInterface or OnlineYuMiInterface,
        state: str
    ):
        """
        Initialize the pick task.

        Args:
        ----
            action_string: name of the action.
            world_interface: interface to the robot.
            state: if the gripper is open or close.

        """
        super().__init__(action_string, world_interface)

        self.state = state
        self.world_interface = world_interface
        self.gripper_task = None

    def get_preconditions(self) -> List[str]:
        """Return the pre-conditions of the action."""
        return []

    def get_postconditions(self) -> List[str]:
        """Return the post-conditions of the action."""
        if self.state == 'open':
            return ['gripper_state open', 'in_gripper none']
        elif self.state == 'closed':
            return ['gripper_state closed']
        else:
            raise ValueError(f'Unknown gripper state "{self.state}".\
                The state must be either "open" or "closed".')

    def initialise(self):
        """Initialize the task as a thread."""
        self.gripper_task = self.world_interface.set_gripper(self.state)
        self.gripper_task.start()

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if self.gripper_task.done():
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status):
        """Terminate the task thread and clear locks."""
        if new_status == pt.common.Status.INVALID and self.gripper_task is not None:
            self.gripper_task.terminate()

    def get_display_name(self) -> str:
        """Returnt the action name."""
        return f'Set gripper {self.state}'

    def cost(self) -> int:
        """Define the cost of the action."""
        return 2


# Conditions.
# Conditions don't need access to the configuration directory


class InGripper(pt.behaviour.Behaviour):
    """
    Returns SUCCESS if the gripper is holding object.

    If object is None the behavior tests empty gripper.
    """

    def __init__(
        self,
        name: str,
        world_interface: OfflineInterface or OnlineYuMiInterface,
        held_object: str
    ):
        """
        Initialize the condition.

        Args:
        ----
            name: name of the condition
            world_interface: interface to the robot.
            held_object: name for the object in the gripper.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.object = held_object
        self.last_state = pt.common.Status.INVALID

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if hasattr(self.world_interface, 'state'):
            if self.name in self.world_interface.state.blocked_conditions:
                return self.last_state

        if self.object is None or self.object == 'none':
            holding = self.world_interface.empty_gripper()
        else:
            holding = self.world_interface.in_gripper(self.object)

        if holding:
            self.last_state = pt.common.Status.SUCCESS
            return pt.common.Status.SUCCESS
        else:
            self.last_state = pt.common.Status.FAILURE
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return f'In gripper {self.object}?'


class GripperState(pt.behaviour.Behaviour):
    """Returns SUCCESS if the gripper is in the desired state."""

    def __init__(
        self,
        name: str,
        world_interface: OfflineInterface or OnlineYuMiInterface,
        state: Any
    ):
        """
        Initialize the condition.

        Args:
        ----
            name: name of the condition
            world_interface: interface to the robot.
            state: state of the gripper.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.state = state
        self.last_state = pt.common.Status.INVALID

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if hasattr(self.world_interface, 'state'):
            if self.world_interface.state.blocking:
                return self.last_state
        else:
            # We are running in offline mode. Return FAILURE first time to expand
            # node and then SUCCESS
            if self.last_state == pt.common.Status.INVALID:
                self.last_state = pt.common.Status.FAILURE
                return pt.common.Status.FAILURE
            else:
                return pt.common.Status.SUCCESS

        if self.world_interface.is_gripper_state(self.state):
            self.last_state = pt.common.Status.SUCCESS
            return pt.common.Status.SUCCESS
        else:
            self.last_state = pt.common.Status.FAILURE
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return f'Gripper {self.state}?'


class ObjectAt(pt.behaviour.Behaviour):
    """Returns SUCCESS if the object is at a location within a tolerance."""

    def __init__(
        self,
        name: str,
        world_interface: OfflineInterface or OnlineYuMiInterface,
        held_object: str,
        x: float,
        y: float,
        z: float,
        tolerance: float,
        frame: str
    ):
        """
        Initialize the condition.

        Args:
        ----
            name: name of the condition
            world_interface: interface to the robot.
            held_object: name for the object in the gripper.
            x: position of the object along X axis.
            y: position of the object along Y axis.
            z: position of the object along Z axis.
            tolerance: error in the object position.
            frame: reference frame in which the position is defined.

        """
        super().__init__(name)

        self.world_interface = world_interface
        self.object = held_object
        self.position = np.array([x, y, z])
        self.tolerance = tolerance
        self.frame = frame
        self.tolerance = 0.05
        self.last_state = pt.common.Status.INVALID

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if hasattr(self.world_interface, 'state'):
            if self.name in self.world_interface.state.blocked_conditions:
                return self.last_state

        current_position = self.world_interface.object_position(self.object, self.frame)
        if np.linalg.norm(self.position - current_position) <= self.tolerance:
            self.last_state = pt.common.Status.SUCCESS
            return pt.common.Status.SUCCESS
        else:
            self.last_state = pt.common.Status.FAILURE
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return '%s at (%.2g, %.2g, %.2g) in %s?' %\
            (self.object, self.position[0], self.position[1], self.position[2], self.frame)


class ObjectRoughlyAt(ObjectAt):

    def __init__(self, *args, **kwargs):
        """
        Initialize the condition.

        Args:
        ----
            name: name of the condition
            world_interface: interface to the robot.
            object: name for the object in the gripper.
            x, y, z: position of the object.
            tolerance: error in the object position.
            frame: reference frame in which the position is defined.

        """
        super().__init__(*args, **kwargs)

        self.tolerance = 0.1
        self.last_state = pt.common.Status.INVALID

    def update(self) -> pt.common.Status:
        """Return the status of the behavior."""
        if hasattr(self.world_interface, 'state'):
            if self.name in self.world_interface.state.blocked_conditions:
                return self.last_state

        current_position = self.world_interface.object_position(
            self.object, self.world_interface.base_frame)
        # Compute the position in the reference frame
        target_position = self.world_interface.transform_position(
            self.world_interface.base_frame, self.frame, self.position)

        # Check deviation in x and y only (cylinder with infinite height)
        if np.linalg.norm(target_position[:2] - current_position[:2]) <= self.tolerance:
            self.last_state = pt.common.Status.SUCCESS
            return pt.common.Status.SUCCESS
        else:
            self.last_state = pt.common.Status.FAILURE
            return pt.common.Status.FAILURE

    def get_display_name(self) -> str:
        """Returnt the condition name."""
        return '%s roughly at (%.2g, %.2g, %.2g) in %s?' %\
            (self.object, self.position[0], self.position[1], self.position[2], self.frame)
