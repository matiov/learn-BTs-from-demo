"""
Definition of Mobile YuMi Behaviors.

Combines YuMi behaviors with MobileYuMi behaviors.
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

from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists
from behaviors.common_behaviors import Behaviour, RandomSelector, RSequence
from bt_learning.learning_from_demo.demonstration import Demonstration
from py_trees.composites import Selector, Sequence
from robot_behaviors.mobile_base_behaviors.lfd_behaviors import MobileBehaviors
from robot_behaviors.yumi_behaviors.lfd_behaviors import YuMiBehaviors
from robot_interface.interface import MobileBaseInterface, YuMiInterface


class MobileYuMiBehaviors(YuMiBehaviors, MobileBehaviors):
    """Combine YuMi behaviors with the Mobile base ones."""

    def __init__(self, directory_path: str):
        """Directory_path is the path to the directory where planner settings are saved."""
        super().__init__(directory_path)

    def get_behavior_list(self) -> BehaviorLists:
        """
        Return the behavior list.

        It just parses a yaml file so it is safe tu use superclasse method.
        """
        self.behavior_list = super().get_behavior_list()
        return self.behavior_list

    def get_node_from_string(
        self,
        string: str,
        world_interface: YuMiInterface or MobileBaseInterface,
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

        try:
            # get nodes from YuMi superclass
            node, has_children = YuMiBehaviors.get_node_from_string(
                self, string, world_interface, condition_parameters)
        except Exception:
            print(string)
            # if it doesn't work, try with the Mobile base instead
            node, has_children = MobileBehaviors.get_node_from_string(
                self, string, world_interface, condition_parameters)

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
        actions = YuMiBehaviors.get_actions(self, demonstrations) +\
            MobileBehaviors.get_actions(self, demonstrations)
        print(actions)

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
        conditions = YuMiBehaviors.get_conditions(self, demonstrations) +\
            MobileBehaviors.get_conditions(self, demonstrations)

        return conditions

    def compatible(
        self,
        condition1: str,
        condition2: str
    ) -> bool:
        """Return True if the conditions are compatible, False otherwise."""
        compatible = YuMiBehaviors.compatible(self, condition1, condition2) and\
            MobileBehaviors.compatible(self, condition1, condition2)
        return compatible
