"""List of behaviors to be used for the genetic programming."""

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

from dataclasses import dataclass
from dataclasses import field
import random
import re
from typing import List


@dataclass
class NodeParameter():
    """Parameter settings."""
    list_of_values: list = field(default_factory=list)
    min: float = 0.0
    max: float = 0.0
    step: float = 0.0
    comparing: bool = False
    in_front: bool = False


# pylint: disable=too-few-public-methods
class ParameterizedNode():
    """A node with parameters."""

    def __init__(self, parameters, condition):
        self.parameters = parameters
        self.condition = condition

    def add_random_parameters(self, node):
        """Adds random parameters to node."""
        for parameter in self.parameters:
            if isinstance(parameter, NodeParameter):
                parameter_string = ''
                if parameter.list_of_values:
                    parameter_string = random.choice(parameter.list_of_values)
                else:
                    if parameter.comparing:
                        if random.random() > 0.5:
                            parameter_string += '> '
                        else:
                            parameter_string += '< '

                    value = random.randrange(
                        parameter.min, parameter.max + parameter.step, parameter.step)
                    parameter_string += str(value)

                if parameter.in_front:
                    node = ''.join((parameter_string, ' ', node))
                else:
                    node = ''.join((node, ' ', parameter_string))
        if self.condition:
            node += '?'
        else:
            node += '!'

        return node


class BehaviorLists():
    """A list of all the available nodes."""

    def __init__(
        self,
        fallback_nodes: List[str] = None,
        sequence_nodes: List[str] = None,
        condition_nodes: List[str] = None,
        action_nodes: List[str] = None
        ):
        # A list of all types of fallback nodes used, typically just one
        if fallback_nodes is not None:
            self.fallback_nodes = fallback_nodes
        else:
            self.fallback_nodes = ['f(']

        # A list of all types of sequence nodes used, typically just one
        if sequence_nodes is not None:
            self.sequence_nodes = sequence_nodes
        else:
            self.sequence_nodes = ['s(']

        # Control nodes are nodes that may have one or more children/subtrees.
        # Subsequent nodes will be children/subtrees until the related up character is reached.
        # List will contain fallback_nodes, sequence_nodes and any other control nodes.
        self.control_nodes = self.fallback_nodes + self.sequence_nodes

        # Conditions nodes are childless leaf nodes that never return RUNNING state.
        self.condition_nodes = {}
        if condition_nodes is not None:
            self.condition_nodes = condition_nodes

        # Action nodes are also childless leaf nodes but may return RUNNING state.
        self.action_nodes = {}
        if action_nodes is not None:
            self.action_nodes = action_nodes

        # Atomic fallback nodes are fallback nodes that have a predetermined set of
        # children/subtrees that cannot be changed. 
        # They behave mostly like action nodes except that they may not be
        # the children of fallback nodes. Length is counted as one.
        self.atomic_fallback_nodes = []

        # Atomic sequence nodes are sequence nodes that have a predetermined set of
        # children/subtrees that cannot be changed. 
        # They behave mostly like action nodes except that they may not be
        # the children of sequence nodes. Length is counted as one.
        self.atomic_sequence_nodes = []

        # The up node is not a node but a character that marks the end of a control nodes
        # set of children and subtrees.
        self.up_node = [')']

        self.behavior_nodes = list(self.action_nodes.keys()) + self.atomic_fallback_nodes + self.atomic_sequence_nodes
        self.leaf_nodes = list(self.condition_nodes.keys()) + self.behavior_nodes
        self.all_nodes = self.control_nodes + self.leaf_nodes + self.up_node

    def is_fallback_node(self, node: str) -> bool:
        """Is node a fallback node."""
        if node in self.fallback_nodes:
            return True
        return False

    def is_sequence_node(self, node: str) -> bool:
        """Is node a sequence node."""
        if node in self.sequence_nodes:
            return True
        return False

    def is_control_node(self, node: str) -> bool:
        """Is node a control node."""
        if node in self.control_nodes:
            return True
        return False

    def get_random_control_node(self) -> str:
        """Return a random control node."""
        node = random.choice(self.control_nodes)

        return node

    def is_condition_node(self, node: str) -> bool:
        """Is node a condition node."""
        if remove_parameters(node) in self.condition_nodes:
            return True
        return False

    def get_random_condition_node(self) -> str:
        """Returns a random condition node."""
        string, node_object = random.choice(list(self.condition_nodes.items()))
        return node_object.add_random_parameters(string)

    def is_action_node(self, node: str) -> bool:
        """Is node an action node."""
        if remove_parameters(node) in self.action_nodes:
            return True
        return False

    def is_behavior_node(self, node: str) -> bool:
        """Is node a behavior node."""
        if remove_parameters(node) in self.behavior_nodes:
            return True
        return False

    def get_random_behavior_node(self) -> str:
        """Returns a random behavior node."""
        node = random.choice(self.behavior_nodes)

        if node in self.action_nodes:
            node = self.action_nodes[node].add_random_parameters(node)

        return node

    def is_leaf_node(self, node: str) -> bool:
        """Is node a leaf node."""
        if remove_parameters(node) in self.leaf_nodes:
            return True
        return False

    def get_random_leaf_node(self) -> str:
        """Return a random leaf node."""
        node = random.choice(self.leaf_nodes)

        if node in self.condition_nodes:
            node = self.condition_nodes[node].add_random_parameters(node)
        elif node in self.action_nodes:
            node = self.action_nodes[node].add_random_parameters(node)

        return node

    def is_up_node(self, node: str) -> bool:
        """Is node an up node."""
        if node in self.up_node:
            return True
        return False

    def get_up_node(self) -> str:
        """Returns up node"""
        return self.up_node[0]

    def is_valid_node(self, node: str) -> bool:
        """Returns True if node is valid node, False otherwise."""
        if remove_parameters(node) in self.all_nodes:
            return True
        return False


def remove_parameters(node: str):
    """Remove all parameters from node."""
    return re.sub(r'\d+', '', node).replace(r'.\d+', '',).\
        replace(' > ', '').replace(' < ', '').translate(str.maketrans('', '', '?!.')).strip()
