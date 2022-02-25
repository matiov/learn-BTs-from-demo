"""Methods that allow to modify a learnt tree."""

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

import logging
import os
from typing import Any, List

from bt_learning.learning_from_demo.demonstration import Demonstration
import py_trees as pt
from py_trees.trees import BehaviourTree
from simulation.py_trees_interface import PyTree
import yaml


logger = logging.getLogger('learning')


def update_bt_settings(
    file_path: str,
    demonstrations: Demonstration,
    behaviors: Any
):
    """Update the BT settings with the added behaviors."""
    file_name = file_path + '/BT_SETTINGS.yaml'
    with open(file_name) as f:
        data = yaml.safe_load(f)

    data_actions = data['action_nodes']
    data_conditions = data['condition_nodes']

    behavior_list = behaviors.get_behavior_list()
    actions = behaviors.get_actions(demonstrations)
    conditions = behaviors.get_conditions(demonstrations)


    for action in actions:
        if action not in data_actions:
            data_actions.append(action)
    for condition in conditions:
        if condition not in data_conditions:
            data_conditions.append(condition)

    settings = {
        'fallback_nodes': ['f('],
        'sequence_nodes': ['s('],
        'condition_nodes': data_conditions,
        'action_nodes': data_actions,
        'up_node': [')']
    }

    with open(file_name, 'w') as f:
        yaml.dump(settings, f)
