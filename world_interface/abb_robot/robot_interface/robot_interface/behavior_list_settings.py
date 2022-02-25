"""Settings for behavior_lists for the task."""

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

from typing import Type

from behaviors.behavior_lists import BehaviorLists
import yaml


def get_behavior_list(file: str = None) -> Type[BehaviorLists]:
    """
    Return the behavior list.

    Args
    ----
        file: path to the file defining the Behaviors.

    Returns
    -------
        behavior_list: BehaviorList object with all behaviors.

    """
    # initialize the dictionary an populate it while parsing the yaml
    condition_nodes = {}
    action_nodes = {}

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

    behavior_list = BehaviorLists(condition_nodes=condition_nodes, action_nodes=action_nodes)

    return behavior_list
