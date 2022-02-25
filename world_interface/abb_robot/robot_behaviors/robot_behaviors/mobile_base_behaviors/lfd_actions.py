"""Definition of Mobile Base Actions with heuristics and pre- and post-conditions."""

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

from typing import List

# TODO: make this a mobileyumi actions library and import from yumi
from bt_learning.learning_from_demo.demonstration import EquivalentAction
from robot_interface.demonstration import RobotAction


# Demonstrated Actions

class MoveAction(RobotAction):
    """Communicates that the base frame is to be ignored for move actions."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # TODO: make this as argument!
        self.exclude_frames = ['base']

# Equivalent actions


class EquivalentMove(EquivalentAction):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.target = self.targets[0, :].reshape((3,))

    def preconditions(self) -> List[str]:
        """Return action's pre-conditions."""
        return []

    def postconditions(self) -> List[str]:
        """Return action's post-conditions."""
        return [str(f'mobilebase_at {self.target[0]} {self.target[1]} {self.target[2]}' +
                f' {self.max_distance[0]} {self.actions[0].frame[0]}')]

    def action_string(self) -> str:
        """Return action's name."""
        return (f'{self.name} {self.target[0]} {self.target[1]} {self.target[2]}' +
                f' {self.max_distance[0]} {self.actions[0].frame[0]}')
