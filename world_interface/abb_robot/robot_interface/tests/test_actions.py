"""Test simple YuMi actions and general action classes."""

# Copyright (c) 2021 Matteo Iovino
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from bt_learning.learning_from_demo.tests.util import TESTDIR
from robot_behaviors.yumi_behaviors.lfd_actions import PickAction, PlaceAction
from robot_interface.demonstration import RobotAction, RobotDemonstration


def test_pick_demo():
    demos = RobotDemonstration(TESTDIR + '/demo_test', {'pick': PickAction, 'place': PlaceAction})

    action = demos.demonstrations()[0][0]

    # In this action we picked 'object'
    assert action.frame == ['object']
    assert action.parameters == ['object']


def test_place_demo():
    demos = RobotDemonstration(TESTDIR + '/demo_test', {'pick': PickAction, 'place': PlaceAction})

    action = demos.demonstrations()[0][1]
    # In this action we placed 'object'
    assert action.parameters == ['object']


class ActionPickTest(RobotAction):

    def __init__(self, data, frames, default_frame, *args, **kwargs):
        super().__init__(data, frames, default_frame, *args, **kwargs)


def test_action_demo():
    demos = RobotDemonstration(TESTDIR + '/demo_test', {'pick': ActionPickTest})

    # All pick actions should use our derived class
    for action in demos.all_actions():
        if action.type == 'pick':
            assert isinstance(action, ActionPickTest)
