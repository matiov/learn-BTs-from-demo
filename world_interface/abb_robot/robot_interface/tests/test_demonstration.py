"""Test the demonstration class."""

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


from copy import deepcopy
import glob

from bt_learning.learning_from_demo.tests.util import TESTDIR
import numpy as np
import pytest
import robot_interface.demonstration as demo
import yaml


def test_action():
    """Test constructing and using the Action class."""
    with open(TESTDIR + '/demo_test/demo1/data_1.yaml') as f:
        data = yaml.safe_load(f)
    action = demo.RobotAction(data, ['base'], 'base')

    # TODO: Check that action has been set

    # Test that we can access position information
    assert np.all(action.target_position('base') == data['vec_pos']['base'][-3:])
    # Test that we can access orientation information
    assert np.array_equal(action.target_orientation('base'), data['vec_quat']['base'][-4:])

    # If we give an undefined reference frame it should raise KeyErrir
    with pytest.raises(KeyError):
        action.target_position('???')
    with pytest.raises(KeyError):
        action.target_orientation('???')


def test_action_equality():
    """Test equality between instances of actions."""
    with open(TESTDIR + '/demo_test/demo1/data_1.yaml') as f:
        action1 = yaml.safe_load(f)
    with open(TESTDIR + '/demo_test/demo1/data_2.yaml') as f:
        action2 = yaml.safe_load(f)
    with open(TESTDIR + '/demo_test/info.yaml') as f:
        data = yaml.safe_load(f)
        frames = data['frames']
        default_frame = data['default_frame']
    a1 = demo.RobotAction(deepcopy(action1), frames, default_frame)
    a2 = demo.RobotAction(deepcopy(action1), frames, default_frame)
    a3 = demo.RobotAction(action2, frames, default_frame)

    assert a1 == a2
    assert a1 != a3


def test_demonstrations_constructor():
    """Test that the constructor of the Demonstrations class constructs the object correctly."""
    # Test that invalid file is detected
    with pytest.raises(FileNotFoundError):
        demo.RobotDemonstration('???????')

    demo_obj = demo.RobotDemonstration(TESTDIR + '/demo_test')

    # Load all recorded actions manually to compare
    with open(TESTDIR + '/demo_test/info.yaml') as f:
        info = yaml.safe_load(f)
    actions = []
    for folder in glob.glob(TESTDIR + '/demo_test/demo[0-9]*/'):
        for file in glob.glob(folder + '/*.yaml'):
            with open(file) as f:
                actions.append(
                    demo.RobotAction(yaml.safe_load(f), info['frames'], info['default_frame'])
                )

    # Check that demonstrations is not empty
    assert len(demo_obj.demonstrations()) > 0
    for demonstration in demo_obj.demonstrations():
        assert len(demonstration) > 0
    first_action = demo_obj.demonstrations()[0][0]
    # All actions should be converted to the ActionDemo class (or a subclass)
    assert isinstance(first_action, demo.RobotAction)
    # Check that all actions have been loaded
    assert demo_obj.all_actions() == actions
