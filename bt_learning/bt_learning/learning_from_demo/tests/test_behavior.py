"""Test LfD specific constructors for Actions."""

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

from bt_learning.learning_from_demo.demonstration import Action, EquivalentAction
import numpy as np
import quaternion


class ActionTest(EquivalentAction):

    def __init__(self, actions):
        super().__init__(actions)

    def preconditions(self):
        pass

    def postconditions(self):
        pass

    def action_string(self):
        pass


class ObservedActionTest(Action):

    def __init__(self):
        super().__init__('test_action', 'test_frame')
        self.n_targets = 2

        # Poisition measurement noise standard deviation
        pos_stddev = 0.1
        # Rotation measurement noise standard deviation
        rot_stddev = 0.1

        # Targets at (0, 0, 0) and (1, 2, 3) with noise
        self.__targets = np.array([[0, 0, 0], [1, 2, 3]]) + pos_stddev*np.random.randn(2, 3)

        # First target is aligned with the frame
        rotation_vector1 = np.array([0, 0, 0])
        # The second target is rotated 90 degrees around the y-axis with measurement
        # noise on the angle
        rotation_vector2 = np.array([0, 1, 0])*(np.pi/2 + rot_stddev*np.random.randn(1))

        self.__target_orientations = quaternion.from_rotation_vector(
            np.stack([rotation_vector1, rotation_vector2]))

    def target_position(self, frame, i):
        """Define target at (0, 0, 0) and (1, 2, 3) with noise."""
        return self.__targets[i]

    def target_orientation(self, frame, i):
        return self.__target_orientations[i]


def test_combining_actions():
    """Test means of the actions when building an EquivalentAction."""
    # Create multiple instances of the same action with noise
    actions = []
    for i in range(1000):  # Many observations to reduce variance and random test failures
        actions.append(ObservedActionTest())

    action = ActionTest(actions)

    # Actions should be summarized into two targets
    assert np.shape(action.targets) == (2, 3)
    assert np.shape(action.max_distance) == (2,)
    assert np.shape(action.target_orientations) == (2,)

    # Target positions close to (0, 0, 0) and (1, 2, 3)
    assert np.allclose(action.targets, np.array([[0, 0, 0], [1, 2, 3]]), rtol=0, atol=0.01)

    # Test orientations. Note that two quaternions q1 and q2 represent the same rotation
    # if either q1 == q2 or -q1 == q2.
    target_orientations = quaternion.from_rotation_vector(np.array([[0, 0, 0], [0, np.pi/2, 0]]))
    # Target orientation 1 close to being aligned
    assert quaternion.allclose(
                action.target_orientations[0], target_orientations[0], rtol=0, atol=0.06) or \
           quaternion.allclose(
                -action.target_orientations[0], target_orientations[0], rtol=0, atol=0.06)
    # Target orientation 2 close to being aligned
    assert quaternion.allclose(
                action.target_orientations[1], target_orientations[1], rtol=0, atol=0.06) or \
           quaternion.allclose(
                -action.target_orientations[1], target_orientations[1], rtol=0, atol=0.06)
