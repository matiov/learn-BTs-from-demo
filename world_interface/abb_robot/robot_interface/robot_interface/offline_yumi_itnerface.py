"""
Offline Interface for execution of behaviors.

Used by the planner in LfD framework to expand the BT.
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

import random
from typing import Dict, List

import numpy as np
# ROS python packages
import robot_interface.interface as wi


class OfflineInterface(wi.YuMiInterface, wi.MobileBaseInterface):
    """An interface for offline execution of YuMi behaviors."""

    # Limits for position initialization
    WORLD_LIMITS_XY = (-5, 5)
    WORLD_LIMITS_Z = (0, 1)
    # Probability of a random event
    RONDOM_EVENT_PROB = 0.3

    def __init__(
        self,
        available_objects: List[str],
        frames: List[str],
        default_frame: str,
        random_events: bool = True,
        init_state: Dict = {},
        base_frame: str = 'base'
    ):
        """
        Instantiate a new OfflineInterface with random initial conditions.

        Args:
        ----
            available_objects: list of the objects that can be manipulated.
            frames: list of reference frames.
            default_frame: used frame if nothing else is specified.
            random_events: if True, random disturbances might occur during execution.
            init_state: state of the robot.
            base_frame: name of the base frame of the robot.

        """
        self.default_frame = default_frame
        self.all_frames = frames + [base_frame]
        self.objects = available_objects
        self.random_events = random_events
        self.init_state = init_state
        self.base_frame = base_frame

        # Dictionary of frames and their position in default_frame
        self.frames = {}

        # Set initial conditions
        self.reset()

    def pick(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        frame: str
    ) -> 'EmptyTask':
        """
        Simulate a picking task.

        Args
        ----
            poistion: position of the target object.
            orientation: orientation of the target object.
            frame: reference frame for the object.

        Returns
        -------
            task: a thread simulating the task.

        """
        self.gripper = 'closed'
        self.holding = frame
        self.__random_event()

        task = EmptyTask()
        return task

    def place(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        frame: str
    ) -> 'EmptyTask':
        """
        Simulate a picking task.

        Args
        ----
            poistion: position of the target object.
            orientation: orientation of the target object.
            frame: reference frame for the object.

        Returns
        -------
            task: a thread simulating the task.

        """
        if frame == self.default_frame:
            self.frames[self.holding] = position
        else:
            # Transform frame
            self.frames[self.holding] = position + self.frames[frame]

        self.gripper = 'open'
        self.holding = ''

        self.__random_event()

        task = EmptyTask()
        return task

    def in_gripper(self, held_object: str) -> bool:
        """Return true if the object is held by the robot."""
        return self.holding == held_object

    def is_gripper_state(self, state: str) -> bool:
        """Return true gripper is open."""
        return False

    def empty_gripper(self) -> bool:
        """Return true gripper is open."""
        return self.holding == ''

    def object_position(
        self,
        target_object: str,
        frame: str
    ) -> np.ndarray:
        """Return the position of object in reference frame."""
        if frame == self.default_frame:
            return self.frames[target_object]
        else:
            return self.frames[target_object] - self.frames[frame]

    def set_gripper(self, state: str) -> 'EmptyTask':
        """Set the gripper state."""
        self.gripper = state
        if state == 'open' and self.holding != '':
            # The robot is holding something that will be dropped at
            # a random location
            self.frames[self.holding] = self.__random_location()
            self.holding = ''
        self.__random_event()
        return EmptyTask()

    def move(self, action: str) -> 'EmptyTask':
        """Simulate a move action for the mobile manipulator."""
        if action.frame[0] == self.default_frame:
            self.frames[self.base_frame] = action.target_position(action.frame[0])
        else:
            # Transform frame
            self.frames[self.base_frame] = action.target_position(action.frame[0]) +\
                self.frames[action.frame[0]]

        self.__random_event()
        return EmptyTask()

    def robot_position(self, frame: str) -> np.ndarray:
        """Return the position of the robot."""
        if frame == self.default_frame:
            return self.frames[self.base_frame]
        else:
            return self.frames[self.base_frame] - self.frames[frame]

    def transform_position(
        self,
        source_frame: str,
        target_frame: str,
        pos: np.ndarray
    ) -> np.ndarray:
        """Transform pos from source_frame to target_frame."""
        return pos + self.frames[source_frame] - self.frames[target_frame]

    def transform_orientation(
        self,
        source_frame: str,
        target_frame: str,
        orientation: np.ndarray
    ) -> np.ndarray:
        """Transform orientation from source_frame to target_frame."""
        return np.ndarray(1, 0, 0, 0)

    def approach(
        self,
        target: str,
        frame: str
    ) -> 'EmptyTask':
        """Move closer to target so it is reachable with the gripper."""
        target = target.copy()
        if frame != self.default_frame:
            target += self.frames[frame]

        # Sample a point uniformely from the 1.5m radius disk centered at target
        r = np.random.uniform(low=0, high=1.5, size=(1,)).item()
        theta = np.random.uniform(low=0, high=np.pi, size=(1,)).item()
        self.frames[self.base_frame] = np.array(
            [r*np.cos(theta) + target[0], r*np.sin(theta) + target[1], 0.5]
        )

        self.__random_event()
        return EmptyTask()

    def reachable(
        self,
        target: str,
        frame: str = None
    ) -> bool:
        """Target is reachable if it is within 1.5m of the robot. Orientation is not considered."""
        if isinstance(target, str):
            target = self.frames[target]
        else:
            target = target.copy()
            if frame != self.default_frame:
                target += self.frames[frame]

        return np.linalg.norm(target - self.frames[self.base_frame]) <= 1.5

    def reset(self):
        """Reset the internal state to a random initial state."""
        for f in self.all_frames:
            if f == self.default_frame:
                self.frames[f] = np.zeros((3,))
            elif f == self.base_frame:
                # Base frame cannot have arbitrary z component
                position = self.__random_location()
                position[2] = 0.5
                self.frames[f] = position
            else:
                self.frames[f] = self.__random_location()

        for f in self.init_state:
            self.frames[f] = self.init_state[f]

        if 'holding' in self.init_state:
            self.holding = self.init_state['holding']
        else:
            self.holding = random.choice(self.objects + [''])

        if self.holding != '':
            self.gripper = 'closed'
        else:
            self.gripper = random.choice(['open', 'closed'])

    def __random_location(self) -> np.ndarray:
        """Generate a random position."""
        return np.random.uniform(
            (self.WORLD_LIMITS_XY[0], self.WORLD_LIMITS_XY[0], self.WORLD_LIMITS_Z[0]),
            (self.WORLD_LIMITS_XY[1], self.WORLD_LIMITS_XY[1], self.WORLD_LIMITS_Z[1]),
            (3,)
        )

    def __random_event(self):
        """
        Produce a random event with probability RANDOM_EVENT_PROB.

        An example is dropping what is being held.
        """
        if not self.random_events:
            return

        if np.random.rand(1).item() > self.RONDOM_EVENT_PROB:
            # No random event
            return

        # Possible random events are change gripper state and randomly move an object
        events = ['gripper', 'displace']
        if self.holding != '':
            # If the robot is holding something it can also drop it
            events.append('drop')

        event = random.choice(events)

        if event == 'gripper':
            self.gripper = 'closed' if self.gripper == 'open' else 'open'
            # If we were holding something when the gripper opened we drop it at a random location
            if self.gripper == 'open' and self.holding != '':
                self.frames[self.holding] = self.__random_location()
                self.holding = ''
        elif event == 'displace':
            target_object = random.choice(self.objects)
            if target_object == self.holding:
                # If the robot is holding the robot it means it has been dropped
                # without opening the gripper
                self.holding == ''
            self.frames[target_object] = self.__random_location()
        elif event == 'drop':
            # Drop without opening gripper
            self.frames[self.holding] = self.__random_location()
            self.holding = ''


class EmptyTask(wi.Task):
    """A task that does nothing."""

    def __init__(self):
        self.ticks = 0
        super().__init__()

    def run(self):
        pass

    def done(self):
        self.ticks += 1
        if self.ticks >= 2:
            self.ticks = 0
            return True
        else:
            return False

    def terminate(self):
        # Interrupt and reset
        self.ticks = 0
