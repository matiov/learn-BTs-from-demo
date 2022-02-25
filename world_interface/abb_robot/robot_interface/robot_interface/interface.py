"""
High level interface for the LfD framework.

It contains high level defitition for robot behaviors methods.
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

from abc import ABC, abstractmethod
from threading import Thread

import numpy as np


class Task(Thread, ABC):
    """
    Represents a task which is executed asynchronously.

    Note that there is no possibility for actions to return Failure.
    They are assumed to return Running until they are done.
    """

    # TODO: add support for FAILURE
    def __init__(self, *args, **kwargs):
        Thread.__init__(self, *args, **kwargs)

    def done(self) -> bool:
        """
        Return whether the action has completed.

        By default it returns True if this thread has terminated.
        Override if another criterium is needed.
        """
        return not self.is_alive()

    def run(self):
        """
        Run the task thread.

        The run function is executed when the task is started.
        The function is executed in a separate thread.
        Override this function to executes behaviors that may block and take time to finish.
        """
        pass

    @abstractmethod
    def terminate(self):
        """
        Terminate the task thread.

        This function is called when a behavior wishes to terminate the task before completion.
        """
        pass


class YuMiInterface(ABC):
    """
    Base class defining the methods that must exist in the world interface of the YuMi experiments.

    Each action retruns an instance of Task.
    """

    @abstractmethod
    def place(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        frame: str
    ):
        """
        Initiate a placing task.

        Args:
        ----
            position: target position for the action.
            orientation: target orientation for the action.
            frame: frame the position and the orientation are referred to.

        """
        pass

    @abstractmethod
    def pick(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        frame: str
    ):
        """
        Initiate a placing task.

        Args:
        ----
            position: target position for the action.
            orientation: target orientation for the action.
            frame: reference frame attached to the object that it is picked.

        """
        pass

    @abstractmethod
    def in_gripper(self, object_name: str) -> bool:
        """Return True if the robot is holding object."""
        pass

    @abstractmethod
    def is_gripper_state(self, state: str) -> bool:
        """Return True if the gripper state is state ("open"/"closed")."""
        pass

    @abstractmethod
    def empty_gripper(self) -> bool:
        """
        Return True if the robot is not holding anything.

        Note that the gripper can be empty while closed.
        """
        pass

    @abstractmethod
    def object_position(
        self,
        object_name: str,
        frame: str
    ) -> np.ndarray:
        """Return the position of object in reference frame."""
        pass

    @abstractmethod
    def set_gripper(self, state: str):
        """Set gripper state to state ("open"/"closed")."""
        pass


class MobileBaseInterface(ABC):
    """
    Base class defining methods that must exist in the world interface of the MobileBase.

    Each action retruns an instance of Task.
    """

    @abstractmethod
    def move(self, action: str):
        """Move the robot according to action."""
        pass

    @abstractmethod
    def robot_position(self, frame: str):
        """Return the robot position in frame."""
        pass

    @abstractmethod
    def approach(
        self,
        target: np.ndarray,
        frame: str
    ):
        """Move closer to target so it is reachable with the gripper."""
        pass

    @abstractmethod
    def reachable(
        self,
        target: str,
        frame: str = None
    ):
        """
        Check if the gripper can reach target.

        If target is a string it should be the name of an object,
        in which case the frame argument is ignored.
        """
        pass
