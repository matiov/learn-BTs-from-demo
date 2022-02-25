"""RAPID-ROS Interface to communicate with YuMi robot through RWS."""

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

import threading
import time

from abb_rapid_sm_addin_msgs.srv import SetRAPIDRoutine, SetSGCommand
from abb_robot_msgs.srv import GetRAPIDBool, SetRAPIDSymbol, TriggerWithResultCode
import numpy as np
from rclpy.client import Client
import robot_interface.interface as wi
from scipy.spatial.transform import Rotation


class RAPIDTask(wi.Task, threading.Thread):
    """General RAPID interface initializing ROS services."""

    def __init__(
        self,
        rapid_stop: Client,
        rapid_start: Client,
        pp_to_main: Client,
        rapid_set: Client,
        rapid_run: Client,
        rapid_symbol: Client,
        rapid_bool: Client,
        rob_task: str = 'T_ROB_R'
    ):
        """
        Initialize a RAPID task with by linking the ROS clients.

        Args:
        ----
            rapid_stop: Client that stops the RAPID routine.
            rapid_start: Client that starts the RAPID routine.
            pp_to_main: : Client that resets the ProgramPointer in the RAPID routine.
            rapid_set: Client that sets the routine to run.
            rapid_run: Client that runs the RAPID routine.
            rapid_symbol: Client that sets a RAPID symbol.
            rapid_bool: Client that sets a RAPID bool.
            rob_task: Left or Right arm of the robot in RAPID language.

        """
        super().__init__()
        self.rapid_stop = rapid_stop
        self.rapid_start = rapid_start
        self.pp_to_main = pp_to_main
        self.rapid_set = rapid_set
        self.rapid_run = rapid_run
        self.rapid_symbol = rapid_symbol
        self.rapid_bool = rapid_bool
        self.rob_task = rob_task
        self.preempted = False
        self.rapid_status = 'INVALID'

    def initialize(self):
        """Initialize RAPID with default symbols."""
        self.rapid_status = 'RUNNING'
        self.rapid_stop.call(TriggerWithResultCode.Request())
        time.sleep(0.2)
        self.pp_to_main.call(TriggerWithResultCode.Request())
        time.sleep(0.2)
        self.rapid_start.call(TriggerWithResultCode.Request())
        time.sleep(0.3)

        symbol_request = SetRAPIDSymbol.Request()
        symbol_request.path.task = self.rob_task
        symbol_request.path.module = 'TRobRAPID'
        symbol_request.path.symbol = 'routine_completed'
        symbol_request.value = 'FALSE'
        response = self.rapid_symbol.call(symbol_request)
        if response.result.message != '':
            print(f'SetRAPIDSymbol returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

    def terminate(self):
        self.preempted = True

    def wait_for_rapid(self):
        """Wait for RAPID to complete the routine."""
        while not self.preempted and not self.rapid_done():
            self.rapid_status = 'RUNNING'
            time.sleep(0.1)
        if self.rapid_done():
            self.rapid_status = 'SUCCESS'

    def rapid_done(self) -> bool:
        """
        Return whether RAPID has finished the execution.

        Returns
        -------
            done: value of the request.

        """
        request = GetRAPIDBool.Request()
        request.path.task = self.rob_task
        request.path.module = 'TRobRAPID'
        request.path.symbol = 'routine_completed'

        done = self.rapid_bool.call(request).value
        return done

    def get_rapid_status(self) -> str:
        """
        Get the RAPID status according to Behavior Tree return statuses.

        Returns
        -------
            return_msg: one of 'RUNNING', 'SUCCESS', 'FAILURE', 'INVALID'.

        """
        return_msg = self.rapid_status
        return return_msg

    def set_rapid_tool(self, tool: str):
        """
        Set the tool in RAPID.

        Args:
        ----
            tool: name of the tool to set, either gripper or suction_cup.

        """
        symbol_request = SetRAPIDSymbol.Request()
        symbol_request.path.task = self.rob_task
        symbol_request.path.module = 'TRobUtility'
        symbol_request.path.symbol = 'current_tool'
        if tool == 'suction_cup':
            symbol_request.value = '[TRUE, [[77, 20, 39], [1, 0, 0 ,0]],\
                [0.248, [8.6, 11.7, 52.7], [1, 0, 0, 0], 0.00021, 0.00024, 0.00009]]'
        else:
            symbol_request.value = '[TRUE, [[0, 0, 138], [1, 0, 0 ,0]],\
                [0.248, [8.6, 11.7, 52.7], [1, 0, 0, 0], 0.00021, 0.00024, 0.00009]]'
        response = self.rapid_symbol.call(symbol_request)
        if response.result.message != '':
            print(f'SetRAPIDSymbol returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

    def set_robtarget(
        self,
        position: np.ndarray,
        orientation: np.ndarray
    ):
        """
        Set a pose target for the robot.

        Args:
        ----
            position: target position value in [mm].
            orientation: targert orientation value.

        """
        position_str = str((position).tolist())

        if self.tool == 'suction_cup':
            if self.rob_task == 'T_ROB_R':
                orientation_str = '[0.5, -0.5, 0.5, 0.5]'
            else:
                orientation_str = '[0.5, 0.5, 0.5, -0.5]'
        else:
            # get rotation around Y axis from quaternion
            angle = Rotation.from_quat(orientation).as_euler('XYZ')[2]
            print('Rotation angle: ' + str(angle))

            if self.rob_task == 'T_ROB_R':
                if angle > np.pi:
                    angle -= 2*np.pi
                elif angle < -np.pi:
                    angle += 2*np.pi
            else:
                if angle > 1.5*np.pi:
                    angle -= 2*np.pi
                elif angle < -0.5*np.pi:
                    angle += 2*np.pi

            print('Normalized angle: ' + str(angle) + ' for ' + str(self.rob_task))
            rotation_quaternion = Rotation.from_euler('z', angle)
            print('Angle as quaternion: ' + str(rotation_quaternion.as_quat()))

            yumi_gripper_rotation_x = -0.707107
            yumi_gripper_rotation_y = 0.707107
            yumi_gripper_rotation_z = 0.0
            yumi_gripper_rotation_w = 0.0

            rotation_YuMi = Rotation.from_quat(
                [
                    yumi_gripper_rotation_x,
                    yumi_gripper_rotation_y,
                    yumi_gripper_rotation_z,
                    yumi_gripper_rotation_w
                ]
            )
            orientation = (rotation_quaternion*rotation_YuMi).as_quat()
            orientation_yumi_format = [
                orientation[3], orientation[0], orientation[1], orientation[2]]
            orientation_str = str(orientation_yumi_format)

        if self.rob_task == 'T_ROB_R':
            if self.tool == 'suction_cup':
                config = '[1, 1, -1, 4]'
            else:
                config = '[1, -1, -1, 4]'
        else:
            if self.tool == 'suction_cup':
                config = '[-1, 1, 1, 4]'
            else:
                config = '[-1, 1, -1, 4]'

        print('Tool: ' + str(self.tool))
        print('RobTask: ' + str(self.rob_task))

        symbol_request = SetRAPIDSymbol.Request()
        symbol_request.path.task = self.rob_task
        symbol_request.path.module = 'TRobRAPID'
        symbol_request.path.symbol = 'move_robtarget_input'
        symbol_request.value = '[%s, %s, %s, [180, 9E9, 9E9, 9E9, 9E9, 9E9]]' %\
            (position_str, orientation_str, config)
        print('REQUEST', symbol_request.value)
        response = self.rapid_symbol.call(symbol_request)
        if response.result.message != '':
            print(f'SetRAPIDSymbol for RobotTarget returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

    def set_jointtarget(self, joints: str):
        """
        Set a joint target for the robot.

        Args:
        ----
            joints: target joint value.

        """
        symbol_request = SetRAPIDSymbol.Request()
        symbol_request.path.task = self.rob_task
        symbol_request.path.module = 'TRobRAPID'
        symbol_request.path.symbol = 'move_jointtarget_input'
        symbol_request.value = joints

        response = self.rapid_symbol.call(symbol_request)
        if response.result.message != '':
            print(f'SetRAPIDSymbol for JointTarget returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

    def execute_rapid(self, routine: str):
        """
        Execute the target RAPID routine.

        Args:
        ----
            routine: name of the routine in RAPID.

        """
        routine_request = SetRAPIDRoutine.Request()
        routine_request.task = self.rob_task
        routine_request.routine = routine
        response = self.rapid_set.call(routine_request)
        if response.result.message != '':
            print(f'SetRAPIDRoutine returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'
        self.rapid_run.call(TriggerWithResultCode.Request())


class HomeTask(RAPIDTask):
    """Move arm to home position."""

    def __init__(self, *args):
        super().__init__(*args)

    def run(self):
        self.initialize()

        symbol_request = SetRAPIDSymbol.Request()
        symbol_request.path.task = self.rob_task
        symbol_request.path.module = 'TRobRAPID'
        symbol_request.path.symbol = 'move_jointtarget_input'
        if self.rob_task == 'T_ROB_R':
            symbol_request.value = '[[49, -66, 13, -94, 87, -90], [-34, 9E9, 9E9, 9E9, 9E9, 9E9]]'
        else:
            symbol_request.value = '[[-49, -66, 13, 94, 87, -90], [34, 9E9, 9E9, 9E9, 9E9, 9E9]]'
        response = self.rapid_symbol.call(symbol_request)
        if response.result.message != '':
            print(f'SetRAPIDSymbol returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

        symbol_request = SetRAPIDSymbol.Request()
        symbol_request.path.task = self.rob_task
        symbol_request.path.module = 'TRobRAPID'
        symbol_request.path.symbol = 'move_speed_input'
        symbol_request.value = '[500, 500, 5000, 1000]'
        response = self.rapid_symbol.call(symbol_request)
        if response.result.message != '':
            print(f'SetRAPIDSymbol returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

        self.execute_rapid('runMoveAbsJ')
        self.wait_for_rapid()


class GripperTask(RAPIDTask):
    """Set target to the grippers."""

    def __init__(
        self,
        state: str,
        robot_state: 'State',
        client_set: Client,
        client_run: Client,
        *args
    ):
        """
        Initialize the gripper task setting the ROS services and states.

        Args:
        ----
            state: target state for the gripper (open/close).
            robot_state: robot internal state defining if it's picking an object
                or executing a blocking task.
            client_set: name of the Client setting the gripper state.
            client_run: name of the Client running the gripper task.
            *args: other ROS clients for the RAPID class.

        """
        super().__init__(*args)
        self.state = state
        self.robot_state = robot_state
        self.client_set = client_set
        self.client_run = client_run

    def run(self):
        self.initialize()

        request = SetSGCommand.Request()
        request.command = 6 if self.state == 'closed' else 7
        request.task = self.rob_task
        response = self.client_set.call(request)

        if response.result.message != '':
            print(f'SetSGCommand returned message "{response.result.message}"\
                and code "{response.result.code}"')
            self.rapid_status = 'FAILURE'

        self.client_run.call(TriggerWithResultCode.Request())

        if self.state == 'open':
            self.robot_state.holding = ''


class MoveJTask(RAPIDTask):
    """Move robot Joints."""

    def __init__(
        self,
        joints: str,
        robot_state: 'State',
        *args
    ):
        """
        Initialize the task.

        Args:
        ----
            joints: joints to move.
            robot_state: robot internal state defining if it's picking an object
                or executing a blocking task.
            *args: other ROS clients for the RAPID class.

        """
        super().__init__(*args)

        self.joints = joints
        self.state = robot_state

    def run(self):
        print('Running')
        self.initialize()

        self.state.blocking = True

        print('Moving to', self.joints)
        self.set_jointtarget(self.joints)
        self.execute_rapid('runMoveAbsJ')

        self.wait_for_rapid()
        print('Rapid done!')
        self.state.holding = ''
        self.state.blocking = False
        self.state.blocked_conditions.clear()


class PlaceTask(RAPIDTask):
    """RAPID interface for the placing task."""

    def __init__(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        tool: str,
        robot_state: 'State',
        *args
    ):
        """
        Position and orientation are in base frame.

        Args:
        ----
            position: given by ROS in [m], it is converted in [mm] and sent to RAPID.
            orientation: orientation of the object to pick.
            tool: picking tool.
            robot_state: robot internal state defining if it's picking an object
                or executing a blocking task.
            object: object to pick (defined by its frame).
            *args: other ROS clients for the RAPID class.

        """
        super().__init__(*args)

        self.position = position*1e3 + np.array([0.0, 0.0, 0.0])
        self.orientation = orientation
        self.state = robot_state
        self.tool = tool

    def run(self):
        self.initialize()

        self.state.blocking = True

        print('Placing at', self.position, '[m] orientation', self.orientation)
        self.set_rapid_tool(self.tool)
        self.set_robtarget(self.position, self.orientation)
        routine = 'place_vacuum' if self.tool == 'suction_cup' else 'place_gripper'
        self.execute_rapid(routine)

        self.wait_for_rapid()

        self.state.holding = ''
        self.state.blocking = False
        self.state.blocked_conditions.clear()


class PickTask(RAPIDTask):
    """RAPID interface for the picking task."""

    def __init__(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        tool: str,
        robot_state: 'State',
        target_object: str,
        *args
    ):
        """
        Position and orientation are in base frame.

        Args:
        ----
            position: given by ROS in [m], it is converted in [mm] and sent to RAPID.
            orientation: orientation of the object to pick.
            tool: picking tool.
            robot_state: robot internal state defining if it's picking an object
                or executing a blocking task.
            target_object: object to pick (defined by its frame).
            *args: other ROS clients for the RAPID class.

        """
        super().__init__(*args)

        self.position = position*1e3
        self.orientation = orientation
        self.state = robot_state
        self.object = target_object
        self.tool = tool

    def run(self):
        print('Inside RAPID task')
        self.initialize()

        self.state.blocking = True

        print('Picking', self.object, 'at', self.position, '[m] orientation', self.orientation)
        self.set_rapid_tool(self.tool)
        self.set_robtarget(self.position, self.orientation)
        routine = 'pick_vacuum' if self.tool == 'suction_cup' else 'pick_gripper'
        self.execute_rapid(routine)

        self.wait_for_rapid()

        self.state.holding = self.object
        self.state.blocking = False


class State:
    """
    Represent part of the robot's internal state that cannot be measured.

    Access to the state is thread safe.
    The state can only be accessed within a "with" statement.
    """

    def __init__(self):
        self.__holding = ''
        self.__blocking = False
        self.__blocked_conditions = []

    @property
    def holding(self):
        return self.__holding

    @holding.setter
    def holding(self, value):
        self.__holding = value

    @property
    def blocking(self):
        return self.__blocking

    @blocking.setter
    def blocking(self, value):
        self.__blocking = value

    @property
    def blocked_conditions(self):
        return self.__blocked_conditions

    @blocked_conditions.setter
    def blocked_conditions(self, value):
        self.__blocked_conditions = value
