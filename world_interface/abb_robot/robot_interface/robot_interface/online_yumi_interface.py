"""Interface to communicate with YuMi robot through RWS."""

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

from threading import Thread
import time

from abb_rapid_sm_addin_msgs.srv import SetRAPIDRoutine, SetSGCommand
from abb_robot_msgs.msg import SystemState
from abb_robot_msgs.srv import GetIOSignal,  GetRAPIDBool, SetRAPIDSymbol, TriggerWithResultCode
import numpy as np
from perception_utils.homogeneous_matrix import homogeneous_matrix
import perception_utils.transformations as tf_utils 
import rclpy
from rclpy.node import Node
import robot_interface.interface as wi
import robot_interface.rapid_interface as rapid


class OnlineYuMiInterface(wi.YuMiInterface):
    """Interface to the physical YuMi robot."""

    GRIPPER_OPEN_THRESHOLD = 230.0
    GRIPPER_COMPLETELY_CLOSED_THRESHOLD = 10.0

    def __init__(
        self,
        node: Node,
        base_frame: str,
        namespace: str = ''
    ):
        """
        Initialize the communication between ROS and RAPID for the YuMi robot.

        Args:
        ----
            node: ROS node that is spinning.
            base_frame: base frame of the robot.
            namespace: node namespace for creating ROS clients and subscriptions.

        """
        # Access the function lookup_transform()
        # It makes sense to attribute it to a variable tf_listener,
        # but it is actually the node buffer that has such method!
        self.tf_listener = node.tf_buffer
        self.node = node
        self.base_frame = base_frame

        self.ns = namespace

        self.state = rapid.State()
        # TODO: add support for both Right and Left gripper
        self.gripper_running = True
        self.gripper_value = [0, 0]

        self.task_status = 'INVALID'
        self.rapid_running = False
        self.motors_on = False

        # TODO: not a good idea to have all these hard coded.
        # TODO: initialize the interface with a dictionary of topics and services.
        # TODO: maybe create a bool variable for every service in a config file.

        # RWS topics
        self.system_states_sub = node.create_subscription(
            SystemState, self.ns + '/robot/rws/system_states', self.system_states_cb, 10)
        # RWS services
        self.get_io_signal_srv = node.create_client(
            GetIOSignal, self.ns + '/robot/rws/get_io_signal')
        self.set_sg_command_srv = node.create_client(
            SetSGCommand, self.ns + '/robot/rws/sm_addin/set_sg_command')
        self.run_sg_routine_srv = node.create_client(
            TriggerWithResultCode, self.ns + '/robot/rws/sm_addin/run_sg_routine')
        self.set_rapid_routine_srv = node.create_client(
            SetRAPIDRoutine, self.ns + '/robot/rws/sm_addin/set_rapid_routine')
        self.run_rapid_routine_srv = node.create_client(
            TriggerWithResultCode, self.ns + '/robot/rws/sm_addin/run_rapid_routine')
        self.set_rapid_symbol_srv = node.create_client(
            SetRAPIDSymbol, self.ns + '/robot/rws/set_rapid_symbol')
        self.stop_rapid_srv = node.create_client(
            TriggerWithResultCode, self.ns + '/robot/rws/stop_rapid')
        self.start_rapid_srv = node.create_client(
            TriggerWithResultCode, self.ns + '/robot/rws/start_rapid')
        self.pp_to_main_srv = node.create_client(
            TriggerWithResultCode, self.ns + '/robot/rws/pp_to_main')
        self.get_rapid_bool_srv = node.create_client(
            GetRAPIDBool, self.ns + '/robot/rws/get_rapid_bool')
        # Other services
        # self.grasping_srv = node.create_client(
        #     GenerateGraspingPoint, self.ns + '/robot/compute_grasping')
        # self.obj_recognition_srv = node.create_client(
        #     GetObjects, self.ns + '/sensors/camera/get_objects')
        # TODO: add navigation stuff

        while not self.get_io_signal_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/get_io_signal...')
        while not self.set_sg_command_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/set_sg_command...')
        while not self.run_sg_routine_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/run_sg_routine...')
        while not self.set_rapid_routine_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/set_rapid_routine')
        while not self.run_rapid_routine_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/run_rapid_routine')
        while not self.set_rapid_symbol_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/set_rapid_symbol')
        while not self.stop_rapid_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/stop_rapid')
        while not self.start_rapid_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/start_rapid')
        while not self.pp_to_main_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/pp_to_main')
        while not self.get_rapid_bool_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/rws/get_rapid_bool')
        # while not self.grasping_srv.wait_for_service(timeout_sec=1.0):
            # self.node.get_logger().warn('Waiting for ' + self.ns + '/robot/compute_grasping')
        # while not self.obj_recognition_srv.wait_for_service(timeout_sec=1.0):
            # self.node.get_logger().warn('Waiting for ' + self.ns + '/sensors/camera/get_objects')

        self.gripper_thread = Thread(target=self.__cache_gripper_value)
        self.gripper_thread.start()

    def __del__(self):
        """Destroy clients and subscriptions."""
        self.gripper_running = False
        self.node.destroy_client(self.get_io_signal_srv)
        self.node.destroy_client(self.set_sg_command_srv)
        self.node.destroy_client(self.run_sg_routine_srv)
        self.node.destroy_client(self.set_rapid_routine_srv)
        self.node.destroy_client(self.run_rapid_routine_srv)
        self.node.destroy_client(self.set_rapid_symbol_srv)
        self.node.destroy_client(self.stop_rapid_srv)
        self.node.destroy_client(self.start_rapid_srv)
        self.node.destroy_client(self.pp_to_main_srv)
        self.node.destroy_client(self.get_rapid_bool_srv)
        self.node.destroy_subscription(self.system_states_sub)

    def __cache_gripper_value(self):
        """Handle the communication with the gripper states."""
        while self.gripper_running:
            request = GetIOSignal.Request()

            request.signal = 'hand_ActualPosition_R'
            response = self.get_io_signal_srv.call(request)
            self.gripper_value[0] = float(response.value)

            if response.result.message != '':
                self.node.get_logger().error(
                    f'GetIOSignal responded with message "{response.result.message}"\
                    and result_code "{response.result.code}"'
                )

            request.signal = 'hand_ActualPosition_L'
            response = self.get_io_signal_srv.call(request)
            self.gripper_value[1] = float(response.value)

            if response.result.message != '':
                self.node.get_logger().error(
                    f'GetIOSignal responded with message "{response.result.message}"\
                    and result_code "{response.result.code}"'
                )

            time.sleep(0.1)

    def system_states_cb(self, msg: SystemState):
        """Check RAPID status, reurns if RUNNING true or false."""
        self.rapid_running = msg.rapid_running
        self.motors_on = msg.motors_on

    def get_task_status(self) -> str:
        """Reurn status of the task (RUNNING, SUCCESS, FAILURE, INVALID)."""
        if self.rapid_running:
            self.task_status = 'RUNNING'
        else:
            if rapid.RAPIDTask.get_rapid_status() != 'RUNNING' and\
               rapid.RAPIDTask.get_rapid_status() != 'INVALID':
                # disregard motors status (RAPIDTask calls stop rapid as well)
                # then we should have SUCCESS or FAILURE from RAPID side
                self.task_status = rapid.RAPIDTask.get_rapid_status()
            elif not self.motors_on:
                self.task_status = 'FAILURE'

        return self.task_status

    def restart_rapid(self):
        """Start the RAPID StateMachine."""
        self.stop_rapid_srv.call(TriggerWithResultCode.Request())
        time.sleep(0.2)
        self.pp_to_main_srv.call(TriggerWithResultCode.Request())
        time.sleep(0.2)
        self.start_rapid_srv.call(TriggerWithResultCode.Request())
        time.sleep(0.3)

    def move_joint(
        self,
        joints: str,
        rob_task: str = 'T_ROB_R'
    ) -> Thread:
        """
        Initiate a moving task in joint space.

        Args
        ----
            joints: joint values in RAPID format
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            move_joint: thread running the pick task in RAPID.

        """
        move_joint = rapid.MoveJTask(
            joints, self.state, self.stop_rapid_srv, self.start_rapid_srv, self.pp_to_main_srv,
            self.set_rapid_routine_srv, self.run_rapid_routine_srv, self.set_rapid_symbol_srv,
            self.get_rapid_bool_srv, rob_task
        )
        return move_joint

    def place(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        frame: str,
        tool: str = 'gripper',
        rob_task: str = 'T_ROB_R'
    ) -> Thread:
        """
        Initiate a picking task.

        If position and orientation are not given, the place pose is intended as defined
        by the object described in the frame.

        Args
        ----
            position: position of the object to pick.
            orientation: orientation of the object to pick.
            frame: frame that determines the place pose of the object.
            tool: either gripper or suction cup.
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            place_task: thread running the pick task in RAPID.

        """
        if position is None and orientation is None:
            # Place in specified frame
            now = rclpy.time.Time()
            tf_msg = self.tf_listener.lookup_transform(self.base_frame, frame, now)
            # TODO: transform this to np array
            position_msg = tf_msg.transform.translation
            position = np.array([position_msg.x, position_msg.y, position_msg.z])
            orientation_msg = tf_msg.transform.rotation
            orientation = np.array(
                [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w])

        # Transform in robot base frame before sending to RAPID
        rapid_pos = self.transform_position(self.base_frame, frame, position)
        rapid_orient = self.transform_orientation(self.base_frame, frame, orientation)

        place_task = rapid.PlaceTask(
            rapid_pos, rapid_orient, tool, self.state, self.stop_rapid_srv, self.start_rapid_srv,
            self.pp_to_main_srv, self.set_rapid_routine_srv, self.run_rapid_routine_srv,
            self.set_rapid_symbol_srv, self.get_rapid_bool_srv, rob_task
        )
        return place_task

    def pick(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        frame: str,
        tool: str = 'gripper',
        rob_task: str = 'T_ROB_R'
    ) -> Thread:
        """
        Initiate a picking task.

        If position and orientation are not given, the pose of the object to pick is retrieved
        computing the transformation between the frame of the object and a reference frame.

        Args
        ----
            position: position of the object to pick.
            orientation: orientation of the object to pick.
            frame: frame that determines the pose of the object.
            tool: either gripper or suction cup.
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            pick_task: thread running the pick task in RAPID.

        """
        if position is None and orientation is None:
            # Pick in specified frame
            now = rclpy.time.Time()
            tf_msg = self.tf_listener.lookup_transform(self.base_frame, frame, now)
            position_msg = tf_msg.transform.translation
            position = np.array([position_msg.x, position_msg.y, position_msg.z])
            orientation_msg = tf_msg.transform.rotation
            orientation = np.array(
                [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w])

        # Transform in robot base frame before sending to RAPID
        rapid_pos = self.transform_position(self.base_frame, frame, position)
        rapid_orient = self.transform_orientation(self.base_frame, frame, orientation)

        pick_task = rapid.PickTask(
            rapid_pos, rapid_orient, tool, self.state, frame, self.stop_rapid_srv,
            self.start_rapid_srv, self.pp_to_main_srv, self.set_rapid_routine_srv,
            self.run_rapid_routine_srv, self.set_rapid_symbol_srv, self.get_rapid_bool_srv,
            rob_task
        )
        return pick_task

    def in_gripper(
        self,
        held_object: str,
        rob_task: str = 'T_ROB_R'
    ) -> bool:
        """
        Return True if the robot is holding object.

        Args
        ----
            object: name of the frame that determines the object pose.
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            in_gripper: True if the robot is holding the specified object.

        """
        holding = self.state.holding

        index = 0
        if rob_task == 'T_ROB_L':
            index = 1

        in_gripper = holding == held_object and\
            self.gripper_value[index] > self.GRIPPER_COMPLETELY_CLOSED_THRESHOLD and\
            self.is_gripper_state('closed', index)
        return in_gripper

    def is_gripper_state(
        self,
        state: str,
        index: int = 0
    ) -> bool:
        """Return True if the gripper state is equal to the given state."""
        current_state = ''

        if self.gripper_value[index] > self.GRIPPER_OPEN_THRESHOLD:
            current_state = 'open'
        else:
            current_state = 'closed'

        return current_state == state

    def empty_gripper(
        self,
        rob_task: str = 'T_ROB_R'
    ) -> bool:
        """
        Return True if the robot is not holding anything.

        Note that the gripper can be empty while closed.

        Args
        ----
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            emtpy: True if the robot is not holding anything.

        """
        holding = self.state.holding

        index = 0
        if rob_task == 'T_ROB_L':
            index = 1

        empty = holding == '' or\
            self.gripper_value[index] < self.GRIPPER_COMPLETELY_CLOSED_THRESHOLD or\
            self.is_gripper_state('open', index)
        return empty

    def object_position(
        self,
        object_frame: str,
        reference_frame: str
    ) -> np.ndarray:
        """
        Return the position of object in reference frame.

        Args
        ----
            object_frame: target frame (object frame or frame in which the action is defined).
            reference_frame: frame with respect to which the transform is computed.

        Returns
        -------
            position: position of the object in the reference frame.

        """
        now = rclpy.time.Time()
        tf_msg = self.tf_listener.lookup_transform(reference_frame, object_frame, now)
        position_msg = tf_msg.transform.translation
        position = np.array([position_msg.x, position_msg.y, position_msg.z])
        return position

    def transform_position(
        self,
        source_frame: str,
        target_frame: str,
        pos: np.ndarray
    ) -> np.ndarray:
        """Transform pos from source_frame to target_frame."""
        # This gets a TransformStamped!
        now = rclpy.time.Time()
        tf_msg = self.tf_listener.lookup_transform(source_frame, target_frame, now)
        transform = tf_msg.transform
        tf_matrix = homogeneous_matrix(transform.translation, transform.rotation)
        translation_matrix = tf_utils.translation_matrix(pos)
        trans_mat_in_frame = tf_matrix @ translation_matrix
        new_position = tf_utils.translation_from_matrix(trans_mat_in_frame)
        return new_position

    def transform_orientation(
        self,
        source_frame: str,
        target_frame: str,
        orientation: np.ndarray
    ) -> np.ndarray:
        """Transform orientation from source_frame to target_frame"""
        # This gets a TransformStamped!
        now = rclpy.time.Time()
        tf_msg = self.tf_listener.lookup_transform(source_frame, target_frame, now)
        transform = tf_msg.transform
        tf_matrix = homogeneous_matrix(transform.translation, transform.rotation)
        orientation_matrix = tf_utils.quaternion_matrix(orientation)
        trans_mat_in_frame = tf_matrix @ orientation_matrix
        new_orientation = tf_utils.quaternion_from_matrix(trans_mat_in_frame)
        return new_orientation

    def set_gripper(
        self,
        state: str,
        rob_task: str = 'T_ROB_R'
    ) -> Thread:
        """
        Set gripper state to state ("open"/"closed").

        Args
        ----
            state: value for the gripper.
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            move_gripper: thread that triggers Gripper movement in RAPID.

        """
        move_gripper = rapid.GripperTask(
            state, self.state, self.set_sg_command_srv, self.run_sg_routine_srv,
            self.stop_rapid_srv, self.start_rapid_srv, self.pp_to_main_srv,
            self.set_rapid_routine_srv, self.run_rapid_routine_srv, self.set_rapid_symbol_srv,
            self.get_rapid_bool_srv, rob_task
        )
        return move_gripper

    def home(
        self,
        rob_task: str = 'T_ROB_R'
    ) -> Thread:
        """
        Move arm to home position.

        Args
        ----
            rob_task: one of the two robot arms in RAPID.

        Returns
        -------
            home_task: thread that executes the Home Task in RAPID.

        """
        home_task = rapid.HomeTask(
            self.stop_rapid_srv, self.start_rapid_srv, self.pp_to_main_srv,
            self.set_rapid_routine_srv, self.run_rapid_routine_srv,
            self.set_rapid_symbol_srv, self.get_rapid_bool_srv, rob_task
        )
        return home_task

    def set_leadthrough(
        self,
        value: str,
        rob_task: str = 'T_ROB_R'
    ):
        """
        Set leadthrough on/off.

        Args:
        ----
            value: On or Off.
            rob_task: one of the two robot arms in RAPID.

        """
        routine_request = SetRAPIDRoutine.Request()
        routine_request.task = rob_task
        routine_request.routine = 'setLeadthrough' + value.title()
        response = self.set_rapid_routine_srv.call(routine_request)
        if response.result.message != '':
            raise RuntimeError(
                f'set_rapid_routine returned message "{response.result.message}"\
                and code "{response.result.code}"'
            )
        self.run_rapid_routine_srv.call(TriggerWithResultCode.Request())
