"""
Simulator for the LfD framework.
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

# TODO: add simulated behaviors
# TODO: add support for simulated camera interface (markers or objects)
import os
import yaml
import numpy as np
from copy import deepcopy

class Simulator:
    """
    Generates simulated data for pick and place tasks.
    """

    def __init__(self, folder, objects, initial=None, default_frame='map'):
        """
        Instantiate a new simulator. The demonstration will be stored in folder.
        The default frame is base and one frame is assigned for each object. If
        initial is given, it is a dictionary where each key is an object name and
        each value is the object's initial position in default frame. If initial
        is None, the objects are intitialized at random positions. No orientation
        information is currently simulated.
        """
        self.__objects = objects
        self.__folder = folder
        self.__current_demo = 0
        self.__current_action = 1
        self.__states = {}
        self.__holding = ''
        self.__initial = initial
        self.__all_frames = ['map', 'base'] + objects
        self.__default_frame = default_frame

        os.mkdir(folder)
        with open(folder + '/info.yaml', 'w') as f:
            f.write(yaml.dump({
                'frames': self.__all_frames,
                'default_frame': default_frame
            }))

    def begin_demonstration(self):
        """Begins a new demonstration"""
        self.__current_demo += 1
        self.__current_action = 1

        if self.__initial is None:
            # Initialize state randomly
            for obj in self.__objects + ['base']:
                position = np.random.uniform(-10, 10, (3,))
                if obj == 'base':
                    position[2] = 0.5
                self.__states[obj] = position
        else:
            # Use initial state
            self.__states = deepcopy(self.__initial)
            # Initialized unspecified frames randomly
            for obj in self.__objects + ['base']:
                if obj not in self.__states:
                    position = np.random.uniform(-10, 10, (3,))
                    if obj == 'base':
                        position[2] = 0.5
                    self.__states[obj] = position
                else:
                    self.__states[obj] += 0.01*np.random.randn(3)

        self.__holding = ''

        os.mkdir('{}/demo{}'.format(self.__folder, self.__current_demo))

    def pick(self, object, variant=0):
        data = {
            'type': 'pick',
            'vec_pos': {},
            'vec_quat': {},
            'variant': variant
        }
        # No orientation information
        for frame in self.__all_frames:
            data['vec_quat'][frame] = [0, 0, 0, 1]

        # Picking location in map frame with noise
        location = self.__states[object] + 0.01*np.random.randn(3)
        data['vec_pos']['map'] = location.tolist()

        # Translate to other frames
        for obj in self.__objects + ['base']:
            data['vec_pos'][obj] = (location - self.__states[obj]).tolist()

        # Update state
        self.__holding = object

        # Save action
        with open('{}/demo{}/data_{}.yaml'.format(self.__folder, self.__current_demo, self.__current_action), 'w') as f:
            f.write(yaml.dump(data, default_flow_style=None))

        self.__current_action += 1

    def place(self, target, frame, variant=0):
        if self.__holding == '':
            raise ValueError('The simulated robot is not holding anything to place')

        if type(target) != np.ndarray:
            target = np.array(target, dtype=np.float64)

        data = {
            'type': 'place',
            'vec_pos': {},
            'vec_quat': {},
            'variant': variant
        }
        # No orientation information
        for f in self.__all_frames:
            data['vec_quat'][f] = [0, 0, 0, 1]

        # Translate target to map frame
        if frame == 'map':
            location = target
        else:
            location = target + self.__states[frame]

        # Add noise
        location += 0.01*np.random.randn(3)
        data['vec_pos']['map'] = location.tolist()

        # Translate to other frames
        for obj in self.__objects + ['base']:
            if obj == self.__holding:
                # We are placing obj so in this frame it is close to zero
                data['vec_pos'][obj] = (0.01*np.random.randn(3)).tolist()
            else:
                data['vec_pos'][obj] = (location - self.__states[obj]).tolist()

        # Update internal state
        self.__states[self.__holding] = location
        self.__holding = ''

        # Save action
        with open('{}/demo{}/data_{}.yaml'.format(self.__folder, self.__current_demo, self.__current_action), 'w') as f:
            f.write(yaml.dump(data, default_flow_style=None))

        self.__current_action += 1

    def move_to(self, target, frame, variant=0):
        if type(target) != np.ndarray:
            target = np.array(target, dtype=np.float64)

        data = {
            'type': 'move',
            'vec_pos': {},
            'vec_quat': {},
            'variant': 0
        }

        # No orientation information
        for f in self.__all_frames:
            data['vec_quat'][f] = [0, 0, 0, 1]

        # Translate target to map frame
        if frame == 'map':
            location = target
        else:
            location = target + self.__states[frame]

        # Add noise
        location += 0.01*np.random.randn(3)
        data['vec_pos']['map'] = location.tolist()

        # Translate to other frames
        for obj in self.__objects + ['base']:
            if obj == 'base':
                # We are moving base so in this frame it is close to zero
                data['vec_pos'][obj] = (0.01*np.random.randn(3)).tolist()
            else:
                data['vec_pos'][obj] = (location - self.__states[obj]).tolist()

        # Update internal state
        self.__states['base'] = location

        # Save action
        with open('{}/demo{}/data_{}.yaml'.format(self.__folder, self.__current_demo, self.__current_action), 'w') as f:
            f.write(yaml.dump(data, default_flow_style=None))
            
        self.__current_action += 1

    def place_waypoint(self, target1, frame1, target2, frame2, variant=0):
        # Perform an action with two targets. Place object at target2 after
        # going through target1.
        data = {
            'type': 'place_waypoint',
            'vec_pos': {},
            'vec_quat': {},
            'variant': 0
        }
        # No orientation information
        for frame in self.__all_frames:
            data['vec_quat'][frame] = [0, 0, 0, 1, 0, 0, 0, 1]

        # Translate target1 to map frame
        if frame1 == 'map':
            location1 = target1
        else:
            location1 = target1 + self.__states[frame1]
        # Add noise
        location1 += 0.01*np.random.randn(3)
        data['vec_pos']['map'] = location1.tolist()

        # Translate to other frames
        for obj in self.__objects + ['base']:
            if obj == self.__holding:
                # We are moving obj so in this frame it is close to zero
                data['vec_pos'][obj] = (0.01*np.random.randn(3)).tolist()
            else:
                data['vec_pos'][obj] = (location1 - self.__states[obj]).tolist()

        # Translate target2 to map frame
        if frame2 == 'map':
            location2 = target2
        else:
            location2 = target2 + self.__states[frame2]
        # Add noise
        location2 += 0.01*np.random.randn(3)
        data['vec_pos']['map'] += location2.tolist()

        # Translate to other frames
        for obj in self.__objects + ['base']:
            if obj == self.__holding:
                # We are placing obj so in this frame it is close to zero
                data['vec_pos'][obj] += (0.01*np.random.randn(3)).tolist()
            else:
                data['vec_pos'][obj] += (location2 - self.__states[obj]).tolist()

        # Update internal state
        self.__states[self.__holding] = location2

        # Save action
        with open('{}/demo{}/data_{}.yaml'.format(self.__folder, self.__current_demo, self.__current_action), 'w') as f:
            f.write(yaml.dump(data, default_flow_style=None))

        self.__current_action += 1