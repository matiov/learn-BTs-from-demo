"""Test the clustering script."""

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

import bt_learning.learning_from_demo.clustering as cl
from bt_learning.learning_from_demo.demonstration import EquivalentAction
from bt_learning.learning_from_demo.tests.util import TESTDIR
import numpy as np
from robot_behaviors.yumi_behaviors.lfd_actions import PickAction, PlaceAction
from robot_interface.demonstration import RobotAction, RobotDemonstration
from simulation.learning_from_demo.lfd_simulator import Simulator


class PlaceWaypointAction(RobotAction):
    """Class representing an action of placing an object by passing through a waypoint."""

    def __init__(self, data, frames, default_frame, *args, **kwargs):
        super().__init__(data, frames, default_frame, n_targets=2, *args, **kwargs)

    def target_position(self, frame, i):
        return self.position[frame][i].reshape((3,))

    def target_orientation(self, frame, i):
        return self.orientation[frame][i]

    def heuristic(self):
        """Determine what object is being placed."""
        least_distance = np.inf
        target_object = ''
        # Iterate over the frames and find the one where the second target is
        # closest to the origin. This is reasonably the object we are placing.
        for f in self.all_frames:
            # Ignore static frames
            if f in ['base', 'world', 'map']:
                continue

            target = self.target_position(f, 1)
            distance = np.linalg.norm(target)
            if distance < least_distance:
                least_distance = distance
                target_object = f

        self.exclude_frames = [target_object]
        self.parameters = [target_object]


class EquivalentPick(EquivalentAction):

    def __init__(self, actions):
        super().__init__(actions)

    def preconditions(self):
        pass

    def postconditions(self):
        pass

    def action_string(self):
        pass


class EquivalentPlace(EquivalentAction):

    def __init__(self, actions):
        super().__init__(actions)

    def preconditions(self):
        pass

    def postconditions(self):
        pass

    def action_string(self):
        pass


def simulate_single_stack(path, n_demos):
    sim = Simulator(path, ['object1', 'object2'])

    for i in range(n_demos):
        sim.begin_demonstration()
        sim.pick('object1')
        sim.place((0, 0, 0.1), 'object2')


def simulate_pick_and_place(path, n_demos):
    sim = Simulator(path, ['object1', 'object2'])

    for i in range(n_demos):
        sim.begin_demonstration()
        sim.pick('object1')
        sim.place((0, 0, 0), 'base')

        # Stack object2 on object1
        sim.pick('object2')
        sim.place((0, 0, 0.1), 'object1')


def simulate_two_configurations(path, n_demos):
    """
    Place object1 and object2 at two locations and then stack object1 on top of object.

    This task includes using the same action (place object1) twice
    but with different targets allowing testing that two actions of the same type
    but different target are identified as different actions.
    """
    sim = Simulator(path, ['object1', 'object2'])
    for i in range(n_demos):
        sim.begin_demonstration()
        sim.pick('object1')
        sim.place((0, 0, 1), 'base')

        sim.pick('object2')
        sim.place((0, 1, 2), 'base')

        sim.pick('object1')
        sim.place((0, 0, 0.1), 'object2')


def simulate_multi_target(path, n_demos):
    """
    Multi target simulation.

    1) Place object1 at (1, 2, 0) in base frame by passing through (0, 0, 0.1) in object2 frame.
    2) Pick object1 and place at (1, 2, 0) in base frame
       by passing through (-2, 0, 1) in base frame.

    Placing it twice allows testing that two actions that are equal in all aspects
    except one target are considered equal.
    """
    sim = Simulator(path, ['object1', 'object2'])
    for i in range(n_demos):
        sim.begin_demonstration()
        sim.pick('object1')
        sim.place_waypoint((0, 0, 0.1), 'object2', (1, 2, 0), 'base')

        sim.pick('object1')
        sim.place_waypoint((-2, 0, 1), 'base', (1, 2, 0), 'base')


def simulate_two_situations(path, n_demos):
    """
    Place B on A, then B in map, then A in map, and then B on A.

    The action B on A is repeated twice but in different situations and
    should therefore be regarded as separate actions.
    """
    sim = Simulator(path, ['A', 'B'])
    for i in range(n_demos):
        sim.begin_demonstration()
        sim.pick('B')
        sim.place([0, 0, 0.1], 'A')

        sim.pick('B')
        sim.place([1, 0, 0], 'map')

        sim.pick('A')
        sim.place([2, 0, 0], 'map')

        sim.pick('B')
        sim.place([0, 0, 0.1], 'A')


def test_initial_partition(tmp_path):
    action_bindings = {'pick': PickAction, 'place': PlaceAction}

    # Test single demonstration with two actions
    demos = RobotDemonstration(TESTDIR + '/demo_test', action_bindings)
    partitions = cl.initial_partition(demos.all_actions())
    assert len(partitions) == 2
    for group in partitions:
        assert len(group) == 1

    # Test a larger set of demonstrations
    folder = str(tmp_path / 'large')
    simulate_pick_and_place(folder, 5)
    demos = RobotDemonstration(folder, action_bindings)
    partitions = cl.initial_partition(demos.all_actions())

    assert len(partitions) == 4
    for group in partitions:
        # 5 demonstrations
        assert len(group) == 5
        # Each group should contain similar actions
        for i in range(1, len(group)):
            assert group[i].type == group[0].type
            assert group[i].parameters == group[0].parameters
            assert group[i].n_targets == group[0].n_targets


def test_cluster(tmp_path):
    action_bindings = {'pick': PickAction, 'place': PlaceAction}

    # Simple demonstration
    demos = RobotDemonstration(TESTDIR + '/demo_test', action_bindings)
    # Only contains two actions. A single action should be partitioned into a single action
    # The action with index 1 is place
    partitioning = cl.cluster(
        [demos.demonstrations()[0][1]], demos.frames, 0, len(demos.demonstrations()))
    assert len(partitioning) == 1
    assert len(partitioning[0]) == 1
    assert partitioning[0][0] == demos.demonstrations()[0][1]
    # The frame should be base since no frame can be inferred from one example
    assert partitioning[0][0].frame == ['base']

    # Pick and place
    folder = str(tmp_path / 'pick_place')
    simulate_single_stack(folder, 5)
    demos = RobotDemonstration(folder, action_bindings)

    # Cluster the second action, placing object1 on object2
    actions = [demo[1] for demo in demos.demonstrations()]
    partitioning = cl.cluster(actions, demos.frames, 0, len(demos.demonstrations()))

    # Only one action
    assert len(partitioning) == 1
    for a in partitioning[0]:
        assert a.frame == ['object2']


def test_partition(tmp_path):
    """Test that a demonstration is correctly partitioned into equivalence classes."""
    action_bindings = {
        'pick': PickAction,
        'place': PlaceAction,
        'place_waypoint': PlaceWaypointAction
    }

    # Simple demonstration
    demos = RobotDemonstration(TESTDIR + '/demo_test', action_bindings)
    partitions = cl.partition(demos.all_actions(), demos.frames, len(demos.demonstrations()))
    assert len(partitions) == 2

    # Test a larger set of demonstrations
    folder = str(tmp_path / 'large')
    simulate_two_configurations(folder, 5)
    demos = RobotDemonstration(folder, action_bindings)
    partitions = cl.partition(demos.all_actions(), demos.frames, len(demos.demonstrations()))

    # 3 pick + 3 place (place obj1, place obj2, place obj1 on obj2)
    assert len(partitions) == 6

    for group in partitions:
        # Each group should contain similar actions
        for i in range(1, len(group)):
            assert group[i].type == group[0].type
            assert group[i].parameters == group[0].parameters
            assert group[i].n_targets == group[0].n_targets
            assert group[i].frame == group[0].frame

    # Test a demonstration with a task that has two targets
    folder = str(tmp_path / 'multi_target')
    # Harder to do inference on multiple targets. Give more demonstrations.
    simulate_multi_target(folder, 10)
    demos = RobotDemonstration(folder, action_bindings)
    partitions = cl.partition(demos.all_actions(), demos.frames, len(demos.demonstrations()))

    # 2 pick + 2 place_waypoint (two different waypoints but same target)
    assert len(partitions) == 4
    # Find a place_waypoint action
    place_waypoint = next(filter(lambda x: x.type == 'place_waypoint', demos.all_actions()))
    # An action with two targets should be assigned two frames
    assert type(place_waypoint.frame) == list
    assert len(place_waypoint.frame) == 2

    # Demonstration where the same action is performed in different situations
    folder = str(tmp_path / 'multi_situation')
    simulate_two_situations(folder, 10)
    demos = RobotDemonstration(folder, action_bindings)
    partitions = cl.partition(demos.all_actions(), demos.frames, len(demos.demonstrations()))

    # 4 pick + 4 place (two place are the same but in different situations and should be
    # regarded as different actions)
    assert len(partitions) == 8
    # 2 actions of type place with parameter B and frame A
    assert len(
        [p for p in partitions if p[0].type == 'place' and
         p[0].parameters == ['B'] and p[0].frame == ['A']]) == 2


def test_find_equivalent(tmp_path):
    """Test that given a Demonstration object, it is divided in instances of EquivalentAction."""
    n_demos = 5

    action_bindings = {'pick': PickAction, 'place': PlaceAction}
    equivalent_action_bindings = {'pick': EquivalentPick, 'place': EquivalentPlace}

    folder = str(tmp_path / 'demo')
    simulate_pick_and_place(folder, n_demos)
    demos = RobotDemonstration(folder, action_bindings)

    equivalent = cl.find_equivalent_actions(demos, equivalent_action_bindings)

    # There are n_demos demonstrations
    assert len(equivalent) == n_demos

    # Each demonstation contains four actions (two pick + two place)
    for demo in equivalent:
        assert len(demo) == 4

    # All demonstrations are perfomred the same so all equivalent demonstrations
    # should be equal
    for i in range(1, len(equivalent)):
        assert equivalent[i-1] == equivalent[i]

    # Each demonstration is pick -> place -> pick -> place
    assert type(equivalent[0][0]) == EquivalentPick
    assert type(equivalent[0][1]) == EquivalentPlace
    assert type(equivalent[0][2]) == EquivalentPick
    assert type(equivalent[0][3]) == EquivalentPlace
