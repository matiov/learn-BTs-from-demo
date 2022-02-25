"""Test routines for the precondition inference."""

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

from bt_learning.learning_from_demo.constraints_identification import infer_preconditions
import bt_learning.learning_from_demo.lfd_behaviors as bt
from bt_learning.learning_from_demo.tests.test_constraints import ActionTest


class BehaviorsTest(bt.Behaviors):

    def get_node_from_string(self, string, world_interface, condition_parameters):
        return super().get_node_from_string(string, world_interface, condition_parameters)

    def get_actions(self):
        pass

    def get_conditions(self):
        pass

    def compatible(self, condition1, condition2):
        # Condition 1 and 2 are incompatible
        pair = (condition1, condition2)
        return pair != ('condition 1', 'condition 2') and pair != ('condition 2', 'condition 1')


def test_no_conflicts_single_demo():
    """Single demonstration with no conflicting conditions."""
    action_2 = ActionTest()
    action_2.preconditions_list = ['condition pre2']
    action_2.postconditions_list = ['condition 2']
    action_3 = ActionTest()
    action_3.preconditions_list = ['condition pre3']
    action_3.postconditions_list = ['condition 3']
    action_4 = ActionTest()
    action_4.preconditions_list = ['condition pre4']
    action_4.postconditions_list = ['condition 4']

    demonstrations = [[action_2, action_3, action_4]]
    infer_preconditions(demonstrations, BehaviorsTest())
    # The actions have no default preconditions and no conflicting conditions so
    # the preconditions should equal the postconditions of the preceeding actions.
    assert __equivalent_lists(
            action_3.preconditions_with_additional(),
            action_3.preconditions() + action_2.postconditions()
        )
    assert __equivalent_lists(
            action_4.preconditions_with_additional(),
            action_4.preconditions() + action_3.postconditions() + action_2.postconditions()
        )
    # action_2 should have no preconditions because no action preceeds it
    assert action_2.preconditions_with_additional() == action_2.preconditions()


def test_no_conflicts_double_demo():
    """Two demonstrations with different action order and no conflicting conditions."""
    action_2 = ActionTest()
    action_2.preconditions_list = ['condition pre2']
    action_2.postconditions_list = ['condition 2']
    action_3 = ActionTest()
    action_3.preconditions_list = ['condition pre3']
    action_3.postconditions_list = ['condition 3']
    action_4 = ActionTest()
    action_4.preconditions_list = ['condition pre4']
    action_4.postconditions_list = ['condition 4']

    # Two demonstrations where the order of two actions has been changed
    demonstrations = [
        [action_2, action_3, action_4],
        [action_2, action_4, action_3]
    ]
    infer_preconditions(demonstrations, BehaviorsTest())
    # Both action_3 and action_4 shoud have the postconditions of action_2
    # as preconditions
    assert __equivalent_lists(
            action_3.preconditions_with_additional(),
            action_3.preconditions() + action_2.postconditions()
        )
    assert __equivalent_lists(
            action_4.preconditions_with_additional(),
            action_4.preconditions() + action_2.postconditions()
        )
    # action_2 should have no preconditions because no action ever preceeds it
    assert action_2.preconditions_with_additional() == action_2.preconditions()


def test_incompatible_precondition():
    """Test that a new precondition is not added if it conflicts with an existing precondition."""
    action_1 = ActionTest()
    action_1.preconditions_list = ['condition pre1']
    action_1.postconditions_list = ['condition 1']
    action_2 = ActionTest()
    action_2.preconditions_list = ['condition 2']
    action_2.postconditions_list = ['condition post2']
    action_3 = ActionTest()
    action_3.preconditions_list = ['condition 3']
    action_3.postconditions_list = ['condition post3']

    demonstrations = [[action_1, action_2, action_3]]
    infer_preconditions(demonstrations, BehaviorsTest())
    # action_3 should inherit the postconditions of the preceeding actions
    assert __equivalent_lists(
            action_3.preconditions_with_additional(),
            action_3.preconditions() + action_2.postconditions() + action_1.postconditions()
        )
    # condition 1 from action_1 should not be added to action_2 because it conflicts
    # with the precondition condition 2.
    assert action_2.preconditions_with_additional() == action_2.preconditions()
    # action_1 should not get any additional preconditions since no action preceeds it
    assert action_1.preconditions_with_additional() == action_1.preconditions()


def test_incompatible_postcondition():
    """Test that a new precondition is not added if conflicting with an existing postcondition."""
    action_1 = ActionTest()
    action_1.preconditions_list = ['condition pre1']
    action_1.postconditions_list = ['condition 1']
    action_2 = ActionTest()
    action_2.preconditions_list = ['condition pre2']
    action_2.postconditions_list = ['condition 2']

    demonstrations = [[action_1, action_2]]
    infer_preconditions(demonstrations, BehaviorsTest())
    # action_2 should not contain any additional preconditions since condition 1 conflicts
    # with the postcondition condition 2.
    assert action_2.preconditions_with_additional() == action_2.preconditions()
    # action_1 should not get any additional preconditions since no action preceeds it
    assert action_1.preconditions_with_additional() == action_1.preconditions()


def test_conflicting_additional():
    """Test that if multiple preconditions are conflicting, none is added."""
    action_1 = ActionTest()
    action_1.preconditions_list = ['condition pre1']
    action_1.postconditions_list = ['condition 1']
    action_2 = ActionTest()
    action_2.preconditions_list = ['condition pre2']
    action_2.postconditions_list = ['condition 2']
    action_3 = ActionTest()
    action_3.preconditions_list = ['condition pre3']
    action_3.postconditions_list = ['condition post3']
    action_4 = ActionTest()
    action_4.preconditions_list = ['condition pre4']
    action_4.postconditions_list = ['condition post4']

    demonstrations = [[action_1, action_2, action_3, action_4]]
    infer_preconditions(demonstrations, BehaviorsTest())
    # action_4 should have the postconditions of action_3 and action_2 as additional preconditions
    # but not those of action_1. action_1 and action_2 have conflicting postconditions but the
    # constraint action_2 < action_4 has less distance than action_1 < action_4.
    assert __equivalent_lists(
            action_4.preconditions_with_additional(),
            action_4.preconditions() + action_3.postconditions() + action_2.postconditions()
        )
    # action_3 should have the postconditions of action_2 since condition 1 and condition 2
    # that would be added are conflicting conditions
    # but action_2 < action_3 has less distance than action_1 action_3.
    assert action_3.preconditions_with_additional() ==\
           action_2.postconditions() + action_3.preconditions()
    # action_2 should have no additional preconditions since condition 1 conflicts
    # with the postcondition condition 2
    assert action_2.preconditions_with_additional() == action_2.preconditions()
    # action_1 should not get any additional preconditions since no action preceeds it
    assert action_1.preconditions_with_additional() == action_1.preconditions()


def __equivalent_lists(list1, list2):
    """Return True if list1 and list2 contain the same items."""
    if len(list1) != len(list2):
        return False

    for item in list1:
        if item not in list2:
            return False

    for item in list2:
        if item not in list1:
            return False

    return True
