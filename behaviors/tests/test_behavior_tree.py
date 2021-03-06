"""Unit test for behavior_tree.py"""

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

import pytest
import random

from behaviors import behavior_lists
from behaviors import behavior_tree
from behaviors.tests import behavior_list_test_settings
from simulation.py_trees_interface import PyTree
import simulation.tests.behaviors_toggleread as behaviors


behavior_list = behavior_lists.BehaviorLists(\
    condition_nodes=behavior_list_test_settings.get_condition_nodes(), \
    action_nodes=behavior_list_test_settings.get_action_nodes())


def test_init():
    """ Tests init function """
    _ = behavior_tree.BT([], behavior_list)


def test_random():
    """ Tests random function """
    bt = behavior_tree.BT([], behavior_list)

    random.seed(1337)

    for length in range(1, 11):
        bt.random(length, 0.5)
        assert bt.length() == length
        assert bt.is_valid()

    for _ in range(10):
        many_leafs_counter = 0
        few_leafs_counter = 0
        many_leafs = bt.random(10, 0.8)
        few_leafs = bt.random(10, 0.2)
        for i in range(10):
            if bt.behaviors.is_leaf_node(many_leafs[i]):
                many_leafs_counter += 1
            if bt.behaviors.is_leaf_node(few_leafs[i]):
                few_leafs_counter += 1
        assert many_leafs_counter > few_leafs_counter


def test_is_valid():
    """ Tests is_valid function """
    bt = behavior_tree.BT([], behavior_list)
    assert not bt.is_valid()

    #Valid tree
    bt.set(['s(', 'b', 'f(', 'c', 'ab!', ')', 'ac!', ')'])
    assert bt.is_valid()

    #Minimal valid tree - just an action node
    bt.set(['ac!'])
    assert bt.is_valid()

    #Two control nodes at root level - not valid
    bt.set(['s(', 'c', 'f(', 'c', 'ab!', ')', 'ab!', ')', 's(', 'ab!', ')'])
    assert not bt.is_valid()

    #Action node at root level - not valid
    bt.set(['s(', 'c', 'f(', 'c', 'ab!', ')', ')', 'ab!', ')'])
    assert not bt.is_valid()

    #Too few up nodes - not valid
    bt.set(['s(', 'c', 'f(', 'c', 'ab!', ')', 'ab!'])
    assert not bt.is_valid()

    #Too few up nodes - not valid
    bt.set(['s(', 'c', 'f(', 'c', 'ab!', ')'])
    assert not bt.is_valid()

    #No control nodes, but more than one action - not valid
    bt.set(['ab!', 'ab!'])
    assert not bt.is_valid()

    #Starts with an up node - not valid
    bt.set([')', 'f(', 'c', 'ab!', ')'])
    assert not bt.is_valid()

    #Just a control node - not valid
    bt.set(['s(', ')'])
    assert not bt.is_valid()

    #Just a control node - not valid
    bt.set(['s(', 's('])
    assert not bt.is_valid()

    #Up just after control node
    bt.set(['s(', 'f(', ')', 'ab!', ')'])
    assert not bt.is_valid()

    #Unknown characters
    bt.set(['s(', 'c', 'x', 'y', 'z', ')'])
    assert not bt.is_valid()


def test_subtree_is_valid():
    """Test subtree_is_valid function."""
    bt = behavior_tree.BT([], behavior_list)
    assert bt.is_subtree_valid(['s(', 'f(', 'ab!', ')', ')', ')'], True, True)

    assert not bt.is_subtree_valid(['s(', 'f(', 'ab!', ')', ')', ')'], True, False)

    assert not bt.is_subtree_valid(['f(', 's(', 'ab!', ')', ')', ')'], False, True)

    assert not bt.is_subtree_valid(['f(', 'f(', 'ab!', ')', ')', ')'], True, True)

    assert not bt.is_subtree_valid(['s(', 's(', 'ab!', ')', ')', ')'], True, True)

    assert not bt.is_subtree_valid(['s(', 'f(', 'ab!', ')', ')'], True, True)

    assert bt.is_subtree_valid(['s(', 'f(', 'c', ')', ')', ')'], True, True)


def test_close():
    """Test close function"""
    bt = behavior_tree.BT([], behavior_list)
    bt.close()
    assert bt.bt == []

    #Correct tree with just one action
    bt.set(['ab!']).close()
    assert bt.bt == ['ab!']

    #Correct tree
    bt.set(['s(', 's(', 'ab!', ')', ')']).close()
    assert bt.bt == ['s(', 's(', 'ab!', ')', ')']

    #Missing up at end
    bt.set(['s(', 's(', 'ab!', ')', 's(', 'ab!', 's(', 'ab!']).close()
    assert bt.bt == ['s(', 's(', 'ab!', ')', 's(', 'ab!', 's(', 'ab!', ')', ')', ')']

    #Too many up at end
    bt.set(['s(', 'ab!', ')', ')', ')']).close()
    assert bt.bt == ['s(', 'ab!', ')']

    #Too many up but not at the end
    bt.set(['s(', 's(', 'ab!', ')', ')', ')', 'ac!', ')']).close()
    assert bt.bt == ['s(', 's(', 'ab!', ')', 'ac!', ')']


def test_trim():
    """Test trim function."""
    bt = behavior_tree.BT([], behavior_list)

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', 's(', 'ab!', ')', ')'])
    bt.trim()
    assert bt.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', ')', 'ab!', 's(', 'ab!', ')', ')'])
    bt.trim()
    assert bt.bt == ['s(', 'ab!', 'ab!', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ac!', 's(', 'ad!', ')', 'ae!', ')', 'af!', ')'])
    bt.trim()
    assert bt.bt == ['s(', 'ab!', 'f(', 'ac!', 'ad!', 'ae!', ')', 'af!', ')']

    bt.set(['s(', 'ab!', 'f(', 's(', 'ad!', 'ae!', ')', ')', 'af!', ')'])
    bt.trim()
    assert bt.bt == ['s(', 'ab!', 'ad!', 'ae!', 'af!', ')']

    bt.set(['s(', 'ab!', ')'])
    bt.trim()
    assert bt.bt == ['ab!']

    bt.set(['f(', 's(', 'ab!', 'ac!', ')', ')'])
    bt.trim()
    assert bt.bt == ['s(', 'ab!', 'ac!', ')']


def test_depth():
    """Test bt_depth function."""
    bt = behavior_tree.BT([], behavior_list)

    #Normal correct tree
    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'])
    assert bt.depth() == 2

    #Goes to 0 before last node - invalid
    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')', 's(', 'ab!', ')'])
    assert bt.depth() == -1

    #Goes to 0 before last node  - invalid
    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', ')', 'ab!', ')'])
    assert bt.depth() == -1

    #Goes to 0 before last node - invalid
    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!'])
    assert bt.depth() == -1

    #Just an action node - no depth
    bt.set(['ab!'])
    assert bt.depth() == 0


def test_length():
    """Test bt_length function."""
    bt = behavior_tree.BT([], behavior_list)

    bt.set(['s(', 'ab!', 'ac!', ')'])
    assert bt.length() == 3

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'])
    assert bt.length() == 6

    bt.set(['s(', ')'])
    assert bt.length() == 1

    bt.set(['ab!'])
    assert bt.length() == 1


def test_change_node():
    """Test change_node function."""
    bt = behavior_tree.BT([], behavior_list)

    random.seed(1337)

    #No new node given, change to random node
    bt.set(['s(', 'ab!', 'ab!', ')']).change_node(2, 0.5)
    assert bt.bt[2] != 'ab!'

    #Change control node to action node
    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).change_node(2, 0.5, 'ab!')
    assert bt.bt == ['s(', 'ab!', 'ab!', 'ab!', ')']

    #Change control node to action node - correct up must be removed too
    bt.set(['s(', 'ab!', 'f(', 's(', 'ab!', ')', 'ab!', ')', 'ab!', ')']).change_node(2, 0.5, 'ab!')
    assert bt.bt == ['s(', 'ab!', 'ab!', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 's(', 'ab!', ')', 'ac!', ')', 'ab!', ')']).change_node(3, 0.5, 'ab!')
    assert bt.bt == ['s(', 'ab!', 'f(', 'ab!', 'ac!', ')', 'ab!', ')']

    #Change action node to control node
    bt.set(['s(', 'ab!', 'ab!', ')']).change_node(1, 0.5, 'f(')
    assert bt.bt == ['s(', 'f(', 'af!', 'ab!', ')', 'ab!', ')']

    #Change action node to action node
    bt.set(['s(', 'ab!', 'ab!', ')']).change_node(1, 0.5, 'ac!')
    assert bt.bt == ['s(', 'ac!', 'ab!', ')']

    #Change control node to control node
    bt.set(['s(', 'ab!', 'ab!', ')']).change_node(0, 0.5, 'f(')
    assert bt.bt == ['f(', 'ab!', 'ab!', ')']

    #Change up node, not possible
    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).change_node(5, 0.5, 'ab!')
    assert bt.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']


def test_add_node():
    """Test add_node function."""
    bt = behavior_tree.BT([], behavior_list)

    random.seed(1337)

    bt.set(['ab!']).add_node(0, 0.5, 's(')
    assert bt.bt == ['s(', 'ab!', ')']

    bt.set(['s(', 'ab!', 'ab!', ')']).add_node(2, 0.5)
    assert bt.bt == ['s(', 'ab!', 's(', 'af!', 'af!', ')', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).add_node(2, 0.5, 'ab!')
    assert bt.bt == ['s(', 'ab!', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).add_node(3, 0.5, 'ab!')
    assert bt.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', 'ab!', ')', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).add_node(0, 0.5, 'f(')
    assert bt.bt == ['f(', 's(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')', ')']

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).add_node(4, 0.5, 's(')
    assert bt.is_valid()

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).add_node(2, 0.5, 'f(')
    assert bt.is_valid()

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).add_node(1, 0.5, 'f(')
    assert bt.is_valid()

    bt.set(['s(', 'ab!', 'f(', 'c1', 'ab!', ')', ')']).add_node(2, 0.5, 'f(')
    assert bt.is_valid()


def plot_individual(individual, plot_name):
    """Save a graphical representation of the individual."""
    pytree = PyTree(individual[:], behaviors=behaviors)
    pytree.save_fig('logs/', name=plot_name)

def test_delete_node():
    """ Tests delete_node function """
    bt = behavior_tree.BT([], behavior_list)

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).delete_node(0)
    assert bt.bt == []

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 's(', 'ab!', ')', ')']).delete_node(0)
    assert bt.bt == []

    bt.set(['s(', 'ab!', 'f(', 'ab!', 's(', 'ab!', ')', ')', 's(', 'ab!', ')', ')']).delete_node(0)
    assert bt.bt == []

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']).delete_node(1)
    assert bt.bt == ['s(', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ac!', 'ad!', ')', 'ae!', ')']).delete_node(2)
    assert bt.bt == ['s(', 'ab!', 'ae!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!', ')']).delete_node(3)
    assert bt.bt == ['s(', 'ab!', 'f(', ')', 'ab!', ')']

    bt.set(['s(', 'ab!', ')']).delete_node(2)
    assert bt.bt == ['s(', 'ab!', ')']


def test_find_parent():
    """Test find_parent function."""
    bt = behavior_tree.BT([], behavior_list)

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!', ')'])
    assert bt.find_parent(0) is None
    assert bt.find_parent(1) == 0
    assert bt.find_parent(2) == 0
    assert bt.find_parent(3) == 2
    assert bt.find_parent(4) == 2
    assert bt.find_parent(5) == 0


def test_find_children():
    """Test find_children function."""
    bt = behavior_tree.BT([], behavior_list)

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!', ')'])
    assert bt.find_children(0) == [1, 2, 5]
    assert bt.find_children(1) == []
    assert bt.find_children(2) == [3]
    assert bt.find_children(3) == []
    assert bt.find_children(4) == []
    assert bt.find_children(5) == []


def test_find_up_node():
    """Test find_up_node function."""
    bt = behavior_tree.BT([], behavior_list)

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!', ')'])
    assert bt.find_up_node(0) == 6

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!', ')'])
    assert bt.find_up_node(2) == 4

    bt.set(['s(', 'ab!', 'f(', 's(', 'ab!', ')', 'ab!', ')'])
    assert bt.find_up_node(2) == 7

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!', ')'])
    with pytest.raises(Exception):
        _ = bt.find_up_node(1)

    bt.set(['s(', 'ab!', 'f(', 'ab!', ')', 'ab!'])
    with pytest.raises(Exception):
        _ = bt.find_up_node(0)

    bt.set(['s(', 's(', 'ab!', 'f(', 'ab!', ')', 'ab!'])
    with pytest.raises(Exception):
        _ = bt.find_up_node(1)


def test_get_subtree():
    """Test get_subtree function."""

    bt = behavior_tree.BT(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'], behavior_list)
    subtree = bt.get_subtree(1)
    assert subtree == ['ab!']

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 's(', 'ab!', 'ab!', ')', ')'])
    subtree = bt.get_subtree(6)
    assert subtree == ['s(', 'ab!', 'ab!', ')']

    bt.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', ')'])
    subtree = bt.get_subtree(2)
    assert subtree == ['f(', 'ab!', 'ab!', ')']

    subtree = bt.get_subtree(5)
    assert subtree == []


def test_insert_subtree():
    """Test insert_subtree function."""
    bt1 = behavior_tree.BT(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'], behavior_list)
    bt1.insert_subtree(['f(', 'ac!', ')'], 1)
    assert bt1.bt == ['s(', 'f(', 'ac!', ')', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']

    bt1.insert_subtree(['f(', 'ac!', ')'], 6)
    assert bt1.bt == ['s(', 'f(', 'ac!', ')', 'ab!', 'f(', 'f(', 'ac!', ')', 'ab!', 'ab!', ')', 'ab!', ')']


def test_swap_subtrees():
    """Test swap_subtrees function."""

    bt1 = behavior_tree.BT(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'], behavior_list)
    bt2 = behavior_tree.BT(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 's(', 'ab!', 'ab!', ')', ')'], behavior_list)
    bt1.swap_subtrees(bt2, 6, 6)
    assert bt1.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 's(', 'ab!', 'ab!', ')', ')']
    assert bt2.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']

    #Invalid subtree because it's an up node, no swap
    bt1.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'])
    bt2.set(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 's(', 'ab!', 'ab!', ')', ')'])
    bt1.swap_subtrees(bt2, 5, 6)
    assert bt1.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')']
    assert bt2.bt == ['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 's(', 'ab!', 'ab!', ')', ')']


def test_is_subtree():
    """Test is_subtree function."""
    bt = behavior_tree.BT(['s(', 'ab!', 'f(', 'ab!', 'ab!', ')', 'ab!', ')'], behavior_list)

    assert bt.is_subtree(0)
    assert bt.is_subtree(1)
    assert not bt.is_subtree(5)
