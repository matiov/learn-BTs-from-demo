"""
Implements a very simple task planner.

Inspired from 'Towards Blended Reactive Planning and Acting using Behavior Trees'.
[https://ieeexplore.ieee.org/abstract/document/8794128].

Generate a behaviors tree to solve task given a set of goals
and behaviors with preconditions and postconditions.
Since the conditions are not always static,
it actually runs the tree while evaluating the conditions.
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

import logging
from typing import Any, List, Tuple

from behaviors.behavior_lists import BehaviorLists
from behaviors.behavior_tree import get_action_list
from behaviors.common_behaviors import RSequence
from bt_learning.learning_from_demo.constraints_identification import contains_conflicting
import bt_learning.learning_from_demo.lfd_behaviors as bt
import py_trees as pt
from simulation.py_trees_interface import PyTree


logger = logging.getLogger('bt-planner')


def handle_precondition(
    precondition: str,
    behaviors: BehaviorLists,
    world_interface: Any
) -> PyTree:
    """
    Handle precondition by creating a subtree whose postconditions fulfill the precondition.

    Args
    ----
        precondition: the precondition to a specific action.
        behaviors: the behavior that has the precodition.
        world_interface: interface to the robot.

    Returns
    -------
        tree: a subtree implemented in py_trees.

    """
    # print("Condition in: ", precondition)
    condition_parameters = behaviors.get_condition_parameters(precondition)
    trees = []
    best_cost = float('inf')

    for action in get_action_list():
        action_node, _ = behaviors.get_node_from_string(
            action, world_interface, condition_parameters)

        if precondition in action_node.get_postconditions():
            # Only keep the actions with lowest cost
            if action_node.cost() > best_cost:
                continue
            elif action_node.cost() < best_cost:
                trees.clear()
                best_cost = action_node.cost()

            action_preconditions = action_node.get_preconditions()
            if action_preconditions != []:
                bt = RSequence('Sequence')

                for action_precondition in action_preconditions:
                    child, _ = behaviors.get_node_from_string(
                            action_precondition,
                            world_interface,
                            behaviors.get_condition_parameters(action_precondition)
                        )
                    bt.add_child(child)

                bt.add_child(action_node)
            else:
                bt = action_node

            trees.append(bt)
    # print("ERROR, no matching action found to ensure precondition")
    return trees


def extend_leaf_node(
    leaf_node: Any,
    behaviors: BehaviorLists,
    world_interface: Any
):
    """
    Extend the leaf node when it fails.

    If leaf node fails, it should be replaced with a selector that checks leaf node
    and a subtree that fixes the conditon whenever it's not met.

    Args:
    ----
        leaf_node: the behavior to extend.
        behaviors: a list with all behaviors in the demonstration.
        world_interface: interface to the robot.

    """
    bt = pt.composites.Selector(name='Fallback')
    # Make the replacing subtree is still failing. The parent node might get confused
    # if one of its children suddenly changes status to INVALID.
    bt.stop(pt.common.Status.FAILURE)
    leaf_node.parent.replace_child(leaf_node, bt)
    bt.add_child(leaf_node)
    # print("What is failing? ", leaf_node.name)

    print('\n Handling:' + str(leaf_node.name))

    extended = handle_precondition(leaf_node.name, behaviors, world_interface)
    for tree in extended:
        bt.add_child(tree)


def expand_tree(
    node: Any,
    behaviors: BehaviorLists,
    world_interface: Any,
    depth: int = 0
):
    """
    Expand the part of the tree that fails.

    Args:
    ----
        node: the behavior to expand.
        behaviors: a list with all behaviors in the demonstration.
        world_interface: interface to the robot.
        depth: depth of the subtree.

    """
    # print("TREE COMING IN :", node)

    # If the recursion is very deep there is some problem and we should abort
    if depth >= 100:
        logger.warning('Maximum condition expansion depth reached')
        return

    if node.name == 'Fallback':
        # print("Fallback node fails\n")
        for index, child in enumerate(node.children):
            if index >= 1:  # Normally there will only be two children
                expand_tree(child, behaviors, world_interface, depth+1)
    elif node.name == 'Sequence' or node.name == 'RandomSelector':
        # print("Sequence node fails\n")
        for i in range(len(node.children)):
            if node.children[i].status == pt.common.Status.FAILURE:
                # print("Child that fails: ", node.children[i].name)
                expand_tree(node.children[i], behaviors, world_interface, depth+1)
    elif isinstance(node, pt.behaviour.Behaviour) and node.status == pt.common.Status.FAILURE:
        extend_leaf_node(node, behaviors, world_interface)
    # else:
        # print("Tree", node.name)


def get_tree_effects(tree: Any) -> List[str]:
    """
    Return all effects of a tree as action strings.

    I.e. traverses the tree and returns the first child of each falback node,
    and the postconditions of the last child of each sequence node.

    Args
    ----
        tree: the Behavior Tree to traverse.

    Returns
    -------
        conditions: list of the post-conditions.

    """
    if isinstance(tree, bt.ActionBehavior):
        return tree.get_postconditions()
    elif len(tree.children) > 0:
        conditions = []
        for child in tree.children:
            conditions += get_tree_effects(child)
        return conditions
    else:
        return []


def conflicting_subtrees(
    trees: List[Any],
    behaviors: BehaviorLists
) -> Tuple(int, int):
    """
    Get the subtrees that are conflicting.

    It is assumed that each element of trees is a fallback node or a condition node.

    Args
    ----
        trees: the whole tree or subtree.
        behaviors: list of all behaviors in the demonstration.

    Returns
    -------
        Indices of the subtrees in the list of trees that are in conflict.
        It returns (-1, -1) if there are no conflicts.

    """
    for i in range(len(trees)-2, -1, -1):
        if isinstance(trees[i], pt.composites.Selector) and not isinstance(trees[i], RSequence):
            # The condition is the first child of a fallback node
            high_priority_condition = trees[i].children[0].name
        else:
            high_priority_condition = trees[i].name

        for j in range(i+1, len(trees)):
            effects = get_tree_effects(trees[j])
            if contains_conflicting(behaviors, effects, high_priority_condition):
                return i, j

    return -1, -1


def handle_priority(
    tree: Any,
    behaviors: BehaviorLists
) -> bool:
    """
    Go through the tree to detect subtrees that have conflicting effects and reorders them.

    Args:
    ----
        trees: the whole tree or subtree.
        behaviors: list of all behaviors in the demonstration.

    Returns
    -------
        True if the priority has been handled correctly.

    """
    if isinstance(tree, RSequence):
        subtrees = tree.children
        # The last child of all sequence nodes is an action node except
        # for the topmost sequence node that contains all goal conditions.
        # Don't include the last child unless it is the topmost node
        i, j = conflicting_subtrees(subtrees, behaviors)

        n_tries = 0
        while i != -1:
            logger.info('Detected conflicting subtrees')
            logger.debug(
                'Conflicting subtrees:\n%s-----------------\n%s',
                pt.display.unicode_tree(subtrees[i]),
                pt.display.unicode_tree(subtrees[j])
            )
            # Place child j before child i
            tree_to_move = subtrees[j]
            tree.remove_child(tree_to_move)
            tree.insert_child(tree_to_move, i)

            i, j = conflicting_subtrees(subtrees, behaviors)

            n_tries += 1
            if n_tries > len(subtrees):
                logger.warning(
                    'Could not find a configuration of subtrees that is free of conflicts.'
                )
                return False

        return True

    for child in tree.children:
        if not handle_priority(child, behaviors):
            return False
    return True


def plan(
    world_interface: Any,
    behaviors: BehaviorLists,
    tree: Any
) -> PyTree:
    """
    Plan a Behavior Tree.

    Generates a behaviors tree to solve task given an initial tree
    and behaviors with preconditions and postconditions.
    Since the conditions are not always static,
    it actually runs the tree while evaluating the conditions.

    Args
    ----
        world_interface: interface to the robot.
        behaviors: list of all available behaviors in the demonstration.
        tree: the behavior tree.

    Returns
    -------
        tree: the final behavior tree.

    """
    # print(pt.display.unicode_tree(root=tree, show_status=True))
    for i in range(100):
        if not handle_priority(tree, behaviors):
            break
        tree.tick_once()
        if tree.status is pt.common.Status.FAILURE:
            expand_tree(tree, behaviors, world_interface)
            # print(pt.display.unicode_tree(root=tree, show_status=True))
        elif tree.status is pt.common.Status.SUCCESS:
            break

    return tree
