"""Test routines for the constraints inference."""

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

import bt_learning.learning_from_demo.constraints_identification as ci
from bt_learning.learning_from_demo.tests.util import ActionTest
import pytest


def test_constraint_set():
    action_a = ActionTest()
    action_b = ActionTest()
    action_c = ActionTest()

    # We cannot have a constraint A < A
    with pytest.raises(ValueError):
        ci.Constraint(action_a, action_a)

    constraints = ci.ConstraintSet()

    # ConstraintSet should initialize empty
    assert len(constraints.active_constraints()) == 0
    assert len(constraints.conflicting_constraints()) == 0

    # Add one constraint. It sohuld become active.
    constraints.add(ci.Constraint(action_a, action_b))
    assert len(constraints.active_constraints()) == 1
    assert constraints.active_constraints()[0] == ci.Constraint(action_a, action_b)
    assert len(constraints.conflicting_constraints()) == 0

    # Add the same constraint again. Nothing should change.
    constraints.add(ci.Constraint(action_a, action_b))
    assert len(constraints.active_constraints()) == 1
    assert constraints.active_constraints()[0] == ci.Constraint(action_a, action_b)
    assert len(constraints.conflicting_constraints()) == 0

    # Add a conflicting constraint. No active and two conflicting
    constraints.add(ci.Constraint(action_b, action_a))
    assert len(constraints.active_constraints()) == 0
    assert len(constraints.conflicting_constraints()) == 2
    assert ci.Constraint(action_a, action_b) in constraints.conflicting_constraints()
    assert ci.Constraint(action_b, action_a) in constraints.conflicting_constraints()

    # Adding one of them again should not change anything
    constraints.add(ci.Constraint(action_b, action_a))
    assert len(constraints.active_constraints()) == 0
    assert len(constraints.conflicting_constraints()) == 2
    assert ci.Constraint(action_a, action_b) in constraints.conflicting_constraints()
    assert ci.Constraint(action_b, action_a) in constraints.conflicting_constraints()

    # Adding a new constraint is ok
    constraints.add(ci.Constraint(action_a, action_c))
    assert len(constraints.active_constraints()) == 1
    assert constraints.active_constraints()[0] == ci.Constraint(action_a, action_c)
    assert len(constraints.conflicting_constraints()) == 2
    assert ci.Constraint(action_a, action_b) in constraints.conflicting_constraints()
    assert ci.Constraint(action_b, action_a) in constraints.conflicting_constraints()


def test_constraint_inference():
    action_a = ActionTest()
    action_b = ActionTest()
    action_c = ActionTest()

    # A single demonstration
    demonstrations = [[action_a, action_b, action_c]]
    constraints = ci.infer_order_constraints(demonstrations)
    # We should get constraints (A < B) (distance 1), (A < C) (distance 2), (B < C) (distance 1)
    assert len(constraints.active_constraints()) == 3
    assert ci.Constraint(action_a, action_b) in constraints.active_constraints()
    assert ci.Constraint(action_a, action_c) in constraints.active_constraints()
    assert ci.Constraint(action_b, action_c) in constraints.active_constraints()
    a_b = constraints.active_constraints().index(ci.Constraint(action_a, action_b))
    a_c = constraints.active_constraints().index(ci.Constraint(action_a, action_c))
    b_c = constraints.active_constraints().index(ci.Constraint(action_b, action_c))
    assert constraints.active_constraints()[a_b].distance == 1
    assert constraints.active_constraints()[a_c].distance == 2
    assert constraints.active_constraints()[b_c].distance == 1

    # Two demonstrations where the order of two actions has been changed
    demonstrations = [
        [action_a, action_b, action_c],
        [action_a, action_c, action_b]
    ]
    constraints = ci.infer_order_constraints(demonstrations)
    # We should only get the constraints (A < B) and (A < C) with
    # both of them having distance 1
    assert len(constraints.active_constraints()) == 2
    assert ci.Constraint(action_a, action_b) in constraints.active_constraints()
    assert ci.Constraint(action_a, action_c) in constraints.active_constraints()
    a_b = constraints.active_constraints().index(ci.Constraint(action_a, action_b))
    a_c = constraints.active_constraints().index(ci.Constraint(action_a, action_c))
    assert constraints.active_constraints()[a_b].distance == 1
    assert constraints.active_constraints()[a_c].distance == 1
