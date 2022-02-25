"""Auxiliary functions to the test routines."""

# Copyright (c) 2021 Matteo Iovino
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from bt_learning.learning_from_demo.demonstration import Action, EquivalentAction


TESTDIR = os.path.dirname(os.path.realpath(__file__))


class DummyActionObservation(Action):
    """An empty action observation for testing purposes."""

    def __init__(self):
        super().__init__('test_action', 'test_frame')
        self.n_targets = 0

    def target_position(self, frame, i):
        return None

    def target_orientation(self, frame, i):
        return None


class ActionTest(EquivalentAction):

    def __init__(self):
        super().__init__([DummyActionObservation()])
        self.preconditions_list = []
        self.postconditions_list = []

    def preconditions(self):
        return self.preconditions_list

    def postconditions(self):
        return self.postconditions_list

    def action_string(self):
        return 'action'
