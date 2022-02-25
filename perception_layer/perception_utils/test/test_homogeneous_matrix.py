"""Test routine for the homogeneous matrix function."""

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


import numpy as np
from perception_utils.homogeneous_matrix import homogeneous_matrix
import perception_utils.transformations as tf


def test_homogeneous_matrix():
    rotation = tf.random_quaternion()
    translation = np.random.rand(3)

    rotation_matrix = tf.quaternion_matrix(rotation)
    translation_matrix = tf.translation_matrix(translation)

    tf_matrix = np.eye(4)
    tf_matrix[:3, :3] = rotation_matrix[:3, :3]
    tf_matrix[:, 3] = translation_matrix[:, 3]

    test_matrix = homogeneous_matrix(translation, rotation, ros_convention=False)

    assert np.allclose(tf_matrix, test_matrix) is True
