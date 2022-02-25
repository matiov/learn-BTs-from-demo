"""Test routines for the transformations library."""

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

import math
import random

import numpy as np
import perception_utils.transformations as tf


# Test helper functions
def test_is_same_transform():
    assert tf.is_same_transform(np.identity(4), np.identity(4)) is True
    assert tf.is_same_transform(np.identity(4), np.random.rand(4, 4)) is False


def test_vector_norm():
    v = np.random.random(3)
    n = tf.vector_norm(v)
    assert np.allclose(n, np.linalg.norm(v)) is True

    v = np.random.rand(6, 5, 3)
    n = tf.vector_norm(v, axis=-1)
    assert np.allclose(n, np.sqrt(np.sum(v*v, axis=2))) is True

    n = tf.vector_norm(v, axis=1)
    assert np.allclose(n, np.sqrt(np.sum(v*v, axis=1))) is True

    v = np.random.rand(5, 4, 3)
    n = np.empty((5, 3), dtype=np.float64)
    tf.vector_norm(v, axis=1, out=n)
    assert np.allclose(n, np.sqrt(np.sum(v*v, axis=1))) is True

    assert tf.vector_norm([]) == 0.0
    assert tf.vector_norm([1.0]) == 1.0


def test_unit_vector():
    v0 = np.random.random(3)
    v1 = tf.unit_vector(v0)
    assert np.allclose(v1, v0 / np.linalg.norm(v0)) is True

    v0 = np.random.rand(5, 4, 3)
    v1 = tf.unit_vector(v0, axis=-1)
    v2 = v0 / np.expand_dims(np.sqrt(np.sum(v0*v0, axis=2)), 2)
    assert np.allclose(v1, v2) is True

    v1 = tf.unit_vector(v0, axis=1)
    v2 = v0 / np.expand_dims(np.sqrt(np.sum(v0*v0, axis=1)), 1)
    assert np.allclose(v1, v2) is True

    v1 = np.empty((5, 4, 3), dtype=np.float64)
    tf.unit_vector(v0, axis=1, out=v1)
    assert np.allclose(v1, v2) is True
    assert list(tf.unit_vector([])) == []
    assert list(tf.unit_vector([1.0])) == [1.0]


def test_random_vector():
    v = tf.random_vector(10000)
    assert bool(np.all(v >= 0.0) and np.all(v < 1.0)) is True

    v0 = tf.random_vector(10)
    v1 = tf.random_vector(10)
    assert bool(np.any(v0 is v1)) is False


def test_inverse_matrix():
    M0 = tf.random_rotation_matrix()
    M1 = tf.inverse_matrix(M0.T)
    assert np.allclose(M1, np.linalg.inv(M0.T)) is True

    for size in range(1, 7):
        M0 = np.random.rand(size, size)
        M1 = tf.inverse_matrix(M0)
        assert np.allclose(M1, np.linalg.inv(M0)) is True


def test_concatenate_matrices():
    M = np.random.rand(16).reshape((4, 4)) - 0.5
    assert np.allclose(M, tf.concatenate_matrices(M)) is True
    assert np.allclose(np.dot(M, M.T), tf.concatenate_matrices(M, M.T)) is True


# Test transformation functions
def test_identity():
    mI = tf.identity_matrix()
    oracle_identity = np.matrix([
        [1., 0., 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.]
    ])
    assert np.array_equal(mI, oracle_identity) is True
    assert np.allclose(mI, np.identity(4, dtype=np.float64)) is True
    assert np.sum(mI) == 4.0
    assert np.trace(mI) == 4.0


def test_translation_matrix():
    direction_vector = np.array([1., 2., 3.])
    M = tf.translation_matrix(direction_vector)
    assert np.array_equal(M[:3, 3], direction_vector[:3]) is True
    assert np.sum(M) == 4.0 + np.sum(direction_vector)
    assert np.trace(M) == 4.0


def test_translation_from_matrix():
    direction_vector = np.array([1., 2., 3.])
    M = tf.translation_matrix(direction_vector)
    translation_vector = tf.translation_from_matrix(M)
    assert np.array_equal(translation_vector, direction_vector) is True


def test_reflection_matrix():
    v0 = np.random.random(4) - 0.5
    v0[3] = 1.0
    v1 = np.random.random(3) - 0.5
    R = tf.reflection_matrix(v0, v1)
    assert np.allclose(2., np.trace(R)) is True
    assert np.allclose(v0, np.dot(R, v0)) is True

    v2 = v0.copy()
    v2[:3] += v1
    v3 = v0.copy()
    v2[:3] -= v1
    assert np.allclose(v2, np.dot(R, v3)) is True


def test_reflection_from_matrix():
    v0 = np.random.random(3) - 0.5
    v1 = np.random.random(3) - 0.5
    M0 = tf.reflection_matrix(v0, v1)
    point, normal = tf.reflection_from_matrix(M0)
    M1 = tf.reflection_matrix(point, normal)
    assert tf.is_same_transform(M0, M1) is True


def test_rotation_matrix():
    angle = (random.random() - 0.5) * (2*math.pi)
    direc = np.random.random(3) - 0.5
    point = np.random.random(3) - 0.5
    R0 = tf.rotation_matrix(angle, direc, point)
    R1 = tf.rotation_matrix(angle-2*math.pi, direc, point)
    assert tf.is_same_transform(R0, R1) is True

    R0 = tf.rotation_matrix(angle, direc, point)
    R1 = tf.rotation_matrix(-angle, -direc, point)
    assert tf.is_same_transform(R0, R1) is True

    mI = np.identity(4, np.float64)
    assert np.allclose(mI, tf.rotation_matrix(math.pi*2, direc)) is True
    assert np.allclose(2., np.trace(tf.rotation_matrix(math.pi/2, direc, point))) is True


def test_rotation_from_matrix():
    angle = (random.random() - 0.5) * (2*math.pi)
    direc = np.random.random(3) - 0.5
    point = np.random.random(3) - 0.5
    R0 = tf.rotation_matrix(angle, direc, point)
    angle, direc, point = tf.rotation_from_matrix(R0)
    R1 = tf.rotation_matrix(angle, direc, point)
    assert tf.is_same_transform(R0, R1) is True


def test_scale_matrix():
    v = (np.random.rand(4, 5) - 0.5) * 20.0
    v[3] = 1.0
    S = tf.scale_matrix(-1.234)
    assert np.allclose(np.dot(S, v)[:3], -1.234*v[:3]) is True


def test_scale_from_matrix():
    factor = random.random() * 10 - 5
    origin = np.random.random(3) - 0.5
    direct = np.random.random(3) - 0.5
    S0 = tf.scale_matrix(factor, origin)
    factor, origin, direction = tf.scale_from_matrix(S0)
    S1 = tf.scale_matrix(factor, origin, direction)
    assert tf.is_same_transform(S0, S1) is True

    S0 = tf.scale_matrix(factor, origin, direct)
    factor, origin, direction = tf.scale_from_matrix(S0)
    S1 = tf.scale_matrix(factor, origin, direction)
    assert tf.is_same_transform(S0, S1) is True


def test_projection_matrix():
    P = tf.projection_matrix((0, 0, 0), (1, 0, 0))
    assert np.allclose(P[1:, 1:], np.identity(4)[1:, 1:]) is True

    point = np.random.random(3) - 0.5
    normal = np.random.random(3) - 0.5
    persp = np.random.random(3) - 0.5
    P0 = tf.projection_matrix(point, normal)
    P2 = tf.projection_matrix(point, normal, perspective=persp)
    P3 = tf.projection_matrix(point, normal, perspective=persp, pseudo=True)
    assert tf.is_same_transform(P2, np.dot(P0, P3)) is True

    P = tf.projection_matrix((3, 0, 0), (1, 1, 0), (1, 0, 0))
    v0 = (np.random.rand(4, 5) - 0.5) * 20.0
    v0[3] = 1.0
    v1 = np.dot(P, v0)
    assert np.allclose(v1[1], v0[1]) is True
    assert np.allclose(v1[0], 3.0-v1[1]) is True


def test_projection_from_matrix():
    point = np.random.random(3) - 0.5
    normal = np.random.random(3) - 0.5
    direct = np.random.random(3) - 0.5
    persp = np.random.random(3) - 0.5
    P0 = tf.projection_matrix(point, normal)
    result = tf.projection_from_matrix(P0)
    P1 = tf.projection_matrix(*result)
    assert tf.is_same_transform(P0, P1) is True

    P0 = tf.projection_matrix(point, normal, direct)
    result = tf.projection_from_matrix(P0)
    P1 = tf.projection_matrix(*result)
    assert tf.is_same_transform(P0, P1) is True

    P0 = tf.projection_matrix(point, normal, perspective=persp, pseudo=False)
    result = tf.projection_from_matrix(P0, pseudo=False)
    P1 = tf.projection_matrix(*result)
    assert tf.is_same_transform(P0, P1) is True

    P0 = tf.projection_matrix(point, normal, perspective=persp, pseudo=True)
    result = tf.projection_from_matrix(P0, pseudo=True)
    P1 = tf.projection_matrix(*result)
    assert tf.is_same_transform(P0, P1) is True


def test_clip_matrix():
    frustrum = np.random.rand(6)
    frustrum[1] += frustrum[0]
    frustrum[3] += frustrum[2]
    frustrum[5] += frustrum[4]
    M = tf.clip_matrix(*frustrum, perspective=False)
    assert np.allclose(
                np.dot(M, [frustrum[0], frustrum[2], frustrum[4], 1.0]),
                np.array([-1., -1., -1., 1.])
            ) is True

    assert np.allclose(
                np.dot(M, [frustrum[1], frustrum[3], frustrum[5], 1.0]),
                np.array([1., 1., 1., 1.])
            ) is True

    M = tf.clip_matrix(*frustrum, perspective=True)
    v = np.dot(M, [frustrum[0], frustrum[2], frustrum[4], 1.0])
    assert np.allclose(v / v[3], np.array([-1., -1., -1., 1.])) is True

    v = np.dot(M, [frustrum[1], frustrum[3], frustrum[4], 1.0])
    assert np.allclose(v / v[3], np.array([1., 1., -1., 1.])) is True


def test_shear_matrix():
    angle = (random.random() - 0.5) * 4*math.pi
    direct = np.random.random(3) - 0.5
    point = np.random.random(3) - 0.5
    normal = np.cross(direct, np.random.random(3))
    S = tf.shear_matrix(angle, direct, point, normal)
    assert np.allclose(1.0, np.linalg.det(S)) is True


def test_shear_from_matrix():
    angle = (random.random() - 0.5) * 4*math.pi
    direct = np.random.random(3) - 0.5
    point = np.random.random(3) - 0.5
    normal = np.cross(direct, np.random.random(3))
    S0 = tf.shear_matrix(angle, direct, point, normal)
    angle, direct, point, normal = tf.shear_from_matrix(S0)
    S1 = tf.shear_matrix(angle, direct, point, normal)
    assert tf.is_same_transform(S0, S1) is True


def test_decompose_matrix():
    T0 = tf.translation_matrix((1, 2, 3))
    scale, shear, angles, trans, persp = tf.decompose_matrix(T0)
    T1 = tf.translation_matrix(trans)
    assert np.allclose(T0, T1) is True

    S = tf.scale_matrix(0.123)
    scale, shear, angles, trans, persp = tf.decompose_matrix(S)
    assert scale[0] == 0.123

    R0 = tf.euler_matrix(1, 2, 3)
    scale, shear, angles, trans, persp = tf.decompose_matrix(R0)
    R1 = tf.euler_matrix(*angles)
    assert np.allclose(R0, R1) is True


def test_compose_matrix():
    scale = np.random.random(3) - 0.5
    shear = np.random.random(3) - 0.5
    angles = (np.random.random(3) - 0.5) * (2*math.pi)
    trans = np.random.random(3) - 0.5
    persp = np.random.random(4) - 0.5
    M0 = tf.compose_matrix(scale, shear, angles, trans, persp)
    result = tf.decompose_matrix(M0)
    M1 = tf.compose_matrix(*result)
    assert tf.is_same_transform(M0, M1) is True


def test_orthogonalization_matrix():
    mO = tf.orthogonalization_matrix((10., 10., 10.), (90., 90., 90.))
    assert np.allclose(mO[:3, :3], np.identity(3, float) * 10) is True

    mO = tf.orthogonalization_matrix([9.8, 12.0, 15.5], [87.2, 80.7, 69.7])
    assert np.allclose(np.sum(mO), 43.063229) is True


def test_superimposition_matrix():
    v0 = np.random.rand(3, 10)
    M = tf.superimposition_matrix(v0, v0)
    assert np.allclose(M, np.identity(4)) is True

    R = tf.random_rotation_matrix(np.random.random(3))
    v0 = ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 1))
    v1 = np.dot(R, v0)
    M = tf.superimposition_matrix(v0, v1)
    assert np.allclose(v1, np.dot(M, v0)) is True

    v0 = (np.random.rand(4, 100) - 0.5) * 20.0
    v0[3] = 1.0
    v1 = np.dot(R, v0)
    M = tf.superimposition_matrix(v0, v1)
    assert np.allclose(v1, np.dot(M, v0)) is True

    S = tf.scale_matrix(random.random())
    T = tf.translation_matrix(np.random.random(3)-0.5)
    M = tf.concatenate_matrices(T, R, S)
    v1 = np.dot(M, v0)
    v0[:3] += np.random.normal(0.0, 1e-9, 300).reshape(3, -1)
    M = tf.superimposition_matrix(v0, v1, scaling=True)
    assert np.allclose(v1, np.dot(M, v0)) is True

    M = tf.superimposition_matrix(v0, v1, scaling=True, usesvd=False)
    assert np.allclose(v1, np.dot(M, v0)) is True

    v = np.empty((4, 100, 3), dtype=np.float64)
    v[:, :, 0] = v0
    M = tf.superimposition_matrix(v0, v1, scaling=True, usesvd=False)
    assert np.allclose(v1, np.dot(M, v[:, :, 0])) is True


def test_euler_matrix():
    R = tf.euler_matrix(1, 2, 3, 'syxz')
    assert np.allclose(np.sum(R[0]), -1.34786452) is True

    R = tf.euler_matrix(1, 2, 3, (0, 1, 0, 1))
    assert np.allclose(np.sum(R[0]), -0.383436184) is True


def test_euler_from_matrix():
    R0 = tf.euler_matrix(1, 2, 3, 'syxz')
    al, be, ga = tf.euler_from_matrix(R0, 'syxz')
    R1 = tf.euler_matrix(al, be, ga, 'syxz')
    assert np.allclose(R0, R1) is True

    angles = (4.0*math.pi) * (np.random.random(3) - 0.5)
    for axes in tf._AXES2TUPLE.keys():
        R0 = tf.euler_matrix(axes=axes, *angles)
        R1 = tf.euler_matrix(axes=axes, *tf.euler_from_matrix(R0, axes))
        assert np.allclose(R0, R1) is True


def test_euler_from_quaternion():
    angles = tf.euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    assert np.allclose(angles, [0.123, 0, 0]) is True


def test_quaternion_from_euler():
    q = tf.quaternion_from_euler(1, 2, 3, 'ryxz')
    assert np.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953]) is True


def test_quaternion_about_axis():
    q = tf.quaternion_about_axis(0.123, (1, 0, 0))
    assert np.allclose(q, [0.06146124, 0, 0, 0.99810947]) is True


def test_quaternion_matrix():
    R = tf.quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    assert np.allclose(R, tf.rotation_matrix(0.123, (1, 0, 0))) is True


def test_quaternion_from_matrix():
    R = tf.rotation_matrix(0.123, (1, 2, 3))
    q = tf.quaternion_from_matrix(R)
    assert np.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095]) is True


def test_quaternion_multiply():
    q = tf.quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    assert np.allclose(q, [-44, -14, 48, 28]) is True


def test_quaternion_conjugate():
    q0 = tf.random_quaternion()
    q1 = tf.quaternion_conjugate(q0)
    assert q1[3] == q0[3] and all(q1[:3] == -q0[:3]) is True


def test_quaternion_inverse():
    q0 = tf.random_quaternion()
    q1 = tf.quaternion_inverse(q0)
    assert np.allclose(tf.quaternion_multiply(q0, q1), [0, 0, 0, 1]) is True


def test_quaternion_slerp():
    q0 = tf.random_quaternion()
    q1 = tf.random_quaternion()
    q = tf.quaternion_slerp(q0, q1, 0.0)
    assert np.allclose(q, q0) is True

    q = tf.quaternion_slerp(q0, q1, 1.0, 1)
    assert np.allclose(q, q1) is True

    q = tf.quaternion_slerp(q0, q1, 0.5)
    angle = math.acos(np.dot(q0, q))
    assert np.allclose(2.0, math.acos(np.dot(q0, q1)) / angle) or \
        np.allclose(2.0, math.acos(-np.dot(q0, q1)) / angle) is True


def test_random_quaternion():
    q = tf.random_quaternion()
    assert np.allclose(1.0, tf.vector_norm(q)) is True

    q = tf.random_quaternion(np.random.random(3))
    assert q.shape == (4,)


def test_random_rotation_matrix():
    R = tf.random_rotation_matrix()
    assert np.allclose(np.dot(R.T, R), np.identity(4)) is True
