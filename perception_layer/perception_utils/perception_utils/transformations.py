"""
Homogeneous Transformation Matrices and Quaternions.

A library for calculating 4x4 matrices for translating, rotating, reflecting,
scaling, shearing, projecting, orthogonalizing, and superimposing arrays of
3D homogeneous coordinates as well as for converting between rotation matrices,
Euler angles, and quaternions. Also includes an Arcball control object and
functions to decompose transformation matrices.

:Authors:
  `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`__,
  Laboratory for Fluorescence Dynamics, University of California, Irvine

:Version: 20090418

Requirements
------------
* `Python 2.6 <http://www.python.org>`__
* `np 1.3 <http://np.scipy.org>`__
* `transformations.c 20090418 <http://www.lfd.uci.edu/~gohlke/>`__
  (optional implementation of some functions in C)

Notes
-----
Matrices (M) can be inverted using np.linalg.inv(M), concatenated using
np.dot(M0, M1), or used to transform homogeneous coordinates (v) using
np.dot(M, v) for shape (4, *) "point of arrays", respectively
np.dot(v, M.T) for shape (*, 4) "array of points".

Calculations are carried out with np.float64 precision.

This Python implementation is not optimized for speed.

Vector, point, quaternion, and matrix function arguments are expected to be
"array like", i.e. tuple, list, or np arrays.

Return types are np arrays unless specified otherwise.

Angles are in radians unless specified otherwise.

Quaternions ix+jy+kz+w are represented as [x, y, z, w].

Use the transpose of transformation matrices for OpenGL glMultMatrixd().

A triple of Euler angles can be applied/interpreted in 24 ways, which can
be specified using a 4 character string or encoded 4-tuple:

  *Axes 4-string*: e.g. 'sxyz' or 'ryxy'

  - first character : rotations are applied to 's'tatic or 'r'otating frame
  - remaining characters : successive rotation axis 'x', 'y', or 'z'

  *Axes 4-tuple*: e.g. (0, 0, 0, 0) or (1, 1, 1, 1)

  - inner axis: code of axis ('x':0, 'y':1, 'z':2) of rightmost matrix.
  - parity : even (0) if inner axis 'x' is followed by 'y', 'y' is followed
    by 'z', or 'z' is followed by 'x'. Otherwise odd (1).
  - repetition : first and last axis are same (1) or different (0).
  - frame : rotations are applied to static (0) or rotating (1) frame.

References
----------
(1)  Matrices and transformations. Ronald Goldman.
     In "Graphics Gems I", pp 472-475. Morgan Kaufmann, 1990.
(2)  More matrices and transformations: shear and pseudo-perspective.
     Ronald Goldman. In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
(3)  Decomposing a matrix into simple transformations. Spencer Thomas.
     In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
(4)  Recovering the data from the transformation matrix. Ronald Goldman.
     In "Graphics Gems II", pp 324-331. Morgan Kaufmann, 1991.
(5)  Euler angle conversion. Ken Shoemake.
     In "Graphics Gems IV", pp 222-229. Morgan Kaufmann, 1994.
(6)  Arcball rotation control. Ken Shoemake.
     In "Graphics Gems IV", pp 175-192. Morgan Kaufmann, 1994.
(7)  Representing attitude: Euler angles, unit quaternions, and rotation
     vectors. James Diebel. 2006.
(8)  A discussion of the solution for the best rotation to relate two sets
     of vectors. W Kabsch. Acta Cryst. 1978. A34, 827-828.
(9)  Closed-form solution of absolute orientation using unit quaternions.
     BKP Horn. J Opt Soc Am A. 1987. 4(4), 629-642.
(10) Quaternions. Ken Shoemake.
     http://www.sfu.ca/~jwa3/cmpt461/files/quatut.pdf
(11) From quaternion to matrix and back. JMP van Waveren. 2005.
     http://www.intel.com/cd/ids/developer/asmo-na/eng/293748.htm
(12) Uniform random rotations. Ken Shoemake.
     In "Graphics Gems III", pp 124-132. Morgan Kaufmann, 1992.


Examples
--------
alpha, beta, gamma = 0.123, -1.234, 2.345
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
I = identity_matrix()
Rx = rotation_matrix(alpha, xaxis)
Ry = rotation_matrix(beta, yaxis)
Rz = rotation_matrix(gamma, zaxis)
R = concatenate_matrices(Rx, Ry, Rz)
euler = euler_from_matrix(R, 'rxyz')
np.allclose([alpha, beta, gamma], euler)
--> True

Re = euler_matrix(alpha, beta, gamma, 'rxyz')
is_same_transform(R, Re)
True
al, be, ga = euler_from_matrix(Re, 'rxyz')
is_same_transform(Re, euler_matrix(al, be, ga, 'rxyz'))
--> True

qx = quaternion_about_axis(alpha, xaxis)
qy = quaternion_about_axis(beta, yaxis)
qz = quaternion_about_axis(gamma, zaxis)
q = quaternion_multiply(qx, qy)
q = quaternion_multiply(q, qz)
Rq = quaternion_matrix(q)
is_same_transform(R, Rq)
--> True

S = scale_matrix(1.23, origin)
T = translation_matrix((1, 2, 3))
Z = shear_matrix(beta, xaxis, origin, zaxis)
R = random_rotation_matrix(np.random.rand(3))
M = concatenate_matrices(T, R, Z, S)
scale, shear, angles, trans, persp = decompose_matrix(M)
np.allclose(scale, 1.23)
--> True

np.allclose(trans, (1, 2, 3))
--> True

np.allclose(shear, (0, math.tan(beta), 0))
--> True

is_same_transform(R, euler_matrix(axes='sxyz', *angles))
--> True

M1 = compose_matrix(scale, shear, angles, trans, persp)
is_same_transform(M, M1)
--> True

"""

# Copyright (c) 2006, Christoph Gohlke
# Copyright (c) 2006-2009, The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holders nor the names of any
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

import math
from typing import List, Tuple
import warnings

import numpy as np

# Documentation in HTML format can be generated with Epydoc
__docformat__ = 'restructuredtext en'


def identity_matrix() -> np.ndarray:
    """Return 4x4 identity/unit matrix."""
    return np.identity(4, dtype=np.float64)


def translation_matrix(direction: np.ndarray) -> np.ndarray:
    """Return matrix to translate by direction vector."""
    M = np.identity(4)
    M[:3, 3] = direction[:3]
    return M


def translation_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Return translation vector from translation matrix."""
    return np.array(matrix, copy=False)[:3, 3].copy()


def reflection_matrix(
    point: np.ndarray,
    normal: np.ndarray
) -> np.ndarray:
    """
    Return matrix to mirror at plane defined by point and normal vector.

    Reference: https://en.wikipedia.org/wiki/Transformation_matrix#Reflection_2

    """
    normal = unit_vector(normal[:3])
    M = np.identity(4)
    M[:3, :3] -= 2.0 * np.outer(normal, normal)
    M[:3, 3] = (2.0 * np.dot(point[:3], normal)) * normal
    return M


def reflection_from_matrix(matrix):
    """Return mirror plane point and normal vector from reflection matrix."""
    M = np.array(matrix, dtype=np.float64, copy=False)
    # normal: unit eigenvector corresponding to eigenvalue -1
    l, V = np.linalg.eig(M[:3, :3])
    i = np.where(abs(np.real(l) + 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError('No unit eigenvector corresponding to eigenvalue -1')
    normal = np.real(V[:, i[0]]).squeeze()
    # point: any unit eigenvector corresponding to eigenvalue 1
    l, V = np.linalg.eig(M)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError('No unit eigenvector corresponding to eigenvalue 1')
    point = np.real(V[:, i[-1]]).squeeze()
    point /= point[3]
    return point, normal


def rotation_matrix(
    angle: float,
    direction: np.ndarray,
    point: np.ndarray = None
) -> np.ndarray:
    """Return matrix to rotate about axis defined by point and direction."""
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.array(
        ((cosa, 0.0, 0.0), (0.0, cosa, 0.0), (0.0,  0.0, cosa)), dtype=np.float64)
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array(
        (
            (0.0, -direction[2], direction[1]),
            (direction[2], 0.0, -direction[0]),
            (-direction[1], direction[0],  0.0)
        ),
        dtype=np.float64
    )
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float64, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M


def rotation_from_matrix(matrix: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray]:
    """Return rotation angle and axis from rotation matrix."""
    R = np.array(matrix, dtype=np.float64, copy=False)
    R33 = R[:3, :3]
    # direction: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = np.linalg.eig(R33.T)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError('No unit eigenvector corresponding to eigenvalue 1')
    direction = np.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = np.linalg.eig(R)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError('No unit eigenvector corresponding to eigenvalue 1')
    point = np.real(Q[:, i[-1]]).squeeze()
    point /= point[3]
    # rotation angle depending on direction
    cosa = (np.trace(R33) - 1.0) / 2.0
    if abs(direction[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*direction[0]*direction[1]) / direction[2]
    elif abs(direction[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*direction[0]*direction[2]) / direction[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*direction[1]*direction[2]) / direction[0]
    angle = math.atan2(sina, cosa)
    return angle, direction, point


def scale_matrix(
    factor: float,
    origin: np.ndarray = None,
    direction: np.ndarray = None
) -> np.ndarray:
    """
    Return matrix to scale by factor around origin in direction.

    Use factor -1 for point symmetry.

    """
    if direction is None:
        # uniform scaling
        M = np.array(
            (
                (factor, 0.0, 0.0, 0.0),
                (0.0, factor, 0.0, 0.0),
                (0.0, 0.0, factor, 0.0),
                (0.0, 0.0, 0.0, 1.0)
            ),
            dtype=np.float64
        )
        if origin is not None:
            M[:3, 3] = origin[:3]
            M[:3, 3] *= 1.0 - factor
    else:
        # nonuniform scaling
        direction = unit_vector(direction[:3])
        factor = 1.0 - factor
        M = np.identity(4)
        M[:3, :3] -= factor * np.outer(direction, direction)
        if origin is not None:
            M[:3, 3] = (factor * np.dot(origin[:3], direction)) * direction
    return M


def scale_from_matrix(matrix: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray]:
    """Return scaling factor, origin and direction from scaling matrix."""
    M = np.array(matrix, dtype=np.float64, copy=False)
    M33 = M[:3, :3]
    factor = np.trace(M33) - 2.0
    try:
        # direction: unit eigenvector corresponding to eigenvalue factor
        l, V = np.linalg.eig(M33)
        i = np.where(abs(np.real(l) - factor) < 1e-8)[0][0]
        direction = np.real(V[:, i]).squeeze()
        direction /= vector_norm(direction)
    except IndexError:
        # uniform scaling
        factor = (factor + 2.0) / 3.0
        direction = None
    # origin: any eigenvector corresponding to eigenvalue 1
    l, V = np.linalg.eig(M)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError('No eigenvector corresponding to eigenvalue 1')
    origin = np.real(V[:, i[-1]]).squeeze()
    origin /= origin[3]
    return factor, origin, direction


def projection_matrix(
    point: np.ndarray,
    normal: np.ndarray,
    direction: np.ndarray = None,
    perspective: np.ndarray = None,
    pseudo: bool = False
) -> np.ndarray:
    """
    Return matrix to project onto plane defined by point and normal.

    Using either perspective point, projection direction, or none of both.

    If pseudo is True, perspective projections will preserve relative depth
    such that Perspective = dot(Orthogonal, PseudoPerspective).

    """
    M = np.identity(4)
    point = np.array(point[:3], dtype=np.float64, copy=False)
    normal = unit_vector(normal[:3])
    if perspective is not None:
        # perspective projection
        perspective = np.array(
            perspective[:3], dtype=np.float64, copy=False)
        M[0, 0] = M[1, 1] = M[2, 2] = np.dot(perspective-point, normal)
        M[:3, :3] -= np.outer(perspective, normal)
        if pseudo:
            # preserve relative depth
            M[:3, :3] -= np.outer(normal, normal)
            M[:3, 3] = np.dot(point, normal) * (perspective+normal)
        else:
            M[:3, 3] = np.dot(point, normal) * perspective
        M[3, :3] = -normal
        M[3, 3] = np.dot(perspective, normal)
    elif direction is not None:
        # parallel projection
        direction = np.array(direction[:3], dtype=np.float64, copy=False)
        scale = np.dot(direction, normal)
        M[:3, :3] -= np.outer(direction, normal) / scale
        M[:3, 3] = direction * (np.dot(point, normal) / scale)
    else:
        # orthogonal projection
        M[:3, :3] -= np.outer(normal, normal)
        M[:3, 3] = np.dot(point, normal) * normal
    return M


def projection_from_matrix(
    matrix: np.ndarray,
    pseudo: bool = False
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]:
    """
    Return projection plane and perspective point from projection matrix.

    Return values are same as arguments for projection_matrix function:
    point, normal, direction, perspective, and pseudo.

    Some return values can be None.

    """
    M = np.array(matrix, dtype=np.float64, copy=False)
    M33 = M[:3, :3]
    l, V = np.linalg.eig(M)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not pseudo and len(i):
        # point: any eigenvector corresponding to eigenvalue 1
        point = np.real(V[:, i[-1]]).squeeze()
        point /= point[3]
        # direction: unit eigenvector corresponding to eigenvalue 0
        l, V = np.linalg.eig(M33)
        i = np.where(abs(np.real(l)) < 1e-8)[0]
        if not len(i):
            raise ValueError('No eigenvector corresponding to eigenvalue 0')
        direction = np.real(V[:, i[0]]).squeeze()
        direction /= vector_norm(direction)
        # normal: unit eigenvector of M33.T corresponding to eigenvalue 0
        l, V = np.linalg.eig(M33.T)
        i = np.where(abs(np.real(l)) < 1e-8)[0]
        if len(i):
            # parallel projection
            normal = np.real(V[:, i[0]]).squeeze()
            normal /= vector_norm(normal)
            return point, normal, direction, None, False
        else:
            # orthogonal projection, where normal equals direction vector
            return point, direction, None, None, False
    else:
        # perspective projection
        i = np.where(abs(np.real(l)) > 1e-8)[0]
        if not len(i):
            raise ValueError(
                'No eigenvector not corresponding to eigenvalue 0')
        point = np.real(V[:, i[-1]]).squeeze()
        point /= point[3]
        normal = - M[3, :3]
        perspective = M[:3, 3] / np.dot(point[:3], normal)
        if pseudo:
            perspective -= normal
        return point, normal, None, perspective, pseudo


def clip_matrix(
    left: float,
    right: float,
    bottom: float,
    top: float,
    near: float,
    far: float,
    perspective: bool = False
) -> np.ndarray:
    """
    Return matrix to obtain normalized device coordinates from frustum.

    The frustum bounds are axis-aligned along x (left, right),
    y (bottom, top) and z (near, far).

    Normalized device coordinates are in range [-1, 1] if coordinates are
    inside the frustum.

    If perspective is True the frustum is a truncated pyramid with the
    perspective point at origin and direction along z axis, otherwise an
    orthographic canonical view volume (a box).

    Homogeneous coordinates transformed by the perspective clip matrix
    need to be dehomogenized (devided by w coordinate).

    """
    if left >= right or bottom >= top or near >= far:
        raise ValueError('Invalid frustum')
    if perspective:
        if near <= _EPS:
            raise ValueError('Invalid frustum: near <= 0')
        t = 2.0 * near
        M = ((-t/(right-left), 0.0, (right+left)/(right-left), 0.0),
             (0.0, -t/(top-bottom), (top+bottom)/(top-bottom), 0.0),
             (0.0, 0.0, -(far+near)/(far-near), t*far/(far-near)),
             (0.0, 0.0, -1.0, 0.0))
    else:
        M = ((2.0/(right-left), 0.0, 0.0, (right+left)/(left-right)),
             (0.0, 2.0/(top-bottom), 0.0, (top+bottom)/(bottom-top)),
             (0.0, 0.0, 2.0/(far-near), (far+near)/(near-far)),
             (0.0, 0.0, 0.0, 1.0))
    return np.array(M, dtype=np.float64)


def shear_matrix(
    angle: float,
    direction: np.ndarray,
    point: np.ndarray,
    normal: np.ndarray
) -> np.ndarray:
    """
    Return matrix to shear by angle along direction vector on shear plane.

    The shear plane is defined by a point and normal vector. The direction
    vector must be orthogonal to the plane's normal vector.

    A point P is transformed by the shear matrix into P" such that
    the vector P-P" is parallel to the direction vector and its extent is
    given by the angle of P-P'-P", where P' is the orthogonal projection
    of P onto the shear plane.

    """
    normal = unit_vector(normal[:3])
    direction = unit_vector(direction[:3])
    if abs(np.dot(normal, direction)) > 1e-6:
        raise ValueError('Direction and normal vectors are not orthogonal')
    angle = math.tan(angle)
    M = np.identity(4)
    M[:3, :3] += angle * np.outer(direction, normal)
    M[:3, 3] = -angle * np.dot(point[:3], normal) * direction
    return M


def shear_from_matrix(
    matrix: np.ndarray
) -> Tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """Return shear angle, direction and plane from shear matrix."""
    M = np.array(matrix, dtype=np.float64, copy=False)
    M33 = M[:3, :3]
    # normal: cross independent eigenvectors corresponding to the eigenvalue 1
    l, V = np.linalg.eig(M33)
    i = np.where(abs(np.real(l) - 1.0) < 1e-4)[0]
    if len(i) < 2:
        raise ValueError('No two linear independent eigenvectors found %s' % l)
    V = np.real(V[:, i]).squeeze().T
    lenorm = -1.0
    for i0, i1 in ((0, 1), (0, 2), (1, 2)):
        n_ = np.cross(V[i0], V[i1])
        l_ = vector_norm(n_)
        if l_ > lenorm:
            lenorm = l_
            normal = n_
    normal /= lenorm
    # direction and angle
    direction = np.dot(M33 - np.identity(3), normal)
    angle = vector_norm(direction)
    direction /= angle
    angle = math.atan(angle)
    # point: eigenvector corresponding to eigenvalue 1
    l, V = np.linalg.eig(M)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError('No eigenvector corresponding to eigenvalue 1')
    point = np.real(V[:, i[-1]]).squeeze()
    point /= point[3]
    return angle, direction, point, normal


def decompose_matrix(
    matrix: np.ndarray
) -> Tuple[np.ndarray, List[float], List[float], np.ndarray, np.ndarray]:
    """
    Return sequence of transformations from transformation matrix.

    Args
    ----
        matrix : Non-degenerative homogeneous transformation matrix

    Returns
    -------
        scale : vector of 3 scaling factors
        shear : list of shear factors for x-y, x-z, y-z axes
        angles : list of Euler angles about static x, y, z axes
        translate : translation vector along x, y, z axes
        perspective : perspective partition of matrix

    Raise ValueError if matrix is of wrong type or degenerative.

    """
    M = np.array(matrix, dtype=np.float64, copy=True).T
    if abs(M[3, 3]) < _EPS:
        raise ValueError('M[3, 3] is zero')
    M /= M[3, 3]
    P = M.copy()
    P[:, 3] = 0, 0, 0, 1
    if not np.linalg.det(P):
        raise ValueError('Matrix is singular')

    scale = np.zeros((3, ), dtype=np.float64)
    shear = [0, 0, 0]
    angles = [0, 0, 0]

    if any(abs(M[:3, 3]) > _EPS):
        perspective = np.dot(M[:, 3], np.linalg.inv(P.T))
        M[:, 3] = 0, 0, 0, 1
    else:
        perspective = np.array((0, 0, 0, 1), dtype=np.float64)

    translate = M[3, :3].copy()
    M[3, :3] = 0

    row = M[:3, :3].copy()
    scale[0] = vector_norm(row[0])
    row[0] /= scale[0]
    shear[0] = np.dot(row[0], row[1])
    row[1] -= row[0] * shear[0]
    scale[1] = vector_norm(row[1])
    row[1] /= scale[1]
    shear[0] /= scale[1]
    shear[1] = np.dot(row[0], row[2])
    row[2] -= row[0] * shear[1]
    shear[2] = np.dot(row[1], row[2])
    row[2] -= row[1] * shear[2]
    scale[2] = vector_norm(row[2])
    row[2] /= scale[2]
    shear[1:] /= scale[2]

    if np.dot(row[0], np.cross(row[1], row[2])) < 0:
        scale *= -1
        row *= -1

    angles[1] = math.asin(-row[0, 2])
    if math.cos(angles[1]):
        angles[0] = math.atan2(row[1, 2], row[2, 2])
        angles[2] = math.atan2(row[0, 1], row[0, 0])
    else:
        # angles[0] = math.atan2(row[1, 0], row[1, 1])
        angles[0] = math.atan2(-row[2, 1], row[1, 1])
        angles[2] = 0.0

    return scale, shear, angles, translate, perspective


def compose_matrix(
    scale: np.ndarray = None,
    shear: List[float] = None,
    angles: List[float] = None,
    translate: np.ndarray = None,
    perspective: np.ndarray = None
) -> np.ndarray:
    """
    Return transformation matrix from sequence of transformations.

    This is the inverse of the decompose_matrix function.

    Args:
    ----
        scale : vector of 3 scaling factors
        shear : list of shear factors for x-y, x-z, y-z axes
        angles : list of Euler angles about static x, y, z axes
        translate : translation vector along x, y, z axes
        perspective : perspective partition of matrix

    """
    M = np.identity(4)
    if perspective is not None:
        P = np.identity(4)
        P[3, :] = perspective[:4]
        M = np.dot(M, P)
    if translate is not None:
        T = np.identity(4)
        T[:3, 3] = translate[:3]
        M = np.dot(M, T)
    if angles is not None:
        R = euler_matrix(angles[0], angles[1], angles[2], 'sxyz')
        M = np.dot(M, R)
    if shear is not None:
        Z = np.identity(4)
        Z[1, 2] = shear[2]
        Z[0, 2] = shear[1]
        Z[0, 1] = shear[0]
        M = np.dot(M, Z)
    if scale is not None:
        S = np.identity(4)
        S[0, 0] = scale[0]
        S[1, 1] = scale[1]
        S[2, 2] = scale[2]
        M = np.dot(M, S)
    M /= M[3, 3]
    return M


def orthogonalization_matrix(
    lengths: List[float],
    angles: List[float]
) -> np.ndarray:
    """
    Return orthogonalization matrix for crystallographic cell coordinates.

    Angles are expected in degrees.

    The de-orthogonalization matrix is the inverse.

    """
    a, b, c = lengths
    angles = np.radians(angles)
    sina, sinb, _ = np.sin(angles)
    cosa, cosb, cosg = np.cos(angles)
    co = (cosa * cosb - cosg) / (sina * sinb)
    return np.array((
        (a*sinb*math.sqrt(1.0-co*co), 0.0, 0.0, 0.0),
        (-a*sinb*co, b*sina, 0.0, 0.0),
        (a*cosb, b*cosa, c, 0.0),
        (0.0, 0.0, 0.0, 1.0)),
        dtype=np.float64)


def superimposition_matrix(
    v0: np.ndarray,
    v1: np.ndarray,
    scaling: bool = False,
    usesvd: bool = True
) -> np.ndarray:
    """
    Return matrix to transform given vector set into second vector set.

    Args:
    ----
        v0: shape (3, *) or (4, *) arrays of at least 3 vectors.
        v1: shape (3, *) or (4, *) arrays of at least 3 vectors.
        scaling: True scaling is desired.
        usesvd: True if SVD decomposition is used.

    If usesvd is True, the weighted sum of squared deviations (RMSD) is
    minimized according to the algorithm by W. Kabsch [8]. Otherwise the
    quaternion based algorithm by B. Horn [9] is used (slower when using
    this Python implementation).

    The returned matrix performs rotation, translation and uniform scaling
    (if specified).

    """
    v0 = np.array(v0, dtype=np.float64, copy=False)[:3]
    v1 = np.array(v1, dtype=np.float64, copy=False)[:3]

    if v0.shape != v1.shape or v0.shape[1] < 3:
        raise ValueError('Vector sets are of wrong shape or type.')

    # move centroids to origin
    t0 = np.mean(v0, axis=1)
    t1 = np.mean(v1, axis=1)
    v0 = v0 - t0.reshape(3, 1)
    v1 = v1 - t1.reshape(3, 1)

    if usesvd:
        # Singular Value Decomposition of covariance matrix
        u, s, vh = np.linalg.svd(np.dot(v1, v0.T))
        # rotation matrix from SVD orthonormal bases
        R = np.dot(u, vh)
        if np.linalg.det(R) < 0.0:
            # R does not constitute right handed system
            R -= np.outer(u[:, 2], vh[2, :]*2.0)
            s[-1] *= -1.0
        # homogeneous transformation matrix
        M = np.identity(4)
        M[:3, :3] = R
    else:
        # compute symmetric matrix N
        xx, yy, zz = np.sum(v0 * v1, axis=1)
        xy, yz, zx = np.sum(v0 * np.roll(v1, -1, axis=0), axis=1)
        xz, yx, zy = np.sum(v0 * np.roll(v1, -2, axis=0), axis=1)
        N = ((xx+yy+zz, yz-zy,    zx-xz,    xy-yx),
             (yz-zy,    xx-yy-zz, xy+yx,    zx+xz),
             (zx-xz,    xy+yx,   -xx+yy-zz, yz+zy),
             (xy-yx,    zx+xz,    yz+zy,   -xx-yy+zz))
        # quaternion: eigenvector corresponding to most positive eigenvalue
        l, V = np.linalg.eig(N)
        q = V[:, np.argmax(l)]
        q /= vector_norm(q)  # unit quaternion
        q = np.roll(q, -1)  # move w component to end
        # homogeneous transformation matrix
        M = quaternion_matrix(q)

    # scale: ratio of rms deviations from centroid
    if scaling:
        v0 *= v0
        v1 *= v1
        M[:3, :3] *= math.sqrt(np.sum(v1) / np.sum(v0))

    # translation
    M[:3, 3] = t1
    T = np.identity(4)
    T[:3, 3] = -t0
    M = np.dot(M, T)
    return M


def euler_matrix(
    ai: float,
    aj: float,
    ak: float,
    axes: str = 'sxyz'
) -> np.ndarray:
    """
    Return homogeneous rotation matrix from Euler angles and axis sequence.

    Args:
    ----
        ai: Euler's roll angle.
        aj: Euler's pitch angle.
        ak: Euler's yaw angle.
        axes : One of 24 axis sequences as string or encoded tuple.

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = np.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M


def euler_from_matrix(
    matrix: np.ndarray,
    axes: str = 'sxyz'
) -> Tuple[float, float, float]:
    """
    Return Euler angles from rotation matrix for specified axis sequence.

    Args:
    ----
        matrix: matrix to extract the Euler angles form.
        axes : One of 24 axis sequences as string or encoded tuple.

    Note that many Euler angle triplets can describe one matrix.

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j],  M[i, k])
            ay = math.atan2(sy,       M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2(M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def euler_from_quaternion(
    quaternion: np.ndarray,
    axes: str = 'sxyz'
) -> np.ndarray:
    """Return Euler angles from quaternion for specified axis sequence."""
    return euler_from_matrix(quaternion_matrix(quaternion), axes)


def quaternion_from_euler(
    ai: float,
    aj: float,
    ak: float,
    axes: str = 'sxyz'
) -> np.ndarray:
    """
    Return quaternion from Euler angles and axis sequence.

    Args:
    ----
        ai: Euler's roll angle.
        aj: Euler's pitch angle.
        ak: Euler's yaw angle.
        axes : One of 24 axis sequences as string or encoded tuple.

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    quaternion = np.empty((4, ), dtype=np.float64)
    if repetition:
        quaternion[i] = cj*(cs + sc)
        quaternion[j] = sj*(cc + ss)
        quaternion[k] = sj*(cs - sc)
        quaternion[3] = cj*(cc - ss)
    else:
        quaternion[i] = cj*sc - sj*cs
        quaternion[j] = cj*ss + sj*cc
        quaternion[k] = cj*cs - sj*sc
        quaternion[3] = cj*cc + sj*ss
    if parity:
        quaternion[j] *= -1

    return quaternion


def quaternion_about_axis(
    angle: float,
    axis: np.ndarray
) -> np.ndarray:
    """Return quaternion for rotation about axis."""
    quaternion = np.zeros((4, ), dtype=np.float64)
    quaternion[:3] = axis[:3]
    qlen = vector_norm(quaternion)
    if qlen > _EPS:
        quaternion *= math.sin(angle/2.0) / qlen
    quaternion[3] = math.cos(angle/2.0)
    return quaternion


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Return homogeneous rotation matrix from quaternion."""
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2], q[0, 1]-q[2, 3], q[0, 2]+q[1, 3], 0.0),
        (q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2], q[1, 2]-q[0, 3], 0.0),
        (q[0, 2]-q[1, 3], q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (0.0, 0.0, 0.0, 1.0)
    ), dtype=np.float64)


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Return quaternion from rotation matrix."""
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q


def quaternion_multiply(quaternion1: np.ndarray, quaternion0: np.ndarray) -> np.ndarray:
    """Return multiplication of two quaternions."""
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array((
        x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
        x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=np.float64)


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    """Return conjugate of quaternion."""
    return np.array(
        (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]), dtype=np.float64)


def quaternion_inverse(quaternion: np.ndarray) -> np.ndarray:
    """Return inverse of quaternion."""
    return quaternion_conjugate(quaternion) / np.dot(quaternion, quaternion)


def quaternion_slerp(
    quat0: np.ndarray,
    quat1: np.ndarray,
    fraction: float,
    spin: int = 0,
    shortestpath: bool = True
) -> np.ndarray:
    """Return spherical linear interpolation between two quaternions."""
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    if fraction == 0.0:
        return q0
    elif fraction == 1.0:
        return q1
    d = np.dot(q0, q1)
    if abs(abs(d) - 1.0) < _EPS:
        return q0
    if shortestpath and d < 0.0:
        # invert rotation
        d = -d
        q1 *= -1.0
    angle = math.acos(d) + spin * math.pi
    if abs(angle) < _EPS:
        return q0
    isin = 1.0 / math.sin(angle)
    q0 *= math.sin((1.0 - fraction) * angle) * isin
    q1 *= math.sin(fraction * angle) * isin
    q0 += q1
    return q0


def random_quaternion(rand: np.ndarray = None) -> np.ndarray:
    """
    Return uniform random unit quaternion.

    Args:
    ----
        rand: Three independent random variables that are uniformly distributed
              between 0 and 1.

    """
    if rand is None:
        rand = np.random.rand(3)
    else:
        assert len(rand) == 3
    r1 = np.sqrt(1.0 - rand[0])
    r2 = np.sqrt(rand[0])
    pi2 = math.pi * 2.0
    t1 = pi2 * rand[1]
    t2 = pi2 * rand[2]
    return np.array((np.sin(t1)*r1,
                     np.cos(t1)*r1,
                     np.sin(t2)*r2,
                     np.cos(t2)*r2), dtype=np.float64)


def random_rotation_matrix(rand: np.ndarray = None) -> np.ndarray:
    """
    Return uniform random rotation matrix.

    Args:
    ----
        rand: Three independent random variables that are uniformly distributed
              between 0 and 1 for each returned quaternion.

    """
    return quaternion_matrix(random_quaternion(rand))


class Arcball(object):
    """
    Virtual Trackball Control.

    ball = Arcball()
    ball = Arcball(initial=np.identity(4))
    ball.place([320, 320], 320)
    ball.down([500, 250])
    ball.drag([475, 275])
    R = ball.matrix()
    np.allclose(np.sum(R), 3.90583455)
    True
    ball = Arcball(initial=[0, 0, 0, 1])
    ball.place([320, 320], 320)
    ball.setaxes([1,1,0], [-1, 1, 0])
    ball.setconstrain(True)
    ball.down([400, 200])
    ball.drag([200, 400])
    R = ball.matrix()
    np.allclose(np.sum(R), 0.2055924)
    True
    ball.next_rotation()

    """

    def __init__(self, initial=None):
        """
        Initialize virtual trackball control.

        initial : quaternion or rotation matrix

        """
        self._axis = None
        self._axes = None
        self._radius = 1.0
        self._center = [0.0, 0.0]
        self._vdown = np.array([0, 0, 1], dtype=np.float64)
        self._constrain = False

        if initial is None:
            self._qdown = np.array([0, 0, 0, 1], dtype=np.float64)
        else:
            initial = np.array(initial, dtype=np.float64)
            if initial.shape == (4, 4):
                self._qdown = quaternion_from_matrix(initial)
            elif initial.shape == (4, ):
                initial /= vector_norm(initial)
                self._qdown = initial
            else:
                raise ValueError('Initial not a quaternion or matrix.')

        self._qnow = self._qpre = self._qdown

    def place(self, center, radius):
        """
        Place Arcball, e.g. when window size changes.

        center : sequence[2]
            Window coordinates of trackball center.
        radius : float
            Radius of trackball in window coordinates.

        """
        self._radius = float(radius)
        self._center[0] = center[0]
        self._center[1] = center[1]

    def setaxes(self, *axes):
        """Set axes to constrain rotations."""
        if axes is None:
            self._axes = None
        else:
            self._axes = [unit_vector(axis) for axis in axes]

    def setconstrain(self, constrain):
        """Set state of constrain to axis mode."""
        self._constrain = constrain is True

    def getconstrain(self):
        """Return state of constrain to axis mode."""
        return self._constrain

    def down(self, point):
        """Set initial cursor window coordinates and pick constrain-axis."""
        self._vdown = arcball_map_to_sphere(point, self._center, self._radius)
        self._qdown = self._qpre = self._qnow

        if self._constrain and self._axes is not None:
            self._axis = arcball_nearest_axis(self._vdown, self._axes)
            self._vdown = arcball_constrain_to_axis(self._vdown, self._axis)
        else:
            self._axis = None

    def drag(self, point):
        """Update current cursor window coordinates."""
        vnow = arcball_map_to_sphere(point, self._center, self._radius)

        if self._axis is not None:
            vnow = arcball_constrain_to_axis(vnow, self._axis)

        self._qpre = self._qnow

        t = np.cross(self._vdown, vnow)
        if np.dot(t, t) < _EPS:
            self._qnow = self._qdown
        else:
            q = [t[0], t[1], t[2], np.dot(self._vdown, vnow)]
            self._qnow = quaternion_multiply(q, self._qdown)

    def next_rotation(self, acceleration=0.0):
        """Continue rotation in direction of last drag."""
        q = quaternion_slerp(self._qpre, self._qnow, 2.0+acceleration, False)
        self._qpre, self._qnow = self._qnow, q

    def matrix(self):
        """Return homogeneous rotation matrix."""
        return quaternion_matrix(self._qnow)


def arcball_map_to_sphere(point, center, radius):
    """Return unit sphere coordinates from window coordinates."""
    v = np.array(
        ((point[0] - center[0]) / radius, (center[1] - point[1]) / radius, 0.0),
        dtype=np.float64
       )
    n = v[0]*v[0] + v[1]*v[1]
    if n > 1.0:
        v /= math.sqrt(n)  # position outside of sphere
    else:
        v[2] = math.sqrt(1.0 - n)
    return v


def arcball_constrain_to_axis(point, axis):
    """Return sphere point perpendicular to axis."""
    v = np.array(point, dtype=np.float64, copy=True)
    a = np.array(axis, dtype=np.float64, copy=True)
    v -= a * np.dot(a, v)  # on plane
    n = vector_norm(v)
    if n > _EPS:
        if v[2] < 0.0:
            v *= -1.0
        v /= n
        return v
    if a[2] == 1.0:
        return np.array([1, 0, 0], dtype=np.float64)
    return unit_vector([-a[1], a[0], 0])


def arcball_nearest_axis(point, axes):
    """Return axis, which arc is nearest to point."""
    point = np.array(point, dtype=np.float64, copy=False)
    nearest = None
    mx = -1.0
    for axis in axes:
        t = np.dot(arcball_constrain_to_axis(point, axis), point)
        if t > mx:
            nearest = axis
            mx = t
    return nearest


# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = {v: k for k, v in _AXES2TUPLE.items()}


# helper functions
def vector_norm(
    data: np.ndarray,
    axis: int = None,
    out: np.ndarray = None
) -> float:
    """Return length, i.e. eucledian norm, of ndarray along axis."""
    data = np.array(data, dtype=np.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return math.sqrt(np.dot(data, data))
        data *= data
        out = np.atleast_1d(np.sum(data, axis=axis))
        np.sqrt(out, out)
        return out
    else:
        data *= data
        np.sum(data, axis=axis, out=out)
        np.sqrt(out, out)


def unit_vector(
    data: np.ndarray,
    axis: int = None,
    out: np.ndarray = None
) -> np.ndarray:
    """Return ndarray normalized by length, i.e. eucledian norm, along axis."""
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data*data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data


def random_vector(size: float) -> np.ndarray:
    """Return array of random doubles in the half-open interval [0.0, 1.0)."""
    return np.random.random(size)


def inverse_matrix(matrix: np.ndarray) -> np.ndarray:
    """Return inverse of square transformation matrix."""
    return np.linalg.inv(matrix)


def concatenate_matrices(*matrices: List[np.ndarray]) -> np.ndarray:
    """Return concatenation of series of transformation matrices."""
    M = np.identity(4)
    for i in matrices:
        M = np.dot(M, i)
    return M


def is_same_transform(matrix0: np.ndarray, matrix1: np.ndarray) -> bool:
    """Return True if two matrices perform same transformation."""
    matrix0 = np.array(matrix0, dtype=np.float64, copy=True)
    matrix0 /= matrix0[3, 3]
    matrix1 = np.array(matrix1, dtype=np.float64, copy=True)
    matrix1 /= matrix1[3, 3]
    return np.allclose(matrix0, matrix1)


def _import_module(module_name, warn=True, prefix='_py_', ignore='_'):
    """
    Try import all public attributes from module into global namespace.

    Existing attributes with name clashes are renamed with prefix.
    Attributes starting with underscore are ignored by default.

    Return True on successful import.

    """
    try:
        module = __import__(module_name)
    except ImportError:
        if warn:
            warnings.warn('Failed to import module ' + module_name)
    else:
        for attr in dir(module):
            if ignore and attr.startswith(ignore):
                continue
            if prefix:
                if attr in globals():
                    globals()[prefix + attr] = globals()[attr]
                elif warn:
                    warnings.warn('No Python implementation of ' + attr)
            globals()[attr] = getattr(module, attr)
        return True
