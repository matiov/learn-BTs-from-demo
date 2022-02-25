"""This scritp allows to plot the clusters of the learning framework."""

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

import argparse
import logging
import math
import pathlib
import sys

import bt_learning.learning_from_demo.clustering as clustering
from matplotlib.colors import hsv_to_rgb
import matplotlib.pyplot as plt
import numpy as np
from robot_behaviors.mobile_base_behaviors.lfd_actions import MoveAction
from robot_behaviors.yumi_behaviors.lfd_actions import PickAction, PlaceAction
import robot_interface.demonstration as demo


logging.basicConfig(level=logging.ERROR)


def plot_clusters():
    """Plot the clusters of the learning framework."""
    arg_parser = argparse.ArgumentParser(
        description='Plot demonstrated actions in all frames to visually inspect clusters.')
    arg_parser.add_argument(
        'dir', type=pathlib.Path, help='Directory where the demonstration is stored.')

    args = arg_parser.parse_args()

    if not args.dir.is_dir():
        print(args.dir, 'is not a folder or does not exist')
        sys.exit(1)

    demonstration = demo.RobotDemonstration(
        str(args.dir),
        {'pick': PickAction, 'place': PlaceAction, 'drop': PlaceAction, 'move': MoveAction}
    )
    partition = clustering.initial_partition(demonstration.all_actions())

    n_frames = len(demonstration.frames)
    n_rows = math.floor(math.sqrt(n_frames))
    for i, group in enumerate(partition):
        fig = plt.figure(f'Action group {i}: {group[0].type}({group[0].parameters})')

        equivalent = clustering.cluster(
            group, demonstration.frames, 0, len(demonstration.demonstrations()))
        # Assign a color to each group
        colors = []
        for eq in equivalent:
            colors.append(np.random.uniform(low=0, high=0.75, size=(1, 3)))

        for j, frame in enumerate(demonstration.frames):
            ax = fig.add_subplot(n_rows, math.ceil(n_frames/n_rows), j+1, projection='3d')
            ax.set_title(frame)

            for action in group:
                # Find action
                equivalent_grouping = next(x for x in equivalent if action in x)
                idx = equivalent.index(equivalent_grouping)
                color = hsv_to_rgb(
                    [idx/len(equivalent), 1, idx/len(equivalent)+0.1]).reshape((1, 3))

                if frame == action.frame[0]:
                    marker = 'x'
                else:
                    marker = 'o'

                position = action.target_position(frame)
                ax.scatter(position[0], position[1], position[2], c=color, marker=marker)

    plt.show()


if __name__ == '__main__':
    plot_clusters()
