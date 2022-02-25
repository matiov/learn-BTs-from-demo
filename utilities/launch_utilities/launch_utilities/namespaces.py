"""Script to allow dynamic remappings of namespaces."""

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

from dataclasses import dataclass


@dataclass
class YuMiNamespaces:
    """Namespaces ABB YuMi robots."""

    root: str = 'abb_yumi'
    sensors: str = 'sensors'
    camera: str = 'camera'
    platform: str = 'platform'
    robot: str = 'robot'

    def __init__(self, root):
        self.root = root

    def get_root(self, relative=False):
        root = ''
        if relative:
            root = self.root
        elif self.root != '':
            root = '/' + self.root
        return root

    def get_root_sensors(self, relative=False):
        return self.get_root(relative) + '/' + self.sensors

    def get_root_camera(self, relative=False):
        return self.get_root_sensors(relative) + '/' + self.camera

    def get_root_platform(self, relative=False):
        return self.get_root(relative) + '/' + self.platform

    def get_root_robot(self, relative=False):
        return self.get_root(relative) + '/' + self.robot
