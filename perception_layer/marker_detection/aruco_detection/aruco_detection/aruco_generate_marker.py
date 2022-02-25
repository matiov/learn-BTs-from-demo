"""
Script for generating Aruco marker images.

Author: Nathan Sprague
Version: 10/26/2020
"""

# Copyright (c) 2020 JMU-ROBOTICS-VIVA
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
import sys
import argparse
import cv2
import numpy as np


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter):
    """ Trick to allow both defaults and nice formatting in the help. """
    pass


def main():
    parser = argparse.ArgumentParser(formatter_class=CustomFormatter,
                                     description="Generate a .png image of a specified maker.")
    parser.add_argument('--id', default=1, type=int,
                        help='Marker id to generate')
    parser.add_argument('--size', default=200, type=int,
                        help='Side length in pixels')
    dict_options = [s for s in dir(cv2.aruco) if s.startswith("DICT")]
    option_str = ", ".join(dict_options)
    dict_help = "Dictionary to use. Valid options include: {}".format(option_str)
    parser.add_argument('--dictionary', default="DICT_5X5_250", type=str,
                        choices=dict_options,
                        help=dict_help, metavar='')
    args = parser.parse_args()

    dictionary_id = cv2.aruco.__getattribute__(args.dictionary)
    dictionary = cv2.aruco.Dictionary_get(dictionary_id)
    image = np.zeros((args.size, args.size), dtype=np.uint8)
    image = cv2.aruco.drawMarker(dictionary, args.id, args.size, image, 1)

    out_path = os.path.join("./", "marker_{:04d}.png".format(args.id))
    cv2.imwrite(out_path, image)


if __name__ == "__main__":
    main()
