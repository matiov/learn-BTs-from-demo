"""Test routine to for the match function."""

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

import re


NUMBER_REGEX = r'[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'


def test_match():
    test_string = 'object_roughly_at B 0.5339738639340457e-05 -0.023059651356835145' +\
        ' 0.24046563415431832 0.0 abb_yumi_base_link'

    match = re.match(
        f'^object_roughly_at (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +
        f' ({NUMBER_REGEX}) (.+)$', test_string)

    assert str(match[0]) == test_string
    assert str(match[1]) == 'B'
    assert float(match[2]) == 0.5339738639340457e-05
    assert float(match[3]) == -0.023059651356835145
    assert float(match[4]) == 0.24046563415431832
    assert float(match[5]) == 0.0
    assert str(match[6]) == 'abb_yumi_base_link'
