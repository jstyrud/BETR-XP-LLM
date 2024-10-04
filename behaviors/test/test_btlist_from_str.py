"""Unit test for behavior_lists.py module."""

# Copyright (c) 2024, ABB
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

import re

from behaviors.common_behaviors import NodeParameter, ParameterizedNode, ParameterTypes


NUMBER_REGEX = r'[-+]?(?:(?:\d*\.\d+)|(?:\d+\.?))(?:[Ee][+-]?\d+)?'


def test_place():
    """Build a Parameterized node from parsing a string with place behavior."""
    behavior_str = 'place0 banana 0.023 0.003 0.056 0.071 bowl'
    match_str = f'^(place\\d+) (.+) ({NUMBER_REGEX}) ({NUMBER_REGEX})' +\
                f' ({NUMBER_REGEX}) ({NUMBER_REGEX}) (.+)$'
    match = re.match(match_str, behavior_str)
    node_name = match[1]
    target_obj = NodeParameter(data_type=ParameterTypes.STRING, value=match[2])
    target_pose = NodeParameter(data_type=ParameterTypes.POSITION, value=[float(match[3]), float(match[4]), float(match[5])])
    tolerance = NodeParameter(data_type=ParameterTypes.FLOAT, value=float(match[6]))
    ref_frame = NodeParameter(data_type=ParameterTypes.STRING, value=match[7])

    params = {"target_object": target_obj, "target_pose": target_pose, "tolerance": tolerance, "frame": ref_frame}

    node = ParameterizedNode(node_name, None, params, False)

    print(node)
    node_params = node.get_parameters()

    assert node_params["target_object"] == 'banana'
    assert node_params["target_pose"] == [0.023, 0.003, 0.056]
    assert node_params["tolerance"] == 0.071
    assert node_params["frame"] == 'bowl'


def test_pick():
    """Build a Parameterized node from parsing a string with pick behavior."""
    behavior_str = 'pick0 banana'
    match_str = '^(pick\\d+) (.+)$'
    match = re.match(match_str, behavior_str)
    node_name = match[1]
    target_obj = NodeParameter(
        data_type=ParameterTypes.STRING, value=match[2])

    params = {"target_object": target_obj}

    node = ParameterizedNode(node_name, None, params, False)

    print(node)
    node_params = node.get_parameters()

    assert node_params["target_object"] == 'banana'
